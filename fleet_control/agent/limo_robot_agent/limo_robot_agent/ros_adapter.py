"""ROS 2 side of the agent. READ-ONLY with respect to the existing stack: it subscribes to topics, reads TF and looks at the ROS graph; the only thing
it ever sends is a Nav2 NavigateToPose goal for an accepted mission, and only if every interlock passes:
  1. missions enabled on this agent (default OFF for the real robot)   2. the navigate_to_pose action server is ready
  3. No /cmd_vel publisher in default/real mode. An explicit sim-only opt-in
     permits one Safety Gate endpoint with simulated time; all other publishers are refused.
It never publishes velocity and never calls the gate's reset service."""
import collections, json, math, threading, time, uuid

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID
from tf2_ros import Buffer, TransformListener

from .pngmap import grid_png, grid_stats
from .motor_policy import motor_path_allowed
from .safety_protocol import (
    GateObservationTracker,
    check_active_goal_safety,
    compute_movement_permitted,
    verify_navigation_execution,
    sim_r7_nav_evidence,
)

try:
    from limo_msgs.msg import LimoStatus
except Exception:                                                     # limo_msgs not installed: battery stays UNKNOWN
    LimoStatus = None

_MISSION_TERMINAL = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED'}
_MISSION_RANK = {
    'DISPATCHED': 1,
    'RECEIVED': 0,
    'SENT': 1,
    'ACCEPTED': 2,
    'EXECUTING': 3,
    'CANCELING': 3,
    'RECONCILIATION_REQUIRED': 3,
    **{s: 4 for s in _MISSION_TERMINAL},
}


def yaw_of(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _goal_uuid_hex(goal_id_obj):
    if goal_id_obj is None:
        return None
    raw = getattr(goal_id_obj, 'uuid', None)
    if raw is None:
        return None
    try:
        return bytes(raw).hex()
    except Exception:
        return None


class RosRobot(Node):
    def __init__(
        self,
        allow_missions,
        mode='real',
        simulated=False,
        allow_sim_safety_gate=False,
        robot_id='robot-1',
        agent_boot_id=None,
        cancel_result_timeout_s=5.0,
        disconnect_cancel_after_s=3.0,
        goal_response_timeout_s=5.0,
        sim_nav_evidence_actions=(),
        sim_r8_supervisor=False,
    ):
        super().__init__('fleet_robot_agent')
        self.allow_missions = allow_missions
        self.mode, self.simulated, self.allow_sim_safety_gate = mode, simulated, allow_sim_safety_gate
        if sim_r8_supervisor and not (mode == 'sim' and simulated is True and allow_sim_safety_gate is True):
            raise ValueError('r8 supervisor is SIM_ONLY')
        self.sim_r8_supervisor = sim_r8_supervisor is True
        self.sim_nav_evidence_actions = tuple(sim_nav_evidence_actions)
        self.robot_id = robot_id
        self.agent_boot_id = agent_boot_id or 'local-boot'
        self.cancel_result_timeout_s = float(cancel_result_timeout_s)
        self.disconnect_cancel_after_s = float(disconnect_cancel_after_s)
        self.goal_response_timeout_s = float(goal_response_timeout_s)
        self.execution_allowed = lambda: False  # AgentCore enables this only after ownership reconciliation.
        self.session_fn = lambda: {'connected': False}
        self.context_fn = None                  # Unimplemented op/map/loc producers remain fail-closed unless hooked cleanly in tests.
        self.localization_fn = None             # LocalizationWorker.contexts (saved map + AMCL), NAVIGATION mode only
        self.on_command_meta = None
        self.lock = threading.RLock()
        self._safety_seq = 0
        self.t = {}                                                    # topic key -> arrival monotonic time
        self.scan_times = collections.deque(maxlen=40); self.scan_beams = collections.deque(maxlen=60); self.scan_frame = None
        self.gate = None; self.depth = None; self.limo = None; self.odom = None
        self.gate_tracker = GateObservationTracker()
        self.map_a = None; self.map_meta = None; self.map_png = None; self.map_version = 0; self.map_hash = None; self.map_changed_t = None; self.map_first_t = None
        self.tfb = Buffer(); self.tfl = TransformListener(self.tfb, self)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(String, '/stall_detector/status_json', self._on_gate_message, 10)
        self.create_subscription(String, '/auto_mapping/depth_status', lambda m: self._set('depth', m), 10)
        self._standby_status = None
        self.create_subscription(String, '/limo/standby_status', self._on_standby_status, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        if LimoStatus is not None: self.create_subscription(LimoStatus, '/limo_status', self._on_limo, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Evidence for operator recovery: which goals does the Nav2 action server itself consider active? (status topic is transient-local, published on change)
        self.nav_status = None
        self.nav_terminal_observed = {}
        self.isolated_nav_observed_active = False
        # Continuity-based idle evidence (status is published on change only; see action_status_evidence.py).
        from .action_status_evidence import ActionStatusEvidence
        self.nav_idle_tracker = ActionStatusEvidence('NAV2', settle_s=1.0)
        self.follow_idle_tracker = ActionStatusEvidence('FOLLOW_PATH', expected_node='controller_server', settle_s=1.0)
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status', self._on_nav_status, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self._r8_started = False
        self._r8_seq = 0
        self._r8_challenge_seq = -1
        self._r8_nonce_history = collections.deque(maxlen=256)
        if self.sim_r8_supervisor:
            self._r8_pub = self.create_publisher(String, '/stall_detector/supervisor_heartbeat', 10)
            self.create_subscription(String, '/stall_detector/supervisor_challenge', self._on_r8_challenge, 10)
        self._sim_child_first_seen = {}; self._sim_overlap = {}; self._sim_followpath_clients_cache = None; self._sim_followpath_client_owners = []
        self.active = None                                             # dict(mission_id, command_id, handle, goal_uuid, ack, last_fb, ...)
        self.last_mission = {}
        # 2026-11-21: the two liveness signals the Motion Watchdog judges in STEADY time (heartbeat 0.3 s, control loop 0.5 s) run on
        # the steady clock. On a sim-time node a /clock delivery pause (Gazebo run after_g03seq_r1: 0.3 s while /scan stamps kept
        # advancing) froze both timers, the Watchdog went FAILSAFE (AGENT_HEARTBEAT_LOST) and the web-mapping session was lost.
        # Real robot (no sim time): unchanged apart from immunity to wall-clock jumps.
        _steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.tasks = collections.deque(); self.create_timer(0.05, self._run_tasks, clock=_steady)
        # Web-mapping worker hooks (default: no worker -> Frontier goals are fenced, Fleet goals unaffected).
        self.frontier_allowed = lambda session_id: False
        self.fleet_goal_allowed = lambda: True
        self.follow_path_topic = '/follow_path/_action/status'
        self.follow_path_status = None
        self.follow_path_isolated_sample = None
        self.follow_path_observed_active = False
        self.create_subscription(GoalStatusArray, self.follow_path_topic, self._on_follow_path_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        # Liveness for the separate motion watchdog: published by the ROS executor, so it stops when this process
        # dies or freezes. (A deadlock limited to non-executor threads is not detected by this signal.)
        self.require_motion_watchdog = False
        self._hb_seq = 0
        self._hb_healthy = True
        self._tasks_t = time.monotonic()
        self._hb_pub = self.create_publisher(String, '/fleet/agent_heartbeat', 10)
        self.create_timer(0.1, self._publish_heartbeat, clock=_steady)
        self.motion_watchdog = None
        self.create_subscription(String, '/fleet/motion_watchdog/status', self._on_watchdog_status, 10)

    # ------------------------------------------------------------ subscriptions
    def _stamp(self, key): self.t[key] = time.monotonic()
    def _age(self, key): return None if key not in self.t else time.monotonic() - self.t[key]

    def _on_scan(self, m):
        with self.lock:
            self._stamp('scan'); self.scan_times.append(time.monotonic()); self.scan_beams.append(len(m.ranges)); self.scan_frame = m.header.frame_id

    def _on_nav_status(self, m):
        self.nav_idle_tracker.on_message(time.monotonic(), m)
        # Terminal states reported by the NavigateToPose server itself, per goal UUID (recovery evidence; bounded).
        names = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED', GoalStatus.STATUS_ABORTED: 'FAILED'}
        with self.lock:
            for g in getattr(m, 'status_list', []) or []:
                if g.status in names:
                    u = _goal_uuid_hex(g.goal_info.goal_id)
                    if u:
                        self.nav_terminal_observed[u] = names[g.status]
            while len(self.nav_terminal_observed) > 64:
                self.nav_terminal_observed.pop(next(iter(self.nav_terminal_observed)))
        terminal_statuses = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
        active_uuids = []
        active_cnt = 0
        for g in getattr(m, 'status_list', []) or []:
            if getattr(g, 'status', None) not in terminal_statuses:
                active_cnt += 1
                u = _goal_uuid_hex(getattr(getattr(g, 'goal_info', None), 'goal_id', None))
                if u:
                    active_uuids.append(u)
        snap = {'t': time.monotonic(), 'active': active_cnt, 'total': len(getattr(m, 'status_list', []) or []), 'active_uuids': active_uuids}
        with self.lock:
            self.nav_status = snap
            if active_cnt:
                self.isolated_nav_observed_active = True
            self._safety_seq += 1

    def _nav2_goals(self):
        pubs = self.count_publishers('navigate_to_pose/_action/status')
        with self.lock:
            st = self.nav_status
            owned_uuid = self.active.get('goal_uuid') if isinstance(self.active, dict) else None
            active_uuids = list(st.get('active_uuids') or []) if (st is not None and pubs >= 1) else []
            age_s = None if st is None else round(max(0.0, time.monotonic() - st['t']), 3)
            active_count = (st['active'] if st else 0) if pubs >= 1 else None
        foreign_uuids = [u for u in active_uuids if u != owned_uuid]
        # Status is published on change only: "unchanged" is proven by the same unique publisher GID since the last
        # message (or since it appeared, when nothing was ever published), not by message age.
        try:
            w = self.motion_watchdog
            vouched = w[1].get('nav_status_gid') if (self.watchdog_ok() and w and isinstance(w[1].get('nav_status_gid'), str)) else None
            ev = self.nav_idle_tracker.evaluate(time.monotonic(), self.get_publishers_info_by_topic('/navigate_to_pose/_action/status'),
                                                vouched_gid=vouched)
            vouch = vouched is not None and ev['publisher_gid'] == vouched
            current = ev['status_from_current_server']
            # Same rule as the idle evidence: the newest status belongs to the current server (or none was received on
            # the first server), or the Watchdog re-proved this server after a change. Unattributed -> not continuous.
            continuous = (ev['publishers'] == 1 and ev['stable_s'] is not None and ev['stable_s'] >= self.nav_idle_tracker.settle_s
                          and ev['reason'] not in ('NAV2_SERVER_IDENTITY_UNKNOWN', 'NAV2_STATUS_UNATTRIBUTED')
                          and (current is True or (current is None and ev.get('generation', 0) <= 1) or vouch))
        except Exception:
            continuous = False
        direct = {'source': 'navigate_to_pose/_action/status', 'known': pubs >= 1, 'publishers': pubs, 'active_count': active_count,
                'received_status_array': st is not None, 'age_s': age_s, 'last_change_age_s': age_s,
                'status_continuous': continuous,
                'active_goal_uuids': active_uuids, 'foreign_active_goal_uuids': foreign_uuids}
        if (self.mode == 'sim' and self.simulated is True and self.allow_sim_safety_gate is True
                and self._use_sim_time() and getattr(self, 'sim_nav_evidence_actions', ())):
            sent = None
            with self.lock:
                if isinstance(self.active, dict) and self.active.get('goal_uuid') == owned_uuid:
                    sent = self.active.get('goal_sent_mono')
            owned_age = None if sent is None else max(0.0, time.monotonic() - sent)
            gate_eval = self._gate_evaluation()
            seen_ids = ((gate_eval.get('nav_evidence') or {}).get('active_uuids') if isinstance(gate_eval, dict) else None)
            if isinstance(seen_ids, list):                       # first-seen time of EVERY active UUID, not only of a candidate child
                now_m = time.monotonic()
                with self.lock:
                    for u in seen_ids:
                        if isinstance(u, str): self._sim_child_first_seen.setdefault(u, now_m)
            evidence = sim_r7_nav_evidence(
                gate_eval, self.sim_nav_evidence_actions, direct, owned_uuid,
                owned_age_s=owned_age, subordinate_ok=lambda child: self._sim_followpath_child_ok(child, sent),
                overlap_ok=self._sim_followpath_overlap_ok)
            if evidence is not None:
                # Check every reported action graph, not just NavigateToPose.
                # This is a graph consistency check, not DDS authentication.
                for action, observed in evidence['gate_nav_evidence']['servers'].items():
                    pubs = self.get_publishers_info_by_topic('/' + action.strip('/') + '/_action/status')
                    if (len(pubs) != 1 or bytes(getattr(pubs[0], 'endpoint_gid', [])).hex()[:16] != observed['writer_gid']):
                        evidence = None
                        break
                if evidence is not None:
                    return evidence
            # No silent legacy fallback after opting into the r7 scope.
            direct['known'] = False
        return direct

    def _sim_followpath_overlap_ok(self, pair, max_s=0.5):
        """Two follow_path goals are a legitimate hand-over only for a moment; the same pair visible longer than max_s is foreign."""
        now = time.monotonic()
        with self.lock:
            since = self._sim_overlap.setdefault(pair, now)
            for k in [k for k in self._sim_overlap if k != pair]: del self._sim_overlap[k]
        return now - since <= max_s

    def _sim_followpath_child_ok(self, child_uuid, owned_sent_mono):
        """SIM_ONLY provenance check for ONE follow_path goal next to the owned NavigateToPose goal (unauthenticated graph evidence).
        True only if (a) the owned goal was submitted by this Agent and the child UUID was first observed after that submission, and
        (b) every ROS node that owns a follow_path action client is a Nav2 bt_navigator node (a rogue node name is not detected).
        Unknown or unreadable graph data => False (the child stays foreign)."""
        now = time.monotonic()
        if owned_sent_mono is None:
            return False
        with self.lock:
            first = self._sim_child_first_seen.setdefault(child_uuid, now)
            if len(self._sim_child_first_seen) > 4096: self._sim_child_first_seen.clear()
        if first < owned_sent_mono:
            return False
        cache = self._sim_followpath_clients_cache
        if cache is not None and now - cache[0] < 0.5:
            return cache[1]
        ok = False
        try:
            from rclpy.action import get_action_client_names_and_types_by_node
            owners = []
            for name, ns in self.get_node_names_and_namespaces():
                if name == self.get_name() and ns == self.get_namespace(): continue
                for act, _types in get_action_client_names_and_types_by_node(self, name, ns):
                    if act.strip('/').split('/')[-1] == 'follow_path': owners.append(name)
            ok = bool(owners) and all(n.startswith('bt_navigator') for n in owners)
            self._sim_followpath_client_owners = sorted(set(owners))
        except Exception:
            ok = False
        self._sim_followpath_clients_cache = (now, ok)
        return ok

    def _on_odom(self, m):
        with self.lock:
            self._stamp('odom'); self.odom = (m.header.frame_id, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))

    def _on_limo(self, m):
        with self.lock:
            self._stamp('limo'); self.limo = {'voltage': float(m.battery_voltage), 'vehicle_state': int(m.vehicle_state), 'control_mode': int(m.control_mode), 'motion_mode': int(m.motion_mode), 'error_code': int(m.error_code)}

    def _set(self, key, m):
        try: v = json.loads(m.data)
        except Exception: v = None
        with self.lock:
            self._stamp(key)
            if key == 'gate':
                self.gate = v
                self.gate_tracker.on_message(v)
                self._safety_seq += 1
            else:
                self.depth = v

    def _on_gate_message(self, m):
        # Humble executor discards MessageInfo. Graph continuity is NOT a
        # per-message source binding and is accepted only for SIM_ONLY evidence.
        with self.lock:
            pubs = self.get_publishers_info_by_topic('/stall_detector/status_json')
            gid = bytes(getattr(pubs[0], 'endpoint_gid', [])).hex() if len(pubs) == 1 else ''
            prior = getattr(self, '_gate_publisher_gid', None)
            if not gid or (prior is not None and prior != gid):
                self.gate_tracker.r7_invalid = True
            self._gate_publisher_gid = gid
            self._set('gate', m)

    def _gate_evaluation(self):
        g = self.gate_tracker.evaluate(expect_source_clock=self._use_sim_time())
        if g.get('contract_version') == 'gate-status/1.2-sim':
            pubs = self.get_publishers_info_by_topic('/stall_detector/status_json')
            expected = getattr(self, '_gate_publisher_gid', None)
            valid = (len(pubs) == 1 and bool(expected)
                     and bytes(getattr(pubs[0], 'endpoint_gid', [])).hex() == expected)
            g['single_publisher_graph'] = valid
            if not valid:
                self.gate_tracker.r7_invalid = True
                g['r7_invalid'] = True
                g['source_fresh'] = False
                g['contract_met'] = False
        return g

    def _on_r8_challenge(self, msg):
        """Unauthenticated test protocol. Never enabled on the real robot."""
        if not (self.sim_r8_supervisor and self.mode == 'sim' and self.simulated is True
                and self.allow_sim_safety_gate is True and self._use_sim_time()):
            return
        try:
            challenge = json.loads(msg.data)
            nonce = challenge['nonce']; seq = challenge['challenge_seq']
            issued = challenge['issued_mono_ns']; boot = challenge['gate_boot_id']
            now = time.monotonic_ns()
            if (challenge['contract'] != 'gate-supervisor-challenge/0-sim'
                    or not isinstance(nonce, str) or len(nonce) != 32
                    or any(c not in '0123456789abcdef' for c in nonce)
                    or type(seq) is not int or seq < 0
                    or type(issued) is not int or not 0 <= now - issued <= 500_000_000):
                return
            session = self._session_context()
            lease = session.get('lease') or {}
            permitted = (self.execution_allowed() is True and session.get('connected') is True
                         and session.get('reconciliation_required') is False
                         and type(lease.get('epoch')) is int and type(lease.get('expires_at')) in (int, float)
                         and lease['expires_at'] > time.time())
            with self.lock:
                gate = self.gate_tracker.evaluate(expect_source_clock=True)
                if (gate.get('boot_id') != boot or gate.get('boot_changed') is True
                        or gate.get('r7_invalid') is True or seq <= self._r8_challenge_seq
                        or nonce in self._r8_nonce_history):
                    return
                if not permitted and not self._r8_started:
                    return
                self._r8_started = True
                self._r8_challenge_seq = seq; self._r8_nonce_history.append(nonce)
                self._r8_seq += 1
                owned = [self.active['goal_uuid']] if self.active and self.active.get('goal_uuid') else []
                payload = dict(contract='agent-supervisor-heartbeat/0-sim', gate_boot_id=boot,
                               nonce=nonce, agent_boot_id=self.agent_boot_id, agent_seq=self._r8_seq,
                               execution_allowed=bool(permitted), lease_epoch=lease.get('epoch'),
                               owned_goal_uuids=owned, sent_mono_ns=now)
                self._r8_pub.publish(String(data=json.dumps(payload)))
        except (KeyError, ValueError, TypeError, AttributeError):
            return

    def _on_map(self, m):
        a, st, h = grid_stats(m)
        with self.lock:
            now = time.monotonic(); self._stamp('map')
            if self.map_first_t is None: self.map_first_t = now
            if h != self.map_hash:
                self.map_hash = h; self.map_version += 1; self.map_changed_t = now; self.map_a = a
                self.map_meta = dict({'version': self.map_version, 'width': m.info.width, 'height': m.info.height, 'resolution': m.info.resolution, 'origin_x': m.info.origin.position.x,
                                      'origin_y': m.info.origin.position.y, 'origin_yaw': yaw_of(m.info.origin.orientation), 'frame_id': m.header.frame_id, 'cell_encoding': 'png: free=255 occupied=0 unknown=128, row0=top(+y)'}, **st)
                self.map_png = None                                    # rendered lazily on upload

    # ------------------------------------------------------------ state snapshot (called from the agent thread)
    def _on_standby_status(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(status, dict) and status.get('mode') == 'STANDBY' and status.get('drive_ready') is False:
            self._standby_status = time.monotonic(), status

    def snapshot(self):
        standby = getattr(self, '_standby_status', None)
        errs = []
        # --- pose: prefer map frame (TF map->base_footprint); age from the dynamic odom->base_footprint transform
        pose = {'status': 'NO_DATA', 'frame_id': None, 'x': None, 'y': None, 'yaw_rad': None, 'source': None, 'age_s': None}
        try:
            tr = self.tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            dyn = self.tfb.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            age = (self.get_clock().now().nanoseconds - rclpy.time.Time.from_msg(dyn.header.stamp).nanoseconds) * 1e-9
            pose.update({'frame_id': 'map', 'x': tr.transform.translation.x, 'y': tr.transform.translation.y, 'yaw_rad': yaw_of(tr.transform.rotation), 'source': 'tf map->base_footprint', 'age_s': round(age, 2), 'status': 'OK' if age < 2.0 else 'STALE'})
        except Exception:
            if self.odom is not None and self._age('odom') is not None:
                f, x, y, yaw = self.odom; age = self._age('odom')
                pose.update({'frame_id': f or 'odom', 'x': x, 'y': y, 'yaw_rad': yaw, 'source': '/odom (no map->base_footprint TF: SLAM not running?)', 'age_s': round(age, 2), 'status': 'OK' if age < 2.0 else 'STALE'})
                errs.append('pose is not in the map frame (no map->base_footprint TF)')
        # --- map
        with self.lock:
            m = {'status': 'NO_MAP', 'version': None, 'known_cells': None, 'last_update_age_s': None}
            if self.map_meta:
                since = time.monotonic() - self.map_changed_t
                m = {'status': 'UPDATING' if since < 10.0 else 'STATIC', 'version': self.map_version, 'width': self.map_meta['width'], 'height': self.map_meta['height'], 'resolution': self.map_meta['resolution'],
                     'frame_id': self.map_meta['frame_id'], 'known_cells': self.map_meta['known_cells'], 'last_update_age_s': round(since, 1), 'last_message_age_s': round(self._age('map'), 1)}
        # --- lidar
        n = len(self.scan_times); a = self._age('scan'); lidar = {'status': 'NO_DATA', 'rate_hz': None, 'age_s': None, 'beams_min': None, 'beams_max': None, 'frame_id': self.scan_frame}
        if a is not None:
            rate = (n - 1) / (self.scan_times[-1] - self.scan_times[0]) if n > 2 and self.scan_times[-1] > self.scan_times[0] else None
            lidar.update({'status': 'OK' if a < 1.0 else 'STALE', 'rate_hz': None if rate is None else round(rate, 2), 'age_s': round(a, 2), 'beams_min': min(self.scan_beams), 'beams_max': max(self.scan_beams)})
            if a >= 1.0: errs.append(f'LiDAR /scan stale ({a:.1f} s)')
        # --- depth camera (status published by limo_depth_obstacles)
        a = self._age('depth'); depth = {'status': 'NO_DATA', 'code': None, 'age_s': None}
        if a is not None and self.depth:
            st = self.depth.get('state'); depth.update({'code': self.depth.get('code'), 'age_s': round(a, 2), 'status': 'STALE' if a > 2.5 else ('OK' if st == 'ready' else 'ERROR'), 'message': self.depth.get('message')})
        # --- safety gate (stall_detector_node status json) and the motor output path
        mp, motor_path_info = self._motor_path_status()
        motor_path_ok = mp in ('LOCKED', 'SIM_GATED')
        op_ctx, map_ctx, loc_ctx = self._producer_contexts()
        session_ctx = self._session_context()
        with self.lock:
            gate_eval = self._gate_evaluation()
            nav2_goals = self._nav2_goals()
            mis = dict(self.last_mission) if self.last_mission else {}
            a = self.active
            active_snap = None
            pending_cancel_ids = []
            if a:
                active_snap = {
                    'mission_id': a['mission_id'],
                    'command_id': a['command_id'],
                    'owner': a.get('owner', 'fleet'),
                    'goal_uuid': a.get('goal_uuid'),
                    'goal_handle_available': a['handle'] is not None,
                    'cancel_requested': bool(a.get('cancel_requested')),
                    'cancel_timeout_reported': bool(a.get('cancel_timeout_reported')),
                    'fault_cancel_reason': a.get('fault_cancel_reason'),
                }
                pending_cancel_ids = [cid for cid, ack in a.get('cancel_acks', [])]
        movement_permitted, block_reasons = compute_movement_permitted(
            allow_missions=self.allow_missions,
            gate_eval=gate_eval,
            motor_path_ok=motor_path_ok,
            op_ctx=op_ctx,
            map_ctx=map_ctx,
            loc_ctx=loc_ctx,
            nav2_goals=nav2_goals,
            session_ctx=session_ctx,
        )
        # Gate attestation exists only for SIM. The isolated mock + locked-arbiter graph has no motor at all (Gate output
        # /cmd_vel_locked, /cmd_vel without endpoints = motor path LOCKED), so Nav2 goals there cannot move hardware.
        locked_mock = (self.mode == 'mock' and self.simulated is True and getattr(self, 'locked_arbiter', False) is True
                       and mp == 'LOCKED')
        if not (self.mode == 'sim' and self.simulated is True) and not locked_mock:
            movement_permitted = False
            block_reasons.append('GATE_ATTESTATION_UNAVAILABLE')
        gate = dict(gate_eval)
        gate['motor_path'] = motor_path_info
        with self.lock:                                   # 2026-11-21: raw Gate fields for the admin Gate-reset view (display only)
            raw = dict(self.gate) if isinstance(self.gate, dict) else {}
        gate['reset_view'] = {k: raw.get(k) for k in ('state', 'fault_reasons', 'last_fault_reasons', 'not_ready', 'cmd_input', 'reset_readiness',
                                                      'last_reset', 'reset_refusals', 'boot_id', 'fault_epoch', 'fault_latched')}
        gate['movement_permitted'] = movement_permitted
        gate['movement_block_reasons'] = block_reasons
        if gate['status'] not in ('NORMAL', 'NO_DATA'):
            errs.append(f"safety gate state {gate['status']}")
        if mp == 'ARMED':
            errs.append('motor path ARMED: /cmd_vel has a publisher')
        # --- battery
        bat = {'status': 'UNKNOWN', 'voltage_v': None, 'percent': None, 'source': '/limo_status' if LimoStatus else 'limo_msgs not available'}
        with self.lock:
            a_limo = self._age('limo')
            limo_copy = dict(self.limo) if self.limo else None
        if a_limo is not None and limo_copy and a_limo < 5.0:
            bat.update({'status': 'VOLTAGE_ONLY', 'voltage_v': round(limo_copy['voltage'], 2), 'age_s': round(a_limo, 1), 'note': 'percent is not reported by the driver; no voltage-to-percent conversion is assumed'})
            for k in ('vehicle_state', 'control_mode', 'motion_mode', 'error_code'): bat[k] = limo_copy[k]
            if limo_copy['error_code']: errs.append(f"LIMO error_code {limo_copy['error_code']}")
        # --- nav2
        ready = self.nav.server_is_ready(); nav2 = {'status': 'ACTIVE_GOAL' if active_snap else ('READY' if ready else 'UNAVAILABLE'), 'action_server_ready': bool(ready), 'action': 'navigate_to_pose'}
        execution = {'active': active_snap, 'pending_cancel_ids': pending_cancel_ids, 'nav2_goals': nav2_goals,
                     'nav_idle': self.manual_idle_evidence(), 'nav_terminal_observed': dict(self.nav_terminal_observed)}
        return {
            'schema_version': 1,
            'standby_status': standby[1] if standby and time.monotonic() - standby[0] <= 3 else None,
            'pose': pose,
            'map': m,
            'nav2': nav2,
            'battery': bat,
            'lidar': lidar,
            'depth_camera': depth,
            'safety_gate': gate,
            'motion_watchdog': self.watchdog_report(),
            'operation_mode': op_ctx.get('operation_mode', 'UNKNOWN'),
            'operation_revision': op_ctx.get('operation_revision'),
            'map_epoch': map_ctx.get('map_epoch'),
            'map_version_id': map_ctx.get('map_version_id'),
            'map_verified': bool(map_ctx.get('map_verified') is True),
            'localization_epoch': loc_ctx.get('localization_epoch'),
            'localization_map_version_id': loc_ctx.get('localization_map_version_id'),
            'localization_status': loc_ctx.get('localization_status', 'UNKNOWN'),
            'mission': mis,
            'execution': execution,
            'errors': errs,
        }

    def _use_sim_time(self):
        try:
            return bool(self.get_parameter('use_sim_time').value)
        except Exception:
            return False

    def _motor_path_status(self):
        try:
            raw_pubs = self.get_publishers_info_by_topic('/cmd_vel')
            pubs = sorted({i.node_name for i in raw_pubs})
            lpubs = sorted({i.node_name for i in self.get_publishers_info_by_topic('/cmd_vel_locked')})
        except Exception:
            raw_pubs, pubs, lpubs = None, None, None
        mp = 'UNKNOWN' if pubs is None else ('ARMED' if pubs else ('LOCKED' if lpubs else 'UNKNOWN'))
        if mp == 'ARMED' and raw_pubs is not None and self._motor_path_allowed(raw_pubs):
            mp = 'SIM_GATED'
        info = {
            'status': mp,
            'cmd_vel_publishers': pubs,
            'cmd_vel_locked_publishers': lpubs,
            'meaning': 'ARMED = something publishes /cmd_vel (motors can move); LOCKED = no /cmd_vel publisher and the gate publishes /cmd_vel_locked',
        }
        return mp, info

    def _server_operation_context(self):
        """Operation producer: the Fleet server's operating mode (None for legacy robots without a mode)."""
        ss = self.session_fn() if callable(self.session_fn) else {}
        om = (ss or {}).get('operating_mode')
        if not isinstance(om, dict) or type(om.get('revision')) is not int:
            return None
        mode = om.get('operation_mode', om.get('mode'))
        return {'operation_mode': mode if mode in ('IDLE', 'MANUAL', 'MAPPING', 'NAVIGATION') else 'UNKNOWN',
                'operation_revision': om['revision']}

    def _producer_contexts(self):
        server_op = self._server_operation_context()
        if callable(self.context_fn):
            ctx = self.context_fn() or {}
            return (
                server_op or dict(ctx.get('operation') or {'operation_mode': 'UNKNOWN', 'operation_revision': None}),
                dict(ctx.get('map') or {'map_version_id': None, 'map_epoch': None, 'map_verified': False}),
                dict(ctx.get('localization') or {'localization_status': 'UNKNOWN', 'localization_map_version_id': None, 'localization_epoch': None}),
            )
        loc = self.localization_fn() if callable(self.localization_fn) else None
        if loc:
            mapc, locc = loc
            return (server_op or {'operation_mode': 'UNKNOWN', 'operation_revision': None}, dict(mapc), dict(locc))
        # Without a localization producer Map / Localization fail closed.
        return (
            server_op or {'operation_mode': 'UNKNOWN', 'operation_revision': None},
            {'map_version_id': None, 'map_epoch': None, 'map_verified': False},
            {'localization_status': 'UNKNOWN', 'localization_map_version_id': None, 'localization_epoch': None},
        )

    def _session_context(self):
        ss = dict(self.session_fn()) if callable(self.session_fn) else {'connected': False}
        if not self.execution_allowed():
            ss['connected'] = False
        if not ss.get('agent_boot_id'):
            ss['agent_boot_id'] = self.agent_boot_id
        return ss

    def map_for_upload(self, last_version):
        with self.lock:
            if self.map_meta is None or self.map_version == last_version: return None
            if self.map_png is None: self.map_png = grid_png(self.map_a)
            return self.map_version, dict(self.map_meta), self.map_png

    # ------------------------------------------------------------ mission execution (called from the agent thread -> queued to the ROS thread)
    def _submission_fence(self):
        """The AgentCore's contract_fence when session_fn is bound to an AgentCore (production wiring and the AgentCore-based tests);
        otherwise a private lock (stand-alone RosRobot tests with synthetic sessions, where nobody else changes the authority)."""
        owner = getattr(self.session_fn, '__self__', None)
        fence = getattr(owner, 'contract_fence', None)
        if fence is None:
            fence = self.__dict__.setdefault('_own_submission_fence', threading.RLock())
        return fence

    def start_navigate(self, cmd, ack):
        if isinstance(cmd, dict) and 'contract_generation' not in cmd and callable(self.session_fn):    # direct callers: stamp at enqueue time
            cmd = dict(cmd, contract_generation=(self.session_fn() or {}).get('contract_generation'))
        self.tasks.append(('nav', cmd, ack))
    def cancel_mission(self, cmd, ack): self.tasks.append(('cancel', cmd, ack))

    def _publish_heartbeat(self):
        self._hb_seq += 1
        a = self.active                                   # reference read only (no lock: keep the heartbeat cheap)
        goals = [a['goal_uuid']] if isinstance(a, dict) and a.get('goal_uuid') and a.get('terminal_status') is None else []
        self._hb_pub.publish(String(data=json.dumps({'boot_id': self.agent_boot_id, 'seq': self._hb_seq,
                                                     'healthy': self._hb_healthy, 't': time.time(),
                                                     'tasks_age_s': round(time.monotonic() - self._tasks_t, 3),
                                                     'goal_uuids': goals})))

    def announce_shutdown(self):
        """Explicit unhealthy heartbeat so the watchdog trips at once on a graceful stop."""
        self._hb_healthy = False
        for _ in range(3):
            self._publish_heartbeat()

    def _on_watchdog_status(self, msg):
        try:
            self.motion_watchdog = (time.monotonic(), json.loads(msg.data))
        except ValueError:
            self.motion_watchdog = None

    def watchdog_ok(self):
        w = self.motion_watchdog
        return bool(w and time.monotonic() - w[0] < 1.0 and w[1].get('state') == 'MOTION_AUTHORIZED'
                    and w[1].get('agent_boot_id') == self.agent_boot_id)

    def watchdog_report(self):
        """Snapshot block for the server: a trip (or a watchdog restart) changes 'epoch', which invalidates queued
        mission admissions, so nothing queued before a motion fault is dispatched afterwards."""
        w = self.motion_watchdog
        if not w:
            return {'required': bool(self.require_motion_watchdog), 'state': None, 'age_s': None, 'epoch': None}
        age = time.monotonic() - w[0]
        d = w[1]
        epoch = f"{d.get('boot_id')}:{d.get('trips')}" if d.get('boot_id') and type(d.get('trips')) is int else None
        return {'required': bool(self.require_motion_watchdog), 'state': d.get('state'), 'age_s': round(age, 3),
                'epoch': epoch, 'last_fault': d.get('last_fault'),
                'agent_boot_matches': d.get('agent_boot_id') == self.agent_boot_id}

    def start_frontier(self, cmd, ack): self.tasks.append(('frontier', cmd, ack))
    def cancel_frontier(self, cmd, ack): self.tasks.append(('cancel', cmd, ack))

    def _run_tasks(self):
        while self.tasks:
            kind, cmd, ack = self.tasks.popleft()
            handler = {'nav': self._do_nav, 'frontier': self._do_frontier}.get(kind, self._do_cancel)
            self._tasks_t = time.monotonic()
            try: handler(cmd, ack)
            except Exception as e: ack('REJECTED' if kind == 'frontier' else ('FAILED' if kind == 'nav' else 'REJECTED'), f'agent error: {e}')
        self._monitor_active_goal()
        self._tasks_t = time.monotonic()      # goal task loop completed a pass (control-loop liveness)

    # ------------------------------------------------------------ web-mapping Frontier goals (motor-locked only)
    def _on_follow_path_status(self, msg):
        self.follow_idle_tracker.on_message(time.monotonic(), msg)
        try:
            publishers = self.get_publishers_info_by_topic(self.follow_path_topic)
        except Exception:
            publishers = []
        if len(publishers) != 1 or publishers[0].node_name != 'controller_server':
            self.follow_path_status = None
            return
        terminal = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
        with self.lock:
            active = any(g.status not in terminal for g in msg.status_list)
            if active:
                self.follow_path_observed_active = True
            self.follow_path_status = (tuple(publishers[0].endpoint_gid), self.follow_path_observed_active and not active)
            self.follow_path_isolated_sample = (tuple(publishers[0].endpoint_gid), time.monotonic(), active)

    def manual_idle_evidence(self):
        """NavigateToPose + FollowPath idle evidence for MANUAL admission (see action_status_evidence.py).

        A stale active status from a previous NavigateToPose server is accepted only when the independent Motion
        Watchdog is authorized for the current server's GID (it re-proves idle with cancel-all after a change)."""
        now = time.monotonic()
        try:
            nav_infos = self.get_publishers_info_by_topic('/navigate_to_pose/_action/status')
            fp_infos = self.get_publishers_info_by_topic(self.follow_path_topic)
        except Exception:
            return {'ok': False, 'reason': 'GRAPH_QUERY_FAILED', 'navigate_to_pose': None, 'follow_path': None}
        w = self.motion_watchdog
        vouched = None
        if self.watchdog_ok() and w and isinstance(w[1].get('nav_status_gid'), str):
            vouched = w[1]['nav_status_gid']
        nav = self.nav_idle_tracker.evaluate(now, nav_infos, vouched_gid=vouched)
        # FollowPath has no independent watchdog proof; after a controller change it is accepted only together with a
        # vouched NavigateToPose (FollowPath goals of Nav2 come from bt_navigator) and 3x settle of a stable new server.
        fp_vouch = (nav['ok'] and vouched is not None and nav.get('publisher_gid') == vouched
                    and (self.follow_idle_tracker.since is not None and now - self.follow_idle_tracker.since >= 3 * self.follow_idle_tracker.settle_s))
        fp = self.follow_idle_tracker.evaluate(now, fp_infos, vouched=fp_vouch)
        reason = nav['reason'] or fp['reason']
        return {'ok': nav['ok'] and fp['ok'], 'reason': reason, 'navigate_to_pose': nav, 'follow_path': fp}

    def isolated_follow_path_never_active(self):
        # Only used by the localhost/mock motor-locked diagnostic path.
        with self.lock:
            sample = self.follow_path_isolated_sample
            previously_active = self.follow_path_observed_active
        try:
            pubs = self.get_publishers_info_by_topic(self.follow_path_topic)
        except Exception:
            return False
        return bool(sample and not previously_active and not sample[2] and
                    0 <= time.monotonic() - sample[1] <= 1.0 and len(pubs) == 1 and
                    pubs[0].node_name == 'controller_server' and tuple(pubs[0].endpoint_gid) == sample[0])

    def follow_path_idle(self):
        """FollowPath (controller sub-action) observed active after our submission and now terminal."""
        status = self.follow_path_status
        try:
            publishers = self.get_publishers_info_by_topic(self.follow_path_topic)
        except Exception:
            return False
        return bool(status and status[1] and len(publishers) == 1 and status[0] == tuple(publishers[0].endpoint_gid))

    def _do_frontier(self, cmd, ack):
        """Submit a mapping-owned NavigateToPose goal. Shares the single goal slot with Fleet missions.

        Fleet Safety Context is not required because no Fleet mission is involved, but the motor path must be
        LOCKED (no /cmd_vel publisher), the live mapping session must allow it, Nav2 must be idle with no
        foreign goals, and the agent must be reconciled with the server."""
        mid, cid = cmd.get('mission_id'), cmd.get('command_id', '')
        def rej(code, why):
            ack('REJECTED', f'{code}: {why}')
        if not self.frontier_allowed(mid):
            return rej('FRONTIER_FENCED', 'mapping session does not allow a goal')
        if not self.execution_allowed():
            return rej('NOT_RECONCILED', 'server session not reconciled; no goal sent')
        if time.time() > cmd.get('expires_at', 0):
            return rej('EXPIRED', 'command expired while queued')
        g = (cmd.get('payload') or {}).get('goal') or {}
        try: x, y, yaw = float(g['x']), float(g['y']), float(g.get('yaw_rad', 0.0))
        except Exception: return rej('BAD_GOAL', 'goal needs numeric x, y')
        if g.get('frame_id') != 'map' or not all(math.isfinite(v) for v in (x, y, yaw)):
            return rej('BAD_GOAL', "goal frame must be 'map' with finite values")
        if self.require_motion_watchdog and not self.watchdog_ok():
            return rej('MOTION_WATCHDOG_NOT_OK', 'independent motion watchdog missing, stale or tripped')
        mp, _ = self._motor_path_status()
        if mp not in ('LOCKED', 'SIM_GATED'):
            return rej('MOTOR_PATH_NOT_LOCKED', f'web mapping needs a locked or SIM_GATED motor path (motor path {mp})')
        if not self.nav.wait_for_server(timeout_sec=2.0):
            return rej('NAV2_UNAVAILABLE', 'navigate_to_pose action server not available')
        goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'; goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        reserved = uuid.uuid4()
        with self.lock:
            if self.active is not None:
                return rej('BUSY', 'the goal slot is reserved')
            if not self.frontier_allowed(mid):
                return rej('FRONTIER_FENCED', 'mapping session changed before send')
            observed = self._nav2_goals()
            if observed['publishers'] != 1 or observed['active_count'] != 0 or observed.get('foreign_active_goal_uuids'):
                return rej('NAV_GOAL_OWNERSHIP_UNAVAILABLE', 'another or indeterminate NavigateToPose goal is present')
            if self._motor_path_status()[0] not in ('LOCKED', 'SIM_GATED') or not self.execution_allowed():
                return rej('ABORTED_PRE_SEND', 'motor path or session changed before send')
            rec = {'mission_id': mid, 'command_id': cid, 'owner': 'frontier', 'submission_agent_boot_id': self.agent_boot_id,
                   'handle': None, 'goal_uuid': reserved.hex, 'ack': ack, 'last_fb': 0.0, 'cancel_requested': False,
                   'cancel_acks': [], 'cancel_sent': False, 'cancel_started_mono': None, 'cancel_timeout_reported': False,
                   'reconciliation_required': False, 'goal_response_done': False, 'goal_response_lost': False,
                   'terminal_status': None, 'goal_sent_mono': time.monotonic(), 'fault_cancel_reason': None, 'safety_context': None}
            self.follow_path_status = None
            self.follow_path_observed_active = False
            self.active = rec
            try:
                fut = self.nav.send_goal_async(goal, feedback_callback=lambda fb: self._on_fb(rec, fb),
                                               goal_uuid=UUID(uuid=list(reserved.bytes)))
            except Exception:
                rec['goal_response_lost'] = True
                rec['reconciliation_required'] = True
                fut = None
        if fut is None:
            ack('RECONCILIATION_REQUIRED', 'Nav2 submission outcome unknown; goal was not retried')
            return
        ack('SUBMITTED', 'goal sent to Nav2', {'goal_uuid': reserved.hex})
        fut.add_done_callback(lambda f: self._on_goal_response(rec, f))

    def _motor_path_allowed(self, publishers):
        return motor_path_allowed(publishers, mode=getattr(self, 'mode', 'real'),
                                  simulated=getattr(self, 'simulated', False),
                                  allow_sim_safety_gate=getattr(self, 'allow_sim_safety_gate', False),
                                  use_sim_time=self._use_sim_time())

    def _verify_nav_preconditions(self, cmd):
        # There is no authenticated Gate attestation producer in the current
        # real-robot integration. Never infer it from Gate NORMAL or an empty
        # status array; simulation fixtures are separately isolated.
        mp, _ = self._motor_path_status()
        motor_path_ok = mp in ('LOCKED', 'SIM_GATED')
        op_ctx, map_ctx, loc_ctx = self._producer_contexts()
        session_ctx = self._session_context()
        with self.lock:
            gate_eval = self._gate_evaluation()
            nav2_goals = self._nav2_goals()
            active_rec = self.active
        result = verify_navigation_execution(
            cmd,
            robot_id=self.robot_id,
            agent_boot_id=self.agent_boot_id,
            allow_missions=self.allow_missions,
            gate_eval=gate_eval,
            motor_path_ok=motor_path_ok,
            op_ctx=op_ctx,
            map_ctx=map_ctx,
            loc_ctx=loc_ctx,
            nav2_goals=nav2_goals,
            session_ctx=session_ctx,
            active_rec=active_rec,
        )
        if result[0] and self.require_motion_watchdog and not self.watchdog_ok():
            return False, 'REJECTED', 'MOTION_WATCHDOG_NOT_OK: independent motion watchdog missing, stale or tripped', None
        locked_mock = (self.mode == 'mock' and self.simulated is True and getattr(self, 'locked_arbiter', False) is True
                       and mp == 'LOCKED')
        if result[0] and not (self.mode == 'sim' and self.simulated is True) and not locked_mock:
            return False, 'REJECTED', 'GATE_ATTESTATION_UNAVAILABLE: real movement remains blocked', None
        if result[0] and not self.fleet_goal_allowed():
            return False, 'REJECTED', 'MAPPING_SESSION_OWNS_GOAL_SLOT', None
        return result

    def _do_nav(self, cmd, ack):
        mid = cmd.get('mission_id')
        cid = cmd.get('command_id', '')
        def abort(status, detail):
            ack(status, detail)
            self._set_mission(mid, cid, status, detail)
        # Stage 1: verify Safety Context v1 & local safety invariants before waiting on Nav2 (never hold lock across wait_for_server).
        ok1, st1, detail1, _ = self._verify_nav_preconditions(cmd)
        if not ok1:
            return abort(st1, detail1)
        if not self.nav.wait_for_server(timeout_sec=2.0):
            return abort('REJECTED', 'NAV2_UNAVAILABLE: navigate_to_pose action server not available')
        # Stage 2: re-verify immediately after wait_for_server before announcing DISPATCHED.
        ok2, st2, detail2, sc2 = self._verify_nav_preconditions(cmd)
        if not ok2:
            return abort(st2, f'ABORTED_PRE_SEND: {detail2}')
        g = (cmd.get('payload') or {}).get('goal') or {}
        x, y, yaw = float(g['x']), float(g['y']), float(g.get('yaw_rad', 0.0))
        goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'; goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y; goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        reserved_uuid = uuid.uuid4()
        ros_goal_uuid = UUID(uuid=list(reserved_uuid.bytes))
        rec = {
            'mission_id': mid,
            'command_id': cid,
            'submission_agent_boot_id': self.agent_boot_id,
            'handle': None,
            'goal_uuid': reserved_uuid.hex,
            'ack': ack,
            'last_fb': 0.0,
            'cancel_requested': False,
            'cancel_acks': [],
            'cancel_sent': False,
            'cancel_started_mono': None,
            'cancel_timeout_reported': False,
            'reconciliation_required': False,
            'goal_response_done': False,
            'goal_response_lost': False,
            'terminal_status': None,
            'goal_sent_mono': None,
            'fault_cancel_reason': None,
            'safety_context': sc2,
            'contract_generation': cmd.get('contract_generation'),      # R1.4.3: the obligation to cancel follows this generation
        }
        # Persist intent before the ROS submission. This is deliberately before
        # the submission lock to avoid inverting AgentCore/ROS lock ordering;
        # Stage 3 revalidates all conditions after the durable write.
        try:
            on_meta = getattr(self, 'on_command_meta', None)
            if not callable(on_meta):
                raise RuntimeError('command journal callback unavailable')
            on_meta(cid, submission_intent=True, submission_agent_boot_id=self.agent_boot_id,
                    submission_mission_id=mid, submission_context=sc2,
                    goal_uuid=reserved_uuid.hex, submission_uncertainty='MAY_HAVE_SENT')
        except Exception:
            return abort('RECONCILIATION_REQUIRED', 'submission intent could not be durably recorded')
        self._set_mission(mid, cid, 'DISPATCHED', 'sending goal to Nav2')
        # Stage 3 (Atomic Submission Fence): re-verify and submit under self.lock so any Gate/Nav2 callback
        # occurring between Stage 2 and send_goal_async aborts cleanly rather than submitting under fault.
        fut = None
        abort_pre = None
        send_failed = False
        # R1.4.4: the final check and the START of the local submission are one step with respect to authority changes: the Agent run
        # loop takes the same contract_fence to change the contract/reconciliation state. Whichever enters first is linearized first:
        # an invalidation that enters first makes Stage 3 refuse; one that waits for us happens after send_goal_async was called and is
        # then handled as a latched cancel duty (the goal record carries the old generation). Nothing below waits for the Future.
        with self._submission_fence(), self.lock:
            ok3, st3, detail3, sc3 = self._verify_nav_preconditions(cmd)
            if not ok3:
                abort_pre = (st3, f'ABORTED_PRE_SEND: {detail3}')
            else:
                rec['safety_context'] = sc3
                rec['goal_sent_mono'] = time.monotonic()
                self.active = rec
                try:
                    fut = self.nav.send_goal_async(goal, feedback_callback=lambda fb: self._on_fb(rec, fb), goal_uuid=ros_goal_uuid)
                except Exception:
                    rec['goal_response_lost'] = True
                    rec['reconciliation_required'] = True
                    send_failed = True
        if abort_pre is not None:
            return abort(abort_pre[0], abort_pre[1])
        if send_failed or fut is None:
            ack('RECONCILIATION_REQUIRED', 'Nav2 submission outcome unknown; goal was not retried')
            self._set_mission(mid, cid, 'RECONCILIATION_REQUIRED', 'Nav2 submission outcome unknown')
            return
        fut.add_done_callback(lambda f: self._on_goal_response(rec, f))
        # Immediately re-check for faults that arrived right around submission so cancel begins without delay.
        self._monitor_active_goal()

    def _monitor_active_goal(self):
        mp, _ = self._motor_path_status()
        motor_path_ok = mp in ('LOCKED', 'SIM_GATED')
        op_ctx, map_ctx, loc_ctx = self._producer_contexts()
        session_ctx = self._session_context()
        timeout_notify = None
        cancel_rec = None
        fault_logged = None
        with self.lock:
            a = self.active
            if a is None or a.get('cancel_timeout_reported') or a.get('goal_response_lost') or a.get('terminal_status') is not None:
                return
            now_m = time.monotonic()
            if a.get('cancel_started_mono') is not None:
                if (now_m - a['cancel_started_mono']) >= self.cancel_result_timeout_s:
                    a['cancel_timeout_reported'] = True
                    a['reconciliation_required'] = True
                    timeout_notify = (a, 'Nav2 cancel result timeout; terminal state unconfirmed', list(a.get('cancel_acks', [])))
            elif a.get('handle') is None and not a.get('cancel_requested') and a.get('goal_sent_mono') is not None and (now_m - a['goal_sent_mono']) >= self.goal_response_timeout_s:
                a['cancel_requested'] = True
                a['fault_cancel_reason'] = 'GOAL_RESPONSE_TIMEOUT'
                a['cancel_timeout_reported'] = True
                a['reconciliation_required'] = True
                timeout_notify = (a, 'Nav2 goal response timeout; terminal state unconfirmed', list(a.get('cancel_acks', [])))
            elif not a.get('cancel_requested'):
                gate_eval = self._gate_evaluation()
                fault = check_active_goal_safety(
                    a,
                    allow_missions=self.allow_missions,
                    gate_eval=gate_eval,
                    motor_path_ok=motor_path_ok,
                    op_ctx=op_ctx,
                    map_ctx=map_ctx,
                    loc_ctx=loc_ctx,
                    session_ctx=session_ctx,
                    disconnect_cancel_after_s=self.disconnect_cancel_after_s,
                )
                if fault is not None:
                    a['cancel_requested'] = True
                    a['fault_cancel_reason'] = fault
                    a['cancel_started_mono'] = now_m
                    fault_logged = (a['command_id'], fault)
                    if a['handle'] is not None:
                        cancel_rec = a
        if timeout_notify is not None:
            a_rec, reason, cacks = timeout_notify
            a_rec['ack']('RECONCILIATION_REQUIRED', reason)
            self._set_mission(a_rec['mission_id'], a_rec['command_id'], 'RECONCILIATION_REQUIRED', reason)
            for _, cack in cacks:
                cack('RECONCILIATION_REQUIRED', reason)
            return
        if fault_logged is not None and hasattr(self, 'get_logger'):
            self.get_logger().warning('SAFETY_CANCEL command=' + fault_logged[0] + ' reason=' + fault_logged[1])
        if cancel_rec is not None:
            self._request_cancel(cancel_rec)

    def _on_goal_response(self, rec, fut):
        with self.lock:
            if rec.get('goal_response_done') or rec.get('handle') is not None or rec.get('goal_response_lost') or rec.get('terminal_status') is not None:
                return
            rec['goal_response_done'] = True
        try: h = fut.result()
        except Exception:
            with self.lock:
                rec['goal_response_lost'] = True
                rec['reconciliation_required'] = True
                cacks = list(rec['cancel_acks'])
            rec['ack']('RECONCILIATION_REQUIRED', 'Nav2 goal response lost; goal was not resubmitted')
            self._set_mission(rec['mission_id'], rec['command_id'], 'RECONCILIATION_REQUIRED', 'goal response lost')
            for _, cack in cacks: cack('RECONCILIATION_REQUIRED', 'goal acceptance unknown; cancel not confirmed')
            return
        with self.lock:
            stale_rec = (getattr(self, 'active', None) is not rec or
                         rec.get('submission_agent_boot_id', self.agent_boot_id) != self.agent_boot_id)
        if stale_rec:
            if h.accepted:
                if _goal_uuid_hex(getattr(h, 'goal_id', None)) == rec.get('goal_uuid') and rec.get('goal_uuid'):
                    try: h.cancel_goal_async()
                    except Exception: pass
                rec['ack']('RECONCILIATION_REQUIRED', 'late acceptance belongs to an inactive submission')
            return
        goal_uuid = _goal_uuid_hex(getattr(h, 'goal_id', None))
        if goal_uuid != rec.get('goal_uuid'):
            with self.lock:
                rec['reconciliation_required'] = True
                rec['cancel_requested'] = True
            # An unrelated handle must never be canceled or treated as proof
            # about the reserved UUID, including when it was rejected.
            rec['ack']('RECONCILIATION_REQUIRED', 'goal response UUID does not match durable reservation')
            self._set_mission(rec['mission_id'], rec['command_id'], 'RECONCILIATION_REQUIRED', 'goal response UUID mismatch')
            return
        if not h.accepted:
            with self.lock:
                if (getattr(self, 'active', None) is not rec or
                        rec.get('submission_agent_boot_id', self.agent_boot_id) != self.agent_boot_id):
                    return
                rec['terminal_status'] = 'REJECTED'
                if self.active is rec: self.active = None
                cacks = list(rec['cancel_acks'])
            rec['ack']('REJECTED', 'Nav2 rejected the goal'); self._set_mission(rec['mission_id'], rec['command_id'], 'REJECTED', 'Nav2 rejected the goal')
            for _, cack in cacks: cack('REJECTED', 'Nav2 rejected the original goal; no canceled result')
            return
        with self.lock:
            stale_rec = (getattr(self, 'active', None) is not rec or
                         rec.get('submission_agent_boot_id', self.agent_boot_id) != self.agent_boot_id)
            if not stale_rec:
                rec['handle'] = h
                rec['goal_uuid'] = goal_uuid
                already_reconciling = bool(rec.get('reconciliation_required') or rec.get('cancel_timeout_reported') or not goal_uuid)
                need_cancel = bool(rec.get('cancel_requested')) or already_reconciling
                if need_cancel:
                    rec['cancel_requested'] = True
                    rec['cancel_started_mono'] = rec.get('cancel_started_mono') or time.monotonic()
        if stale_rec:
            # This handle came from our earlier request, but it must never
            # overwrite the current active goal. Best-effort targeted cancel.
            if goal_uuid == rec.get('goal_uuid') and goal_uuid:
                try: h.cancel_goal_async()
                except Exception: pass
            rec['ack']('RECONCILIATION_REQUIRED', 'late acceptance belongs to an inactive submission')
            return
        if hasattr(self, 'get_logger'):
            tag = 'NAV2_LATE_ACCEPTED_RECONCILING' if already_reconciling else 'NAV2_ACCEPTED'
            self.get_logger().info(tag + ' command=' + rec['command_id'] + ' goal=' + str(goal_uuid or ''))
        if not goal_uuid:
            rec['ack']('RECONCILIATION_REQUIRED', 'accepted goal UUID unavailable')
        on_meta = getattr(self, 'on_command_meta', None) if rec.get('owner') != 'frontier' else None
        if callable(on_meta) and goal_uuid:
            try:
                on_meta(rec['command_id'], goal_uuid=goal_uuid)
            except Exception:
                with self.lock:
                    rec['reconciliation_required'] = True
                    rec['cancel_requested'] = True
                    rec['cancel_started_mono'] = rec.get('cancel_started_mono') or time.monotonic()
                already_reconciling = True
                need_cancel = True
                rec['ack']('RECONCILIATION_REQUIRED', 'accepted goal UUID could not be durably recorded')
        if not already_reconciling and rec.get('owner') == 'frontier':
            rec['ack']('ACCEPTED', 'goal accepted by Nav2', {'goal_uuid': goal_uuid})
        elif not already_reconciling:
            rec['ack']('ACCEPTED', 'goal accepted by Nav2'); self._set_mission(rec['mission_id'], rec['command_id'], 'ACCEPTED', 'goal accepted by Nav2')
        h.get_result_async().add_done_callback(lambda f, u=goal_uuid: self._on_result(rec, f, expected_goal_uuid=u))
        if need_cancel:
            self._request_cancel(rec)
        else:
            self._monitor_active_goal()

    def _on_fb(self, rec, fb):
        with self.lock:
            if self.active is not rec: return  # Ignore feedback arriving after a terminal result.
            if rec.get('reconciliation_required') or rec.get('cancel_timeout_reported') or rec.get('goal_response_lost') or rec.get('terminal_status') is not None:
                return
            fb_uuid = _goal_uuid_hex(getattr(fb, 'goal_id', None))
            if fb_uuid and rec.get('goal_uuid') and fb_uuid != rec['goal_uuid']:
                return
            now = time.monotonic()
            if now - rec['last_fb'] < 1.0: return
            rec['last_fb'] = now
        f = fb.feedback
        pr = {'distance_remaining_m': round(float(f.distance_remaining), 3), 'recoveries': int(f.number_of_recoveries), 'navigation_time_s': round(f.navigation_time.sec + f.navigation_time.nanosec * 1e-9, 1)}
        rec['ack']('EXECUTING', 'navigating', pr); self._set_mission(rec['mission_id'], rec['command_id'], 'EXECUTING', 'navigating', pr)

    def _on_result(self, rec, fut, expected_goal_uuid=None):
        with self.lock:
            if rec.get('submission_agent_boot_id', self.agent_boot_id) != self.agent_boot_id:
                return
            if rec.get('terminal_status') is not None or rec.get('goal_response_lost'):
                return
            if expected_goal_uuid is not None and rec.get('goal_uuid') is not None and rec.get('goal_uuid') != expected_goal_uuid:
                return
        try:
            res = fut.result()
            st = res.status
        except Exception:
            with self.lock:
                if rec.get('terminal_status') is not None or rec.get('goal_response_lost'):
                    return
                rec['handle'] = None
                rec['goal_response_lost'] = True
                rec['reconciliation_required'] = True
                cacks = list(rec['cancel_acks'])
            rec['ack']('RECONCILIATION_REQUIRED', 'Nav2 result lost; outcome unknown')
            self._set_mission(rec['mission_id'], rec['command_id'], 'RECONCILIATION_REQUIRED', 'Nav2 result lost; outcome unknown')
            for _, cack in cacks: cack('RECONCILIATION_REQUIRED', 'Nav2 result lost; cancel not confirmed')
            return
        res_uuid = _goal_uuid_hex(getattr(res, 'goal_id', None))
        if st not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            with self.lock:
                if rec.get('terminal_status') is not None or rec.get('goal_response_lost'):
                    return
                rec['handle'] = None
                rec['goal_response_lost'] = True
                rec['reconciliation_required'] = True
                cacks = list(rec['cancel_acks'])
            rec['ack']('RECONCILIATION_REQUIRED', f'unexpected Nav2 result status {st}; outcome unknown')
            self._set_mission(rec['mission_id'], rec['command_id'], 'RECONCILIATION_REQUIRED', 'unexpected Nav2 result')
            for _, cack in cacks: cack('RECONCILIATION_REQUIRED', 'unexpected Nav2 result; cancel not confirmed')
            return
        s, d = {GoalStatus.STATUS_SUCCEEDED: ('SUCCEEDED', 'goal reached'), GoalStatus.STATUS_CANCELED: ('CANCELED', 'canceled'), GoalStatus.STATUS_ABORTED: ('FAILED', 'Nav2 aborted the goal')}.get(st, ('FAILED', f'unexpected Nav2 status {st}'))
        # A terminal status is not published or used to clear ownership until
        # the actual result's UUID and status have been persisted in the journal.
        with self.lock:
            if (getattr(self, 'active', rec) is not rec or not rec.get('goal_uuid') or
                    (expected_goal_uuid is not None and rec['goal_uuid'] != expected_goal_uuid) or
                    (res_uuid and res_uuid != rec['goal_uuid'])):
                return
        try:
            on_meta = getattr(self, 'on_command_meta', None)
            if rec.get('owner') == 'frontier':
                pass  # mapping goals are journaled by the mapping worker, not the Fleet command journal
            elif not callable(on_meta): raise RuntimeError('command journal unavailable')
            else:
                on_meta(rec['command_id'], goal_uuid=rec['goal_uuid'], terminal_status=s,
                        terminal_source='nav2_action_result', terminal_agent_boot_id=self.agent_boot_id)
        except Exception:
            with self.lock: rec['reconciliation_required'] = True
            rec['ack']('RECONCILIATION_REQUIRED', 'Nav2 terminal result could not be durably recorded')
            return
        with self.lock:
            fault_reason = rec.get('fault_cancel_reason')
        if s == 'CANCELED' and fault_reason:
            d = f'canceled due to safety fault: {fault_reason}'
        # Persist the terminal ACK before dropping the owned handle. A state
        # snapshot may otherwise expose neither ownership nor durable outcome.
        if rec['ack'](s, d) is False:
            with self.lock: rec['reconciliation_required'] = True
            return
        with self.lock:
            if rec.get('terminal_status') is not None or rec.get('goal_response_lost'):
                return
            if expected_goal_uuid is not None and rec.get('goal_uuid') is not None and rec.get('goal_uuid') != expected_goal_uuid:
                return
            if res_uuid and rec.get('goal_uuid') and res_uuid != rec['goal_uuid']:
                return
            if getattr(self, 'active', rec) is not rec:
                return
            rec['terminal_status'] = s
            cacks = list(rec['cancel_acks'])
            if self.active is rec: self.active = None
        self._set_mission(rec['mission_id'], rec['command_id'], s, d)
        for _, cack in cacks:
            cack('SUCCEEDED' if s == 'CANCELED' else 'REJECTED', 'Nav2 terminal CANCELED confirmed' if s == 'CANCELED' else 'goal terminated as ' + s + '; cancellation not confirmed')

    def _do_cancel(self, cmd, ack):
        mid = cmd.get('mission_id')
        owned_rec = None
        has_handle = False
        lm = {}
        with self.lock:
            a = self.active
            if a is not None and a['mission_id'] == mid:
                a['cancel_requested'] = True
                a['cancel_started_mono'] = a.get('cancel_started_mono') or time.monotonic()
                a['cancel_acks'].append((cmd['command_id'], ack))
                has_handle = a['handle'] is not None
                owned_rec = a
            else:
                lm = dict(self.last_mission) if self.last_mission else {}
        if owned_rec is not None:
            ack('ACCEPTED', 'cancel forwarded to Nav2')
            if has_handle:
                self._request_cancel(owned_rec)
            return
        if lm.get('active_mission_id') == mid and lm.get('state') == 'CANCELED':
            ack('SUCCEEDED', 'previous Nav2 terminal CANCELED confirmed')
        elif lm.get('active_mission_id') == mid and lm.get('state') in ('SUCCEEDED', 'FAILED', 'REJECTED', 'EXPIRED'):
            # This agent itself observed the terminal Nav2 result of that goal: there is nothing left to cancel. Terminal, journal-safe (does not block later goals).
            ack('REJECTED', f"NOTHING_TO_CANCEL: the goal had already ended with {lm['state']} (observed by this agent)")
        else:
            ng = self._nav2_goals()
            note = ' (foreign Nav2 goal(s) ignored)' if ng.get('foreign_active_goal_uuids') else ''
            ack('RECONCILIATION_REQUIRED', f'no owned goal handle for this mission{note}; cancellation cannot be confirmed')

    def _request_cancel(self, rec):
        with self.lock:
            if rec['cancel_sent'] or rec['handle'] is None or rec.get('terminal_status') is not None:
                return
            rec['cancel_sent'] = True
            handle = rec['handle']
        try:
            fut = handle.cancel_goal_async()
        except Exception as e:
            msg = f'Nav2 cancel_goal_async error: {e}'
            with self.lock:
                if rec.get('terminal_status') is not None:
                    return
                rec['reconciliation_required'] = True
                cacks = list(rec['cancel_acks'])
                fault_r = rec.get('fault_cancel_reason')
            for _, ack in cacks:
                ack('RECONCILIATION_REQUIRED', msg)
            if fault_r:
                rec['ack']('RECONCILIATION_REQUIRED', msg)
            return
        def response(f):
            try:
                result = f.result()
                accepted = result.return_code == 0 and any(bytes(g.goal_id.uuid) == bytes(handle.goal_id.uuid) for g in result.goals_canceling)
            except Exception: accepted = False
            if not accepted:
                with self.lock:
                    if rec.get('terminal_status') is not None or getattr(self, 'active', rec) is not rec:
                        return
                    cacks = list(rec['cancel_acks'])
                for _, ack in cacks: ack('RECONCILIATION_REQUIRED', 'Nav2 cancel rejected or response lost; awaiting terminal result')
            # A cancel-service response alone is NOT a terminal canceled result.
        fut.add_done_callback(response)

    def _set_mission(self, mid, cid, state, detail, progress=None):
        with self.lock:
            cur = self.last_mission if isinstance(getattr(self, 'last_mission', None), dict) else {}
            if cur.get('active_mission_id') == mid and cur.get('command_id') == cid:
                cur_state = cur.get('state')
                if cur_state in _MISSION_TERMINAL and state != cur_state:
                    return
                if cur_state == 'RECONCILIATION_REQUIRED' and state not in _MISSION_TERMINAL and state != 'RECONCILIATION_REQUIRED':
                    return
                cur_rank = _MISSION_RANK.get(cur_state, -1)
                new_rank = _MISSION_RANK.get(state, -1)
                if cur_rank >= 0 and new_rank >= 0 and new_rank < cur_rank:
                    return
            self.last_mission = {'active_mission_id': mid, 'command_id': cid, 'state': state, 'detail': detail, 'progress': progress}
