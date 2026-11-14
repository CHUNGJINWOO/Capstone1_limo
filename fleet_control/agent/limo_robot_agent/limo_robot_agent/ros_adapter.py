"""ROS 2 side of the agent. READ-ONLY with respect to the existing stack: it subscribes to topics, reads TF and looks at the ROS graph; the only thing
it ever sends is a Nav2 NavigateToPose goal for an accepted mission, and only if every interlock passes:
  1. missions enabled on this agent (default OFF for the real robot)   2. the navigate_to_pose action server is ready
  3. No /cmd_vel publisher in default/real mode. An explicit sim-only opt-in
     permits one Safety Gate endpoint with simulated time; all other publishers are refused.
It never publishes velocity and never calls the gate's reset service."""
import collections, json, math, threading, time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .pngmap import grid_png, grid_stats
from .motor_policy import motor_path_allowed

try:
    from limo_msgs.msg import LimoStatus
except Exception:                                                     # limo_msgs not installed: battery stays UNKNOWN
    LimoStatus = None


def yaw_of(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RosRobot(Node):
    def __init__(self, allow_missions, mode='real', simulated=False, allow_sim_safety_gate=False):
        super().__init__('fleet_robot_agent')
        self.allow_missions = allow_missions
        self.mode, self.simulated, self.allow_sim_safety_gate = mode, simulated, allow_sim_safety_gate
        self.execution_allowed = lambda: False  # AgentCore enables this only after ownership reconciliation.
        self.lock = threading.RLock()
        self.t = {}                                                    # topic key -> arrival monotonic time
        self.scan_times = collections.deque(maxlen=40); self.scan_beams = collections.deque(maxlen=60); self.scan_frame = None
        self.gate = None; self.depth = None; self.limo = None; self.odom = None
        self.map_a = None; self.map_meta = None; self.map_png = None; self.map_version = 0; self.map_hash = None; self.map_changed_t = None; self.map_first_t = None
        self.tfb = Buffer(); self.tfl = TransformListener(self.tfb, self)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(String, '/stall_detector/status_json', lambda m: self._set('gate', m), 10)
        self.create_subscription(String, '/auto_mapping/depth_status', lambda m: self._set('depth', m), 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        if LimoStatus is not None: self.create_subscription(LimoStatus, '/limo_status', self._on_limo, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Evidence for operator recovery: which goals does the Nav2 action server itself consider active? (status topic is transient-local, published on change)
        self.nav_status = None
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status', self._on_nav_status, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.active = None                                             # dict(mission_id, command_id, handle, ack, last_fb_t)
        self.last_mission = {}
        self.tasks = collections.deque(); self.create_timer(0.05, self._run_tasks)

    # ------------------------------------------------------------ subscriptions
    def _stamp(self, key): self.t[key] = time.monotonic()
    def _age(self, key): return None if key not in self.t else time.monotonic() - self.t[key]

    def _on_scan(self, m):
        self._stamp('scan'); self.scan_times.append(time.monotonic()); self.scan_beams.append(len(m.ranges)); self.scan_frame = m.header.frame_id

    def _on_nav_status(self, m):
        self.nav_status = {'t': time.monotonic(), 'active': sum(1 for g in m.status_list if g.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)), 'total': len(m.status_list)}

    def _nav2_goals(self):
        pubs = self.count_publishers('navigate_to_pose/_action/status')
        st = self.nav_status
        return {'source': 'navigate_to_pose/_action/status', 'known': pubs >= 1, 'publishers': pubs, 'active_count': (st['active'] if st else 0) if pubs >= 1 else None,
                'received_status_array': st is not None, 'last_change_age_s': None if not st else round(time.monotonic() - st['t'], 1)}

    def _on_odom(self, m): self._stamp('odom'); self.odom = (m.header.frame_id, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))

    def _on_limo(self, m): self._stamp('limo'); self.limo = {'voltage': float(m.battery_voltage), 'vehicle_state': int(m.vehicle_state), 'control_mode': int(m.control_mode), 'motion_mode': int(m.motion_mode), 'error_code': int(m.error_code)}

    def _set(self, key, m):
        self._stamp(key)
        try: v = json.loads(m.data)
        except Exception: v = None
        if key == 'gate': self.gate = v
        else: self.depth = v

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
    def snapshot(self):
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
        a = self._age('gate'); gate = {'status': 'NO_DATA', 'age_s': None}
        if a is not None and self.gate:
            g = self.gate; gate = {'status': 'NO_DATA' if a > 2.0 else str(g.get('state', 'UNKNOWN')), 'age_s': round(a, 2), 'state': g.get('state'), 'cmd_blocked': g.get('cmd_blocked'),
                                   'wheel_odom_stale': g.get('wheel_odom_stale'), 'est_odom_stale': g.get('est_odom_stale'), 'depth_stale': g.get('depth_stale'),
                                   'counters': {k: g.get(k) for k in ('stall_events', 'failsafe_events', 'icp_degraded_events', 'operator_unlatch_count')}}
            if gate['status'] not in ('NORMAL', 'NO_DATA'): errs.append(f"safety gate state {gate['status']}")
        try:
            pubs = sorted({i.node_name for i in self.get_publishers_info_by_topic('/cmd_vel')}); lpubs = sorted({i.node_name for i in self.get_publishers_info_by_topic('/cmd_vel_locked')})
        except Exception: pubs, lpubs = None, None
        mp = 'UNKNOWN' if pubs is None else ('ARMED' if pubs else ('LOCKED' if lpubs else 'UNKNOWN'))
        gate['motor_path'] = {'status': mp, 'cmd_vel_publishers': pubs, 'cmd_vel_locked_publishers': lpubs, 'meaning': 'ARMED = something publishes /cmd_vel (motors can move); LOCKED = no /cmd_vel publisher and the gate publishes /cmd_vel_locked'}
        if mp == 'ARMED' and self._motor_path_allowed(self.get_publishers_info_by_topic('/cmd_vel')): mp = 'SIM_GATED'; gate['motor_path']['status'] = mp
        if mp == 'ARMED': errs.append('motor path ARMED: /cmd_vel has a publisher')
        # --- battery
        bat = {'status': 'UNKNOWN', 'voltage_v': None, 'percent': None, 'source': '/limo_status' if LimoStatus else 'limo_msgs not available'}
        a = self._age('limo')
        if a is not None and self.limo and a < 5.0:
            bat.update({'status': 'VOLTAGE_ONLY', 'voltage_v': round(self.limo['voltage'], 2), 'age_s': round(a, 1), 'note': 'percent is not reported by the driver; no voltage-to-percent conversion is assumed'})
            for k in ('vehicle_state', 'control_mode', 'motion_mode', 'error_code'): bat[k] = self.limo[k]
            if self.limo['error_code']: errs.append(f"LIMO error_code {self.limo['error_code']}")
        # --- nav2
        ready = self.nav.server_is_ready(); nav2 = {'status': 'ACTIVE_GOAL' if self.active else ('READY' if ready else 'UNAVAILABLE'), 'action_server_ready': bool(ready), 'action': 'navigate_to_pose'}
        with self.lock: mis = dict(self.last_mission) if self.last_mission else {}
        a = self.active
        execution = {'active': None, 'pending_cancel_ids': [], 'nav2_goals': self._nav2_goals()}
        if a:
            execution['active'] = {'mission_id': a['mission_id'], 'command_id': a['command_id'], 'goal_handle_available': a['handle'] is not None}
            execution['pending_cancel_ids'] = [cid for cid, ack in a.get('cancel_acks', [])]
        return {'schema_version': 1, 'pose': pose, 'map': m, 'nav2': nav2, 'battery': bat, 'lidar': lidar, 'depth_camera': depth, 'safety_gate': gate, 'mission': mis, 'execution': execution, 'errors': errs}

    def map_for_upload(self, last_version):
        with self.lock:
            if self.map_meta is None or self.map_version == last_version: return None
            if self.map_png is None: self.map_png = grid_png(self.map_a)
            return self.map_version, dict(self.map_meta), self.map_png

    # ------------------------------------------------------------ mission execution (called from the agent thread -> queued to the ROS thread)
    def start_navigate(self, cmd, ack): self.tasks.append(('nav', cmd, ack))
    def cancel_mission(self, cmd, ack): self.tasks.append(('cancel', cmd, ack))

    def _run_tasks(self):
        while self.tasks:
            kind, cmd, ack = self.tasks.popleft()
            try: (self._do_nav if kind == 'nav' else self._do_cancel)(cmd, ack)
            except Exception as e: ack('FAILED' if kind == 'nav' else 'REJECTED', f'agent error: {e}')

    def _motor_path_allowed(self, publishers):
        return motor_path_allowed(publishers, mode=getattr(self, 'mode', 'real'),
                                  simulated=getattr(self, 'simulated', False),
                                  allow_sim_safety_gate=getattr(self, 'allow_sim_safety_gate', False),
                                  use_sim_time=bool(self.get_parameter('use_sim_time').value))

    def _do_nav(self, cmd, ack):
        mid, g = cmd.get('mission_id'), (cmd.get('payload') or {}).get('goal') or {}
        def rej(code, why): ack('REJECTED', f'{code}: {why}'); self._set_mission(mid, cmd['command_id'], 'REJECTED', f'{code}: {why}')
        if not self.allow_missions: return rej('MISSIONS_DISABLED', 'mission execution is disabled on this agent')
        if not self.execution_allowed():
            ack('RECONCILIATION_REQUIRED', 'server not reconciled; no goal sent'); return
        if time.time() > cmd.get('expires_at', 0): return rej('EXPIRED', 'command expired while queued')
        try: x, y, yaw = float(g['x']), float(g['y']), float(g.get('yaw_rad', 0.0))
        except Exception: return rej('BAD_GOAL', 'goal needs numeric x, y')
        if g.get('frame_id') != 'map' or not all(math.isfinite(v) for v in (x, y, yaw)): return rej('BAD_GOAL', "goal frame must be 'map' with finite values")
        if self.active: return rej('BUSY', f"mission {self.active['mission_id']} is still active")
        try: pubs = self.get_publishers_info_by_topic('/cmd_vel')
        except Exception: pubs = None
        if pubs is None or (pubs and not self._motor_path_allowed(pubs)): return rej('MOTOR_PATH_ARMED', f'/cmd_vel publishers: {sorted(i.node_name for i in pubs) if pubs is not None else "unknown"}; stage 1 never sends goals while the motor path may be live')
        if not self.nav.wait_for_server(timeout_sec=2.0): return rej('NAV2_UNAVAILABLE', 'navigate_to_pose action server not available')
        if not self.execution_allowed() or time.time() > cmd.get('expires_at', 0):
            ack('RECONCILIATION_REQUIRED', 'connection/deadline changed before Nav2 submission; no goal sent'); return
        goal = NavigateToPose.Goal(); goal.pose.header.frame_id = 'map'; goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y; goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        rec = {'mission_id': mid, 'command_id': cmd['command_id'], 'handle': None, 'ack': ack, 'last_fb': 0.0, 'cancel_requested': False, 'cancel_acks': [], 'cancel_sent': False}
        self.active = rec; self._set_mission(mid, cmd['command_id'], 'DISPATCHED', 'sending goal to Nav2')
        try: fut = self.nav.send_goal_async(goal, feedback_callback=lambda fb: self._on_fb(rec, fb))
        except Exception:
            ack('RECONCILIATION_REQUIRED', 'Nav2 submission outcome unknown; goal was not retried')
            self._set_mission(mid, cmd['command_id'], 'RECONCILIATION_REQUIRED', 'Nav2 submission outcome unknown')
            return
        fut.add_done_callback(lambda f: self._on_goal_response(rec, f))

    def _on_goal_response(self, rec, fut):
        try: h = fut.result()
        except Exception:
            rec['ack']('RECONCILIATION_REQUIRED', 'Nav2 goal response lost; goal was not resubmitted')
            self._set_mission(rec['mission_id'], rec['command_id'], 'RECONCILIATION_REQUIRED', 'goal response lost')
            for cid, ack in rec['cancel_acks']: ack('RECONCILIATION_REQUIRED', 'goal acceptance unknown; cancel not confirmed')
            return
        if not h.accepted:
            self.active = None; rec['ack']('REJECTED', 'Nav2 rejected the goal'); self._set_mission(rec['mission_id'], rec['command_id'], 'REJECTED', 'Nav2 rejected the goal')
            for cid, ack in rec['cancel_acks']: ack('REJECTED', 'Nav2 rejected the original goal; no canceled result')
            return
        if hasattr(self, 'get_logger'):
            self.get_logger().info('NAV2_ACCEPTED command=' + rec['command_id'] + ' goal=' + bytes(h.goal_id.uuid).hex())
        rec['handle'] = h; rec['ack']('ACCEPTED', 'goal accepted by Nav2'); self._set_mission(rec['mission_id'], rec['command_id'], 'ACCEPTED', 'goal accepted by Nav2')
        if rec['cancel_requested']: self._request_cancel(rec)
        h.get_result_async().add_done_callback(lambda f: self._on_result(rec, f))

    def _on_fb(self, rec, fb):
        if self.active is not rec: return  # Ignore feedback arriving after a terminal result.
        now = time.monotonic()
        if now - rec['last_fb'] < 1.0: return
        rec['last_fb'] = now; f = fb.feedback
        pr = {'distance_remaining_m': round(float(f.distance_remaining), 3), 'recoveries': int(f.number_of_recoveries), 'navigation_time_s': round(f.navigation_time.sec + f.navigation_time.nanosec * 1e-9, 1)}
        rec['ack']('EXECUTING', 'navigating', pr); self._set_mission(rec['mission_id'], rec['command_id'], 'EXECUTING', 'navigating', pr)

    def _on_result(self, rec, fut):
        try: st = fut.result().status
        except Exception:
            rec['handle'] = None
            rec['ack']('RECONCILIATION_REQUIRED', 'Nav2 result lost; outcome unknown')
            self._set_mission(rec['mission_id'], rec['command_id'], 'RECONCILIATION_REQUIRED', 'Nav2 result lost; outcome unknown')
            for cid, ack in rec['cancel_acks']: ack('RECONCILIATION_REQUIRED', 'Nav2 result lost; cancel not confirmed')
            return
        if st not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
            rec['handle'] = None
            rec['ack']('RECONCILIATION_REQUIRED', f'unexpected Nav2 result status {st}; outcome unknown')
            self._set_mission(rec['mission_id'], rec['command_id'], 'RECONCILIATION_REQUIRED', 'unexpected Nav2 result')
            for cid, ack in rec['cancel_acks']: ack('RECONCILIATION_REQUIRED', 'unexpected Nav2 result; cancel not confirmed')
            return
        s, d = {GoalStatus.STATUS_SUCCEEDED: ('SUCCEEDED', 'goal reached'), GoalStatus.STATUS_CANCELED: ('CANCELED', 'canceled'), GoalStatus.STATUS_ABORTED: ('FAILED', 'Nav2 aborted the goal')}.get(st, ('FAILED', f'unexpected Nav2 status {st}'))
        rec['ack'](s, d); self._set_mission(rec['mission_id'], rec['command_id'], s, d)
        for cid, ack in rec['cancel_acks']:
            ack('SUCCEEDED' if s == 'CANCELED' else 'REJECTED', 'Nav2 terminal CANCELED confirmed' if s == 'CANCELED' else 'goal terminated as ' + s + '; cancellation not confirmed')
        if self.active is rec: self.active = None

    def _do_cancel(self, cmd, ack):
        mid = cmd.get('mission_id'); a = self.active
        if a is None or a['mission_id'] != mid:
            lm = self.last_mission or {}
            if lm.get('active_mission_id') == mid and lm.get('state') == 'CANCELED':
                ack('SUCCEEDED', 'previous Nav2 terminal CANCELED confirmed')
            elif lm.get('active_mission_id') == mid and lm.get('state') in ('SUCCEEDED', 'FAILED', 'REJECTED', 'EXPIRED'):
                # This agent itself observed the terminal Nav2 result of that goal: there is nothing left to cancel. Terminal, journal-safe (does not block later goals).
                ack('REJECTED', f"NOTHING_TO_CANCEL: the goal had already ended with {lm['state']} (observed by this agent)")
            else: ack('RECONCILIATION_REQUIRED', 'no owned goal handle for this mission; cancellation cannot be confirmed')
            return
        a['cancel_requested'] = True; ack('ACCEPTED', 'cancel forwarded to Nav2')
        a['cancel_acks'].append((cmd['command_id'], ack))
        if a['handle'] is not None: self._request_cancel(a)

    def _request_cancel(self, rec):
        if rec['cancel_sent']: return
        rec['cancel_sent'] = True
        fut = rec['handle'].cancel_goal_async()
        def response(f):
            try:
                result = f.result()
                accepted = result.return_code == 0 and any(bytes(g.goal_id.uuid) == bytes(rec['handle'].goal_id.uuid) for g in result.goals_canceling)
            except Exception: accepted = False
            if not accepted:
                for cid, ack in rec['cancel_acks']: ack('RECONCILIATION_REQUIRED', 'Nav2 cancel rejected or response lost; awaiting terminal result')
            # A cancel-service response alone is NOT a terminal canceled result.
        fut.add_done_callback(response)

    def _set_mission(self, mid, cid, state, detail, progress=None):
        with self.lock: self.last_mission = {'active_mission_id': mid, 'command_id': cid, 'state': state, 'detail': detail, 'progress': progress}
