"""Agent-independent motion watchdog (separate process, never imported by the Agent).

Placement (velocity path, Safety Gate unchanged):
    Nav2 controller/smoother + behavior_server -> /fleet/nav_cmd_vel -> [motion_watchdog] -> /cmd_vel_raw -> Safety Gate

States
  LOCKED            start-up or after a fault: zero velocity, inputs dropped. Leaves only with positive evidence:
                    continuous healthy heartbeat for rearm_hold_s AND an idle proof (below).
  MOTION_AUTHORIZED inputs are forwarded unchanged (zero commands included; non-finite commands are replaced by zero).
  FAILSAFE          heartbeat lost / explicitly unhealthy / control loop stalled / Gate status lost / foreign or
                    duplicate Nav2 goal source / this process itself was not scheduled for self_stall_timeout_s: zero velocity, cancel-all retried until a new idle proof; then LOCKED.
                    A goal running before the fault can therefore never resume driving.

Idle proof (all required; an empty goals_canceling list alone is NOT proof: an rclpy action server whose cancel
callback rejects returns return_code 0 with an empty list for an ACTIVE goal, observed 2026-11-20):
  * exactly one publisher on the NavigateToPose status topic (one action server; a duplicate could hide goals),
    the same publisher GID seen for at least status_settle_s (so its transient-local status had time to arrive;
    the status subscription is re-created whenever the publisher set changes so a retained status is re-delivered),
  * the cancel-all answer has return_code ERROR_NONE and an empty goals_canceling list, at most idle_proof_max_age_s old,
  * the newest status from the current subscription shows no non-terminal goal, or no status arrived at all.
    Action servers publish status only on goal state changes; an idle server that never had a goal publishes
    nothing (rclcpp and rclpy Humble servers checked 2026-11-20). A goal acceptance always publishes a status, so
    "settled unique server, nothing received" plus the cancel-all answer is idle evidence; message AGE is not used.
  * A change of the status publisher GID while authorized is a fault (the previous observations no longer apply).

Goal ownership while authorized: every non-terminal goal in the status array must be one the Agent declares in its
heartbeat ('goal_uuids'); an undeclared goal persisting for foreign_goal_grace_s trips FAILSAFE (explore_lite, RViz,
a second client, or a goal accepted in the race window after the idle proof).

If this process dies, /cmd_vel_raw stops and the Safety Gate zeroes its output after its 0.25 s stale-input timeout.
If the Gate dies, nothing here stops a motor driver that holds its last command (MCU timeout unverified).

Heartbeat (/fleet/agent_heartbeat, JSON) carries process liveness (it is published by the Agent's ROS executor),
control-loop liveness ('tasks_age_s') and the Agent's own goal UUIDs ('goal_uuids'). All must be fresh.
"""
import json
import math
import signal
import sys
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from . import topics
from .mode_policy import ModePolicy
from .speed_clamp import SCOPE_BY_NODE, WatchdogSpeedClamp
from .ros_graph import gid_hex

TERMINAL = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
LOCKED, AUTHORIZED, FAILSAFE = 'LOCKED', 'MOTION_AUTHORIZED', 'FAILSAFE'
STATUS_TOPIC = '/navigate_to_pose/_action/status'


def _finite_twist(m):
    return all(math.isfinite(v) for v in (m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z))


class MotionWatchdog(Node):
    def __init__(self):
        super().__init__(topics.WATCHDOG_NODE)
        p = lambda n, d: self.declare_parameter(n, d).value
        self.hb_timeout = float(p('heartbeat_timeout_s', 0.3))
        self.loop_timeout = float(p('control_loop_timeout_s', 0.5))
        self.rearm_hold = float(p('rearm_hold_s', 1.0))
        # 2026-11-27: while AUTHORIZED and nothing is being forwarded, publish an explicit zero every idle_zero_period_s (0 = off, the previous behaviour). It is the 'latest zero input'
        # a gated Gate reset needs from the single identified writer; a zero never starts motion and is not published between forwarded commands.
        self.idle_zero_period = float(p('idle_zero_period_s', 0.0))
        # 2026-11-27 (C4 investigation, diagnostics only, no behaviour): timing of the graph view versus the data seen on /fleet/speed_limit and the scheduling of this node's own timer
        self.sl_diag = {'msgs': 0, 'n_pubs_hist': {}, 'first_n2_t': None, 'first_msg_t': {}, 'ticks': 0, 'tick_gap_max': 0.0, 'tick_gaps_gt_150ms': 0, 'tick_gap_last': 0.0}
        self.probe_period = float(p('cancel_retry_s', 1.0))
        self.probe_timeout = float(p('cancel_response_timeout_s', 2.0))
        self.proof_max_age = float(p('idle_proof_max_age_s', 1.5))
        self.status_settle = float(p('status_settle_s', 0.5))
        self.status_timeout = float(p('status_timeout_s', 2.0))
        self.foreign_grace = float(p('foreign_goal_grace_s', 0.5))
        # Safety Gate liveness by DATA freshness (its 10 Hz status), not ROS graph presence: a SIGKILLed process stays
        # in the DDS graph until its participant lease expires (~20 s observed), far too slow for a stop decision.
        self.gate_timeout = float(p('gate_status_timeout_s', 0.5))
        # Own scheduling gap (this process frozen/starved): on resume a queued heartbeat must not let an old goal drive.
        self.self_stall = float(p('self_stall_timeout_s', 0.25))
        self.last_tick = None
        self.gate_t = None
        rate = float(p('publish_hz', 20.0))
        self.boot_id = uuid.uuid4().hex
        self.out = self.create_publisher(Twist, p('output_topic', topics.CMD_VEL_RAW), 10)
        self.status_pub = self.create_publisher(String, topics.WATCHDOG_STATUS, 10)
        self.input_topic = p('input_topic', topics.NAV_CMD_VEL)
        # Only these nodes may publish the input (Agent VelocityArbiter, or Nav2 velocity_smoother in the legacy mapping
        # stack). Any other or a second publisher could inject velocity past the Arbiter: fault, stop forwarding.
        self.input_nodes = tuple(p('input_publisher_nodes', ['fleet_robot_agent', 'velocity_smoother']))
        self.input_ok = True
        self.input_scope = None          # 'auto' / 'manual' by the single allowed input publisher; None = the Agent's own (Arbiter) output
        # G2 2026-11-23: optional speed clamp (real robot enforcement point). Off by default: every R2 behaviour is unchanged.
        self.clamp = None
        self.sl_pub_ok = False
        self.speed_limit_nodes = tuple(p('speed_limit_publisher_nodes', ['fleet_robot_agent']))
        if bool(p('speed_clamp', False)):
            self.clamp = WatchdogSpeedClamp(state_path=str(p('speed_limit_state_path', '')))
            self.create_subscription(String, p('speed_limit_topic', topics.SPEED_LIMIT), self.on_speed_limit, 10)
        # G2 2026-11-23: optional mode gate (needs the clamp's Agent sync): forward only the source allowed in the operating mode the Agent reports.
        self.mode_policy = ModePolicy() if (self.clamp is not None and bool(p('mode_gate', False))) else None
        # Second input channel (manual joystick), only with the mode gate. Its publisher rules mirror the nav input: at most one, and only an allowed node.
        self.manual_topic = str(p('manual_input_topic', '/fleet/manual_cmd_vel'))
        self.manual_nodes = tuple(p('manual_input_publisher_nodes', ['manual_mapping_teleop']))
        self.manual_input_ok, self.manual_pubs, self.nav_pubs = True, [], []
        self.source_conflict_t = None
        # MANUAL re-arm (the Arbiter's deadman rule): a joystick that is already deflected when MANUAL is granted is not forwarded until it has been released once
        self.man_key, self.man_window_end, self.man_rearmed, self.man_dirty, self.man_last = None, 0.0, False, False, None
        # Release-before-resume (2026-11-25): whenever this node leaves AUTHORIZED (and at boot) the joystick must be seen released (a zero command, or a silence gap > 0.5 s on the
        # manual channel, measured on every received message even while they are dropped) before any manual command is forwarded again. Without it a stick held through a Safety Gate
        # outage was forwarded again the instant the Gate status returned, to a restarted Gate that had armed against the zeros this node published meanwhile.
        # 2026-11-26: silence is NOT a release. A release is an explicit neutral (zero) command; a stream that was non-zero and then went silent for > 0.5 s, a new manual-channel
        # endpoint (same node name or not), and every non-authorized episode each require a neutral command before a non-zero one is forwarded again (then a fresh deflection).
        self.man_need_release, self.man_release_seen, self.man_rx_t, self.man_neutral, self.man_pub_gids = False, False, None, None, None
        if self.mode_policy is not None:
            self.create_subscription(Twist, self.manual_topic, self.on_manual_cmd, 10)
        self.last_out = (None, 0.0, 0.0)  # (monotonic time, linear.x, angular.z) of the last command this node published
        self.create_subscription(Twist, self.input_topic, self.on_cmd, 10)
        self.create_subscription(String, p('heartbeat_topic', topics.AGENT_HEARTBEAT), self.on_hb, 10)
        self.create_subscription(String, p('gate_status_topic', topics.GATE_STATUS), self.on_gate, 10)
        self.status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.status_sub = self.create_subscription(GoalStatusArray, STATUS_TOPIC, self.on_status, self.status_qos)
        self.status_gid = None          # GID of the unique status publisher the observations belong to
        self.status_identity_ok = None
        self.resubscribe_pending = False
        self.counters_resub = 0
        self.cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        self.state, self.reason = LOCKED, 'STARTUP'
        self._publish_lock = threading.Lock()
        self._stop_requested = False
        self._stop_reason = None
        self.shutting_down = False
        self.shutdown_reason = None
        self.hb_t = self.hb_since = self.hb_boot = None
        self.hb_unhealthy = False
        self.loop_age = None
        self.agent_goals = set()        # goal UUIDs (hex) the Agent declares as its own
        self.nav_active = None          # from status messages only; None = never received
        self.nav_status_t = None
        self.active_uuids = set()
        self.foreign_since = None
        self.status_pubs = 0
        self.status_pub_since = None
        self.idle_proof_t = None        # monotonic time the last accepted (empty, ERROR_NONE) cancel-all was SENT
        self.last_probe = 0.0
        self.probe_inflight = None      # monotonic send time of the outstanding probe
        self.last_cancel_rc = None
        self.t_fault = None
        self.last_fault = None
        self.counters = dict(trips=0, cancel_requests=0, cancel_nonempty=0, cancel_empty=0, cancel_rc_nonzero=0,
                             cancel_timeouts=0, dropped_inputs=0, nonfinite_inputs=0, forwarded=0, rearms=0)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(0.2, self.publish_status)

    def mark_stop_requested(self, reason='SHUTDOWN'):
        """Signal-safe (lock-free, no-I/O, no-publish) stop request marker.
        Immediately blocks any new non-zero publish or re-arm without acquiring _publish_lock;
        shutdown finalization (shutting_down=True + zero Twist) runs in normal execution context."""
        self._stop_requested = True
        if self._stop_reason is None:
            self._stop_reason = reason

    def _publish_zero(self):
        with self._publish_lock:
            self.out.publish(Twist())
            self.last_out = (time.monotonic(), 0.0, 0.0)

    def request_shutdown(self, reason='SHUTDOWN'):
        """Finalize fail-closed shutdown atomically with respect to velocity publishing under _publish_lock
        and emit zero Twist. Must be called from a normal execution context (never from a Python signal handler)."""
        self.mark_stop_requested(reason)
        with self._publish_lock:
            self.shutting_down = True
            if self.shutdown_reason is None:
                self.shutdown_reason = reason
            self.state, self.reason = LOCKED, reason
            try:
                self.out.publish(Twist())
            except Exception:
                pass

    def _publish_authorized(self, msg, channel='nav'):
        """Linearization point for non-zero velocity emission serialized with request_shutdown() via _publish_lock.
        Because Python signal handlers only call mark_stop_requested() (lock-free, no publish), neither
        main-thread nor worker-thread signal delivery can deadlock or emit zero before an in-flight publish."""
        with self._publish_lock:
            if self.shutting_down or self._stop_requested or self.state != AUTHORIZED or not self.input_ok:
                self.counters['dropped_inputs'] += 1
                return
            scope = self.input_scope
            if self.mode_policy is not None:
                allowed, why = self.mode_policy.permits(channel, self.hb_boot)
                if not allowed:                                       # the channel is not the one the operating mode selects: zero, never the command
                    self.mode_policy.counters['mode_blocked'] += 1
                    self.out.publish(Twist()); self.last_out = (time.monotonic(), 0.0, 0.0)
                    if why and why.startswith('CHANNEL_NOT_SELECTED') and (abs(msg.linear.x) > 1e-6 or abs(msg.angular.z) > 1e-6):
                        self.source_conflict_t = time.monotonic()      # a non-selected source is commanding motion while a mode selects another: conflict
                    return
                if channel == 'manual' and not self._manual_rearmed(msg):
                    self.mode_policy.counters['manual_rearm_blocked'] = self.mode_policy.counters.get('manual_rearm_blocked', 0) + 1
                    self.out.publish(Twist()); self.last_out = (time.monotonic(), 0.0, 0.0)
                    return
                scope = 'manual' if channel == 'manual' else 'auto'
            if self.clamp is not None:
                msg = self.clamp.apply(scope, msg)       # non-zero commands only, inside the single publish point
            self.out.publish(msg)
            self.last_out = (time.monotonic(), msg.linear.x, msg.angular.z)
            self.counters['forwarded'] += 1

    # ---------------------------------------------------------------- inputs
    def on_hb(self, msg):
        try:
            hb = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(hb, dict):
            return
        now = time.monotonic()
        if hb.get('healthy') is not True:
            self.hb_unhealthy = True
            self.hb_t = self.hb_since = None
            return
        self.hb_unhealthy = False
        if self.hb_t is None or now - self.hb_t > self.hb_timeout or hb.get('boot_id') != self.hb_boot:
            self.hb_since = now          # new continuous run (also after an Agent boot change)
        self.hb_t, self.hb_boot = now, hb.get('boot_id')
        self.loop_age = hb.get('tasks_age_s')
        goals = hb.get('goal_uuids')
        self.agent_goals = {g for g in goals if isinstance(g, str)} if isinstance(goals, list) else set()

    def on_speed_limit(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        try:                                   # re-read the graph at the moment a message is judged, not only at the last tick
            lp = self.get_publishers_info_by_topic(self.clamp_topic())
            self.sl_pub_ok = len(lp) == 1 and lp[0].node_name in self.speed_limit_nodes
        except Exception:
            self.sl_pub_ok = False
        try:
            now_d = time.monotonic(); n_d = len(lp); d = self.sl_diag; d['msgs'] += 1; d['n_pubs_hist'][str(n_d)] = d['n_pubs_hist'].get(str(n_d), 0) + 1
            if n_d >= 2 and d['first_n2_t'] is None: d['first_n2_t'] = now_d
            rid = str(((data.get('scopes') or {}).get('auto') or {}).get('request_id'))[:24]
            if rid not in d['first_msg_t'] and len(d['first_msg_t']) < 8: d['first_msg_t'][rid] = now_d
        except Exception:
            pass
        self.clamp.accept(data, self.hb_boot, self.sl_pub_ok)
        if self.mode_policy is not None:
            self.mode_policy.accept(data, self.hb_boot, self.sl_pub_ok)

    def on_gate(self, msg):
        self.gate_t = time.monotonic()

    def on_status(self, msg):
        active = {bytes(g.goal_info.goal_id.uuid).hex() for g in msg.status_list if g.status not in TERMINAL}
        self.active_uuids = active
        self.nav_active = len(active)
        self.nav_status_t = time.monotonic()

    def _manual_rearmed(self, msg):
        """Called under the publish lock for a manual-channel command that is about to be forwarded. A new MANUAL grant starts a 0.5 s window; only a zero (neutral) command re-arms the joystick;
        a deflection held through the window, or any silence, does not (it stays blocked until a neutral command is seen, then a fresh deflection moves)."""
        c = self.mode_policy.cur
        key = None if c is None else (c['name'], c['revision'], c['epoch'], c['boot'])
        now = time.monotonic()
        if self.man_need_release:
            if not self.man_release_seen:
                return False                                                   # still the same deflection that was held through the non-authorized episode
            self.man_need_release = False
            self.man_key, self.man_window_end, self.man_rearmed, self.man_dirty = key, 0.0, True, False     # released once: this (new) input is a fresh deflection
            self.counters['release_rearms'] = self.counters.get('release_rearms', 0) + 1
            return True
        if key != self.man_key:
            self.man_key, self.man_window_end, self.man_rearmed, self.man_dirty = key, (c['granted_t'] if c else now) + 0.5, False, False
        zero = abs(msg.linear.x) < 1e-6 and abs(msg.angular.z) < 1e-6
        self.man_last = now
        if not self.man_rearmed:
            if zero:
                self.man_rearmed = True
            elif now < self.man_window_end:
                self.man_dirty = True
            elif not self.man_dirty:
                self.man_rearmed = True
        return self.man_rearmed

    def on_manual_cmd(self, msg):
        self._on_channel_cmd('manual', msg)

    def on_cmd(self, msg):
        self._on_channel_cmd('nav', msg)

    def _note_manual_release(self, msg):
        now = time.monotonic()
        zero = abs(msg.linear.x) < 1e-6 and abs(msg.angular.z) < 1e-6
        if self.man_rx_t is not None and now - self.man_rx_t > 0.5 and self.man_neutral is False:
            self.man_need_release, self.man_release_seen = True, False         # a deflected stream went silent > 0.5 s: when it comes back it is not a continuation
            self.counters['silence_latches'] = self.counters.get('silence_latches', 0) + 1
        if zero:
            self.man_release_seen, self.man_neutral = True, True
        else:
            self.man_neutral = False
        self.man_rx_t = now

    def _on_channel_cmd(self, channel, msg):
        if channel == 'manual':
            self._note_manual_release(msg)
        if self.shutting_down or self._stop_requested or self.state != AUTHORIZED or not self.input_ok or not self.manual_input_ok:
            self.counters['dropped_inputs'] += 1
            return
        # 2026-11-22: after this process was frozen, queued inputs can be dispatched before the tick that detects the stall
        # (WATCHDOG_SELF_STALL); they predate the freeze and are never forwarded. The next tick trips FAILSAFE as before.
        if self.last_tick is not None and time.monotonic() - self.last_tick > self.self_stall:
            self.counters['dropped_inputs'] += 1
            self._publish_zero()
            return
        if not _finite_twist(msg):
            self.counters['nonfinite_inputs'] += 1
            self._publish_zero()
            return
        if channel == 'nav':
            self._publish_authorized(msg)               # unchanged call shape for the original input (R2 tests hook this signature)
        else:
            self._publish_authorized(msg, channel)

    # ---------------------------------------------------------------- state machine
    def supervisor_fault(self, now):
        if self.hb_unhealthy:
            return 'AGENT_REPORTED_UNHEALTHY'
        if self.hb_t is None:
            return 'NO_HEARTBEAT_YET'
        if now - self.hb_t > self.hb_timeout:
            return 'AGENT_HEARTBEAT_LOST'
        la = self.loop_age
        if (type(la) not in (int, float) or not math.isfinite(la) or not 0 <= la <= self.loop_timeout):
            return 'AGENT_CONTROL_LOOP_STALLED'          # also NaN / Inf / negative / bool / missing
        if self.gate_t is None:
            return 'NO_GATE_STATUS_YET'
        if now - self.gate_t > self.gate_timeout:
            return 'SAFETY_GATE_STATUS_LOST'
        if not self.input_ok or not getattr(self, 'manual_input_ok', True):
            return 'VELOCITY_INPUT_NOT_EXCLUSIVE'
        if getattr(self, 'source_conflict_t', None) is not None and now - self.source_conflict_t < 2.0:
            return 'MOTION_SOURCE_CONFLICT'
        clamp = getattr(self, 'clamp', None)          # getattr: some R2 tests call supervisor_fault on a bare namespace
        if clamp is not None and not (getattr(self, 'sl_pub_ok', False) and clamp.synced(self.hb_boot)):
            return 'SPEED_LIMIT_NOT_SYNCED'                  # the Agent must have told this Watchdog its current limits (or none) lately
        if self.status_pubs != 1:
            return 'NAV2_ACTION_SERVER_NOT_UNIQUE'
        if not getattr(self, 'status_identity_ok', False):
            return 'NAV2_SERVER_IDENTITY_UNKNOWN'        # graph count 1 but the endpoint GID is unreadable
        if self.status_pub_since is None or now - self.status_pub_since < self.status_settle:
            return 'NAV2_STATUS_SETTLING'
        return None

    def goal_fault(self, now):
        """Undeclared non-terminal goal (after a grace for the heartbeat that declares it)."""
        foreign = self.active_uuids - self.agent_goals
        if not foreign:
            self.foreign_since = None
            return None
        if self.foreign_since is None:
            self.foreign_since = now
        return 'FOREIGN_NAV_GOAL' if now - self.foreign_since >= self.foreign_grace else None

    def to(self, state, reason):
        if state == AUTHORIZED and (self.shutting_down or self._stop_requested):
            return
        if state != self.state:
            self.get_logger().warning(f'MOTION_WATCHDOG {self.state} -> {state} ({reason})')
            if state == FAILSAFE:
                self.counters['trips'] += 1
                self.t_fault = time.time()
                self.last_fault = reason
            if state == AUTHORIZED:
                self.counters['rearms'] += 1
            if self.state == AUTHORIZED and self.man_neutral is False:         # leaving AUTHORIZED with a deflected stick: a neutral command is required before it moves anything again
                self.man_need_release, self.man_release_seen = True, False
        self.state, self.reason = state, reason
        if state != AUTHORIZED:
            self._publish_zero()

    def _status_gids(self):
        """One entry per status publisher endpoint (None when its GID is unreadable/empty); None if the query failed."""
        try:
            infos = self.get_publishers_info_by_topic(STATUS_TOPIC)
        except Exception:
            return None
        out = []
        for i in infos:
            try:
                g = bytes(i.endpoint_gid)
            except Exception:
                g = b''
            out.append(g.hex() if g and any(g) else None)
        return tuple(out)

    def _reset_status(self):
        self.nav_active = None
        self.nav_status_t = None
        self.active_uuids = set()
        self.idle_proof_t = None
        self.resubscribe_pending = True

    def _update_graph(self, now):
        try:
            pubs = self.get_publishers_info_by_topic(self.input_topic)
            self.input_ok = len(pubs) <= 1 and all(i.node_name in self.input_nodes for i in pubs)
            self.input_scope = SCOPE_BY_NODE.get(pubs[0].node_name) if len(pubs) == 1 else None
            self.nav_pubs = sorted(i.node_name for i in pubs)
        except Exception:
            self.input_ok = False
            self.input_scope = None
            self.nav_pubs = ['?']
        if self.mode_policy is not None:
            try:
                mp = self.get_publishers_info_by_topic(self.manual_topic)
                self.manual_pubs = sorted(i.node_name for i in mp)
                gids = frozenset(bytes(i.endpoint_gid).hex() for i in mp)
                if self.man_pub_gids is not None and gids != self.man_pub_gids and self.man_neutral is False:
                    self.man_need_release, self.man_release_seen = True, False   # the manual endpoint changed (publisher recreated, even under the same node name): a deflected stick is not trusted across it
                    self.counters['endpoint_latches'] = self.counters.get('endpoint_latches', 0) + 1
                self.man_pub_gids = gids
                self.manual_input_ok = len(mp) <= 1 and all(i.node_name in self.manual_nodes for i in mp)
            except Exception:
                self.manual_input_ok, self.manual_pubs = False, ['?']
        if self.clamp is not None:
            try:
                lp = self.get_publishers_info_by_topic(self.clamp_topic())
                self.sl_pub_ok = len(lp) == 1 and lp[0].node_name in self.speed_limit_nodes
            except Exception:
                self.sl_pub_ok = False
        n = self.count_publishers(STATUS_TOPIC)
        gids = self._status_gids()
        # Identity must be complete: every endpoint counted by the graph has a readable GID. A failed or partial lookup
        # is NOT "fewer publishers"; it voids the observations and blocks authorization (no GID, no continuity proof).
        identity_ok = gids is not None and len(gids) == n and all(gids)
        gid = gids[0] if identity_ok and n == 1 else None
        was_ok = getattr(self, 'status_identity_ok', None)     # None = never observed yet
        self.status_identity_ok = identity_ok
        changed = n == 1 and gid is not None and self.status_gid is not None and gid != self.status_gid
        if n >= 1 and not identity_ok:
            if self.state == AUTHORIZED:
                self.to(FAILSAFE, 'NAV2_SERVER_IDENTITY_UNKNOWN')
            self._reset_status()
            self.status_pub_since = None
            self.status_gid = None
            self.status_pubs = n
            return
        if was_ok is False and identity_ok:
            self._reset_status()                         # identity recovered after a bad period: new observation window
        if n != 1 or self.status_pubs > 1 or changed:
            # A publisher-set change makes retained observations ambiguous.
            self._reset_status()
            if changed and self.state == AUTHORIZED:
                self.to(FAILSAFE, 'NAV2_ACTION_SERVER_CHANGED')
        if n == 1:
            if self.resubscribe_pending:
                # A retained status may have been delivered (and discarded above) before the graph showed the new
                # publisher; a fresh subscription receives the transient-local status again.
                try:
                    self.destroy_subscription(self.status_sub)
                except Exception:
                    pass
                self.status_sub = self.create_subscription(GoalStatusArray, STATUS_TOPIC, self.on_status, self.status_qos)
                self.resubscribe_pending = False
                self.counters_resub += 1
                self.status_pub_since = now
            if self.status_pubs != 1 or self.status_pub_since is None or changed:
                self.status_pub_since = now
            if gid is not None:
                self.status_gid = gid
        else:
            self.status_pub_since = None
        self.status_pubs = n

    def tick(self):
        if self.shutting_down or self._stop_requested:
            try:
                self._publish_zero()
            except Exception:
                pass
            return
        now = time.monotonic()
        stalled = self.last_tick is not None and now - self.last_tick > self.self_stall
        if self.last_tick is not None:
            gap = now - self.last_tick; d = self.sl_diag; d['ticks'] += 1; d['tick_gap_last'] = gap; d['tick_gap_max'] = max(d['tick_gap_max'], gap)
            if gap > 0.15: d['tick_gaps_gt_150ms'] += 1
        self.last_tick = now
        self._update_graph(now)
        if self.idle_zero_period > 0 and self.state == AUTHORIZED and not stalled:
            with self._publish_lock:
                if self.state == AUTHORIZED and not self.shutting_down and not self._stop_requested and (self.last_out[0] is None or time.monotonic() - self.last_out[0] >= self.idle_zero_period):
                    self.out.publish(Twist()); self.last_out = (time.monotonic(), 0.0, 0.0); self.counters['idle_zeros'] = self.counters.get('idle_zeros', 0) + 1
        fault = 'WATCHDOG_SELF_STALL' if stalled else self.supervisor_fault(now)
        if self.state == AUTHORIZED:
            fault = fault or self.goal_fault(now)
            if fault:
                self.to(FAILSAFE, fault)
                self.idle_proof_t = None
        if self.state != AUTHORIZED:
            self._publish_zero()
            if self.probe_inflight is not None and now - self.probe_inflight > self.probe_timeout:
                self.probe_inflight = None              # lost answer (server died mid-call): no proof, try again
                self.counters['cancel_timeouts'] += 1
            if now - self.last_probe >= self.probe_period:
                self.probe_cancel_all()
        if self.state == FAILSAFE and self.idle_proven(now):
            self.to(LOCKED, 'GOALS_CLEARED_AWAITING_REARM')
        if self.state == LOCKED:
            if fault:
                self.reason = fault if self.reason != 'GOALS_CLEARED_AWAITING_REARM' else self.reason
            elif now - self.hb_since >= self.rearm_hold and self.idle_proven(now):
                self.foreign_since = None
                self.to(AUTHORIZED, 'HEARTBEAT_AND_IDLE_PROVEN')
            else:
                self.reason = 'AWAITING_IDLE_PROOF' if not self.idle_proven(now) else 'AWAITING_HEARTBEAT_HOLD'

    def idle_proven(self, now):
        return (self.idle_proof_t is not None and now - self.idle_proof_t <= self.proof_max_age
                and getattr(self, 'status_identity_ok', False) and self.status_gid is not None
                and self.status_pubs == 1 and self.status_pub_since is not None
                and self.idle_proof_t - self.status_pub_since >= self.status_settle
                and (self.nav_active == 0 or (self.nav_active is None and self.nav_status_t is None)))

    def probe_cancel_all(self):
        """Cancel every NavigateToPose goal. The answer is one input of the idle proof (see module docstring)."""
        self.last_probe = time.monotonic()
        if self.probe_inflight is not None or not self.cancel.service_is_ready():
            return
        req = CancelGoal.Request()
        req.goal_info = GoalInfo()          # zero UUID + zero stamp = all goals
        self.counters['cancel_requests'] += 1
        sent = time.monotonic()
        self.probe_inflight = sent
        fut = self.cancel.call_async(req)

        def done(f):
            if self.probe_inflight != sent:
                return                       # answer to a timed-out probe: ignore
            self.probe_inflight = None
            try:
                res = f.result()
            except Exception:
                return
            if res is None:
                return
            self.last_cancel_rc = int(res.return_code)
            if res.return_code != CancelGoal.Response.ERROR_NONE:
                self.counters['cancel_rc_nonzero'] += 1
                self.idle_proof_t = None
            elif len(res.goals_canceling) == 0:
                self.counters['cancel_empty'] += 1
                self.idle_proof_t = sent       # proof is as old as the request, not the reply
            else:
                self.counters['cancel_nonempty'] += 1
                self.idle_proof_t = None
        fut.add_done_callback(done)

    def own_output_gid(self):
        """GID of this node's /cmd_vel_raw writer from the DDS graph: exactly one endpoint under this node's name, else None
        (a second process using the same node name makes it ambiguous). Graph evidence, not authentication."""
        try:
            mine = [x for x in self.get_publishers_info_by_topic(self.out.topic_name)
                    if x.node_name == self.get_name() and x.node_namespace == self.get_namespace()]
            if len(mine) != 1: return None
            return gid_hex(mine[0], require_nonzero=True)      # Phase 3.5: shared helper, same semantics
        except Exception:
            return None

    def clamp_topic(self):
        return topics.SPEED_LIMIT

    def publish_status(self):
        now = time.monotonic()
        self.status_pub.publish(String(data=json.dumps({
            'output_topic': self.out.topic_name, 'output_gid': self.own_output_gid(),
            'state': self.state, 'reason': self.reason,
            'ready_to_submit': self.state == AUTHORIZED and not self.shutting_down and not self._stop_requested,
            'shutting_down': self.shutting_down,
            'boot_id': self.boot_id, 'trips': self.counters['trips'],
            'heartbeat_age_s': None if self.hb_t is None else round(now - self.hb_t, 3),
            'gate_status_age_s': None if self.gate_t is None else round(now - self.gate_t, 3),
            'control_loop_age_s': self.loop_age, 'agent_boot_id': self.hb_boot, 'nav_active': self.nav_active,
            'nav_status_publishers': self.status_pubs, 'nav_status_gid': self.status_gid if self.status_pubs == 1 else None,
            'nav_status_received': self.nav_status_t is not None, 'status_resubscriptions': self.counters_resub,
            'agent_goal_uuids': sorted(self.agent_goals),
            'foreign_goal_uuids': sorted(self.active_uuids - self.agent_goals),
            'idle_proof_age_s': None if self.idle_proof_t is None else round(now - self.idle_proof_t, 3),
            'last_cancel_return_code': self.last_cancel_rc,
            'speed_clamp': {'enabled': False} if self.clamp is None else dict(self.clamp.status(self.hb_boot), publisher_ok=self.sl_pub_ok, input_scope=self.input_scope),
            'mode_policy': {'enabled': False} if self.mode_policy is None else dict(self.mode_policy.status(self.hb_boot), nav_publishers=self.nav_pubs, nav_input_ok=self.input_ok,
                                                                                    manual_publishers=self.manual_pubs, manual_input_ok=self.manual_input_ok, manual_rearmed=self.man_rearmed, manual_need_release=self.man_need_release,
                                                                                    source_conflict_age_s=None if self.source_conflict_t is None else round(now - self.source_conflict_t, 3)),
            'sl_diag': dict(self.sl_diag, first_n2_age_s=None if self.sl_diag['first_n2_t'] is None else round(now - self.sl_diag['first_n2_t'], 3),
                            first_msg_age_s={k: round(now - v, 3) for k, v in self.sl_diag['first_msg_t'].items()}),
            'last_output': {'linear_x': self.last_out[1], 'angular_z': self.last_out[2], 'age_s': None if self.last_out[0] is None else round(now - self.last_out[0], 3)},
            'fault_wall_time': self.t_fault, 'last_fault': self.last_fault, **self.counters, 't': time.time()})))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = MotionWatchdog()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_state = {'signal': None, 'name': None, 'count': 0}
    stop_logged = {'first': False, 'dup': False}

    def _on_stop_signal(signum, _frame):
        # Strictly lock-free, no-publish, no-I/O signal handler: records stop request only.
        # Actual shutdown finalization (request_shutdown under _publish_lock) and logging execute
        # in main()'s normal execution context so neither main-thread nor worker-thread signal
        # delivery can deadlock or emit zero Twist before an in-flight publish finishes.
        stop_state['count'] += 1
        try:
            name = signal.Signals(signum).name
        except ValueError:
            name = str(signum)
        if stop_state['signal'] is None:
            stop_state['signal'] = signum
            stop_state['name'] = name
        node.mark_stop_requested(f'SIGNAL_{name}')

    def _finalize_signal_stop():
        if stop_state['signal'] is not None and not stop_logged['first']:
            sig_num = stop_state['signal']
            sig_name = stop_state['name'] or str(sig_num)
            node.request_shutdown(f'SIGNAL_{sig_name}')
            print(f'motion_watchdog: signal {sig_num} ({sig_name}) received: stopping cleanly', flush=True)
            stop_logged['first'] = True

    signal.signal(signal.SIGINT, _on_stop_signal)
    signal.signal(signal.SIGTERM, _on_stop_signal)
    exit_code = 0
    unhandled_exc = None
    try:
        while stop_state['signal'] is None:
            if not rclpy.ok():
                print('motion_watchdog: ABNORMAL_EXTERNAL_SHUTDOWN: ROS context shut down without stop signal', file=sys.stderr, flush=True)
                exit_code = 1
                break
            executor.spin_once(timeout_sec=0.05)
        _finalize_signal_stop()
    except KeyboardInterrupt:
        if stop_state['signal'] is None:
            stop_state['signal'] = signal.SIGINT
            stop_state['name'] = 'SIGINT'
        node.request_shutdown('SIGNAL_SIGINT')
        print('motion_watchdog: KeyboardInterrupt received: stopping cleanly', flush=True)
        stop_logged['first'] = True
    except ExternalShutdownException as exc:
        if stop_state['signal'] is not None:
            _finalize_signal_stop()
            print(f'motion_watchdog: ROS context shut down during signal {stop_state["signal"]} stop', flush=True)
        else:
            print(f'motion_watchdog: ABNORMAL_EXTERNAL_SHUTDOWN: {exc}', file=sys.stderr, flush=True)
            exit_code = 1
    except Exception as exc:
        node.request_shutdown('ABNORMAL_EXCEPTION')
        print(f'motion_watchdog: ABNORMAL_EXCEPTION: {type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        unhandled_exc = exc
        exit_code = 1
    finally:
        def _step(what, fn):
            try:
                fn()
            except Exception as exc:
                print(f'motion_watchdog: shutdown step {what} failed ({type(exc).__name__}: {exc})', file=sys.stderr, flush=True)

        _finalize_signal_stop()
        if not node.shutting_down:
            node.request_shutdown('SHUTDOWN_FINALLY')
        if rclpy.ok():
            _step('zero_twist_publish', node._publish_zero)
            _step('final_status_publish', node.publish_status)
        _step('executor.shutdown', executor.shutdown)
        _step('node.destroy_node', node.destroy_node)
        _step('rclpy.try_shutdown', rclpy.try_shutdown)
        if stop_state['count'] > 1 and not stop_logged['dup']:
            sig_num = stop_state['signal'] or signal.SIGTERM
            sig_name = stop_state['name'] or str(sig_num)
            print(f'motion_watchdog: duplicate signal {sig_num} ({sig_name}) ignored during shutdown (count={stop_state["count"]})', flush=True)
            stop_logged['dup'] = True
        if exit_code == 0:
            print(f'motion_watchdog: stopped cleanly (exit_code=0, signals={stop_state["count"]})', flush=True)
        else:
            print(f'motion_watchdog: stopped abnormally (exit_code={exit_code})', file=sys.stderr, flush=True)
        # Prevent CPython Py_FinalizeEx() -> PyOS_FiniInterrupts() from resetting custom Python handlers to SIG_DFL
        # during interpreter teardown (PyOS_FiniInterrupts preserves SIG_IGN at the OS level).
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
        except Exception:
            pass
    if unhandled_exc is not None:
        raise unhandled_exc
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
