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
import time
import uuid

import rclpy
from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from . import topics
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

    def on_gate(self, msg):
        self.gate_t = time.monotonic()

    def on_status(self, msg):
        active = {bytes(g.goal_info.goal_id.uuid).hex() for g in msg.status_list if g.status not in TERMINAL}
        self.active_uuids = active
        self.nav_active = len(active)
        self.nav_status_t = time.monotonic()

    def on_cmd(self, msg):
        if self.state != AUTHORIZED or not self.input_ok:
            self.counters['dropped_inputs'] += 1
            return
        # 2026-11-22: after this process was frozen, queued inputs can be dispatched before the tick that detects the stall
        # (WATCHDOG_SELF_STALL); they predate the freeze and are never forwarded. The next tick trips FAILSAFE as before.
        if self.last_tick is not None and time.monotonic() - self.last_tick > self.self_stall:
            self.counters['dropped_inputs'] += 1
            self.out.publish(Twist())
            return
        if not _finite_twist(msg):
            self.counters['nonfinite_inputs'] += 1
            self.out.publish(Twist())
            return
        self.out.publish(msg)
        self.counters['forwarded'] += 1

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
        if not self.input_ok:
            return 'VELOCITY_INPUT_NOT_EXCLUSIVE'
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
        if state != self.state:
            self.get_logger().warning(f'MOTION_WATCHDOG {self.state} -> {state} ({reason})')
            if state == FAILSAFE:
                self.counters['trips'] += 1
                self.t_fault = time.time()
                self.last_fault = reason
            if state == AUTHORIZED:
                self.counters['rearms'] += 1
        self.state, self.reason = state, reason
        if state != AUTHORIZED:
            self.out.publish(Twist())

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
        except Exception:
            self.input_ok = False
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
        now = time.monotonic()
        stalled = self.last_tick is not None and now - self.last_tick > self.self_stall
        self.last_tick = now
        self._update_graph(now)
        fault = 'WATCHDOG_SELF_STALL' if stalled else self.supervisor_fault(now)
        if self.state == AUTHORIZED:
            fault = fault or self.goal_fault(now)
            if fault:
                self.to(FAILSAFE, fault)
                self.idle_proof_t = None
        if self.state != AUTHORIZED:
            self.out.publish(Twist())
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

    def publish_status(self):
        now = time.monotonic()
        self.status_pub.publish(String(data=json.dumps({
            'output_topic': self.out.topic_name, 'output_gid': self.own_output_gid(),
            'state': self.state, 'reason': self.reason, 'ready_to_submit': self.state == AUTHORIZED,
            'boot_id': self.boot_id,
            'heartbeat_age_s': None if self.hb_t is None else round(now - self.hb_t, 3),
            'gate_status_age_s': None if self.gate_t is None else round(now - self.gate_t, 3),
            'control_loop_age_s': self.loop_age, 'agent_boot_id': self.hb_boot, 'nav_active': self.nav_active,
            'nav_status_publishers': self.status_pubs, 'nav_status_gid': self.status_gid if self.status_pubs == 1 else None,
            'nav_status_received': self.nav_status_t is not None, 'status_resubscriptions': self.counters_resub,
            'agent_goal_uuids': sorted(self.agent_goals),
            'foreign_goal_uuids': sorted(self.active_uuids - self.agent_goals),
            'idle_proof_age_s': None if self.idle_proof_t is None else round(now - self.idle_proof_t, 3),
            'last_cancel_return_code': self.last_cancel_rc,
            'fault_wall_time': self.t_fault, 'last_fault': self.last_fault, **self.counters, 't': time.time()})))


def main(args=None):
    rclpy.init(args=args)
    node = MotionWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.out.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
