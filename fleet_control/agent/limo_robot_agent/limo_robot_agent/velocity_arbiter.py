"""Localhost mock-only velocity selector feeding the existing motion watchdog (single velocity source).

Never instantiated by the real agent. The gate must publish /cmd_vel_locked and there must be no /cmd_vel endpoints.
This is not a real-world goal fence.

Selection follows the Fleet server's operating mode (delivered with every state response):
  MANUAL (or a legacy robot without a mode)  live web bench sample (Deadman, 250 ms TTL), bounded 0..0.10 m/s, +-0.30 rad/s
  MAPPING                                    Nav2 velocity_smoother output, only while the Agent owns a Frontier goal
  NAVIGATION                                 Nav2 velocity_smoother output, only while the Agent owns a Fleet mission goal
  IDLE / TRANSITIONING / FAULT / unknown     zero
Every source additionally needs the common conditions (Fleet session, Lease, Watchdog authorized, Gate NORMAL and
fresh, odom fresh, locked velocity graph). Any failure publishes zero; the first failing condition is reported.
"""
import math
import time
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType

from . import topics

NAV_INPUT = topics.NAV_SMOOTHED
NAV_INPUT_MAX_AGE_S = .25
NAV_LINEAR = (-.10, .15)        # velocity_smoother caps of integrated_nav2_lidar_only_slow_params.yaml
NAV_TURN = .5
GOAL_OWNER = {'MAPPING': 'frontier', 'NAVIGATION': 'fleet'}
# 2026-11-22 control generation: Nav2 samples must be received at least this long after the current generation started, so the
# velocity_smoother's ramp-down tail of a cancelled/previous goal is never forwarded under a newly admitted goal.
NAV_GEN_SETTLE_S = .3
# 2026-11-22 (2): the smoother output is a plain Twist (no stamp, no goal id) and the Humble velocity_smoother re-publishes its last
# command for velocity_timeout (1.0 s, measured 0.91-0.95 s: tests/nav2_smoother_tail_probe_20261122.py) when its input stops
# without a zero. A smoothed sample is therefore forwarded only while the smoother's own input (the controller / behavior server
# command) is live: received within this age and after the current generation started. Content is never forwarded from NAV_RAW.
NAV_RAW = topics.NAV_RAW
NAV_RAW_MAX_AGE_S = .25


def generation_key(*, mode, session, active, common_ok):
    """Everything whose change invalidates commands received before it (see VelocityArbiter.update)."""
    lease = (session or {}).get('lease') or {}
    a = active if isinstance(active, dict) else {}
    return (mode, (session or {}).get('server_boot_id'), lease.get('epoch'), a.get('goal_uuid') if a else None,
            bool(a.get('cancel_requested')) if a else False, common_ok)


def select_locked(*, allowed, manual, now):
    """Select a live bounded bench sample; all other inputs yield zero."""
    if allowed is not True or not isinstance(manual, dict) or manual.get('deadman') is not True:
        return 0., 0.
    expiry, v, w = (manual.get(k) for k in ('expires', 'requested_linear', 'requested_turn'))
    if any(type(x) not in (float, int) or not math.isfinite(x) for x in (now, expiry, v, w)):
        return 0., 0.
    if not now < expiry or not 0 <= v <= .1 or not -.3 <= w <= .3:
        return 0., 0.
    return v, w


def select_nav(*, allowed, sample, now):
    """Pass a fresh finite Nav2 command, clamped to the slow-profile caps; otherwise zero."""
    if allowed is not True or not sample:
        return 0., 0.
    t, v, w = sample
    if not all(type(x) in (float, int) and math.isfinite(x) for x in (t, v, w)) or not 0 <= now - t <= NAV_INPUT_MAX_AGE_S:
        return 0., 0.
    return min(max(v, NAV_LINEAR[0]), NAV_LINEAR[1]), min(max(w, -NAV_TURN), NAV_TURN)


class VelocityArbiter:
    OUTPUT = topics.NAV_CMD_VEL

    def __init__(self, robot, agent, bench):
        if robot.mode != 'mock' or robot.simulated is not True:
            raise ValueError('locked arbiter is restricted to simulated mock agents')
        self.robot, self.agent, self.bench = robot, agent, bench
        self.output = (0., 0.)
        self.ready = False
        self.reason = 'STARTUP'
        self.source = 'NONE'
        self.operating_mode = None
        self.nav_sample = None
        self.nav_raw_t = None              # receipt time of the latest controller/behavior command (smoother input)
        self.gen_key, self.gen_t, self.gen_n = None, time.monotonic(), 0
        self.manual_rearmed = False        # deadman seen released (or absent) since the current generation started
        self.pub = robot.create_publisher(Twist, self.OUTPUT, 10)
        self.sub = robot.create_subscription(Twist, NAV_INPUT, self.on_nav, 10)
        self.raw_sub = robot.create_subscription(Twist, NAV_RAW, self.on_nav_raw, 10)
        self.timer = robot.create_timer(.05, self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_nav(self, msg):
        self.nav_sample = (time.monotonic(), msg.linear.x, msg.angular.z)

    def on_nav_raw(self, msg):
        self.nav_raw_t = time.monotonic()

    def graph_locked(self):
        r = self.robot
        def endpoints(query, topic):
            return [(v.node_namespace, v.node_name) for v in query(topic)]
        pubs, subs = r.get_publishers_info_by_topic, r.get_subscriptions_info_by_topic
        nav_pubs = endpoints(pubs, NAV_INPUT)
        return (not endpoints(pubs, topics.CMD_VEL) and not endpoints(subs, topics.CMD_VEL)
                and endpoints(pubs, topics.CMD_VEL_LOCKED) == [('/', topics.GATE_NODE)]
                and endpoints(pubs, topics.GATE_STATUS) == [('/', topics.GATE_NODE)]
                and endpoints(subs, topics.CMD_VEL_RAW) == [('/', topics.GATE_NODE)]
                and endpoints(pubs, topics.CMD_VEL_RAW) == [('/', topics.WATCHDOG_NODE)]
                and endpoints(subs, self.OUTPUT) == [('/', topics.WATCHDOG_NODE)]
                and endpoints(pubs, self.OUTPUT) == [('/', r.get_name())]
                and nav_pubs in ([], [('/', topics.SMOOTHER_NODE)]))

    def mode(self):
        om = self.agent.session_snapshot().get('operating_mode')
        if om is None:
            return None                            # legacy robot: manual bench only (Phase 1 behaviour)
        return om.get('mode') if isinstance(om, dict) and isinstance(om.get('mode'), str) else 'UNKNOWN'

    def common_blocker(self):
        r = self.robot
        session = self.agent.session_snapshot()
        lease = session.get('lease') or {}
        if not self.agent.is_execution_allowed():
            return 'AGENT_EXECUTION_NOT_ALLOWED'
        if session.get('connected') is not True:
            return 'FLEET_DISCONNECTED'
        if session.get('reconciliation_required') is not False:
            return 'RECONCILIATION_REQUIRED'
        if not (type(lease.get('epoch')) is int and type(lease.get('expires_at')) in (int, float)
                and math.isfinite(lease['expires_at']) and lease['expires_at'] > time.time()):
            return 'LEASE_MISSING_OR_EXPIRED'
        if not r.watchdog_ok():
            return 'WATCHDOG_NOT_AUTHORIZED'
        with r.lock:
            gate = dict(r.gate) if isinstance(r.gate, dict) else {}
            age, odom_age = r._age('gate'), r._age('odom')
        if age is None or not 0 <= age < .3:
            return 'GATE_STATUS_STALE'
        # STALL_PENDING is the Gate's own confirmation window (commanded motion, wheels not yet moving) and does not
        # block output; a confirmed stall or any fault sets cmd_blocked / another state and blocks here.
        if gate.get('state') not in ('NORMAL', 'STALL_PENDING') or gate.get('cmd_blocked') is not False:
            return 'GATE_NOT_NORMAL'
        if gate.get('wheel_odom_stale') is not False or odom_age is None or not 0 <= odom_age < .38:
            return 'ODOM_STALE'
        if not self.graph_locked():
            return 'VELOCITY_GRAPH_NOT_LOCKED'
        return None

    def manual_blocker(self):
        r = self.robot
        if r.active is not None:
            return 'FLEET_MISSION_ACTIVE'
        # Mapping goals are always Agent-owned (r.active); a running mapping session has an operating mode MAPPING.
        idle = r.manual_idle_evidence()
        if idle.get('ok') is not True:
            return idle.get('reason') or 'NAV2_IDLE_UNPROVEN'
        return None

    def nav_blocker(self, mode):
        r = self.robot
        active = r.active
        if not isinstance(active, dict):
            return 'NO_OWNED_GOAL'
        if active.get('owner', 'fleet') != GOAL_OWNER[mode]:
            return 'GOAL_OWNER_MISMATCH'
        # 2026-11-22 (Gazebo f07): a requested cancel stops forwarding Nav2 output at once. Before, the Arbiter kept forwarding the
        # controller's commands until Nav2 confirmed the cancel or cancel_result_timeout_s (5 s) expired, so an unresponsive
        # bt_navigator let the robot keep driving >= 4 s / 0.5 m after the operator's cancel. Terminal handling is unchanged.
        if active.get('cancel_requested'):
            return 'CANCEL_REQUESTED'
        bench = self.bench.snapshot() if self.bench else {}
        if bench.get('deadman') is True:
            return 'MANUAL_INPUT_PRESENT'
        sample = self.nav_sample
        if not sample or not 0 <= time.monotonic() - sample[0] <= NAV_INPUT_MAX_AGE_S:
            return 'NAV_INPUT_STALE'
        if sample[0] < self.gen_t + NAV_GEN_SETTLE_S:
            return 'NAV_INPUT_PREDATES_GENERATION'
        raw = self.nav_raw_t
        if raw is None or raw < self.gen_t or not 0 <= time.monotonic() - raw <= NAV_RAW_MAX_AGE_S:
            return 'NAV_PRODUCER_NOT_LIVE'       # smoother output without a live controller command = replay of an older command
        return None

    def manual_generation_blocker(self, bench):
        # A deadman already held when the generation started (mode switch, Gate resume, Watchdog re-authorization, new
        # lease) must be released once before it can drive: a pre-switch hold is never reused.
        if bench.get('deadman') is not True:
            self.manual_rearmed = True
        return None if self.manual_rearmed else 'MANUAL_DEADMAN_NOT_REARMED'

    def update_generation(self, mode, common_ok):
        key = generation_key(mode=mode, session=self.agent.session_snapshot(), active=self.robot.active, common_ok=common_ok)
        if key != self.gen_key:
            self.gen_key, self.gen_t, self.gen_n = key, time.monotonic(), self.gen_n + 1
            self.nav_sample = self.nav_raw_t = None
            self.manual_rearmed = False

    def update(self):
        mode = self.mode()
        self.operating_mode = mode
        try:
            reason = self.common_blocker()
            self.update_generation(mode, reason is None)
            if mode in (None, 'MANUAL'):
                source = 'MANUAL'
                reason = reason or self.manual_blocker() or self.manual_generation_blocker(self.bench.snapshot() if self.bench else {})
            elif mode in GOAL_OWNER:
                source = 'NAV2'
                reason = reason or self.nav_blocker(mode)
            else:
                source = 'NONE'
                reason = 'MODE_' + str(mode)
        except Exception as exc:
            source, reason = 'NONE', f'ARBITER_ERROR:{type(exc).__name__}'
        self.reason, self.ready = reason, reason is None
        now = time.monotonic()
        if source == 'MANUAL':
            v, w = select_locked(allowed=self.ready, manual=self.bench.snapshot(), now=now)
        elif source == 'NAV2':
            v, w = select_nav(allowed=self.ready, sample=self.nav_sample, now=now)
        else:
            v, w = 0., 0.
        self.source = source
        msg = Twist(); msg.linear.x = v; msg.angular.z = w
        self.output = v, w
        self.pub.publish(msg)

    # Kept for callers/tests that use the previous name.
    def blocker(self):
        return self.common_blocker() or self.manual_blocker()

    def allowed(self):
        try:
            self.reason = self.blocker()
        except Exception as exc:
            self.reason = f'ARBITER_ERROR:{type(exc).__name__}'
        return self.reason is None

    def snapshot(self):
        manual = self.bench.snapshot() if self.bench else {}
        active = 'NONE' if self.output == (0., 0.) else self.source
        if active == 'NONE' and self.source == 'MANUAL' and manual.get('deadman'):
            active = 'MANUAL_ZERO'
        nav_age = None if not self.nav_sample else round(time.monotonic() - self.nav_sample[0], 3)
        return {'mode': 'MOCK_LOCKED', 'operating_mode': self.operating_mode, 'ready': self.ready, 'blocker': self.reason,
                'source': active, 'selected_source': self.source, 'deadman': manual.get('deadman') is True,
                'nav_input_age_s': nav_age, 'generation': self.gen_n,
                'nav_producer_age_s': None if self.nav_raw_t is None else round(time.monotonic() - self.nav_raw_t, 3), 'selected_linear': self.output[0], 'selected_turn': self.output[1],
                'motor_output': False}

    def close(self):
        self.timer.cancel()
        try:      # after SIGINT the rclpy context may already be invalid; the Watchdog/Gate zero on lost input anyway
            self.pub.publish(Twist())
            self.robot.destroy_publisher(self.pub)
        except Exception:
            pass
