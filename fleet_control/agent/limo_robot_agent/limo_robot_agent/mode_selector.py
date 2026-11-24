"""Mode selector for an Agent WITHOUT an Arbiter (the real-robot shape), G2 2026-11-23.

The server confirms an operating mode only on evidence that a velocity selector for that mode exists and is ready (`state.velocity_arbiter`). Mock Agents
have the Arbiter. A real Agent has none: Nav2's smoother (and the on-board joystick) feed the Motion Watchdog directly, so no mode could ever be confirmed.

This class is that evidence, produced from facts instead of asserted:
  * the selection conditions are the Arbiter's own (it subclasses VelocityArbiter and reuses common_blocker / manual_blocker / mode / update_generation:
    Fleet session, Lease, Watchdog AUTHORIZED, Gate NORMAL and fresh, odom fresh, no Agent goal for MANUAL, Nav2 idle proof, owned goal for MAPPING/NAVIGATION);
  * the velocity graph must be the real locked/unlocked shape (no endpoint on /cmd_vel, only the Watchdog writes /cmd_vel_raw and only the Gate reads it,
    only the Watchdog reads the two input channels, at most one allowed publisher per channel, nothing but the Gate on the driver input);
  * the Watchdog itself must confirm the SAME mode and revision it was told (its status `mode_policy`), with its speed-clamp sync intact, both input channels
    exclusive and no source conflict. The Watchdog is the process that actually forwards or blocks motion, so its confirmation is the evidence that the mode
    is in force on the velocity path; an Agent-only claim would not be accepted by this class;
  * it publishes nothing (motor_output is always False); selected_linear/turn are what the Watchdog last forwarded (zero when it is idle), so the server's
    "robot is idle" proof for a mode change looks at the real output.
Not authentication: node names and GIDs come from the DDS graph and the Watchdog's status is an unauthenticated JSON topic.
"""
import time

from rclpy.clock import Clock, ClockType

from . import topics
from .mode_policy import ALLOWED_INPUT_NODES, MODE_CHANNEL
from .velocity_arbiter import GOAL_OWNER, VelocityArbiter

KIND = 'WATCHDOG_SELECTOR'
STATUS_MAX_AGE_S = 1.0
OUTPUT_FRESH_S = 0.5


class RealModeSelector(VelocityArbiter):
    OUTPUT = None

    def __init__(self, robot, agent, period=0.1):          # deliberately NOT calling VelocityArbiter.__init__ (it creates the velocity publisher)
        self.robot, self.agent, self.bench = robot, agent, None
        self.output = (0., 0.)
        self.ready, self.reason, self.source, self.operating_mode = False, 'STARTUP', 'NONE', None
        self.nav_sample = self.nav_raw_t = None
        self.gen_key, self.gen_t, self.gen_n = None, time.monotonic(), 0
        self.manual_rearmed = False
        self.evidence = {}
        self.timer = robot.create_timer(period, self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    # ------------------------------------------------------------------ evidence
    def wd(self):
        w = getattr(self.robot, 'motion_watchdog', None)
        if not w or time.monotonic() - w[0] > STATUS_MAX_AGE_S or not isinstance(w[1], dict):
            return None
        return w[1]

    def _eps(self, query, topic):
        return sorted((v.node_namespace, v.node_name) for v in query(topic))

    def graph_locked(self):
        r = self.robot
        pubs, subs = r.get_publishers_info_by_topic, r.get_subscriptions_info_by_topic
        gate, wd = ('/', topics.GATE_NODE), ('/', topics.WATCHDOG_NODE)
        nav_pubs = [n for _, n in self._eps(pubs, topics.NAV_CMD_VEL)]
        man_pubs = [n for _, n in self._eps(pubs, topics.MANUAL_CMD_VEL)]
        ok = (not self._eps(pubs, topics.CMD_VEL) and not self._eps(subs, topics.CMD_VEL)
              and set(self._eps(pubs, topics.CMD_VEL_DRIVER)) <= {gate} and set(self._eps(pubs, topics.CMD_VEL_LOCKED)) <= {gate}
              and self._eps(pubs, topics.GATE_STATUS) == [gate]
              and self._eps(subs, topics.CMD_VEL_RAW) == [gate] and self._eps(pubs, topics.CMD_VEL_RAW) == [wd]
              and self._eps(subs, topics.NAV_CMD_VEL) == [wd] and self._eps(subs, topics.MANUAL_CMD_VEL) == [wd]
              and len(nav_pubs) <= 1 and set(nav_pubs) <= set(ALLOWED_INPUT_NODES['nav'])
              and len(man_pubs) <= 1 and set(man_pubs) <= set(ALLOWED_INPUT_NODES['manual']))
        self.evidence['agent_graph'] = {'nav_publishers': nav_pubs, 'manual_publishers': man_pubs}
        return ok

    def watchdog_mode_blocker(self, mode):
        w = self.wd()
        if w is None:
            return 'WATCHDOG_STATUS_STALE'
        mp, sc = w.get('mode_policy'), w.get('speed_clamp')
        if not isinstance(mp, dict) or mp.get('enabled') is not True:
            return 'WATCHDOG_MODE_GATE_DISABLED'
        if not (isinstance(sc, dict) and sc.get('enabled') is True and sc.get('synced') is True and sc.get('publisher_ok') is True and sc.get('fault') is None):
            return 'WATCHDOG_AGENT_SYNC_LOST'
        om = self.agent.session_snapshot().get('operating_mode') or {}
        if mp.get('mode') != mode or mp.get('revision') != om.get('revision'):
            return 'WATCHDOG_MODE_NOT_CONFIRMED'
        if mp.get('selected_channel') != MODE_CHANNEL.get(mode) or not (mp.get('age_s') is not None and mp['age_s'] <= 1.5):
            return 'WATCHDOG_MODE_NOT_CONFIRMED'
        if mp.get('nav_input_ok') is not True or mp.get('manual_input_ok') is not True:
            return 'WATCHDOG_INPUT_NOT_EXCLUSIVE'
        age = mp.get('source_conflict_age_s')
        if isinstance(age, (int, float)) and age < 3.0:
            return 'MOTION_SOURCE_CONFLICT'
        return None

    def owned_goal_blocker(self, mode):
        active = self.robot.active
        if not isinstance(active, dict):
            return 'NO_OWNED_GOAL'
        if active.get('owner', 'fleet') != GOAL_OWNER[mode]:
            return 'GOAL_OWNER_MISMATCH'
        if active.get('cancel_requested'):
            return 'CANCEL_REQUESTED'
        return None

    # ------------------------------------------------------------------ selection (the Arbiter's order, without a velocity output)
    def update(self):
        mode = self.mode()
        self.operating_mode = mode
        try:
            reason = self.common_blocker()
            self.update_generation(mode, reason is None)
            if mode is None:
                source, reason = 'NONE', 'NO_OPERATING_MODE'              # a legacy robot (no mode) is never confirmed here
            elif mode == 'MANUAL':
                source = 'MANUAL'
                reason = reason or self.watchdog_mode_blocker(mode) or self.manual_blocker() or self.manual_generation_blocker({})
            elif mode in GOAL_OWNER:
                source = 'NAV2'
                reason = reason or self.watchdog_mode_blocker(mode) or self.owned_goal_blocker(mode)
            else:
                source, reason = 'NONE', 'MODE_' + str(mode)
        except Exception as exc:
            source, reason = 'NONE', f'SELECTOR_ERROR:{type(exc).__name__}'
        self.reason, self.ready, self.source = reason, reason is None, source
        w = self.wd()
        lo = (w or {}).get('last_output') or {}
        fresh = isinstance(lo.get('age_s'), (int, float)) and lo['age_s'] <= OUTPUT_FRESH_S
        self.output = (lo.get('linear_x', 0.) if fresh else 0., lo.get('angular_z', 0.) if fresh else 0.)
        mp = (w or {}).get('mode_policy') or {}
        self.evidence.update({'watchdog_boot_id': (w or {}).get('boot_id'), 'watchdog_trips': (w or {}).get('trips'), 'watchdog_state': (w or {}).get('state'), 'watchdog_mode': mp.get('mode'),
                              'watchdog_revision': mp.get('revision'), 'watchdog_epoch': mp.get('epoch'), 'selected_channel': mp.get('selected_channel'),
                              'watchdog_nav_publishers': mp.get('nav_publishers'), 'watchdog_manual_publishers': mp.get('manual_publishers'),
                              'nav_input_ok': mp.get('nav_input_ok'), 'manual_input_ok': mp.get('manual_input_ok'),
                              'source_conflict_age_s': mp.get('source_conflict_age_s'), 'status_age_s': None})

    def snapshot(self):
        moving = self.output != (0., 0.)
        return {'mode': KIND, 'kind': KIND, 'operating_mode': self.operating_mode, 'ready': self.ready, 'blocker': self.reason,
                'source': self.source if moving else 'NONE', 'selected_source': self.source, 'deadman': False, 'generation': self.gen_n,
                'selected_linear': self.output[0], 'selected_turn': self.output[1], 'evidence': dict(self.evidence),
                'note': 'evidence from the Motion Watchdog status and the DDS graph (graph names are not authentication); this selector publishes no velocity',
                'motor_output': False}

    def close(self):
        self.timer.cancel()
