"""SIM/UNIT ONLY (2026-11-22): the VelocityArbiter never reuses a velocity command from before the current control generation.

Generation = (operating mode, server boot, lease epoch, owned goal UUID, cancel_requested, common conditions OK). Checked here with the
real VelocityArbiter.update() and fakes for the ROS node / Agent (no graph, no motor):
  * cancel requested -> zero at once, also while late Nav2 samples keep arriving and the cancel result never comes;
  * a new goal: Nav2 samples received before (or < NAV_GEN_SETTLE_S after) its admission are not forwarded;
  * NAVIGATION -> MANUAL with the deadman already held: zero until released and pressed again;
  * Gate / Watchdog blocked -> unblocked (resume, restart): Nav2 and manual commands need a fresh generation;
  * normal: a new goal drives after the settle time, a re-pressed deadman drives.
2026-11-22 (2): the smoother re-publishes its last command ~1 s after its input stops (nav2_smoother_tail_probe_20261122.py), so a
smoothed sample is forwarded only while the controller/behavior command (smoother input) is live in the current generation. Also:
Mapping end, server (Fleet) boot change, Agent restart (new Arbiter), Gate FAILSAFE -> reset -> NORMAL, late old samples.
"""
import threading
import time
import unittest
from unittest import mock

from limo_robot_agent import velocity_arbiter as va


class Pub:
    def __init__(self): self.out = []
    def publish(self, m): self.out.append((m.linear.x, m.angular.z))


class Robot:
    mode, simulated = 'mock', True
    def __init__(self):
        self.lock = threading.Lock(); self.active = None; self.gate = {'state': 'NORMAL', 'cmd_blocked': False, 'wheel_odom_stale': False}
        self.wd = True; self.pub = Pub()
    def create_publisher(self, *a): return self.pub
    def create_subscription(self, *a): return None
    def create_timer(self, *a, **k): return mock.Mock()
    def _age(self, k): return .05
    def watchdog_ok(self): return self.wd
    def manual_idle_evidence(self): return {'ok': True}
    def get_name(self): return 'fleet_robot_agent'


class Agent:
    def __init__(self): self.mode = 'NAVIGATION'; self.epoch = 1; self.boot = 'b1'
    def is_execution_allowed(self): return True
    def session_snapshot(self):
        return {'connected': True, 'reconciliation_required': False, 'server_boot_id': self.boot,
                'lease': {'epoch': self.epoch, 'expires_at': time.time() + 60}, 'operating_mode': {'mode': self.mode}}


class Bench:
    def __init__(self): self.s = {}
    def snapshot(self): return dict(self.s)


class Gen(unittest.TestCase):
    def setUp(self):
        self.r, self.a, self.b = Robot(), Agent(), Bench()
        self.arb = va.VelocityArbiter(self.r, self.a, self.b)
        self.arb.graph_locked = lambda: True

    def nav(self, v=.1, w=0., producer=True):
        if producer: self.arb.on_nav_raw(None)        # controller / behavior server command reached the smoother
        self.arb.on_nav(mock.Mock(linear=mock.Mock(x=v), angular=mock.Mock(z=w)))

    def tick(self):
        self.arb.update(); return self.arb.output, self.arb.reason

    def assert_rejected_after_switch(self):
        # the sample that arrives before the tick that detects the change is discarded (STALE); later ones are too young (PREDATES)
        o, why = self.tick(); self.assertEqual(o, (0., 0.)); self.assertIn(why, ('NAV_INPUT_STALE', 'NAV_INPUT_PREDATES_GENERATION'))
        self.nav(); self.assertEqual(self.tick(), ((0., 0.), 'NAV_INPUT_PREDATES_GENERATION'))

    def goal(self, uuid='g1', owner='fleet'):
        self.r.active = {'goal_uuid': uuid, 'owner': owner, 'cancel_requested': False}

    def drive_nav(self, secs, producer=True, v=.1):
        end = time.monotonic() + secs; out = []
        while time.monotonic() < end:
            self.nav(v, producer=producer); out.append(self.tick()); time.sleep(.02)
        return out

    def test_new_goal_settles_then_drives(self):
        self.nav(); self.goal()                        # sample from before admission
        self.assertEqual(self.tick()[0], (0., 0.))
        out = self.drive_nav(.6)
        first = next(i for i, o in enumerate(out) if o[0] != (0., 0.))
        self.assertTrue(all(o[1] == 'NAV_INPUT_PREDATES_GENERATION' for o in out[:first]))
        self.assertGreaterEqual(first, 10)             # >= 0.3 s at 20 ms per step (wall timing, lower bound ~0.2 s with slack)
        self.assertEqual(out[-1][0], (.1, 0.))

    def test_cancel_zero_at_once_and_late_samples_rejected(self):
        self.goal(); self.drive_nav(.5)
        self.assertEqual(self.arb.output, (.1, 0.))
        self.r.active['cancel_requested'] = True        # cancel result never arrives
        out = self.drive_nav(1.0)
        self.assertTrue(all(o == ((0., 0.), 'CANCEL_REQUESTED') for o in out), out[:3])

    def test_terminal_then_new_goal_does_not_reuse_tail(self):
        self.goal('g1'); self.drive_nav(.5)
        self.r.active['cancel_requested'] = True; self.drive_nav(.1)
        self.r.active = None; self.nav(.08)              # smoother ramp-down tail
        self.assertEqual(self.tick(), ((0., 0.), 'NO_OWNED_GOAL'))
        self.goal('g2'); self.nav(.08)                   # tail sample lands right after the new admission
        self.assert_rejected_after_switch()

    def test_navigation_to_manual_held_deadman_needs_release(self):
        self.goal(); self.drive_nav(.5)
        self.r.active = None; self.a.mode = 'MANUAL'
        self.b.s = {'deadman': True, 'expires': time.monotonic() + 10, 'requested_linear': .08, 'requested_turn': 0.}
        self.assertEqual(self.tick(), ((0., 0.), 'MANUAL_DEADMAN_NOT_REARMED'))
        self.b.s = {'deadman': False}; self.assertEqual(self.tick()[0], (0., 0.))
        self.b.s = {'deadman': True, 'expires': time.monotonic() + 10, 'requested_linear': .08, 'requested_turn': 0.}
        self.assertEqual(self.tick(), ((.08, 0.), None))

    def test_gate_resume_and_watchdog_reauth_start_new_generation(self):
        self.a.mode = 'MANUAL'; self.b.s = {}
        self.tick()
        self.b.s = {'deadman': True, 'expires': time.monotonic() + 10, 'requested_linear': .08, 'requested_turn': 0.}
        self.assertEqual(self.tick()[0], (.08, 0.))
        for blocker in ('gate', 'watchdog'):
            with self.subTest(blocker=blocker):
                if blocker == 'gate': self.r.gate = dict(self.r.gate, state='FAILSAFE_SENSOR_TIMEOUT')
                else: self.r.wd = False
                self.assertEqual(self.tick()[0], (0., 0.))
                self.r.gate = dict(self.r.gate, state='NORMAL'); self.r.wd = True        # resumed, deadman still held
                self.assertEqual(self.tick(), ((0., 0.), 'MANUAL_DEADMAN_NOT_REARMED'))
                self.b.s['deadman'] = False; self.tick(); self.b.s['deadman'] = True
                self.assertEqual(self.tick()[0], (.08, 0.))
        self.a.mode = 'NAVIGATION'; self.b.s = {}; self.goal(); self.drive_nav(.5)
        self.r.gate = dict(self.r.gate, state='STALL_LATCHED'); self.nav(); self.tick()
        self.r.gate = dict(self.r.gate, state='NORMAL'); self.nav()
        self.assert_rejected_after_switch()

    def test_lease_epoch_change_is_new_generation(self):
        self.goal(); self.drive_nav(.5); self.a.epoch = 2; self.nav()
        self.assert_rejected_after_switch()

    def test_mapping_owner_still_enforced(self):
        self.a.mode = 'MAPPING'; self.goal(owner='fleet'); self.nav()
        self.assertEqual(self.tick(), ((0., 0.), 'GOAL_OWNER_MISMATCH'))

    # ---- 2026-11-22 (2)
    def test_smoother_replay_without_live_controller_is_cut(self):
        self.goal(); self.drive_nav(.5); self.assertEqual(self.arb.output, (.1, 0.))
        t0 = time.monotonic()
        out = self.drive_nav(.8, producer=False)       # controller died / froze without a zero; smoother repeats 0.1 for ~1 s
        last = max((i for i, o in enumerate(out) if o[0] != (0., 0.)), default=-1)
        self.assertLessEqual(last * .02, va.NAV_RAW_MAX_AGE_S + .06, out[:3])
        self.assertEqual(out[-1], ((0., 0.), 'NAV_PRODUCER_NOT_LIVE'))

    def test_new_goal_never_forwards_smoother_tail_without_new_controller_command(self):
        self.goal('g1'); self.drive_nav(.5); self.r.active = None; self.tick()
        self.goal('g2'); out = self.drive_nav(.8, producer=False)
        self.assertTrue(all(o[0] == (0., 0.) for o in out), out[-3:])
        self.assertEqual(out[-1][1], 'NAV_PRODUCER_NOT_LIVE')
        out = self.drive_nav(.5)                       # the new goal's controller output arrives -> drives
        self.assertEqual(out[-1], ((.1, 0.), None))

    def test_late_old_sample_inside_settle_rejected(self):
        self.goal('g1'); self.drive_nav(.5); self.r.active = None; self.tick()
        self.goal('g2'); self.tick(); time.sleep(.1); self.nav(.1)        # old sample delayed 0.1 s past the admission
        self.assertEqual(self.tick(), ((0., 0.), 'NAV_INPUT_PREDATES_GENERATION'))

    def test_mapping_end_then_new_mapping_goal(self):
        self.a.mode = 'MAPPING'; self.goal('f1', owner='frontier'); out = self.drive_nav(.5)
        self.assertEqual(out[-1], ((.1, 0.), None))
        self.a.mode = 'IDLE'; self.r.active = None; self.nav()
        self.assertEqual(self.tick(), ((0., 0.), 'MODE_IDLE'))
        self.a.mode = 'MAPPING'; self.goal('f2', owner='frontier'); self.nav()
        self.assert_rejected_after_switch()

    def test_server_boot_change_is_new_generation(self):
        self.goal(); self.drive_nav(.5); self.a.boot = 'b2'; self.nav()
        self.assert_rejected_after_switch()

    def test_agent_restart_new_arbiter_does_not_forward_first_samples(self):
        self.goal(); self.drive_nav(.5)
        self.arb = va.VelocityArbiter(self.r, self.a, self.b); self.arb.graph_locked = lambda: True
        self.nav()
        o, why = self.tick(); self.assertEqual(o, (0., 0.))
        self.nav(); self.assertEqual(self.tick(), ((0., 0.), 'NAV_INPUT_PREDATES_GENERATION'))
        self.assertEqual(self.drive_nav(.5)[-1], ((.1, 0.), None))

    def test_gate_failsafe_reset_resume_sequence(self):
        self.goal(); self.drive_nav(.5)
        for st in ('FAILSAFE_SENSOR_TIMEOUT', 'INITIALIZING'):
            self.r.gate = dict(self.r.gate, state=st, cmd_blocked=True); self.nav()
            self.assertEqual(self.tick(), ((0., 0.), 'GATE_NOT_NORMAL'))
        self.r.gate = dict(self.r.gate, state='NORMAL', cmd_blocked=False); self.nav()
        self.assert_rejected_after_switch()
        self.assertEqual(self.drive_nav(.5)[-1], ((.1, 0.), None))


if __name__ == '__main__':
    unittest.main(verbosity=2)
