"""Agent side of the Watchdog clamp: enforced_by, confirmation before 'applied', published limits (G2, 2026-11-23). Stub ROS, no graph."""
import json, sys, time, unittest
from pathlib import Path
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'tests')]
import test_agent_safety_v1 as _stub
from limo_robot_agent.speed_limits import SpeedLimiter
from limo_robot_agent.watchdog_speed_link import WatchdogSpeedLink
from test_speed_limits_agent_20261123 import session, request


class Robot:
    def __init__(self):
        self.sent = []; self.motion_watchdog = None
        self.create_publisher = lambda *a, **k: SimpleNamespace(publish=lambda m: self.sent.append(json.loads(m.data)))
        self.create_timer = lambda *a, **k: SimpleNamespace(cancel=lambda: None)

    def wd(self, applied=None, synced=True, enabled=True, boot='ab1', fault=None, publisher_ok=True, age=0.0):
        self.motion_watchdog = (time.monotonic() - age, {'state': 'MOTION_AUTHORIZED', 'agent_boot_id': boot,
                                                          'speed_clamp': {'enabled': enabled, 'synced': synced, 'publisher_ok': publisher_ok, 'fault': fault, 'applied': applied or {}}})


def wd_applied(scope='auto', lin=0.12, ang=0.35, gen=1, rid='r1', stale=False):
    return {scope: {'linear_mps': lin, 'angular_rps': ang, 'generation': gen, 'request_id': rid, 'stale': stale}}


class Link(unittest.TestCase):
    def setUp(self):
        self.robot, self.sess = Robot(), session()
        self.limiter = SpeedLimiter()
        self.agent = SimpleNamespace(session_snapshot=lambda: self.sess)
        self.link = WatchdogSpeedLink(self.robot, self.agent, self.limiter); self.link._String = lambda data: SimpleNamespace(data=data)    # the stub String is a bare object

    def test_enforced_by_only_while_the_watchdog_reports_it_can(self):
        self.link.tick(); self.assertIsNone(self.limiter.enforced_by)                                        # no Watchdog status at all
        for kw in (dict(enabled=False), dict(synced=False), dict(boot='other'), dict(fault='X'), dict(publisher_ok=False), dict(age=3.0)):
            with self.subTest(kw):
                self.robot.wd(**kw); self.link.tick(); self.assertIsNone(self.limiter.enforced_by)
        self.robot.wd(); self.link.tick(); self.assertEqual(self.limiter.enforced_by, 'WATCHDOG')

    def test_request_without_a_working_watchdog_is_rejected_and_nothing_is_applied(self):
        self.sess['speed_limit_request'] = request(); self.link.tick()
        self.assertEqual(self.limiter.last_result['reason'], 'NO_ENFORCEMENT_POINT'); self.assertEqual(self.limiter.limits('auto')[0], 0.15)

    def test_not_applied_until_the_watchdog_shows_the_same_generation_and_values(self):
        self.robot.wd(); self.sess['speed_limit_request'] = request(gen=1, lin=0.12, ang=0.35, rid='r1'); self.link.tick()
        self.assertEqual(self.limiter.last_result['status'], 'PENDING_ENFORCEMENT'); self.assertEqual(self.limiter.limits('auto')[0], 0.15, 'not applied yet')
        self.assertEqual(self.robot.sent[-1]['scopes']['auto']['max_linear_mps'], 0.12, 'the candidate is offered to the Watchdog')
        for wrong in (wd_applied(gen=2), wd_applied(rid='x'), wd_applied(lin=0.10), wd_applied(ang=0.3), wd_applied(stale=True), wd_applied(scope='manual')):
            with self.subTest(wrong):
                self.robot.wd(applied=wrong); self.link.tick(); self.assertEqual(self.limiter.last_result['status'], 'PENDING_ENFORCEMENT')
        self.robot.wd(applied=wd_applied()); self.link.tick()
        self.assertEqual(self.limiter.last_result['status'], 'APPLIED'); self.assertEqual(self.limiter.limits('auto')[0], 0.12); self.assertEqual(self.limiter.snapshot()['enforced_by'], 'WATCHDOG')

    def test_no_confirmation_in_time_rejects_and_leaves_the_old_value(self):
        self.robot.wd(); self.sess['speed_limit_request'] = request(); self.link.tick(); self.limiter.candidate['deadline'] = time.monotonic() - 1; self.link.tick()
        self.assertEqual(self.limiter.last_result['reason'], 'WATCHDOG_NO_CONFIRMATION'); self.assertEqual(self.limiter.limits('auto')[0], 0.15); self.assertIsNone(self.limiter.candidate)

    def test_binding_change_before_confirmation_drops_the_candidate(self):
        self.robot.wd(); self.sess['speed_limit_request'] = request(); self.link.tick()
        self.sess['lease'] = {'epoch': 9, 'expires_at': time.time() + 60}; self.link.tick()
        self.assertEqual(self.limiter.last_result['reason'], 'BINDING_CHANGED_BEFORE_ENFORCEMENT'); self.assertEqual(self.limiter.limits('auto')[0], 0.15)

    def test_published_message_carries_boot_binding_and_only_current_settings(self):
        self.robot.wd(); self.sess['speed_limit_request'] = request(gen=1, lin=0.08, ang=0.2); self.link.tick(); self.robot.wd(applied=wd_applied(lin=0.08, ang=0.2)); self.link.tick()
        self.sess['speed_limit_request'] = None; self.link.tick(); m = self.robot.sent[-1]
        self.assertEqual((m['agent_boot_id'], m['binding']), ('ab1', ['sb1', 5, 3])); self.assertEqual(m['scopes']['auto']['generation'], 1); self.assertEqual(m['scopes']['auto']['request_id'], 'r1')
        self.sess['lease'] = {'epoch': 9, 'expires_at': time.time() + 60}; self.link.tick(); m = self.robot.sent[-1]
        self.assertEqual(m['scopes'], {}, 'a stale setting is not offered again'); self.assertEqual(m['binding'][1], 9); self.assertEqual(self.limiter.limits('auto')[0], 0.08, 'but it keeps clamping in the Agent')

    def test_the_message_states_the_operating_mode_and_the_owned_goal_only_while_the_agent_would_execute(self):
        self.agent.is_execution_allowed = lambda: True
        self.sess['operating_mode'] = {'mode': 'MAPPING', 'revision': 7, 'epoch': 5}; self.robot.active = {'owner': 'frontier', 'cancel_requested': False}
        self.link.tick(); m = self.robot.sent[-1]; self.assertEqual(m['mode'], {'name': 'MAPPING', 'revision': 7, 'epoch': 5}); self.assertEqual(m['goal'], {'owner': 'frontier', 'cancel_requested': False})
        self.robot.active = {'owner': 'frontier', 'cancel_requested': True}; self.link.tick(); self.assertTrue(self.robot.sent[-1]['goal']['cancel_requested'])
        self.robot.active = None; self.link.tick(); self.assertEqual(self.robot.sent[-1]['goal'], {'owner': None, 'cancel_requested': False})
        self.agent.is_execution_allowed = lambda: False; self.link.tick(); self.assertEqual(self.robot.sent[-1]['mode']['name'], 'IDLE')       # not connected / stale: IDLE, never the old mode
        self.agent.is_execution_allowed = lambda: True; self.sess['reconciliation_required'] = True; self.link.tick(); self.assertEqual(self.robot.sent[-1]['mode']['name'], 'IDLE')
        self.sess['reconciliation_required'] = False; self.sess['connected'] = False; self.link.tick(); self.assertEqual(self.robot.sent[-1]['mode']['name'], 'IDLE')
        self.sess['connected'] = True; self.sess['operating_mode'] = None; self.link.tick(); self.assertEqual(self.robot.sent[-1]['mode']['name'], 'IDLE')

    def test_a_connection_loss_withdraws_the_mode_revision_for_good(self):
        self.agent.is_execution_allowed = lambda: True
        self.sess['operating_mode'] = {'mode': 'MANUAL', 'revision': 7, 'epoch': 5}; self.link.tick(); self.assertEqual(self.robot.sent[-1]['mode']['name'], 'MANUAL')
        self.agent.is_execution_allowed = lambda: False; self.link.tick(); self.assertEqual(self.robot.sent[-1]['mode']['name'], 'IDLE')
        self.agent.is_execution_allowed = lambda: True; self.link.tick(); self.assertEqual(self.robot.sent[-1]['mode']['name'], 'IDLE', 'the reconnect must not bring MANUAL back')
        self.sess['operating_mode'] = {'mode': 'IDLE', 'revision': 8, 'epoch': 5}; self.link.tick(); self.sess['operating_mode'] = {'mode': 'MANUAL', 'revision': 9, 'epoch': 5}; self.link.tick()
        self.assertEqual(self.robot.sent[-1]['mode'], {'name': 'MANUAL', 'revision': 9, 'epoch': 5}, 'a new revision (explicit request) is stated')

    def test_a_broken_tick_publishes_nothing_so_the_watchdog_loses_sync(self):
        n = len(self.robot.sent); self.agent.session_snapshot = lambda: (_ for _ in ()).throw(RuntimeError('x')); self.link.tick(); self.assertEqual(len(self.robot.sent), n)


if __name__ == '__main__':
    unittest.main()
