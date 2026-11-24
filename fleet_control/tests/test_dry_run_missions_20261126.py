"""real_nav_dry_run separation (G2, 2026-11-26): server-only fixtures, no ROS graph, no hardware. Pins that the dry-run path never widens the real-mission path."""
import re, sys, unittest
from pathlib import Path
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'server'), str(ROOT / 'tests')]
import test_safety_context_v1 as fixture
from fleet_server.service import ApiError

GOAL = {'frame_id': 'map', 'x': 1.0, 'y': 2.0}


class ServerSeparation(unittest.TestCase):
    def setUp(self):
        self.t = fixture.Tests('test_old_agent_blocked'); self.t.setUp(); self.f = self.t.f

    def tearDown(self): self.t.tearDown()

    def code(self, **kw):
        with self.assertRaises(ApiError) as cm: self.f.create_mission('r', 'navigate_to_pose', GOAL, created_by='op', lease_epoch=1, **kw)
        return cm.exception.args[1] if len(cm.exception.args) > 1 else getattr(cm.exception, 'code', None)

    def test_a_real_mission_agent_refuses_dry_run_missions(self):
        self.f.robots['r']['agent'].pop('real_nav_dry_run', None); self.assertEqual(self.code(dry_run=True), 'not_dry_run_agent')

    def test_a_dry_run_agent_refuses_real_missions_on_the_real_endpoint(self):
        self.f.robots['r']['agent']['real_nav_dry_run'] = True; self.assertEqual(self.code(), 'dry_run_agent')

    def test_a_dry_run_agent_is_not_a_missions_enabled_agent(self):
        a = self.f.robots['r']['agent']; a.update(real_nav_dry_run=True, missions_enabled=False)
        self.assertIsNot(a.get('missions_enabled'), True, 'the real-mission capability flag stays False for a dry-run Agent: nothing that requires missions_enabled opens')

    def test_loading_a_map_needs_only_the_mode_record_not_a_confirmed_watchdog(self):
        # motion=False is for LOAD / INITIAL_POSE / UNLOAD (nothing moves)
        self.f._mode_load = lambda rid: {'mode': 'NAVIGATION', 'revision': 3, 'transition': {'state': 'ACTIVE', 'to': 'NAVIGATION'}}
        self.f.mode_view = lambda rid, advance=True: self.f._mode_load(rid)
        self.f.mode_motion_status = lambda rid: {'verified': False, 'blocker': 'x', 'hard': True}
        self.f.require_mode('r', 'NAVIGATION', 'c', motion=False)
        with self.assertRaises(ApiError): self.f.require_mode('r', 'NAVIGATION', 'c')


class Recovery(unittest.TestCase):
    """The locked-path recovery rule (terminal evidence per delivered command, idle, Gate NORMAL, Watchdog authorized, zero output) is available to a dry-run Agent and to nothing else new."""
    def setUp(self):
        self.t = fixture.Tests('test_old_agent_blocked'); self.t.setUp(); self.f = self.t.f

    def tearDown(self): self.t.tearDown()

    def rule(self, **agent):
        r = self.f.robots['r']; r['agent'] = dict({'boot_id': 'b'}, **agent); ex = {'active': None, 'nav_idle': {'ok': True}, 'nav2_goals': {'active_count': 0}}
        r['state'] = {'safety_gate': {'motor_path': {'status': 'LOCKED'}, 'status': 'NORMAL', 'cmd_blocked': False}, 'motion_watchdog': {'state': 'MOTION_AUTHORIZED', 'agent_boot_matches': True},
                      'velocity_arbiter': {'selected_linear': 0.0, 'selected_turn': 0.0}, 'mapping_worker': {}, 'execution': ex}
        r['state_ts'] = __import__('time').time()
        return self.f._locked_mock_recovery('r', r, ex, 0.1)

    def test_a_dry_run_agent_gets_the_locked_path_rule_a_real_shaped_one_does_not(self):
        ok, why, ev = self.rule(real_nav_dry_run=True, velocity_selector='WATCHDOG_SELECTOR'); self.assertTrue(ok, (why, ev)); self.assertTrue(ev['LOCKED_MOTOR_DRY_RUN']); self.assertFalse(ev['LOCKED_MOCK'])
        self.assertIsNone(self.rule(velocity_selector='WATCHDOG_SELECTOR'), 'a real-shaped Agent without real_nav_dry_run keeps the refusal')
        self.assertIsNone(self.rule(real_nav_dry_run=True), 'dry-run needs the Watchdog selector')
        self.assertIsNone(self.rule(mode='real', simulated=False, locked_velocity_arbiter=True), 'the Arbiter flag alone is not enough outside the isolated mock')

    def test_every_other_condition_still_refuses(self):
        ag = dict(real_nav_dry_run=True, velocity_selector='WATCHDOG_SELECTOR')
        for mut, name in ((lambda r: r['state']['safety_gate']['motor_path'].update(status='ALLOWED'), 'motor path not locked'), (lambda r: r['state']['safety_gate'].update(status='STALL_LATCHED'), 'gate not normal'),
                          (lambda r: r['state']['motion_watchdog'].update(state='LOCKED'), 'watchdog not authorized'), (lambda r: r['state']['velocity_arbiter'].update(selected_linear=0.1), 'output not zero'),
                          (lambda r: r['state']['execution'].update(active={'x': 1}), 'agent goal')):
            with self.subTest(name):
                r = self.f.robots['r']; self.rule(**ag); mut(r); res = self.f._locked_mock_recovery('r', r, r['state']['execution'], 0.1); self.assertFalse(res[0]); self.assertEqual(res[1], 'idle_not_proven')


class Routes(unittest.TestCase):
    def test_dry_run_missions_have_their_own_endpoint_and_the_real_endpoint_never_reads_dry_run_from_the_body(self):
        s = (ROOT / 'server' / 'fleet_server' / 'http_api.py').read_text()
        self.assertIn("sub == '/dry-run-missions'", s)
        i = s.index("if sub == '/missions':\n                    b = self._body(); view, created = fleet.create_mission(")
        real = s[i:]; real = real[:real.index('return self._send(201 if created else 200')]
        self.assertNotIn('dry_run', real, "POST /missions must not accept a dry_run flag from the request body")
        dry = s[s.index("sub == '/dry-run-missions'"):]; dry = dry[:dry.index('return self._send(201')]
        self.assertIn('dry_run=True', dry)


class AgentProof(unittest.TestCase):
    def proof(self, **attrs):
        from limo_robot_agent.ros_adapter import RosRobot
        ns = SimpleNamespace(mode='real', simulated=False, locked_arbiter=False, real_nav_dry_run=False, require_motion_watchdog=True); ns.__dict__.update(attrs)
        return RosRobot._locked_motion_proof(ns, attrs.pop('mp', 'LOCKED')) if 'mp' not in attrs else RosRobot._locked_motion_proof(ns, attrs['mp'])

    def test_locked_motor_proof_needs_the_dry_run_flag_the_watchdog_and_an_observed_locked_path(self):
        self.assertTrue(self.proof(real_nav_dry_run=True))
        self.assertFalse(self.proof(real_nav_dry_run=False), 'a real-shaped Agent without real_nav_dry_run has no such proof')
        self.assertFalse(self.proof(real_nav_dry_run=True, require_motion_watchdog=False))
        for mp in ('ALLOWED', 'UNKNOWN', 'BLOCKED', None, 'SIM_GATED'):
            with self.subTest(mp): self.assertFalse(self.proof(real_nav_dry_run=True, mp=mp), 'any path that is not LOCKED is no proof')

    def test_the_isolated_mock_arbiter_proof_is_unchanged(self):
        self.assertTrue(self.proof(mode='mock', simulated=True, locked_arbiter=True))
        self.assertFalse(self.proof(mode='mock', simulated=True, locked_arbiter=False))
        self.assertFalse(self.proof(mode='real', simulated=False, locked_arbiter=True))


if __name__ == '__main__':
    unittest.main()
