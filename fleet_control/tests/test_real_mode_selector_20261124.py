"""RealModeSelector (G2, 2026-11-24): evidence for a real Agent (no Arbiter). Stub ROS and a modelled DDS graph; not a real-robot result."""
import sys, threading, time, unittest
from pathlib import Path
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'tests')]
import test_agent_safety_v1 as _stub
from limo_robot_agent.mode_selector import RealModeSelector, KIND


class Fixture:
    def __init__(self, mode='MAPPING', rev=3):
        self.graph_pubs = {'/cmd_vel': [], '/limo_driver/cmd_vel': [], '/cmd_vel_locked': ['stall_detector_node'], '/stall_detector/status_json': ['stall_detector_node'],
                           '/cmd_vel_raw': ['fleet_motion_watchdog'], '/fleet/nav_cmd_vel': ['velocity_smoother'], '/fleet/manual_cmd_vel': ['manual_mapping_teleop']}
        self.graph_subs = {'/cmd_vel': [], '/cmd_vel_raw': ['stall_detector_node'], '/fleet/nav_cmd_vel': ['fleet_motion_watchdog'], '/fleet/manual_cmd_vel': ['fleet_motion_watchdog']}
        self.sess = {'connected': True, 'reconciliation_required': False, 'server_boot_id': 'sb1', 'agent_boot_id': 'ab1', 'contract_generation': 1,
                     'lease': {'epoch': 5, 'expires_at': time.time() + 60}, 'operating_mode': {'mode': mode, 'revision': rev, 'epoch': 5}}
        self.wd = {'state': 'MOTION_AUTHORIZED', 'boot_id': 'wd1', 'agent_boot_id': 'ab1',
                   'speed_clamp': {'enabled': True, 'synced': True, 'publisher_ok': True, 'fault': None},
                   'mode_policy': {'enabled': True, 'mode': mode, 'revision': rev, 'epoch': 5, 'age_s': 0.2, 'selected_channel': {'MANUAL': 'manual'}.get(mode, 'nav'),
                                   'nav_input_ok': True, 'manual_input_ok': True, 'source_conflict_age_s': None, 'nav_publishers': ['velocity_smoother'], 'manual_publishers': ['manual_mapping_teleop']},
                   'last_output': {'linear_x': 0.0, 'angular_z': 0.0, 'age_s': 0.1}}
        self.wd_age = 0.0
        self.active, self.idle_ok, self.gate_age = None, True, 0.1
        pub = lambda topic: [SimpleNamespace(node_namespace='/', node_name=n) for n in self.graph_pubs.get(topic, [])]
        sub = lambda topic: [SimpleNamespace(node_namespace='/', node_name=n) for n in self.graph_subs.get(topic, [])]
        fx = self
        class Robot:
            lock = threading.RLock()
            gate = {'state': 'NORMAL', 'cmd_blocked': False, 'wheel_odom_stale': False}
            get_publishers_info_by_topic = staticmethod(pub); get_subscriptions_info_by_topic = staticmethod(sub)
            create_timer = staticmethod(lambda *a, **k: SimpleNamespace(cancel=lambda: None))
            def watchdog_ok(s): return fx.wd.get('state') == 'MOTION_AUTHORIZED'
            def _age(s, key): return fx.gate_age if key == 'gate' else 0.1
            @property
            def active(s): return fx.active
            @property
            def motion_watchdog(s): return (time.monotonic() - fx.wd_age, fx.wd) if fx.wd is not None else None
            def manual_idle_evidence(s): return {'ok': fx.idle_ok, 'reason': None if fx.idle_ok else 'NAV2_BUSY'}
        self.robot = Robot()
        self.agent = SimpleNamespace(session_snapshot=lambda: self.sess, is_execution_allowed=lambda: True)
        self.sel = RealModeSelector(self.robot, self.agent)

    def run(self):
        self.sel.update(); self.sel.update(); return self.sel.reason


class Selector(unittest.TestCase):
    def test_server_expected_blockers_when_everything_is_in_order(self):
        for mode, expected in (('MAPPING', 'NO_OWNED_GOAL'), ('NAVIGATION', 'NO_OWNED_GOAL'), ('MANUAL', None), ('IDLE', 'MODE_IDLE')):
            with self.subTest(mode):
                self.assertEqual(Fixture(mode).run(), expected)

    def test_it_publishes_nothing_and_reports_the_watchdog_as_evidence(self):
        f = Fixture('MAPPING'); f.run(); s = f.sel.snapshot()
        self.assertEqual((s['kind'], s['mode'], s['motor_output'], s['operating_mode']), (KIND, KIND, False, 'MAPPING'))
        ev = s['evidence']; self.assertEqual((ev['watchdog_mode'], ev['watchdog_revision'], ev['selected_channel'], ev['watchdog_boot_id']), ('MAPPING', 3, 'nav', 'wd1'))
        self.assertEqual(ev['agent_graph'], {'nav_publishers': ['velocity_smoother'], 'manual_publishers': ['manual_mapping_teleop']})

    def test_it_never_confirms_without_the_watchdog(self):
        cases = {'WATCHDOG_STATUS_STALE': lambda f: setattr(f, 'wd_age', 5.0), 'WATCHDOG_MODE_NOT_CONFIRMED': lambda f: f.wd['mode_policy'].update(mode='MANUAL'),
                 'WATCHDOG_MODE_NOT_CONFIRMED ': lambda f: f.wd['mode_policy'].update(revision=2), 'WATCHDOG_MODE_NOT_CONFIRMED  ': lambda f: f.wd['mode_policy'].update(selected_channel='manual'),
                 'WATCHDOG_MODE_GATE_DISABLED': lambda f: f.wd['mode_policy'].update(enabled=False), 'WATCHDOG_AGENT_SYNC_LOST': lambda f: f.wd['speed_clamp'].update(synced=False),
                 'WATCHDOG_INPUT_NOT_EXCLUSIVE': lambda f: f.wd['mode_policy'].update(nav_input_ok=False), 'WATCHDOG_INPUT_NOT_EXCLUSIVE ': lambda f: f.wd['mode_policy'].update(manual_input_ok=False),
                 'MOTION_SOURCE_CONFLICT': lambda f: f.wd['mode_policy'].update(source_conflict_age_s=1.0), 'WATCHDOG_NOT_AUTHORIZED': lambda f: f.wd.update(state='FAILSAFE')}
        for want, mutate in cases.items():
            with self.subTest(want):
                f = Fixture('MAPPING'); mutate(f); self.assertEqual(f.run(), want.strip())

    def test_graph_violations_block(self):
        cases = {'second nav writer': lambda f: f.graph_pubs.update({'/fleet/nav_cmd_vel': ['velocity_smoother', 'rogue']}), 'foreign nav writer': lambda f: f.graph_pubs.update({'/fleet/nav_cmd_vel': ['rogue']}),
                 'foreign manual writer': lambda f: f.graph_pubs.update({'/fleet/manual_cmd_vel': ['rogue']}), 'driver input writer': lambda f: f.graph_pubs.update({'/limo_driver/cmd_vel': ['rogue']}),
                 '/cmd_vel writer': lambda f: f.graph_pubs.update({'/cmd_vel': ['vendor_demo']}), '/cmd_vel subscriber': lambda f: f.graph_subs.update({'/cmd_vel': ['vendor_driver']}),
                 'second /cmd_vel_raw writer': lambda f: f.graph_pubs.update({'/cmd_vel_raw': ['fleet_motion_watchdog', 'rogue']}), 'nobody listens to the Gate input': lambda f: f.graph_subs.update({'/cmd_vel_raw': []}),
                 'someone else reads the nav channel': lambda f: f.graph_subs.update({'/fleet/nav_cmd_vel': ['fleet_motion_watchdog', 'spy']})}
        for name, mutate in cases.items():
            with self.subTest(name):
                f = Fixture('MAPPING'); mutate(f); self.assertEqual(f.run(), 'VELOCITY_GRAPH_NOT_LOCKED')

    def test_session_conditions_are_the_arbiters(self):
        f = Fixture('MAPPING'); f.sess['lease'] = None; self.assertEqual(f.run(), 'LEASE_MISSING_OR_EXPIRED')
        f = Fixture('MAPPING'); f.sess['connected'] = False; self.assertEqual(f.run(), 'FLEET_DISCONNECTED')
        f = Fixture('MAPPING'); f.sess['reconciliation_required'] = True; self.assertEqual(f.run(), 'RECONCILIATION_REQUIRED')
        f = Fixture('MAPPING'); f.gate_age = 5.0; self.assertEqual(f.run(), 'GATE_STATUS_STALE')
        f = Fixture('MAPPING'); f.robot.gate = {'state': 'FAILSAFE_SENSOR_TIMEOUT', 'cmd_blocked': False, 'wheel_odom_stale': False}; self.assertEqual(f.run(), 'GATE_NOT_NORMAL')
        f = Fixture('MAPPING'); f.sess['operating_mode'] = None; self.assertEqual(f.run(), 'NO_OPERATING_MODE')

    def test_goal_and_idle_rules(self):
        f = Fixture('MANUAL'); f.active = {'owner': 'frontier'}; self.assertEqual(f.run(), 'FLEET_MISSION_ACTIVE')
        f = Fixture('MANUAL'); f.idle_ok = False; self.assertEqual(f.run(), 'NAV2_BUSY')
        f = Fixture('MAPPING'); f.active = {'owner': 'fleet'}; self.assertEqual(f.run(), 'GOAL_OWNER_MISMATCH')
        f = Fixture('MAPPING'); f.active = {'owner': 'frontier', 'cancel_requested': True}; self.assertEqual(f.run(), 'CANCEL_REQUESTED')
        f = Fixture('NAVIGATION'); f.active = {'owner': 'fleet'}; self.assertIsNone(f.run())

    def test_selected_output_is_what_the_watchdog_forwarded_and_goes_stale_to_zero(self):
        f = Fixture('MAPPING'); f.wd['last_output'] = {'linear_x': 0.05, 'angular_z': 0.1, 'age_s': 0.1}; f.run(); s = f.sel.snapshot()
        self.assertEqual((s['selected_linear'], s['selected_turn'], s['source']), (0.05, 0.1, 'NAV2'))
        f.wd['last_output']['age_s'] = 2.0; f.run(); s = f.sel.snapshot(); self.assertEqual((s['selected_linear'], s['selected_turn'], s['source']), (0.0, 0.0, 'NONE'))

    def test_a_mode_change_starts_a_new_generation(self):
        f = Fixture('MAPPING'); f.run(); g = f.sel.gen_n; f.sess['operating_mode'] = {'mode': 'IDLE', 'revision': 4, 'epoch': 5}; f.run(); self.assertGreater(f.sel.gen_n, g)

    def test_an_exception_never_yields_ready(self):
        f = Fixture('MAPPING'); f.robot.get_publishers_info_by_topic = staticmethod(lambda t: (_ for _ in ()).throw(RuntimeError('graph')))
        f.sel.update(); self.assertFalse(f.sel.ready); self.assertTrue(f.sel.reason.startswith('SELECTOR_ERROR'))


if __name__ == '__main__':
    unittest.main()
