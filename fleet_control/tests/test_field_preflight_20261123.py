"""Field preflight (2026-11-23): pure evaluation of the robot record - every unsafe or stale condition is a FAIL."""
import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import field_preflight_20261123 as fp

GOOD = {
    'online': True, 'last_seen_age_s': 0.8, 'reconciled': True, 'reconciliation_required': False,
    'agent': {'protocol_version': 2, 'safety_contract_version': 1, 'mode': 'real', 'simulated': False, 'missions_enabled': False, 'velocity_selector': 'WATCHDOG_SELECTOR', 'locked_velocity_arbiter': False},
    'lease': None, 'operating_mode': {'mode': 'IDLE'},
    'state': {'safety_gate': {'status': 'NORMAL', 'movement_permitted': False, 'motor_path': {'status': 'LOCKED', 'cmd_vel_publishers': []}},
              'motion_watchdog': {'state': 'MOTION_AUTHORIZED', 'age_s': 0.2, 'last_fault': None},
              'lidar': {'status': 'OK'}, 'pose': {'status': 'OK'}, 'map': {'status': 'STATIC'}, 'battery': {'status': 'OK'},
              'depth_camera': {'status': 'OK'}, 'mapping_worker': {'enabled': False},
              'velocity': {'status': 'OK'},
              'velocity_arbiter': {'kind': 'WATCHDOG_SELECTOR', 'motor_output': False, 'selected_linear': 0.0, 'selected_turn': 0.0, 'evidence': {'nav_input_ok': True, 'manual_input_ok': True, 'source_conflict_age_s': None}},
              'speed_limits': {'enforced_by': 'WATCHDOG', 'stale': False, 'applied': {'manual': None, 'auto': None},
                               'watchdog_evidence': {'server_verified': True, 'agent_verified': True, 'evidence_age_s': 0.4, 'watchdog_boot_id': 'w' * 32, 'agent_boot_id': 'a' * 12,
                                                     'output_gid': '010f' + '0' * 20, 'synced': True, 'publisher_ok': True, 'fault': None}}},
}


def levels(record, real=True):
    return {item.split()[0]: level for level, item, _ in fp.evaluate(record, real)}


class Preflight(unittest.TestCase):
    def test_good_locked_real_robot_has_no_fail(self):
        self.assertNotIn('FAIL', levels(GOOD).values())

    def mutate(self, path, value):
        r = copy.deepcopy(GOOD)
        cur = r
        for k in path[:-1]:
            cur = cur[k]
        cur[path[-1]] = value
        return r

    def test_each_unsafe_or_stale_condition_fails(self):
        cases = {
            'A1': (['online'], False), 'A2': (['last_seen_age_s'], 9.0), 'A3': (['reconciled'], False),
            'A4': (['reconciliation_required'], True), 'A5': (['agent', 'protocol_version'], 1),
            'B1': (['state', 'safety_gate', 'status'], 'FAILSAFE_SENSOR_TIMEOUT'),
            'B2': (['state', 'safety_gate', 'motor_path', 'status'], 'ARMED'),
            'B3': (['state', 'motion_watchdog', 'age_s'], 2.0),
            'E1': (['agent', 'mode'], 'mock'), 'E2': (['agent', 'missions_enabled'], True),
            'E3': (['state', 'mapping_worker', 'enabled'], True),
        }
        for key, (path, value) in cases.items():
            with self.subTest(key=key):
                self.assertEqual(levels(self.mutate(path, value))[key], 'FAIL')

    def test_watchdog_unauthorized_fails(self):
        self.assertEqual(levels(self.mutate(['state', 'motion_watchdog', 'state'], 'LOCKED'))['B3'], 'FAIL')

    def test_movement_permitted_is_only_a_warning(self):
        self.assertEqual(levels(self.mutate(['state', 'safety_gate', 'movement_permitted'], True))['B5'], 'WARN')

    def test_non_real_check_skips_real_items(self):
        self.assertFalse([k for k in levels(GOOD, real=False) if k.startswith('E')])


if __name__ == '__main__':
    unittest.main()


class SpeedLimitChecks(unittest.TestCase):
    """F checks (read-only): the real robot needs a server- and Agent-verified Watchdog clamp; a latched foreign writer fails."""
    def robot(self, **over):
        import copy
        r = copy.deepcopy(GOOD) if 'GOOD' in globals() else None
        return r

    def run_eval(self, sl, mp_extra=None, real=True):
        import field_preflight_20261123 as fp
        rob = {'online': True, 'last_seen_age_s': 0.5, 'reconciled': True, 'reconciliation_required': False,
               'agent': {'protocol_version': 2, 'safety_contract_version': 1, 'mode': 'real', 'simulated': False, 'missions_enabled': False, 'velocity_selector': 'WATCHDOG_SELECTOR', 'locked_velocity_arbiter': False},
               'state': {'safety_gate': {'status': 'NORMAL', 'motor_path': dict({'status': 'LOCKED', 'cmd_vel_publishers': []}, **(mp_extra or {})), 'movement_permitted': False},
                         'motion_watchdog': {'state': 'MOTION_AUTHORIZED', 'age_s': 0.2}, 'speed_limits': sl, 'velocity': {'status': 'OK'}, 'velocity_arbiter': {'kind': 'WATCHDOG_SELECTOR', 'motor_output': False, 'selected_linear': 0.0, 'selected_turn': 0.0, 'evidence': {'nav_input_ok': True, 'manual_input_ok': True, 'source_conflict_age_s': None}}, 'mapping_worker': {'enabled': False}}}
        return {i: l for l, i, _ in fp.evaluate(rob, real)}

    def good_sl(self):
        return {'enforced_by': 'WATCHDOG', 'stale': False, 'applied': {'manual': None, 'auto': None},
                'watchdog_evidence': {'server_verified': True, 'agent_verified': True, 'evidence_age_s': 0.4, 'watchdog_boot_id': 'w' * 32, 'agent_boot_id': 'a' * 12, 'output_gid': '010f' + '0' * 20,
                                      'synced': True, 'publisher_ok': True, 'fault': None}}

    def test_good_real_robot(self):
        r = self.run_eval(self.good_sl())
        self.assertTrue(all(r[k] == 'PASS' for k in r if k.startswith('F')), {k: v for k, v in r.items() if k.startswith('F')})

    def test_no_enforcement_point_fails_on_a_real_robot(self):
        r = self.run_eval({'enforced_by': None, 'stale': False}); self.assertEqual(r['F1 a speed-limit enforcement point is reported (WATCHDOG on a real robot)'], 'FAIL')

    def test_unverified_or_old_evidence_fails(self):
        for mut in ({'server_verified': False}, {'agent_verified': False}, {'evidence_age_s': 5.0}, {'evidence_age_s': None}):
            sl = self.good_sl(); sl['watchdog_evidence'].update(mut)
            self.assertEqual(self.run_eval(sl)['F3 Watchdog evidence verified by the SERVER and the Agent, fresh (<= 2 s)'], 'FAIL', mut)

    def test_latched_foreign_writer_fails_and_stale_setting_warns(self):
        r = self.run_eval(self.good_sl(), {'foreign_writer_latched': True, 'foreign_writers_seen': ['rogue']})
        self.assertEqual(r['F6 no non-Gate writer was ever seen on /cmd_vel or the driver input (latched until Agent restart; detection only)'], 'FAIL')
        sl = self.good_sl(); sl['stale'] = True; sl['applied'] = {'auto': {'linear_mps': 0.06}}; sl['stale_reason'] = 'X'
        self.assertEqual(self.run_eval(sl)['F5 no stale speed-limit setting (a stale setting still clamps but needs a new request)'], 'WARN')

    def test_locked_with_a_writer_fails(self):
        r = self.run_eval(self.good_sl(), {'cmd_vel_publishers': ['rogue']}); self.assertEqual(r['F7 LOCKED means no writer on /cmd_vel or /limo_driver/cmd_vel'], 'FAIL')


class SelectorChecks(SpeedLimitChecks):
    def test_g_items_pass_on_a_good_real_robot_and_fail_without_a_selector_or_with_a_conflict(self):
        r = self.run_eval(self.good_sl()); self.assertTrue(all(v == 'PASS' for k, v in r.items() if k.startswith('G')), {k: v for k, v in r.items() if k.startswith('G')})
        import copy
        import field_preflight_20261123 as fp
        rob = copy.deepcopy(GOOD)
        rob['agent']['velocity_selector'] = None; self.assertEqual({i.split()[0]: l for l, i, _ in fp.evaluate(rob, True)}['G1'], 'FAIL')
        rob = copy.deepcopy(GOOD); rob['state']['velocity_arbiter']['evidence']['source_conflict_age_s'] = 1.0; self.assertEqual({i.split()[0]: l for l, i, _ in fp.evaluate(rob, True)}['G3'], 'FAIL')
        rob = copy.deepcopy(GOOD); rob['state']['velocity_arbiter']['evidence']['manual_input_ok'] = False; self.assertEqual({i.split()[0]: l for l, i, _ in fp.evaluate(rob, True)}['G2'], 'FAIL')
        rob = copy.deepcopy(GOOD); rob['state']['velocity_arbiter']['selected_linear'] = 0.05; self.assertEqual({i.split()[0]: l for l, i, _ in fp.evaluate(rob, True)}['G4'], 'FAIL')
