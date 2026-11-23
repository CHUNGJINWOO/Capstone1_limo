"""Field preflight (2026-11-23): pure evaluation of the robot record - every unsafe or stale condition is a FAIL."""
import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import field_preflight_20261123 as fp

GOOD = {
    'online': True, 'last_seen_age_s': 0.8, 'reconciled': True, 'reconciliation_required': False,
    'agent': {'protocol_version': 2, 'safety_contract_version': 1, 'mode': 'real', 'simulated': False, 'missions_enabled': False},
    'lease': None, 'operating_mode': {'mode': 'IDLE'},
    'state': {'safety_gate': {'status': 'NORMAL', 'movement_permitted': False, 'motor_path': {'status': 'LOCKED', 'cmd_vel_publishers': []}},
              'motion_watchdog': {'state': 'MOTION_AUTHORIZED', 'age_s': 0.2, 'last_fault': None},
              'lidar': {'status': 'OK'}, 'pose': {'status': 'OK'}, 'map': {'status': 'STATIC'}, 'battery': {'status': 'OK'},
              'depth_camera': {'status': 'OK'}, 'mapping_worker': {'enabled': False}},
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
