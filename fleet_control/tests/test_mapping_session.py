"""SQLite/API contract tests using authenticated simulated worker evidence."""
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.db import DB
from fleet_server.service import ApiError, Fleet


class MappingTest(unittest.TestCase):
    def setUp(self):
        root = tempfile.TemporaryDirectory()
        self.addCleanup(root.cleanup)
        self.db = DB(Path(root.name) / 'fleet.sqlite')
        self.addCleanup(self.db.c.close)
        self.fleet = Fleet(self.db, SimpleNamespace(offline_after_s=3))
        self.fleet.robots['a'] = dict(reconciled=True, online=True, state_ts=time.time(),
                                     agent=dict(mode='mock', simulated=True, mapping_fixture=True,
                                                missions_enabled=False, boot_id='worker',
                                                protocol_version=2),
                                     state=dict(execution=dict(active=None, nav2_goals=dict(
                                         known=True, publishers=1, active_count=0))))
        self.db.x('INSERT INTO locks VALUES(?,?,?,?,?,?,?)', ('a', 'operator', 1, time.time(),
                                                             time.time()+30, 30, ''))
        self.seq = 0

    def queue(self, action, kind='mapping'):
        state = self.fleet.operation_status('a', kind)
        self.seq += 1
        return self.fleet.request_operation('a', kind, 'operator', dict(
            action=action, epoch=1, idempotency_key=str(self.seq), session_id=state.get('session_id')))

    def worker(self, action, kind='mapping', **data):
        return self.fleet.worker_operation('a', 'worker', dict(kind=kind, action=action, **data))

    def accept(self, kind='mapping'):
        command = self.worker('poll', kind)['command']
        ref = {key: command[key] for key in ('command_id', 'session_id')}
        self.assertTrue(self.worker('accept', kind, **ref)['accepted'])
        self.assertFalse(self.worker('accept', kind, **ref)['accepted'])
        self.assertIsNone(self.worker('poll', kind)['command'])
        return ref

    def perform(self, action, kind='mapping', **evidence):
        self.queue(action, kind)
        ref = self.accept(kind)
        return self.worker('result', kind, **ref, outcome='OK', evidence=evidence)

    def start(self):
        return self.perform('START', slam_running=True, frontier_running=True)

    def test_lifecycle_and_independent_bag(self):
        self.queue('START')
        ref = self.accept()
        self.assertEqual(self.fleet.operation_status('a', 'mapping')['state'], 'STARTING')
        result = self.worker('result', **ref, outcome='OK',
                             evidence=dict(slam_running=True, frontier_running=True))
        self.assertEqual(result['state'], 'MAPPING')
        self.assertEqual(self.perform('START', 'bag', recorder_running=True)['state'], 'RECORDING')
        self.assertEqual(self.perform('PAUSE', frontier_paused=True, goal_terminal=True)['state'], 'PAUSED')
        result = self.perform('SAVE', yaml_ok=True, pgm_ok=True, posegraph_ok=False)
        self.assertEqual(result['command']['status'], 'FAILED')
        self.assertEqual(result['state'], 'PAUSED')
        self.assertFalse(result['last_save']['evidence']['posegraph_ok'])
        self.assertEqual(self.perform('SAVE', yaml_ok=True, pgm_ok=True, posegraph_ok=True)['command']['status'], 'COMPLETED')
        self.assertEqual(self.perform('RESUME', slam_running=True, frontier_running=True)['state'], 'MAPPING')
        self.assertEqual(self.perform('STOP', frontier_stopped=True, goal_terminal=True)['state'], 'STOPPED')
        self.assertEqual(self.fleet.operation_status('a', 'bag')['state'], 'RECORDING')
        self.assertEqual(self.perform('STOP', 'bag', owned_recorder_stopped=True, bag_verified=True)['state'], 'STOPPED')

    def test_idempotency_and_unknown_terminal_fence(self):
        first = self.queue('START')
        duplicate = self.fleet.request_operation('a', 'mapping', 'operator', dict(
            action='START', idempotency_key='1', epoch=1))
        self.assertEqual(duplicate['command_id'], first['command_id'])
        ref = self.accept()
        self.worker('result', **ref, outcome='OK', evidence=dict(slam_running=True, frontier_running=True))
        result = self.perform('STOP', frontier_stopped=True)
        self.assertEqual(result['state'], 'UNKNOWN')
        self.assertTrue(self.fleet.mapping_blocks_motion('a'))
        with self.assertRaises(ApiError):
            self.queue('START')

    def test_save_disconnect_and_boot_change_do_not_replay(self):
        self.start()
        self.perform('PAUSE', frontier_paused=True, goal_terminal=True)
        self.queue('SAVE')
        ref = self.accept()
        self.fleet.robots['a']['state_ts'] = time.time() - 8
        self.assertEqual(self.fleet.operation_status('a', 'mapping')['state'], 'UNKNOWN')
        self.fleet.robots['a']['state_ts'] = time.time()
        with self.assertRaises(ApiError):
            self.worker('result', **ref, outcome='OK',
                        evidence=dict(yaml_ok=True, pgm_ok=True, posegraph_ok=True))
        self.fleet.robots['a']['agent']['boot_id'] = 'second'
        self.assertEqual(self.fleet.operation_status('a', 'mapping')['state'], 'UNKNOWN')

    def test_real_and_external_goal_rejected(self):
        r = self.fleet.robots['a']
        r['state']['execution']['nav2_goals']['active_count'] = 1
        with self.assertRaises(ApiError):
            self.queue('START')
        r['state']['execution']['nav2_goals']['active_count'] = 0
        r['agent']['mode'] = 'real'
        with self.assertRaises(ApiError):
            self.queue('START')


if __name__ == '__main__':
    unittest.main()
