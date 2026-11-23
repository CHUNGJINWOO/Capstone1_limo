"""G2 (2026-11-23): origin/main Bag sessions on the integrated server (bag_operation.py) - ownership and contract rules.

One rosbag owner per Agent: an Agent that also runs the local recording_session is refused; START needs a compatible Agent
contract, a running recorder can always be stopped; the Mapping journal (kind 'mapping') and the Bag journal stay independent.
In-memory SQLite, no network, no ROS.
"""
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.db import DB
from fleet_server.service import ApiError, Fleet


class BagSessions(unittest.TestCase):
    def setUp(self):
        root = tempfile.TemporaryDirectory()
        self.addCleanup(root.cleanup)
        self.db = DB(Path(root.name) / 'fleet.sqlite')
        self.addCleanup(self.db.c.close)
        self.fleet = Fleet(self.db, SimpleNamespace(offline_after_s=3))
        self.fleet.robots['a'] = dict(reconciled=True, online=True, state_ts=time.time(),
                                     agent=dict(mode='mock', simulated=True, isolated_ros_bag=True, missions_enabled=False,
                                                boot_id='worker', protocol_version=2, safety_contract_version=1),
                                     state=dict(execution=dict(active=None, nav2_goals=dict(known=True, publishers=1, active_count=0))))
        self.db.x('INSERT INTO locks VALUES(?,?,?,?,?,?,?)', ('a', 'operator', 1, time.time(), time.time() + 30, 30, ''))
        self.n = 0

    def req(self, action, **extra):
        self.n += 1
        sid = self.fleet.bag_status('a').get('session_id')
        return self.fleet.request_bag('a', 'operator', dict(action=action, epoch=1, idempotency_key=str(self.n), session_id=sid, **extra))

    def run_cmd(self, **evidence):
        w = lambda action, **d: self.fleet.worker_operation('a', 'worker', dict(kind='bag', action=action, **d))
        c = w('poll')['command']
        ref = dict(command_id=c['command_id'], session_id=c['session_id'])
        self.assertTrue(w('accept', **ref)['accepted'])
        return w('result', **ref, outcome='OK', evidence=evidence)

    def test_start_stop_lifecycle(self):
        self.req('START')
        self.assertEqual(self.run_cmd(recorder_running=True)['state'], 'RECORDING')
        self.req('STOP')
        self.assertEqual(self.run_cmd(owned_recorder_stopped=True, bag_verified=False)['state'], 'FAILED')

    def test_agent_with_local_recording_session_is_refused(self):
        self.fleet.robots['a']['agent']['recording_enabled'] = True
        with self.assertRaises(ApiError) as cm:
            self.req('START')
        self.assertEqual(cm.exception.code, 'worker_unavailable')

    def test_real_or_mission_agent_is_refused(self):
        for change in (dict(mode='real'), dict(simulated=False), dict(missions_enabled=True), dict(isolated_ros_bag=False)):
            with self.subTest(change=change):
                self.setUp()
                self.fleet.robots['a']['agent'].update(change)
                with self.assertRaises(ApiError):
                    self.req('START')

    def test_incompatible_contract_blocks_start_but_not_stop(self):
        self.req('START')
        self.run_cmd(recorder_running=True)
        self.fleet.robots['a']['agent']['safety_contract_version'] = 2
        self.assertEqual(self.req('STOP')['status'], 'QUEUED')
        self.assertEqual(self.run_cmd(owned_recorder_stopped=True, bag_verified=True)['state'], 'STOPPED')
        with self.assertRaises(ApiError) as cm:
            self.req('START')
        self.assertEqual(cm.exception.code, 'agent_protocol')

    def test_lease_required(self):
        with self.assertRaises(ApiError) as cm:
            self.fleet.request_bag('a', 'someone-else', dict(action='START', epoch=1, idempotency_key='x'))
        self.assertEqual(cm.exception.code, 'lease_required')

    def test_bag_does_not_block_motion_or_touch_the_mapping_journal(self):
        self.req('START')
        self.run_cmd(recorder_running=True)
        self.assertFalse(self.fleet.mapping_blocks_motion('a'))
        self.assertEqual(self.fleet.operation_status('a', 'mapping')['state'], 'IDLE')
        self.assertEqual(self.fleet.operation_status('a', 'bag')['state'], 'RECORDING')


if __name__ == '__main__':
    unittest.main()
