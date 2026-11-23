"""Isolated in-memory Fleet manual-bench contract tests; no network or robot."""
import math
import threading
import time
import unittest
from types import SimpleNamespace

from fleet_server.service import ApiError, Fleet


class FakeDB:
    def __init__(self):
        self.statuses = []
    def q(self, _sql, _args):
        return [{'status': s} for s in self.statuses]
    def one(self, *_args, **_kwargs):
        return None
    def event(self, *_args, **_kwargs):
        pass


class ManualBenchContract(unittest.TestCase):
    def setUp(self):
        self.fleet = Fleet.__new__(Fleet)
        self.fleet.lock = threading.RLock()
        self.fleet.db = FakeDB()
        self.fleet.manual_benches = {}
        self.fleet.robots = {'pcu': {'agent': {'manual_web_bench': True, 'mode': 'real',
            'simulated': False, 'missions_enabled': False, 'boot_id': 'boot1',
            'protocol_version': 2, 'safety_contract_version': 1},
            'state_ts': time.time(), 'online': True, 'reconciled': True,
            'state': {'execution': {'active': None, 'nav2_goals': {'known': True, 'publishers': 1,
                'active_count': 0, 'received_status_array': True, 'age_s': 0.0}}}}}
        self.lease = {'owner': 'alice', 'epoch': 5, 'expires_at': time.time() + 30}
        self.fleet.lease_view = lambda _rid: self.lease
        self.fleet.check_session = lambda _rid, boot: self.assertEqual(boot, 'boot1')
    def call(self, actor='alice', **body):
        return self.fleet.manual_bench('pcu', actor, {'epoch': 5, **body})
    def started(self):
        return self.call(action='start')['session_id']
    def sample(self, sid, seq=1, **overrides):
        body = {'action': 'sample', 'input_format': 'normalized_axes_v1', 'session_id': sid, 'seq': seq, 'linear': 1., 'turn': -.5, 'deadman': True}
        body.update(overrides)
        return self.call(**body)
    def test_lease_owner_and_second_browser(self):
        with self.assertRaises(ApiError) as e: self.call(actor='bob', action='start')
        self.assertEqual(e.exception.code, 'lease_required')
        self.started()
        with self.assertRaises(ApiError) as e: self.call(action='start')
        self.assertEqual(e.exception.code, 'manual_bench_busy')
    def test_deadman_sample_duplicate_bad_numbers_and_expiry(self):
        sid = self.started()
        self.sample(sid)
        self.assertTrue(self.fleet.pull_manual_bench('pcu','boot1')['sample']['deadman'])
        with self.assertRaises(ApiError): self.sample(sid)
        for i, value in enumerate((math.nan, math.inf, -math.inf, 1.01, True), 2):
            with self.assertRaises(ApiError): self.sample(sid, i, linear=value)
        self.sample(sid, 10, deadman=False, linear=0., turn=0.)
        self.assertFalse(self.fleet.pull_manual_bench('pcu','boot1')['sample']['deadman'])
        self.fleet.manual_benches['pcu']['until'] = time.monotonic() - .01
        self.assertIsNone(self.fleet.pull_manual_bench('pcu','boot1')['sample'])
        with self.assertRaises(ApiError): self.sample(sid, 11)
    def test_disconnect_lease_and_server_restart_do_not_replay(self):
        sid = self.started(); self.sample(sid)
        self.lease = None
        self.assertIsNone(self.fleet.pull_manual_bench('pcu','boot1')['sample'])
        self.lease = {'owner':'alice','epoch':6,'expires_at':time.time()+30}
        self.assertIsNone(self.fleet.pull_manual_bench('pcu','boot1')['sample'])
        self.fleet.manual_benches = {}  # new Fleet process has no prior session
        self.assertIsNone(self.fleet.pull_manual_bench('pcu','boot1')['sample'])
        self.fleet.robots['pcu']['online'] = False
        with self.assertRaises(ApiError): self.fleet.pull_manual_bench('pcu','boot1')
    def test_nonterminal_fleet_mission_refused(self):
        self.fleet.db.statuses = ['EXECUTING']
        with self.assertRaises(ApiError) as e: self.started()
        self.assertEqual(e.exception.code,'robot_busy')
    def test_missing_nav_evidence_blocks_bench(self):
        self.fleet.robots['pcu']['state'] = {'operation_mode':'MAPPING','nav2':{'active_goals':1}}
        with self.assertRaises(ApiError): self.started()

if __name__ == '__main__': unittest.main(verbosity=2)
