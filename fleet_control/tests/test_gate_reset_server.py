"""Server side of the admin web Gate reset (no ROS; a fake Agent report stands in for the robot)."""
import json
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.db import DB  # noqa: E402
from fleet_server import gate_reset as gr  # noqa: E402
from fleet_server.service import ApiError, Fleet  # noqa: E402

ADMIN = SimpleNamespace(name='admin', role='admin')
OPER = SimpleNamespace(name='operator', role='operator')
VIEW = SimpleNamespace(name='viewer', role='viewer')
AGENT = SimpleNamespace(name='agent-R', role='agent', robot_id='R')
CFG = SimpleNamespace(offline_after_s=6, command_ttl_s=120, resend_after_s=1, require_operating_mode=True)


def gate_view(state='FAILSAFE_SENSOR_TIMEOUT', epoch=3, sticky='NOT_EXCLUSIVE'):
    return {'state': state, 'boot_id': 'gateboot', 'fault_epoch': epoch, 'fault_latched': state != 'NORMAL',
            'fault_reasons': [], 'last_fault_reasons': ['cmd_input:NOT_EXCLUSIVE'],
            'cmd_input': {'state': sticky or 'BOUND', 'sticky': sticky, 'publishers': ['/fleet_motion_watchdog:010f']},
            'reset_readiness': {'enforced': True, 'ready': True, 'blockers': []}}


class GateResetServer(unittest.TestCase):
    def make(self, db=None):
        f = Fleet(db or self.db, CFG)
        f.robots['R'] = {'robot_id': 'R', 'online': True, 'reconciled': True, 'state_ts': time.time(),
                         'agent': {'boot_id': 'agentboot1', 'protocol_version': 2, 'safety_contract_version': 1, 'gate_reset': True, 'mode': 'mock'},
                         'state': {'safety_gate': {'status': 'FAILSAFE_SENSOR_TIMEOUT', 'reset_view': gate_view()},
                                   'mapping_worker': {'status': 'IDLE'},
                                   'motion_watchdog': {'state': 'MOTION_AUTHORIZED', 'agent_boot_matches': True}}}
        return f

    def setUp(self):
        self.db = DB(':memory:')
        self.f = self.make()
        self.epoch = self.f.lease_action('R', ADMIN, 'acquire', 120)['epoch']
        self.assertEqual(self.f.mode_view('R')['mode'], 'IDLE')

    def body(self, **kw):
        b = {'epoch': self.epoch, 'expected_gate_boot_id': 'gateboot', 'expected_fault_epoch': 3, 'note': 'test'}
        b.update(kw); return b

    def events(self, kind):
        return [json.loads(e['data_json'] or '{}') for e in self.db.q('SELECT * FROM events WHERE kind=? ORDER BY id', (kind,))]

    def refused(self, principal, body=None):
        with self.assertRaises(ApiError) as e:
            self.f.request_gate_reset('R', principal, body or self.body())
        return e.exception

    # ------------------------------------------------------------ role and server preconditions
    def test_non_admin_roles_refused_and_audited(self):
        for p in (OPER, VIEW, AGENT, None):
            e = self.refused(p); self.assertEqual(e.status, 403)
        ev = self.events('gate_reset_refused'); self.assertEqual(len(ev), 4); self.assertTrue(all(x['reason'] == 'ROLE_NOT_ADMIN' for x in ev))
        self.assertIsNone(self.f._gr_load('R'))

    def test_admin_without_lease_or_wrong_epoch(self):
        self.f.lease_action('R', ADMIN, 'release')
        self.assertIn('LEASE_NOT_HELD_BY_CALLER', self.refused(ADMIN).message)
        self.epoch = self.f.lease_action('R', ADMIN, 'acquire', 120)['epoch']
        self.assertIn('LEASE_EPOCH_MISMATCH', self.refused(ADMIN, self.body(epoch=self.epoch + 99)).message)

    def test_gate_normal_or_changed_since_viewed(self):
        self.assertIn('GATE_CHANGED_SINCE_VIEWED', self.refused(ADMIN, self.body(expected_fault_epoch=2)).message)
        self.f.robots['R']['state']['safety_gate']['reset_view'] = gate_view('NORMAL', sticky=None) | {'fault_latched': False}
        self.assertIn('GATE_NOT_FAULTED', self.refused(ADMIN).message)

    def test_stale_report_offline_unreconciled_unsupported(self):
        r = self.f.robots['R']
        r['state']['motion_watchdog'] = {'state': 'LOCKED', 'agent_boot_matches': True}
        self.assertIn('WATCHDOG_NOT_AUTHORIZED:LOCKED', self.refused(ADMIN).message)
        r['state']['motion_watchdog'] = {'state': 'MOTION_AUTHORIZED', 'agent_boot_matches': True}
        r['state_ts'] = time.time() - 10; self.assertIn('GATE_REPORT_STALE', self.refused(ADMIN).message); r['state_ts'] = time.time()
        r['online'] = False; self.assertIn('AGENT_OFFLINE', self.refused(ADMIN).message); r['online'] = True
        r['reconciliation_required'] = True; self.assertIn('AGENT_NOT_RECONCILED', self.refused(ADMIN).message); r['reconciliation_required'] = False
        r['agent']['gate_reset'] = False; self.assertIn('AGENT_GATE_RESET_UNSUPPORTED', self.refused(ADMIN).message)

    def test_mode_mission_manual_mapping_block(self):
        rec = self.f.mode_view('R'); rec['mode'] = 'MANUAL'
        self.f.mode_view = lambda rid, advance=True: dict(rec)
        self.assertIn('MODE_NOT_IDLE:MANUAL', self.refused(ADMIN).message)
        rec['mode'] = 'IDLE'; rec['transition'] = {'state': 'VERIFYING_IDLE'}
        self.assertIn('MODE_TRANSITION:VERIFYING_IDLE', self.refused(ADMIN).message)
        rec['transition'] = None
        self.db.x("INSERT INTO missions(mission_id,robot_id,type,goal_json,status) VALUES('m_000000000001','R','navigate_to_pose','{}','EXECUTING')")
        self.assertIn('MISSION_ACTIVE', self.refused(ADMIN).message)
        self.db.x("UPDATE missions SET status='CANCELED'")
        self.f.manual_benches['R'] = {'id': 'x'}
        self.assertIn('MANUAL_SESSION_ACTIVE', self.refused(ADMIN).message)

    # ------------------------------------------------------------ queue, delivery, result
    def test_happy_path_delivered_once_then_result_and_idle_kept(self):
        q = self.f.request_gate_reset('R', ADMIN, self.body())
        self.assertEqual(q['status'], 'QUEUED')
        d = self.f.gate_reset_for_agent('R')
        self.assertEqual(d['agent_boot_id'], 'agentboot1'); self.assertEqual(d['fault_epoch'], 3); self.assertEqual(d['lease_epoch'], self.epoch)
        self.assertIsNone(self.f.gate_reset_for_agent('R'))                      # never delivered twice
        self.f.manual_benches['R'] = {'id': 'stale'}
        self.f.gate_reset_ingest('R', {'gate_reset': {'last': {'request_id': d['request_id'], 'status': 'SUCCEEDED', 'gate_message': 'Latch cleared',
                                                               'checks': {'watchdog': {'ok': True}}}}})
        rec = self.f._gr_load('R'); self.assertEqual(rec['status'], 'SUCCEEDED')
        self.assertNotIn('R', self.f.manual_benches)                             # nothing resumes
        self.assertEqual(self.f.mode_view('R')['mode'], 'IDLE')                  # mode unchanged
        for k in ('gate_reset_requested', 'gate_reset_delivered', 'gate_reset_result'):
            self.assertEqual(len(self.events(k)), 1, k)
        self.assertEqual(self.events('gate_reset_requested')[0]['actor'], 'admin')

    def test_same_fault_epoch_cannot_be_reset_twice(self):
        d = (self.f.request_gate_reset('R', ADMIN, self.body()), self.f.gate_reset_for_agent('R'))[1]
        self.f.gate_reset_ingest('R', {'gate_reset': {'last': {'request_id': d['request_id'], 'status': 'SUCCEEDED'}}})
        self.assertIn('FAULT_EPOCH_ALREADY_RESET', self.refused(ADMIN).message)     # view still shows the old fault (report lag)
        self.f.robots['R']['state']['safety_gate']['reset_view'] = gate_view(epoch=4)    # a new fault -> new epoch -> allowed
        self.assertEqual(self.f.request_gate_reset('R', ADMIN, self.body(expected_fault_epoch=4))['status'], 'QUEUED')

    def test_refused_result_records_failed_checks(self):
        d = (self.f.request_gate_reset('R', ADMIN, self.body()), self.f.gate_reset_for_agent('R'))[1]
        self.f.gate_reset_ingest('R', {'gate_reset': {'last': {'request_id': d['request_id'], 'status': 'REFUSED', 'reason': 'AGENT_PRECHECK:publisher_identity',
                                                               'checks': {'publisher_identity': {'ok': False}, 'watchdog': {'ok': True}}}}})
        rec = self.f._gr_load('R'); self.assertEqual(rec['status'], 'REFUSED'); self.assertEqual(rec['failed_checks'], ['publisher_identity'])
        self.assertEqual(self.events('gate_reset_result')[0]['status'], 'REFUSED')

    def test_result_replay_and_foreign_ids_ignored(self):
        d1 = (self.f.request_gate_reset('R', ADMIN, self.body()), self.f.gate_reset_for_agent('R'))[1]
        self.f.gate_reset_ingest('R', {'gate_reset': {'last': {'request_id': 'not-this-one', 'status': 'SUCCEEDED'}}})
        self.assertEqual(self.f._gr_load('R')['status'], 'DELIVERED')
        ok = {'gate_reset': {'last': {'request_id': d1['request_id'], 'status': 'REFUSED', 'reason': 'GATE:REFUSED: INPUT_WRITERS:2'}}}
        self.f.gate_reset_ingest('R', ok); self.f.gate_reset_ingest('R', ok)    # repeated report of the same result
        self.assertEqual(len(self.events('gate_reset_result')), 1)
        # a later request; the OLD result arriving again must not complete the new one
        self.f.robots['R']['state']['safety_gate']['reset_view'] = gate_view(epoch=4)
        self.f.request_gate_reset('R', ADMIN, self.body(expected_fault_epoch=4)); self.f.gate_reset_for_agent('R')
        self.f.gate_reset_ingest('R', {'gate_reset': {'last': {'request_id': d1['request_id'], 'status': 'SUCCEEDED'}}})
        self.assertEqual(self.f._gr_load('R')['status'], 'DELIVERED')

    def test_single_request_in_flight_and_concurrent_requests(self):
        self.f.request_gate_reset('R', ADMIN, self.body())
        e = self.refused(ADMIN); self.assertEqual(e.code, 'gate_reset_in_progress')
        # fresh Fleet: 8 concurrent admin requests -> exactly one is queued
        db = DB(':memory:'); f = self.make(db); ep = f.lease_action('R', ADMIN, 'acquire', 120)['epoch']; f.mode_view('R')
        out = []; barrier = threading.Barrier(8)
        def go():
            barrier.wait()
            try: out.append(f.request_gate_reset('R', ADMIN, {'epoch': ep, 'expected_gate_boot_id': 'gateboot', 'expected_fault_epoch': 3})['status'])
            except ApiError as x: out.append(x.code)
        ts = [threading.Thread(target=go) for _ in range(8)]; [t.start() for t in ts]; [t.join() for t in ts]
        self.assertEqual(out.count('QUEUED'), 1); self.assertEqual(out.count('gate_reset_in_progress'), 7)

    def test_server_restart_abandons_pending_request(self):
        self.f.request_gate_reset('R', ADMIN, self.body())
        f2 = self.make(self.db)                                                   # same DB, new server boot id
        self.assertNotEqual(f2.boot_id, self.f.boot_id)
        self.assertIsNone(f2.gate_reset_for_agent('R'))
        self.assertEqual(f2._gr_load('R')['status'], 'ABANDONED_SERVER_RESTART')

    def test_agent_restart_abandons_pending_request(self):
        self.f.request_gate_reset('R', ADMIN, self.body())
        self.f.robots['R']['agent']['boot_id'] = 'agentboot2'
        self.assertIsNone(self.f.gate_reset_for_agent('R'))
        self.assertEqual(self.f._gr_load('R')['status'], 'ABANDONED_AGENT_RESTART')

    def test_expiry_without_delivery_and_without_result(self):
        old = gr.QUEUE_TTL_S, gr.RESULT_TTL_S
        try:
            gr.QUEUE_TTL_S = 0.05; self.f.request_gate_reset('R', ADMIN, self.body()); time.sleep(.08)
            self.assertIsNone(self.f.gate_reset_for_agent('R')); self.assertEqual(self.f._gr_load('R')['status'], 'EXPIRED')
            gr.QUEUE_TTL_S, gr.RESULT_TTL_S = 10, 0.05
            self.f.request_gate_reset('R', ADMIN, self.body()); self.f.gate_reset_for_agent('R'); time.sleep(.08)
            self.assertEqual(self.f.gate_reset_view('R', ADMIN)['request']['status'], 'EXPIRED_NO_RESULT')
        finally:
            gr.QUEUE_TTL_S, gr.RESULT_TTL_S = old

    def test_view_shows_blockers_per_role(self):
        self.assertEqual(self.f.gate_reset_view('R', ADMIN)['server_blockers'], [])
        self.assertIn('ROLE_NOT_ADMIN', self.f.gate_reset_view('R', VIEW)['server_blockers'])
        v = self.f.gate_reset_view('R', ADMIN)['gate']['view']; self.assertTrue(v['reset_readiness']['ready'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
