"""Server: reconciling a changed Gate boot through the existing admin recovery (2026-11-21, no ROS).

Before this change the `gate_boot_reconciliation:<robot>` flag (set when a new Gate boot is observed) was never cleared by any code
path, so after any Gate restart every mission was refused (`safety_not_ready`) for good, and /recover answered `not_recoverable`.
"""
import json
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.db import DB  # noqa: E402
from fleet_server.service import ApiError, Fleet  # noqa: E402

ADMIN = SimpleNamespace(name='admin', role='admin')
CFG = SimpleNamespace(offline_after_s=6, command_ttl_s=120, resend_after_s=1)


class GateBootReconcile(unittest.TestCase):
    def setUp(self):
        self.db = DB(':memory:'); self.f = Fleet(self.db, CFG)
        self.f.robots['R'] = {'robot_id': 'R', 'online': True, 'reconciled': True, 'state_ts': time.time(), 'last_seen': time.time(),
                              'first_seen': time.time(), 'map_version': None, 'map_meta': None,
                              'agent': {'boot_id': 'agentboot1', 'protocol_version': 2, 'safety_contract_version': 1},
                              'state': {'safety_gate': {'status': 'NORMAL', 'boot_id': 'gateboot2', 'fault_latched': False, 'cmd_blocked': False},
                                        'execution': {'active': None, 'nav2_goals': {'active_count': 0}},
                                        'motion_watchdog': {'required': True, 'state': 'MOTION_AUTHORIZED', 'agent_boot_matches': True}}}
        self.db.x("INSERT INTO meta(k,v) VALUES('observed_gate_boot:R','gateboot1')")
        self.db.x("INSERT INTO meta(k,v) VALUES('gate_boot_reconciliation:R','gate boot changed; reconciliation required')")

    def flag(self): return self.db.one("SELECT 1 FROM meta WHERE k='gate_boot_reconciliation:R'") is not None
    def events(self, kind): return [json.loads(e['data_json'] or '{}') for e in self.db.q('SELECT * FROM events WHERE kind=?', (kind,))]

    def test_flag_blocks_safety_context_and_reports_reason(self):
        self.assertIsNone(self.f.safety_context('R'))
        self.assertEqual(self.f.safety_context_reasons['R']['reason'], 'GATE_BOOT_RECONCILIATION')
        self.assertTrue(self.f.robot_view(self.f.robots['R'])['gate_boot_reconciliation'])

    def test_recover_now_reaches_gate_boot_path_and_needs_a_note(self):
        with self.assertRaises(ApiError) as e: self.f.recover_robot_block('R', ADMIN, 'short')
        self.assertEqual(e.exception.code, 'note_required')

    def test_refused_without_evidence_and_audited(self):
        st = self.f.robots['R']['state']
        for change, why in ((lambda: st['safety_gate'].update(status='FAILSAFE_SENSOR_TIMEOUT'), 'gate_not_normal'),
                            (lambda: st['safety_gate'].update(fault_latched=True), 'gate_not_normal'),
                            (lambda: st['execution'].update(active={'goal_uuid': 'x'}), 'goal_still_active'),
                            (lambda: st['execution'].update(nav2_goals={'active_count': 1}), 'goal_still_active'),
                            (lambda: st['motion_watchdog'].update(state='LOCKED'), 'watchdog_not_authorized'),
                            (lambda: self.f.robots['R'].update(state_ts=time.time() - 9), 'stale_state'),
                            (lambda: st['safety_gate'].update(boot_id=None), 'gate_boot_unknown')):
            self.setUp(); st = self.f.robots['R']['state']; change()
            with self.assertRaises(ApiError) as e: self.f.recover_robot_block('R', ADMIN, 'checked gate restart in the isolated sim')
            self.assertEqual(e.exception.code, why); self.assertTrue(self.flag())
            self.assertEqual(len(self.events('gate_boot_reconcile_refused')), 1)

    def test_reconciled_with_evidence_adopts_new_boot(self):
        v = self.f.recover_robot_block('R', ADMIN, 'checked gate restart in the isolated sim')
        self.assertFalse(self.flag()); self.assertFalse(v['gate_boot_reconciliation'])
        self.assertEqual(self.db.one("SELECT v FROM meta WHERE k='observed_gate_boot:R'")['v'], 'gateboot2')
        ev = self.events('gate_boot_reconciled'); self.assertEqual(len(ev), 1)
        self.assertEqual((ev[0]['previous_gate_boot'], ev[0]['current_gate_boot']), ('gateboot1', 'gateboot2'))
        self.f.safety_context('R')
        self.assertNotEqual(self.f.safety_context_reasons['R']['reason'], 'GATE_BOOT_RECONCILIATION')   # the flag no longer blocks

    def test_no_flag_no_block_is_still_not_recoverable(self):
        self.db.x("DELETE FROM meta WHERE k='gate_boot_reconciliation:R'")
        with self.assertRaises(ApiError) as e: self.f.recover_robot_block('R', ADMIN, 'nothing to recover here at all')
        self.assertEqual(e.exception.code, 'not_recoverable')


if __name__ == '__main__':
    unittest.main(verbosity=2)
