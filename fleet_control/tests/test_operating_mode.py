"""Server-side operating-mode transitions (no ROS): a fake Agent state stands in for the robot.

Covers: legacy pass-through, Lease checks, IDLE->MANUAL steps (verify hold, Agent confirmation), blockers and timeout
(FAILED -> IDLE, no auto retry), manual session dropped on leaving MANUAL, mapping UNKNOWN -> FAULT, mapping STOP issued
by the transition, missions canceled by the transition, Lease loss and server restart invalidation, busy rejections.
"""
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.db import DB  # noqa: E402
from fleet_server import operating_mode as om  # noqa: E402
from fleet_server.service import ApiError, Fleet  # noqa: E402

OP = SimpleNamespace(name='operator', role='operator')
BLOCKER = {'IDLE': 'MODE_IDLE', 'MANUAL': None, 'MAPPING': 'NO_OWNED_GOAL', 'NAVIGATION': 'NO_OWNED_GOAL', 'TRANSITIONING': 'MODE_TRANSITIONING'}
CFG = SimpleNamespace(offline_after_s=6, command_ttl_s=120, resend_after_s=1)


def idle_state(arb_mode='IDLE'):
    # velocity_arbiter blocker values mirror the real selector in each mode (MODE_IDLE / ready / NO_OWNED_GOAL)
    return {'execution': {'active': None, 'nav2_goals': {'known': True, 'publishers': 1, 'active_count': 0, 'foreign_active_goal_uuids': []},
                          'nav_idle': {'ok': True, 'reason': None}},
            'motion_watchdog': {'required': True, 'state': 'MOTION_AUTHORIZED', 'agent_boot_matches': True},
            'velocity_arbiter': {'operating_mode': arb_mode, 'selected_linear': 0.0, 'selected_turn': 0.0, 'blocker': BLOCKER[arb_mode]},
            'mapping_worker': {'enabled': True, 'status': 'IDLE', 'processes_started': False}}


class ModeTests(unittest.TestCase):
    def setUp(self):
        om.VERIFY_HOLD_S = 0.05
        self.db = DB(':memory:')
        self.f = Fleet(self.db, CFG)
        self.boot = 'b' * 12
        self.f.robots['R'] = {'robot_id': 'R', 'online': True, 'reconciled': True, 'state_ts': time.time(),
                              'agent': {'boot_id': self.boot, 'protocol_version': 2, 'safety_contract_version': 1, 'mode': 'mock', 'simulated': True, 'mapping_worker': True,
                                        'manual_web_bench': True, 'missions_enabled': True, 'locked_velocity_arbiter': True},
                              'state': idle_state()}
        self.epoch = self.f.lease_action('R', OP, 'acquire', 60)['epoch']
        self.n = 0

    def report(self, **changes):
        st = self.f.robots['R']['state']
        for k, v in changes.items():
            st[k] = v
        time.sleep(0.002)                     # a new report is strictly newer than the previous step
        self.f.robots['R']['state_ts'] = time.time()
        return self.f.mode_view('R')

    def request(self, target):
        self.n += 1
        return self.f.request_mode('R', 'operator', {'target': target, 'epoch': self.epoch, 'idempotency_key': f'k{self.n}'})

    def drive_to(self, target, arb=None):
        self.request(target)
        for _ in range(40):
            m = arb or self.f.mode_view('R', advance=False)['mode']
            rec = self.report(velocity_arbiter={**self.f.robots['R']['state']['velocity_arbiter'], 'operating_mode': m,
                                                'blocker': BLOCKER.get(m, 'MODE_' + m)})
            if rec['transition']['state'] in ('ACTIVE', 'FAILED'):
                return rec
            time.sleep(0.02)
        return self.f.mode_view('R')

    def test_legacy_robot_passes_and_strict_config_starts_idle(self):
        self.assertIsNone(self.f.mode_view('R'))
        self.f.require_mode('R', 'MANUAL', 'x')                      # legacy: no record -> allowed
        strict = Fleet(DB(':memory:'), SimpleNamespace(**vars(CFG), require_operating_mode=True))
        strict.robots['R'] = dict(self.f.robots['R'])
        with self.assertRaises(ApiError):
            strict.require_mode('R', 'MANUAL', 'mode_not_manual')

    def test_lease_required(self):
        with self.assertRaises(ApiError) as e:
            self.f.request_mode('R', 'someone', {'target': 'MANUAL', 'epoch': self.epoch, 'idempotency_key': 'x'})
        self.assertEqual(e.exception.code, 'lease_required')

    def test_idle_to_manual_needs_agent_confirmation(self):
        self.request('MANUAL')
        rec = self.report(); rec = self.report()
        time.sleep(0.06); rec = self.report()
        self.assertEqual(rec['transition']['state'], 'CONFIRMING')
        with self.assertRaises(ApiError):                             # commands wait for ACTIVE
            self.f.require_mode('R', 'MANUAL', 'mode_not_manual')
        rec = self.report(velocity_arbiter={**idle_state('MANUAL')['velocity_arbiter']})
        self.assertEqual((rec['mode'], rec['transition']['state']), ('MANUAL', 'ACTIVE'))
        self.f.require_mode('R', 'MANUAL', 'mode_not_manual')
        with self.assertRaises(ApiError):
            self.f.require_mode('R', 'MAPPING', 'mode_not_mapping')

    def test_unproven_idle_times_out_to_idle_without_retry(self):
        self.f.robots['R']['state']['execution']['nav_idle'] = {'ok': False, 'reason': 'NAV2_SERVER_MISSING'}
        self.request('MANUAL')
        rec = self.report()
        self.assertEqual(rec['transition']['state'], 'VERIFYING_IDLE')
        self.assertIn('NAV2_SERVER_MISSING', rec['transition']['blocker'])
        rec['transition']['deadline'] = time.time() - 1
        self.f._mode_save('R', rec)
        rec = self.report()
        # 2026-11-20 (Codex review): an unproven stop is FAULT, never IDLE (previous expectation 'IDLE' in before_fix/)
        self.assertEqual((rec['mode'], rec['transition']['state']), ('FAULT', 'FAILED'))
        self.f.robots['R']['state']['execution']['nav_idle'] = {'ok': True}
        self.assertEqual(self.report()['mode'], 'FAULT')              # no automatic re-attempt

    def test_each_blocker_is_reported(self):
        cases = [('execution', {'active': {'mission_id': 'm'}, 'nav2_goals': {'active_count': 0}, 'nav_idle': {'ok': True}}, 'owns a Nav2 goal'),
                 ('motion_watchdog', {'required': True, 'state': 'FAILSAFE', 'agent_boot_matches': True}, 'watchdog'),
                 ('velocity_arbiter', {'operating_mode': 'IDLE', 'selected_linear': 0.1, 'selected_turn': 0.0}, 'arbiter output'),
                 ('mapping_worker', {'enabled': True, 'status': 'UNKNOWN', 'processes_started': True}, 'mapping worker')]
        for key, value, text in cases:
            self.f.robots['R']['state'] = idle_state(); self.f.robots['R']['state'][key] = value
            self.assertIn(text, self.f._mode_idle_blocker('R', {'stopped': {'missions': []}}))

    def test_leaving_manual_drops_session(self):
        self.drive_to('MANUAL')
        self.f.manual_benches['R'] = {'id': 's', 'epoch': self.epoch, 'boot': self.boot, 'seq': 1, 'until': time.monotonic() + 10, 'sample': None}
        self.request('IDLE')
        self.assertNotIn('R', self.f.manual_benches)

    def test_mapping_unknown_makes_fault(self):
        self.drive_to('MAPPING')
        self.f._write_operation({'robot_id': 'R', 'kind': 'mapping', 'session_id': 's' * 32, 'state': 'UNKNOWN', 'boot': self.boot,
                                 'server_boot': self.f.boot_id, 'owner': 'operator', 'epoch': self.epoch,
                                 'command': {'status': 'UNKNOWN', 'action': 'START', 'expires_at': 0}, 'result': None})
        rec = self.drive_to('MANUAL')
        self.assertEqual((rec['mode'], rec['transition']['state']), ('FAULT', 'FAILED'))
        self.assertIn('UNKNOWN', rec['transition']['reason'])

    def test_transition_stops_mapping_and_waits_for_goal_terminal_evidence(self):
        self.drive_to('MAPPING')
        self.f.require_mode('R', 'MAPPING', 'x')
        out = self.f.request_operation('R', 'operator', {'action': 'START', 'epoch': self.epoch, 'idempotency_key': 's1'})
        def complete(evidence):
            w = lambda a, **k: self.f.worker_operation('R', self.boot, dict(kind='mapping', action=a, **k))
            c = w('poll')['command']; ref = dict(session_id=c['session_id'], command_id=c['command_id'])
            w('accept', **ref); return w('result', **ref, outcome='OK', evidence=evidence)
        complete(dict(slam_running=True, frontier_running=True))
        self.request('MANUAL')
        s = self.f.operation_status('R')
        self.assertEqual((s['state'], s['command']['action']), ('STOPPING', 'STOP'))
        with self.assertRaises(ApiError):
            self.request('IDLE')                                      # transition_in_progress
        complete(dict(frontier_stopped=True, goal_terminal=True))
        for _ in range(20):
            rec = self.report(velocity_arbiter=idle_state('MANUAL')['velocity_arbiter']); time.sleep(0.02)
        self.assertEqual((rec['mode'], rec['transition']['state']), ('MANUAL', 'ACTIVE'))

    def test_transition_cancels_missions(self):
        self.db.x("INSERT INTO missions(mission_id,robot_id,type,goal_json,idempotency_key,status,detail,created_by,note,created_at,updated_at) "
                  "VALUES('m_000000000001','R','navigate_to_pose','{}',NULL,'EXECUTING','x','operator','',?,?)", (time.time(), time.time()))
        self.request('MANUAL')
        rec = self.report()
        self.assertEqual(rec['transition']['state'], 'STOPPING_PREVIOUS')
        self.assertEqual(self.db.one("SELECT status FROM missions WHERE mission_id='m_000000000001'")['status'], 'CANCELING')
        self.db.x("UPDATE missions SET status='CANCELED' WHERE mission_id='m_000000000001'")
        rec = self.report()
        self.assertEqual(rec['transition']['state'], 'VERIFYING_IDLE')

    def test_lease_loss_and_server_restart_invalidate(self):
        self.drive_to('MANUAL')
        self.f.lease_action('R', OP, 'release')
        rec = self.f.mode_view('R')
        self.assertEqual((rec['mode'], rec['reason']), ('IDLE', 'lease expired or changed'))
        self.epoch = self.f.lease_action('R', OP, 'acquire', 60)['epoch']
        self.drive_to('MANUAL')
        f2 = Fleet(self.db, CFG); f2.robots['R'] = self.f.robots['R']
        self.assertEqual(f2.mode_view('R')['mode'], 'IDLE')

    def test_mapping_busy_rejected(self):
        self.drive_to('MAPPING')
        self.f.request_operation('R', 'operator', {'action': 'START', 'epoch': self.epoch, 'idempotency_key': 's1'})
        with self.assertRaises(ApiError) as e:
            self.request('MANUAL')
        self.assertEqual(e.exception.code, 'mapping_busy')


if __name__ == '__main__':
    unittest.main(verbosity=1)
