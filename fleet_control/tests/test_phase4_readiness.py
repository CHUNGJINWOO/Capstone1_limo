"""Phase 4 server side (no ROS): supervisor reports, READY/DEGRADED/FAULT, READY gate on MAPPING/NAVIGATION, admin-only actions."""
import sys, time, unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.db import DB  # noqa: E402
from fleet_server.service import ApiError, Fleet  # noqa: E402

ADMIN = SimpleNamespace(name='admin', role='admin')
OPER = SimpleNamespace(name='operator', role='operator')
VIEW = SimpleNamespace(name='viewer', role='viewer')


def cfg(require=True):
    return SimpleNamespace(offline_after_s=6, command_ttl_s=120, resend_after_s=1, require_operating_mode=True, require_supervisor_ready=require)


def services(**states):
    base = {'stack': 'READY', 'watchdog': 'READY', 'agent': 'READY'}
    base.update(states)
    return [{'name': n, 'state': s, 'restarts': 0, 'required': True, 'actions': ['start', 'stop', 'restart', 'retry']} for n, s in base.items()]


GATE_OK = {'status': 'NORMAL', 'fault_latched': False, 'age_s': 0.1, 'rx_fresh': True, 'cmd_blocked': False, 'movement_permitted': True}
WD_OK = {'state': 'MOTION_AUTHORIZED', 'age_s': 0.1}
ROS_OK = {'topics': {'/scan': {'age_s': 0.1, 'max_age_s': 1.0}, '/odom': {'age_s': 0.05, 'max_age_s': 1.0}}}


class Readiness(unittest.TestCase):
    def setUp(self):
        self.db = DB(':memory:'); self.f = Fleet(self.db, cfg())
        self.f.robots['R'] = {'robot_id': 'R', 'online': True, 'reconciled': True, 'state_ts': time.time(), 'last_seen': time.time(),
                              'agent': {'boot_id': 'a1', 'protocol_version': 2, 'safety_contract_version': 1},
                              'state': {'safety_gate': dict(GATE_OK), 'motion_watchdog': dict(WD_OK)}}
        self.epoch = self.f.lease_action('R', OPER, 'acquire', 120)['epoch']

    def report(self, boot='s1', ros=ROS_OK, **states):
        return self.f.supervisor_report('R', {'boot_id': boot, 'services': services(**states), 'ros': ros})

    def mode(self, target):
        return self.f.request_mode('R', 'operator', {'target': target, 'epoch': self.epoch, 'idempotency_key': f'k{time.monotonic_ns()}'})

    def passes_ready_gate(self, target):
        """True if the request got past the READY gate (it may still be refused later by an unrelated capability check)."""
        try: self.mode(target); return True
        except ApiError as e: return e.code != 'robot_not_ready'

    def test_no_supervisor_is_fault_when_required_and_blocks_mapping_and_navigation(self):
        rd = self.f.readiness('R'); self.assertEqual(rd['level'], 'FAULT'); self.assertIn('SUPERVISOR_NOT_REPORTING', rd['reasons'])
        for t in ('MAPPING', 'NAVIGATION'):
            with self.assertRaises(ApiError) as e: self.mode(t)
            self.assertEqual(e.exception.code, 'robot_not_ready')

    def test_ready_allows_mode_request(self):
        self.report(); rd = self.f.readiness('R'); self.assertEqual(rd['level'], 'READY', rd['reasons'])
        self.assertTrue(self.passes_ready_gate('MAPPING')); self.assertTrue(self.passes_ready_gate('NAVIGATION'))

    def test_manual_and_idle_never_need_ready(self):
        self.assertTrue(self.passes_ready_gate('MANUAL'))      # no supervisor: MANUAL is still decided by the existing chain

    def test_degraded_and_fault_reasons(self):
        self.report(stack='STARTING'); self.assertEqual(self.f.readiness('R')['level'], 'DEGRADED')
        self.report(watchdog='FAILED'); rd = self.f.readiness('R'); self.assertEqual(rd['level'], 'FAULT'); self.assertIn('SERVICE_FAILED:watchdog', rd['reasons'])
        self.report(ros={'topics': {'/scan': {'age_s': 3.0, 'max_age_s': 1.0}}}); self.assertIn('STALE:/scan', self.f.readiness('R')['reasons'])
        self.report(); self.f.robots['R']['state']['safety_gate'] = {'status': 'FAILSAFE_SENSOR_TIMEOUT', 'fault_latched': True}
        rd = self.f.readiness('R'); self.assertEqual(rd['level'], 'FAULT'); self.assertIn('GATE_FAULT:FAILSAFE_SENSOR_TIMEOUT', rd['reasons'])
        with self.assertRaises(ApiError): self.mode('NAVIGATION')

    def test_nav2_server_missing_is_not_ready(self):
        self.report(); self.f.robots['R']['state']['execution'] = {'nav_idle': {'ok': False, 'navigate_to_pose': {'ok': False, 'reason': 'NAV2_SERVER_MISSING'}}}
        rd = self.f.readiness('R'); self.assertEqual(rd['level'], 'DEGRADED'); self.assertIn('NAV2:NAV2_SERVER_MISSING', rd['reasons'])
        self.f.robots['R']['state']['execution'] = {'nav_idle': {'ok': False, 'navigate_to_pose': {'ok': False, 'reason': 'NAV2_GOAL_ACTIVE'}}}
        self.assertEqual(self.f.readiness('R')['level'], 'READY')               # an active goal is not a readiness problem

    def test_supervisor_stale_and_agent_offline(self):
        self.report(); self.f.supervisors['R']['received_at'] -= 10
        self.assertIn('SUPERVISOR_STALE', self.f.readiness('R')['reasons'])
        self.report(); self.f.robots['R']['online'] = False
        rd = self.f.readiness('R'); self.assertEqual(rd['level'], 'FAULT'); self.assertIn('AGENT_OFFLINE', rd['reasons'])

    def test_not_required_flag_keeps_legacy_behaviour(self):
        f = Fleet(DB(':memory:'), cfg(False)); f.robots['R'] = dict(self.f.robots['R'])
        self.assertEqual(f.readiness('R')['level'], 'DEGRADED')     # informational only
        f.require_ready('R', 'x')                                     # no exception

    def admin_lease(self):
        self.f.lease_action('R', OPER, 'release'); self.aep = self.f.lease_action('R', ADMIN, 'acquire', 120)['epoch']; return self.aep

    def act(self, who, **body):
        return self.f.request_service_action('R', who, dict(body, epoch=getattr(self, 'aep', None)))

    def test_service_actions_admin_only_allowlisted_and_idle_only(self):
        self.report(); self.admin_lease()
        for p in (OPER, VIEW):
            with self.assertRaises(ApiError) as e: self.act(p, service='watchdog', action='restart')
            self.assertEqual(e.exception.status, 403)
        with self.assertRaises(ApiError) as e: self.act(ADMIN, service='watchdog', action='exec', cmd='rm -rf /')
        self.assertEqual(e.exception.status, 400)
        with self.assertRaises(ApiError) as e: self.act(ADMIN, service='bash', action='start')
        self.assertEqual(e.exception.code, 'unknown_service')
        a = self.act(ADMIN, service='watchdog', action='restart')
        self.assertEqual(a['status'], 'QUEUED')
        with self.assertRaises(ApiError) as e: self.act(ADMIN, service='agent', action='restart')
        self.assertEqual(e.exception.code, 'action_pending')
        resp = self.report(); self.assertEqual([x['action'] for x in resp['actions']], ['restart'])     # delivered once
        self.assertEqual(self.report()['actions'], [])
        self.f.supervisor_report('R', {'boot_id': 's1', 'services': services(), 'action_results': [{'action_id': a['action_id'], 'ok': True, 'detail': 'restarting'}]})
        self.assertEqual(self.f.readiness('R')['actions'][-1]['status'], 'DONE')

    def test_service_action_refused_while_busy(self):
        self.report(); ep = self.admin_lease()
        for m in ('MANUAL', 'MAPPING', 'NAVIGATION'):
            self.f._mode_save('R', dict(self.f._mode_new(m, None, 'test'), transition=None, epoch=ep, owner='admin', agent_boot='a1'))
            with self.assertRaises(ApiError) as e: self.act(ADMIN, service='watchdog', action='stop')
            self.assertEqual(e.exception.code, 'robot_busy', m)

    def test_new_supervisor_boot_expires_queued_actions_and_logs(self):
        self.report(); self.admin_lease(); a = self.act(ADMIN, service='watchdog', action='restart')
        self.report(boot='s2')
        self.assertEqual(self.f.service_actions['R'][-1]['status'], 'EXPIRED')
        kinds = [e['kind'] for e in self.db.q('SELECT kind FROM events')]
        self.assertGreaterEqual(kinds.count('supervisor_boot'), 2)


if __name__ == '__main__':
    unittest.main()
