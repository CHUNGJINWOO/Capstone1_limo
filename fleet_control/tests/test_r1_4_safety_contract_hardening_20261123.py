"""R1.4 Agent-Server safety contract hardening tests (2026-11-22).

Server + pure-Python Agent (`safety_protocol`) unit tests; no ROS, no sockets, no hardware.
Covers:
  - strict `agent_contract_valid` (exact int protocol_version=2, safety_contract_version=1, valid boot_id)
  - incompatible Agent monitoring/read-only compatibility preserved while ALL control paths are blocked
  - safe STOP/IDLE/UNLOAD actions preserved
  - reconcile + legacy `protocol_version=1` cannot bypass `_deliver_atomic`
  - Gate Reset enforces `safety_contract_version` while keeping its independent FAULT-state recovery contract
  - Agent boot change / lease epoch change invalidation
  - Agent-side `safety_protocol.compute_movement_permitted` strict int rejection of bool/float versions
"""
import json
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'server'), str(ROOT / 'agent' / 'limo_robot_agent')]

from fleet_server.auth import Principal  # noqa: E402
from fleet_server.db import DB  # noqa: E402
from fleet_server.safety import agent_contract_valid  # noqa: E402
from fleet_server.service import ApiError, Fleet  # noqa: E402
from limo_robot_agent import safety_protocol as sp  # noqa: E402

ADMIN = Principal('admin', 'admin')
OPER = Principal('operator', 'operator')
CFG = SimpleNamespace(offline_after_s=6.0, command_ttl_s=120.0, resend_after_s=1.0, require_operating_mode=True)

BAD_CONTRACTS = [
    ('proto_old', {'protocol_version': 1, 'safety_contract_version': 1, 'boot_id': 'b1'}),
    ('proto_new', {'protocol_version': 3, 'safety_contract_version': 1, 'boot_id': 'b1'}),
    ('proto_float', {'protocol_version': 2.0, 'safety_contract_version': 1, 'boot_id': 'b1'}),
    ('proto_bool', {'protocol_version': True, 'safety_contract_version': 1, 'boot_id': 'b1'}),
    ('proto_str', {'protocol_version': '2', 'safety_contract_version': 1, 'boot_id': 'b1'}),
    ('proto_missing', {'safety_contract_version': 1, 'boot_id': 'b1'}),
    ('safety_0', {'protocol_version': 2, 'safety_contract_version': 0, 'boot_id': 'b1'}),
    ('safety_2', {'protocol_version': 2, 'safety_contract_version': 2, 'boot_id': 'b1'}),
    ('safety_float', {'protocol_version': 2, 'safety_contract_version': 1.0, 'boot_id': 'b1'}),
    ('safety_bool', {'protocol_version': 2, 'safety_contract_version': True, 'boot_id': 'b1'}),
    ('safety_str', {'protocol_version': 2, 'safety_contract_version': '1', 'boot_id': 'b1'}),
    ('safety_missing', {'protocol_version': 2, 'boot_id': 'b1'}),
    ('boot_empty', {'protocol_version': 2, 'safety_contract_version': 1, 'boot_id': ''}),
    ('boot_blank', {'protocol_version': 2, 'safety_contract_version': 1, 'boot_id': '   '}),
    ('boot_int', {'protocol_version': 2, 'safety_contract_version': 1, 'boot_id': 123}),
    ('boot_long', {'protocol_version': 2, 'safety_contract_version': 1, 'boot_id': 'x' * 257}),
]


def _idle_state(gate_status='NORMAL', fault_latched=False):
    reset_view = {
        'state': gate_status,
        'boot_id': 'gateboot1',
        'fault_epoch': 3,
        'fault_latched': fault_latched,
        'fault_reasons': ['STALL'] if fault_latched else [],
        'last_fault_reasons': ['STALL'] if fault_latched else [],
        'cmd_input': {'state': 'BOUND', 'sticky': None},
        'reset_readiness': {'enforced': True, 'ready': True, 'blockers': []},
    }
    return {
        'safety_gate': {
            'boot_id': 'gateboot1',
            'status': gate_status,
            'age_s': 0.05,
            'fault_epoch': 3,
            'fault_latched': fault_latched,
            'cmd_blocked': fault_latched,
            'movement_permitted': not fault_latched,
            'reset_view': reset_view,
        },
        'operation_mode': 'NAVIGATION',
        'operation_revision': 1,
        'map_epoch': 1,
        'localization_epoch': 1,
        'map_version_id': 'map1',
        'localization_map_version_id': 'map1',
        'map_verified': True,
        'localization_status': 'CONFIRMED',
        'execution': {
            'active': None,
            'nav2_goals': {'known': True, 'publishers': 1, 'active_count': 0, 'received_status_array': True, 'age_s': 0.05},
            'nav_idle': {'ok': True, 'reason': None},
        },
        'motion_watchdog': {'required': True, 'state': 'MOTION_AUTHORIZED', 'agent_boot_matches': True, 'epoch': 'wd1', 'age_s': 0.05},
        'velocity_arbiter': {'mode': 'MOCK_LOCKED', 'ready': True, 'operating_mode': 'IDLE', 'selected_linear': 0.0, 'selected_turn': 0.0, 'blocker': 'MODE_IDLE'},
        'mapping_worker': {'enabled': True, 'status': 'IDLE', 'processes_started': False},
        'slam': {'enabled': True, 'status': 'IDLE'},
        'recording': {'enabled': True, 'status': 'IDLE'},
    }


class ContractHelperTests(unittest.TestCase):
    def test_valid_contract_accepted(self):
        self.assertTrue(agent_contract_valid({'protocol_version': 2, 'safety_contract_version': 1, 'boot_id': 'boot1'}))

    def test_invalid_contracts_rejected(self):
        for label, bad in BAD_CONTRACTS:
            with self.subTest(label=label):
                self.assertFalse(agent_contract_valid(bad))
        self.assertFalse(agent_contract_valid(None))
        self.assertFalse(agent_contract_valid('not-a-dict'))


class ServerControlPathHardeningTests(unittest.TestCase):
    def setUp(self):
        self.db = DB(':memory:')
        self.f = Fleet(self.db, CFG)

    def _register(self, agent_overrides, *, gate_status='NORMAL', fault_latched=False, reconcile=True):
        agent = {
            'mode': 'mock',
            'simulated': True,
            'missions_enabled': True,
            'manual_web_bench': True,
            'mapping_worker': True,
            'navigation_localization': True,
            'locked_velocity_arbiter': True,
            'gate_reset': True,
        }
        agent.update(agent_overrides)
        self.f.hello('R', agent)
        boot = agent.get('boot_id')
        if reconcile and boot:
            self.f.reconcile('R', boot, {}, {}, True)
        if boot:
            self.f.ingest_state('R', 1, _idle_state(gate_status=gate_status, fault_latched=fault_latched), boot)
        else:
            # Empty boot_id is already refused by check_session on ingest_state; inject state directly to verify downstream guards too.
            with self.assertRaises(ApiError) as cm:
                self.f.ingest_state('R', 1, _idle_state(gate_status=gate_status, fault_latched=fault_latched), boot)
            self.assertEqual(cm.exception.code, 'agent_session_mismatch')
            self.f.robots['R']['reconciled'] = True
            self.f.robots['R']['state'] = _idle_state(gate_status=gate_status, fault_latched=fault_latched)
            self.f.robots['R']['state_ts'] = time.time()
        return self.f.lease_action('R', ADMIN, 'acquire', 120)['epoch']

    def test_incompatible_agent_can_be_monitored_but_cannot_control(self):
        for label, bad in BAD_CONTRACTS:
            with self.subTest(label=label):
                self.db = DB(':memory:')
                self.f = Fleet(self.db, CFG)
                epoch = self._register(bad)

                # Monitoring / status read remains available for legacy/incompatible agents
                view = self.f.robot_view(self.f.robots['R'])
                self.assertTrue(view['online'])
                self.assertEqual(len(self.f.robots_view()), 1)
                rd = self.f.readiness('R')
                self.assertEqual(rd['robot_id'], 'R')

                # 1. Mode transitions to MANUAL / MAPPING / NAVIGATION blocked; IDLE allowed
                for target in ('MANUAL', 'MAPPING', 'NAVIGATION'):
                    with self.assertRaises(ApiError) as cm:
                        self.f.request_mode('R', 'admin', {'target': target, 'epoch': epoch, 'idempotency_key': f'm-{target}'})
                    self.assertEqual(cm.exception.code, 'agent_protocol')
                idle_res = self.f.request_mode('R', 'admin', {'target': 'IDLE', 'epoch': epoch, 'idempotency_key': 'm-idle'})
                self.assertIn('mode', idle_res)

                # 2. Mapping START blocked
                with self.assertRaises(ApiError) as cm:
                    self.f.request_operation('R', 'admin', {'action': 'START', 'epoch': epoch, 'idempotency_key': 'map-1'})
                self.assertEqual(cm.exception.code, 'agent_protocol')

                # 3. Manual bench start blocked (and pull blocked if boot is valid)
                self.f._mode_save('R', dict(self.f._mode_new('MANUAL', None, 't'), epoch=epoch, owner='admin', agent_boot=bad.get('boot_id')))
                # mode_view itself invalidates non-IDLE mode for incompatible agent!
                self.assertEqual(self.f.mode_view('R')['mode'], 'IDLE')
                with self.assertRaises(ApiError) as cm:
                    self.f.manual_bench('R', 'admin', {'action': 'start', 'epoch': epoch})
                self.assertIn(cm.exception.code, ('mode_not_manual', 'agent_protocol'))
                with self.assertRaises(ApiError) as cm:
                    self.f._manual_bench_robot('R')
                self.assertEqual(cm.exception.code, 'agent_protocol')

                # 4. Gate Reset blocked with AGENT_PROTOCOL
                self.f.robots['R']['state'] = _idle_state(gate_status='STALL_LATCHED', fault_latched=True)
                self.f.robots['R']['state_ts'] = time.time()
                blockers = self.f.gate_reset_blockers('R', ADMIN)
                self.assertIn('AGENT_PROTOCOL', blockers)
                self.assertIsNone(self.f.gate_reset_for_agent('R'))

                # 5. Saved-Map Navigation LOAD / INITIAL_POSE blocked; UNLOAD allowed
                self.f.robots['R']['state'] = _idle_state()
                self.f.robots['R']['state_ts'] = time.time()
                self.assertIsNone(self.f.navigation_for_agent('R'))
                # Even on a legacy server (no operating_mode row), LOAD/INITIAL_POSE are blocked
                self.f.db.x('DELETE FROM meta WHERE k=?', ('operating_mode:R',))
                self.f.cfg = SimpleNamespace(offline_after_s=6.0, command_ttl_s=120.0, resend_after_s=1.0, require_operating_mode=False)
                for action in ('LOAD', 'INITIAL_POSE'):
                    with self.assertRaises(ApiError) as cm:
                        self.f.request_navigation('R', 'admin', {'action': action, 'epoch': epoch})
                    self.assertEqual(cm.exception.code, 'agent_protocol')
                self.assertIsNotNone(self.f.request_navigation('R', 'admin', {'action': 'UNLOAD', 'epoch': epoch}))

                # 6. Navigation mission blocked via safety_context
                self.assertIsNone(self.f.safety_context('R'))
                self.assertEqual(self.f.safety_context_reasons['R']['reason'], 'AGENT_PROTOCOL')
                with self.assertRaises(ApiError) as cm:
                    self.f.create_mission('R', 'navigate_to_pose', {'frame_id': 'map', 'x': 1.0, 'y': 0.0}, 'nav-1', '', 'admin', epoch)
                self.assertEqual(cm.exception.code, 'safety_not_ready')

                # 7. Legacy SLAM start/save & recording start blocked; safe stop allowed
                for action in ('start', 'save'):
                    with self.assertRaises(ApiError) as cm:
                        self.f.slam_command('R', action, 'sess1', 'admin', floor_id='fl1' if action == 'save' else None)
                    self.assertEqual(cm.exception.code, 'agent_protocol')
                with self.assertRaises(ApiError) as cm:
                    self.f.recording_command('R', 'start', 'sess1', 'admin')
                self.assertEqual(cm.exception.code, 'agent_protocol')
                self.f.robots['R']['state']['slam'] = {'enabled': True, 'status': 'RUNNING', 'session_id': 'sess1'}
                stop_slam = self.f.slam_command('R', 'stop', 'sess1', 'admin')
                self.assertEqual(stop_slam['status'], 'PENDING')
                self.f.robots['R']['state']['recording'] = {'enabled': True, 'status': 'RECORDING', 'session_id': 'sess1'}
                stop_rec = self.f.recording_command('R', 'stop', 'sess1', 'admin')
                self.assertEqual(stop_rec['status'], 'PENDING')

    def test_legacy_protocol_1_after_reconcile_never_receives_control_commands(self):
        # Legacy protocol_version=1 skips check_session/check_execution in ingest_state,
        # yet _deliver_atomic MUST NOT deliver non-stop control commands even if queued in DB.
        self._register({'protocol_version': 1, 'safety_contract_version': 1, 'boot_id': 'legacy1'})
        now = time.time()
        for idx, ctype in enumerate(('slam_start', 'record_start', 'navigate_to_pose'), 1):
            self.db.x(
                'INSERT INTO commands(command_id,robot_id,mission_id,type,payload_json,status,seq,created_at,expires_at) VALUES(?,?,?,?,?,?,?,?,?)',
                (f'c_leg_{idx}', 'R', None, ctype, json.dumps({'session_id': 's1'}), 'PENDING', idx, now, now + 60),
            )
        res = self.f.ingest_state('R', 2, _idle_state(), 'legacy1')
        self.assertEqual(res['commands'], [])
        self.assertIsNone(res['navigation_request'])
        self.assertIsNone(res['gate_reset_request'])
        self.assertEqual(res['operating_mode']['mode'], 'IDLE')

    def test_gate_reset_keeps_independent_fault_recovery_contract(self):
        # Valid contract agent in IDLE with FAULTED gate: safety_context() is None (cannot drive),
        # yet Gate Reset is admitted via its independent recovery contract.
        epoch = self._register({'protocol_version': 2, 'safety_contract_version': 1, 'boot_id': 'goodboot'},
                               gate_status='STALL_LATCHED', fault_latched=True)
        self.assertIsNone(self.f.safety_context('R'))
        self.assertEqual(self.f.gate_reset_blockers('R', ADMIN), [])
        req = self.f.request_gate_reset('R', ADMIN, {
            'epoch': epoch, 'expected_gate_boot_id': 'gateboot1', 'expected_fault_epoch': 3, 'note': 'recover fault',
        })
        self.assertEqual(req['status'], 'QUEUED')
        delivered = self.f.gate_reset_for_agent('R')
        self.assertIsNotNone(delivered)
        self.assertEqual(delivered['request_id'], req['request_id'])

    def test_boot_change_and_lease_change_invalidate_prior_control(self):
        epoch = self._register({'protocol_version': 2, 'safety_contract_version': 1, 'boot_id': 'bootA'},
                               gate_status='STALL_LATCHED', fault_latched=True)
        # Queue a gate reset for bootA, then reconnect as bootB -> abandoned
        self.f.request_gate_reset('R', ADMIN, {
            'epoch': epoch, 'expected_gate_boot_id': 'gateboot1', 'expected_fault_epoch': 3, 'note': 'before restart',
        })
        self.f.hello('R', {'protocol_version': 2, 'safety_contract_version': 1, 'boot_id': 'bootB', 'mode': 'mock', 'simulated': True, 'gate_reset': True})
        self.assertIsNone(self.f.gate_reset_for_agent('R'))
        self.assertEqual(self.f._gr_load('R')['status'], 'ABANDONED_AGENT_RESTART')

        # Active operating mode invalidated on lease release / epoch change
        self.f.reconcile('R', 'bootB', {}, {}, True)
        self.f.ingest_state('R', 1, _idle_state(), 'bootB')
        self.f._mode_save('R', dict(self.f._mode_new('MANUAL', None, 't'), epoch=epoch, owner='admin', agent_boot='bootB'))
        self.assertEqual(self.f.mode_view('R')['mode'], 'MANUAL')
        self.f.lease_action('R', ADMIN, 'release')
        self.assertEqual(self.f.mode_view('R')['mode'], 'IDLE')


class AgentSideStrictVersionTests(unittest.TestCase):
    def _base_kwargs(self):
        now = time.time()
        return dict(
            allow_missions=True,
            gate_eval={
                'rx_fresh': True, 'source_fresh': True, 'status': 'NORMAL',
                'fault_latched': False, 'cmd_blocked': False,
                'boot_id': 'g1', 'fault_epoch': 1, 'epoch_regressed': False, 'boot_changed': False,
            },
            motor_path_ok=True,
            op_ctx={'operation_mode': 'NAVIGATION', 'operation_revision': 1},
            map_ctx={'map_verified': True, 'map_version_id': 'm1', 'map_epoch': 1},
            loc_ctx={'localization_status': 'CONFIRMED', 'localization_map_version_id': 'm1', 'localization_epoch': 1},
            nav2_goals={'known': True, 'received_status_array': True, 'active_count': 0, 'age_s': 0.05},
            session_ctx={
                'connected': True, 'last_state_age_s': 0.05, 'server_boot_id': 'sb1',
                'safety_contract_version': 1, 'protocol_version': 2,
                'lease': {'owner': 'op', 'epoch': 1, 'expires_at': now + 30.0},
            },
            now_wall=now,
        )

    def test_valid_exact_ints_permitted(self):
        ok, reasons = sp.compute_movement_permitted(**self._base_kwargs())
        self.assertTrue(ok, reasons)

    def test_bool_float_or_wrong_server_versions_refused(self):
        for field, bad_vals in (
            ('safety_contract_version', (True, False, 1.0, 0, 2, '1', None)),
            ('protocol_version', (True, False, 2.0, 1, 3, '2', None)),
        ):
            for bad in bad_vals:
                with self.subTest(field=field, bad=bad):
                    kw = self._base_kwargs()
                    kw['session_ctx'][field] = bad
                    ok, reasons = sp.compute_movement_permitted(**kw)
                    self.assertFalse(ok)
                    self.assertIn('SERVER_DISCONNECTED_OR_STALE', reasons)

    def test_parse_safety_context_rejects_bool_and_float_version(self):
        now = time.time()
        base = {
            'version': 1, 'server_boot_id': 'sb1', 'agent_boot_id': 'ab1', 'gate_boot_id': 'gb1',
            'fault_epoch': 1, 'operation_mode': 'NAVIGATION', 'operation_revision': 1,
            'map_version_id': 'm1', 'map_epoch': 1, 'localization_status': 'CONFIRMED', 'localization_epoch': 1,
            'lease_owner': 'op', 'lease_epoch': 1, 'issued_at': now, 'expires_at': now + 10.0,
        }
        sc, err = sp.parse_safety_context(base)
        self.assertIsNotNone(sc, err)
        for bad in (True, False, 1.0, 0, 2, '1', None):
            with self.subTest(version=bad):
                sc, err = sp.parse_safety_context({**base, 'version': bad})
                self.assertIsNone(sc)
                self.assertIn('SAFETY_CONTEXT_INVALID', err)


if __name__ == '__main__':
    unittest.main(verbosity=2)
