"""R1.4.1 P1-1: the Agent must not submit Nav2 goals unless the CURRENT server session carries protocol_version == 2 (exact int).
Real AgentCore.run -> _on_command -> RosRobot._do_nav -> (stub) send_goal_async. Inputs are the Codex R1.4 re-audit probe inputs
(codex_r14_reaudit_20261123_AdGJNw/probe.py): missing, null, bool, float, string, old; plus boot/reconnect/lease cases.
Uses the stub ROS of test_agent_safety_v1 (no rclpy needed). Run: PYTHONPATH=server:agent/limo_robot_agent:tests python3 -m unittest test_r1_4_1_agent_protocol_20261123"""
import sys, tempfile, time, unittest
from pathlib import Path
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'server'), str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'tests')]
import test_agent_safety_v1 as base                                   # installs the ROS stubs (restored in tearDownModule below)
from test_agent_safety_v1 import _make_ready_robot, _make_cmd, _valid_safety_context, core, safety_protocol as sp

MISSING = object()

def tearDownModule(): base.tearDownModule()

def run(hello_proto, state_proto=MISSING, *, state_contract=1, expire_lease=False, stale_context=None, reconnect=False,
        old_owner=False, second_hello_proto=MISSING, boot='srv-boot-1'):
    """One Agent run loop with a synthetic server. Returns (stub Nav2 submissions, command detail, hello count, final snapshot)."""
    with tempfile.TemporaryDirectory() as directory:
        robot = _make_ready_robot()
        agent = core.AgentCore('robot-1', 'http://unused', 'synthetic', lambda: {}, robot, {'boot_id': 'agt-boot-1'}, directory,
                               require_safety_context=True, period_s=0, log=lambda _: None)
        robot.session_fn = agent.session_snapshot; robot.execution_allowed = agent.is_execution_allowed; robot.on_command_meta = agent.record_command_meta
        cmd = _make_cmd(sc=_valid_safety_context(**(stale_context or {})), owner_boot_id='old-agent' if old_owner else agent.boot_id)
        lease = {'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + (-1 if expire_lease else 30)}
        calls = []; snaps = []
        def post(path, body, timeout=None):
            calls.append(path)
            if path == '/agent/hello':
                proto = hello_proto if calls.count(path) == 1 else second_hello_proto
                response = {'server_boot_id': boot}
                if proto is not MISSING: response['protocol_version'] = proto
                return response
            if path == '/agent/reconcile': return {'reconciliation_required': False}
            if path == '/agent/state':
                if reconnect and calls.count(path) == 1:
                    return {'server_boot_id': 'new-server', 'commands': [cmd]}      # server changed: the Agent must re-hello
                agent.stop(); snaps.append(agent.session_snapshot())
                response = {'server_boot_id': boot, 'lease': lease, 'commands': [cmd], 'reconciliation_required': False}   # R1.4.5: the server states it in every state reply
                if state_proto is not MISSING: response['protocol_version'] = state_proto
                if state_contract is not MISSING: response['safety_contract_version'] = state_contract
                return response
            return {'ok': True}
        agent.http = SimpleNamespace(post=post)
        agent.run(); robot._run_tasks()
        return len(robot.nav.sent_goals), agent.seen.get('cmd-1', {}).get('detail'), calls.count('/agent/hello'), (snaps[-1] if snaps else None)

class AgentProtocolTests(unittest.TestCase):
    def test_valid_protocol_2_still_navigates(self):
        sent, detail, _, snap = run(2)
        self.assertEqual(sent, 1, detail); self.assertEqual(snap['protocol_version'], 2)
    def test_bad_protocol_never_reaches_nav2(self):
        for label, value in [('missing', MISSING), ('null', None), ('True', True), ('False', False), ('2.0', 2.0), ("'2'", '2'), ('old 1', 1), ('0', 0), ('3', 3)]:
            with self.subTest(hello=label):
                sent, detail, _, snap = run(value)
                self.assertEqual(sent, 0, f'protocol {label} reached send_goal_async')
                self.assertIn('SERVER_DISCONNECTED_OR_STALE', str(detail))
    def test_same_bad_values_in_state_response_replace_a_good_hello(self):
        for label, value in [('null', None), ('True', True), ('2.0', 2.0), ("'2'", '2'), ('1', 1)]:
            with self.subTest(state=label): self.assertEqual(run(2, value)[0], 0)
    def test_bad_safety_contract_still_blocked(self):
        for label, kw in {'missing': MISSING, 'null': None, 'float': 1.0, 'old': 0}.items():
            with self.subTest(contract=label): self.assertEqual(run(2, state_contract=kw)[0], 0)
    def test_reconnect_rehello_without_protocol_invalidates_old_approval(self):
        sent, _, hellos, snap = run(2, reconnect=True, second_hello_proto=MISSING)
        self.assertEqual(hellos, 2); self.assertEqual(sent, 0); self.assertIsNone(snap['protocol_version'])
    def test_reconnect_rehello_with_protocol_2_recovers(self):
        sent, _, hellos, snap = run(MISSING, reconnect=True, second_hello_proto=2)
        self.assertEqual((hellos, sent, snap['protocol_version']), (2, 1, 2))
    def test_protocol_is_bound_to_the_server_boot(self):
        with tempfile.TemporaryDirectory() as d:
            agent = core.AgentCore('robot-1', 'http://unused', 't', lambda: {}, None, {'boot_id': 'a'}, d, log=lambda _: None)
            agent.server_boot = 'srv-A'; agent.server_protocol_version = 2; agent.server_protocol_boot = 'srv-A'
            self.assertEqual(agent.session_snapshot()['protocol_version'], 2)
            agent.server_boot = 'srv-B'                                   # server restart seen without a new hello
            self.assertIsNone(agent.session_snapshot()['protocol_version'])
            agent.server_boot = None; self.assertIsNone(agent.session_snapshot()['protocol_version'])
    def test_state_reply_from_a_new_server_boot_clears_the_version(self):
        sent, _, hellos, snap = run(2, reconnect=True, second_hello_proto=MISSING); self.assertIsNone(snap['protocol_version'])
    def test_lease_expiry_and_epoch_change_still_refused(self):
        self.assertEqual(run(2, expire_lease=True)[0], 0)
        self.assertEqual(run(2, stale_context={'lease_epoch': 6})[0], 0)
        self.assertEqual(run(2, stale_context={'server_boot_id': 'stale-server'})[0], 0)
        self.assertEqual(run(2, old_owner=True)[0], 0)

class PureFunctionTests(unittest.TestCase):
    def test_server_contract_valid_requires_both_exact_ints(self):
        ok = {'protocol_version': 2, 'safety_contract_version': 1}
        self.assertTrue(sp.server_contract_valid(ok))
        for k, bad in [('protocol_version', None), ('protocol_version', True), ('protocol_version', 2.0), ('protocol_version', '2'), ('protocol_version', 1),
                       ('safety_contract_version', None), ('safety_contract_version', True), ('safety_contract_version', 1.0), ('safety_contract_version', 2)]:
            self.assertFalse(sp.server_contract_valid(dict(ok, **{k: bad})), (k, bad))
        for k in ok: self.assertFalse(sp.server_contract_valid({x: y for x, y in ok.items() if x != k}), k)
        self.assertFalse(sp.server_contract_valid(None)); self.assertFalse(sp.server_contract_valid('x'))
    def test_active_goal_is_cancelled_when_the_contract_is_lost(self):
        robot = _make_ready_robot()
        # reuse the suite's own active-goal fixture through the public check function
        sc = sp.parse_safety_context(_valid_safety_context())[0]
        gate = {'rx_fresh': True, 'source_fresh': True, 'status': 'NORMAL', 'fault_latched': False, 'cmd_blocked': False, 'boot_id': sc['gate_boot_id'], 'fault_epoch': sc['fault_epoch']}
        sess = {'connected': True, 'last_state_age_s': 0.1, 'server_boot_id': sc['server_boot_id'], 'safety_contract_version': 1,
                'lease': {'owner': sc['lease_owner'], 'epoch': sc['lease_epoch'], 'expires_at': time.time() + 30}}
        common = dict(allow_missions=True, gate_eval=gate, motor_path_ok=True, op_ctx={'operation_mode': sc['operation_mode'], 'operation_revision': sc['operation_revision']},
                      map_ctx={'map_verified': True, 'map_version_id': sc['map_version_id'], 'map_epoch': sc['map_epoch']},
                      loc_ctx={'localization_status': sc['localization_status'], 'localization_map_version_id': sc['map_version_id'], 'localization_epoch': sc['localization_epoch']})
        rec = {'safety_context': sc}
        self.assertEqual(sp.check_active_goal_safety(rec, session_ctx=dict(sess, protocol_version=None), **common), 'SERVER_CONTRACT_INVALID')
        self.assertEqual(sp.check_active_goal_safety(rec, session_ctx=dict(sess, protocol_version=1), **common), 'SERVER_CONTRACT_INVALID')
        self.assertIsNone(sp.check_active_goal_safety(rec, session_ctx=dict(sess, protocol_version=2), **common))

if __name__ == '__main__': unittest.main(verbosity=2)
