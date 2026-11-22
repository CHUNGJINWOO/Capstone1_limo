"""R1.4.2: (A) every state reply must carry a valid safety_contract_version, a missing key may not keep the previous approval, and a live
goal is cancelled; a state reply can withdraw but never grant the hello protocol approval. (B) the STOP exception of an incompatible Mapping
Worker needs boot AND session. Real AgentCore.run -> RosRobot (stub ROS) and the real Fleet server code; inputs follow the Codex R1.4.1
audit probe (codex_r141_audit_20261123_b4bnJg/edge.py). A cancel REQUEST is observed on the stub goal handle: this proves nothing about
the real robot stopping.
Run (no ROS sourced): PYTHONPATH=server:agent/limo_robot_agent:tests python3 -m unittest test_r1_4_2_contract_followup_20261123"""
import sys, tempfile, time, unittest
from pathlib import Path
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'server'), str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'tests')]
import test_agent_safety_v1 as base
from test_agent_safety_v1 import _make_ready_robot, _make_cmd, _valid_safety_context, StubGoalHandle, GoalStatus, core
from test_r1_4_1_mapping_stop_20261123 import MappingStop, worker, code, EVID
from test_r1_4_safety_contract_hardening_20261123 import _idle_state

MISSING = object()
def tearDownModule(): base.tearDownModule()

def flow(second=None, *, active=False, third=None, hello_proto=2, second_hello=MISSING, lease_expired_second=False, needs_reconcile=False, stale_epoch_cmd=False):
    """state 1: valid contract (command delivered first if `active`); state 2: `second` fields (command delivered unless `active`);
    optional state 3: `third` fields. Returns dict(sent, cancels, reason, status, hellos, handle)."""
    second = {} if second is None else second
    with tempfile.TemporaryDirectory() as d:
        robot = _make_ready_robot()
        agent = core.AgentCore('robot-1', 'http://unused', 't', lambda: {}, robot, {'boot_id': 'agt-boot-1'}, d, period_s=0, require_safety_context=True, log=lambda _: None)
        robot.session_fn = agent.session_snapshot; robot.execution_allowed = agent.is_execution_allowed; robot.on_command_meta = agent.record_command_meta
        cmd = _make_cmd(sc=_valid_safety_context(**({'lease_epoch': 6} if stale_epoch_cmd else {})))
        lease = {'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 30}
        handle = StubGoalHandle(uuid_hex='11' * 16); n = {'state': 0, 'hello': 0}
        def post(path, body, timeout=None):
            if path.endswith('/hello'):
                n['hello'] += 1
                p = hello_proto if n['hello'] == 1 else second_hello
                return dict(server_boot_id='srv-boot-1', **({} if p is MISSING else {'protocol_version': p}))
            if path.endswith('/reconcile'): return {'reconciliation_required': False}
            if path.endswith('/state'):
                n['state'] += 1
                env = dict(server_boot_id='srv-boot-1', lease=dict(lease), reconciliation_required=False)
                if n['state'] == 1: return dict(env, safety_contract_version=1, commands=[cmd] if active else [])
                if n['state'] == 2:
                    robot._run_tasks()
                    if active and robot.nav.sent_goals: robot.nav.sent_goals[0][2].set_result(handle)
                    if lease_expired_second: env['lease']['expires_at'] = time.time() - 1
                    if needs_reconcile: return dict(env, needs_reconcile=True, **second)
                    if third is None: agent.stop()
                    return dict(env, commands=[] if active else [cmd], **second)
                agent.stop(); robot._run_tasks()
                return dict(env, commands=[cmd], **(third or {}))
            return {'ok': True}
        agent.http = SimpleNamespace(post=post)
        agent.run(); robot._run_tasks(); robot._monitor_active_goal()
        out = dict(sent=len(robot.nav.sent_goals), cancels=handle.cancel_calls, reason=(robot.active or {}).get('fault_cancel_reason'),
                   status=agent.seen.get('cmd-1', {}).get('status'), hellos=n['hello'], handle=handle, agent=agent, robot=robot)
        if handle.cancel_calls: handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
        return out

class StateReplyContract(unittest.TestCase):
    def test_normal_goal_still_submitted(self):
        r = flow({'safety_contract_version': 1}); self.assertEqual((r['sent'], r['cancels']), (1, 0))
    def test_missing_key_after_valid_reply_blocks_new_goal(self):
        r = flow({}); self.assertEqual(r['sent'], 0)                                           # Codex counterexample: was 1
        self.assertIsNone(r['agent'].session_snapshot()['safety_contract_version'])
    def test_each_invalid_value_blocks_new_goal(self):
        for label, v in [('null', None), ('True', True), ('False', False), ('1.0', 1.0), ("'1'", '1'), ('0', 0), ('2', 2), ('-1', -1)]:
            with self.subTest(value=label): self.assertEqual(flow({'safety_contract_version': v})['sent'], 0)
    def test_live_goal_is_cancelled_when_the_key_goes_missing(self):
        r = flow({}, active=True)                                                                # Codex counterexample: cancel calls was 0
        self.assertEqual((r['sent'], r['cancels'], r['reason']), (1, 1, 'SERVER_CONTRACT_INVALID'))
    def test_live_goal_is_cancelled_for_every_invalid_value(self):
        for label, v in [('null', None), ('True', True), ('1.0', 1.0), ("'1'", '1'), ('0', 0), ('2', 2)]:
            with self.subTest(value=label):
                r = flow({'safety_contract_version': v}, active=True); self.assertEqual((r['cancels'], r['reason']), (1, 'SERVER_CONTRACT_INVALID'))
    def test_valid_goal_is_not_cancelled(self):
        r = flow({'safety_contract_version': 1}, active=True); self.assertEqual((r['sent'], r['cancels']), (1, 0))
    def test_withdrawn_then_valid_reply_restores_contract_but_refused_command_is_not_replayed(self):
        r = flow({}, third={'safety_contract_version': 1})                                       # state 2 invalid (command refused), state 3 valid again
        self.assertEqual(r['agent'].session_snapshot()['safety_contract_version'], 1)
        self.assertEqual(r['sent'], 0)                                                           # the command refused while invalid is terminal: never replayed
        self.assertEqual(r['status'], 'REJECTED')
    def test_cancel_then_stale_redelivery_does_not_submit_again(self):
        r = flow({}, active=True, third={'safety_contract_version': 1})                          # goal cancelled; the same command is delivered again
        self.assertEqual((r['sent'], r['cancels']), (1, 1))
    def test_state_reply_cannot_repair_an_invalid_hello(self):
        r = flow({'protocol_version': 2, 'safety_contract_version': 1}, hello_proto=False)       # Codex: invalid hello corrected by state was allowed
        self.assertEqual(r['sent'], 0); self.assertFalse(base.safety_protocol.server_contract_valid(r['agent'].session_snapshot()))
        self.assertIs(r['agent'].server_protocol_version, False)
    def test_state_reply_can_withdraw_a_valid_hello(self):
        for v in (None, True, 2.0, '2', 1, 3):
            with self.subTest(protocol=v):
                r = flow({'protocol_version': v, 'safety_contract_version': 1}); self.assertEqual(r['sent'], 0)
    def test_absent_protocol_in_state_reply_is_normal(self):
        r = flow({'safety_contract_version': 1}); self.assertEqual(r['agent'].session_snapshot()['protocol_version'], 2); self.assertEqual(r['sent'], 1)
    def test_valid_protocol_2_in_state_reply_is_harmless(self):
        r = flow({'protocol_version': 2, 'safety_contract_version': 1}); self.assertEqual(r['sent'], 1)
    def test_server_boot_change_reconnect_clears_approval(self):
        r = flow({'safety_contract_version': 1}, needs_reconcile=True, second_hello=MISSING)    # re-hello without protocol: no approval, so nothing is submitted
        self.assertEqual((r['hellos'], r['sent']), (2, 0)); self.assertIsNone(r['agent'].session_snapshot()['protocol_version'])
    def test_reconnect_with_new_valid_hello_recovers(self):
        r = flow({'safety_contract_version': 1}, needs_reconcile=True, second_hello=2, third={'safety_contract_version': 1})
        self.assertEqual(r['hellos'], 2); self.assertEqual(r['agent'].session_snapshot()['protocol_version'], 2)
    def test_lease_expiry_and_epoch_change_still_refused(self):
        self.assertEqual(flow({'safety_contract_version': 1}, lease_expired_second=True)['sent'], 0)
        self.assertEqual(flow({'safety_contract_version': 1}, stale_epoch_cmd=True)['sent'], 0)

class StopPollSession(MappingStop):
    def test_incompatible_poll_for_another_session_is_refused(self):
        self.incompatible()
        r = worker(self.f, 'poll', 'wb1', session_id='wrong-session')                            # Codex counterexample: returned the STOP body
        self.assertIsNone(r['command']); self.assertIs(r['renew'], False)
        self.assertIsNone(worker(self.f, 'poll', 'wb1')['command'])                              # no session id at all
        self.assertIsNone(worker(self.f, 'poll', 'wb1', session_id=None)['command'])
        self.assertIsNone(worker(self.f, 'poll', 'wb1', session_id=['session1'])['command'])
        self.assertEqual(worker(self.f, 'poll', 'wb1', session_id='session1')['command']['action'], 'STOP')   # the right session still gets it
    def test_other_boot_same_session_poll_is_refused(self):
        self.incompatible(); self.hello('wb2', proto=1)
        self.assertEqual(code(lambda: worker(self.f, 'poll', 'wb1', session_id='session1')), 'agent_session_mismatch')
        self.assertIsNone(worker(self.f, 'poll', 'wb2', session_id='session1')['command'])
    def test_full_round_trip_with_matching_session_and_single_result(self):
        self.incompatible(); cmd = worker(self.f, 'poll', 'wb1', session_id='session1')['command']; kw = dict(command_id=cmd['command_id'], session_id='session1')
        self.assertTrue(worker(self.f, 'accept', 'wb1', **kw)['accepted'])
        for outcome in ('OK', 'FAILED'): self.assertEqual(worker(self.f, 'result', 'wb1', outcome=outcome, evidence=EVID, **kw)['state'], 'STOPPED')
        self.assertEqual(len(self.f.db.q("SELECT * FROM events WHERE kind='mapping_result'")), 1)
    def test_forged_ids_and_other_actions_still_refused(self):
        self.incompatible(); cid = worker(self.f, 'poll', 'wb1', session_id='session1')['command']['command_id']
        self.assertEqual(code(lambda: worker(self.f, 'accept', 'wb1', command_id='forged', session_id='session1')), 'operation_mismatch')
        self.assertEqual(code(lambda: worker(self.f, 'accept', 'wb1', command_id=cid, session_id='wrong-session')), 'operation_mismatch')
        self.assertEqual(code(lambda: worker(self.f, 'reserve_goal', 'wb1', session_id='session1', goal_id='a' * 32)), 'agent_protocol')
        self.assertIs(worker(self.f, 'poll', 'wb1', session_id='session1')['renew'], False)
    def test_compatible_worker_stop_unchanged(self):
        self.assertEqual(self.req('STOP')['status'], 'QUEUED'); self.assertEqual(worker(self.f, 'poll', 'wb1', session_id='session1')['command']['action'], 'STOP')

if __name__ == '__main__': unittest.main(verbosity=2)
