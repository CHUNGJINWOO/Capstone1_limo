"""R1.4.5: (P1-A) losing the server session (HTTP error or any exception in the run loop) is an authority withdrawal that goes through
the same contract_fence and generation rule as every other one; (P1-B) only the exact boolean False in reconciliation_required counts as
"no reconciliation pending"; the real Fleet server now states it in every state reply. Real AgentCore.run + RosRobot (stub ROS / Nav2)
and the real Fleet server methods; ordering fixed with Events and the instrumented fence from the R1.4.4 tests (timeouts only detect
deadlocks). Cancel REQUESTS on stub handles are observed, not real stopping.
Run (no ROS sourced): PYTHONPATH=server:agent/limo_robot_agent:tests python3 -m unittest test_r1_4_5_withdrawal_paths_20261123"""
import sys, tempfile, threading, time, unittest
from pathlib import Path
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'server'), str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'tests')]
import test_agent_safety_v1 as base
from test_agent_safety_v1 import _make_ready_robot, _make_cmd, _valid_safety_context, StubGoalHandle, GoalStatus, core
from test_r1_4_4_submission_fence_20261123 import TrackingFence, Sim as Sim4
from test_r1_4_safety_contract_hardening_20261123 import DB, Fleet, CFG, _idle_state

def tearDownModule(): base.tearDownModule()
T = 5.0
class FastEvt:
    """stop_evt replacement: the retry back-off returns at once (no sleeping) but stop() still ends the loop."""
    def __init__(self): self.e = threading.Event()
    def set(self): self.e.set()
    def is_set(self): return self.e.is_set()
    def wait(self, t=None): return self.e.is_set()

def failing(boundary, kind='http', *, recover=True):
    """Reply 1 valid + cmd-1; reply 2 raises (HTTP 503 or a generic exception) when the test releases it; the loop re-hellos; reply 3 valid."""
    with tempfile.TemporaryDirectory() as d:
        robot = _make_ready_robot()
        agent = core.AgentCore('robot-1', 'http://unused', 't', lambda: {}, robot, {'boot_id': 'agt-boot-1'}, d, period_s=0, require_safety_context=True, log=lambda _: None)
        fence = agent.contract_fence = TrackingFence(agent.contract_fence); agent.stop_evt = FastEvt()
        robot.session_fn = agent.session_snapshot; robot.execution_allowed = agent.is_execution_allowed; robot.on_command_meta = agent.record_command_meta
        env = dict(server_boot_id='srv-boot-1', lease={'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 30}, reconciliation_required=False, safety_contract_version=1, commands=[])
        cmd = _make_cmd(sc=_valid_safety_context()); go2, at2, applied, done = (threading.Event() for _ in range(4)); n = {'s': 0, 'h': 0}; gen = {}
        orig_note = agent._contract_note_locked
        def note():                                     # observe the moment the withdrawal is applied (inside fence + lock)
            orig_note()
            if n['s'] == 2 and not agent.connected: gen['at_withdrawal'] = agent.contract_generation; applied.set()
        agent._contract_note_locked = note
        def post(path, body, timeout=None):
            if path.endswith('/hello'): n['h'] += 1; return dict(server_boot_id='srv-boot-1', protocol_version=2)
            if path.endswith('/reconcile'): return {'reconciliation_required': False}
            if not path.endswith('/state'): return {'ok': True}
            n['s'] += 1
            if n['s'] == 1: return dict(env, commands=[cmd])
            if n['s'] == 2:
                at2.set()
                if not go2.wait(T): agent.stop(); raise RuntimeError('boundary not reached')
                if kind == 'http': raise core.HttpError(503, 'test offline')
                raise ConnectionResetError('test reset')
            if n['s'] == 3 and not recover: agent.stop(); done.set(); raise core.HttpError(503, 'still offline')
            agent.stop(); done.set(); return dict(env)
        agent.http = SimpleNamespace(post=post)
        th = threading.Thread(target=agent.run, name='agent-run'); th.start(); out = {}
        try:
            assert at2.wait(T)
            calls = {'n': 0}; orig = robot._verify_nav_preconditions
            def verify(c):
                calls['n'] += 1
                if calls['n'] == 2 and boundary == 'before-stage3': go2.set(); assert applied.wait(T)
                res = orig(c)
                if calls['n'] == 3 and boundary == 'inside-stage3':
                    assert res[0]; go2.set()
                    assert fence.waiting.wait(T), 'withdrawal never tried to enter the fence'
                    out['applied_before_send'] = applied.is_set(); out['gen_at_send'] = agent.session_snapshot()['contract_generation']; out['connected_at_send'] = agent.connected; out['lease_at_send'] = agent.lease is not None
                return res
            robot._verify_nav_preconditions = verify
            if boundary == 'before-drain': go2.set(); assert applied.wait(T); assert done.wait(T)
            robot._run_tasks()
            if boundary == 'after-send': go2.set()
            assert applied.wait(T) and done.wait(T)
            out.update(sent=len(robot.nav.sent_goals), journal=agent.seen['cmd-1']['status'], detail=agent.seen['cmd-1'].get('detail'), hellos=n['h'], **gen)
            if robot.nav.sent_goals:
                rec = robot.active; h = StubGoalHandle(uuid_hex=rec['goal_uuid'])
                robot.nav.sent_goals[0][2].set_result(h); robot._monitor_active_goal(); robot._monitor_active_goal()
                out.update(cancels=h.cancel_calls, reason=rec['fault_cancel_reason'])
            else: out.update(cancels=0, reason=None)
            out['waiters'] = list(fence.waiters); return out
        finally:
            go2.set(); agent.stop(); th.join(T); assert not th.is_alive(), 'run loop did not terminate'

class HttpErrorWithdrawal(unittest.TestCase):
    def test_A_error_before_stage3_blocks_submission(self):
        for kind in ('http', 'exception'):
            with self.subTest(kind=kind):
                r = failing('before-stage3', kind); self.assertEqual((r['sent'], r['journal']), (0, 'REJECTED')); self.assertEqual(r['at_withdrawal'], 1)
    def test_B_error_released_inside_stage3_is_serialized_after_the_send(self):
        for kind in ('http', 'exception'):
            with self.subTest(kind=kind):
                r = failing('inside-stage3', kind)
                self.assertFalse(r['applied_before_send']); self.assertEqual(r['gen_at_send'], 0); self.assertIn('agent-run', r['waiters'])
                self.assertTrue(r['connected_at_send']); self.assertTrue(r['lease_at_send'])        # Codex: connected=False/lease=None were applied before the send
                self.assertEqual((r['sent'], r['cancels']), (1, 1)); self.assertEqual(r['at_withdrawal'], 1)
    def test_C_error_after_send_keeps_the_cancel_duty(self):
        for kind in ('http', 'exception'):
            with self.subTest(kind=kind):
                r = failing('after-send', kind); self.assertEqual((r['sent'], r['cancels'], r['reason']), (1, 1, 'SERVER_CONTRACT_INVALID'))
    def test_C2_cancel_duty_survives_the_reconnect(self):
        r = failing('after-send', 'http'); self.assertEqual(r['hellos'], 2); self.assertEqual(r['cancels'], 1)
    def test_D_error_then_reconnect_never_runs_the_queued_command(self):
        for kind in ('http', 'exception'):
            with self.subTest(kind=kind):
                r = failing('before-drain', kind); self.assertEqual((r['hellos'], r['sent'], r['journal']), (2, 0, 'REJECTED'))
    def test_D2_still_offline_variant(self):
        r = failing('after-send', 'http', recover=False); self.assertEqual((r['sent'], r['cancels']), (1, 1))

MISSING = object()
class Sim(Sim4):
    def post(self, path, body, timeout=None):
        out = super().post(path, body, timeout)
        if path.endswith('/state'):
            for k, v in list(out.items()):
                if v is MISSING: out.pop(k)
        return out

class ReconciliationFlag(unittest.TestCase):
    BAD = [('missing', MISSING), ('null', None), ('0', 0), ('1', 1), ("'false'", 'false'), ("''", ''), ('[]', []), ('{}', {}), ('0.0', 0.0)]
    def test_E_true_then_exact_false_allows_a_new_goal(self):
        s = Sim([({}, []), ({'reconciliation_required': True}, []), ({'reconciliation_required': False}, [])]); s.replies[2] = ({'reconciliation_required': False}, [s.cmd()])
        s.hooks[4] = lambda s: s.drain(); s.run(); self.assertEqual(s.sent, 1)
    def test_F_G_true_then_malformed_or_missing_never_allows_a_new_goal(self):
        for label, bad in self.BAD:
            with self.subTest(value=label):
                s = Sim([({}, []), ({'reconciliation_required': True}, []), ({}, [])]); s.replies[2] = ({'reconciliation_required': bad}, [s.cmd()])
                s.hooks[4] = lambda s: (setattr(s, 'snap', s.agent.session_snapshot()), s.drain()); s.run(); s.drain()   # Codex: each of these submitted 1
                self.assertEqual((s.sent, s.journal()), (0, 'REJECTED')); self.assertIs(s.snap['reconciliation_required'], True)   # right after the bad reply
    def test_G2_malformed_value_after_a_valid_session_also_withdraws(self):
        for label, bad in self.BAD:
            with self.subTest(value=label):
                s = Sim([({}, []), ({'reconciliation_required': bad}, [])]); s.replies[0] = ({}, [s.cmd()]); s.run(); s.drain()
                self.assertEqual(s.sent, 0)                                            # the command of reply 1 lost its generation
    def test_H_malformed_then_valid_never_revives_the_old_command(self):
        s = Sim([({}, []), ({'reconciliation_required': None}, []), ({}, [])]); c = s.cmd(); new = s.cmd('cmd-new', seq=2)
        s.replies[1] = ({'reconciliation_required': None}, [c]); s.replies[2] = ({}, [c, new]); s.hooks[4] = lambda s: s.drain(); s.run(); s.drain()
        self.assertEqual(s.journal(), 'REJECTED'); self.assertEqual(s.sent, 1); self.assertNotEqual(s.journal('cmd-new'), 'REJECTED')
    def test_I_lease_epoch_change_still_refuses_the_old_goal(self):
        s = Sim([({}, [])]); c = s.cmd(); s.replies[0] = ({'lease': {'owner': 'op-alice', 'epoch': 8, 'expires_at': time.time() + 60}}, [c]); s.hooks[2] = lambda s: s.drain(); s.run()   # ROS runs while the session lease is epoch 8
        self.assertEqual(s.sent, 0)

class ServerWire(unittest.TestCase):
    def test_real_server_states_the_flag_and_the_agent_follows_it(self):
        f = Fleet(DB(':memory:'), CFG)
        with tempfile.TemporaryDirectory() as d:
            a = core.AgentCore('R', 'http://unused', 't', lambda: {'execution': {}}, None, {'boot_id': 'b'}, d, period_s=0, log=lambda _: None)
            a.agent_info['boot_id'] = 'b'; seen = []
            def post(path, body, timeout=None):
                if path.endswith('/hello'): return f.hello('R', body['agent'])
                if path.endswith('/reconcile'): return f.reconcile('R', body['agent_boot_id'], body['journal'], body['execution'], body['journal_ok'])
                if path.endswith('/state'):
                    if not seen:                                                                        # an operator holds the lease (R1.4.6)
                        from fleet_server.auth import Principal; f.lease_action('R', Principal('op', 'operator'), 'acquire', 300)
                    if len(seen) == 1: f.robots['R']['reconciliation_required'] = True                 # server-side block appears
                    if len(seen) == 2: f.robots['R']['reconciliation_required'] = False                # and is cleared
                    r = f.ingest_state('R', body['seq'], _idle_state(), body['agent_boot_id']); seen.append((r.get('reconciliation_required', 'ABSENT'), a.contract_generation))
                    if len(seen) == 3: a.stop()
                    return r
                return {'ok': True}
            a.http = SimpleNamespace(post=post); a.run()
            self.assertEqual([x[0] for x in seen], [False, True, False])                              # explicit booleans in every normal/blocked reply
            self.assertEqual(a.session_snapshot()['reconciliation_required'], False)
            self.assertGreaterEqual(a.contract_generation, 1)
            with a.lock: self.assertTrue(a._contract_ok_locked())

class Concurrency(unittest.TestCase):
    def test_J_errors_reconnects_flaps_submissions_cancels_and_shutdown(self):
        with tempfile.TemporaryDirectory() as d:
            robot = _make_ready_robot()
            agent = core.AgentCore('robot-1', 'http://unused', 't', lambda: {}, robot, {'boot_id': 'agt-boot-1'}, d, period_s=0, require_safety_context=True, log=lambda _: None)
            agent.stop_evt = FastEvt()
            robot.session_fn = agent.session_snapshot; robot.execution_allowed = agent.is_execution_allowed; robot.on_command_meta = agent.record_command_meta
            env = dict(server_boot_id='srv-boot-1', lease={'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 60})
            n = {'s': 0}; stop = threading.Event(); errors = []; bad_ids = set(); submitted = []
            send = robot.nav.send_goal_async
            def capture(*a, **k): submitted.append((robot.active or {}).get('command_id')); return send(*a, **k)
            robot.nav.send_goal_async = capture
            def post(path, body, timeout=None):
                if path.endswith('/hello'): return dict(server_boot_id='srv-boot-1', protocol_version=2)
                if path.endswith('/reconcile'): return {'reconciliation_required': False}
                if not path.endswith('/state'): return {'ok': True}
                n['s'] += 1; k = n['s']
                if k >= 500: agent.stop()
                if k % 11 == 5: raise core.HttpError(503, 'flap')
                if k % 13 == 7: raise ConnectionResetError('flap')
                recon = [False, True, None, 0, 'x'][k % 5] if k % 4 == 1 else False
                scv = None if k % 7 == 3 else 1
                cmds = [_make_cmd(cmd_id=f'c{k}', mission_id=f'm{k}', seq=k, sc=_valid_safety_context())] if k % 3 == 1 else []
                if cmds and (recon is not False or scv != 1): bad_ids.add(f'c{k}')
                return dict(env, reconciliation_required=recon, safety_contract_version=scv, commands=cmds)
            agent.http = SimpleNamespace(post=post)
            def ros():
                try:
                    while not stop.is_set():
                        robot._run_tasks(); robot._monitor_active_goal()
                        for g in list(robot.nav.sent_goals):
                            if not g[2]._done:
                                rec = robot.active; g[2].set_result(StubGoalHandle(uuid_hex=(rec or {}).get('goal_uuid') or '22' * 16))
                        a = robot.active
                        if a and a.get('handle') is not None and a['handle'].cancel_calls and not a['handle'].result_future._done:
                            a['handle'].result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
                except Exception as e: errors.append(repr(e))
            r = threading.Thread(target=ros, name='ros'); th = threading.Thread(target=agent.run, name='agent-run')
            r.start(); th.start(); th.join(30); stop.set(); r.join(30)
            self.assertFalse(th.is_alive(), 'run loop deadlocked'); self.assertFalse(r.is_alive(), 'ros thread deadlocked'); self.assertEqual(errors, [])
            self.assertGreaterEqual(n['s'], 500)
            self.assertEqual(sorted(set(submitted) & bad_ids), [])                  # nothing received under a withdrawn authority was ever submitted

if __name__ == '__main__': unittest.main(verbosity=2)
