"""R1.4.4: (P1-A) the final local check and the START of the local Nav2 submission are serialized with every authority change of the
Agent run loop through AgentCore.contract_fence; (P1-B) reconciliation_required=True withdraws the execution authority like an invalid
contract (new generation), so a later False neither revives queued commands nor erases the cancel duty of a live goal.
Real AgentCore.run + real RosRobot._do_nav (stub ROS / stub Nav2). Ordering is fixed with Events and an instrumented fence that reports
when the run-loop thread is WAITING to enter it; waits have timeouts only as deadlock detectors. A cancel REQUEST on a stub handle is
observed: nothing shows a real Nav2 goal ending or a robot stopping.
Run (no ROS sourced): PYTHONPATH=server:agent/limo_robot_agent:tests python3 -m unittest test_r1_4_4_submission_fence_20261123"""
import sys, tempfile, threading, time, unittest
from pathlib import Path
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'server'), str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'tests')]
import test_agent_safety_v1 as base
from test_agent_safety_v1 import _make_ready_robot, _make_cmd, _valid_safety_context, StubGoalHandle, GoalStatus, core

def tearDownModule(): base.tearDownModule()
T = 5.0                                   # deadlock detector only

class TrackingFence:
    """Wraps the AgentCore RLock; sets `waiting` when a thread that does not own it starts to acquire it while another thread holds it."""
    def __init__(self, inner): self.inner, self.owner, self.waiting, self.waiters = inner, None, threading.Event(), []
    def acquire(self, *a, **k):
        me = threading.get_ident()
        if self.owner not in (None, me): self.waiters.append(threading.current_thread().name); self.waiting.set()
        got = self.inner.acquire(*a, **k)
        if got: self.owner = me
        return got
    def release(self):
        self.inner.release()
        try:                                  # RLock: owner stays while still held recursively
            if not self.inner._is_owned(): self.owner = None
        except AttributeError: self.owner = None
    def __enter__(self): self.acquire(); return self
    def __exit__(self, *e): self.release()

def threaded(boundary, second, *, recover=True):
    """Valid reply 1 delivers cmd-1; reply 2 (`second` fields) is released by the test at `boundary`; reply 3 recovers (if `recover`)."""
    with tempfile.TemporaryDirectory() as d:
        robot = _make_ready_robot()
        agent = core.AgentCore('robot-1', 'http://unused', 't', lambda: {}, robot, {'boot_id': 'agt-boot-1'}, d, period_s=0,
                               require_safety_context=True, log=lambda _: None)
        fence = agent.contract_fence = TrackingFence(agent.contract_fence)
        robot.session_fn = agent.session_snapshot; robot.execution_allowed = agent.is_execution_allowed; robot.on_command_meta = agent.record_command_meta
        env = dict(server_boot_id='srv-boot-1', lease={'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 30}, reconciliation_required=False, commands=[])
        cmd = _make_cmd(sc=_valid_safety_context()); go2, at2, done2, done3 = (threading.Event() for _ in range(4)); n = {'s': 0}; gen = {}
        def post(path, body, timeout=None):
            if path.endswith('/hello'): return dict(server_boot_id='srv-boot-1', protocol_version=2)
            if path.endswith('/reconcile'): return {'reconciliation_required': False}
            if not path.endswith('/state'): return {'ok': True}
            n['s'] += 1
            if n['s'] == 1: return dict(env, safety_contract_version=1, commands=[cmd])
            if n['s'] == 2:
                at2.set()
                if not go2.wait(T): agent.stop(); raise RuntimeError('boundary not reached')
                r = dict(env); r.update(second); return r
            if n['s'] == 3:
                gen['after_invalid'] = agent.session_snapshot()['contract_generation']; done2.set()
                if not recover: agent.stop(); done3.set(); r = dict(env); r.update(second); return r
                return dict(env, safety_contract_version=1)
            gen['after_recovery'] = agent.session_snapshot()['contract_generation']; agent.stop(); done3.set()
            return dict(env, safety_contract_version=1)
        agent.http = SimpleNamespace(post=post)
        th = threading.Thread(target=agent.run, name='agent-run'); th.start()
        out = {}
        try:
            assert at2.wait(T)
            calls = {'n': 0}; orig = robot._verify_nav_preconditions
            def verify(c):
                calls['n'] += 1
                if calls['n'] == 2 and boundary == 'before-stage3':            # Stage 2 (outside the fence): the invalidation completes first
                    go2.set(); assert done2.wait(T)
                res = orig(c)
                if calls['n'] == 3 and boundary == 'inside-stage3':            # Stage 3 passed, fence held, send not yet called
                    assert res[0]
                    go2.set()
                    assert fence.waiting.wait(T), 'run loop never tried to enter the fence'
                    out['gen_while_blocked'] = agent.session_snapshot()['contract_generation']
                    out['invalid_seen_before_send'] = done2.is_set()
                return res
            robot._verify_nav_preconditions = verify
            if boundary == 'before-drain': go2.set(); assert done2.wait(T)
            robot._run_tasks()
            if boundary == 'after-send': go2.set()
            assert done2.wait(T) and done3.wait(T)
            out.update(sent=len(robot.nav.sent_goals), journal=agent.seen['cmd-1']['status'], detail=agent.seen['cmd-1'].get('detail'), **gen)
            if robot.nav.sent_goals:
                rec = robot.active; h = StubGoalHandle(uuid_hex=rec['goal_uuid'])
                robot.nav.sent_goals[0][2].set_result(h); robot._monitor_active_goal(); robot._monitor_active_goal()
                out.update(cancels=h.cancel_calls, reason=rec['fault_cancel_reason'])
            else: out.update(cancels=0, reason=None)
            out['waiters'] = list(fence.waiters)
            return out
        finally:
            go2.set(); th.join(T); assert not th.is_alive(), 'agent run loop did not terminate'

INVALID = {'safety_contract_version': None}
RECON = {'safety_contract_version': 1, 'reconciliation_required': True}

class SubmissionFence(unittest.TestCase):
    def test_A_invalidation_before_stage3_blocks_submission(self):
        for second in (INVALID, RECON):
            with self.subTest(second=second):
                r = threaded('before-stage3', second)
                self.assertEqual((r['sent'], r['journal']), (0, 'REJECTED')); self.assertIn('CONTRACT_GENERATION_CHANGED', r['detail'])
    def test_B_invalidation_attempted_inside_stage3_is_linearized_after_the_send(self):
        for second in (INVALID, RECON):
            with self.subTest(second=second):
                r = threaded('inside-stage3', second)
                self.assertEqual(r['gen_while_blocked'], 0)                    # the run loop was blocked at the fence: no change before the send
                self.assertFalse(r['invalid_seen_before_send'])
                self.assertIn('agent-run', r['waiters'])
                self.assertEqual(r['after_invalid'], 1)                         # it then happened, after the submission started
                self.assertEqual((r['sent'], r['cancels'], r['reason']), (1, 1, 'SERVER_CONTRACT_INVALID'))   # ... so the cancel duty holds (also after recovery)
    def test_C_invalidation_after_send_keeps_the_cancel_duty(self):
        for second in (INVALID, RECON):
            with self.subTest(second=second):
                r = threaded('after-send', second)
                self.assertEqual((r['sent'], r['cancels'], r['reason']), (1, 1, 'SERVER_CONTRACT_INVALID'))
    def test_D_reconciliation_flap_with_queued_command(self):
        r = threaded('before-drain', RECON)                                  # Codex: was 1 submission
        self.assertEqual((r['after_invalid'], r['sent'], r['journal']), (1, 0, 'REJECTED'))
    def test_no_recovery_variant(self):
        r = threaded('after-send', RECON, recover=False); self.assertEqual((r['sent'], r['cancels']), (1, 1))

class Sim:
    """Single-thread harness (as in the R1.4.3 tests) but `fields` may override reconciliation_required."""
    def __init__(self, replies, hooks=None):
        self.replies, self.hooks = replies, hooks or {}
        self.tmp = tempfile.TemporaryDirectory(); self.robot = _make_ready_robot()
        self.agent = core.AgentCore('robot-1', 'http://unused', 't', lambda: {}, self.robot, {'boot_id': 'agt-boot-1'}, self.tmp.name, period_s=0, require_safety_context=True, log=lambda _: None)
        r = self.robot; a = self.agent; r.session_fn = a.session_snapshot; r.execution_allowed = a.is_execution_allowed; r.on_command_meta = a.record_command_meta
        self.handle = StubGoalHandle(uuid_hex='11' * 16); self.n = 0; a.http = SimpleNamespace(post=self.post)
    def cmd(self, cid='cmd-1', **kw): return _make_cmd(cmd_id=cid, mission_id='m-' + cid, sc=_valid_safety_context(), **kw)
    def post(self, path, body, timeout=None):
        if path.endswith('/hello'): return dict(server_boot_id='srv-boot-1', protocol_version=2)
        if path.endswith('/reconcile'): return {'reconciliation_required': False}
        if not path.endswith('/state'): return {'ok': True}
        self.n += 1
        if self.n in self.hooks: self.hooks[self.n](self)
        env = dict(server_boot_id='srv-boot-1', lease={'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 60}, reconciliation_required=False, safety_contract_version=1, commands=[])
        if self.n > len(self.replies): self.agent.stop(); return env
        fields, cmds = self.replies[self.n - 1]; env.update(fields); env['commands'] = list(cmds); return env
    def run(self): self.agent.run(); return self
    def drain(self): self.robot._run_tasks()
    def monitor(self): self.robot._monitor_active_goal()
    def resolve(self): self.robot.nav.sent_goals[0][2].set_result(self.handle)
    @property
    def sent(self): return len(self.robot.nav.sent_goals)
    def journal(self, c='cmd-1'): return self.agent.seen.get(c, {}).get('status')

OK, RQ = {}, {'reconciliation_required': True}
class Reconciliation(unittest.TestCase):
    def test_E_active_goal_reconciliation_flap_then_monitor(self):
        s = Sim([(OK, []), (RQ, []), (OK, []), (OK, [])]); s.replies[0] = (OK, [s.cmd()])
        s.hooks[2] = lambda s: (s.drain(), s.resolve()); s.hooks[5] = lambda s: s.monitor(); s.run()     # monitor only after recovery
        self.assertEqual((s.sent, s.handle.cancel_calls, s.robot.active['fault_cancel_reason']), (1, 1, 'SERVER_CONTRACT_INVALID'))
        s.monitor(); s.monitor(); self.assertEqual(s.handle.cancel_calls, 1)                              # no repeated request
    def test_F_late_handle_reconciliation_flap(self):
        s = Sim([(OK, []), (RQ, []), (OK, [])]); s.replies[0] = (OK, [s.cmd()]); s.hooks[2] = lambda s: s.drain(); s.run()
        self.assertIsNone(s.robot.active['handle']); s.monitor(); self.assertEqual(s.handle.cancel_calls, 0)
        s.resolve(); self.assertEqual(s.handle.cancel_calls, 1)
    def test_G_repeated_mixed_withdrawals_never_replay_old_commands(self):
        s = Sim([(OK, []), (RQ, []), (OK, []), ({'safety_contract_version': None}, []), (OK, []), (RQ, []), (OK, [])])
        a, b = s.cmd('cmd-a', seq=1), s.cmd('cmd-b', seq=2)
        s.replies[0] = (OK, [a]); s.replies[2] = (OK, [b]); s.replies[6] = (OK, [a, b]); s.run(); s.drain()
        self.assertEqual(s.sent, 0); self.assertEqual((s.journal('cmd-a'), s.journal('cmd-b')), ('REJECTED', 'REJECTED'))
        self.assertGreaterEqual(s.agent.contract_generation, 3)
    def test_H_new_command_after_recovery_runs(self):
        s = Sim([(OK, []), (RQ, []), (OK, []), (OK, [])]); s.replies[2] = (OK, [s.cmd('cmd-new', seq=5)]); s.hooks[4] = lambda s: s.drain(); s.run()
        self.assertEqual(s.sent, 1)
    def test_H2_command_received_while_reconciliation_required_is_refused_for_good(self):
        s = Sim([(OK, []), (RQ, []), (OK, [])]); c = s.cmd(); s.replies[1] = (RQ, [c]); s.replies[2] = (OK, [c]); s.run(); s.drain()
        self.assertEqual((s.sent, s.journal()), (0, 'REJECTED'))
    def test_I_cancel_honoured_while_reconciliation_required(self):
        s = Sim([(OK, []), (RQ, []), (OK, [])]); s.replies[0] = (OK, [s.cmd()]); s.hooks[2] = lambda s: (s.drain(), s.resolve())
        s.replies[1] = (RQ, [_make_cmd(cmd_id='cancel-1', seq=3, type='cancel_mission', mission_id='m-cmd-1'), s.cmd('cmd-2', seq=2)])
        s.run(); s.drain(); s.monitor()
        self.assertGreaterEqual(s.handle.cancel_calls, 1); self.assertEqual(s.sent, 1)
        self.assertEqual(s.journal('cmd-2'), 'REJECTED'); self.assertNotEqual(s.journal('cancel-1'), 'REJECTED')

class Concurrency(unittest.TestCase):
    def test_J_no_deadlock_between_run_loop_ros_tasks_monitor_and_callbacks(self):
        with tempfile.TemporaryDirectory() as d:
            robot = _make_ready_robot()
            agent = core.AgentCore('robot-1', 'http://unused', 't', lambda: {}, robot, {'boot_id': 'agt-boot-1'}, d, period_s=0, require_safety_context=True, log=lambda _: None)
            robot.session_fn = agent.session_snapshot; robot.execution_allowed = agent.is_execution_allowed; robot.on_command_meta = agent.record_command_meta
            env = dict(server_boot_id='srv-boot-1', lease={'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 60})
            n = {'s': 0}; stop = threading.Event(); errors = []
            def post(path, body, timeout=None):
                if path.endswith('/hello'): return dict(server_boot_id='srv-boot-1', protocol_version=2)
                if path.endswith('/reconcile'): return {'reconciliation_required': False}
                if not path.endswith('/state'): return {'ok': True}
                n['s'] += 1; k = n['s']
                if k >= 400: agent.stop()
                cmds = [_make_cmd(cmd_id=f'c{k}', mission_id=f'm{k}', seq=k, sc=_valid_safety_context())] if k % 3 == 1 else []
                return dict(env, reconciliation_required=(k % 5 == 2), safety_contract_version=(None if k % 7 == 3 else 1), commands=cmds)
            agent.http = SimpleNamespace(post=post)
            def ros():
                try:
                    while not stop.is_set():
                        robot._run_tasks(); robot._monitor_active_goal()
                        for g in list(robot.nav.sent_goals):
                            fut = g[2]
                            if not fut._done:
                                rec = robot.active; fut.set_result(StubGoalHandle(uuid_hex=(rec or {}).get('goal_uuid') or '22' * 16))
                        a = robot.active
                        if a and a.get('handle') is not None and a['handle'].cancel_calls and not a['handle'].result_future._done:
                            a['handle'].result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
                except Exception as e: errors.append(repr(e))
            r = threading.Thread(target=ros, name='ros'); th = threading.Thread(target=agent.run, name='agent-run')
            r.start(); th.start(); th.join(20); stop.set(); r.join(20)
            self.assertFalse(th.is_alive(), 'run loop deadlocked'); self.assertFalse(r.is_alive(), 'ros thread deadlocked'); self.assertEqual(errors, [])
            self.assertGreaterEqual(n['s'], 400)

if __name__ == '__main__': unittest.main(verbosity=2)
