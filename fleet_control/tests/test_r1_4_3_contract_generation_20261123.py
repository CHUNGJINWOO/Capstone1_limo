"""R1.4.3: the approval of a motion command is fixed when it is RECEIVED, and the duty to cancel a live goal is latched when the contract is
invalidated; a later valid reply never revives either. Real AgentCore.run (single HTTP-style loop) + real RosRobot (stub ROS, stub Nav2
action client). The order of events is fixed by hooks that run inside the synthetic server's `post` at an exact point of the loop: hook N
runs when the Agent asks for state reply N, i.e. after reply N-1 was fully processed and before reply N is returned. There are no sleeps.
A cancel REQUEST is observed on the stub goal handle: nothing here shows that a real Nav2 goal ends or that a real robot stops.
Run (no ROS sourced): PYTHONPATH=server:agent/limo_robot_agent:tests python3 -m unittest test_r1_4_3_contract_generation_20261123"""
import sys, tempfile, time, unittest
from pathlib import Path
from types import SimpleNamespace
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'server'), str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'tests')]
import test_agent_safety_v1 as base
from test_agent_safety_v1 import _make_ready_robot, _make_cmd, _valid_safety_context, StubGoalHandle, GoalStatus, core

def tearDownModule(): base.tearDownModule()

OK, BAD = {'safety_contract_version': 1}, {}        # BAD: the required key is missing (the Codex counterexample); other invalid values behave alike

class Sim:
    def __init__(self, replies, hooks=None, *, rehello_at=(), lease_epoch=7):
        """replies: list of (contract_fields, [commands]); hooks: {n: fn(sim)}; rehello_at: reply numbers answered with needs_reconcile."""
        self.replies, self.hooks, self.rehello_at = replies, hooks or {}, set(rehello_at)
        self.tmp = tempfile.TemporaryDirectory(); self.robot = _make_ready_robot()
        self.agent = core.AgentCore('robot-1', 'http://unused', 't', lambda: {}, self.robot, {'boot_id': 'agt-boot-1'}, self.tmp.name,
                                    period_s=0, require_safety_context=True, log=lambda _: None)
        r = self.robot; a = self.agent
        r.session_fn = a.session_snapshot; r.execution_allowed = a.is_execution_allowed; r.on_command_meta = a.record_command_meta
        self.lease = {'owner': 'op-alice', 'epoch': lease_epoch, 'expires_at': time.time() + 60}
        self.handle = StubGoalHandle(uuid_hex='11' * 16); self.acks = []; self.n = 0; self.hellos = 0; self.snaps = {}
        a.http = SimpleNamespace(post=self.post)
    def cmd(self, cid='cmd-1', **kw): return _make_cmd(cmd_id=cid, mission_id='m-' + cid, sc=_valid_safety_context(), **kw)
    def post(self, path, body, timeout=None):
        if path.endswith('/hello'):
            self.hellos += 1; return dict(server_boot_id='srv-boot-1', protocol_version=2)
        if path.endswith('/reconcile'): return {'reconciliation_required': False}
        if '/commands/' in path and path.endswith('/ack'): self.acks.append((path.split('/')[-2], body.get('status'))); return {'ok': True}
        if not path.endswith('/state'): return {'ok': True}
        self.n += 1
        self.snaps[self.n] = self.agent.session_snapshot()
        if self.n in self.hooks: self.hooks[self.n](self)
        if self.n > len(self.replies): self.agent.stop(); return dict(server_boot_id='srv-boot-1', lease=dict(self.lease), reconciliation_required=False, **OK, commands=[])
        fields, cmds = self.replies[self.n - 1]
        env = dict(server_boot_id='srv-boot-1', lease=dict(self.lease), reconciliation_required=False, commands=list(cmds), **fields)
        if self.n in self.rehello_at: env['needs_reconcile'] = True
        return env
    def run(self):
        self.agent.run(); return self
    def drain(self): self.robot._run_tasks()
    def monitor(self): self.robot._monitor_active_goal()
    def resolve(self):                                   # the stub Nav2 accepts the goal (handle now exists)
        self.robot.nav.sent_goals[0][2].set_result(self.handle)
    @property
    def sent(self): return len(self.robot.nav.sent_goals)
    @property
    def cancels(self): return self.handle.cancel_calls
    def journal(self, cid='cmd-1'): return self.agent.seen.get(cid, {}).get('status')
    def done(self): self.tmp.cleanup()

class Receipt(unittest.TestCase):
    def test_1_invalid_reply_command_ros_runs_then_recovery(self):
        c = Sim([(OK, []), (BAD, []), (OK, [])]); cmd = c.cmd(); c.replies[1] = (BAD, [cmd]); c.replies[2] = (OK, [cmd])      # redelivered after recovery
        c.hooks[3] = lambda s: s.drain(); s = c.run(); s.drain()
        self.assertEqual((s.sent, s.journal()), (0, 'REJECTED')); self.assertEqual({st for cid, st in s.acks if cid == 'cmd-1'}, {'REJECTED'})   # a redelivery re-sends the stored terminal result (existing ACK-loss design), never another status
        s.done()
    def test_2_invalid_reply_command_recovery_then_ros_runs(self):
        c = Sim([(OK, []), (BAD, []), (OK, [])]); cmd = c.cmd(); c.replies[1] = (BAD, [cmd]); s = c.run()
        self.assertEqual(len(s.robot.tasks), 0)                         # never queued at all (was: queued RECEIVED, then executed)
        s.drain(); self.assertEqual((s.sent, s.journal()), (0, 'REJECTED')); s.done()
    def test_2b_command_queued_while_valid_dies_when_the_contract_flaps_before_ros_runs(self):
        c = Sim([(OK, []), (OK, []), (BAD, []), (OK, [])]); cmd = c.cmd(); c.replies[1] = (OK, [cmd]); s = c.run()   # ROS drains only at the end
        self.assertEqual(len(s.snaps), 5); self.assertGreater(s.snaps[5]['contract_generation'], s.snaps[2]['contract_generation'])
        s.drain(); self.assertEqual(s.sent, 0); self.assertEqual(s.journal(), 'REJECTED')
        self.assertIn('CONTRACT_GENERATION_CHANGED', s.agent.seen['cmd-1']['detail']); s.done()
    def test_2c_same_flap_but_ros_runs_in_time_still_executes(self):
        c = Sim([(OK, []), (OK, []), (BAD, []), (OK, [])]); cmd = c.cmd(); c.replies[1] = (OK, [cmd]); c.hooks[3] = lambda s: s.drain(); s = c.run()
        self.assertEqual(s.sent, 1); s.done()                          # control: a command executed under its own generation is unaffected

class ActiveGoal(unittest.TestCase):
    def active(self, hooks, replies):
        c = Sim(replies); cmd = c.cmd(); c.replies[0] = (OK, [cmd]); c.hooks.update(hooks)
        c.hooks[2] = lambda s: (s.drain(), s.resolve()); return c
    def test_3_invalidation_monitor_runs_then_recovery(self):
        c = self.active({3: lambda s: s.monitor()}, [(OK, []), (BAD, []), (OK, []), (OK, [])]); s = c.run()
        self.assertEqual((s.sent, s.cancels), (1, 1)); s.done()
    def test_4_invalidation_recovery_then_monitor_runs(self):
        seen = {}
        def late(s): seen['before'] = (s.cancels, s.agent.session_snapshot()['safety_contract_version']); s.monitor()
        c = self.active({5: late}, [(OK, []), (BAD, []), (OK, []), (OK, []), (OK, [])]); s = c.run()
        self.assertEqual(seen['before'], (0, 1))                         # contract is valid again, nothing cancelled yet ...
        self.assertEqual((s.sent, s.cancels), (1, 1))                    # ... and the cancel is still issued (was: 0)
        s.monitor(); s.monitor(); self.assertEqual(s.cancels, 1)         # no repeat request while one is in progress
        self.assertEqual(s.robot.active['fault_cancel_reason'], 'SERVER_CONTRACT_INVALID'); s.done()
    def test_4b_later_valid_replies_never_clear_the_obligation_or_resume_the_goal(self):
        c = self.active({}, [(OK, []), (BAD, []), (OK, []), (OK, []), (OK, [])]); s = c.run()
        for _ in range(3): s.monitor()
        self.assertEqual((s.sent, s.cancels), (1, 1)); self.assertTrue(s.robot.active['cancel_requested']); s.done()
    def test_5_goal_without_handle_gets_the_cancel_when_the_handle_arrives(self):
        c = Sim([(OK, []), (BAD, []), (OK, []), (OK, [])]); cmd = c.cmd(); c.replies[0] = (OK, [cmd]); c.hooks[2] = lambda s: s.drain()   # submitted, not yet accepted
        s = c.run(); self.assertIsNone(s.robot.active['handle']); s.monitor()
        self.assertTrue(s.robot.active['cancel_requested']); self.assertEqual(s.cancels, 0)                   # nothing to cancel yet
        s.resolve(); self.assertEqual(s.cancels, 1); s.done()                                                  # the late acceptance is cancelled at once
    def test_5b_goal_finished_before_invalidation_leaves_no_obligation_for_the_next_goal(self):
        c = Sim([(OK, []), (OK, []), (BAD, []), (OK, []), (OK, [])]); first = c.cmd('cmd-1'); c.replies[0] = (OK, [first])
        def finish(s):
            s.drain(); s.resolve(); s.handle.result_future.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
        c.hooks[2] = finish
        def second(s): s.replies[3] = (OK, [s.cmd('cmd-2', seq=2)])
        c.hooks[3] = lambda s: second(s); c.hooks[6] = lambda s: s.drain()
        s = c.run(); s.drain(); self.assertEqual(s.cancels, 0)
        self.assertEqual(s.sent, 2); self.assertNotEqual((s.robot.active or {}).get('fault_cancel_reason'), 'SERVER_CONTRACT_INVALID'); s.done()

class Sequences(unittest.TestCase):
    def test_6_repeated_flapping_and_redelivery(self):
        c = Sim([(OK, []), (BAD, []), (OK, []), (BAD, []), (OK, [])]); a, b, d = c.cmd('cmd-a', seq=1), c.cmd('cmd-b', seq=2), c.cmd('cmd-c', seq=3)
        c.replies[0] = (OK, [a]); c.replies[1] = (BAD, [b]); c.replies[2] = (OK, [b, d]); c.replies[3] = (BAD, []); c.replies[4] = (OK, [a, b, d])
        s = c.run(); s.drain()                                                       # ROS only runs at the very end
        self.assertEqual(s.sent, 0)
        for cid in ('cmd-a', 'cmd-b', 'cmd-c'): self.assertEqual(s.journal(cid), 'REJECTED', cid)
        for cid in ('cmd-a', 'cmd-b', 'cmd-c'): self.assertLessEqual({st for c_, st in s.acks if c_ == cid}, {'REJECTED'}, f'conflicting ACK status for {cid}')   # (an ACK still pending when the loop stops is not yet posted)
        s.done()
    def test_6b_new_command_in_a_later_valid_generation_executes_once(self):
        c = Sim([(OK, []), (BAD, []), (OK, []), (OK, [])]); b, d = c.cmd('cmd-b', seq=2), c.cmd('cmd-c', seq=3)
        c.replies[1] = (BAD, [b]); c.replies[2] = (OK, [b, d]); c.hooks[4] = lambda s: s.drain(); s = c.run(); s.drain()
        self.assertEqual(s.sent, 1); self.assertEqual(s.journal('cmd-b'), 'REJECTED'); self.assertNotEqual(s.journal('cmd-c'), 'REJECTED'); s.done()
    def test_6c_rehello_and_boot_style_reset_kills_older_approval(self):
        c = Sim([(OK, []), (OK, []), (OK, [])], rehello_at=(2,)); cmd = c.cmd(); c.replies[0] = (OK, [cmd]); s = c.run(); s.drain()
        self.assertEqual(s.hellos, 2); self.assertEqual((s.sent, s.journal()), (0, 'REJECTED')); s.done()
    def test_6d_lease_epoch_change_still_refused(self):
        c = Sim([(OK, []), (OK, [])], lease_epoch=8); cmd = c.cmd(); c.replies[0] = (OK, [cmd]); s = c.run(); s.drain()     # command admitted under epoch 7
        self.assertEqual(s.sent, 0); s.done()
    def test_7_cancel_is_honoured_under_an_invalid_contract_and_never_enables_motion(self):
        c = Sim([(OK, []), (BAD, []), (OK, [])]); first = c.cmd('cmd-1'); newer = c.cmd('cmd-2', seq=2)
        c.replies[0] = (OK, [first]); c.hooks[2] = lambda s: (s.drain(), s.resolve())
        c.replies[1] = (BAD, [_make_cmd(cmd_id='cancel-1', seq=3, type='cancel_mission', mission_id='m-cmd-1'), newer])
        s = c.run(); s.drain(); s.monitor()
        self.assertGreaterEqual(s.cancels, 1); self.assertEqual(s.sent, 1)                  # the explicit cancel reached the goal; the new motion command did not run
        self.assertEqual(s.journal('cmd-2'), 'REJECTED'); self.assertNotEqual(s.journal('cancel-1'), 'REJECTED'); s.done()

class Unit(unittest.TestCase):
    def test_generation_counts_only_valid_to_invalid_transitions(self):
        with tempfile.TemporaryDirectory() as d:
            a = core.AgentCore('robot-1', 'http://unused', 't', lambda: {}, None, {'boot_id': 'a'}, d, log=lambda _: None)
            a.server_boot = 'srv'; a.server_protocol_boot = 'srv'; a.server_protocol_version = 2; a.server_reconciliation_required = False; a.connected = True; a.lease = {'owner': 'op', 'epoch': 1, 'expires_at': time.time() + 300}   # a reconciled, connected session with a lease
            with a.lock:
                a._contract_note_locked(); self.assertEqual(a.contract_generation, 0)      # still invalid (no safety version): not a transition
                a.server_safety_contract_version = 1; a._contract_note_locked(); self.assertEqual(a.contract_generation, 0)
                a.server_safety_contract_version = None; a._contract_note_locked(); self.assertEqual(a.contract_generation, 1)
                a._contract_note_locked(); self.assertEqual(a.contract_generation, 1)      # staying invalid does not count again
                a.server_safety_contract_version = 1; a._contract_note_locked(); self.assertEqual(a.contract_generation, 1)
            self.assertEqual(a.session_snapshot()['contract_generation'], 1)

if __name__ == '__main__': unittest.main(verbosity=2)
