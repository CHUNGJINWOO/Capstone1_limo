"""R1.4.6: no motion approval may be completed from a previous reply. Every field the Agent needs for movement is re-read from the CURRENT
reply of the type that carries it; missing, null, wrong type or a wrong/old value withdraws the authority (new generation), a queued
command is never submitted and a live goal keeps its cancel duty, also after a later valid reply. Real AgentCore.run + RosRobot (stub
ROS / Nav2), the same single-thread hook harness as R1.4.4/R1.4.5 and their threaded fence harness. Cancel REQUESTS on stub handles are
observed, not real stopping.
Run (no ROS sourced): PYTHONPATH=server:agent/limo_robot_agent:tests python3 -m unittest test_r1_4_6_required_fields_20261123"""
import sys, time, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'server'), str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'tests')]
import test_agent_safety_v1 as base
from test_agent_safety_v1 import _make_cmd
from test_r1_4_5_withdrawal_paths_20261123 import Sim, MISSING
from test_r1_4_4_submission_fence_20261123 import threaded

def tearDownModule(): base.tearDownModule()
NOW = time.time()
def LEASE(**kw):
    d = {'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 60}; d.update(kw); return d

# State reply: field -> list of (label, bad value). Every value must withdraw. (lease: ownership.lease_view, sent in every state reply.)
STATE_MATRIX = {
    'lease': [('missing', MISSING), ('null', None), ('list', []), ('string', 'op-alice'), ('empty object', {}),
              ('no owner', {'epoch': 7, 'expires_at': NOW + 60}), ('empty owner', LEASE(owner='')), ('owner int', LEASE(owner=5)),
              ('epoch string', LEASE(epoch='7')), ('epoch bool', LEASE(epoch=True)), ('epoch float', LEASE(epoch=7.0)), ('epoch negative', LEASE(epoch=-1)),
              ('no expiry', {'owner': 'op-alice', 'epoch': 7}), ('expiry string', LEASE(expires_at='later')), ('expiry nan', LEASE(expires_at=float('nan'))),
              ('expired (old value)', LEASE(expires_at=NOW - 1)), ('other epoch (old session)', LEASE(epoch=6)), ('other owner', LEASE(owner='mallory'))],
    'safety_contract_version': [('missing', MISSING), ('null', None), ('bool', True), ('float', 1.0), ('string', '1'), ('wrong', 2)],
    'reconciliation_required': [('missing', MISSING), ('null', None), ('int', 0), ('string', 'false'), ('list', []), ('true', True)],
    'protocol_version': [('null', None), ('bool', True), ('float', 2.0), ('string', '2'), ('old', 1)],        # optional in state replies: may only withdraw
    'server_boot_id': [('missing', MISSING), ('null', None), ('int', 5), ('old boot', 'srv-boot-0')],          # -> reset + re-hello
}

class StateReplyMatrix(unittest.TestCase):
    def test_queued_command_is_never_submitted(self):
        for field, cases in STATE_MATRIX.items():
            for label, bad in cases:
                with self.subTest(field=field, value=label):
                    s = Sim([({}, []), ({field: bad}, []), ({}, [])]); s.replies[0] = ({}, [s.cmd()]); s.run(); s.drain()   # ROS runs after recovery
                    self.assertEqual(s.sent, 0); self.assertEqual(s.journal(), 'REJECTED')
    def test_live_goal_keeps_its_cancel_duty_across_recovery(self):
        for field, cases in STATE_MATRIX.items():
            for label, bad in cases:
                with self.subTest(field=field, value=label):
                    s = Sim([({}, []), ({field: bad}, []), ({}, []), ({}, [])]); s.replies[0] = ({}, [s.cmd()])
                    s.hooks[2] = lambda s: (s.drain(), s.resolve()); s.hooks[5] = lambda s: s.monitor(); s.run()   # monitor only after recovery
                    self.assertEqual((s.sent, s.handle.cancel_calls), (1, 1))
    def test_command_delivered_with_the_bad_reply_is_refused_for_good(self):
        for field, cases in STATE_MATRIX.items():
            for label, bad in cases:
                with self.subTest(field=field, value=label):
                    s = Sim([({}, []), ({field: bad}, []), ({}, [])]); c = s.cmd(); s.replies[1] = ({field: bad}, [c]); s.replies[2] = ({}, [c])
                    s.hooks[3] = lambda s: setattr(s, 'after_bad', dict(s.agent.seen)); s.hooks[4] = lambda s: s.drain(); s.run(); s.drain()
                    if field == 'server_boot_id':
                        # a reply from another/unknown server boot is discarded as a whole (reset + re-hello): its commands are never
                        # received; the same command delivered later by a valid reply of the current boot is a new, valid delivery
                        self.assertNotIn('cmd-1', s.after_bad)
                    else:
                        self.assertEqual(s.sent, 0); self.assertEqual(s.after_bad['cmd-1']['status'], 'REJECTED')
    def test_valid_replies_still_work(self):
        s = Sim([({}, []), ({'lease': LEASE(expires_at=time.time() + 120)}, [])]); s.replies[1] = ({'lease': LEASE(expires_at=time.time() + 120)}, [s.cmd()])  # renewal keeps the epoch
        s.hooks[3] = lambda s: s.drain(); s.run(); self.assertEqual(s.sent, 1)

class LeaseScenarios(unittest.TestCase):
    def test_A_missing_lease_queued_goal_not_submitted(self):                       # Codex: submitted 1
        s = Sim([({}, []), ({'lease': MISSING}, [])]); s.replies[0] = ({}, [s.cmd()]); s.hooks[3] = lambda s: s.drain(); s.run()
        self.assertEqual((s.sent, s.journal()), (0, 'REJECTED')); self.assertGreaterEqual(s.agent.contract_generation, 1)
    def test_B_missing_lease_live_goal_cancelled(self):                              # Codex: 0 cancel requests
        s = Sim([({}, []), ({'lease': MISSING}, [])]); s.replies[0] = ({}, [s.cmd()]); s.hooks[2] = lambda s: (s.drain(), s.resolve()); s.hooks[3] = lambda s: s.monitor(); s.run()
        self.assertEqual((s.sent, s.handle.cancel_calls, s.robot.active['fault_cancel_reason']), (1, 1, 'SERVER_CONTRACT_INVALID'))
    def test_C_D_recovery_does_not_revive_old_but_runs_new(self):
        s = Sim([({}, []), ({'lease': MISSING}, []), ({}, [])]); old = s.cmd('cmd-old', seq=1); new = s.cmd('cmd-new', seq=2)
        s.replies[0] = ({}, [old]); s.replies[2] = ({}, [old, new]); s.hooks[4] = lambda s: s.drain(); s.run(); s.drain()
        self.assertEqual(s.journal('cmd-old'), 'REJECTED'); self.assertEqual(s.sent, 1); self.assertNotEqual(s.journal('cmd-new'), 'REJECTED')
    def test_E_null_lease_unchanged(self):
        s = Sim([({}, []), ({'lease': None}, [])]); s.replies[0] = ({}, [s.cmd()]); s.hooks[3] = lambda s: s.drain(); s.run(); self.assertEqual(s.sent, 0)
    def test_G_lease_epoch_change_refuses_the_old_command(self):
        s = Sim([({}, []), ({'lease': LEASE(epoch=8)}, [])]); s.replies[0] = ({}, [s.cmd()]); s.hooks[3] = lambda s: s.drain(); s.run(); self.assertEqual(s.sent, 0)
    def test_I_cancel_honoured_without_a_lease(self):
        s = Sim([({}, []), ({'lease': MISSING}, [])]); s.replies[0] = ({}, [s.cmd()]); s.hooks[2] = lambda s: (s.drain(), s.resolve())
        s.replies[1] = ({'lease': MISSING}, [_make_cmd(cmd_id='cancel-1', seq=3, type='cancel_mission', mission_id='m-cmd-1'), s.cmd('cmd-2', seq=2)])
        s.run(); s.drain(); s.monitor()
        self.assertGreaterEqual(s.handle.cancel_calls, 1); self.assertEqual(s.journal('cmd-2'), 'REJECTED'); self.assertNotEqual(s.journal('cancel-1'), 'REJECTED')

class LeaseFence(unittest.TestCase):
    def test_H_lease_withdrawn_before_and_inside_stage3(self):
        for second in ({'lease': None}, {'lease': []}, {'lease': LEASE(epoch='7')}):
            with self.subTest(second=second):
                a = threaded('before-stage3', second); self.assertEqual(a['sent'], 0)
                b = threaded('inside-stage3', second)
                self.assertEqual(b['gen_while_blocked'], 0); self.assertFalse(b['invalid_seen_before_send']); self.assertIn('agent-run', b['waiters'])
                self.assertEqual((b['sent'], b['cancels'], b['reason']), (1, 1, 'SERVER_CONTRACT_INVALID'))
                c = threaded('after-send', second); self.assertEqual((c['sent'], c['cancels']), (1, 1))

class HelloAndReconcile(unittest.TestCase):
    def run_with(self, hello=None, reconcile=None):
        s = Sim([({}, [])]); c = s.cmd(); s.replies[0] = ({}, [c]); orig = s.post
        def post(path, body, timeout=None):
            if path.endswith('/hello') and hello is not None: return dict(hello)
            if path.endswith('/reconcile') and reconcile is not None: return dict(reconcile)
            return orig(path, body, timeout)
        s.agent.http.post = post; s.hooks[2] = lambda s: s.drain(); s.run(); return s
    def test_hello_server_boot_and_protocol_required(self):
        for label, h in [('no boot', {'protocol_version': 2}), ('null boot', {'server_boot_id': None, 'protocol_version': 2}),
                         ('no protocol', {'server_boot_id': 'srv-boot-1'}), ('protocol str', {'server_boot_id': 'srv-boot-1', 'protocol_version': '2'})]:
            with self.subTest(hello=label): self.assertEqual(self.run_with(hello=h).sent, 0)
    def test_reconcile_flag_alone_never_grants(self):
        for label, r in [('missing', {}), ('null', {'reconciliation_required': None}), ('true', {'reconciliation_required': True})]:
            with self.subTest(reconcile=label):
                s = self.run_with(reconcile=r)
                self.assertEqual(s.sent, 1)   # the CURRENT state reply carries exact False: authority comes from it, not from the reconcile reply
    def test_valid_hello_and_reconcile(self):
        self.assertEqual(self.run_with().sent, 1)

if __name__ == '__main__': unittest.main(verbosity=2)
