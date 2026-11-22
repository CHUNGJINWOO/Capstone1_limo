"""R1.4.1 P1-2: an Agent whose contract became incompatible must still be able to receive, accept and report the STOP of the mapping
session that already belongs to its own boot, and nothing else. Server only (no ROS, sockets or hardware). Uses the Codex R1.4 re-audit
scenario (server-probe.py: legacy Agent + pre-existing MAPPING session + operator STOP). Physical stopping is NOT verified here: this proves
only that the server delivers and records the STOP.
Run: PYTHONPATH=server:agent/limo_robot_agent:tests python3 -m unittest test_r1_4_1_mapping_stop_20261123"""
import sys, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'server'), str(ROOT / 'agent' / 'limo_robot_agent'), str(ROOT / 'tests')]
from test_r1_4_safety_contract_hardening_20261123 import ADMIN, ApiError, CFG, DB, Fleet, _idle_state

EVID = {'frontier_stopped': True, 'goal_terminal': True}
def worker(f, action, boot, **kw): return f.worker_operation('R', boot, dict(kind='mapping', action=action, **kw))
def code(call):
    try: call(); return None
    except ApiError as e: return e.code

class MappingStop(unittest.TestCase):
    def hello(self, boot, proto=2, safety=1):
        agent = {'boot_id': boot, 'mode': 'sim', 'simulated': True, 'mapping_worker': True}
        if proto is not None: agent['protocol_version'] = proto
        if safety is not None: agent['safety_contract_version'] = safety
        self.f.hello('R', agent); self.f.reconcile('R', boot, {}, {}, True); self.f.ingest_state('R', self.seq, _idle_state(), boot); self.seq += 1
    def setUp(self):
        self.f = Fleet(DB(':memory:'), CFG); self.seq = 1; self.hello('wb1')
        self.epoch = self.f.lease_action('R', ADMIN, 'acquire', 120)['epoch']
        self.f._write_operation(dict(robot_id='R', kind='mapping', session_id='session1', state='MAPPING', boot='wb1', server_boot=self.f.boot_id,
                                     owner='admin', epoch=self.epoch, command={'status': 'COMPLETED'}))
    def req(self, action, key='k'): return self.f.request_operation('R', 'admin', {'action': action, 'session_id': 'session1', 'epoch': self.epoch, 'idempotency_key': key})
    def status(self): return self.f.operation_status('R')

    # ---- normal contract (unchanged behaviour)
    def test_valid_contract_delivers_accepts_and_records_stop(self):
        self.assertEqual(self.req('STOP')['status'], 'QUEUED')
        cmd = worker(self.f, 'poll', 'wb1', session_id='session1')['command']; self.assertEqual((cmd['action'], cmd['status']), ('STOP', 'QUEUED'))
        self.assertTrue(worker(self.f, 'accept', 'wb1', command_id=cmd['command_id'], session_id='session1')['accepted'])
        s = worker(self.f, 'result', 'wb1', command_id=cmd['command_id'], session_id='session1', outcome='OK', evidence=EVID)
        self.assertEqual(s['state'], 'STOPPED')
    # ---- incompatible contract: STOP only
    def incompatible(self):
        """The queued STOP exists first (admitted while compatible, or by the operator path that allows STOP); then the Agent re-hellos incompatibly."""
        self.req('STOP'); self.hello('wb1', proto=1)
        self.assertFalse(self.f.agent_contract_ok('R'))
    def test_incompatible_agent_receives_its_own_session_stop(self):
        self.incompatible()
        r = worker(self.f, 'poll', 'wb1', session_id='session1')
        self.assertEqual(r['command']['action'], 'STOP'); self.assertIs(r['renew'], False)
    def test_incompatible_agent_acks_and_reports_stop_and_server_records_it(self):
        self.incompatible()
        cmd = worker(self.f, 'poll', 'wb1', session_id='session1')['command']
        self.assertTrue(worker(self.f, 'accept', 'wb1', command_id=cmd['command_id'], session_id='session1')['accepted'])
        s = worker(self.f, 'result', 'wb1', command_id=cmd['command_id'], session_id='session1', outcome='OK', evidence=EVID)
        self.assertEqual((s['state'], s['result']['status']), ('STOPPED', 'COMPLETED'))
        events = self.f.db.q("SELECT * FROM events WHERE kind='mapping_result'"); self.assertTrue(any('STOP' in e['message'] for e in events))
    def test_duplicate_stop_is_idempotent(self):
        self.incompatible()
        cmd = worker(self.f, 'poll', 'wb1', session_id='session1')['command']; kw = dict(command_id=cmd['command_id'], session_id='session1')
        self.assertTrue(worker(self.f, 'accept', 'wb1', **kw)['accepted']); self.assertFalse(worker(self.f, 'accept', 'wb1', **kw)['accepted'])
        self.assertIsNone(worker(self.f, 'poll', 'wb1', session_id='session1')['command'])         # accepted STOP is not offered again
        first = worker(self.f, 'result', 'wb1', outcome='OK', evidence=EVID, **kw)
        again = worker(self.f, 'result', 'wb1', outcome='FAILED', evidence={}, **kw)                # a late duplicate never rewrites the first result
        self.assertEqual(again['result'], first['result']); self.assertEqual(again['state'], 'STOPPED')
        self.assertIsNone(worker(self.f, 'poll', 'wb1', session_id='session1')['command'])
    def test_other_commands_and_actions_stay_blocked(self):
        self.req('PAUSE', 'p'); self.hello('wb1', proto=1)                                         # a queued PAUSE (not STOP) must not be delivered
        self.assertIsNone(worker(self.f, 'poll', 'wb1', session_id='session1')['command'])
        cid = self.status()['command']['command_id']
        self.assertEqual(code(lambda: worker(self.f, 'accept', 'wb1', command_id=cid, session_id='session1')), 'agent_protocol')
        self.assertEqual(code(lambda: worker(self.f, 'result', 'wb1', command_id=cid, session_id='session1', outcome='OK', evidence={'frontier_paused': True, 'goal_terminal': True})), 'agent_protocol')
        self.assertEqual(code(lambda: worker(self.f, 'reserve_goal', 'wb1', session_id='session1', goal_id='a' * 32)), 'agent_protocol')
        for action in ('START', 'RESUME', 'PAUSE', 'SAVE'):                                         # operators cannot create them either
            self.assertIsNotNone(code(lambda a=action: self.req(a, 'x' + a)), action)
    def test_incompatible_poll_never_renews_the_session(self):
        self.hello('wb1', proto=1)
        self.assertIs(worker(self.f, 'poll', 'wb1', session_id='session1')['renew'], False)
    def test_old_boot_and_foreign_session_cannot_use_the_exception(self):
        self.incompatible()
        cid = worker(self.f, 'poll', 'wb1', session_id='session1')['command']['command_id']
        self.hello('wb2', proto=1)                                                                  # newer incompatible Agent boot
        self.assertEqual(code(lambda: worker(self.f, 'poll', 'wb1', session_id='session1')), 'agent_session_mismatch')       # old boot refused
        self.assertIsNone(worker(self.f, 'poll', 'wb2', session_id='session1')['command'])         # the STOP belongs to wb1's session
        self.assertEqual(code(lambda: worker(self.f, 'accept', 'wb2', command_id=cid, session_id='session1')), 'agent_protocol')
        self.hello('wb1x', proto=1); s = self.status()
        self.assertEqual(code(lambda: worker(self.f, 'accept', 'wb1x', command_id='nope', session_id='other')), 'agent_protocol')
    def test_wrong_command_or_session_id_is_refused_after_acceptance_path(self):
        self.incompatible(); cid = worker(self.f, 'poll', 'wb1', session_id='session1')['command']['command_id']
        self.assertEqual(code(lambda: worker(self.f, 'accept', 'wb1', command_id='forged', session_id='session1')), 'operation_mismatch')
        self.assertEqual(code(lambda: worker(self.f, 'accept', 'wb1', command_id=cid, session_id='other')), 'operation_mismatch')
    def test_stop_delivery_failures_are_explicit_and_leave_the_session(self):
        self.incompatible()
        self.f.robots['R']['state_ts'] -= 60                                                         # worker silent: explicit error, session untouched
        self.assertEqual(code(lambda: worker(self.f, 'poll', 'wb1', session_id='session1')), 'worker_unavailable')
        self.assertEqual(self.status()['state'], 'UNKNOWN')                                          # existing fail-closed rule; never reported as STOPPED
        self.f.robots['R']['agent'].update(mode='real', simulated=False)                             # real robot: mapping stays blocked
        self.assertEqual(code(lambda: worker(self.f, 'poll', 'wb1', session_id='session1')), 'mapping_real_blocked')
    def test_incompatible_agent_cannot_start_a_new_session(self):
        self.f._write_operation(dict(robot_id='R', kind='mapping', session_id=None, state='IDLE', boot=None, server_boot=self.f.boot_id, owner=None, epoch=None, command={'status': 'COMPLETED'}))
        self.hello('wb1', proto=1)
        self.assertIsNotNone(code(lambda: self.f.request_operation('R', 'admin', {'action': 'START', 'epoch': self.epoch, 'idempotency_key': 's', 'mode': 'NEW_MAP'})))

if __name__ == '__main__': unittest.main(verbosity=2)
