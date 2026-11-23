"""G2 (2026-11-23): UNKNOWN Mapping sessions - authorization vs tracking, and late Worker results (decided: origin/main meaning).

- An UNKNOWN (or RECONCILING) session never owns a Frontier goal for authorization; an active goal of such a session blocks
  the robot (reconciliation_required -> the Agent withdraws authority and cancels it) and its identity evidence is kept.
- A result that arrives after the session became UNKNOWN is refused with 409, rewrites nothing and leaves a diagnostic event;
  a replay of an already settled result stays idempotent.
In-memory server, no network, no ROS.
"""
import json
import time
import unittest

from test_web_mapping_session import MappingSessionTests as _Src, OP


class Base(unittest.TestCase):          # the v5b fixture and helpers only (the v5b tests are not re-run here)
    setUp, fresh, req, worker, complete, start = (_Src.setUp, _Src.fresh, _Src.req, _Src.worker, _Src.complete, _Src.start)


del _Src
from fleet_server.service import ApiError


class UnknownSessionGoal(Base):
    def _reserved(self):
        sid = self.start()
        g = 'c' * 32
        self.assertTrue(self.worker('reserve_goal', session_id=sid, goal_id=g)['reserved'])
        return sid, {'owner': 'frontier', 'mission_id': sid, 'command_id': g, 'goal_uuid': 'd' * 32, 'goal_handle_available': True}

    def test_live_session_owns_and_does_not_block(self):
        sid, active = self._reserved()
        self.f.check_execution('R', {'active': active, 'nav2_goals': {}})
        self.assertFalse(self.f.robots['R'].get('reconciliation_required'))
        self.assertTrue(self.f.owns_mapping_goal('R', self.boot, active))
        self.assertIsNone(self.f.tracked_mapping_goal('R', self.boot, active))

    def test_unknown_session_goal_blocks_the_robot_and_keeps_its_identity(self):
        sid, active = self._reserved()
        self.f.db.x('UPDATE locks SET expires_at=? WHERE robot_id=?', (time.time() - 1, 'R'))
        self.assertEqual(self.f.operation_status('R')['state'], 'UNKNOWN')
        self.fresh()
        self.f.check_execution('R', {'active': active, 'nav2_goals': {}})
        # not a robot-wide block: the UNKNOWN session already forbids new motion and the operator must still be able to take the
        # Lease and RECONCILE (E2E f03). The Agent cancels the goal by itself (Mapping Worker / R1 Frontier duty).
        self.assertFalse(self.f.robots['R'].get('reconciliation_required'))
        self.assertTrue(self.f.mapping_blocks_motion('R'))
        ev = self.f.db.one("SELECT data_json FROM events WHERE kind='mapping_goal_untrusted' AND robot_id='R'")
        data = json.loads(ev['data_json'])
        self.assertEqual((data['session_id'], data['boot'], data['command_id'], data['goal_uuid'], data['state']),
                         (sid, self.boot, active['command_id'], active['goal_uuid'], 'UNKNOWN'))
        self.assertIsNone(self.f.db.one("SELECT 1 AS x FROM events WHERE kind='ownership_mismatch' AND robot_id='R'"))
        # reported again: one diagnostic event, the block stays
        self.f.check_execution('R', {'active': active, 'nav2_goals': {}})
        self.assertEqual(self.f.db.one("SELECT COUNT(*) AS n FROM events WHERE kind='mapping_goal_untrusted'")['n'], 1)

    def test_operator_can_retake_the_lease_while_the_goal_is_tracked(self):
        """E2E f03 deadlock: a robot-wide reconciliation block would refuse the Lease, and RECONCILE needs the Lease."""
        sid, active = self._reserved()
        self.f.db.x('UPDATE locks SET expires_at=? WHERE robot_id=?', (time.time() - 1, 'R'))
        self.fresh()
        self.f.check_execution('R', {'active': active, 'nav2_goals': {}})
        lease = self.f.lease_action('R', OP, 'acquire', 30)
        self.assertEqual(lease['owner'], OP.name)
        self.assertTrue(self.f.mapping_blocks_motion('R'))                            # still no new motion

    def test_unknown_session_cannot_start_reserve_or_unblock_motion(self):
        sid, active = self._reserved()
        self.f.db.x('UPDATE locks SET expires_at=? WHERE robot_id=?', (time.time() - 1, 'R'))
        self.assertEqual(self.f.operation_status('R')['state'], 'UNKNOWN')
        self.assertTrue(self.f.mapping_blocks_motion('R'))
        with self.assertRaises(ApiError):
            self.worker('reserve_goal', session_id=sid, goal_id='e' * 32)
        with self.assertRaises(ApiError):
            self.req('START')

    def test_foreign_goal_is_still_the_generic_ownership_fault(self):
        sid, active = self._reserved()
        self.f.db.x('UPDATE locks SET expires_at=? WHERE robot_id=?', (time.time() - 1, 'R'))
        self.fresh()
        self.f.check_execution('R', {'active': dict(active, command_id='f' * 32), 'nav2_goals': {}})
        self.assertIs(self.f.robots['R']['reconciliation_required'], True)
        self.assertIsNotNone(self.f.db.one("SELECT 1 AS x FROM events WHERE kind='ownership_mismatch' AND robot_id='R'"))


class LateResult(Base):
    def test_result_after_unknown_is_409_and_rewrites_nothing(self):
        sid = self.start()
        out = self.req('PAUSE', sid)
        c = self.worker('poll')['command']
        ref = dict(session_id=c['session_id'], command_id=c['command_id'])
        self.assertTrue(self.worker('accept', **ref)['accepted'])
        self.f.robots['R']['state_ts'] = time.time() - 30                       # Worker evidence lost
        before = self.f.operation_status('R')
        self.assertEqual((before['state'], before['command']['status']), ('UNKNOWN', 'UNKNOWN'))
        with self.assertRaises(ApiError) as cm:
            self.worker('result', **ref, outcome='OK', evidence=dict(frontier_paused=True, goal_terminal=True))
        self.assertEqual((cm.exception.status, cm.exception.code), (409, 'operation_not_active'))
        after = self.f.operation_status('R')
        self.assertEqual((after['state'], after['command']['status'], after.get('result')),
                         (before['state'], before['command']['status'], before.get('result')))
        ev = json.loads(self.f.db.one("SELECT data_json FROM events WHERE kind='mapping_late_result'")['data_json'])
        self.assertEqual((ev['session_id'], ev['command_id'], ev['outcome']), (sid, out['command_id'], 'OK'))

    def test_mismatched_session_command_or_boot_is_refused(self):
        sid = self.start()
        out = self.req('PAUSE', sid)
        c = self.worker('poll')['command']
        ref = dict(session_id=c['session_id'], command_id=c['command_id'])
        self.worker('accept', **ref)
        for bad in (dict(ref, session_id='x' * 32), dict(ref, command_id='y' * 32)):
            with self.subTest(bad=bad), self.assertRaises(ApiError) as cm:
                self.worker('result', **bad, outcome='OK', evidence={})
            self.assertEqual(cm.exception.code, 'operation_mismatch')
        with self.assertRaises(ApiError):
            self.f.worker_operation('R', 'other-boot', dict(kind='mapping', action='result', **ref, outcome='OK', evidence={}))

    def test_settled_result_replay_stays_idempotent(self):
        sid = self.start()
        out = self.req('PAUSE', sid)
        first = self.complete(out, dict(frontier_paused=True, goal_terminal=True))
        again = self.worker('result', session_id=sid, command_id=out['command_id'], outcome='UNKNOWN', evidence={})
        self.assertEqual((again['state'], again['result']), (first['state'], first['result']))

    def test_stop_still_completes_normally(self):
        sid = self.start()
        out = self.req('STOP', sid)
        s = self.complete(out, dict(frontier_stopped=True, goal_terminal=True))
        self.assertEqual(s['state'], 'STOPPED')


if __name__ == '__main__':
    unittest.main()
