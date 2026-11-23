"""G2 (2026-11-23): a mapping-owned (Frontier) NavigateToPose goal needs the R1 session authority.

Before G2 the Frontier path (RosRobot._do_frontier) checked only execution_allowed() (connected + fresh state): it sent a goal
without a lease, with reconciliation pending or with an invalid server contract, it did not use the submission fence and an
active Frontier goal (no Fleet Safety Context) was never cancelled on a lost contract. These tests fix the R1 rules for it:
Stage 1 and the fenced Stage 3 refuse, the contract generation stamped at enqueue is checked, and an active Frontier goal keeps
the cancel duty. Cancel / STOP stays possible without a lease. Stub ROS / Nav2 (no rclpy graph, no robot, no motor).
"""
import time
import types
import unittest

from test_agent_safety_v1 import _make_ready_robot, StubGoalHandle, safety_protocol


def _robot(generation=3, **session):
    robot = _make_ready_robot(allow_missions=False)
    robot._session_state.update(reconciliation_required=False, contract_generation=generation, **session)
    robot.frontier_allowed = lambda sid: sid == 'sess-1'
    return robot


def _cmd(**extra):
    return dict(mission_id='sess-1', command_id='frontier-1', expires_at=time.time() + 3,
                payload={'goal': {'x': 1.0, 'y': 2.0, 'yaw_rad': 0.0, 'frame_id': 'map'}}, **extra)


def _start(robot, cmd=None):
    acks = []
    robot.start_frontier(cmd or _cmd(), lambda s, d='', extra=None: acks.append((s, d)))
    robot._run_tasks()
    return acks


class FrontierSubmission(unittest.TestCase):
    def test_valid_authority_submits_once_and_stamps_the_generation(self):
        robot = _robot()
        acks = _start(robot)
        self.assertEqual(len(robot.nav.sent_goals), 1, acks)
        self.assertEqual(acks[-1][0], 'SUBMITTED')
        self.assertEqual(robot.active['owner'], 'frontier')
        self.assertEqual(robot.active['contract_generation'], 3)

    def test_each_missing_authority_blocks_the_goal(self):
        now = time.time()
        cases = {
            'lease_missing': dict(lease=None),
            'lease_expired': dict(lease={'owner': 'op-alice', 'epoch': 7, 'expires_at': now - 1}),
            'lease_malformed': dict(lease={'owner': '', 'epoch': 7, 'expires_at': now + 30}),
            'reconcile_pending': dict(reconciliation_required=True),
            'reconcile_unknown': dict(reconciliation_required=None),
            'protocol_missing': dict(protocol_version=None),
            'safety_version_wrong': dict(safety_contract_version=2),
            'disconnected': dict(connected=False),
        }
        for label, change in cases.items():
            with self.subTest(label=label):
                robot = _robot()
                robot._session_state.update(change)
                acks = _start(robot)
                self.assertEqual(robot.nav.sent_goals, [], acks)
                self.assertEqual(acks[-1][0], 'REJECTED')
                self.assertIsNone(robot.active)

    def test_generation_changed_after_enqueue_blocks_the_goal(self):
        robot = _robot()
        robot.start_frontier(_cmd(), lambda *a: None)          # stamped with generation 3
        robot._session_state['contract_generation'] = 4        # a withdrawal and a recovery happened while it was queued
        acks = []
        robot.tasks[-1] = (robot.tasks[-1][0], robot.tasks[-1][1], lambda s, d='', extra=None: acks.append((s, d)))
        robot._run_tasks()
        self.assertEqual(robot.nav.sent_goals, [])
        self.assertTrue(acks[-1][1].startswith('SERVER_CONTRACT_INVALID'), acks)

    def test_withdrawal_seen_only_inside_the_fence_blocks_the_goal(self):
        robot = _robot()
        real = robot._session_context
        calls = {'n': 0}
        def ctx():
            calls['n'] += 1
            s = real()
            if calls['n'] >= 2:                                 # Stage 1 sees a valid session, the fenced re-check a withdrawn one
                s['lease'] = None
            return s
        robot._session_context = ctx
        acks = _start(robot)
        self.assertEqual(robot.nav.sent_goals, [])
        self.assertIn('before send', acks[-1][1])
        self.assertGreaterEqual(calls['n'], 2)

    def test_the_fence_is_held_while_sending(self):
        robot = _robot()
        held = []
        class Fence:
            def __init__(self): self.depth = 0
            def __enter__(self): self.depth += 1
            def __exit__(self, *a): self.depth -= 1
        fence = Fence()
        robot._submission_fence = lambda: fence
        send = robot.nav.send_goal_async
        def spy(*a, **k):
            held.append(fence.depth)
            return send(*a, **k)
        robot.nav.send_goal_async = spy
        _start(robot)
        self.assertEqual(held, [1])


class ActiveFrontierGoal(unittest.TestCase):
    def _active(self):
        robot = _robot()
        _start(robot)
        handle = StubGoalHandle(uuid_hex=robot.active['goal_uuid'], accepted=True)
        robot.nav.sent_goals[0][2].set_result(handle)
        self.assertIs(robot.active['handle'], handle)
        return robot, handle

    def test_lost_authority_cancels_and_a_later_recovery_keeps_the_duty(self):
        for label, change in {'generation': dict(contract_generation=4), 'lease': dict(lease=None),
                              'reconcile': dict(reconciliation_required=True), 'contract': dict(protocol_version=None)}.items():
            with self.subTest(label=label):
                robot, handle = self._active()
                robot._session_state.update(change)
                if label != 'generation':
                    robot._session_state['contract_generation'] = 4            # AgentCore bumps it on every withdrawal
                robot._monitor_active_goal()
                self.assertEqual(handle.cancel_calls, 1)
                self.assertIn(robot.active['fault_cancel_reason'],
                              ('SERVER_CONTRACT_INVALID', 'LEASE_REVOKED_OR_EXPIRED', 'SERVER_RECONCILIATION_REQUIRED'))

    def test_recovered_session_with_a_new_generation_still_cancels(self):
        robot, handle = self._active()
        robot._session_state.update(lease=None, contract_generation=4)
        robot._session_state.update(lease={'owner': 'op-alice', 'epoch': 7, 'expires_at': time.time() + 30}, contract_generation=5)
        robot._monitor_active_goal()                                  # monitor runs only after the recovery
        self.assertEqual(handle.cancel_calls, 1)
        self.assertEqual(robot.active['fault_cancel_reason'], 'SERVER_CONTRACT_INVALID')

    def test_valid_authority_keeps_the_goal(self):
        robot, handle = self._active()
        robot._monitor_active_goal()
        self.assertEqual(handle.cancel_calls, 0)

    def test_cancel_is_allowed_without_a_lease(self):
        robot, handle = self._active()
        robot._session_state.update(lease=None, reconciliation_required=None, connected=False)
        acks = []
        robot.cancel_frontier(dict(mission_id='sess-1', command_id='stop-1'), lambda s, d='', extra=None: acks.append((s, d)))
        robot._run_tasks()
        self.assertEqual(acks[0][0], 'ACCEPTED')
        self.assertEqual(handle.cancel_calls, 1)


class Predicate(unittest.TestCase):
    def test_predicate_is_strict(self):
        now = time.time()
        ok = {'reconciliation_required': False, 'protocol_version': 2, 'safety_contract_version': 1, 'connected': True,
              'lease': {'owner': 'a', 'epoch': 1, 'expires_at': now + 5}, 'contract_generation': 2}
        self.assertIsNone(safety_protocol.frontier_authority_fault(ok, 2))
        for stamp in (None, True, 2.0, '2', 1):
            with self.subTest(stamp=stamp):
                self.assertEqual(safety_protocol.frontier_authority_fault(ok, stamp), 'SERVER_CONTRACT_INVALID')
        self.assertEqual(safety_protocol.frontier_authority_fault(dict(ok, reconciliation_required=0), 2), 'SERVER_RECONCILIATION_REQUIRED')
        self.assertEqual(safety_protocol.frontier_authority_fault(None, 2), 'SERVER_RECONCILIATION_REQUIRED')


if __name__ == '__main__':
    unittest.main()
