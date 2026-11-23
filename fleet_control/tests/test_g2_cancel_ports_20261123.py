"""G2 (2026-11-23): origin/main test_cancel_results cases ported to the v5 API (run from test_cancel_results in a subprocess).

The v5 Fleet goal is admitted by the real Safety Context v1 preconditions at Stage 1-3, so these cases use the v5 fixture
(test_agent_safety_v1: stub ROS messages, stub Nav2) instead of origin/main's Harness, whose preconditions are stubbed to "ok".
"""
import unittest

from test_agent_safety_v1 import _make_ready_robot, _make_cmd, _valid_safety_context


class ForeignGoals(unittest.TestCase):
    def test_live_or_ambiguous_foreign_goals_prevent_submission(self):
        # G2 2026-11-23 port to the v5 API: the Fleet goal is admitted by the real Safety Context v1 preconditions (Stage 1-3),
        # so the robot comes from the v5 fixture instead of the Harness (whose preconditions are stubbed to "ok"). Each case
        # must be REJECTED with nothing sent; v5 names an active foreign goal NAV2_NOT_IDLE, an ambiguous or missing server
        # NAV_GOAL_OWNERSHIP_UNAVAILABLE (origin/main used that name for all three).
        for observed, code in (({'publishers': 1, 'active_count': 1, 'active_goal_uuids': ['ab' * 16]}, 'NAV2_NOT_IDLE'),
                               ({'publishers': 2, 'active_count': 0}, 'NAV_GOAL_OWNERSHIP_UNAVAILABLE'),
                               ({'publishers': 0, 'active_count': None, 'known': False}, 'NAV_GOAL_OWNERSHIP_UNAVAILABLE')):
            with self.subTest(observed=observed):
                robot = _make_ready_robot(allow_missions=True)
                base = robot._nav2_goals()
                robot._nav2_goals = lambda o=observed, b=base: dict(b, **o)
                responses = []
                robot.start_navigate(_make_cmd(sc=_valid_safety_context()), lambda status, detail='', *a: responses.append((status, detail)))
                robot._run_tasks()
                self.assertEqual(responses[0][0], 'REJECTED')
                self.assertIn(code, responses[0][1])
                self.assertEqual(robot.nav.sent_goals, [])
                self.assertIsNone(robot.active)



if __name__ == '__main__':
    unittest.main()
