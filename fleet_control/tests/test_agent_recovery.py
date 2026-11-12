#!/usr/bin/env python3
"""Agent-side rule for operator-recovered commands: the journal is cleared only when the agent's OWN Nav2 evidence agrees. python3 tests/test_agent_recovery.py"""
import os, sys, tempfile, unittest
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'agent', 'limo_robot_agent'))
from limo_robot_agent.core import AgentCore


class H:
    def start_navigate(self, *a): self.started = getattr(self, 'started', 0) + 1
    def cancel_mission(self, *a): pass


class T(unittest.TestCase):
    def mk(self, ex):
        self.ex = ex; d = tempfile.mkdtemp(); a = AgentCore('r', 'http://x', 't', lambda: {'execution': self.ex}, H(), {}, d, log=lambda *_: None)
        a.seen['old'] = {'status': 'RECONCILIATION_REQUIRED', 'mission_id': 'm_old', 'type': 'navigate_to_pose', 'seq': 1}; a._save(); return a

    def test_cleared_only_with_local_idle_evidence(self):
        a = self.mk({'active': None, 'nav2_goals': {'known': True, 'active_count': 1}}); a._apply_recovery(['old'])
        self.assertEqual(a.seen['old']['status'], 'RECONCILIATION_REQUIRED', 'Nav2 still has a goal: stay blocked')
        self.ex['nav2_goals'] = {'known': False, 'active_count': None}; a._apply_recovery(['old'])
        self.assertEqual(a.seen['old']['status'], 'RECONCILIATION_REQUIRED', 'no evidence: stay blocked')
        self.ex['nav2_goals'] = {'known': True, 'active_count': 0}; self.ex['active'] = {'mission_id': 'x', 'command_id': 'y', 'goal_handle_available': True}; a._apply_recovery(['old'])
        self.assertEqual(a.seen['old']['status'], 'RECONCILIATION_REQUIRED', 'agent still holds a goal handle: stay blocked')
        self.ex['active'] = None; a._apply_recovery(['old'])
        self.assertEqual(a.seen['old']['status'], 'FAILED'); self.assertIn('unverified', a.seen['old']['detail'])

    def test_unknown_or_foreign_ids_are_ignored_and_new_goal_unblocked_afterwards(self):
        a = self.mk({'active': None, 'nav2_goals': {'known': True, 'active_count': 0}}); a._apply_recovery(['nonexistent'])
        self.assertEqual(a.seen['old']['status'], 'RECONCILIATION_REQUIRED')
        import time
        cmd = {'command_id': 'new', 'robot_id': 'r', 'mission_id': 'm_new', 'type': 'navigate_to_pose', 'seq': 5, 'attempt': 1, 'payload': {'goal': {'frame_id': 'map', 'x': 1, 'y': 1}},
               'issued_at': time.time(), 'expires_at': time.time() + 60, 'owner_boot_id': a.boot_id}
        a._on_command(dict(cmd)); self.assertFalse(hasattr(a.handler, 'started'), 'blocked before recovery')
        a._apply_recovery(['old']); cmd2 = dict(cmd, command_id='new2', seq=6); a._on_command(cmd2)
        self.assertEqual(getattr(a.handler, 'started', 0), 1, 'after the verified recovery the next goal is accepted')


if __name__ == '__main__': unittest.main()
