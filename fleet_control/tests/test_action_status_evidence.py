"""Continuity-based action idle evidence (no ROS graph; message/graph objects are synthetic)."""
import unittest
from types import SimpleNamespace
from action_msgs.msg import GoalStatus, GoalStatusArray
from limo_robot_agent.action_status_evidence import ActionStatusEvidence


def info(gid, node='bt_navigator'):
    return SimpleNamespace(endpoint_gid=bytes([gid]) * 16, node_name=node)


def msg(*codes):
    m = GoalStatusArray()
    for i, c in enumerate(codes):
        g = GoalStatus(); g.status = c; g.goal_info.goal_id.uuid = [i + 1] * 16; m.status_list.append(g)
    return m


class Evidence(unittest.TestCase):
    def setUp(self):
        self.e = ActionStatusEvidence('NAV2', settle_s=1.0)

    def test_silent_unique_server_is_idle_after_settle(self):
        self.assertEqual(self.e.evaluate(0., [info(1)])['reason'], 'NAV2_STATUS_SETTLING')
        ev = self.e.evaluate(1.2, [info(1)])
        self.assertTrue(ev['ok']); self.assertEqual(ev['active_count'], 0); self.assertFalse(ev['received'])

    def test_missing_duplicate_and_unexpected_node(self):
        self.assertEqual(self.e.evaluate(5., [])['reason'], 'NAV2_SERVER_MISSING')
        self.assertEqual(self.e.evaluate(9., [info(1), info(2)])['reason'], 'NAV2_SERVER_NOT_UNIQUE')
        fp = ActionStatusEvidence('FOLLOW_PATH', expected_node='controller_server', settle_s=0.)
        self.assertEqual(fp.evaluate(1., [info(3, 'rogue')])['reason'], 'FOLLOW_PATH_UNEXPECTED_SERVER_NODE')
        self.assertTrue(fp.evaluate(1., [info(3, 'controller_server')])['ok'])

    def test_active_goal_blocks_and_terminal_clears(self):
        self.e.evaluate(0., [info(1)])
        self.e.on_message(2., msg(GoalStatus.STATUS_EXECUTING))
        self.assertEqual(self.e.evaluate(2.1, [info(1)])['reason'], 'NAV2_GOAL_ACTIVE')
        self.e.on_message(3., msg(GoalStatus.STATUS_CANCELED))
        self.assertTrue(self.e.evaluate(3.1, [info(1)])['ok'])
        self.e.on_message(4., msg())                       # goal expired from the result cache
        self.assertTrue(self.e.evaluate(4.1, [info(1)])['ok'])

    def test_server_change_resets_settle_and_stale_active_needs_vouch(self):
        self.e.evaluate(0., [info(1)])
        self.e.on_message(1., msg(GoalStatus.STATUS_EXECUTING))
        self.assertEqual(self.e.evaluate(5., [info(2)])['reason'], 'NAV2_STATUS_SETTLING')
        ev = self.e.evaluate(6.5, [info(2)])
        self.assertEqual(ev['reason'], 'NAV2_SERVER_CHANGED_WITH_ACTIVE_GOAL')
        self.assertFalse(self.e.evaluate(6.6, [info(2)], vouched_gid=(bytes([1]) * 16).hex())['ok'])   # vouch for old GID
        self.assertTrue(self.e.evaluate(6.7, [info(2)], vouched_gid=(bytes([2]) * 16).hex())['ok'])
        self.e.on_message(7., msg(GoalStatus.STATUS_EXECUTING))  # new server's own active goal: vouch irrelevant
        self.assertEqual(self.e.evaluate(7.1, [info(2)], vouched_gid=(bytes([2]) * 16).hex())['reason'], 'NAV2_GOAL_ACTIVE')

    def test_partial_or_failed_identity_is_unknown(self):
        bad = SimpleNamespace(endpoint_gid=b'', node_name='bt_navigator')
        self.e.evaluate(0., [info(1)])                                  # the writer was known before the lookup failure
        self.assertEqual(self.e.evaluate(5., [info(1), bad])['reason'], 'NAV2_SERVER_IDENTITY_UNKNOWN')
        self.assertEqual(self.e.evaluate(9., [bad])['reason'], 'NAV2_SERVER_IDENTITY_UNKNOWN')
        self.assertTrue(self.e.evaluate(10., [info(1)])['ok'])           # the same GID again = the same writer
        self.assertEqual(self.e.evaluate(11., [info(3)])['reason'], 'NAV2_STATUS_SETTLING')   # a different writer restarts

    def test_message_in_attribution_window_after_change_is_unattributed(self):
        self.e.evaluate(0., [info(1)]); self.e.evaluate(3., [info(2)])
        self.e.on_message(3.5, msg())                                 # could be the old writer's delayed message
        ev = self.e.evaluate(5., [info(2)])
        self.assertEqual(ev['reason'], 'NAV2_STATUS_UNATTRIBUTED')
        self.assertTrue(self.e.evaluate(5.1, [info(2)], vouched_gid=(bytes([2]) * 16).hex())['ok'])
        self.e.on_message(6., msg())                                  # after the window: attributed to the new writer
        self.assertTrue(self.e.evaluate(6.1, [info(2)])['ok'])

    def test_changed_silent_server_needs_vouch(self):
        self.e.evaluate(0., [info(1)]); self.e.evaluate(3., [info(2)])
        self.assertEqual(self.e.evaluate(9., [info(2)])['reason'], 'NAV2_SERVER_CHANGED_UNVOUCHED')
        self.assertTrue(self.e.evaluate(9.1, [info(2)], vouched=True)['ok'])


if __name__ == '__main__':
    unittest.main(verbosity=1)
