"""Terminal persistence precedes releasing the owned GoalHandle."""
import types, unittest
import test_agent_safety_v1 as fixture
from action_msgs.msg import GoalStatus

class TerminalOrder(unittest.TestCase):
 def make(self, fail=False):
  robot=fixture._make_ready_robot(allow_missions=True); seen=[]
  def ack(st,detail='',progress=None):
   if st == 'SUCCEEDED':
    seen.append(robot.active is not None)
    return not fail
   return True
  robot.start_navigate(fixture._make_cmd(sc=fixture._valid_safety_context()),ack)
  robot._run_tasks();h=fixture.StubGoalHandle();robot.nav.sent_goals[0][2].set_result(h)
  return robot,h,seen
 def test_handle_owned_until_terminal_ack_persisted(self):
  robot,h,seen=self.make();h.result_future.set_result(types.SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
  self.assertEqual(seen,[True]);self.assertIsNone(robot.active)
 def test_failed_terminal_persistence_keeps_ownership(self):
  robot,h,seen=self.make(fail=True);h.result_future.set_result(types.SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
  self.assertEqual(seen,[True]);self.assertIsNotNone(robot.active);self.assertTrue(robot.active['reconciliation_required'])

if __name__=='__main__':unittest.main()
