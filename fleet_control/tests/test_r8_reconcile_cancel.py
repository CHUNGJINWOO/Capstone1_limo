"""Server reconciliation notification forces owned-goal cancel, not terminal."""
import time, unittest
import test_agent_safety_v1 as fixture
from limo_robot_agent import safety_protocol

class Tests(unittest.TestCase):
 def test_reconciliation_owned_goal_cancel_and_ack_not_terminal(self):
  robot=fixture._make_ready_robot(allow_missions=True)
  session={'connected':True,'server_boot_id':'srv-boot-1','agent_boot_id':'agt-boot-1','last_state_age_s':0.,'lease':{'owner':'op-alice','epoch':7,'expires_at':time.time()+60},'safety_contract_version':1,'protocol_version':2,'reconciliation_required':False}
  robot.session_fn=lambda:dict(session)
  acks=[];robot.start_navigate(fixture._make_cmd(sc=fixture._valid_safety_context()),lambda s,d='',pr=None:acks.append((s,d)))
  robot._run_tasks();handle=fixture.StubGoalHandle(uuid_hex='ab'*16,accepted=True)
  robot.nav.sent_goals[0][2].set_result(handle)
  session['reconciliation_required']=True;robot._run_tasks()
  self.assertEqual(handle.cancel_calls,1)
  self.assertTrue(robot.active['cancel_requested'])
  self.assertIsNone(robot.active['terminal_status'])
  self.assertNotEqual(acks[-1][0],'CANCELED')

if __name__=='__main__':unittest.main(verbosity=2)
