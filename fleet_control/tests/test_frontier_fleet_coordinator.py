import sys, threading, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent'))
from limo_robot_agent.frontier_fleet_coordinator import FrontierFleetCoordinator

class Tests(unittest.TestCase):
 def setUp(self):
  self.submitted=[];self.canceled=[]
  def submit(goal,key):self.submitted.append((goal,key));return {'mission_id':'m1'}
  self.c=FrontierFleetCoordinator(submit,self.canceled.append);self.c.start()
 def test_one_goal_until_journal_terminal(self):
  self.assertTrue(self.c.offer({'x':1,'y':2}));self.assertFalse(self.c.offer({'x':2,'y':3}))
  m={'mission_id':'m1','status':'SUCCEEDED','commands':[{'type':'navigate_to_pose','command_id':'c1'}]}
  self.assertIsNone(self.c.observe(m,{'active':None,'nav2_goals':{'known':True,'active_count':0},'journal_terminals':[]},'boot'));self.assertEqual('UNKNOWN',self.c.state)
  result=self.c.observe(m,{'active':None,'nav2_goals':{'known':True,'active_count':0},'journal_terminals':[{'command_id':'c1','mission_id':'m1','goal_uuid':'a'*32,'status':'SUCCEEDED','terminal_status':'SUCCEEDED','terminal_source':'nav2_action_result','terminal_agent_boot_id':'boot'}]},'boot')
  self.assertEqual('SUCCEEDED',result['status']);self.assertEqual('a'*32,result['goal_uuid']);self.assertEqual('SEARCHING',self.c.state)
 def test_cancel_ack_is_not_terminal(self):
  self.c.offer({'x':1,'y':2});self.c.stop();self.assertEqual(['m1'],self.canceled);self.assertEqual('CANCELING',self.c.state)
  self.assertFalse(self.c.offer({'x':3,'y':4}));self.assertIsNotNone(self.c.current)
 def test_unknown_submission_never_retries(self):
  def fail(*_):raise TimeoutError('response lost')
  c=FrontierFleetCoordinator(fail,lambda _:self.fail('unknown mission must not be canceled'));c.start();self.assertFalse(c.offer({'x':1}));self.assertEqual('UNKNOWN',c.state);self.assertFalse(c.offer({'x':2}));c.stop();self.assertEqual('UNKNOWN',c.state)
 def test_stop_racing_submission_cancels_only_returned_id(self):
  began=threading.Event();release=threading.Event();canceled=[]
  def submit(*_):began.set();release.wait(2);return {'mission_id':'m1'}
  c=FrontierFleetCoordinator(submit,canceled.append);c.start();t=threading.Thread(target=lambda:c.offer({'x':1}));t.start();self.assertTrue(began.wait(1))
  t2=threading.Thread(target=c.stop);t2.start();self.assertTrue(c.stop_requested.wait(1));release.set();t.join();t2.join();self.assertEqual(['m1'],canceled);self.assertEqual('CANCELING',c.state)
if __name__=='__main__':unittest.main()
