"""Phase 1.1 synthetic scenarios: no ROS, IO, server or robot."""
import copy,dataclasses,unittest
from operation_goal_mock import Generation,Goal,GoalModel

class Tests(unittest.TestCase):
 def setUp(self):self.g=Generation('agent1','gate1',5,8,9,10);self.m=GoalModel(self.g)
 def submit(self,owner='WEB_NAVIGATION',handle=True):
  goal=Goal('op1','cmd1',owner,'owner1','uuid1',self.g,handle_owned=handle)
  self.m.submit(goal,lease_valid=True,proof_fresh=True,idle_proven=True,external_goals=[]);return goal
 def stop(self):self.m.stop('op1',role='operator',key='stop1',current_generation=self.m.current)
 def reconcile(self,**kw):
  args={'role':'admin','proof_fresh':True,'idle_proven':True,'external_goals':[],'process_stopped':True};args.update(kw);self.m.reconcile(**args)
 def test_01_web_normal_terminal(self):
  self.submit();self.m.acceptance('uuid1');self.assertTrue(self.m.terminal('uuid1','SUCCEEDED'));self.assertEqual(self.m.state,'TERMINAL_CONFIRMED')
 def test_02_rmf_cancel(self):
  self.submit('RMF_NAVIGATION');self.m.acceptance('uuid1');self.stop();self.m.cancel_ack('uuid1',True);self.m.terminal('uuid1','CANCELED')
  self.assertTrue(self.m.finish_stop(process_stopped=False,proof_fresh=True,idle_proven=True,external_goals=[]))
 def test_03_explore_requires_process_and_terminal(self):
  self.submit('AUTO_MAPPING_EXPLORE');self.m.acceptance('uuid1');self.stop();self.m.cancel_ack('uuid1',True);self.m.terminal('uuid1','CANCELED')
  self.assertTrue(self.m.finish_stop(process_stopped=True,proof_fresh=True,idle_proven=True,external_goals=[]));self.assertEqual(self.m.state,'STOPPED')
 def test_04_cancel_ack_is_not_terminal(self):
  self.submit();self.m.acceptance('uuid1');self.stop();self.m.cancel_ack('uuid1',True)
  self.assertEqual(self.m.state,'CANCEL_ACCEPTED');self.assertFalse(self.m.finish_stop(process_stopped=True,proof_fresh=True,idle_proven=True,external_goals=[]))
 def test_05_lost_handle(self):
  self.submit(handle=False);self.m.acceptance('uuid1');self.stop();self.assertEqual(self.m.state,'RECONCILIATION_REQUIRED');self.assertEqual(self.m.cancel_requests,[])
 def test_06_foreign_goal(self):
  with self.assertRaises(ValueError):self.m.submit(Goal('a','b','WEB_NAVIGATION','c','d',self.g),lease_valid=True,proof_fresh=True,idle_proven=True,external_goals=['foreign'])
 def test_07_gate_boot_change(self):
  self.submit();self.m.acceptance('uuid1');self.m.generation_change_or_restart(dataclasses.replace(self.g,gate_boot_id='gate2',fault_epoch=0))
  self.assertTrue(self.m.blocked);self.assertEqual(self.m.cancel_requests,['uuid1'])
 def test_08_fault_epoch_increase(self):
  self.submit();self.m.acceptance('uuid1');self.m.generation_change_or_restart(dataclasses.replace(self.g,fault_epoch=6));self.assertTrue(self.m.blocked)
 def test_09_operation_revision_change(self):
  self.submit();self.m.generation_change_or_restart(dataclasses.replace(self.g,operation_revision=9));self.assertTrue(self.m.blocked)
 def test_10_map_epoch_change(self):
  self.submit();self.m.generation_change_or_restart(dataclasses.replace(self.g,map_epoch=10));self.assertTrue(self.m.blocked)
 def test_11_restart_and_old_command(self):
  self.submit();self.m.acceptance('uuid1');self.m.generation_change_or_restart(dataclasses.replace(self.g,agent_boot_id='agent2'))
  restarted=copy.deepcopy(self.m)
  with self.assertRaises(ValueError):restarted.submit(Goal('op1','cmd1','WEB_NAVIGATION','owner1','uuid1',self.g),lease_valid=True,proof_fresh=True,idle_proven=True,external_goals=[])
  self.assertEqual(restarted.state,'RECONCILIATION_REQUIRED')
 def test_12_stop_no_lease_idempotent(self):
  self.submit();self.m.acceptance('uuid1');self.stop();self.stop();self.assertEqual(self.m.cancel_requests,['uuid1'])
  with self.assertRaises(ValueError):self.m.stop('op1',role='viewer',key='x',current_generation=self.g)
 def test_13_late_acceptance(self):
  self.submit();self.stop();self.m.acceptance('uuid1');self.assertEqual(self.m.state,'RECONCILIATION_REQUIRED');self.assertEqual(self.m.cancel_requests,['uuid1'])
 def test_14_late_terminal_wrong_uuid_and_reuse(self):
  self.submit();self.m.generation_change_or_restart(dataclasses.replace(self.g,gate_boot_id='gate2'))
  self.assertFalse(self.m.terminal('wrong','CANCELED'));self.assertTrue(self.m.terminal('uuid1','CANCELED'));self.assertTrue(self.m.blocked)
  self.reconcile()
  with self.assertRaises(ValueError):self.m.submit(Goal('op2','cmd2','WEB_NAVIGATION','owner1','uuid1',self.m.current),lease_valid=True,proof_fresh=True,idle_proven=True,external_goals=[])
 def test_15_no_fresh_evidence_no_rearm(self):
  self.submit('AUTO_MAPPING_EXPLORE');self.m.generation_change_or_restart(self.g);self.m.terminal('uuid1','CANCELED')
  for change in ({'proof_fresh':False},{'idle_proven':False},{'external_goals':['other']},{'process_stopped':False}):
   with self.subTest(change=change),self.assertRaises(ValueError):self.reconcile(**change)
  self.reconcile();self.assertEqual(self.m.state,'IDLE')
 def test_manual_mapping_needs_process_stop(self):
  self.submit('MANUAL_MAPPING');self.m.acceptance('uuid1');self.stop();self.m.terminal('uuid1','CANCELED')
  self.assertFalse(self.m.finish_stop(process_stopped=False,proof_fresh=True,idle_proven=True,external_goals=[]))
 def test_server_restart_and_late_terminal_order(self):
  self.submit();self.m.acceptance('uuid1');self.m.generation_change_or_restart(self.g)
  server_after_restart=copy.deepcopy(self.m)
  self.assertTrue(server_after_restart.terminal('uuid1','CANCELED'));self.assertFalse(server_after_restart.terminal('uuid1','SUCCEEDED'))
  self.assertTrue(server_after_restart.blocked);self.assertEqual(server_after_restart.state,'RECONCILIATION_REQUIRED')
 def test_foreign_goal_never_automatically_canceled(self):
  self.submit('RMF_NAVIGATION');self.m.acceptance('uuid1');self.stop()
  self.assertEqual(self.m.cancel_requests,['uuid1'])
  self.m.terminal('uuid1','CANCELED')
  self.assertFalse(self.m.finish_stop(process_stopped=True,proof_fresh=True,idle_proven=True,external_goals=['foreign']))
 def test_same_boot_epoch_regression_blocks(self):
  self.submit();self.m.acceptance('uuid1');self.m.generation_change_or_restart(dataclasses.replace(self.g,fault_epoch=4))
  self.assertTrue(self.m.blocked);self.assertEqual(self.m.cancel_requests,['uuid1'])
 def test_explore_terminal_without_process_exit_remains_blocked(self):
  self.submit('AUTO_MAPPING_EXPLORE');self.m.acceptance('uuid1');self.stop();self.m.terminal('uuid1','CANCELED')
  self.assertFalse(self.m.finish_stop(process_stopped=False,proof_fresh=True,idle_proven=True,external_goals=[]))
  with self.assertRaises(ValueError):self.reconcile(process_stopped=False)
  self.reconcile(process_stopped=True);self.assertEqual(self.m.state,'IDLE')

if __name__=='__main__':unittest.main(verbosity=2)
