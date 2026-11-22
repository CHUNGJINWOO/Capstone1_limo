"""SIM_ONLY, explicit operator recovery of a journal-terminal memory latch."""
import copy,json,unittest,time
from types import SimpleNamespace
from test_r3_r4_server_safety import Tests as Fixture, GOAL
from test_gate_r7_evidence import Tests as Evidence
from fleet_server.auth import Principal
from fleet_server.service import ApiError

class Tests(unittest.TestCase):
 def setUp(self):
  self.base=Fixture(); self.base.setUp();self.f=self.base.f;self.db=self.base.db
  self.f.cfg=SimpleNamespace(**vars(self.f.cfg),sim_nav_evidence_actions=('navigate_to_pose',))
  self.f.robots['r']['agent'].update(mode='sim',simulated=True)
  self.m,_=self.base.create();self.c=self.f._deliver('r')['commands'][0]
  with self.f.lock,self.db.transaction():
   self.f.check_execution('r',{'active':dict(command_id=self.c['command_id'],mission_id=self.m['mission_id'],goal_uuid='1'*32,goal_handle_available=True)})
  self.base.fault();self.f.ack_command('r',self.c['command_id'],'CANCELED',agent_boot_id='boot1')
  ev=Evidence();ev.setUp();ev.m.update(boot_id='gate1',gate_boot_id='gate1',fault_epoch=2)
  gate=ev.evaluate();gate['movement_permitted']=True
  self.state=copy.deepcopy(self.base.state);self.state['safety_gate']=gate
  self.record=dict(command_id=self.c['command_id'],mission_id=self.m['mission_id'],goal_uuid='1'*32,
   terminal_source='nav2_action_result',terminal_agent_boot_id='boot1',terminal_status='CANCELED',status='CANCELED',safety_context=self.c['payload']['safety_context'])
  self.state['execution'].update(journal_terminals=[self.record],nav2_goals=ev.proof(gate))
  self.f.ingest_state('r',3,self.state,'boot1')
 def tearDown(self):self.base.tearDown()
 def recover(self):return self.f.recover_robot_block('r',Principal('admin','admin'),'SIM_ONLY operator reviewed terminal UUID and current Gate evidence')
 def test_explicit_recovery_only(self):
  self.assertTrue(self.f.robots['r']['reconciliation_required'])
  with self.assertRaises(ApiError):self.base.create('blocked')
  self.recover();self.assertFalse(self.f.robots['r']['reconciliation_required'])
  self.assertEqual(self.f._deliver('r')['commands'],[])
  m,_=self.base.create('new');self.assertNotEqual(m['mission_id'],self.m['mission_id'])
  delivery=self.f.ingest_state('r',4,self.state,'boot1')
  self.assertEqual(len(delivery['commands']),1)
  self.assertEqual(delivery['commands'][0]['mission_id'],m['mission_id'])
 def test_missing_terminal(self):
  self.state['execution']['journal_terminals']=[]
  with self.assertRaises(ApiError):self.recover()
 def test_cancel_ack_not_terminal(self):
  self.record['terminal_source']='cancel_ack'
  with self.assertRaises(ApiError):self.recover()
 def test_wrong_uuid(self):
  self.record['goal_uuid']='2'*32
  with self.assertRaises(ApiError):self.recover()
 def test_wrong_owner(self):
  self.record['terminal_agent_boot_id']='old'
  with self.assertRaises(ApiError):self.recover()
 def test_unknown_external_goal(self):
  self.db.x('INSERT INTO meta(k,v) VALUES(?,?)',('reconciliation_block:r','unknown'))
  with self.assertRaises(ApiError):self.recover()
 def test_gate_boot_block(self):
  self.db.x('INSERT INTO meta(k,v) VALUES(?,?)',('gate_boot_reconciliation:r','changed'))
  with self.assertRaises(ApiError):self.recover()
 def test_real_mode_refused(self):
  self.f.robots['r']['agent']['mode']='real'
  with self.assertRaises(ApiError):self.recover()
 def test_unresolved_second_command(self):
  self.db.x("UPDATE commands SET status='RECONCILIATION_REQUIRED' WHERE command_id=?",(self.c['command_id'],))
  with self.assertRaises(ApiError):self.recover()

if __name__=='__main__':unittest.main(verbosity=2,defaultTest='Tests')
