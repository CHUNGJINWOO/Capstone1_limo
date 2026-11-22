"""Running-only SIM Gate STALL_PENDING cannot authorize new motion."""
import copy,json,time,unittest
from types import SimpleNamespace
from test_r3_r4_server_safety import Tests as Fixture
from test_gate_r7_evidence import Tests as Evidence

class Tests(unittest.TestCase):
 def setUp(self):
  self.b=Fixture();self.b.setUp();self.f=self.b.f;self.db=self.b.db
  self.f.cfg=SimpleNamespace(**vars(self.f.cfg),sim_nav_evidence_actions=('navigate_to_pose',));self.f.robots['r']['agent'].update(mode='sim',simulated=True)
  m,_=self.b.create();cmd=self.f._deliver('r')['commands'][0];self.cmd=cmd;self.uuid='a'*32
  self.f.ack_command('r',cmd['command_id'],'ACCEPTED',agent_boot_id='boot1')
  e=Evidence();e.setUp();e.m.update(boot_id='gate1',gate_boot_id='gate1',fault_epoch=1)
  e.m['nav_evidence'].update(all_idle_proven=False,active_uuids=[self.uuid]);e.m['nav_evidence']['servers']['navigate_to_pose']['verdict']='ACTIVE_GOALS'
  gate=e.evaluate();gate.update(status='STALL_PENDING',movement_permitted=False,movement_block_reasons=['GATE_NOT_NORMAL:STALL_PENDING'])
  nav=e.proof(gate);nav['foreign_active_goal_uuids']=[]
  self.s=copy.deepcopy(self.b.state);self.s['safety_gate']=gate;self.s['execution']=dict(active=dict(command_id=cmd['command_id'],mission_id=m['mission_id'],goal_uuid=self.uuid,goal_handle_available=True),nav2_goals=nav)
  self.f.robots['r']['state']=self.s;self.f.robots['r']['state_ts']=time.time()
  with self.f.lock,self.db.transaction():self.f.check_execution('r',self.s['execution'])
 def tearDown(self):self.b.tearDown()
 def test_only_continuation(self):
  self.assertIsNone(self.f.safety_context('r'))
  self.assertIsNotNone(self.f.safety_context('r',active_continuation=True))
 def test_faults_and_missing_ownership_rejected(self):
  for section,key,value in [('safety_gate','fault_epoch_pending',1),('safety_gate','cmd_blocked',True),('safety_gate','safety_state','FAULT'),('safety_gate','source_fresh',False),('execution','active',None)]:
   with self.subTest(key=key):
    original=copy.deepcopy(self.s);self.s[section][key]=value
    self.assertIsNone(self.f.safety_context('r',active_continuation=True))
    self.s.clear();self.s.update(original)
 def test_foreign_and_mismatch_rejected(self):
  self.s['execution']['active']['goal_uuid']='b'*32
  self.assertIsNone(self.f.safety_context('r',active_continuation=True))

if __name__=='__main__':unittest.main(verbosity=2,defaultTest='Tests')
