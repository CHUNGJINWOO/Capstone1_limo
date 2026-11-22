"""Server-only synthetic protocol tests; no ROS or Agent implementation."""
import copy, unittest
import test_r3_r4_server_safety as base
from fleet_server.service import ApiError

class Tests(unittest.TestCase):
 def setUp(self):
  self.fixture=base.Tests('test_new_mission_and_idempotency'); self.fixture.setUp()
  self.f=self.fixture.f; self.robot=self.f.robots['r']; self.robot['agent']['safety_contract_version']=1
  self.state=copy.deepcopy(self.fixture.state)
  self.state['safety_gate']['boot_id']='gate1'
  self.state.update(operation_revision=1,localization_status='CONFIRMED')
  self.f.ingest_state('r',2,self.state,'boot1')
 def tearDown(self): self.fixture.tearDown()
 def create(self): return self.fixture.create()
 def test_context_and_lease_response(self):
  m,_=self.create(); reply=self.f._deliver('r'); cmd=reply['commands'][0]; c=cmd['payload']['safety_context']
  self.assertEqual(c['agent_boot_id'],'boot1'); self.assertEqual(c['gate_boot_id'],'gate1')
  self.assertEqual(c['operation_revision'],1); self.assertEqual(c['lease_epoch'],self.fixture.lease)
  self.assertEqual(c['expires_at'],cmd['expires_at']); self.assertEqual(c['localization_status'],'CONFIRMED')
  s=self.f.ingest_state('r',3,self.state,'boot1'); self.assertEqual(s['lease']['epoch'],self.fixture.lease)
 def test_old_agent_blocked(self):
  self.robot['agent'].pop('safety_contract_version')
  with self.assertRaises(ApiError): self.create()
 def test_context_change_fences_command(self):
  for field,value in [('operation_revision',2),('gate_boot','gate2')]:
   with self.subTest(field=field):
    self.setUp2(field,value)
 def setUp2(self,field,value):
  # Independently allocated fixture for each changed epoch.
  t=Tests('test_old_agent_blocked'); t.setUp()
  try:
   m,_=t.create(); state=copy.deepcopy(t.state)
   if field=='gate_boot': state['safety_gate']['boot_id']=value
   else: state[field]=value
   self.assertEqual(t.f.ingest_state('r',3,state,'boot1')['commands'],[])
   self.assertEqual(t.f.mission(m['mission_id'])['status'],'FAILED')
  finally:t.tearDown()
 def test_missing_required_and_ready_blocked(self):
  for key in ('operation_revision','gate_boot_id','localization_status'):
   s=copy.deepcopy(self.state)
   if key=='gate_boot_id':del s['safety_gate']['boot_id']
   elif key=='localization_status':s[key]='READY'
   else:del s[key]
   self.robot['state']=s
   with self.subTest(key=key), self.assertRaises(ApiError):self.create()
 def test_immutable_on_resend(self):
  self.create(); c=self.f._deliver('r')['commands'][0]
  self.f.db.x('UPDATE commands SET sent_at=0 WHERE command_id=?',(c['command_id'],))
  again=self.f._deliver('r')['commands'][0]
  self.assertEqual(c['payload']['safety_context'],again['payload']['safety_context'])

if __name__=='__main__': unittest.main(verbosity=2)
