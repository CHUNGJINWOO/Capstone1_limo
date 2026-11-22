"""Independent boundary assertions against production code; no ROS nodes/robot/network."""
import time
import threading
import unittest
from types import SimpleNamespace
import test_operating_mode as existing
from fleet_server.service import ApiError
from limo_robot_agent.action_status_evidence import ActionStatusEvidence

class ModeBoundary(unittest.TestCase):
 def setUp(self):
  self.fixture=existing.ModeTests('test_lease_required');self.fixture.setUp();self.f=self.fixture.f
 def tearDown(self):self.fixture.tearDown()
 def test_missing_arbiter_cannot_confirm_manual(self):
  self.f.robots['R']['state']['velocity_arbiter']=None
  self.f.robots['R']['state']['motion_watchdog']={}
  self.fixture.request('MANUAL')
  self.fixture.report();time.sleep(.07);self.fixture.report();rec=self.fixture.report()
  print('OBSERVED missing arbiter and watchdog:',rec['mode'],rec['transition']['state'],flush=True)
  self.assertNotEqual(rec['transition']['state'],'ACTIVE','missing Agent arbiter/watchdog report cannot confirm MANUAL')
 def to_confirming(self):
  self.fixture.request('MANUAL');self.fixture.report();time.sleep(.07);return self.fixture.report()
 def test_agent_boot_change_during_transition_does_not_auto_finish(self):
  self.fixture.request('MANUAL');self.fixture.report()
  self.f.robots['R']['agent']['boot_id']='new-agent-boot'
  time.sleep(.07);rec=self.fixture.report()
  if rec['transition']['state']=='CONFIRMING':
   state=self.f.robots['R']['state']['velocity_arbiter'];state['operating_mode']='MANUAL';rec=self.fixture.report()
  print('OBSERVED boot change during mode transition:',rec['mode'],rec['transition']['state'],flush=True)
  self.assertNotEqual(rec['transition']['state'],'ACTIVE','old request must not complete automatically on a new Agent boot')
 def test_gate_fault_between_verify_and_confirmation_cannot_activate(self):
  rec=self.to_confirming();self.assertEqual(rec['transition']['state'],'CONFIRMING')
  state=self.f.robots['R']['state'];state['execution']['nav_idle']={'ok':False,'reason':'external_goal'}
  state['motion_watchdog']={'required':True,'state':'FAILSAFE','agent_boot_matches':True}
  state['velocity_arbiter']={'operating_mode':'MANUAL','selected_linear':0.,'selected_turn':0.,'ready':False}
  rec=self.fixture.report();print('OBSERVED fault during confirmation:',rec['mode'],rec['transition']['state'],flush=True)
  self.assertNotEqual(rec['transition']['state'],'ACTIVE','confirmation must revalidate current stop and safety evidence')
 def test_mapping_saving_blocks_mode_change(self):
  self.f.operation_status=lambda *a,**k:{'state':'SAVING'}
  with self.assertRaises(ApiError):self.fixture.request('MANUAL')
 def test_concurrent_modes_only_one_transition(self):
  barrier=threading.Barrier(3);results=[]
  def request(target,key):
   barrier.wait()
   try:self.f.request_mode('R','operator',{'target':target,'epoch':self.fixture.epoch,'idempotency_key':key});results.append('accepted')
   except ApiError:results.append('denied')
  threads=[threading.Thread(target=request,args=(m,str(i))) for i,m in enumerate(('MANUAL','NAVIGATION'))]
  for t in threads:t.start()
  barrier.wait()
  for t in threads:t.join(2)
  self.assertCountEqual(results,['accepted','denied'])
 def test_unknown_idle_timeout_cannot_claim_idle(self):
  self.f.robots['R']['state']['execution']['nav_idle']={'ok':False,'reason':'NAV2_SERVER_MISSING'}
  self.fixture.request('MANUAL');rec=self.fixture.report();rec['transition']['deadline']=time.time()-1;self.f._mode_save('R',rec)
  rec=self.fixture.report();print('OBSERVED missing Nav2 at timeout:',rec['mode'],rec['transition']['state'],flush=True)
  self.assertEqual(rec['mode'],'FAULT','uncertain external Goal state must not be reported as confirmed IDLE')

class PublisherBoundary(unittest.TestCase):
 def test_missing_gid_among_duplicate_publishers_is_not_unique(self):
  good=SimpleNamespace(endpoint_gid=b'A'*16,node_name='controller_server')
  unknown=SimpleNamespace(endpoint_gid=None,node_name='controller_server')
  e=ActionStatusEvidence('FOLLOW_PATH',expected_node='controller_server',settle_s=1.)
  e.observe(1.,[good,unknown]);result=e.evaluate(3.,[good,unknown])
  print('OBSERVED two publishers with one unreadable GID:',result,flush=True)
  self.assertFalse(result['ok'],'an unidentifiable second endpoint must prevent a uniqueness proof')
 def test_queued_old_writer_empty_message_not_current_evidence(self):
  a=SimpleNamespace(endpoint_gid=b'A'*16,node_name='controller_server')
  b=SimpleNamespace(endpoint_gid=b'B'*16,node_name='controller_server')
  e=ActionStatusEvidence('FOLLOW_PATH',expected_node='controller_server',settle_s=1.)
  e.observe(1.,[a]);e.observe(3.,[b])
  # Simulate delayed old-writer message delivered after a writer change: the
  # production callback receives only payload, not sender GID.
  e.on_message(3.5,SimpleNamespace(status_list=[]))
  result=e.evaluate(5.,[b]);print('OBSERVED delayed old-writer empty payload:',result,flush=True)
  self.assertFalse(result['ok'],'unattributed old-writer payload must not establish new writer idle')

if __name__=='__main__':unittest.main(verbosity=2)
