import dataclasses,threading,unittest
from goal_boundary_attestation_mock import Attestation,MockBoundary,MockGate

class Tests(unittest.TestCase):
 def setUp(self):
  self.b=MockBoundary();self.g=MockGate('gate1',3,'nonce1',('writer1',))
  self.a=Attestation('gate1',3,'nonce1','OPERATION_MANAGER',1.,('writer1',),('navigate_to_pose',),(('u1','CANCELED','RESULT_RESPONSE',1.),),(),True,True)
 def verify(self,a=None,**kw):
  opts={'current_active':[],'required_goals':['u1'],'status_seen':True};opts.update(kw);return self.g.verify(a or self.a,**opts)
 def test_explore_stop_after_submit(self):
  self.b.submit('EXPLORE','u1');self.b.accept('u1');self.b.stop();self.assertEqual(self.b.cancel,['u1']);self.assertTrue(self.b.terminal('u1','CANCELED'))
 def test_stop_before_acceptance(self):
  self.b.submit('EXPLORE','u1');self.b.stop();self.b.accept('u1');self.assertTrue(self.b.reconciliation);self.assertEqual(self.b.cancel,['u1'])
 def test_process_exit_before_terminal(self):
  self.b.submit('EXPLORE','u1');self.b.accept('u1');self.b.process_exit();self.assertTrue(self.b.reconciliation);self.assertIsNone(self.b.active['terminal'])
 def test_terminal_before_process_exit(self):
  self.b.submit('EXPLORE','u1');self.b.accept('u1');self.b.terminal('u1','CANCELED');self.assertFalse(self.b.process_exited);self.b.process_exit();self.assertTrue(self.b.process_exited)
 def test_external_web_and_rmf_block(self):
  for owner in ('WEB','RMF'):
   b=MockBoundary();b.submit(owner,'external')
   with self.assertRaises(ValueError):b.submit('EXPLORE','u1')
   self.assertEqual(b.cancel,[])
 def test_competing_submit_no_preemption(self):
  barrier=threading.Barrier(3);results=[]
  def run(owner):
   barrier.wait()
   try:self.b.submit(owner,owner);results.append('admitted')
   except ValueError:results.append('blocked')
  ts=[threading.Thread(target=run,args=(o,)) for o in ('WEB','EXPLORE')]
  for t in ts:t.start()
  barrier.wait()
  for t in ts:t.join(3)
  self.assertFalse(any(t.is_alive() for t in ts));self.assertEqual(sorted(results),['admitted','blocked']);self.assertEqual(self.b.cancel,[])
 def test_gate_restart_old_callback(self):
  self.b.submit('EXPLORE','u1');self.b.restart();self.b.accept('u1');self.assertTrue(self.b.reconciliation);self.assertEqual(self.b.cancel,['u1'])
 def test_mock_restart_is_not_disk_journal(self):
  self.b.submit('WEB','u1');self.b.restart()
  with self.assertRaises(ValueError):self.b.submit('WEB','u1')
 def test_duplicate_stop(self):
  self.b.submit('EXPLORE','u1');self.b.accept('u1');self.b.stop();self.b.stop();self.assertEqual(self.b.cancel,['u1'])
 def test_wrong_uuid_and_stale_evidence(self):
  self.b.submit('EXPLORE','u1');self.assertFalse(self.b.terminal('u2','CANCELED'))
  self.assertFalse(self.verify(dataclasses.replace(self.a,owned_terminal=(('u1','CANCELED','RESULT_RESPONSE',6.),))))
 def test_attestation_expired_and_generation_mismatch(self):
  for a in (dataclasses.replace(self.a,age_s=6),dataclasses.replace(self.a,gate_boot='gate2'),dataclasses.replace(self.a,fault_epoch=4)):
   with self.subTest(a=a):self.assertFalse(self.verify(a))
 def test_missing_evidence_never_rearms(self):
  self.assertFalse(self.verify(dataclasses.replace(self.a,owned_terminal=())))
  self.assertFalse(self.verify(current_active=['u1']))
  self.assertFalse(self.verify(dataclasses.replace(self.a,role='AGENT'),status_seen=False))
 def test_challenge_single_use(self):
  self.assertTrue(self.verify());self.assertFalse(self.verify())
 def test_untrusted_mac_writer_scope_and_cancel_ack(self):
  bad=[dataclasses.replace(self.a,mac_verified=False),dataclasses.replace(self.a,writer_ids=('other',)),
       dataclasses.replace(self.a,action_servers=()),dataclasses.replace(self.a,owned_terminal=(('u1','CANCELED','CANCEL_ACK',1.),))]
  for a in bad:
   with self.subTest(a=a):self.assertFalse(self.verify(a))
 def test_absent_status_requires_operation_manager_basis(self):
  self.assertFalse(self.verify(dataclasses.replace(self.a,unknown_goal_basis=False),status_seen=False))
  self.assertFalse(self.verify(dataclasses.replace(self.a,role='AGENT'),status_seen=False))

if __name__=='__main__':unittest.main(verbosity=2)
