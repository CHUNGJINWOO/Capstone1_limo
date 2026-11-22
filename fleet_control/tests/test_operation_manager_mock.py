"""Synthetic model tests only: no ROS, hardware, servers or filesystem state."""
import dataclasses,unittest
from operation_manager_mock import Evidence,MockOperations

class Tests(unittest.TestCase):
 def setUp(self):
  self.e=Evidence('gate',1,2,3,True,True,False,True);self.m=MockOperations();self.m.reconcile(self.e,role='admin')
 def request(self,action='start_mapping',key='a'):
  return self.m.request(action,key,self.e,principal='op',role='operator',expected_revision=self.m.revision)
 def test_idempotency_and_conflict(self):
  before=self.m.revision;result=self.request();self.assertEqual(self.m.request('start_mapping','a',self.e,principal='op',role='operator',expected_revision=before),result)
  with self.assertRaises(ValueError):self.m.request('load_map','a',self.e,principal='op',role='operator',expected_revision=before)
 def test_mapping_start_and_stop(self):
  self.request();self.m.complete(self.e,proof=True);self.assertEqual(self.m.state,'MAPPING')
  self.request('stop_mapping','b');self.m.complete(self.e,proof=True);self.assertEqual(self.m.state,'IDLE')
 def test_missing_proof_timeout_and_reset_no_resume(self):
  self.request();self.m.complete(self.e,proof=False);self.assertEqual(self.m.state,'RECONCILIATION_REQUIRED')
  self.m.reconcile(self.e,role='admin');self.assertEqual(self.m.state,'IDLE');self.assertIsNone(self.m.pending)
 def test_restart_and_gate_change_fence(self):
  self.request();self.m.fault_or_restart('gate boot changed');self.assertEqual(self.m.state,'RECONCILIATION_REQUIRED')
  with self.assertRaises(ValueError):self.request('start_mapping','new')
  newer=dataclasses.replace(self.e,boot='new-gate',fault_epoch=0);self.m.reconcile(newer,role='admin');self.assertEqual(self.m.state,'IDLE')
 def test_external_goal_or_stale_or_lease_or_map_change(self):
  for change in ({'external_goal':True},{'fresh':False},{'nav_idle_proven':False},{'lease_epoch':4},{'map_epoch':3},{'healthy':False}):
   with self.subTest(change=change),self.assertRaises(ValueError):self.m.request('start_mapping',str(change),dataclasses.replace(self.e,**change),principal='op',role='operator',expected_revision=self.m.revision)
 def test_profile_failure_and_permissions(self):
  with self.assertRaises(ValueError):self.m.request('switch_profile','x',self.e,principal='viewer',role='viewer',expected_revision=self.m.revision)
  self.request('switch_profile');self.m.fault_or_restart('profile error');self.assertEqual(self.m.state,'RECONCILIATION_REQUIRED')

if __name__=='__main__':unittest.main(verbosity=2)
