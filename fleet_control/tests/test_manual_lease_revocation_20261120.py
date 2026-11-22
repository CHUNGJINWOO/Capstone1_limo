"""Lease changes retire the in-memory bench stream, using existing Fleet fixture."""
import unittest
import test_safety_context_v1 as fixture
from fleet_server.auth import Principal
from fleet_server.service import ApiError

class LeaseRevocation(unittest.TestCase):
 def setUp(self):
  self.t=fixture.Tests('test_old_agent_blocked');self.t.setUp();self.f=self.t.f
 def tearDown(self):self.t.tearDown()
 def test_release_retires_session_but_intruder_cannot(self):
  self.f.manual_benches['r']={'id':'old'}
  with self.assertRaises(ApiError):self.f.lease_action('r',Principal('other','operator'),'release')
  self.assertIn('r',self.f.manual_benches)
  self.f.lease_action('r',Principal('op','operator'),'release')
  self.assertNotIn('r',self.f.manual_benches)
 def test_new_acquire_retires_stale_session(self):
  self.f.lease_action('r',Principal('op','operator'),'release')
  self.f.manual_benches['r']={'id':'old'}
  self.f.lease_action('r',Principal('other','operator'),'acquire')
  self.assertNotIn('r',self.f.manual_benches)
if __name__=='__main__':unittest.main(verbosity=2)
