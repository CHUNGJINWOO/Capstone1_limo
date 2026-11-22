"""Lease and recovery regression under explicit synthetic Safety Context v1."""
import threading,unittest
import test_safety_context_v1 as fixture
from fleet_server.auth import Principal
from fleet_server.service import ApiError

class Tests(unittest.TestCase):
 def setUp(self):self.t=fixture.Tests('test_old_agent_blocked');self.t.setUp();self.f=self.t.f
 def tearDown(self):self.t.tearDown()
 def test_competing_leases_one_owner_and_mission(self):
  self.f.lease_action('r',Principal('op','operator'),'release')
  barrier=threading.Barrier(3);won=[];denied=[];unexpected=[]
  def acquire(name):
   barrier.wait()
   try:won.append((name,self.f.lease_action('r',Principal(name,'operator'),'acquire')))
   except ApiError as e:denied.append(e.status)
   except Exception as e:unexpected.append(repr(e))
  threads=[threading.Thread(target=acquire,args=(name,)) for name in ('web','rmf')]
  for thread in threads:thread.start()
  barrier.wait()
  for thread in threads:thread.join(3)
  self.assertFalse(any(t.is_alive() for t in threads));self.assertFalse(unexpected)
  self.assertEqual(len(won),1);self.assertEqual(denied,[409])
  name,lease=won[0];other='rmf' if name=='web' else 'web'
  m,created=self.f.create_mission('r','navigate_to_pose',fixture.base.GOAL,'lease-race',created_by=name,lease_epoch=lease['epoch'])
  self.assertTrue(created)
  with self.assertRaises(ApiError) as err:self.f.create_mission('r','navigate_to_pose',fixture.base.GOAL,'other',created_by=other,lease_epoch=lease['epoch'])
  self.assertEqual(err.exception.status,409)
 def test_missing_and_expired_lease_rejected(self):
  self.f.lease_action('r',Principal('op','operator'),'release')
  with self.assertRaises(ApiError):self.f.create_mission('r','navigate_to_pose',fixture.base.GOAL,'no-lease',created_by='op')
  lease=self.f.lease_action('r',Principal('op','operator'),'acquire')
  self.f.db.x('UPDATE locks SET expires_at=0 WHERE robot_id=?',('r',))
  with self.assertRaises(ApiError):self.f.create_mission('r','navigate_to_pose',fixture.base.GOAL,'expired',created_by='op',lease_epoch=lease['epoch'])

if __name__=='__main__':unittest.main(verbosity=2)
