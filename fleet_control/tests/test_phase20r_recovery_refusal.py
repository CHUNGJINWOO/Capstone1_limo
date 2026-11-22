"""Expected 409 must not disturb unrelated sessions; refusal is auditable."""
import time,unittest
from unittest.mock import patch
import test_safety_context_v1 as fixture
from fleet_server.auth import Principal
from fleet_server.service import ApiError

class Tests(unittest.TestCase):
 def setUp(self):
  self.t=fixture.Tests('test_old_agent_blocked');self.t.setUp();self.f=self.t.f;self.db=self.t.fixture.db
  self.f.hello('s',{'boot_id':'other','protocol_version':2,'safety_contract_version':1})
  self.f.robots['s']['reconciled']=True
 def tearDown(self):self.t.tearDown()
 def test_robot_refusal_preserves_unrelated_reconciled_state(self):
  self.db.x('INSERT INTO meta(k,v) VALUES(?,?)',('reconciliation_block:r','unknown'))
  self.f.robots['r']['reconciliation_required']=True
  with self.assertRaises(ApiError) as err:self.f.recover_robot_block('r',Principal('admin','admin'),'empty status is not evidence')
  self.assertEqual(err.exception.status,409)
  self.assertTrue(self.f.robots['s']['reconciled'])
  self.assertTrue(self.db.one("SELECT 1 FROM events WHERE kind='recovery_refused' AND robot_id='r'"))
  self.assertTrue(self.db.one('SELECT 1 FROM meta WHERE k=?',('reconciliation_block:r',)))
 def test_mission_refusal_is_auditable_and_does_not_reconcile_other_robot(self):
  mid='m_123456789abc';self.db.x('INSERT INTO missions(mission_id,robot_id,type,goal_json,status,created_at,updated_at) VALUES(?,?,?,?,?,?,?)',(mid,'r','navigate_to_pose','{}','RECONCILIATION_REQUIRED',time.time(),time.time()))
  with self.assertRaises(ApiError) as err:self.f.recover_mission(mid,Principal('admin','admin'),'empty status is not evidence')
  self.assertEqual(err.exception.status,409)
  self.assertTrue(self.f.robots['s']['reconciled'])
  self.assertTrue(self.db.one("SELECT 1 FROM events WHERE kind='recovery_refused' AND mission_id=?",(mid,)))
  self.assertEqual(self.f.mission(mid)['status'],'RECONCILIATION_REQUIRED')
 def test_failed_refusal_audit_leaves_both_blocks_unchanged(self):
  self.db.x('INSERT INTO meta(k,v) VALUES(?,?)',('reconciliation_block:r','unknown'))
  self.f.robots['r']['reconciliation_required']=True
  with patch.object(self.db,'event',side_effect=RuntimeError('audit failed')):
   with self.assertRaises(RuntimeError):self.f.recover_robot_block('r',Principal('admin','admin'),'empty status is not evidence')
  self.assertTrue(self.f.robots['s']['reconciled'])
  self.assertTrue(self.f.robots['r']['reconciliation_required'])
  self.assertTrue(self.db.one('SELECT 1 FROM meta WHERE k=?',('reconciliation_block:r',)))

if __name__=='__main__':unittest.main(verbosity=2)
