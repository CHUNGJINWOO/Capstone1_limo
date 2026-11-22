"""Real loopback Fleet HTTP: empty ROS status cannot clear recovery block."""
import time, unittest
from pathlib import Path
import test_r3_r4_http as fixture

class Tests(fixture.Tests):
 def test_http_admission_and_fault(self):
  admin=(Path(self.tmp.name)/'sec/admin.token').read_text().strip()
  self.assertEqual(self.req('POST','/agent/hello',self.agent,{'robot_id':'r','agent':{'boot_id':'b','protocol_version':2,'safety_contract_version':1}})[0],200)
  ex={'active':None,'nav2_goals':{'known':True,'active_count':0,'received_status_array':True,'age_s':0.}}
  self.assertEqual(self.req('POST','/agent/reconcile',self.agent,{'robot_id':'r','agent_boot_id':'b','journal':{},'journal_ok':True,'execution':ex})[0],200)
  self.f.robots['r']['state']={'execution':ex};self.f.robots['r']['state_ts']=time.time()
  self.f.robots['r']['online']=True;self.f.robots['r']['reconciled']=True;self.f.robots['r']['reconciliation_required']=True
  self.db.x('INSERT INTO meta(k,v) VALUES(?,?)',('reconciliation_block:r','unknown'))
  code,body=self.req('POST','/robots/r/recover',admin,{'note':'verified empty status only'})
  self.assertEqual(code,409);self.assertTrue(self.db.one('SELECT 1 FROM meta WHERE k=?',('reconciliation_block:r',)))
  mid='m_123456789abc'
  self.db.x('INSERT INTO missions(mission_id,robot_id,type,goal_json,status,created_at,updated_at) VALUES(?,?,?,?,?,?,?)',
            (mid,'r','navigate_to_pose','{}','RECONCILIATION_REQUIRED',time.time(),time.time()))
  code,body=self.req('POST',f'/missions/{mid}/recover',admin,{'note':'verified empty status only'})
  self.assertEqual(code,409);self.assertEqual(self.f.mission(mid)['status'],'RECONCILIATION_REQUIRED')

if __name__=='__main__':unittest.main(verbosity=2)
