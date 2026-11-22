"""R3/R4 server-only reproduction: no ROS, sockets, or hardware."""
import sys, tempfile, unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'server'))
from fleet_server.db import DB
from fleet_server.service import Fleet, ApiError
from fleet_server.auth import Principal

GOAL = {'frame_id':'map','x':1.,'y':2.,'yaw_rad':0.}
class ContractTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory(); self.db=DB(str(Path(self.temp.name)/'test.db'))
  self.f=Fleet(self.db,SimpleNamespace(offline_after_s=10,command_ttl_s=120,resend_after_s=.1))
  self.f.hello('r',{'boot_id':'b','protocol_version':2,'safety_contract_version':1}); self.f.robots['r']['reconciled']=True
 def tearDown(self):self.db.c.close();self.temp.cleanup()
 def state(self,status):
  failed=status!='NORMAL'
  return {'safety_gate':{'boot_id':'g','status':status,'age_s':0.,'fault_latched':failed,'cmd_blocked':failed,'movement_permitted':not failed,'fault_epoch':2 if failed else 1},'execution':{'active':None,'nav2_goals':{'known':True,'active_count':0,'received_status_array':True,'age_s':0.}},'operation_mode':'NAVIGATION','operation_revision':1,'map_epoch':1,'localization_epoch':1,'map_version_id':'m','localization_map_version_id':'m','map_verified':True,'localization_status':'CONFIRMED'}
 def test_r3_mission_during_latch_is_rejected(self):
  self.f.ingest_state('r',1,self.state('FAILSAFE_SENSOR_TIMEOUT'),'b')
  lease=self.f.lease_action('r',Principal('op','operator'),'acquire')['epoch']
  with self.assertRaises(ApiError): self.f.create_mission('r','navigate_to_pose',GOAL,'r3',created_by='op',lease_epoch=lease)
 def test_r4_recovery_must_not_dispatch_preexisting_queue(self):
  self.f.ingest_state('r',1,self.state('NORMAL'),'b')
  lease=self.f.lease_action('r',Principal('op','operator'),'acquire')['epoch']
  self.f.create_mission('r','navigate_to_pose',GOAL,'first',created_by='op',lease_epoch=lease)
  m,_=self.f.create_mission('r','navigate_to_pose',GOAL,'r4',created_by='op',lease_epoch=lease)
  self.f.ingest_state('r',2,self.state('FAILSAFE_SENSOR_TIMEOUT'),'b')
  self.f.robots['r']['reconciliation_required']=True
  self.db.x('INSERT INTO meta(k,v) VALUES(?,?)',('reconciliation_block:r','unknown'))
  with self.assertRaises(ApiError) as refused:
   self.f.recover_robot_block('r',Principal('admin','admin'),'verified no active goal')
  self.assertEqual(refused.exception.status,409)
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
  self.assertEqual(self.db.q('SELECT * FROM commands WHERE mission_id=?',(m['mission_id'],)),[])

if __name__ == '__main__':
 unittest.main(verbosity=2)
