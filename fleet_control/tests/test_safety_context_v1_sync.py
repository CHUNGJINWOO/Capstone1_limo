"""Safety context persistence and boundary checks, synthetic and server-only."""
import copy, json, time, unittest
from unittest.mock import patch
import test_safety_context_v1 as fixture
from fleet_server.db import DB
from fleet_server.service import Fleet, ApiError

class Tests(unittest.TestCase):
 def setUp(self):
  self.t=fixture.Tests('test_old_agent_blocked'); self.t.setUp()
  self.f=self.t.f; self.db=self.t.fixture.db
 def tearDown(self): self.t.tearDown()
 def test_db_delivery_and_restart_preserve_context_without_replay(self):
  mission,_=self.t.create()
  row=self.db.one('SELECT * FROM commands WHERE mission_id=?',(mission['mission_id'],))
  before=json.loads(row['payload_json'])['safety_context']
  delivered=self.f._deliver('r')['commands'][0]
  self.assertEqual(before,delivered['payload']['safety_context'])
  self.db.c.close(); self.t.fixture.db=DB(self.t.fixture.path)
  db=self.t.fixture.db; fresh=Fleet(db,fixture.base.CFG)
  after=json.loads(db.one('SELECT payload_json FROM commands WHERE command_id=?',(row['command_id'],))['payload_json'])['safety_context']
  self.assertEqual(before,after)
  self.assertEqual(fresh.mission(mission['mission_id'])['status'],'RECONCILIATION_REQUIRED')
  self.assertEqual(fresh._deliver('r')['commands'],[])
 def test_expiration_prevents_delivery(self):
  mission,_=self.t.create(); row=self.db.one('SELECT * FROM commands WHERE mission_id=?',(mission['mission_id'],))
  with patch('fleet_server.service.time.time',return_value=row['expires_at']+1):
   self.assertEqual(self.f._deliver('r')['commands'],[])
  self.assertEqual(self.db.one('SELECT attempts FROM commands WHERE command_id=?',(row['command_id'],))['attempts'],0)
 def test_expired_after_delivery_requires_reconciliation(self):
  mission,_=self.t.create(); cmd=self.f._deliver('r')['commands'][0]
  with patch('fleet_server.service.time.time',return_value=cmd['expires_at']+1):
   self.f.housekeeping()
  self.assertEqual(self.f.mission(mission['mission_id'])['status'],'RECONCILIATION_REQUIRED')
 def test_fault_epoch_change_and_gate_restart_fence(self):
  mission,_=self.t.create(); changed=copy.deepcopy(self.t.state)
  changed['safety_gate']['boot_id']='gate2'; changed['safety_gate']['fault_epoch']=0
  self.assertEqual(self.f.ingest_state('r',3,changed,'boot1')['commands'],[])
  self.assertEqual(self.f.mission(mission['mission_id'])['status'],'FAILED')
  with self.assertRaises(ApiError):self.f.create_mission('r','navigate_to_pose',fixture.base.GOAL,'later',created_by='op',lease_epoch=self.t.fixture.lease)
 def test_missing_status_observation_never_inferred_from_publisher(self):
  self.t.robot['state']['execution']['nav2_goals'].update(publishers=1,received_status_array=False)
  with self.assertRaises(ApiError):self.t.create()
  self.t.robot['state']['execution']['nav2_goals'].update(received_status_array=True)
  del self.t.robot['state']['execution']['nav2_goals']['age_s']
  with self.assertRaises(ApiError):self.t.create()

if __name__=='__main__':unittest.main(verbosity=2)
