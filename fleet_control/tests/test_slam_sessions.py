import sys,time,unittest
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'server'))
from fleet_server.db import DB
from fleet_server.service import Fleet,ApiError

class Tests(unittest.TestCase):
 def setUp(self):
  self.f=Fleet(DB(':memory:'),SimpleNamespace(offline_after_s=6,command_ttl_s=120,resend_after_s=1))
  self.f.robots['R']={'online':True,'state_ts':time.time(),'state':{'slam':{'enabled':True,'status':'IDLE'},'recording':{'enabled':True,'status':'IDLE'}}}
 def test_active_session_mismatch(self):
  self.f.robots['R']['state']['recording']={'enabled':True,'status':'RECORDING','session_id':'A'}
  with self.assertRaises(ApiError) as ctx:self.f.slam_command('R','start','B','operator')
  self.assertEqual(ctx.exception.code,'session_mismatch')
  self.f.robots['R']['state']['recording']['status']='IDLE'
  self.f.robots['R']['state']['slam']={'enabled':True,'status':'RUNNING','session_id':'A'}
  with self.assertRaises(ApiError) as ctx:self.f.recording_command('R','start','B','operator')
  self.assertEqual(ctx.exception.code,'session_mismatch')
 def test_pending_peer_conflict(self):
  self.f.recording_command('R','start','A','operator')
  with self.assertRaises(ApiError) as ctx:self.f.slam_command('R','start','B','operator')
  self.assertEqual(ctx.exception.code,'session_mismatch')
 def test_manual_stop_without_floor_can_inhibit_and_retry(self):
  self.f.robots['R']['state']['joystick']={'enabled':True}
  self.f.robots['R']['state']['slam']={'enabled':True,'status':'RUNNING','session_id':'A'}
  first=self.f.slam_command('R','stop','A','operator')
  row=self.f.db.one('SELECT payload_json FROM commands WHERE command_id=?',(first['command_id'],))
  import json
  self.assertTrue(json.loads(row['payload_json'])['manual_stop']);self.assertIsNone(json.loads(row['payload_json'])['floor_id'])
  self.f.db.x('UPDATE commands SET status=? WHERE command_id=?',('FAILED',first['command_id']))
  self.f.robots['R']['state']['slam']['status']='ERROR'
  retry=self.f.slam_command('R','stop','A','operator','test-floor')
  self.assertNotEqual(first['command_id'],retry['command_id'])
 def test_status_remains_readable(self):
  self.assertEqual('IDLE',self.f.recording_status('R')['recording']['status'])
if __name__=='__main__':unittest.main()
