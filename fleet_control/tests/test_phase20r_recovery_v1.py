"""Legacy recovery regressions with an explicit synthetic Safety Context v1 fixture.

This is a non-ROS mock; it does not implement Gate attestation or robot control.
"""
import json,time,unittest
import test_recovery as old
from fleet_server.auth import Principal
from fleet_server.service import ApiError

BaseRobot=old.Robot
class SafetyRobot(BaseRobot):
 def snapshot(self):
  result=super().snapshot();ex=result['execution']
  ex['nav2_goals']={'known':True,'received_status_array':True,'active_count':1 if self.active else 0,'age_s':0.0}
  result.update(safety_gate={'boot_id':'synthetic-gate','status':'NORMAL','age_s':0.,'fault_epoch':1,'fault_latched':False,'cmd_blocked':False,'movement_permitted':True},operation_mode='NAVIGATION',operation_revision=1,map_epoch=1,localization_epoch=1,map_version_id='m',localization_map_version_id='m',map_verified=True,localization_status='CONFIRMED')
  return result
old.Robot=SafetyRobot

class Tests(old.RecoveryTests):
 def mission(self):
  lease=self.fleet.lease_view('r')
  if not lease:lease=self.fleet.lease_action('r',Principal('op','operator'),'acquire')
  return self.fleet.create_mission('r','navigate_to_pose',old.GOAL,'key'+str(time.time_ns()),created_by='op',lease_epoch=lease['epoch'])[0]['mission_id']
 def test_agent_restart_lost_handle_blocks_reexecution_and_next_mission(self):
  a,r=self.agent();mid=self.mission();old.wait(lambda:self.status(mid)=='EXECUTING');self.stop_agent(a)
  a2,r2=self.agent(SafetyRobot(r.backend),'boot2');old.wait(lambda:self.status(mid)=='RECONCILIATION_REQUIRED')
  with self.assertRaises(ApiError):self.mission()
  self.assertEqual(r.backend['starts'],1);self.assertTrue(r.backend['running']);self.assertIsNone(r2.active)
 def test_server_restart_retains_owned_handle_without_goal_replay(self):
  a,r=self.agent();mid=self.mission();old.wait(lambda:self.status(mid)=='EXECUTING')
  self.stop_server();old.wait(lambda:not a.connected);self.start_server()
  old.wait(lambda:a.stats['reconnects']>=2 and a.connected)
  self.assertEqual(self.status(mid),'RECONCILIATION_REQUIRED');self.assertEqual(r.backend['starts'],1)
  with self.assertRaises(ApiError):self.mission()
 def test_timeout_during_execution_offline_detection_and_recovery(self):
  a,r=self.agent();mid=self.mission();old.wait(lambda:self.status(mid)=='EXECUTING')
  a.http.block=True;old.wait(lambda:not a.connected);time.sleep(.25);self.fleet.housekeeping()
  self.assertFalse(self.fleet.robots['r']['online']);self.assertEqual(self.status(mid),'RECONCILIATION_REQUIRED')
  a.http.block=False;old.wait(lambda:a.connected)
  self.assertEqual(self.status(mid),'RECONCILIATION_REQUIRED');self.assertEqual(r.backend['starts'],1)
 def test_same_command_repeated_executes_once_and_ownership_is_checked(self):
  a,r=self.agent();mid=self.mission();old.wait(lambda:self.status(mid)=='EXECUTING')
  c=self.db.one('SELECT * FROM commands WHERE mission_id=?',(mid,))
  cmd={'command_id':c['command_id'],'robot_id':'r','mission_id':mid,'type':'navigate_to_pose','owner_boot_id':a.boot_id,'seq':c['seq'],'issued_at':c['created_at'],'expires_at':c['expires_at'],'payload':json.loads(c['payload_json'])}
  for _ in range(5):a._on_command(cmd)
  self.assertEqual(r.backend['starts'],1)
  with self.assertRaises(ApiError):self.fleet.ack_command('r',c['command_id'],'SUCCEEDED',agent_boot_id='wrong-boot')
 def test_unknown_delivered_command_is_fenced_after_reconnect(self):
  self.fleet.hello('r',{'protocol_version':2,'safety_contract_version':1,'boot_id':'boot1'});self.fleet.reconcile('r','boot1',{}, {},True)
  st=SafetyRobot().snapshot();self.fleet.ingest_state('r',1,st,'boot1');mid=self.mission();self.fleet.ingest_state('r',2,st,'boot1')
  self.fleet.hello('r',{'protocol_version':2,'safety_contract_version':1,'boot_id':'boot1'});self.fleet.reconcile('r','boot1',{}, {},True)
  self.assertEqual(self.status(mid),'RECONCILIATION_REQUIRED')
  self.assertEqual(self.fleet.ingest_state('r',3,st,'boot1')['commands'],[])
 def test_unowned_active_mission_blocks_dispatch_and_survives_restart(self):
  a,r=self.agent();self.stop_agent(a)
  unknown=SafetyRobot().snapshot();unknown['execution']['active']={'command_id':'unknown','mission_id':'unknown','goal_handle_available':True}
  response=self.fleet.ingest_state('r',999,unknown,a.boot_id);self.assertTrue(response['reconciliation_required'])
  with self.assertRaises(ApiError):self.mission()
  self.stop_server();self.start_server();self.fleet.hello('r',{'protocol_version':2,'safety_contract_version':1,'boot_id':a.boot_id})
  result=self.fleet.reconcile('r',a.boot_id,{}, {},True);self.assertTrue(result['reconciliation_required'])

if __name__=='__main__':unittest.main(verbosity=2)
