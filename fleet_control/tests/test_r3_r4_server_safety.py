"""Safety admission and fault injection with synthetic server data only."""
import copy, sys, tempfile, threading, time, unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'server'))
from fleet_server.auth import Principal
from fleet_server.db import DB
from fleet_server.service import Fleet, ApiError

GOAL={'frame_id':'map','x':1.,'y':2.,'yaw_rad':0.}
CFG=SimpleNamespace(offline_after_s=10.,command_ttl_s=120.,resend_after_s=.01)
class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.path=str(Path(self.tmp.name)/'test.db');self.db=DB(self.path);self.f=Fleet(self.db,CFG)
  self.f.hello('r',{'boot_id':'boot1','protocol_version':2,'safety_contract_version':1});self.f.robots['r']['reconciled']=True
  self.state={'safety_gate':{'boot_id':'gate1','status':'NORMAL','age_s':0.,'fault_epoch':1,'fault_latched':False,'cmd_blocked':False,'movement_permitted':True},'operation_mode':'NAVIGATION','operation_revision':1,'map_epoch':1,'localization_epoch':1,'map_version_id':'map1','localization_map_version_id':'map1','map_verified':True,'localization_status':'CONFIRMED','execution':{'active':None,'nav2_goals':{'known':True,'active_count':0,'received_status_array':True,'age_s':0.}}}
  self.f.ingest_state('r',1,self.state,'boot1');self.lease=self.f.lease_action('r',Principal('op','operator'),'acquire')['epoch']
 def tearDown(self):self.db.c.close();self.tmp.cleanup()
 def create(self,key='k'):
  return self.f.create_mission('r','navigate_to_pose',GOAL,key,created_by='op',lease_epoch=self.lease)
 def fault(self):
  s=copy.deepcopy(self.state);s['safety_gate'].update(status='FAILSAFE_SENSOR_TIMEOUT',fault_latched=True,cmd_blocked=True,fault_epoch=2)
  return self.f.ingest_state('r',2,s,'boot1')
 def test_new_mission_and_idempotency(self):
  m,created=self.create();n,again=self.create();self.assertTrue(created);self.assertFalse(again);self.assertEqual(m['mission_id'],n['mission_id'])
  with self.assertRaises(ApiError):self.f.create_mission('r','navigate_to_pose',{**GOAL,'x':9},'k',created_by='op',lease_epoch=self.lease)
  delivery=self.f._deliver('r');self.assertEqual(len(delivery['commands']),1)
 def test_fault_rejects_and_invalidates_undelivered_then_no_recovery_replay(self):
  m,_=self.create();queued,_=self.create('second')
  self.assertEqual(len(self.f.mission(queued['mission_id'])['commands']),0)
  self.fault();self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
  self.assertEqual(self.f.mission(queued['mission_id'])['status'],'FAILED')
  self.assertEqual(self.f._deliver('r')['commands'],[])
  with self.assertRaises(ApiError):self.create('new')
  self.db.x('INSERT INTO meta(k,v) VALUES(?,?)',('reconciliation_block:r','unknown'))
  self.f.robots['r']['reconciliation_required']=True
  with self.assertRaises(ApiError) as refused:
   self.f.recover_robot_block('r',Principal('admin','admin'),'verified no active goal')
  self.assertEqual(refused.exception.status,409)
  self.assertEqual(self.f._deliver('r')['commands'],[])
  self.db.c.close();self.db=DB(self.path);self.f=Fleet(self.db,CFG)
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
  self.assertEqual(self.f.mission(queued['mission_id'])['status'],'FAILED')
 def test_delivered_fault_outcome_uncertain(self):
  m,_=self.create();self.assertEqual(len(self.f._deliver('r')['commands']),1);self.fault()
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'RECONCILIATION_REQUIRED')
  self.assertEqual(self.f._deliver('r')['commands'],[])
 def test_missing_stale_and_map_mismatch_fail_closed(self):
  for update in ({'safety_gate':None},{'operation_mode':None},{'map_epoch':None},{'map_verified':False},{'localization_status':'UNKNOWN'},{'localization_map_version_id':'other'},{'execution':{'nav2_goals':{'known':True,'active_count':0}}},{'execution':{'active':None,'nav2_goals':{'known':True,'active_count':0,'received_status_array':False,'age_s':0}}}):
   with self.subTest(update=update):
    old=self.f.robots['r']['state'];self.f.robots['r']['state']={**self.state,**update}
    with self.assertRaises(ApiError):self.create(str(update))
    self.f.robots['r']['state']=old
  self.f.robots['r']['state_ts']=time.time()-3
  with self.assertRaises(ApiError):self.create('stale')
 def test_map_or_localization_epoch_change_invalidates_pending(self):
  m,_=self.create();changed=copy.deepcopy(self.state);changed['localization_epoch']=2
  self.f.ingest_state('r',2,changed,'boot1')
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
  self.assertEqual(self.f._deliver('r')['commands'],[])
 def test_lease_and_epoch_change_fence_pending(self):
  m,_=self.create();self.f.lease_action('r',Principal('op','operator'),'release')
  self.assertEqual(self.f._deliver('r')['commands'],[]);self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
 def test_lease_reacquisition_does_not_reuse_authorization(self):
  m,_=self.create();self.f.lease_action('r',Principal('op','operator'),'release')
  epoch=self.f.lease_action('r',Principal('op','operator'),'acquire')['epoch']
  self.assertNotEqual(epoch,self.lease);self.assertEqual(self.f._deliver('r')['commands'],[])
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
 def test_state_stales_after_admission_before_delivery(self):
  m,_=self.create();self.f.robots['r']['state_ts']=time.time()-3
  self.assertEqual(self.f._deliver('r')['commands'],[])
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
 def test_rollback_fault_invalidation(self):
  m,_=self.create();before=self.f.robots['r']['state']
  original=self.db.event
  def event(kind,*args,**kwargs):
   if kind == 'mission_status':raise RuntimeError('injected after command and mission updates')
   return original(kind,*args,**kwargs)
  with patch.object(self.db,'event',side_effect=event):
   with self.assertRaises(RuntimeError):self.fault()
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'QUEUED')
  self.assertEqual(self.f.mission(m['mission_id'])['commands'][0]['status'],'PENDING')
  self.assertEqual(self.f.robots['r']['state'],before);self.assertFalse(self.f.robots['r']['reconciled'])
 def test_new_boot_fences_undelivered_and_old_boot_rejected(self):
  m,_=self.create();self.f.hello('r',{'boot_id':'boot2','protocol_version':2})
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
  with self.assertRaises(ApiError):self.f.hello('r',{'boot_id':'boot1','protocol_version':2})
 def test_concurrent_fault_and_create_no_late_admission(self):
  barrier=threading.Barrier(3);errors=[]
  def create():
   barrier.wait()
   try:self.create('race')
   except ApiError:pass
   except Exception as e:errors.append(e)
  def fault():
   barrier.wait()
   try:self.fault()
   except Exception as e:errors.append(e)
  ts=[threading.Thread(target=create),threading.Thread(target=fault)]
  for t in ts:t.start()
  barrier.wait()
  for t in ts:t.join(5)
  self.assertFalse(errors);self.assertFalse(any(t.is_alive() for t in ts));self.assertEqual(self.f._deliver('r')['commands'],[])
 def test_lease_expiry_and_wrong_epoch_rejected(self):
  with self.assertRaises(ApiError):
   self.f.create_mission('r','navigate_to_pose',GOAL,'wrong',created_by='op',lease_epoch=self.lease+1)
  m,_=self.create();self.db.x('UPDATE locks SET expires_at=? WHERE robot_id=?',(time.time()-1,'r'))
  self.assertEqual(self.f._deliver('r')['commands'],[])
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
 def test_fault_epoch_change_then_reset_cannot_replay(self):
  m,_=self.create();self.fault()
  recovered=copy.deepcopy(self.state);recovered['safety_gate']['fault_epoch']=2
  self.f.ingest_state('r',3,recovered,'boot1')
  self.assertEqual(self.f._deliver('r')['commands'],[])
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
 def test_epoch_regression_fails_closed(self):
  s=copy.deepcopy(self.state);s['map_epoch']=2
  self.f.ingest_state('r',2,s,'boot1');self.f.ingest_state('r',3,self.state,'boot1')
  with self.assertRaises(ApiError):self.create('regressed')
 def test_fault_epoch_cannot_regress_on_reset(self):
  self.fault();self.f.ingest_state('r',3,self.state,'boot1')
  with self.assertRaises(ApiError):self.create('old-fault-epoch')
 def test_server_restart_invalidates_pending(self):
  m,_=self.create();self.db.c.close();self.db=DB(self.path);self.f=Fleet(self.db,CFG)
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
 def test_same_boot_reconnect_invalidates_pending(self):
  m,_=self.create();self.f.hello('r',{'boot_id':'boot1','protocol_version':2})
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
 def test_cancel_result_lost_is_reconciliation(self):
  m,_=self.create();nav=self.f._deliver('r')['commands'][0]
  self.f.ack_command('r',nav['command_id'],'ACCEPTED',agent_boot_id='boot1')
  self.f.cancel_mission(m['mission_id']);cancel=self.f._deliver('r')['commands'][0]
  self.f.ack_command('r',cancel['command_id'],'RECONCILIATION_REQUIRED','lost cancel result',agent_boot_id='boot1')
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'RECONCILIATION_REQUIRED')
 def test_recover_mission_does_not_dispatch_prior_queue(self):
  m,_=self.create();self.f._deliver('r');self.fault()
  with self.assertRaises(ApiError) as refused:
   self.f.recover_mission(m['mission_id'],Principal('admin','admin'),'verified no active goal')
  self.assertEqual(refused.exception.status,409)
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'RECONCILIATION_REQUIRED')
  self.assertEqual(self.f._deliver('r')['commands'],[])
 def test_concurrent_recovery_and_dispatch_do_not_restart(self):
  m,_=self.create();self.fault()
  self.db.x('INSERT INTO meta(k,v) VALUES(?,?)',('reconciliation_block:r','unknown'))
  self.f.robots['r']['reconciliation_required']=True
  b=threading.Barrier(3);errors=[];refused=[]
  def recover():
   b.wait()
   try:self.f.recover_robot_block('r',Principal('admin','admin'),'verified no active goal')
   except ApiError as e:refused.append(e.status)
   except Exception as e:errors.append(e)
  def dispatch():
   b.wait()
   try:self.f.dispatch_next('r')
   except Exception as e:errors.append(e)
  ts=[threading.Thread(target=recover),threading.Thread(target=dispatch)]
  for t in ts:t.start()
  b.wait()
  for t in ts:t.join(5)
  self.assertFalse(errors);self.assertEqual(refused,[409]);self.assertFalse(any(t.is_alive() for t in ts))
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'FAILED')
  self.assertEqual(self.f._deliver('r')['commands'],[])
 def test_creation_event_failure_rolls_back_mission_and_command(self):
  with patch.object(self.db,'event',side_effect=RuntimeError('injected')):
   with self.assertRaises(RuntimeError):self.create('rollback')
  self.assertEqual(self.db.q('SELECT * FROM missions'),[])
  self.assertEqual(self.db.q('SELECT * FROM commands'),[])
 def test_recovery_failure_rolls_back_and_stays_blocked(self):
  m,_=self.create();self.f._deliver('r');self.fault()
  before_reconciled=self.f.robots['r']['reconciled']
  original=self.db.event
  def event(kind,*args,**kwargs):
   if kind == 'recovery_refused':raise RuntimeError('injected during recovery refusal')
   return original(kind,*args,**kwargs)
  with patch.object(self.db,'event',side_effect=event):
   with self.assertRaises(RuntimeError):self.f.recover_mission(m['mission_id'],Principal('admin','admin'),'verified no active goal')
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'RECONCILIATION_REQUIRED')
  self.assertEqual(self.f.mission(m['mission_id'])['commands'][0]['status'],'RECONCILIATION_REQUIRED')
  self.assertEqual(self.f.robots['r']['reconciled'],before_reconciled)
if __name__=='__main__':unittest.main(verbosity=2)
