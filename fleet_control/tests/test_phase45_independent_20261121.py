"""Independent Phase 4/5 review against a fixed snapshot. No live services or hardware."""
import os,sys,time,threading,unittest
from unittest.mock import patch
from pathlib import Path
root=Path(os.environ['CAPSTONE45_SNAPSHOT'])
sys.path.insert(0,str(root/'fleet_control/tests'))
sys.path.insert(0,str(root/'fleet_control/server'))
import test_phase4_readiness as p4
import test_phase5_guide as p5
from fleet_server.service import ApiError

class ReadyBoundaries(unittest.TestCase):
 def setUp(self):self.t=p4.Readiness('test_ready_allows_mode_request');self.t.setUp();self.f=self.t.f
 def test_empty_supervisor_report_cannot_be_ready(self):
  self.f.robots['R']['state'].pop('motion_watchdog',None)
  self.f.supervisor_report('R',{'boot_id':'empty','services':[],'ros':{'topics':{}}})
  result=self.f.readiness('R');print('OBSERVED empty services/topics and missing watchdog:',result['level'],result['reasons'],flush=True)
  self.assertNotEqual(result['level'],'READY')
 def test_stale_watchdog_and_gate_age_cannot_be_ready(self):
  self.t.report();state=self.f.robots['R']['state']
  state['safety_gate']['age_s']=30.;state['motion_watchdog']['age_s']=30.
  result=self.f.readiness('R');print('OBSERVED stale Gate and Watchdog ages:',result['level'],result['reasons'],flush=True)
  self.assertNotEqual(result['level'],'READY')
 def test_watchdog_locked_is_not_ready_for_motion(self):
  self.t.report();self.f.robots['R']['state']['motion_watchdog']['state']='LOCKED'
  result=self.f.readiness('R');print('OBSERVED Watchdog LOCKED:',result['level'],result['reasons'],flush=True)
  self.assertNotEqual(result['level'],'READY')
 def test_gate_pending_but_command_blocked_is_not_ready(self):
  self.t.report();self.f.robots['R']['state']['safety_gate'].update(status='STALL_PENDING',cmd_blocked=True)
  result=self.f.readiness('R');print('OBSERVED STALL_PENDING with cmd_blocked:',result['level'],result['reasons'],flush=True)
  self.assertNotEqual(result['level'],'READY')
 def test_nonfinite_sensor_age_cannot_be_ready(self):
  self.t.report(ros={'topics':{'/scan':{'age_s':float('nan'),'max_age_s':1.0}}})
  result=self.f.readiness('R');print('OBSERVED NaN sensor age:',result['level'],result['reasons'],flush=True)
  self.assertNotEqual(result['level'],'READY')
 def test_legacy_unknown_mode_with_active_goal_cannot_restart(self):
  self.f.cfg.require_operating_mode=False;self.t.report()
  self.f.robots['R']['state']['execution']={'active':{'owner':'external','goal_uuid':'a'*32}}
  with self.assertRaises(ApiError):self.f.request_service_action('R',p4.ADMIN,{'service':'watchdog','action':'restart'})
 def test_fault_or_reconciliation_is_not_idle_for_service_stop(self):
  self.t.report();r=self.f.robots['R'];r['reconciliation_required']=True
  rec=self.f._mode_new('FAULT',None,'unresolved goal');self.f._mode_save('R',rec)
  with self.assertRaises(ApiError):self.f.request_service_action('R',p4.ADMIN,{'service':'watchdog','action':'restart'})
 def test_missing_flag_allows_navigation_mode_without_supervisor(self):
  self.f.cfg.require_supervisor_ready=False; r=self.f.robots['R'];r['agent'].update(missions_enabled=True,locked_velocity_arbiter=True)
  r['state'].update(execution={'active':None,'nav2_goals':{'active_count':0},'nav_idle':{'ok':True}},
   motion_watchdog={'required':True,'state':'MOTION_AUTHORIZED','agent_boot_matches':True},
   velocity_arbiter={'operating_mode':'IDLE','selected_linear':0.,'selected_turn':0.,'ready':True,'blocker':None},
   mapping_worker={'enabled':True,'status':'IDLE','processes_started':False})
  self.assertEqual(self.f.readiness('R')['level'],'DEGRADED')
  self.t.mode('NAVIGATION');r['state_ts']=time.time();self.f.mode_view('R');time.sleep(1.05);r['state_ts']=time.time();rec=self.f.mode_view('R')
  if rec['transition']['state']=='CONFIRMING':
   r['state']['velocity_arbiter'].update(operating_mode='NAVIGATION',blocker='NO_OWNED_GOAL');time.sleep(.002);r['state_ts']=time.time();rec=self.f.mode_view('R')
  print('OBSERVED no supervisor, no READY flag:',rec['mode'],rec['transition']['state'],rec['transition'].get('blocker'),flush=True)
  self.assertEqual((rec['mode'],rec['transition']['state']),('NAVIGATION','ACTIVE'))

class VisitorBoundaries(unittest.TestCase):
 def setUp(self):self.t=p5.Guide('test_visitor_links');self.t.setUp();self.f=self.t.f
 def tearDown(self):self.t.tearDown()
 def test_nonowner_cannot_close_service_using_nonboolean_enabled(self):
  from fleet_server.auth import Principal
  self.f.set_guide_service(p5.OPER,{'robot_id':'R','enabled':True})
  with self.assertRaises(ApiError):
   self.f.set_guide_service(Principal('other_operator','operator'),{'robot_id':'R','enabled':'false'})
  self.assertTrue(self.f.guide_service_view('R')['enabled'])
 def test_different_link_cannot_view_or_cancel_when_session_id_known(self):
  d=self.t.dest();self.f.set_guide_service(p5.OPER,{'robot_id':'R','enabled':True})
  a,_=self.t.visitor(session='known_session_123456789');b,_=self.t.visitor(session='known_session_123456789')
  req=self.f.create_guide_request(a,{'destination_id':d['destination_id']})
  print('OBSERVED different-link same-session view:',self.f.guide_request_view(req['request_id'],b)['status'],flush=True)
  with self.assertRaises(ApiError):self.f.cancel_guide_request(b,req['request_id'])
 def test_older_queued_visitor_is_not_overtaken_at_completion(self):
  d=self.t.dest();self.f.set_guide_service(p5.OPER,{'robot_id':'R','enabled':True})
  v1,_=self.t.visitor('first_visitor_123456789');v2,_=self.t.visitor('queued_visitor_1234567');v3,_=self.t.visitor('new_visitor_1234567890')
  r1=self.f.create_guide_request(v1,{'destination_id':d['destination_id']})
  r2=self.f.create_guide_request(v2,{'destination_id':d['destination_id']});self.assertEqual(r2['status'],'QUEUED')
  mid=self.f.guide_request_view(r1['request_id'])['mission_id']
  self.t.db.x("UPDATE missions SET status='SUCCEEDED' WHERE mission_id=?",(mid,))
  # Interleave a new browser request after guide_tick's completion update but before its queued-dispatch loop.
  self.f._req_update(r1['request_id'],'COMPLETED')
  r3=self.f.create_guide_request(v3,{'destination_id':d['destination_id']})
  print('OBSERVED older queued and later arrival:',self.f.guide_request_view(r2['request_id'],v2)['status'],r3['status'],flush=True)
  self.assertEqual(r3['status'],'QUEUED','new arrivals must not overtake the older queued request')
 def test_housekeeping_and_new_request_do_not_create_two_missions(self):
  d=self.t.dest();self.f.set_guide_service(p5.OPER,{'robot_id':'R','enabled':True})
  v1,_=self.t.visitor('first_visitor_123456789');v2,_=self.t.visitor('queued_visitor_1234567');v3,_=self.t.visitor('new_visitor_1234567890')
  r1=self.f.create_guide_request(v1,{'destination_id':d['destination_id']})
  r2=self.f.create_guide_request(v2,{'destination_id':d['destination_id']});self.assertEqual(r2['status'],'QUEUED')
  mid=self.f.guide_request_view(r1['request_id'])['mission_id']
  self.t.db.x("UPDATE missions SET status='SUCCEEDED' WHERE mission_id=?",(mid,))
  barrier=threading.Barrier(2);entered=threading.Event();original=self.f._robot_busy;errors=[]
  def synchronized_busy(rid):
   result=original(rid);entered.set();barrier.wait(timeout=3);return result
  def housekeeping():
   try:self.f.guide_tick()
   except Exception as e:errors.append(repr(e))
  def arrival():
   try:self.f.create_guide_request(v3,{'destination_id':d['destination_id']})
   except Exception as e:errors.append(repr(e))
  with patch.object(self.f,'_robot_busy',side_effect=synchronized_busy):
   t1=threading.Thread(target=housekeeping);t1.start();self.assertTrue(entered.wait(3))
   t2=threading.Thread(target=arrival);t2.start();t1.join(5);t2.join(5)
  self.assertFalse(t1.is_alive() or t2.is_alive());self.assertFalse(errors,errors)
  active=self.t.db.q("SELECT request_id,status FROM guide_requests WHERE status IN ('ACCEPTED','IN_PROGRESS')")
  print('OBSERVED simultaneous dispatch active requests:',len(active),flush=True)
  self.assertLessEqual(len(active),1,'only one guide request may be accepted per robot')
 def test_on_disk_map_tamper_after_load_fails_closed(self):
  d=self.t.dest();self.f.set_guide_service(p5.OPER,{'robot_id':'R','enabled':True})
  row=self.t.db.one('SELECT * FROM guide_destinations WHERE destination_id=?',(d['destination_id'],))
  version=self.t.reg.one('SELECT storage_key FROM map_versions WHERE map_version_id=?',(d['map_version_id'],))
  path=Path(self.t.reg.bundles_dir)/version['storage_key']/'map.pgm'
  path.chmod(0o600)  # only this test fixture's temporary copy
  with path.open('r+b') as f:
   f.seek(-1,2);b=f.read(1);f.seek(-1,2);f.write(bytes([b[0]^1]))
  self.assertFalse(self.t.reg.verify_bundle_integrity(d['map_version_id'])['ok'])
  result=self.f.destination_block(row);print('OBSERVED verified registry metadata with tampered bundle:',result,flush=True)
  self.assertIsNotNone(result,'tampered map bundle must not remain available')
 def test_quarantine_lookup_error_cannot_mark_available(self):
  d=self.t.dest();self.f.set_guide_service(p5.OPER,{'robot_id':'R','enabled':True})
  row=self.t.db.one('SELECT * FROM guide_destinations WHERE destination_id=?',(d['destination_id'],))
  self.t.reg.quarantine_map_version(d['map_version_id'],reason='independent review fixture',actor='test',actor_role='admin')
  self.assertEqual(self.f.destination_block(row)[0],'MAP_QUARANTINED')
  original=self.t.reg.one
  def fail_query(sql,*args,**kwargs):
   if 'map_quarantines' in sql:raise RuntimeError('injected registry failure')
   return original(sql,*args,**kwargs)
  with patch.object(self.t.reg,'one',side_effect=fail_query):
   result=self.f.destination_block(row);print('OBSERVED failed quarantine check:',result,flush=True)
   self.assertIsNotNone(result,'quarantine lookup failure must not permit dispatch')
if __name__=='__main__':unittest.main(verbosity=2)
