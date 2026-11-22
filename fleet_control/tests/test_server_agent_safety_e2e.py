"""Real loopback HTTP server + AgentCore + RosRobot with explicit ROS stubs."""
import time, unittest
import test_r3_r4_http as server_fixture
import test_agent_safety_v1 as agent_fixture
from limo_robot_agent.core import AgentCore

class Tests(server_fixture.Tests):
 def setUp(self):
  super().setUp()
  self.robot=agent_fixture._make_ready_robot(); self.robot.robot_id='r'; self.robot.agent_boot_id='b'
  self.robot.mode='sim'; self.robot.simulated=True  # Synthetic fixture; real mode is fail-closed without Gate attestation.
  self.a=AgentCore('r','http://127.0.0.1:'+str(self.port),self.agent,self.robot.snapshot,self.robot,{'boot_id':'b'},str(self.root if hasattr(self,'root') else self.tmp.name),log=lambda x:None,require_safety_context=True)
  self.robot.session_fn=self.a.session_snapshot; self.robot.execution_allowed=self.a.is_execution_allowed; self.robot.on_command_meta=self.a.record_command_meta
  self.a._hello(); self.seq=0
  code,self.lease=self.req('POST','/robots/r/lock',self.op,{'action':'acquire'});self.assertEqual(code,200)
  self.tick(); self.tick()
 def tick(self):
  self.seq+=1
  resp=self.a.http.post('/agent/state',{'robot_id':'r','agent_boot_id':'b','seq':self.seq,'state':self.robot.snapshot()})
  with self.a.lock:
   self.a.last_state_ok_mono=time.monotonic();self.a.lease=resp.get('lease');self.a.server_safety_contract_version=resp.get('safety_contract_version')
  for cmd in resp['commands']: self.a._on_command(cmd)
  return resp
 def mission(self,key='m'):
  return self.req('POST','/missions',self.op,{'robot_id':'r','idempotency_key':key,'lease_epoch':self.lease['epoch'],'goal':{'frame_id':'map','x':1,'y':2}})
 def test_http_admission_and_fault(self):
  code,m=self.mission();self.assertEqual(code,201);resp=self.tick();self.assertEqual(len(resp['commands']),1)
  self.robot._run_tasks();self.assertEqual(len(self.robot.nav.sent_goals),1)
  h=agent_fixture.StubGoalHandle();self.robot.nav.sent_goals[0][2].set_result(h);self.a._flush_acks()
  self.assertEqual(self.f.mission(m['mission_id'])['status'],'ACCEPTED')
  self.robot._set('gate',agent_fixture._make_gate_msg(state='STALL_LATCHED',fault_epoch=2,fault_latched=True,cmd_blocked=True))
  self.tick();self.robot._run_tasks();self.assertEqual(h.cancel_calls,1)
  self.assertEqual(self.mission('fault')[0],409)
 def test_all_context_fields_parse_and_match_snapshot(self):
  self.assertEqual(self.mission()[0],201);cmd=self.tick()['commands'][0]
  from limo_robot_agent.safety_protocol import parse_safety_context, REQUIRED_SAFETY_CONTEXT_FIELDS
  context,error=parse_safety_context(cmd['payload']['safety_context']);self.assertIsNone(error)
  self.assertEqual(set(context),set(REQUIRED_SAFETY_CONTEXT_FIELDS));self.assertEqual(len(context),15)
  self.assertEqual(context['agent_boot_id'],self.a.boot_id);self.assertEqual(context['server_boot_id'],self.a.server_boot)
  self.assertEqual(context['lease_epoch'],self.lease['epoch']);self.assertEqual(context['expires_at'],cmd['expires_at'])
  self.robot._run_tasks();self.assertEqual(len(self.robot.nav.sent_goals),1)
 def test_fault_after_submission_before_goal_response(self):
  self.assertEqual(self.mission()[0],201);self.tick();self.robot._run_tasks()
  future=self.robot.nav.sent_goals[0][2]
  self.robot._set('gate',agent_fixture._make_gate_msg(state='STALL_LATCHED',fault_epoch=2,fault_latched=True,cmd_blocked=True))
  self.robot._run_tasks();self.assertTrue(self.robot.active['cancel_requested'])
  handle=agent_fixture.StubGoalHandle();future.set_result(handle);self.assertEqual(handle.cancel_calls,1)
  self.robot.cancel_result_timeout_s=0;self.robot._run_tasks()
  self.assertEqual(next(iter(self.a.seen.values()))['status'],'RECONCILIATION_REQUIRED')
  self.a._flush_acks();self.tick()
  self.assertEqual(self.mission('after-fault')[0],409)
 def test_fault_during_wait_blocks_send(self):
  self.assertEqual(self.mission()[0],201);self.tick()
  self.robot.nav.on_wait=lambda:self.robot._set('gate',agent_fixture._make_gate_msg(state='STALL_LATCHED',fault_epoch=2,fault_latched=True,cmd_blocked=True))
  self.robot._run_tasks();self.assertEqual(len(self.robot.nav.sent_goals),0)
  self.assertEqual(next(iter(self.a.seen.values()))['status'],'REJECTED')
 def test_external_goal_blocks_admission(self):
  self.robot._on_nav_status(agent_fixture._make_nav_status_msg([('fe'*16,agent_fixture.GoalStatus.STATUS_EXECUTING)]))
  self.tick();self.assertEqual(self.mission()[0],409);self.assertEqual(len(self.robot.nav.sent_goals),0)
 def test_lease_release_before_submission_blocks(self):
  self.assertEqual(self.mission()[0],201);self.tick()
  self.assertEqual(self.req('POST','/robots/r/lock',self.op,{'action':'release'})[0],200)
  self.tick();self.robot._run_tasks();self.assertEqual(len(self.robot.nav.sent_goals),0)
 def test_expiry_during_wait_blocks_send(self):
  self.assertEqual(self.mission()[0],201);resp=self.tick();expiry=resp['commands'][0]['expires_at']
  self.robot.nav.on_wait=lambda:None
  self.robot.nav.wait_for_server=lambda timeout_sec=2.0: self.expire_wait(expiry)
  self.robot._run_tasks();self.assertEqual(len(self.robot.nav.sent_goals),0)
  self.assertEqual(next(iter(self.a.seen.values()))['status'],'EXPIRED')
 def expire_wait(self,expiry):
  # Advance the wall clock only for the stage-two validation.
  import unittest.mock as mock
  original=self.robot._verify_nav_preconditions
  def check(c):
   with mock.patch('limo_robot_agent.safety_protocol.time.time',return_value=expiry+1):return original(c)
  self.robot._verify_nav_preconditions=check
  return True
 def test_cancel_result_loss_blocks_and_reset_does_not_replay(self):
  self.assertEqual(self.mission()[0],201);self.tick();self.robot._run_tasks()
  h=agent_fixture.StubGoalHandle();self.robot.nav.sent_goals[0][2].set_result(h);self.a._flush_acks()
  self.robot.cancel_result_timeout_s=0
  self.robot._set('gate',agent_fixture._make_gate_msg(state='STALL_LATCHED',fault_epoch=2,fault_latched=True,cmd_blocked=True))
  self.robot._run_tasks();self.robot._run_tasks();self.assertEqual(h.cancel_calls,1)
  self.assertEqual(next(iter(self.a.seen.values()))['status'],'RECONCILIATION_REQUIRED')
  self.a._flush_acks();self.robot._set('gate',agent_fixture._make_gate_msg(fault_epoch=2))
  self.tick();self.robot._run_tasks();self.assertEqual(len(self.robot.nav.sent_goals),1);self.assertEqual(self.mission('after-reset')[0],409)
 def test_stale_nav_observation_blocks(self):
  self.robot.nav_status['t']=time.monotonic()-3
  self.tick();self.assertEqual(self.mission()[0],409)
 def test_lease_epoch_change_during_wait_blocks_submission(self):
  self.assertEqual(self.mission()[0],201);self.tick()
  def change():
   with self.a.lock:self.a.lease={**self.a.lease,'epoch':self.a.lease['epoch']+1}
  self.robot.nav.on_wait=change;self.robot._run_tasks();self.assertEqual(len(self.robot.nav.sent_goals),0)
 def test_stale_server_session_blocks_submission(self):
  self.assertEqual(self.mission()[0],201);self.tick()
  with self.a.lock:self.a.last_state_ok_mono=time.monotonic()-3
  self.robot._run_tasks();self.assertEqual(len(self.robot.nav.sent_goals),0)
 def test_agent_restart_preserves_uncertain_journal(self):
  self.assertEqual(self.mission()[0],201);self.tick();self.robot._run_tasks()
  restarted=AgentCore('r','http://127.0.0.1:'+str(self.port),self.agent,self.robot.snapshot,self.robot,{'boot_id':'b2'},str(self.tmp.name),log=lambda x:None,require_safety_context=True)
  self.assertEqual(next(iter(restarted.seen.values()))['status'],'RECONCILIATION_REQUIRED')
  restarted._hello();self.assertEqual(len(self.robot.nav.sent_goals),1)
 def test_gate_restart_fences_pending_goal(self):
  self.assertEqual(self.mission()[0],201);self.tick()
  self.robot._set('gate',agent_fixture._make_gate_msg(boot_id='different-gate',fault_epoch=0))
  self.tick();self.robot._run_tasks();self.assertEqual(len(self.robot.nav.sent_goals),0)
 def test_server_restart_fences_delivery(self):
  self.assertEqual(self.mission()[0],201);resp=self.tick();cmd=resp['commands'][0]
  self.s.stop_evt.set();self.t.join(3)
  from fleet_server.service import Fleet
  from fleet_server.http_api import Servers
  from fleet_server.auth import Auth
  import threading
  self.f=Fleet(self.db,self.f.cfg);self.s=Servers(self.f,Auth(self.tmp.name),['127.0.0.1'],0)
  self.port=self.s.srvs[0].server_address[1];self.a.http.base='http://127.0.0.1:'+str(self.port)
  self.t=threading.Thread(target=self.s.serve_forever,daemon=True);self.t.start()
  self.a._hello();self.a._on_command(cmd);self.robot._run_tasks()
  self.assertEqual(len(self.robot.nav.sent_goals),0)
 def test_fault_in_gap_after_final_check_must_not_submit(self):
  self.assertEqual(self.mission()[0],201);self.tick()
  previous=self.robot._set_mission
  def changed(*args,**kwargs):
   previous(*args,**kwargs)
   if len(args)>2 and args[2]=='DISPATCHED':
    self.robot._set('gate',agent_fixture._make_gate_msg(state='STALL_LATCHED',fault_epoch=2,fault_latched=True,cmd_blocked=True))
  self.robot._set_mission=changed
  self.robot._run_tasks()
  self.assertEqual(len(self.robot.nav.sent_goals),0,'Gate fault between final validation and submission')
 def test_async_gate_callback_before_stage3(self):
  import threading
  self.assertEqual(self.mission()[0],201);self.tick();previous=self.robot._set_mission
  def changed(*args,**kwargs):
   previous(*args,**kwargs)
   if len(args)>2 and args[2]=='DISPATCHED':
    t=threading.Thread(target=self.robot._set,args=('gate',agent_fixture._make_gate_msg(state='STALL_LATCHED',fault_epoch=2,fault_latched=True,cmd_blocked=True)))
    t.start();t.join(2);self.assertFalse(t.is_alive())
  self.robot._set_mission=changed;self.robot._run_tasks()
  self.assertEqual(len(self.robot.nav.sent_goals),0)

if __name__=='__main__':unittest.main(verbosity=2)
