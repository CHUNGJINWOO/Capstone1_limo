"""Actual rclpy test: run ONLY inside isolated net+mount namespaces, no robot.

The test supplies synthetic Gate, localization and idle-status evidence. It does
not certify an actual Nav2 stack or physical Gate.
"""
import json, os, socket, sys, tempfile, threading, time, unittest
from pathlib import Path
from types import SimpleNamespace

if os.environ.get('CAPSTONE_PRIVATE_ROS_TEST') != '1' or {name for _,name in socket.if_nameindex()} != {'lo'}:
 raise SystemExit('refusing: private network namespace required')
sys.path[:0]=[str(Path(__file__).resolve().parents[1]/'server'),str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent')]
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from fleet_server.db import DB
from fleet_server.service import Fleet
from fleet_server.auth import Auth, init_tokens
from fleet_server.http_api import Servers
from limo_robot_agent.core import AgentCore
from limo_robot_agent.ros_adapter import RosRobot

def wait(fn,timeout=5):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if fn():return
  time.sleep(.02)
 raise AssertionError('timeout waiting for condition')

class Tests(unittest.TestCase):
 def test_action_fault_cancel(self):
  rclpy.init();tmp=tempfile.TemporaryDirectory();db=None;server=None;executor=None
  stop=threading.Event(); accepted=[]; canceled=[];times={};mode=os.environ.get('CAPSTONE_TEST_CASE','normal')
  try:
   root=Path(tmp.name);init_tokens(str(root),str(root/'sec'),['r'])
   token=(root/'sec/agent-r.token').read_text().strip();op=(root/'sec/operator.token').read_text().strip()
   db=DB(str(root/'db'));fleet=Fleet(db,SimpleNamespace(offline_after_s=10,command_ttl_s=120,resend_after_s=.1))
   server=Servers(fleet,Auth(str(root)),['127.0.0.1'],0);port=server.srvs[0].server_address[1]
   http_thread=threading.Thread(target=server.serve_forever,daemon=True);http_thread.start()
   mock=Node('capstone_isolated_mock');group=ReentrantCallbackGroup()
   def execute(handle):
    accepted.append(True);times['goal_execute']=time.monotonic()
    while not stop.is_set() and not handle.is_cancel_requested:time.sleep(.02)
    if handle.is_cancel_requested:
     if mode in ('lost_terminal','delayed_timeout','epoch_pending_lost','boot_replay_lost'):
      while not stop.is_set():time.sleep(.02)
     handle.canceled();canceled.append(True);times['terminal']=time.monotonic()
    else:handle.abort()
    return NavigateToPose.Result()
   def cancel(handle):
    times['cancel_request']=time.monotonic();times['cancel_uuid']=bytes(handle.goal_id.uuid).hex();return CancelResponse.ACCEPT
   def goal(_):
    times['goal_received']=time.monotonic()
    if mode=='delayed_timeout':time.sleep(.7)
    return GoalResponse.ACCEPT
   action=ActionServer(mock,NavigateToPose,'navigate_to_pose',execute_callback=execute,goal_callback=goal,cancel_callback=cancel,callback_group=group)
   gate=mock.create_publisher(String,'/stall_detector/status_json',10)
   qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
   idle=mock.create_publisher(GoalStatusArray,'navigate_to_pose/_action/status',qos)
   robot=RosRobot(allow_missions=True,mode='sim',simulated=True,robot_id='r',agent_boot_id='b')
   original_cancel=robot._request_cancel
   def traced_cancel(rec):
    handle=rec['handle'];original=handle.cancel_goal_async
    def tracked():
     future=original()
     def observed_cancel(f):
      result=f.result();times['cancel_ack_code']=result.return_code
      times['cancel_ack_uuid_matches']=any(bytes(g.goal_id.uuid)==bytes(handle.goal_id.uuid) for g in result.goals_canceling)
     future.add_done_callback(observed_cancel);return future
    handle.cancel_goal_async=tracked
    return original_cancel(rec)
   if mode in ('epoch_pending_lost','boot_replay_lost'):robot._request_cancel=traced_cancel
   # TEST ONLY: mocked motor path; no actual /cmd_vel publisher is created.
   robot._motor_path_status=lambda:('LOCKED',{})
   robot.cancel_result_timeout_s=.4
   if mode=='delayed_timeout':robot.goal_response_timeout_s=.2;robot.cancel_result_timeout_s=2.
   original_set=robot._set
   def observed(key,msg):
    original_set(key,msg)
    if key=='gate' and (json.loads(msg.data).get('state')=='STALL_LATCHED' or json.loads(msg.data).get('fault_epoch_pending')==1):times['fault_callback']=time.monotonic()
   robot._set=observed
   # Subscription's lambda resolves robot._set dynamically, so it exercises the actual callback.
   send=robot.nav.send_goal_async
   def submit(*a,**kw):
    if mode=='pending_fault':
     publish('STALL_LATCHED',2)
     time.sleep(.05)  # Gate callback is queued while submission holds the Agent lock.
    times['goal_submit']=time.monotonic();return send(*a,**kw)
   robot.nav.send_goal_async=submit
   robot.context_fn=lambda:{'operation':{'operation_mode':'NAVIGATION','operation_revision':1},'map':{'map_verified':True,'map_version_id':'m','map_epoch':1},'localization':{'localization_status':'CONFIRMED','localization_map_version_id':'m','localization_epoch':1}}
   agent=AgentCore('r',f'http://127.0.0.1:{port}',token,robot.snapshot,robot,{'boot_id':'b'},str(root),require_safety_context=True,log=lambda _:None)
   robot.session_fn=agent.session_snapshot;robot.execution_allowed=agent.is_execution_allowed;robot.on_command_meta=agent.record_command_meta
   executor=MultiThreadedExecutor(num_threads=4);executor.add_node(mock);executor.add_node(robot)
   thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
   wait(lambda:gate.get_subscription_count()>0 and idle.get_subscription_count()>0)
   def publish(status='NORMAL',epoch=1,pending=None,boot='synthetic-gate'):
    if status!='NORMAL' or pending==1:times['fault_published']=time.monotonic()
    msg={'state':status,'boot_id':boot,'fault_epoch':epoch,'fault_latched':status!='NORMAL','cmd_blocked':status!='NORMAL'}
    if pending is not None:msg['fault_epoch_pending']=pending
    gate.publish(String(data=json.dumps(msg)))
   publish();idle.publish(GoalStatusArray());wait(lambda:robot.gate is not None and robot.nav_status is not None)
   agent._hello()
   import urllib.request
   req=urllib.request.Request(f'http://127.0.0.1:{port}/api/v1/robots/r/lock',data=json.dumps({'action':'acquire'}).encode(),headers={'Authorization':'Bearer '+op,'Content-Type':'application/json'},method='POST')
   with urllib.request.urlopen(req) as response:lease=json.load(response)
   def tick(seq):
    resp=agent.http.post('/agent/state',{'robot_id':'r','agent_boot_id':'b','seq':seq,'state':robot.snapshot()})
    with agent.lock:agent.last_state_ok_mono=time.monotonic();agent.lease=resp['lease'];agent.server_safety_contract_version=resp['safety_contract_version']
    for c in resp['commands']:agent._on_command(c)
    return resp
   tick(1);publish();idle.publish(GoalStatusArray());
   # G2 2026-11-23 fixture: v5 admits a goal only with ONE navigate_to_pose status publisher (origin/main rule); the synthetic
   # idle publisher that seeded the status is removed so the action server is the unique one again (assertions unchanged).
   mock.destroy_publisher(idle);wait(lambda:robot.count_publishers('navigate_to_pose/_action/status')==1)
   time.sleep(.1);tick(2)
   req=urllib.request.Request(f'http://127.0.0.1:{port}/api/v1/missions',data=json.dumps({'robot_id':'r','lease_epoch':lease['epoch'],'goal':{'frame_id':'map','x':1,'y':2}}).encode(),headers={'Authorization':'Bearer '+op,'Content-Type':'application/json'},method='POST')
   with urllib.request.urlopen(req) as response:self.assertEqual(response.status,201)
   if mode=='pre_stage3':
    queued=[];robot.start_navigate=lambda cmd,ack:queued.append((cmd,ack))
    orig_mission=robot._set_mission
    def mission(*args,**kwargs):
     orig_mission(*args,**kwargs)
     if len(args)>2 and args[2]=='DISPATCHED':publish('STALL_LATCHED',2);wait(lambda:'fault_callback' in times)
    robot._set_mission=mission
    tick(3);self.assertEqual(len(queued),1);robot._do_nav(*queued[0])
    self.assertNotIn('goal_submit',times);self.assertEqual(accepted,[])
    self.assertEqual(next(iter(agent.seen.values()))['status'],'REJECTED')
    print('TIMING',json.dumps({'case':mode,**times}),flush=True);return
   tick(3)
   if mode=='delayed_timeout':
    wait(lambda:any(v.get('status')=='RECONCILIATION_REQUIRED' for v in agent.seen.values()))
    times['reconciliation_reported']=time.monotonic()
    wait(lambda:'cancel_request' in times)
    current=next(iter(agent.seen.values()))['status']
    print('TIMING',json.dumps({'case':mode,'journal_status_after_late_accept':current,**times}),flush=True)
    self.assertEqual(current,'RECONCILIATION_REQUIRED','late acceptance must not clear unknown outcome without terminal result')
    return
   wait(lambda:bool(accepted))
   wait(lambda:robot.active is not None and robot.active.get('goal_uuid') is not None)
   with robot.lock: submitted_uuid=robot.active['goal_uuid']
   with agent.lock:
    entry=next(iter(agent.seen.values()))
    self.assertEqual(entry.get('goal_uuid'),submitted_uuid)
    self.assertEqual(entry.get('submission_uncertainty'),'MAY_HAVE_SENT')
   if mode in ('epoch_pending_lost','boot_replay_lost'):
    wait(lambda:robot.active is not None and robot.active.get('goal_uuid') is not None)
    with robot.lock:owned_uuid=robot.active['goal_uuid']
    if mode=='epoch_pending_lost':publish(pending=1)
    else:
     publish(boot='synthetic-gate-B')
     wait(lambda:robot.gate_tracker.evaluate()['boot_changed'])
     publish(boot='synthetic-gate')
   elif mode!='pending_fault':publish('STALL_LATCHED',2)
   if mode in ('lost_terminal','epoch_pending_lost','boot_replay_lost'):
    wait(lambda:'cancel_request' in times)
    if mode in ('epoch_pending_lost','boot_replay_lost'):
     self.assertEqual(times['cancel_uuid'],owned_uuid)
     wait(lambda:'cancel_ack_code' in times)
     self.assertEqual(times['cancel_ack_code'],0);self.assertTrue(times['cancel_ack_uuid_matches'])
    wait(lambda:any(v.get('status')=='RECONCILIATION_REQUIRED' for v in agent.seen.values()))
    self.assertEqual(len(accepted),1);self.assertNotIn('terminal',times)
    if mode in ('epoch_pending_lost','boot_replay_lost'):
     with robot.lock:rec=robot.active
     stale=dict(rec,submission_agent_boot_id='previous-agent-boot')
     fake=SimpleNamespace(result=lambda:SimpleNamespace(status=5))
     robot._on_result(stale,fake,expected_goal_uuid=owned_uuid)
     self.assertIs(robot.active,rec)
     self.assertEqual(next(iter(agent.seen.values()))['status'],'RECONCILIATION_REQUIRED')
    agent._flush_acks();tick(4)
    if mode in ('epoch_pending_lost','boot_replay_lost'):
     record=next(iter(agent.seen.values()))
     self.assertEqual(record['status'],'RECONCILIATION_REQUIRED')
     self.assertEqual(fleet.mission(record['submission_mission_id'])['status'],'RECONCILIATION_REQUIRED')
    import urllib.error
    request=urllib.request.Request(f'http://127.0.0.1:{port}/api/v1/missions',data=json.dumps({'robot_id':'r','lease_epoch':lease['epoch'],'goal':{'frame_id':'map','x':3,'y':2}}).encode(),headers={'Authorization':'Bearer '+op,'Content-Type':'application/json'},method='POST')
    with self.assertRaises(urllib.error.HTTPError) as blocked:urllib.request.urlopen(request)
    self.assertEqual(blocked.exception.code,409);self.assertEqual(len(accepted),1)
    print('TIMING',json.dumps({'case':mode,**times}),flush=True);return
   wait(lambda:bool(canceled));wait(lambda:any(v.get('status')=='CANCELED' for v in agent.seen.values()))
   if mode=='pending_fault':self.assertLess(times['fault_published'],times['goal_submit']);self.assertGreater(times['fault_callback'],times['goal_submit'])
   self.assertEqual(len(accepted),1)
   agent._flush_acks();tick(4)
   import urllib.error
   req=urllib.request.Request(f'http://127.0.0.1:{port}/api/v1/missions',data=json.dumps({'robot_id':'r','lease_epoch':lease['epoch'],'goal':{'frame_id':'map','x':3,'y':2}}).encode(),headers={'Authorization':'Bearer '+op,'Content-Type':'application/json'},method='POST')
   with self.assertRaises(urllib.error.HTTPError) as rejected:urllib.request.urlopen(req)
   self.assertEqual(rejected.exception.code,409);self.assertEqual(len(accepted),1)
   print('TIMING',json.dumps({'case':mode,**times}),flush=True)
  finally:
   stop.set()
   if executor:executor.shutdown(timeout_sec=3)
   if server:server.stop_evt.set();http_thread.join(3)
   if db:db.c.close()
   rclpy.shutdown();tmp.cleanup()

if __name__=='__main__':unittest.main(verbosity=2)
