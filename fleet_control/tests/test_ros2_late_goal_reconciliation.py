"""Actual ROS action timeout/late acceptance; isolated test fixtures only."""
import json, os, socket, sys, tempfile, threading, time, unittest
from pathlib import Path
from types import SimpleNamespace
if os.environ.get('CAPSTONE_PRIVATE_ROS_TEST')!='1' or {n for _,n in socket.if_nameindex()}!={'lo'}:
 raise SystemExit('refusing: private network namespace required')
sys.path[:0]=[str(Path(__file__).resolve().parents[1]/'server'),str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent')]
import rclpy
from rclpy.action import ActionServer,CancelResponse,GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from fleet_server.db import DB
from fleet_server.service import Fleet
from fleet_server.auth import Auth,init_tokens
from fleet_server.http_api import Servers
from limo_robot_agent.core import AgentCore
from limo_robot_agent.ros_adapter import RosRobot

def wait(fn,timeout=5):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  if fn():return
  time.sleep(.01)
 raise AssertionError('timeout waiting for condition')

class Tests(unittest.TestCase):
 def test_late_accept_and_terminal(self):
  rclpy.init();tmp=tempfile.TemporaryDirectory();db=None;server=None;executor=None
  release=threading.Event();closing=threading.Event();trace={};handles=[]
  try:
   root=Path(tmp.name);init_tokens(str(root),str(root/'sec'),['r'])
   token=(root/'sec/agent-r.token').read_text().strip();op=(root/'sec/operator.token').read_text().strip()
   db=DB(str(root/'db'));fleet=Fleet(db,SimpleNamespace(offline_after_s=10,command_ttl_s=120,resend_after_s=.1))
   server=Servers(fleet,Auth(str(root)),['127.0.0.1'],0);port=server.srvs[0].server_address[1]
   st=threading.Thread(target=server.serve_forever,daemon=True);st.start()
   node=Node('isolated_late_action');group=ReentrantCallbackGroup()
   def goal(_):trace['goal_received']=time.monotonic();time.sleep(.7);return GoalResponse.ACCEPT
   def cancel(_):trace['cancel_requested']=time.monotonic();return CancelResponse.ACCEPT
   def execute(handle):
    handles.append(handle)
    while not handle.is_cancel_requested and not closing.is_set():time.sleep(.01)
    if handle.is_cancel_requested:
     while not release.is_set() and not closing.is_set():time.sleep(.01)
     if release.is_set():handle.canceled();trace['terminal_sent']=time.monotonic()
     else:handle.abort()
    else:handle.abort()
    return NavigateToPose.Result()
   action=ActionServer(node,NavigateToPose,'navigate_to_pose',goal_callback=goal,cancel_callback=cancel,execute_callback=execute,callback_group=group)
   gate=node.create_publisher(String,'/stall_detector/status_json',10)
   idle=node.create_publisher(GoalStatusArray,'navigate_to_pose/_action/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
   robot=RosRobot(allow_missions=True,mode='sim',simulated=True,robot_id='r',agent_boot_id='b',goal_response_timeout_s=.2,cancel_result_timeout_s=2.)
   robot._motor_path_status=lambda:('LOCKED',{})
   original_cancel=robot._request_cancel
   def traced_cancel(rec):
    handle=rec['handle'];original=handle.cancel_goal_async
    def tracked():
     future=original()
     def observed(f):
      result=f.result();trace['cancel_response_code']=result.return_code
      trace['cancel_response_uuid_match']=any(bytes(g.goal_id.uuid)==bytes(handle.goal_id.uuid) for g in result.goals_canceling)
     future.add_done_callback(observed);return future
    handle.cancel_goal_async=tracked
    return original_cancel(rec)
   robot._request_cancel=traced_cancel
   robot.context_fn=lambda:{'operation':{'operation_mode':'NAVIGATION','operation_revision':1},'map':{'map_verified':True,'map_version_id':'m','map_epoch':1},'localization':{'localization_status':'CONFIRMED','localization_map_version_id':'m','localization_epoch':1}}
   agent=AgentCore('r',f'http://127.0.0.1:{port}',token,robot.snapshot,robot,{'boot_id':'b'},str(root),require_safety_context=True,log=lambda _:None)
   robot.session_fn=agent.session_snapshot;robot.execution_allowed=agent.is_execution_allowed;robot.on_command_meta=agent.record_command_meta
   executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node);executor.add_node(robot);thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
   wait(lambda:gate.get_subscription_count()>0 and idle.get_subscription_count()>0)
   def pub():gate.publish(String(data=json.dumps({'state':'NORMAL','boot_id':'synthetic-gate','fault_epoch':1,'fault_latched':False,'cmd_blocked':False})))
   pub();idle.publish(GoalStatusArray());wait(lambda:robot.gate is not None and robot.nav_status is not None);agent._hello()
   import urllib.request,urllib.error
   def request(path,body):
    req=urllib.request.Request(f'http://127.0.0.1:{port}/api/v1'+path,data=json.dumps(body).encode(),headers={'Authorization':'Bearer '+op,'Content-Type':'application/json'},method='POST')
    with urllib.request.urlopen(req) as response:return json.load(response)
   lease=request('/robots/r/lock',{'action':'acquire'})
   seq=0
   def tick():
    nonlocal seq;seq+=1
    resp=agent.http.post('/agent/state',{'robot_id':'r','agent_boot_id':'b','seq':seq,'state':robot.snapshot()})
    with agent.lock:agent.last_state_ok_mono=time.monotonic();agent.lease=resp['lease'];agent.server_safety_contract_version=resp['safety_contract_version']
    for c in resp['commands']:agent._on_command(c)
    return resp
   tick();pub();idle.publish(GoalStatusArray());
   # G2 2026-11-23 fixture: v5 admits a goal only with ONE navigate_to_pose status publisher (origin/main rule); the synthetic
   # idle publisher that seeded the status is removed so the action server is the unique one again (assertions unchanged).
   node.destroy_publisher(idle);wait(lambda:robot.count_publishers('navigate_to_pose/_action/status')==1)
   time.sleep(.1);tick()
   body={'robot_id':'r','lease_epoch':lease['epoch'],'goal':{'frame_id':'map','x':1,'y':2}}
   mid=request('/missions',body)['mission_id'];cmd=tick()['commands'][0]
   wait(lambda:agent.seen.get(cmd['command_id'],{}).get('status')=='RECONCILIATION_REQUIRED')
   agent._flush_acks();self.assertEqual(fleet.mission(mid)['status'],'RECONCILIATION_REQUIRED')
   wait(lambda:'cancel_requested' in trace);self.assertEqual(len(handles),1)
   wait(lambda:'cancel_response_code' in trace)
   self.assertEqual(trace['cancel_response_code'],0);self.assertTrue(trace['cancel_response_uuid_match'])
   self.assertEqual(agent.seen[cmd['command_id']]['status'],'RECONCILIATION_REQUIRED')
   self.assertEqual(fleet.mission(mid)['status'],'RECONCILIATION_REQUIRED')
   with robot.lock:rec=robot.active;owned_uuid=rec['goal_uuid'];owned_handle=rec['handle']
   fake_future=SimpleNamespace(result=lambda:owned_handle)
   robot._on_goal_response(rec,fake_future)  # test-only duplicate callback
   wrong=SimpleNamespace(result=lambda:SimpleNamespace(status=4,goal_id=SimpleNamespace(uuid=bytes.fromhex('ff'*16))))
   robot._on_result(rec,wrong,expected_goal_uuid='ff'*16)  # test-only UUID mismatch
   self.assertEqual(agent.seen[cmd['command_id']]['status'],'RECONCILIATION_REQUIRED')
   self.assertEqual(rec['goal_uuid'],owned_uuid);self.assertIs(robot.active,rec)
   with self.assertRaises(urllib.error.HTTPError) as denied:request('/missions',body)
   self.assertEqual(denied.exception.code,409)
   release.set();wait(lambda:agent.seen[cmd['command_id']]['status']=='CANCELED');agent._flush_acks()
   self.assertEqual(fleet.mission(mid)['status'],'CANCELED')
   print('RESULT',json.dumps({'journal_before_terminal':'RECONCILIATION_REQUIRED','server_before_terminal':'RECONCILIATION_REQUIRED','duplicate_callback':'ignored','wrong_uuid_result':'ignored','agent_terminal':'CANCELED','server_terminal':fleet.mission(mid)['status'],'goals':len(handles),**trace}),flush=True)
  finally:
   closing.set();release.set()
   if executor:executor.shutdown(timeout_sec=3)
   if server:server.stop_evt.set();st.join(3)
   if db:db.c.close()
   rclpy.shutdown();tmp.cleanup()
if __name__=='__main__':unittest.main(verbosity=2)
