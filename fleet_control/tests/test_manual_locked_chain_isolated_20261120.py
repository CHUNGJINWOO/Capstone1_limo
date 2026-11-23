"""Isolated Fleet HTTP -> mock Agent -> selector -> Watchdog -> real Gate.

Uses existing Fleet/Gate/Agent classes and synthetic sensors and action status.
Run only on isolated localhost domain 225, without a /cmd_vel driver.
"""
import json, os, signal, subprocess, sys, tempfile, threading, time
from pathlib import Path
from types import SimpleNamespace
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from fleet_server.auth import Auth, init_tokens
from fleet_server.db import DB
from fleet_server.http_api import Servers
from fleet_server.service import Fleet
from limo_robot_agent.core import Http, HttpError

assert os.environ.get('ROS_DOMAIN_ID') == '225' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert not os.environ.get('ROS_DISCOVERY_SERVER')
here=Path(os.environ['MANUAL_CHAIN_RESULTS']).resolve();here.mkdir(parents=True,exist_ok=False)
private=Path(tempfile.mkdtemp(prefix='manual-locked-chain-'))
fleet=Fleet(DB(str(private/'fleet.db')),SimpleNamespace(offline_after_s=6,command_ttl_s=120,resend_after_s=1))
init_tokens(str(private),str(private),['test-lock'])
server=Servers(fleet,Auth(str(private)),['127.0.0.1'],0)
worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
url='http://127.0.0.1:'+str(server.srvs[0].server_port)
assert server.srvs[0].server_port not in (8080,18190,18191,18192)
operator=Http(url,(private/'operator.token').read_text().strip())
rclpy.init(); fixture=rclpy.create_node('manual_chain_fixture'); navigator=rclpy.create_node('mock_navigator');controller=rclpy.create_node('controller_server')
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
navpub=navigator.create_publisher(GoalStatusArray,'/navigate_to_pose/_action/status',qos)
followpub=controller.create_publisher(GoalStatusArray,'/follow_path/_action/status',qos)
def cancel(req,res):res.return_code=CancelGoal.Response.ERROR_NONE;return res
navigator.create_service(CancelGoal,'/navigate_to_pose/_action/cancel_goal',cancel)
messages={'/scan':LaserScan(range_min=.1,range_max=8.,ranges=[3.]*20),'/odom':Odometry(),
 '/odom_nav2':Odometry(),'/odometry/lidar_odom':Odometry(),
 '/diagnostics/f4_latency':Float32MultiArray(data=[0.,0.,0.,220.]),
 '/auto_mapping/depth_status':String(data='{"state":"ready","code":"DEPTH_READY"}')}
pubs={k:fixture.create_publisher(type(m),k,10) for k,m in messages.items()}
locked=[];status=[];watch=[]
fixture.create_subscription(Twist,'/cmd_vel_locked',lambda m:locked.append((time.monotonic(),m.linear.x,m.angular.z)),10)
fixture.create_subscription(String,'/stall_detector/status_json',lambda m:status.append(json.loads(m.data)),10)
fixture.create_subscription(String,'/fleet/motion_watchdog/status',lambda m:watch.append(json.loads(m.data)),10)
children=[]; records=[]; nav_active=[False]
def pump(seconds,sender=None,send_sensors=True):
 end=time.monotonic()+seconds; nxt=0.; frame=0.
 while time.monotonic()<end:
  now=time.monotonic()
  if sender and now>=frame:sender();frame=now+.06
  if now>=nxt:
   if send_sensors:
    for k,p in pubs.items():p.publish(messages[k])
   navmsg=GoalStatusArray()
   if nav_active[0]:
    goal=GoalStatus();goal.status=GoalStatus.STATUS_EXECUTING;goal.goal_info.goal_id.uuid=[9]*16;navmsg.status_list=[goal]
   navpub.publish(navmsg);followpub.publish(GoalStatusArray());nxt=now+.08
  for node in (fixture,navigator,controller):rclpy.spin_once(node,timeout_sec=.001)
  time.sleep(.001)
def stop_child(child):
 if child.poll() is None:
  os.killpg(child.pid,signal.SIGINT)
  try:child.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
try:
 jobs=[('gate',[sys.executable,'experiments/real_auto_mapping_ready_20261113/src/limo_capstone/limo_capstone/stall_detector_node.py','--ros-args','-r','/cmd_vel:=/cmd_vel_locked','-p','estimator_odom_topic:=/odom_nav2']),
 ('watchdog',[sys.executable,'-m','limo_robot_agent.motion_watchdog']),
 ('agent',[sys.executable,'-m','limo_robot_agent.agent_node','--ros-args','-p','mode:=mock','-p','simulated:=true','-p','manual_web_bench:=true','-p','manual_locked_arbiter:=true','-p','require_motion_watchdog:=true','-p','allow_missions:=false','-p','server_url:='+url,'-p','token_file:='+str(private/'agent-test-lock.token'),'-p','robot_id:=test-lock','-p','state_dir:='+str(private/'agent_state'),'-p','period_s:=0.10'])]
 for name,args in jobs:
  f=(here/(name+'.log')).open('x');records.append(f);children.append(subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
 pump(5)
 assert all(p.poll() is None for p in children), 'child exited, inspect logs'
 assert fixture.count_publishers('/cmd_vel') == 0 and fixture.count_subscribers('/cmd_vel') == 0
 print('watchdog',watch[-1] if watch else None,flush=True)
 assert watch and watch[-1]['state']=='MOTION_AUTHORIZED'
 lease=operator.post('/robots/test-lock/lock',{'action':'acquire','ttl_s':30}); epoch=lease['epoch']; pump(.7)
 if os.environ.get('MANUAL_CHAIN_BROWSER') == '1':
  start=time.monotonic()
  with (here/'browser.log').open('x') as browser_log:
   browser=subprocess.Popen(['node','fleet_control/tests/manual_locked_dashboard_browser_20261120.cjs',url,str(private/'operator.token')],stdout=browser_log,stderr=subprocess.STDOUT)
   deadline=time.monotonic()+18
   while browser.poll() is None and time.monotonic()<deadline:pump(.1)
   if browser.poll() is None:browser.kill();browser.wait();raise AssertionError('browser timed out')
   assert browser.returncode==0, 'browser failed; inspect browser.log'
  assert any(v>.05 and w>.05 for t,v,w in locked if t>=start), 'web W+A did not reach locked Gate'
  pump(.5)
  assert all(abs(v)<1e-6 and abs(w)<1e-6 for t,v,w in locked if t>time.monotonic()-.2)
  print('PASS browser W+A -> Fleet HTTP -> Agent -> Arbiter -> Watchdog -> locked Gate',flush=True)
 path='/robots/test-lock/manual-bench';session=operator.post(path,{'action':'start','epoch':epoch})['session_id'];seq=[0]
 def frame(v,w,pressed=True):
  def send():
   seq[0]+=1;operator.post(path,{'action':'sample', 'input_format': 'normalized_axes_v1','epoch':epoch,'session_id':session,'seq':seq[0],'linear':v,'turn':w,'deadman':pressed})
  return send
 start=time.monotonic();pump(.9,frame(1.,.5))
 print('nonzero locked',[(v,w) for t,v,w in locked if t>=start][-8:],flush=True)
 assert any(v>.05 and w>.05 for t,v,w in locked if t>=start)
 start=time.monotonic();pump(.5,frame(1.,.5,False))
 assert locked and all(abs(v)<1e-6 and abs(w)<1e-6 for t,v,w in locked if t>start+.3)
 pump(.5,frame(1.,.5));start=time.monotonic();pump(.8)
 assert all(abs(v)<1e-6 and abs(w)<1e-6 for t,v,w in locked if t>start+.5)
 # A new session produces motion again, then STOP revokes it.
 session=operator.post(path,{'action':'start','epoch':epoch})['session_id'];seq[0]=0
 start=time.monotonic();pump(.5,frame(1.,-.5))
 assert any(v>.05 and w<-.05 for t,v,w in locked if t>=start)
 operator.post(path,{'action':'stop','epoch':epoch,'session_id':session})
 start=time.monotonic();pump(.6)
 assert all(abs(v)<1e-6 and abs(w)<1e-6 for t,v,w in locked if t>start+.35)
 # Lease expiry while moving invalidates the stream and yields zero.
 session=operator.post(path,{'action':'start','epoch':epoch})['session_id'];seq[0]=0
 pump(.4,frame(1.,.5))
 with fleet.lock, fleet.db.transaction():fleet.db.x('UPDATE locks SET expires_at=0 WHERE robot_id=?',('test-lock',))
 start=time.monotonic();pump(.8)
 assert all(abs(v)<1e-6 and abs(w)<1e-6 for t,v,w in locked if t>start+.5)
 try:frame(1.,0.)()
 except HttpError as e:assert e.status==409
 else:raise AssertionError('expired lease accepted input')
 epoch=operator.post('/robots/test-lock/lock',{'action':'acquire','ttl_s':30})['epoch'];pump(.5)
 # Fault injection at the existing Mapping guard: no new manual session.
 original_mapping=fleet.mapping_blocks_motion;fleet.mapping_blocks_motion=lambda rid:True
 try:
  try:operator.post(path,{'action':'start','epoch':epoch})
  except HttpError as e:assert e.status==409
  else:raise AssertionError('Mapping did not reject MANUAL')
 finally:fleet.mapping_blocks_motion=original_mapping
 session=operator.post(path,{'action':'start','epoch':epoch})['session_id'];seq[0]=0
 pump(.4,frame(1.,.5))
 if os.environ.get('MANUAL_CHAIN_FAULT') == 'active_goal':
  nav_active[0]=True;start=time.monotonic();pump(.8)
  assert all(abs(v)<1e-6 and abs(w)<1e-6 for t,v,w in locked if t>start+.5)
  try:operator.post(path,{'action':'start','epoch':epoch})
  except HttpError as e:assert e.status==409
  else:raise AssertionError('active Nav2 did not reject MANUAL')
  nav_active[0]=False;pump(2.)
  # Empty status is not a verified terminal result: local selector remains locked.
  with fleet.lock:
   r=(fleet.robots.get('test-lock',{}).get('state') or {}).get('velocity_arbiter') or {}
  assert r.get('ready') is False
  try:operator.post(path,{'action':'start','epoch':epoch})
  except HttpError as e:assert e.status==409
  else:raise AssertionError('empty Nav2 status rearmed manual')
  print('PASS active Nav2 stops output and empty status does not rearm selector',flush=True)
 elif os.environ.get('MANUAL_CHAIN_FAULT') == 'agent_restart':
  children[2].kill();children[2].wait(timeout=4);start=time.monotonic();pump(1.)
  assert all(abs(v)<1e-6 and abs(w)<1e-6 for t,v,w in locked if t>start+.55)
  f=(here/'agent_restart.log').open('x');records.append(f)
  children.append(subprocess.Popen(jobs[2][1],stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
  pump(4.)
  try:frame(1.,.5)()
  except HttpError as e:assert e.status==409
  else:raise AssertionError('old session reused after Agent restart')
  print('PASS Agent SIGKILL, locked zero, restart rejects previous session',flush=True)
 else:
  # Gate must stop if its independent Watchdog dies during nonzero input.
  stop_child(children[1]);start=time.monotonic();pump(1.)
  assert all(abs(v)<1e-6 and abs(w)<1e-6 for t,v,w in locked if t>start+.55)
  print('PASS independent Watchdog termination yields locked zero',flush=True)
 assert fixture.count_publishers('/cmd_vel') == 0 and fixture.count_subscribers('/cmd_vel') == 0
 print('PASS HTTP -> Agent -> Arbiter -> Watchdog -> locked Gate; deadman, timeout, STOP, lease expiry and Mapping',flush=True)
finally:
 for p in reversed(children):stop_child(p)
 for f in records:f.close()
 server.stop_evt.set();worker.join(timeout=4)
 for n in (fixture,navigator,controller):n.destroy_node()
 rclpy.try_shutdown()
