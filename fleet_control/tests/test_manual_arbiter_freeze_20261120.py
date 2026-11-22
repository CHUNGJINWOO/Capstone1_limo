"""Isolated public Arbiter SIGSTOP against real Gate code; never use a robot.

The smoother is a synthetic pass-through, HTTP and Nav2 evidence are synthetic.
A paused child is killed without resuming. /cmd_vel has no endpoints.
"""
import json, os, signal, subprocess, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from types import SimpleNamespace
from limo_robot_agent.manual_web_bench import ManualWebBench
from limo_robot_agent.velocity_arbiter import VelocityArbiter

assert os.environ.get('ROS_DOMAIN_ID')=='213' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
assert not os.environ.get('ROS_DISCOVERY_SERVER')

class Robot(Node):
 def __init__(self):
  super().__init__('fleet_robot_agent');self.gate=None;self.gate_time=0.;self.active=None
  self.create_subscription(String,'/stall_detector/status_json',self.cb,10)
 def cb(self,m):self.gate=json.loads(m.data);self.gate_time=time.monotonic()
 def _age(self,_):return time.monotonic()-self.gate_time if self.gate_time else None
 def _nav2_goals(self):return {'publishers':1,'active_count':0,'received_status_array':True}
 def follow_path_idle(self):return True
 def frontier_allowed(self,_):return False
class Http:
 def post(self,*_,**__):return {'sample':{'linear':.08,'turn':.18,'deadman':True,'session_id':'test','seq':1,'ttl_s':.20}}

def child():
 rclpy.init();robot=Robot();agent=SimpleNamespace(connected=True,http=Http(),robot_id='test',boot_id='test')
 bench=ManualWebBench(robot,agent);arbiter=VelocityArbiter(robot,agent,bench)
 try:
  while rclpy.ok():rclpy.spin_once(robot,timeout_sec=.05)
 finally:bench.close();robot.destroy_node();rclpy.try_shutdown()

def main():
 root=os.environ['MANUAL_ARBITER_REPO'];out=os.environ['MANUAL_FREEZE_RESULTS'];os.makedirs(out,exist_ok=True)
 gate_src=os.path.join(root,'experiments/real_auto_mapping_ready_20261113/src/limo_capstone/limo_capstone/stall_detector_node.py')
 gate_log=open(os.path.join(out,'gate.log'),'w');child_log=open(os.path.join(out,'agent.log'),'w')
 gate=subprocess.Popen([sys.executable,gate_src,'--ros-args','-r','/cmd_vel:=/cmd_vel_locked','-p','estimator_odom_topic:=/odom_nav2'],stdout=gate_log,stderr=subprocess.STDOUT,start_new_session=True)
 rclpy.init();fixture=Node('freeze_sensor_fixture');smoother=Node('velocity_smoother');controller=Node('controller_server');state=[];locked=[]
 raw=smoother.create_publisher(Twist,'/cmd_vel_raw',10)
 smoother.create_subscription(Twist,'/fleet/cmd_vel_selected',lambda m:raw.publish(m),10)
 nav=controller.create_publisher(Twist,'/fleet/cmd_vel_nav',10)
 fixture.create_subscription(String,'/stall_detector/status_json',lambda m:state.append((time.monotonic(),json.loads(m.data))),10)
 messages={'/scan':LaserScan(range_min=.1,range_max=8.,ranges=[3.]*20),'/odom':Odometry(),'/odom_nav2':Odometry(),'/odometry/lidar_odom':Odometry(),'/diagnostics/f4_latency':Float32MultiArray(data=[0.,0.,0.,220.]),'/auto_mapping/depth_status':String(data='{"state":"ready","code":"DEPTH_READY"}')}
 pubs={topic:fixture.create_publisher(type(msg),topic,10) for topic,msg in messages.items()}
 agent=subprocess.Popen([sys.executable,__file__,'--child'],stdout=child_log,stderr=subprocess.STDOUT,start_new_session=True)
 nodes=(fixture,smoother,controller);next_sensor=0.
 def spin():
  nonlocal next_sensor
  if time.monotonic()>=next_sensor:
   for t,p in pubs.items():p.publish(messages[t])
   next_sensor=time.monotonic()+.025
  for n in nodes:rclpy.spin_once(n,timeout_sec=.002)
 try:
  deadline=time.monotonic()+12
  while time.monotonic()<deadline:
   assert gate.poll() is None and agent.poll() is None
   spin()
   if state and state[-1][1].get('state')=='NORMAL' and state[-1][1].get('cmd_vx_out',0)>.05:break
  else:raise AssertionError('nonzero manual command did not reach locked Gate')
  assert fixture.count_publishers('/cmd_vel')==0 and fixture.count_subscribers('/cmd_vel')==0
  assert fixture.count_publishers('/cmd_vel_locked')==1 and fixture.count_subscribers('/cmd_vel_locked')==0
  os.killpg(agent.pid,signal.SIGSTOP);t0=time.monotonic()
  deadline=t0+2
  while time.monotonic()<deadline:
   spin()
   if state and state[-1][0]>t0 and state[-1][1].get('cmd_vx_out')==0:break
  else:raise AssertionError('Gate did not time out after Agent freeze')
  elapsed=state[-1][0]-t0
  fixture.create_subscription(Twist,'/cmd_vel_locked',lambda m:locked.append((m.linear.x,m.angular.z)),10)
  end=time.monotonic()+.45
  while time.monotonic()<end:spin()
  assert locked and all(v==(0.,0.) for v in locked)
  assert fixture.count_publishers('/cmd_vel')==0 and fixture.count_subscribers('/cmd_vel')==0
  print(f'PASS Gate zero after Agent SIGSTOP: {elapsed:.3f}s; locked messages {len(locked)} all zero; /cmd_vel endpoints 0',flush=True)
 finally:
  if agent.poll() is None:os.killpg(agent.pid,signal.SIGKILL);agent.wait(timeout=5)
  if gate.poll() is None:
   os.killpg(gate.pid,signal.SIGINT)
   try:gate.wait(timeout=7)
   except subprocess.TimeoutExpired:os.killpg(gate.pid,signal.SIGKILL);gate.wait()
  for n in reversed(nodes):n.destroy_node()
  rclpy.shutdown();gate_log.close();child_log.close()
if __name__=='__main__':child() if '--child' in sys.argv else main()
