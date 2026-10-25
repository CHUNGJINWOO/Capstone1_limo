import os,time,math,sys,signal,subprocess,json,pathlib,hashlib,re
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import Image as ImageMsg,LaserScan
from nav_msgs.msg import Odometry,Path,OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped,Twist
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from lifecycle_msgs.srv import GetState
import tf2_ros
ROOT=pathlib.Path(__file__).resolve().parents[4];TMP=pathlib.Path(__file__).resolve().parent
case=sys.argv[1] if len(sys.argv)>1 else 'baseline';OUT=ROOT/'output/simulation/2026-10-24'/case;OUT.mkdir(parents=True,exist_ok=True);(OUT/'frames').mkdir(exist_ok=True)
children=[];logs=[]
def launch(args,name):
 f=(OUT/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,cwd=ROOT);children.append(p);return p
class Recorder(Node):
 def __init__(self):
  super().__init__('simulation_video_recorder',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);self.sim=0.;self.scan=None;self.odom=None;self.amcl=None;self.plan=[];self.cost=None;self.image_count=0;self.last_image_sim=-100.;self.recording=False;self.status='준비';self.frames=[];self.cmd=[0,0];self.truth=None
  self.create_subscription(Clock,'/clock',lambda m:setattr(self,'sim',m.clock.sec+m.clock.nanosec/1e9),10)
  self.create_subscription(LaserScan,'/scan',lambda m:setattr(self,'scan',m),qos_profile_sensor_data)
  self.create_subscription(Odometry,'/odom',self.odom_cb,10)
  self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:setattr(self,'amcl',m),10)
  self.create_subscription(Path,'/plan',lambda m:setattr(self,'plan',[[p.pose.position.x,p.pose.position.y] for p in m.poses]),10)
  self.create_subscription(OccupancyGrid,'/global_costmap/costmap',lambda m:setattr(self,'cost',m),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(Twist,'/cmd_vel',lambda m:setattr(self,'cmd',[m.linear.x,m.angular.z]),10)
  self.create_subscription(ImageMsg,'/demo/overhead',self.image_cb,qos_profile_sensor_data)
  self.create_subscription(TFMessage,'/world/mc3f_world/pose/info',self.truth_cb,10)
  self.pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10);self.client=ActionClient(self,NavigateToPose,'/navigate_to_pose');self.tfbuf=tf2_ros.Buffer();self.tfl=tf2_ros.TransformListener(self.tfbuf,self);self.tele=(OUT/'telemetry.jsonl').open('w');self.feedback=None
 def odom_cb(self,m):self.odom=m
 def truth_cb(self,m):
  for t in m.transforms:
   if t.child_frame_id=='limo':self.truth=[t.transform.translation.x,t.transform.translation.y,t.transform.rotation.z,t.transform.rotation.w]
 def image_cb(self,m):
  self.image_count+=1
  if not self.recording:return
  t=m.header.stamp.sec+m.header.stamp.nanosec/1e9
  if t-self.last_image_sim<.095:return
  self.last_image_sim=t
  enc=m.encoding.lower();mode='RGB' if enc in ['rgb8','8uc3'] else 'RGBA' if enc=='rgba8' else 'RGB'
  im=Image.frombytes(mode,(m.width,m.height),bytes(m.data),'raw','BGR' if enc=='bgr8' else mode,m.step,1).convert('RGB');file=f'frames/{len(self.frames):05}.png';im.save(OUT/file)
  pose=None
  try:
   tr=self.tfbuf.lookup_transform('map','base_link',rclpy.time.Time());q=tr.transform.rotation;pose=[tr.transform.translation.x,tr.transform.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
  except Exception:pass
  od=None
  if self.odom:
   p=self.odom.pose.pose.position;od=[p.x,p.y]
  cost=None
  if self.cost:cost={'width':self.cost.info.width,'height':self.cost.info.height,'resolution':self.cost.info.resolution,'origin':[self.cost.info.origin.position.x,self.cost.info.origin.position.y],'data':list(self.cost.data)}
  rec={'image':file,'sim_time':t,'pose':pose,'truth':self.truth,'odom':od,'plan':self.plan,'cmd':self.cmd,'status':self.status,'costmap':cost,'distance_remaining':self.feedback.distance_remaining if self.feedback else None}
  self.frames.append({'image':file,'sim_time':t});self.tele.write(json.dumps(rec,ensure_ascii=False)+'\n');self.tele.flush()
 def seed(self):
  m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.pose.pose.position.x=-1.5;m.pose.pose.orientation.w=1.;m.pose.covariance[0]=.04;m.pose.covariance[7]=.04;m.pose.covariance[35]=.03;self.pub.publish(m)
def spin_for(node,dur,seed=False):
 end=time.monotonic()+dur
 while time.monotonic()<end:
  if seed:node.seed()
  rclpy.spin_once(node,timeout_sec=.1)
try:
 rclpy.init();node=Recorder()
 launch(['ros2','launch','limo_capstone','limo_mc3f_sim.launch.py','world:='+str(TMP/('room_camera.sdf' if case=='baseline' else 'room_obstacle.sdf')),'model_file:='+str(TMP/'limo_contact_check.sdf'),'headless:=true','rviz:=false','use_sim_time:=true','spawn_x:=-1.5','spawn_y:=0.0','spawn_yaw:=0.0'],'gazebo')
 launch(['ros2','run','ros_gz_bridge','parameter_bridge','/demo/overhead@sensor_msgs/msg/Image[ignition.msgs.Image','/world/mc3f_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],'video_bridge')
 end=time.monotonic()+45
 while time.monotonic()<end and not(node.scan and node.odom and node.image_count):rclpy.spin_once(node,timeout_sec=.1)
 if not(node.scan and node.odom and node.image_count):raise RuntimeError(f'INPUT_TIMEOUT scan={bool(node.scan)} odom={bool(node.odom)} camera_frames={node.image_count}')
 print('INPUTS_READY',node.sim,'camera_frames',node.image_count,flush=True)
 launch(['ros2','launch','limo_capstone','limo_nav2_sim.launch.py','map:='+str(TMP/'room_map.yaml'),'use_sim_time:=true','autostart:=true'],'nav2')
 end=time.monotonic()+45
 while time.monotonic()<end:
  node.seed();rclpy.spin_once(node,timeout_sec=.1)
  if node.client.server_is_ready() and node.cost is not None and node.amcl is not None:break
 else:raise RuntimeError('NAV2_READY_TIMEOUT')
 spin_for(node,3,seed=True)
 print('NAV2_READY',node.sim,'costmap',node.cost.info.width,node.cost.info.height,flush=True)
 node.recording=True;node.status='목표 전송 전';spin_for(node,2)
 goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg();goal.pose.pose.position.x=1.5;goal.pose.pose.orientation.w=1.
 f=node.client.send_goal_async(goal,feedback_callback=lambda m:setattr(node,'feedback',m.feedback));rclpy.spin_until_future_complete(node,f,timeout_sec=10);gh=f.result()
 if gh is None or not gh.accepted:raise RuntimeError('GOAL_REJECTED')
 node.status='Nav2 주행 중';result=gh.get_result_async();start=time.monotonic();sim_start=node.sim;last_print=start
 while time.monotonic()-start<180 and node.sim-sim_start<50:
  rclpy.spin_once(node,timeout_sec=.05)
  if result.done():break
  if time.monotonic()-last_print>10:
   print('RUNNING',round(node.sim-sim_start,2),'sim seconds','frames',len(node.frames),'cmd',node.cmd,flush=True);last_print=time.monotonic()
 if result.done():status=result.result().status
 else:
  cancel=gh.cancel_goal_async();rclpy.spin_until_future_complete(node,cancel,timeout_sec=5);status=None
 node.status='목표 도달 (Nav2 판정)' if status==4 else '주행 종료 / 추가 확인';spin_for(node,4)
 summary={'case':case,'action_status':status,'nav2_succeeded':status==4,'goal':[1.5,0,0],'initial_pose':[-1.5,0,0],'camera_frames':len(node.frames),'sim_time_first':node.frames[0]['sim_time'] if node.frames else None,'sim_time_last':node.frames[-1]['sim_time'] if node.frames else None,'host_goal_seconds':time.monotonic()-start,'simulation_goal_seconds':node.sim-sim_start,'ground_truth_validated':False,'final_odom':[node.odom.pose.pose.position.x,node.odom.pose.pose.position.y] if node.odom else None,'domain_id':os.environ.get('ROS_DOMAIN_ID'),'simulation_only':True}
 (OUT/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));print('RESULT',json.dumps(summary,ensure_ascii=False),flush=True)
 node.tele.close();node.destroy_node();rclpy.shutdown()
except Exception as e:
 (OUT/'error.txt').write_text(repr(e));print('ERROR',repr(e),flush=True);sys.exit(1)
finally:
 owned=set()
 for logfile in OUT.glob("*.log"):
  for pid in re.findall(r"process started with pid \[(\d+)\]",logfile.read_text(errors="replace")):
   owned.add(int(pid))
 for pid in owned:
  try:os.kill(pid,signal.SIGINT)
  except ProcessLookupError:pass
 for p in reversed(children):
  if p.poll() is None:
   try:os.killpg(p.pid,signal.SIGINT)
   except ProcessLookupError:pass
 for p in reversed(children):
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:
   try:os.killpg(p.pid,signal.SIGTERM)
   except ProcessLookupError:pass
 for pid in owned:
  try:os.kill(pid,signal.SIGTERM)
  except ProcessLookupError:pass
 for pid in owned:
  try:
   os.kill(pid,0);os.kill(pid,signal.SIGKILL)
  except ProcessLookupError:pass
 for f in logs:f.close()
