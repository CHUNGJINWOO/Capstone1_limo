"""SYNTHETIC ROS TEST ONLY. Publishes visualization messages, never velocity/action/control."""
import math,os,time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid,Path
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped,TransformStamped
from sensor_msgs.msg import Image,CameraInfo
from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

class Fixture(Node):
 def __init__(self):
  super().__init__('dashboard_SYNTHETIC_fixture');self.start=time.monotonic();self.count=0
  q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.maps={k:self.create_publisher(OccupancyGrid,t,q) for k,t in [('map','/map'),('global_costmap','/global_costmap/costmap'),('local_costmap','/local_costmap/costmap')]}
  self.patch_pub=self.create_publisher(OccupancyGridUpdate,'/global_costmap/costmap_updates',10)
  self.plan=self.create_publisher(Path,'/plan',10);self.local=self.create_publisher(Path,'/local_plan',10)
  self.depth=self.create_publisher(Image,'/camera/depth/image_raw',qos_profile_sensor_data)
  self.info=self.create_publisher(CameraInfo,'/camera/depth/camera_info',qos_profile_sensor_data)
  self.front=self.create_publisher(MarkerArray,'/explore/frontiers',10);self.goal=self.create_publisher(PoseStamped,'/test/selected_frontier',10)
  self.explore=self.create_publisher(String,'/test/explore',10);self.tf=TransformBroadcaster(self)
  self.create_timer(.5,self.publish);self.create_timer(.2,self.publish_depth);self.create_timer(.05,self.transforms)
 def transforms(self):
  now=self.get_clock().now().to_msg();t=time.monotonic()-self.start
  a=TransformStamped();a.header.stamp=now;a.header.frame_id='map';a.child_frame_id='odom';a.transform.translation.x=.18;a.transform.translation.y=-.12;a.transform.rotation.z=math.sin(.12/2);a.transform.rotation.w=math.cos(.12/2)
  b=TransformStamped();b.header.stamp=now;b.header.frame_id='odom';b.child_frame_id='base_footprint';b.transform.translation.x=.4*math.sin(t*.08);b.transform.translation.y=.35*math.cos(t*.08);b.transform.rotation.z=math.sin(.25/2);b.transform.rotation.w=math.cos(.25/2)
  self.tf.sendTransform([a,b])
 def grid(self,name,a,origin,frame='map',angle=0):
  m=OccupancyGrid();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id=frame
  m.info.resolution=.05;m.info.width=a.shape[1];m.info.height=a.shape[0];m.info.origin.position.x=float(origin[0]);m.info.origin.position.y=float(origin[1]);m.info.origin.orientation.z=math.sin(angle/2);m.info.origin.orientation.w=math.cos(angle/2);m.data=a.astype(np.int8).ravel().tolist();self.maps[name].publish(m)
 def path(self,points,frame='map'):
  m=Path();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id=frame
  for x,y in points:
   p=PoseStamped();p.header=m.header;p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.w=1.;m.poses.append(p)
  return m
 def publish(self):
  self.count+=1;t=time.monotonic()-self.start
  a=np.full((110,140),-1,np.int8);a[8:102,8:132]=0;a[8:11,8:132]=100;a[99:102,8:132]=100;a[8:102,8:11]=100;a[8:102,129:132]=100
  a[10:47,48:51]=100;a[64:100,48:51]=100;a[63:66,50:104]=100;a[64:100,102:105]=100
  a[27:35,75:83]=100;a[78:86,21:29]=100
  self.grid('map',a,(-3.5,-2.5),angle=.22)
  cost=np.zeros_like(a);cost[a<0]=-1
  for dy in range(-5,6):
   for dx in range(-5,6):
    if dx*dx+dy*dy>25:continue
    shifted=np.roll(np.roll(a==100,dy,0),dx,1);value=100 if dx==dy==0 else max(5,90-int(math.hypot(dx,dy)*15));cost[shifted]=np.maximum(cost[shifted],value)
  self.grid('global_costmap',cost,(-3.5,-2.5),angle=.22)
  patch=OccupancyGridUpdate();patch.header.stamp=self.get_clock().now().to_msg();patch.header.frame_id='map';patch.x=60;patch.y=45;patch.width=8;patch.height=8;patch.data=([100]*64 if self.count%12<6 else [0]*64);self.patch_pub.publish(patch)
  yy,xx=np.indices((60,60));local=np.maximum(0,95-np.hypot(xx-43,yy-36)*12).astype(np.int8);local[(xx-43)**2+(yy-36)**2<9]=100
  self.grid('local_costmap',local,(-1.5+.4*math.sin(t*.08),-1.5+.35*math.cos(t*.08)),frame='odom')
  self.plan.publish(self.path([(-.1,.1),(.5,.3),(1.1,.35),(1.55,.7),(1.7,1.3)]))
  self.local.publish(self.path([(.1,.1),(.3,.23),(.5,.3),(.8,.31)],frame='odom'))
  markers=MarkerArray()
  for i,(x,y) in enumerate([(-1.4,1.8),(1.7,1.3),(2.1,-1.1),(-2.1,-.7)]):
   m=Marker();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='map';m.ns='SYNTHETIC_candidates';m.id=i;m.type=Marker.SPHERE;m.action=Marker.ADD;m.pose.position.x=x;m.pose.position.y=y;m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=.08;m.color.a=1.;markers.markers.append(m)
  self.front.publish(markers)
  target=PoseStamped();target.header.stamp=self.get_clock().now().to_msg();target.header.frame_id='map';target.pose.position.x=1.7;target.pose.position.y=1.3;target.pose.orientation.w=1.;self.goal.publish(target)
  self.explore.publish(String(data='SYNTHETIC_EXPLORING'))
 def publish_depth(self):
  yy,xx=np.indices((240,320));z=(.45+xx*.009+yy*.0015+math.sin((time.monotonic()-self.start)*.3)*.08)
  z[(xx-95)**2+(yy-100)**2<35**2]=.85;z[(xx<20)|(yy<10)]=0
  values=(z*1000).astype('<u2');m=Image();m.header.stamp=self.get_clock().now().to_msg();m.header.frame_id='SYNTHETIC_depth_optical';m.width=320;m.height=240;m.encoding='16UC1';m.step=640;m.is_bigendian=0;m.data=values.tobytes();self.depth.publish(m)
  c=CameraInfo();c.header=m.header;c.width=320;c.height=240;c.k=[285.,0.,160.,0.,285.,120.,0.,0.,1.];c.distortion_model='plumb_bob';c.d=[0.]*5;self.info.publish(c)

def main():
 assert int(os.environ.get('ROS_DOMAIN_ID','0'))>=180 and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
 rclpy.init();n=Fixture()
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:
  n.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
