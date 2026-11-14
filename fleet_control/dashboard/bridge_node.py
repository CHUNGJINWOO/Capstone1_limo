"""Independent ROS 2 subscriber bridge. No publishers, action goals, resets or robot configuration writes."""
import argparse, json, math, os, threading, time, urllib.request, uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener
from .core import Grid, pack, metric_depth, compose

DEFAULT_TOPICS={'map':'/map','global_costmap':'/global_costmap/costmap','local_costmap':'/local_costmap/costmap',
 'global_plan':'/plan','local_plan':'/local_plan','frontiers':'/explore/frontiers',
 'depth':'/camera/depth/image_raw','camera_info':'/camera/depth/camera_info','explore':'/explore/status'}

def stamp(header):return header.stamp.sec*1_000_000_000+header.stamp.nanosec

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Bridge(Node):
 def __init__(self,args):
  super().__init__('dashboard_readonly_bridge',start_parameter_services=False,enable_rosout=False)
  self.args=args;self.topics=dict(DEFAULT_TOPICS)
  if args.topics:self.topics.update(json.loads(Path(args.topics).read_text()))
  self.lock=threading.RLock();self.raw={};self.grids={};self.counts={};self.errors={};self.stop=threading.Event()
  self.tf=Buffer();self.listener=TransformListener(self.tf,self);self.session=uuid.uuid4().hex;self.seq=0;self.trail=[]
  self.metrics={'uploaded':0,'upload_errors':0,'bytes':0,'conversions':0,'last_conversion_ms':0,'old_depth_dropped':0,'depth_overwritten':0}
  latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  for name in ['map','global_costmap','local_costmap']:
   self.create_subscription(OccupancyGrid,self.topics[name],lambda m,n=name:self.grid(n,m),latched)
   if name!='map':self.create_subscription(OccupancyGridUpdate,self.topics[name]+'_updates',lambda m,n=name:self.patch(n,m),10)
  for name in ['global_plan','local_plan']:self.create_subscription(RosPath,self.topics[name],lambda m,n=name:self.put(n,m),10)
  self.create_subscription(MarkerArray,self.topics['frontiers'],lambda m:self.put('frontiers',m),10)
  self.create_subscription(Image,self.topics['depth'],lambda m:self.put('depth',m),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
  self.create_subscription(CameraInfo,self.topics['camera_info'],lambda m:self.put('camera_info',m),qos_profile_sensor_data)
  self.create_subscription(String,self.topics['explore'],lambda m:self.put('explore',m),10) if args.explore_json else self.explore_subscription()
  if self.topics.get('selected_frontier'):self.create_subscription(PoseStamped,self.topics['selected_frontier'],lambda m:self.put('selected_frontier',m),10)
  self.thread=threading.Thread(target=self.upload,daemon=True);self.thread.start()
 def explore_subscription(self):
  from explore_lite_msgs.msg import ExploreStatus
  return self.create_subscription(ExploreStatus,self.topics['explore'],lambda m:self.put('explore',m),10)
 def put(self,name,m):
  with self.lock:
   if name=='depth' and name in self.raw and stamp(self.raw[name].header)!=getattr(self,'processed_depth_stamp',None):self.metrics['depth_overwritten']+=1
   self.raw[name]=m;self.counts[name]=self.counts.get(name,0)+1
 def grid(self,name,m):
  try:
   value={'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution,
    'origin':{'x':m.info.origin.position.x,'y':m.info.origin.position.y,'yaw':yaw(m.info.origin.orientation)},
    'frame_id':m.header.frame_id,'stamp_ns':stamp(m.header),'topic':self.topics[name],
    'cells_zlib':pack(np.asarray(m.data,dtype=np.int8).tobytes())}
   g=Grid(value)
   with self.lock:self.grids[name]=g;self.counts[name]=self.counts.get(name,0)+1
  except Exception as e:self.errors[name]=type(e).__name__
 def patch(self,name,m):
  try:
   with self.lock:
    if name not in self.grids:return
    self.grids[name].update({'x':m.x,'y':m.y,'width':m.width,'height':m.height,'frame_id':m.header.frame_id or self.grids[name].frame,
     'stamp_ns':stamp(m.header),'cells_zlib':pack(np.asarray(m.data,dtype=np.int8).tobytes())})
    self.counts[name+'_updates']=self.counts.get(name+'_updates',0)+1
  except Exception as e:self.errors[name+'_updates']=type(e).__name__
 def age(self,ns):return (self.get_clock().now().nanoseconds-ns)/1e9 if ns else None
 def transform(self,frame,ns):
  if frame=='map':return {'x':0.,'y':0.,'yaw':0.}
  t=self.tf.lookup_transform('map',frame,rclpy.time.Time(nanoseconds=ns))
  q=t.transform.rotation
  # OccupancyGrid is planar; reject tilted transforms rather than projecting silently.
  if abs(q.x)>.025 or abs(q.y)>.025:raise ValueError('nonplanar map transform')
  return {'x':t.transform.translation.x,'y':t.transform.translation.y,'yaw':yaw(q),'tf_stamp_ns':stamp(t.header)}
 def point(self,x,y,frame,ns):
  t=self.transform(frame,ns);c,s=math.cos(t['yaw']),math.sin(t['yaw'])
  return [t['x']+c*x-s*y,t['y']+s*x+c*y]
 def path(self,m,name):
  ns=stamp(m.header);points=[]
  for p in m.poses:
   points.append(self.point(p.pose.position.x,p.pose.position.y,p.header.frame_id or m.header.frame_id,stamp(p.header) or ns))
  return {'frame_id':'map','source_frame_id':m.header.frame_id,'stamp_ns':ns,'source_age_s':self.age(ns),
   'topic':self.topics[name],'points':points[:10000],'renderable':True,'tf_policy':'message_stamp; latest only for zero stamp'}
 def payload(self):
  t=time.perf_counter();p={'kind':self.args.kind,'session':self.session,'sequence':self.seq,'robot_id':self.args.robot_id,
   'ros_time_ns':self.get_clock().now().nanoseconds,'clock_mode':'ros','grids':{},'paths':{},'unavailable':{}}
  with self.lock:
   for name,g in self.grids.items():
    try:tf=self.transform(g.frame,g.stamp_ns)
    except Exception:tf=None
    p['grids'][name]={'width':g.width,'height':g.height,'resolution':g.resolution,'origin':g.origin,'frame_id':g.frame,
     'map_from_frame':tf,'stamp_ns':g.stamp_ns,'source_age_s':self.age(g.stamp_ns),'topic':g.topic,'cells_zlib':pack(g.cells.tobytes())}
   raw=dict(self.raw)
  for name in ['global_plan','local_plan']:
   if name in raw:
    try:p['paths'][name]=self.path(raw[name],name)
    except Exception:p['unavailable'][name]='TF unavailable at message timestamp'
  if 'frontiers' in raw:
   try:
    points=[];ns=0
    for marker in raw['frontiers'].markers:
     if marker.action in [2,3]:continue
     ns=max(ns,stamp(marker.header))
     base={'x':marker.pose.position.x,'y':marker.pose.position.y,'yaw':yaw(marker.pose.orientation)}
     values=marker.points or [None]
     for v in values:
      local=compose(base,{'x':v.x if v else 0,'y':v.y if v else 0,'yaw':0})
      points.append(self.point(local['x'],local['y'],marker.header.frame_id,stamp(marker.header)))
    p['paths']['frontiers']={'frame_id':'map','stamp_ns':ns,'source_age_s':self.age(ns),'points':points[:10000],'renderable':True,'meaning':'candidate markers, not selected goal'}
   except Exception:p['unavailable']['frontiers']='TF unavailable'
  if 'selected_frontier' in raw:
   m=raw['selected_frontier'];ns=stamp(m.header)
   try:p['paths']['selected_frontier']={'frame_id':'map','stamp_ns':ns,'source_age_s':self.age(ns),'points':[self.point(m.pose.position.x,m.pose.position.y,m.header.frame_id,ns)],'renderable':True}
   except Exception:p['unavailable']['selected_frontier']='TF unavailable'
  else:p['unavailable']['selected_frontier']='No actual selected-goal topic configured'
  try:
   tmap=self.transform('base_footprint',0)
   if self.age(tmap.get('tf_stamp_ns',0)) is None or self.age(tmap['tf_stamp_ns'])>2:raise ValueError('stale robot TF')
   xy=[tmap['x'],tmap['y']];p['robot_pose']={'x':tmap['x'],'y':tmap['y'],'yaw_rad':tmap['yaw']}
   if not self.trail or math.dist(self.trail[-1],xy)>.02:self.trail.append(xy);self.trail=self.trail[-1000:]
   p['paths']['trail']={'frame_id':'map','points':self.trail,'renderable':True,'source_age_s':0,'meaning':'measured TF sample history'}
  except Exception:p['unavailable']['trail']='map->base_footprint unavailable'
  if 'depth' in raw:
   m=raw['depth'];ns=stamp(m.header);age=self.age(ns);self.processed_depth_stamp=ns
   if age is not None and -.2<=age<=2:
    try:
     p['depth']=dict(metric_depth(bytes(m.data),width=m.width,height=m.height,step=m.step,encoding=m.encoding,
       bigendian=bool(m.is_bigendian),scale_m=self.args.depth_scale_m,max_width=320),
       frame_id=m.header.frame_id,stamp_ns=ns,source_age_s=age,topic=self.topics['depth'],
       scale_source='configured ROS depth convention; device calibration unverified')
    except Exception as e:p['unavailable']['depth']=type(e).__name__
   else:self.metrics['old_depth_dropped']+=1;p['unavailable']['depth']='depth timestamp stale/unverified'
  if 'camera_info' in raw:
   m=raw['camera_info'];p['camera_info']={'frame_id':m.header.frame_id,'stamp_ns':stamp(m.header),'width':m.width,'height':m.height,'k':list(m.k),'d':list(m.d),'distortion_model':m.distortion_model,'rgb_registration_verified':False}
  if 'explore' in raw:
   m=raw['explore'];p['explore']={'status':getattr(m,'status',getattr(m,'data','UNKNOWN')),'topic':self.topics['explore']}
  for name,topic in self.topics.items():
   if self.count_publishers(topic)==0:p['unavailable'][name]='no active publisher'
  self.metrics['conversions']+=1;self.metrics['last_conversion_ms']=round((time.perf_counter()-t)*1000,2)
  p['metrics']={**self.metrics,'callbacks':dict(self.counts),'errors':dict(self.errors)}
  return p
 def upload(self):
  token=Path(self.args.token_file).read_text().strip() if self.args.token_file else None;backoff=.5
  while not self.stop.is_set():
   try:
    if self.args.lock_check=='required' and self.count_publishers('/cmd_vel'):
     self.errors['lock_check']='cmd_vel publisher present; initial bridge validation requires motor-locked mode';self.stop.wait(1);continue
    self.seq+=1;p=self.payload();body=json.dumps(p,allow_nan=False).encode()
    if self.args.stdout:print(body.decode(),flush=True)
    else:
     request=urllib.request.Request(self.args.endpoint,data=body,headers={'Content-Type':'application/json','Authorization':'Bearer '+token})
     with urllib.request.urlopen(request,timeout=2) as r:r.read()
    self.metrics['uploaded']+=1;self.metrics['bytes']+=len(body);backoff=.5;self.stop.wait(1/self.args.fps)
   except Exception as e:
    self.metrics['upload_errors']+=1;self.errors['upload']=type(e).__name__;self.stop.wait(backoff);backoff=min(8,backoff*2)

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--endpoint');ap.add_argument('--token-file');ap.add_argument('--stdout',action='store_true')
 ap.add_argument('--topics');ap.add_argument('--robot-id',default='pcu');ap.add_argument('--fps',type=float,default=1)
 ap.add_argument('--depth-scale-m',type=float,default=.001);ap.add_argument('--explore-json',action='store_true')
 ap.add_argument('--lock-check',choices=['required','observe-only'],default='required')
 ap.add_argument('--kind',choices=['ROS_LIVE','SYNTHETIC_ROS_TEST'],default='ROS_LIVE')
 args,rosargs=ap.parse_known_args()
 if not args.stdout and (not args.endpoint or not args.token_file):ap.error('--endpoint and --token-file required for upload')
 if not 0<args.fps<=3:ap.error('fps must be <=3')
 if args.kind!='ROS_LIVE' and (int(os.environ.get('ROS_DOMAIN_ID','0'))<180 or os.environ.get('ROS_LOCALHOST_ONLY')!='1'):ap.error('test source requires localhost-only isolated domain >=180')
 if args.token_file and Path(args.token_file).stat().st_mode & 0o077:ap.error('token file must be 0600')
 rclpy.init(args=rosargs);node=Bridge(args)
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:
  node.stop.set();node.thread.join(timeout=3);node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
