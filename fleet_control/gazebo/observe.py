"""Read-only Gazebo evidence: native GT, TF localization, gate and accepted Nav2 goal UUIDs."""
import sys,json,time,math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from action_msgs.msg import GoalStatusArray
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from tf2_ros import Buffer,TransformListener
rclpy.init();n=Node('rmf_gazebo_observer',parameter_overrides=[Parameter('use_sim_time',value=True)])
b=Buffer();listener=TransformListener(b,n);data={'goals':{}}
def gt(m):
 q=m.pose.pose.orientation;data['gt']=[m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
def gate(m):
 try:data['gate']=json.loads(m.data)
 except ValueError:pass
def goals(m):
 for g in m.status_list:
  key=bytes(g.goal_info.goal_id.uuid).hex();data['goals'][key]={'status':g.status,'first_seen':data['goals'].get(key,{}).get('first_seen',time.time())}
n.create_subscription(Odometry,'/ground_truth/world_odom',gt,20)
n.create_subscription(String,'/stall_detector/status_json',gate,10)
n.create_subscription(GoalStatusArray,'navigate_to_pose/_action/status',goals,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
with open(sys.argv[1],'w') as f:
 try:
  last=0
  while rclpy.ok():
   rclpy.spin_once(n,timeout_sec=.05)
   if time.monotonic()-last<.3:continue
   last=time.monotonic();data['wall']=time.time();data['sim']=n.get_clock().now().nanoseconds/1e9
   try:
    t=b.lookup_transform('map','base_footprint',rclpy.time.Time());q=t.transform.rotation
    data['pose']=[t.transform.translation.x,t.transform.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
   except Exception:pass
   data['cmd_publishers']=[{'name':p.node_name,'namespace':p.node_namespace} for p in n.get_publishers_info_by_topic('/cmd_vel')]
   data['nodes']=[name for name,ns in n.get_node_names_and_namespaces()]
   f.write(json.dumps(data)+'\n');f.flush()
 except KeyboardInterrupt:pass
n.destroy_node()
if rclpy.ok():rclpy.shutdown()
