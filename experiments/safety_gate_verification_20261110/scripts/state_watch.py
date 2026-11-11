import json, rclpy, sys
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
class W(Node):
    def __init__(s):
        super().__init__('state_watch'); s.prev=None; s.nz=0; s.n=0
        s.create_subscription(String,'/stall_detector/status_json',s.cb,50)
        s.create_subscription(Twist,'/cmd_vel',s.cv,50)
    def cv(s,m):
        s.n+=1; s.nz+= (abs(m.linear.x)>1e-3 or abs(m.angular.z)>1e-3)
    def cb(s,m):
        d=json.loads(m.data)
        if d['state']!=s.prev:
            print(f"sim={d['sim_time']:.2f} state={d['state']} depth={d['depth_state']}/{d['depth_code']} cmd_in={d['cmd_vx_in']} /cmd_vel msgs={s.n} nonzero={s.nz}",flush=True); s.prev=d['state']
rclpy.init(); rclpy.spin(W())
