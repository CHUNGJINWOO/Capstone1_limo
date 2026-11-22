import rclpy,math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
class N(Node):
 def __init__(self):
  super().__init__("synthetic_scanner");self.pub=self.create_publisher(LaserScan,"/scan",10);self.tf=TransformBroadcaster(self);self.create_timer(.1,self.tick)
 def tick(self):
  now=self.get_clock().now().to_msg()
  for parent,child in [("odom","base_footprint"),("base_footprint","laser")]:
   x=TransformStamped();x.header.stamp=now;x.header.frame_id=parent;x.child_frame_id=child;x.transform.rotation.w=1.0;self.tf.sendTransform(x)
  s=LaserScan();s.header.stamp=now;s.header.frame_id="laser";s.angle_min=-math.pi;s.angle_max=math.pi;s.angle_increment=2*math.pi/359;s.scan_time=.1;s.time_increment=.1/360;s.range_min=.1;s.range_max=8.;s.ranges=[2.0+0.3*math.sin(i*.1) for i in range(360)];self.pub.publish(s)
rclpy.init();n=N();rclpy.spin(n)
