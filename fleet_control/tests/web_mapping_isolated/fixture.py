"""Static synthetic robot for the isolated web-mapping run: closed room with an open 70 deg sector (frontier)."""
import math, os, rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
from limo_msgs.msg import LimoStatus

class Sensors(Node):
    def __init__(self):
        super().__init__('iso_web_mapping_sensors')
        self.scan = self.create_publisher(LaserScan, '/scan', 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.status = self.create_publisher(LimoStatus, '/limo_status', 10)
        self.tf = StaticTransformBroadcaster(self)
        t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'; t.child_frame_id = 'laser'; t.transform.rotation.w = 1.
        self.tf.sendTransform(t)
        self.create_timer(.1, self.publish)
    def publish(self):
        stamp = self.get_clock().now().to_msg()
        o = Odometry(); o.header.stamp, o.header.frame_id = stamp, 'odom'; o.child_frame_id = 'base_footprint'; o.pose.pose.orientation.w = 1.
        self.odom.publish(o)
        s = LaserScan(); s.header.stamp, s.header.frame_id = stamp, 'laser'
        s.angle_min, s.angle_max = -math.pi, math.pi; s.angle_increment = 2 * math.pi / 359
        s.range_min, s.range_max, s.scan_time = .12, 8., .1
        r = []
        for i in range(360):
            a = -math.pi + i * s.angle_increment
            r.append(float('inf') if 0.2 < a < 1.42 else 2.5 + .3 * math.cos(i * .021) + .15 * math.cos(i * .057))
        s.ranges = r; self.scan.publish(s)
        st = LimoStatus(); st.header.stamp = stamp; st.battery_voltage = 11.6; self.status.publish(st)

if __name__ == '__main__':
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init(); n = Sensors()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
