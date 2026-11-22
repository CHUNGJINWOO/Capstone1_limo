"""TEST ONLY: zero Twist on /cmd_vel_raw under whatever node name --ros-args gives (used to impersonate the watchdog's name)."""
import os, rclpy
from geometry_msgs.msg import Twist
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0', '77')
rclpy.init(); n = rclpy.create_node('impostor'); p = n.create_publisher(Twist, '/cmd_vel_raw', 10)
n.create_timer(.05, lambda: p.publish(Twist()))
try: rclpy.spin(n)
except KeyboardInterrupt: pass
