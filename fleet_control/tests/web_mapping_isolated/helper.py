"""Isolated-domain fault helpers. usage: helper.py foreign_goal X Y | helper.py bypass_publisher"""
import os, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
rclpy.init()
n = Node('iso_fault_helper')
if sys.argv[1] == 'foreign_goal':           # a NavigateToPose client that is not the Agent (like explore_lite or RViz)
    ac = ActionClient(n, NavigateToPose, 'navigate_to_pose'); ac.wait_for_server(timeout_sec=5)
    g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'
    g.pose.pose.position.x, g.pose.pose.position.y = float(sys.argv[2]), float(sys.argv[3]); g.pose.pose.orientation.w = 1.0
    f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=5)
    print('accepted', f.result().accepted if f.result() else None, flush=True)
    end = time.time() + 30
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.2)
else:                                       # a second /cmd_vel_raw source bypassing the watchdog
    pub = n.create_publisher(Twist, '/cmd_vel_raw', 10)
    t = Twist(); t.linear.x = 0.05
    n.create_timer(0.05, lambda: pub.publish(t))
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
