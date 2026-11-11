import os
import sys
import time
import subprocess
import signal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

WORLD_SDF = "/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping/worlds/c103_obstacle_exploration.sdf"

subprocess.run(['killall', '-9', 'ign', 'ruby', 'gzserver', 'gzclient', 'ros2'], stderr=subprocess.DEVNULL)
time.sleep(2)

print("[1] Launching Gazebo...")
gz_cmd = (
    "source /opt/ros/humble/setup.bash && "
    "source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash && "
    f"ros2 launch limo_capstone limo_mc3f_sim.launch.py "
    f"world:={WORLD_SDF} spawn_x:=-3.0 spawn_y:=-1.1 spawn_z:=0.05 spawn_yaw:=0.0 "
    f"headless:=true rviz:=false"
)
gz_proc = subprocess.Popen(["bash", "-c", gz_cmd], preexec_fn=os.setsid)
time.sleep(10)

rclpy.init()
node = Node('cmd_test')
pub = node.create_publisher(Twist, '/cmd_vel', 10)
odom_poses = []

def on_odom(msg):
    p = msg.pose.pose.position
    odom_poses.append((p.x, p.y))

sub = node.create_subscription(Odometry, '/odom', on_odom, 10)

print("[2] Publishing forward cmd_vel: linear.x = 0.25, angular.z = 0.0 for 5 seconds...")
t0 = time.time()
while time.time() - t0 < 5.0:
    msg = Twist()
    msg.linear.x = 0.25
    msg.angular.z = 0.0
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)

print(f"Odom poses received: {len(odom_poses)}")
if odom_poses:
    p0 = odom_poses[0]
    p1 = odom_poses[-1]
    import math
    dist = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
    print(f"Start pos: {p0}, End pos: {p1}, Dist: {dist:.3f}m")

os.killpg(os.getpgid(gz_proc.pid), signal.SIGINT)
rclpy.shutdown()
