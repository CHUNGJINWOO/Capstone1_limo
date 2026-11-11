import os
import sys
import time
import math
import subprocess
import signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

subprocess.run(['killall', '-9', 'ign', 'ruby', 'gzserver', 'gzclient', 'ros2', 'mapping_server', 'frontier_explorer', 'async_slam_toolbox_node', 'controller_server', 'planner_server', 'bt_navigator'], stderr=subprocess.DEVNULL)
time.sleep(2)

WORLD_SDF = "/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping/worlds/c103_obstacle_exploration.sdf"
WS_DIR = "/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping/ws"

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

print("[2] Launching Mapping & Nav2...")
map_cmd = (
    "source /opt/ros/humble/setup.bash && "
    "source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash && "
    f"source {WS_DIR}/install/setup.bash && "
    "ros2 launch limo_auto_mapping auto_mapping.launch.py use_sim_time:=true"
)
map_proc = subprocess.Popen(["bash", "-c", map_cmd], preexec_fn=os.setsid)
time.sleep(12)

rclpy.init()
node = Node('track_robot_motion')

odom_log = []
def on_odom(msg):
    p = msg.pose.pose.position
    r = msg.pose.pose.orientation
    siny_cosp = 2 * (r.w * r.z + r.x * r.y)
    cosy_cosp = 1 - 2 * (r.y * r.y + r.z * r.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    odom_log.append((time.time(), p.x, p.y, yaw))

last_cmd = (0.0, 0.0)
def on_cmd(msg):
    global last_cmd
    last_cmd = (msg.linear.x, msg.angular.z)

node.create_subscription(Odometry, '/odom', on_odom, 10)
node.create_subscription(Twist, '/cmd_vel', on_cmd, 10)

nav_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
nav_client.wait_for_server(timeout_sec=15.0)

goal = NavigateToPose.Goal()
goal.pose.header.frame_id = 'map'
goal.pose.pose.position.x = 3.0
goal.pose.pose.position.y = 0.0
goal.pose.pose.orientation.w = 1.0

print("[3] Sending goal to (3.0, 0.0)...")
fut = nav_client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)

t0 = time.time()
print("[4] Logging robot motion for 15 seconds...")
while time.time() - t0 < 15.0:
    rclpy.spin_once(node, timeout_sec=0.1)

print("\n--- MOTION LOG (every 1 second) ---")
if odom_log:
    t_start = odom_log[0][0]
    last_sec = -1
    for t, x, y, yaw in odom_log:
        sec = int(t - t_start)
        if sec != last_sec:
            last_sec = sec
            print(f"t={sec:2d}s: odom_x={x:+.3f}m, odom_y={y:+.3f}m, yaw={math.degrees(yaw):+.1f} deg | last_cmd: vx={last_cmd[0]:.2f}, wz={last_cmd[1]:.2f}")

os.killpg(os.getpgid(map_proc.pid), signal.SIGINT)
os.killpg(os.getpgid(gz_proc.pid), signal.SIGINT)
rclpy.shutdown()
