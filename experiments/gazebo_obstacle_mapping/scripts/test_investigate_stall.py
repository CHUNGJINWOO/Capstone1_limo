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
from nav_msgs.msg import Odometry, OccupancyGrid
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
node = Node('investigate_stall')

latest_odom = None
def on_odom(msg):
    global latest_odom
    p = msg.pose.pose.position
    r = msg.pose.pose.orientation
    siny_cosp = 2 * (r.w * r.z + r.x * r.y)
    cosy_cosp = 1 - 2 * (r.y * r.y + r.z * r.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    latest_odom = (p.x, p.y, yaw)

latest_cmd = (0.0, 0.0)
def on_cmd(msg):
    global latest_cmd
    latest_cmd = (msg.linear.x, msg.angular.z)

node.create_subscription(Odometry, '/odom', on_odom, 10)
node.create_subscription(Twist, '/cmd_vel', on_cmd, 10)

nav_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
nav_client.wait_for_server(timeout_sec=15.0)

# Goal 1: 0.97, -0.01 (the one that succeeded quickly)
print("\n--- Sending Goal 1 (0.97, -0.01) ---")
g1 = NavigateToPose.Goal()
g1.pose.header.frame_id = 'map'
g1.pose.pose.position.x = 0.97
g1.pose.pose.position.y = -0.01
g1.pose.pose.orientation.w = 1.0
fut1 = nav_client.send_goal_async(g1)
rclpy.spin_until_future_complete(node, fut1, timeout_sec=5.0)
handle1 = fut1.result()
res_fut1 = handle1.get_result_async()

t0 = time.time()
while time.time() - t0 < 8.0:
    rclpy.spin_once(node, timeout_sec=0.2)
    if res_fut1.done():
        print(f"Goal 1 finished at t={time.time()-t0:.1f}s, status: {res_fut1.result().status}")
        break
    if latest_odom:
        print(f"Goal 1 | Odom: x={latest_odom[0]:.3f}, y={latest_odom[1]:.3f}, yaw={math.degrees(latest_odom[2]):.1f}° | cmd: vx={latest_cmd[0]:.2f}, wz={latest_cmd[1]:.2f}")

time.sleep(1)

# Goal 2: (3.77, -0.41) - The one where stall was observed
print("\n--- Sending Goal 2 (3.77, -0.41) ---")
g2 = NavigateToPose.Goal()
g2.pose.header.frame_id = 'map'
g2.pose.pose.position.x = 3.77
g2.pose.pose.position.y = -0.41
g2.pose.pose.orientation.w = 1.0
fut2 = nav_client.send_goal_async(g2)
rclpy.spin_until_future_complete(node, fut2, timeout_sec=5.0)
handle2 = fut2.result()
res_fut2 = handle2.get_result_async()

t0 = time.time()
while time.time() - t0 < 25.0:
    rclpy.spin_once(node, timeout_sec=0.5)
    if res_fut2.done():
        print(f"Goal 2 finished at t={time.time()-t0:.1f}s, status: {res_fut2.result().status}")
        break
    if latest_odom:
        print(f"Goal 2 | t={time.time()-t0:4.1f}s | Odom: x={latest_odom[0]:.3f}, y={latest_odom[1]:.3f}, yaw={math.degrees(latest_odom[2]):.1f}° | cmd: vx={latest_cmd[0]:.3f}, wz={latest_cmd[1]:.3f}")

os.killpg(os.getpgid(map_proc.pid), signal.SIGINT)
os.killpg(os.getpgid(gz_proc.pid), signal.SIGINT)
rclpy.shutdown()
