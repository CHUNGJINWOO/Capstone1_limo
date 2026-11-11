import os
import sys
import time
import subprocess
import signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

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
node = Node('diag_dwb')
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

cmd_vels = []
def on_cmd(msg):
    cmd_vels.append((time.time(), msg.linear.x, msg.angular.z))

node.create_subscription(Twist, '/cmd_vel', on_cmd, 10)
nav_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

print("[3] Waiting for Nav2 action server...")
nav_client.wait_for_server(timeout_sec=15.0)

# Send a navigation goal down the corridor: x=3.0, y=0.0
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = 'map'
goal.pose.pose.position.x = 3.0
goal.pose.pose.position.y = 0.0
goal.pose.pose.orientation.w = 1.0

print("[4] Sending goal to (3.0, 0.0)...")
fut = nav_client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
handle = fut.result()
print(f"Goal accepted: {handle.accepted}")

t0 = time.time()
print("[5] Monitoring cmd_vel for 15s...")
while time.time() - t0 < 15.0:
    rclpy.spin_once(node, timeout_sec=0.2)

# Print sampled cmd_vels
print(f"\nTotal cmd_vel samples: {len(cmd_vels)}")
non_zeros = [c for c in cmd_vels if abs(c[1]) > 0.001 or abs(c[2]) > 0.001]
print(f"Non-zero cmd_vel count: {len(non_zeros)}")
for s in cmd_vels[::10]:
    print(f"t={s[0]-t0:.1f}s: linear.x={s[1]:.4f}, angular.z={s[2]:.4f}")

try:
    tf = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
    print(f"Final robot map pose: x={tf.transform.translation.x:.3f}, y={tf.transform.translation.y:.3f}")
except Exception as e:
    print(f"TF lookup failed: {e}")

os.killpg(os.getpgid(map_proc.pid), signal.SIGINT)
os.killpg(os.getpgid(gz_proc.pid), signal.SIGINT)
rclpy.shutdown()
