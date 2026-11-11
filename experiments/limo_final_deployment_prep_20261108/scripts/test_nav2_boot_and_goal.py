#!/usr/bin/env -S python3 -s
"""
test_nav2_boot_and_goal.py
Quick test to verify Nav2 lifecycle activation and 1 trial goal dispatch.
"""
import os
import sys
import time
import math
import signal
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap

EXP_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_final_deployment_prep_20261108'
base_sp = {'x': -1.10, 'y': -0.60, 'z': 0.05, 'yaw': 1.5708}
goal = {'x': -2.20, 'y': 3.35, 'yaw': 2.28}

env = dict(os.environ)
env['PYTHONNOUSERSITE'] = '1'

def kill_p(p):
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            p.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass

print("1. Launching Base Gazebo...")
p_gz = subprocess.Popen(
    ['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'base_gazebo_for_trials.launch.py')],
    env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
)
time.sleep(7.0)

print("2. Starting Boot TF & Relays for Nav2 Lifecycle...")
boot_procs = []
boot_procs.append(subprocess.Popen([
    'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
    '--x', str(base_sp['x']), '--y', str(base_sp['y']), '--z', '0.0',
    '--yaw', str(base_sp['yaw']), '--pitch', '0.0', '--roll', '0.0',
    '--frame-id', 'map', '--child-frame-id', 'odom',
    '--ros-args', '-p', 'use_sim_time:=true'
], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
boot_procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
                                   env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
boot_procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
                                   env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
boot_procs.append(subprocess.Popen([
    'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'odom_tf_relay_node.py'),
    '--ros-args', '-p', 'active_odom_topic:=/odometry/lidar_odom'
], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
boot_procs.append(subprocess.Popen(['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'state_estimators.launch.py')],
                                   env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

print("3. Launching Nav2 Stack...")
p_nav = subprocess.Popen(
    ['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'closed_loop_nav2_stack.launch.py')],
    env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
)
time.sleep(9.0)

print("4. Testing Goal Dispatch with Monitor Node...")
rclpy.init()
node = Node('test_nav2_boot_node', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

srv_local = node.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
srv_global = node.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')

if srv_local.wait_for_service(timeout_sec=3.0):
    srv_local.call_async(ClearEntireCostmap.Request())
if srv_global.wait_for_service(timeout_sec=3.0):
    srv_global.call_async(ClearEntireCostmap.Request())

for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.05)

print("Waiting for Action Server...")
if not client.wait_for_server(timeout_sec=5.0):
    print("Action server NOT available!")
else:
    print("Action Server READY! Sending Goal...")
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = 'map'
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = float(goal['x'])
    goal_msg.pose.pose.position.y = float(goal['y'])
    goal_msg.pose.pose.orientation.z = math.sin(goal['yaw'] * 0.5)
    goal_msg.pose.pose.orientation.w = math.cos(goal['yaw'] * 0.5)

    fut = client.send_goal_async(goal_msg)
    t0 = time.time()
    while time.time() - t0 < 4.0 and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.04)

    gh = fut.result() if fut.done() else None
    if gh:
        print(f"Goal Handle Accepted: {gh.accepted}")
        if gh.accepted:
            print("Nav2 Successfully Accepted Goal! Canceling test goal...")
            gh.cancel_goal_async()
            time.sleep(1.0)
    else:
        print("Goal future not done or None!")

node.destroy_node()
rclpy.shutdown()

print("Cleaning up processes...")
for p in boot_procs:
    kill_p(p)
kill_p(p_nav)
kill_p(p_gz)
print("Done.")
