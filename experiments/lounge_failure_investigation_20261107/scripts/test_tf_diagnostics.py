#!/usr/bin/env python3
"""
test_tf_diagnostics.py:
Verifies TF frames between Gazebo, Robot State Publisher, and AMCL/Nav2.
"""
import os
import signal
import subprocess
import time
from pathlib import Path

BASE_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107')

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

def kill_proc(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        p.wait(timeout=3)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            p.wait(timeout=2)
        except Exception:
            pass

def main():
    env = os.environ.copy()
    models_dir = BASE_DIR / 'models'
    worlds_dir = BASE_DIR / 'worlds'
    mesh_path = '/home/cjinw/simulator-research/limo_sim_ws/install/limo_description/share'
    env['IGN_GAZEBO_RESOURCE_PATH'] = f"{str(models_dir)}:{mesh_path}:{str(worlds_dir)}"
    env['GZ_SIM_RESOURCE_PATH'] = f"{str(models_dir)}:{mesh_path}:{str(worlds_dir)}"
    env['SDF_PATH'] = str(models_dir)

    print("[1] Launching Gazebo Fortress...")
    sim_cmd = [
        'ros2', 'launch',
        str(BASE_DIR / 'launch' / 'lounge_sim.launch.py'),
        'headless:=true'
    ]
    sim_proc = subprocess.Popen(sim_cmd, env=env, start_new_session=True)
    time.sleep(6)

    print("[2] Initializing ROS 2 Node with use_sim_time=True...")
    rclpy.init()
    node = Node('tf_diagnostic_node', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)
    ])
    buf = Buffer()
    tl = TransformListener(buf, node)
    initialpose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    # Spin a little to get initial TFs from Gazebo & robot_state_publisher
    t_start = time.time()
    while time.time() - t_start < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("\n--- Testing Robot TF Frames (Gazebo + Robot State Publisher) ---")
    frames_to_test = [
        ('odom', 'base_footprint'),
        ('base_footprint', 'base_link'),
        ('base_link', 'laser_link'),
        ('odom', 'base_link')
    ]
    for parent, child in frames_to_test:
        can = buf.can_transform(parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1))
        print(f"Transform [{parent} -> {child}]: {'AVAILABLE' if can else 'MISSING'}")

    print("\n[3] Launching Nav2 Stack...")
    nav2_cmd = [
        'ros2', 'launch',
        str(BASE_DIR / 'launch' / 'lounge_nav2.launch.py'),
        'use_sim_time:=true',
        'autostart:=true'
    ]
    nav2_proc = subprocess.Popen(nav2_cmd, env=env, start_new_session=True)
    time.sleep(9)

    # Publish initial pose repeatedly
    print("[4] Publishing /initialpose...")
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06

    for _ in range(15):
        initialpose_pub.publish(msg)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)

    print("\n--- Testing Map TF Frames (AMCL) ---")
    map_frames = [
        ('map', 'odom'),
        ('map', 'base_footprint'),
        ('map', 'base_link')
    ]
    for parent, child in map_frames:
        can = buf.can_transform(parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3))
        print(f"Transform [{parent} -> {child}]: {'AVAILABLE' if can else 'MISSING'}")

    # Cleanup
    kill_proc(nav2_proc)
    kill_proc(sim_proc)
    rclpy.shutdown()
    print("\n[5] Diagnostics completed.")

if __name__ == '__main__':
    main()
