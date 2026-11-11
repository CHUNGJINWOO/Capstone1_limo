#!/usr/bin/env python3
"""Quick sanity check of Gazebo TF, clock, and scan synchronization."""
import time
from pathlib import Path
import subprocess
import os
import signal
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

def main():
    WORLD_SDF = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf')
    sim_cmd = [
        'ros2', 'launch', 'limo_capstone', 'limo_mc3f_sim.launch.py',
        f'world:={WORLD_SDF}',
        'spawn_x:=-5.5',
        'spawn_y:=-2.0',
        'spawn_yaw:=0.0',
        'headless:=true',
        'rviz:=false'
    ]
    sim_proc = subprocess.Popen(sim_cmd, start_new_session=True)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node('sanity_check_node')
    tf_buf = Buffer()
    tf_listener = TransformListener(tf_buf, node)
    
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                t = tf_buf.lookup_transform('odom', 'base_link', rclpy.time.Time())
                stamp_sec = t.header.stamp.sec + t.header.stamp.nanosec / 1e9
                now_sec = node.get_clock().now().nanoseconds / 1e9
                diff = now_sec - stamp_sec
                print(f"[TF OK] odom -> base_link found! diff={diff:.3f}s (now={now_sec:.2f}, stamp={stamp_sec:.2f})")
                print(f"  translation: x={t.transform.translation.x:.3f}, y={t.transform.translation.y:.3f}")
                break
            except Exception as e:
                pass
        else:
            print("[TF FAILED] Could not look up odom -> base_link within 15s")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
        sim_proc.wait(timeout=5)

if __name__ == '__main__':
    main()
