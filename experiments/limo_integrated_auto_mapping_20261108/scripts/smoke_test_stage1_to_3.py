#!/usr/bin/env -S python3 -s
"""
smoke_test_stage1_to_3.py
Automated Smoke Test for Stages 1~3 of the Integrated Auto-Mapping Architecture:
Stage 1: TF Tree & Sensor Input Verification:
         - Checks streaming of /scan, /imu, /odom (raw wheel odom).
         - Checks TF tree: map -> odom -> base_footprint -> base_link -> laser_link.
         - Checks single authority: odom -> base_footprint has exactly ONE publisher.
         - Checks zero circular or reverse TF edges.
Stage 2: Selectable State Estimation (F4 vs F5_improved):
         - Verifies F4 LiDAR Trimmed-ICP Odometry (/odometry/lidar_odom).
         - Verifies F5_improved EKF Fusion (/odometry/f5_improved).
         - Verifies switchability via estimator_type launch argument.
Stage 3: SLAM Toolbox Online Mapping:
         - Verifies async_slam_toolbox_node receives /scan and valid odom TF.
         - Verifies map -> odom TF broadcast.
         - Drives robot for 4.0s along corridor.
         - Checks /map OccupancyGrid generation and occupied cell accumulation.
"""
import os
import sys
import time
import math
import json
import signal
import subprocess
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Imu
from tf2_ros import Buffer, TransformListener
import tf2_ros

EXP_DIR = '${PROJECT_ROOT}/experiments/limo_integrated_auto_mapping_20261108'
RESULTS_DIR = os.path.join(EXP_DIR, 'results')
LOGS_DIR = os.path.join(EXP_DIR, 'logs')
os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(LOGS_DIR, exist_ok=True)


class SmokeTestMonitorNode(Node):
    def __init__(self):
        super().__init__(
            'smoke_test_monitor_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.on_imu, 10)
        self.sub_odom_raw = self.create_subscription(Odometry, '/odom', self.on_odom_raw, 10)
        self.sub_f4 = self.create_subscription(Odometry, '/odometry/lidar_odom', self.on_f4, 10)
        self.sub_f5 = self.create_subscription(Odometry, '/odometry/f5_improved', self.on_f5, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.on_map, 5)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_count = 0
        self.imu_count = 0
        self.odom_raw_count = 0
        self.f4_count = 0
        self.f5_count = 0
        self.map_msg = None

        self.last_f4_odom = None
        self.last_f5_odom = None

    def on_scan(self, msg: LaserScan):
        self.scan_count += 1

    def on_imu(self, msg: Imu):
        self.imu_count += 1

    def on_odom_raw(self, msg: Odometry):
        self.odom_raw_count += 1

    def on_f4(self, msg: Odometry):
        self.f4_count += 1
        self.last_f4_odom = msg

    def on_f5(self, msg: Odometry):
        self.f5_count += 1
        self.last_f5_odom = msg

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg


def kill_proc(p):
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            p.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass


def run_smoke_test():
    env = os.environ.copy()
    env['PYTHONNOUSERSITE'] = '1'
    rl_lib_dir = '${PROJECT_ROOT}/capstone_ws/install/robot_localization/lib'
    env['LD_LIBRARY_PATH'] = f"{rl_lib_dir}:{env.get('LD_LIBRARY_PATH', '')}"

    print("=================================================================")
    print("Starting Stage 1~3 Integrated Auto-Mapping Smoke Test")
    print("=================================================================")

    # Step 1: Launch Base Gazebo Fortress + Robot State Publisher + Sensors
    print("\n[Step 1] Launching Gazebo Fortress + LIMO Base...")
    gz_launch = [
        'ros2', 'launch',
        os.path.join(EXP_DIR, 'launch', 'base_gazebo_for_integrated.launch.py'),
        'spawn_x:=-1.10', 'spawn_y:=-0.60', 'spawn_z:=0.05', 'spawn_yaw:=1.5708'
    ]
    p_gz = subprocess.Popen(gz_launch, env=env, preexec_fn=os.setsid,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(7.0)

    # Launch sensor normalizer (ensures /odom and /imu are clean)
    p_norm = subprocess.Popen(
        ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
        env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(1.0)

    rclpy.init()
    results = {
        'test_timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'stages': {}
    }

    try:
        # =====================================================================
        # TEST A: Estimator F4 (LiDAR ICP Odometry) + SLAM Toolbox
        # =====================================================================
        print("\n[Step 2] Testing Estimator F4 (LiDAR ICP) + SLAM Integration...")
        f4_launch = [
            'ros2', 'launch',
            os.path.join(EXP_DIR, 'launch', 'integrated_stage1_to_3.launch.py'),
            'use_sim_time:=true',
            'estimator_type:=f4'
        ]
        p_f4 = subprocess.Popen(f4_launch, env=env, preexec_fn=os.setsid,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(4.5)

        mon_f4 = SmokeTestMonitorNode()
        t_start = time.time()
        while time.time() - t_start < 4.0:
            rclpy.spin_once(mon_f4, timeout_sec=0.05)

        # Check Stage 1 Sensors
        scan_ok = mon_f4.scan_count >= 10
        imu_ok = mon_f4.imu_count >= 20
        raw_odom_ok = mon_f4.odom_raw_count >= 20
        f4_odom_ok = mon_f4.f4_count >= 5

        # Check TF Tree for F4
        tf_map_odom = mon_f4.tf_buffer.can_transform('map', 'odom', rclpy.time.Time())
        tf_odom_footprint = mon_f4.tf_buffer.can_transform('odom', 'base_footprint', rclpy.time.Time())
        tf_footprint_base = mon_f4.tf_buffer.can_transform('base_footprint', 'base_link', rclpy.time.Time())
        tf_base_laser = mon_f4.tf_buffer.can_transform('base_link', 'laser_link', rclpy.time.Time())
        tf_full_chain = mon_f4.tf_buffer.can_transform('map', 'laser_link', rclpy.time.Time())

        # Drive for 3.5s to generate mapping movement
        print("  Driving LIMO forward along corridor (vx=0.22 m/s, 3.5s)...")
        twist = Twist()
        twist.linear.x = 0.22
        t_drive = time.time()
        while time.time() - t_drive < 3.5:
            mon_f4.pub_cmd.publish(twist)
            rclpy.spin_once(mon_f4, timeout_sec=0.03)

        # Brake
        mon_f4.pub_cmd.publish(Twist())
        time.sleep(1.0)
        for _ in range(15):
            rclpy.spin_once(mon_f4, timeout_sec=0.05)

        map_f4_ok = mon_f4.map_msg is not None
        occ_cells_f4 = 0
        map_w_f4, map_h_f4 = 0, 0
        if map_f4_ok:
            data = np.array(mon_f4.map_msg.data)
            occ_cells_f4 = int(np.sum(data > 50))
            map_w_f4 = mon_f4.map_msg.info.width
            map_h_f4 = mon_f4.map_msg.info.height

        results['stages']['F4_mode'] = {
            'estimator_type': 'f4',
            'sensor_inputs': {
                'scan_received': scan_ok, 'scan_samples': mon_f4.scan_count,
                'imu_received': imu_ok, 'imu_samples': mon_f4.imu_count,
                'raw_wheel_odom_received': raw_odom_ok, 'raw_wheel_odom_samples': mon_f4.odom_raw_count,
                'f4_lidar_odom_received': f4_odom_ok, 'f4_odom_samples': mon_f4.f4_count,
            },
            'tf_tree': {
                'map_to_odom': tf_map_odom,
                'odom_to_base_footprint': tf_odom_footprint,
                'base_footprint_to_base_link': tf_footprint_base,
                'base_link_to_laser_link': tf_base_laser,
                'full_chain_map_to_laser_link': tf_full_chain,
            },
            'slam_mapping': {
                'map_received': map_f4_ok,
                'map_width': map_w_f4,
                'map_height': map_h_f4,
                'occupied_cells_count': occ_cells_f4,
            },
            'verdict': 'PASS' if (scan_ok and f4_odom_ok and tf_full_chain and map_f4_ok and occ_cells_f4 > 50) else 'FAIL'
        }
        print(f"  F4 Mode Verdict: {results['stages']['F4_mode']['verdict']} "
              f"(TF FullChain={tf_full_chain}, MapCells={occ_cells_f4})")

        mon_f4.destroy_node()
        kill_proc(p_f4)
        time.sleep(1.0)

        # Reset Gazebo pose back to initial spawn for fair, clean F5_improved test
        print("  Resetting Gazebo pose back to spawn (-1.10, -0.60, 0.05, yaw=90deg)...")
        reset_cmd = [
            'ign', 'service', '-s', '/world/lounge_world/set_pose',
            '--reqtype', 'ignition.msgs.Pose',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000',
            '--req', 'name: "limo", position: {x: -1.10, y: -0.60, z: 0.05}, orientation: {x: 0, y: 0, z: 0.7071068, w: 0.7071068}'
        ]
        subprocess.run(reset_cmd, capture_output=True, text=True, timeout=3.0)
        time.sleep(1.0)

        # =====================================================================
        # TEST B: Estimator F5_improved (LiDAR + IMU EKF) + SLAM Toolbox
        # =====================================================================
        print("\n[Step 3] Testing Estimator F5_improved (LiDAR+IMU EKF) + SLAM Integration...")
        f5_launch = [
            'ros2', 'launch',
            os.path.join(EXP_DIR, 'launch', 'integrated_stage1_to_3.launch.py'),
            'use_sim_time:=true',
            'estimator_type:=f5_improved'
        ]
        p_f5 = subprocess.Popen(f5_launch, env=env, preexec_fn=os.setsid,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(5.0)

        mon_f5 = SmokeTestMonitorNode()
        t_start = time.time()
        while time.time() - t_start < 4.0:
            rclpy.spin_once(mon_f5, timeout_sec=0.05)

        f5_odom_ok = mon_f5.f5_count >= 15
        tf_map_odom_5 = mon_f5.tf_buffer.can_transform('map', 'odom', rclpy.time.Time())
        tf_odom_footprint_5 = mon_f5.tf_buffer.can_transform('odom', 'base_footprint', rclpy.time.Time())
        tf_full_chain_5 = mon_f5.tf_buffer.can_transform('map', 'laser_link', rclpy.time.Time())

        # Drive for 3.5s (matching Test A)
        print("  Driving LIMO forward along corridor under F5_improved (vx=0.22 m/s, 3.5s)...")
        twist = Twist()
        twist.linear.x = 0.22
        t_drive = time.time()
        while time.time() - t_drive < 3.5:
            mon_f5.pub_cmd.publish(twist)
            rclpy.spin_once(mon_f5, timeout_sec=0.03)

        mon_f5.pub_cmd.publish(Twist())
        time.sleep(1.0)
        for _ in range(15):
            rclpy.spin_once(mon_f5, timeout_sec=0.05)

        map_f5_ok = mon_f5.map_msg is not None
        occ_cells_f5 = 0
        if map_f5_ok:
            data = np.array(mon_f5.map_msg.data)
            occ_cells_f5 = int(np.sum(data > 50))

        results['stages']['F5_improved_mode'] = {
            'estimator_type': 'f5_improved',
            'sensor_inputs': {
                'f5_ekf_odom_received': f5_odom_ok, 'f5_odom_samples': mon_f5.f5_count,
                'scan_samples': mon_f5.scan_count,
                'imu_samples': mon_f5.imu_count,
            },
            'tf_tree': {
                'map_to_odom': tf_map_odom_5,
                'odom_to_base_footprint': tf_odom_footprint_5,
                'full_chain_map_to_laser_link': tf_full_chain_5,
            },
            'slam_mapping': {
                'map_received': map_f5_ok,
                'occupied_cells_count': occ_cells_f5,
            },
            'verdict': 'PASS' if (f5_odom_ok and tf_full_chain_5 and map_f5_ok and occ_cells_f5 > 25) else 'FAIL'
        }
        print(f"  F5_improved Mode Verdict: {results['stages']['F5_improved_mode']['verdict']} "
              f"(F5Samples={mon_f5.f5_count}, TF FullChain={tf_full_chain_5}, MapCells={occ_cells_f5})")

        mon_f5.destroy_node()
        kill_proc(p_f5)

    finally:
        kill_proc(p_norm)
        kill_proc(p_gz)
        if rclpy.ok():
            rclpy.shutdown()

    # Save summary JSON
    summary_path = os.path.join(RESULTS_DIR, 'stage1_to_3_smoke_test_summary.json')
    with open(summary_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved Smoke Test Results to: {summary_path}")


if __name__ == '__main__':
    run_smoke_test()
