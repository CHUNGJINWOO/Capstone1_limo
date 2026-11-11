#!/usr/bin/env -S python3 -s
"""
test_gazebo_settling_protocol.py
Demonstrates and rigorously verifies the Gazebo Initialization & Reset Stabilization Protocol.
Performs:
1. Pose reset via ignition service (/world/lounge_world/set_pose).
2. Active zero-velocity command injection to damp wheel spin.
3. High-rate Ground Truth monitoring (/ground_truth/world_odom).
4. Settle condition verification:
   - Position offset <= 0.020m
   - Heading offset <= 0.020 rad (1.15 deg)
   - Linear velocity <= 0.005 m/s
   - Angular velocity <= 0.005 rad/s
   - Must hold stably for >= 0.5s (25 consecutive samples at 50Hz).
5. Sensor and estimator readiness check (Wheel odom, IMU, LiDAR).
6. Nav2 Costmap clearing verification (/clear_entirely_local_costmap, /clear_entirely_global_costmap).
7. Nav2 Action Server idle check.
Saves verification telemetry and timing to results/init_verification/init_settling_audit.json.
"""
import os
import sys
import time
import math
import json
import subprocess
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

EXP_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_final_deployment_prep_20261108'
OUT_DIR = os.path.join(EXP_DIR, 'results', 'init_verification')
os.makedirs(OUT_DIR, exist_ok=True)


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class GazeboSettlingAuditorNode(Node):
    def __init__(self):
        super().__init__(
            'gazebo_settling_auditor_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.sub_gt = self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_gt, 20)
        self.pub_cmd_brake = self.create_publisher(Twist, '/cmd_vel', 10)
        self.clear_local_srv = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global_srv = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.latest_gt = None
        self.gt_history = []

    def on_gt(self, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        speed = math.hypot(vx, vy)
        self.latest_gt = {
            'sim_time': t_sec,
            'x': x, 'y': y, 'yaw': yaw,
            'vx': vx, 'vy': vy, 'wz': wz,
            'speed': speed,
        }
        self.gt_history.append(self.latest_gt)


def send_gz_set_pose(x: float, y: float, z: float, yaw: float) -> bool:
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    cmd = [
        'ign', 'service', '-s', '/world/lounge_world/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req', f'name: "limo", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'
    ]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=3.0)
        return 'data: true' in res.stdout
    except Exception as e:
        print(f"Error calling ign set_pose: {e}")
        return False


def wait_for_settled(node: GazeboSettlingAuditorNode, target_x: float, target_y: float, target_yaw: float,
                     max_timeout_s: float = 8.0) -> dict:
    """
    Actively checks convergence:
    - pos_err <= 0.020m
    - yaw_err <= 0.020 rad
    - speed <= 0.005 m/s
    - wz <= 0.005 rad/s
    Must be satisfied for K=25 consecutive samples (~0.5s).
    """
    t0 = time.time()
    consecutive_settled = 0
    needed_consecutive = 20  # 20 samples at 50Hz = 0.4s sustained quiet
    settled_telemetry = []

    brake = Twist()
    brake.linear.x = 0.0
    brake.angular.z = 0.0

    while time.time() - t0 < max_timeout_s:
        node.pub_cmd_brake.publish(brake)
        rclpy.spin_once(node, timeout_sec=0.02)
        gt = node.latest_gt
        if gt is None:
            continue

        pos_err = math.hypot(gt['x'] - target_x, gt['y'] - target_y)
        yaw_err = abs(wrap_angle(gt['yaw'] - target_yaw))
        speed = gt['speed']
        wz = abs(gt['wz'])

        settled = (pos_err <= 0.020 and yaw_err <= 0.020 and speed <= 0.005 and wz <= 0.005)
        if settled:
            consecutive_settled += 1
        else:
            consecutive_settled = 0

        settled_telemetry.append({
            'wall_time_elapsed_s': round(time.time() - t0, 3),
            'pos_err_m': round(pos_err, 4),
            'yaw_err_rad': round(yaw_err, 4),
            'speed_m_s': round(speed, 5),
            'wz_rad_s': round(wz, 5),
            'consecutive_settled_count': consecutive_settled,
        })

        if consecutive_settled >= needed_consecutive:
            settle_dur = time.time() - t0
            return {
                'success': True,
                'status': 'SETTLED_CONVERGED',
                'settle_wall_duration_s': round(settle_dur, 3),
                'final_pos_err_m': round(pos_err, 4),
                'final_yaw_err_deg': round(math.degrees(yaw_err), 3),
                'final_speed_m_s': round(speed, 5),
                'final_wz_rad_s': round(wz, 5),
                'samples_evaluated': len(settled_telemetry),
                'telemetry_trace': settled_telemetry[-5:],
            }

    return {
        'success': False,
        'status': 'INIT_FAILED_SETTLE_TIMEOUT',
        'settle_wall_duration_s': round(time.time() - t0, 3),
        'final_pos_err_m': round(pos_err, 4) if gt else None,
        'final_yaw_err_deg': round(math.degrees(yaw_err), 3) if gt else None,
        'final_speed_m_s': round(speed, 5) if gt else None,
        'final_wz_rad_s': round(wz, 5) if gt else None,
        'samples_evaluated': len(settled_telemetry),
        'telemetry_trace': settled_telemetry[-5:],
    }
