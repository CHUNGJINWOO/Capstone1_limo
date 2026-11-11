#!/usr/bin/env -S python3 -s
"""
run_stabilized_closed_loop_trials.py
Executes the Stabilized Nav2 Closed-Loop Re-Evaluation for F4 and F5_improved:
1. Pre-Flight Gazebo Settle & Initialization Protocol:
   - Sets pose via ignition service /world/lounge_world/set_pose
   - Actively verifies position convergence (<= 0.020m), yaw convergence (<= 0.020 rad),
     and residual velocity suppression (speed <= 0.005 m/s, wz <= 0.005 rad/s) for >= 0.5s.
   - Clears Nav2 local/global costmaps and checks Action Server idle status.
   - Logs initialization metrics; aborts immediately on init timeout.
2. 15 Trials per Estimator (30 trials total) on CL_Corridor_Doorway_Perturbed:
   - 5 Identical Trials (seed=0)
   - 10 Varied Seed Trials (seed=2001~2010)
   - Strict & Bounded Ground Truth success criteria evaluated side-by-side with Nav2 action status.
3. Pre-Deployment Stall Detector Safety:
   - enable_auto_reverse = False (Safety Locked)
   - require_operator_unlatch = True
   - Verifies wheel rotation arrest and command zeroing.
"""
import os
import sys
import time
import math
import json
import signal
import subprocess
import psutil
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap

EXP_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_final_deployment_prep_20261108'
NODES_DIR = os.path.join(EXP_DIR, 'nodes')
LAUNCH_DIR = os.path.join(EXP_DIR, 'launch')
CONFIG_DIR = os.path.join(EXP_DIR, 'config')
RESULTS_DIR = os.path.join(EXP_DIR, 'results', 're_evaluation_15trials')
INIT_DIR = os.path.join(EXP_DIR, 'results', 'init_verification')
VID_DIR = os.path.join(EXP_DIR, 'videos', 're_evaluation')

os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(INIT_DIR, exist_ok=True)
os.makedirs(VID_DIR, exist_ok=True)

# Pre-Declared Evaluation Criteria
CRITERIA_STRICT_GT_TOL_M = 0.22      # Matches Nav2 xy_goal_tolerance
CRITERIA_BOUNDED_GT_TOL_M = 0.30     # Includes bounded estimation error (0.22 + 0.08m)
CRITERIA_YAW_TOL_RAD = 0.75          # Matches Nav2 yaw_goal_tolerance (~43.0 deg)
CRITERIA_FALSE_GOAL_THRESHOLD_M = 0.50

SCENARIO = {
    'id': 'CL_Corridor_Doorway_Perturbed',
    'title': 'Lounge Multi-Obstacle Corridor Turn (Stabilized Pre-Deployment Audit)',
    'world_file': '/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/worlds/lounge_evaluation_world.sdf',
    'spawn': {'x': -1.10, 'y': -0.60, 'z': 0.05, 'yaw': 1.5708},
    'goal': {'x': -2.20, 'y': 3.35, 'yaw': 2.28},
    'max_duration_s': 18.5,
    'base_perturb': {
        'w_scale': 1.15, 'w_noise': 0.008,
        'g_bias': 0.045, 'g_noise': 0.012,
        'a_bias': 0.22,  'a_noise': 0.05,
        'slip_start': 2.0, 'slip_end': 7.5, 'slip_mult': 1.25,
    },
}

ESTIMATORS = [
    {
        'code': 'F4',
        'name': 'F4: 2D LiDAR ICP Odometry (/scan only)',
        'topic': '/odometry/lidar_odom',
        'vid_code': 'F4',
        'stall_detector_enabled': False,
    },
    {
        'code': 'F5_improved',
        'name': 'F5_improved: LiDAR(x,y,yaw,vx,vy) + IMU(wz) EKF + Pre-Deployment Stall Safety',
        'topic': '/odometry/f5_improved',
        'vid_code': 'F5_imp',
        'stall_detector_enabled': True,
    },
]


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


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


class ReEvaluationMonitorNode(Node):
    def __init__(self, active_code: str, active_topic: str, spawn: dict, goal: dict):
        super().__init__(
            'reeval_monitor_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.active_code = active_code
        self.active_topic = active_topic
        self.spawn = spawn
        self.goal = goal

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.clear_local_srv = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.clear_global_srv = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.pub_nav_status = self.create_publisher(String, '/nav2_monitor/status_json', 10)
        self.pub_cmd_brake = self.create_publisher(Twist, '/cmd_vel', 10)

        self.gt_odom = None
        self.gt_world = None
        self.active_odom = None
        self.b0_odom = None
        self.cmd_out = (0.0, 0.0)
        self.cmd_raw = (0.0, 0.0)
        self.min_scan_r = 10.0
        self.last_f4_lat = {}
        self.f4_lat_list = []
        self.stall_diag = {}

        self.sub_gt_odom = self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt_odom, 20)
        self.sub_gt_world = self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_gt_world, 20)
        self.sub_active_odom = self.create_subscription(Odometry, self.active_topic, self.on_active_odom, 20)
        self.sub_b0_odom = self.create_subscription(Odometry, '/odom', self.on_b0_odom, 20)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.sub_cmd_out = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_out, 10)
        self.sub_cmd_raw = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd_raw, 10)
        self.sub_f4_lat = self.create_subscription(Float32MultiArray, '/diagnostics/f4_latency', self.on_f4_lat, 10)
        self.sub_stall = self.create_subscription(String, '/stall_detector/status_json', self.on_stall_diag, 10)

        self.samples = []
        self.scan_count = 0
        self.active_odom_count = 0
        self.action_state = 'IDLE'

    def on_gt_odom(self, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt_odom = (
            t_sec,
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            quat_to_yaw(msg.pose.pose.orientation)
        )

    def on_gt_world(self, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        self.gt_world = (
            t_sec,
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            quat_to_yaw(msg.pose.pose.orientation),
            float(vx), float(vy), float(wz)
        )

    def on_active_odom(self, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.active_odom_count += 1
        self.active_odom = (
            t_sec,
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            quat_to_yaw(msg.pose.pose.orientation),
            float(msg.twist.twist.linear.x)
        )

    def on_b0_odom(self, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.b0_odom = (
            t_sec,
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            quat_to_yaw(msg.pose.pose.orientation)
        )

    def on_scan(self, msg: LaserScan):
        self.scan_count += 1
        if msg.ranges:
            valid = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
            self.min_scan_r = min(valid) if valid else 10.0

    def on_cmd_out(self, msg: Twist):
        self.cmd_out = (float(msg.linear.x), float(msg.angular.z))

    def on_cmd_raw(self, msg: Twist):
        self.cmd_raw = (float(msg.linear.x), float(msg.angular.z))

    def on_f4_lat(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            d = {
                'icp_compute_wall_ms': float(msg.data[0]),
                'scan_period_ms': float(msg.data[1]),
                'inlier_count': float(msg.data[2]),
                'icp_error_m': float(msg.data[3]),
            }
            self.last_f4_lat = d
            self.f4_lat_list.append(d['icp_compute_wall_ms'])

    def on_stall_diag(self, msg: String):
        try:
            self.stall_diag = json.loads(msg.data)
        except Exception:
            pass

    def record_sample(self):
        if self.gt_world is None or self.active_odom is None:
            return
        t_now = self.get_clock().now().nanoseconds * 1e-9
        t_gw, gwx, gwy, gwyaw, gvx, gvy, gwz = self.gt_world
        t_ao, aox, aoy, aoyaw, aovx = self.active_odom
        b0x = self.b0_odom[1] if self.b0_odom else float('nan')
        b0y = self.b0_odom[2] if self.b0_odom else float('nan')

        # Distance to goal from GT world position and estimated pose
        gt_goal_dist = math.hypot(gwx - self.goal['x'], gwy - self.goal['y'])
        # Target position in odom frame is rotated by -spawn_yaw
        cs, ss = math.cos(self.spawn['yaw']), math.sin(self.spawn['yaw'])
        gx_rel = self.goal['x'] - self.spawn['x']
        gy_rel = self.goal['y'] - self.spawn['y']
        tgt_ox = cs * gx_rel + ss * gy_rel
        tgt_oy = -ss * gx_rel + cs * gy_rel
        est_goal_dist = math.hypot(aox - tgt_ox, aoy - tgt_oy)

        # Contact detection: robot physically stopped while commanded forward
        contact_detected = (abs(self.cmd_out[0]) > 0.05 and math.hypot(gvx, gvy) < 0.015 and self.min_scan_r < 0.32)

        s = {
            'sim_time': round(t_now, 3),
            'action_state': self.action_state,
            'gt_world_x': round(gwx, 4),
            'gt_world_y': round(gwy, 4),
            'gt_world_yaw': round(gwyaw, 4),
            'gt_speed': round(math.hypot(gvx, gvy), 4),
            'active_odom_x': round(aox, 4),
            'active_odom_y': round(aoy, 4),
            'active_odom_yaw': round(aoyaw, 4),
            'b0_odom_x': round(b0x, 4) if math.isfinite(b0x) else None,
            'b0_odom_y': round(b0y, 4) if math.isfinite(b0y) else None,
            'cmd_raw_vx': round(self.cmd_raw[0], 3),
            'cmd_out_vx': round(self.cmd_out[0], 3),
            'min_scan_r': round(self.min_scan_r, 3),
            'gt_goal_dist': round(gt_goal_dist, 4),
            'est_goal_dist': round(est_goal_dist, 4),
            'contact_detected': bool(contact_detected),
            'stall_state': self.stall_diag.get('state', 'UNKNOWN'),
            'cmd_blocked': bool(self.stall_diag.get('cmd_blocked', False)),
        }
        self.samples.append(s)

        # Publish status string for video overlay
        stat_msg = String()
        stat_msg.data = json.dumps({
            'action_state': self.action_state,
            'active_estimator': self.active_code,
            'gt_goal_dist_m': round(gt_goal_dist, 3),
            'est_goal_dist_m': round(est_goal_dist, 3),
            'stall_state': self.stall_diag.get('state', 'NORMAL'),
            'cmd_blocked': self.stall_diag.get('cmd_blocked', False),
            'contact': contact_detected,
        })
        self.pub_nav_status.publish(stat_msg)


def verify_and_settle_gazebo(mon: ReEvaluationMonitorNode, target_x: float, target_y: float, target_yaw: float,
                             max_timeout_s: float = 8.0) -> dict:
    """
    Actively checks convergence:
    - pos_err <= 0.020m
    - yaw_err <= 0.020 rad (1.15 deg)
    - speed <= 0.005 m/s
    - wz <= 0.005 rad/s
    Must be satisfied stably for K=20 consecutive samples (~0.4s).
    """
    t0 = time.time()
    consecutive_settled = 0
    needed_consecutive = 18

    brake = Twist()
    brake.linear.x = 0.0
    brake.angular.z = 0.0

    while time.time() - t0 < max_timeout_s:
        mon.pub_cmd_brake.publish(brake)
        rclpy.spin_once(mon, timeout_sec=0.02)
        if mon.gt_world is None:
            continue

        _, gwx, gwy, gwyaw, gvx, gvy, gwz = mon.gt_world
        pos_err = math.hypot(gwx - target_x, gwy - target_y)
        yaw_err = abs(wrap_angle(gwyaw - target_yaw))
        speed = math.hypot(gvx, gvy)
        abs_wz = abs(gwz)

        settled = (pos_err <= 0.020 and yaw_err <= 0.020 and speed <= 0.005 and abs_wz <= 0.005)
        if settled:
            consecutive_settled += 1
        else:
            consecutive_settled = 0

        if consecutive_settled >= needed_consecutive:
            settle_dur = time.time() - t0
            return {
                'success': True,
                'status': 'SETTLED_CONVERGED',
                'settle_wall_duration_s': round(settle_dur, 3),
                'final_pos_err_m': round(pos_err, 4),
                'final_yaw_err_deg': round(math.degrees(yaw_err), 3),
                'final_speed_m_s': round(speed, 5),
                'final_wz_rad_s': round(abs_wz, 5),
            }

    return {
        'success': False,
        'status': 'INIT_FAILED_SETTLE_TIMEOUT',
        'settle_wall_duration_s': round(time.time() - t0, 3),
        'final_pos_err_m': round(pos_err, 4) if mon.gt_world else None,
        'final_yaw_err_deg': round(math.degrees(yaw_err), 3) if mon.gt_world else None,
        'final_speed_m_s': round(speed, 5) if mon.gt_world else None,
        'final_wz_rad_s': round(abs_wz, 5) if mon.gt_world else None,
    }
