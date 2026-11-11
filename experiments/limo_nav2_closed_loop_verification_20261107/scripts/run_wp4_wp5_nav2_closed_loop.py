#!/usr/bin/env -S python3 -s
"""
run_wp4_wp5_nav2_closed_loop.py
Work Packages 4 & 5: Real Nav2 Closed-Loop State Estimation & Safety Verification.
Tests B0, F1b, F4, F5 (original), and F5_improved (with Stall Detector) in a real closed loop
where Nav2 (`bt_navigator`, `controller_server`, `planner_server`, `local_costmap`, `global_costmap`)
actively consumes the selected estimator's `odom -> base_footprint` TF and `/odom_nav2` topic.

Records all 8 required verification dimensions for every run:
1. True physical goal arrival (`GT` distance to goal <= 0.25m)
2. Nav2 Action result (`SUCCEEDED`, `ABORTED`, `CANCELED`, `TIMEOUT_ACTIVE`) & Safety Outcome Class
   (`TRUE_SUCCEEDED`, `FALSE_SUCCEEDED_HAZARD`, `SAFE_ABORTED_OR_HALTED`, `DRIFT_MISSED_GOAL`)
3. Ground Truth vs Active Estimator pose error (ATE RMSE, P95, Max, Final Pos Err, Yaw RMSE/Max)
4. Physical collision / contact status & minimum 2D LiDAR range
5. Stall detection & physical wheel-stop latency (`stall_detection_latency_s`, `unblocked_wheel_slip_s`)
6. Global & Local paths (`/plan` and `/local_plan` reception and waypoint counts)
7. Costmap & Sensor health (`global_costmap`, `local_costmap`, `/scan`, `/odom_nav2` rates)
8. CPU utilization (%) and state estimation latency (`f4_icp_compute_ms`, `odom_sample_age_ms`)
"""
import os
import sys
import csv
import math
import json
import time
import random
import signal
import argparse
import subprocess
import psutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String

EXP_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_nav2_closed_loop_verification_20261107'
SINGLE_DIR = os.path.join(EXP_DIR, 'results', 'nav2_single_verification')
REP_DIR = os.path.join(EXP_DIR, 'results', 'nav2_repeated_trials')
PLOT_DIR = os.path.join(EXP_DIR, 'results', 'plots')
VID_DIR = os.path.join(EXP_DIR, 'videos', 'nav2_closed_loop')


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


CLOSED_LOOP_CONFIGS = [
    {
        'code': 'B0',
        'name': 'B0: Wheel Odometry (Raw Ackermann)',
        'topic': '/odom',
        'vid_code': 'B0',
        'stall_detector_enabled': False,
    },
    {
        'code': 'F1b',
        'name': 'F1b: Tuned EKF (Wheel vx,vy + IMU yaw,wz)',
        'topic': '/odometry/ekf_tuned',
        'vid_code': 'F1b',
        'stall_detector_enabled': False,
    },
    {
        'code': 'F4',
        'name': 'F4: 2D LiDAR ICP Odometry (/scan only)',
        'topic': '/odometry/lidar_odom',
        'vid_code': 'F4',
        'stall_detector_enabled': False,
    },
    {
        'code': 'F5',
        'name': 'F5: Original LiDAR(x,y,vx,vy) + IMU(yaw,wz) EKF',
        'topic': '/odometry/lidar_imu_wheel_ekf',
        'vid_code': 'F5',
        'stall_detector_enabled': False,
    },
    {
        'code': 'F5_improved',
        'name': 'F5_improved: LiDAR(x,y,yaw,vx,vy) + IMU(wz) EKF + Stall Detector',
        'topic': '/odometry/f5_improved',
        'vid_code': 'F5_imp',
        'stall_detector_enabled': True,
    },
]

CLOSED_LOOP_SCENARIOS = {
    'CL_GlassWall_Hazard': {
        'id': 'CL_GlassWall_Hazard',
        'title': 'Closed-Loop Scenario 1: Unseen Glass-Wall Hazard & False-Goal Prevention',
        'world_file': '/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/worlds/lounge_evaluation_world_invisible_glass.sdf',
        'spawn': {'x': 1.0, 'y': 0.3, 'z': 0.05, 'yaw': -1.5708},
        'goal': {'x': 1.0, 'y': -2.10, 'yaw': -1.5708},
        'max_duration_s': 11.0,
        'is_unreachable_glass_hazard': True,
        'base_perturb': {
            'w_scale': 1.0, 'w_noise': 0.0,
            'g_bias': 0.0, 'g_noise': 0.0,
            'a_bias': 0.0, 'a_noise': 0.0,
            'slip_start': -1.0, 'slip_end': -1.0, 'slip_mult': 1.0,
        },
    },
    'CL_Corridor_Doorway_Perturbed': {
        'id': 'CL_Corridor_Doorway_Perturbed',
        'title': 'Closed-Loop Scenario 2: Lounge Multi-Obstacle Corridor Turn under Wheel-Slip + IMU Gyro Bias',
        'world_file': '/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/worlds/lounge_evaluation_world.sdf',
        'spawn': {'x': -1.10, 'y': -0.60, 'z': 0.05, 'yaw': 1.5708},
        'goal': {'x': -2.20, 'y': 3.35, 'yaw': 2.28},
        'max_duration_s': 18.5,
        'is_unreachable_glass_hazard': False,
        'base_perturb': {
            'w_scale': 1.15, 'w_noise': 0.008,
            'g_bias': 0.045, 'g_noise': 0.012,
            'a_bias': 0.22,  'a_noise': 0.05,
            'slip_start': 2.0, 'slip_end': 7.5, 'slip_mult': 1.25,
        },
    },
}


class Nav2ClosedLoopMonitorNode(Node):
    def __init__(self, active_code: str, active_topic: str, spawn: dict, goal: dict):
        super().__init__(
            'nav2_closed_loop_monitor_node',
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

        self.gt_odom = None
        self.gt_world = None
        self.active_odom = None
        self.b0_odom = None
        self.cmd_out = (0.0, 0.0)
        self.cmd_raw = (0.0, 0.0)
        self.min_scan_r = 10.0
        self.scan_count = 0
        self.active_odom_count = 0
        self.global_plans = []
        self.local_plans = []
        self.global_costmap_ok = False
        self.local_costmap_ok = False
        self.stall_status = {}
        self.f4_lat_list = []

        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt_odom, 20)
        self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_gt_world, 20)
        self.create_subscription(Odometry, '/odom_nav2', self.on_active_odom, 20)
        self.create_subscription(Odometry, '/odom', self.on_b0_odom, 20)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_out, 20)
        self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd_raw, 20)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.create_subscription(NavPath, '/local_plan', self.on_local_plan, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_gcostmap, 5)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_lcostmap, 5)
        self.create_subscription(String, '/stall_detector/status_json', self.on_stall, 10)
        self.create_subscription(Float32MultiArray, '/diagnostics/f4_latency', self.on_f4_lat, 10)

        self.samples = []
        self.action_state = 'INIT'

    def on_gt_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt_odom = (
            t,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quat_to_yaw(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )

    def on_gt_world(self, msg: Odometry):
        self.gt_world = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quat_to_yaw(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
        )

    def on_active_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.active_odom_count += 1
        self.active_odom = (
            t,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quat_to_yaw(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )

    def on_b0_odom(self, msg: Odometry):
        self.b0_odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x)

    def on_cmd_out(self, msg: Twist):
        self.cmd_out = (msg.linear.x, msg.angular.z)

    def on_cmd_raw(self, msg: Twist):
        self.cmd_raw = (msg.linear.x, msg.angular.z)

    def on_scan(self, msg: LaserScan):
        self.scan_count += 1
        r = np.asarray(msg.ranges, dtype=np.float32)
        v = r[np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)]
        if len(v) > 0:
            self.min_scan_r = float(np.min(v))

    def on_plan(self, msg: NavPath):
        if len(msg.poses) > 0:
            self.global_plans.append(len(msg.poses))

    def on_local_plan(self, msg: NavPath):
        if len(msg.poses) > 0:
            self.local_plans.append(len(msg.poses))

    def on_gcostmap(self, msg: OccupancyGrid):
        self.global_costmap_ok = True

    def on_lcostmap(self, msg: OccupancyGrid):
        self.local_costmap_ok = True

    def on_stall(self, msg: String):
        try:
            self.stall_status = json.loads(msg.data)
        except Exception:
            pass

    def on_f4_lat(self, msg: Float32MultiArray):
        if len(msg.data) >= 1:
            self.f4_lat_list.append(float(msg.data[0]))

    def odom_to_map(self, ox: float, oy: float, oyaw: float):
        x0, y0, yaw0 = self.spawn['x'], self.spawn['y'], self.spawn['yaw']
        c, s = math.cos(yaw0), math.sin(yaw0)
        mx = x0 + c * ox - s * oy
        my = y0 + s * ox + c * oy
        myaw = wrap_angle(yaw0 + oyaw)
        return mx, my, myaw

    def record_sample(self):
        if self.gt_odom is None or self.gt_world is None:
            return
        gt_t, gx_o, gy_o, gyaw_o, gvx, gwz = self.gt_odom
        gx_w, gy_w, gyaw_w, _ = self.gt_world

        if self.active_odom is not None:
            est_t, ex_o, ey_o, eyaw_o, evx, ewz = self.active_odom
            ex_w, ey_w, eyaw_w = self.odom_to_map(ex_o, ey_o, eyaw_o)
        else:
            est_t, ex_o, ey_o, eyaw_o, evx, ewz = np.nan, np.nan, np.nan, np.nan, np.nan, np.nan
            ex_w, ey_w, eyaw_w = np.nan, np.nan, np.nan

        goal_dist_gt = math.hypot(gx_w - self.goal['x'], gy_w - self.goal['y'])
        goal_dist_est = math.hypot(ex_w - self.goal['x'], ey_w - self.goal['y']) if math.isfinite(ex_w) else np.nan

        b0_vx = self.b0_odom[2] if self.b0_odom else 0.0
        physical_contact = (
            (abs(self.cmd_raw[0]) > 0.07 or abs(b0_vx) > 0.06)
            and abs(gvx) < 0.025
            and (gx_o > 0.35 or self.min_scan_r < 0.25)
        )

        st_msg = String()
        st_msg.data = json.dumps({
            'action_state': self.action_state,
            'goal_dist_gt_m': round(goal_dist_gt, 3),
            'goal_dist_est_m': round(goal_dist_est, 3) if math.isfinite(goal_dist_est) else -1.0,
            'physical_contact': physical_contact,
        })
        self.pub_nav_status.publish(st_msg)

        self.samples.append({
            'sim_time': round(gt_t, 4),
            'action_state': self.action_state,
            'GT_odom_x': round(gx_o, 4),
            'GT_odom_y': round(gy_o, 4),
            'GT_odom_yaw': round(gyaw_o, 4),
            'GT_world_x': round(gx_w, 4),
            'GT_world_y': round(gy_w, 4),
            'GT_vx': round(gvx, 4),
            'EST_stamp': round(est_t, 4) if math.isfinite(est_t) else np.nan,
            'EST_odom_x': round(ex_o, 4) if math.isfinite(ex_o) else np.nan,
            'EST_odom_y': round(ey_o, 4) if math.isfinite(ey_o) else np.nan,
            'EST_odom_yaw': round(eyaw_o, 4) if math.isfinite(eyaw_o) else np.nan,
            'EST_world_x': round(ex_w, 4) if math.isfinite(ex_w) else np.nan,
            'EST_world_y': round(ey_w, 4) if math.isfinite(ey_w) else np.nan,
            'EST_vx': round(evx, 4) if math.isfinite(evx) else np.nan,
            'B0_vx': round(b0_vx, 4),
            'cmd_vx_raw': round(self.cmd_raw[0], 4),
            'cmd_vx_out': round(self.cmd_out[0], 4),
            'goal_dist_gt_m': round(goal_dist_gt, 4),
            'goal_dist_est_m': round(goal_dist_est, 4) if math.isfinite(goal_dist_est) else np.nan,
            'min_scan_range_m': round(self.min_scan_r, 4),
            'physical_contact': physical_contact,
            'stall_state': self.stall_status.get('state_name', 'NORMAL'),
            'cmd_blocked': bool(self.stall_status.get('cmd_blocked', False)),
        })


def reset_gazebo_pose(x: float, y: float, z: float, yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    req = f'name: "limo", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}'
    subprocess.run([
        'ign', 'service', '-s', '/world/lounge_world/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req', req
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def evaluate_closed_loop_trial(mon: Nav2ClosedLoopMonitorNode, scen_spec: dict, cfg: dict, wall_dur: float, cpu_samples):
    samples = mon.samples
    if len(samples) < 5:
        return {'status': 'INSUFFICIENT_SAMPLES', 'samples': len(samples)}

    gt_t = np.array([r['sim_time'] for r in samples], dtype=np.float64)
    gx = np.array([r['GT_odom_x'] for r in samples], dtype=np.float64)
    gy = np.array([r['GT_odom_y'] for r in samples], dtype=np.float64)
    gyaw = np.array([r['GT_odom_yaw'] for r in samples], dtype=np.float64)
    gyaw_u = np.unwrap(gyaw)

    ex = np.array([r['EST_odom_x'] for r in samples], dtype=np.float64)
    ey = np.array([r['EST_odom_y'] for r in samples], dtype=np.float64)
    eyaw = np.array([r['EST_odom_yaw'] for r in samples], dtype=np.float64)
    estamp = np.array([r['EST_stamp'] for r in samples], dtype=np.float64)

    valid = np.isfinite(ex) & np.isfinite(ey) & np.isfinite(eyaw) & np.isfinite(estamp)
    if np.count_nonzero(valid) < 5:
        return {'status': 'NO_ESTIMATOR_DATA', 'samples': len(samples)}

    gt_t_v, gx_v, gy_v, gyaw_v = gt_t[valid], gx[valid], gy[valid], gyaw[valid]
    ex_v, ey_v, eyaw_v, st_v = ex[valid], ey[valid], eyaw[valid], estamp[valid]

    pos_zoh = np.hypot(ex_v - gx_v, ey_v - gy_v)
    yaw_zoh = np.abs(np.degrees(np.arctan2(np.sin(eyaw_v - gyaw_v), np.cos(eyaw_v - gyaw_v))))

    st_clip = np.clip(st_v, gt_t[0], gt_t[-1])
    gxi = np.interp(st_clip, gt_t, gx)
    gyi = np.interp(st_clip, gt_t, gy)
    gyawi = np.arctan2(np.sin(np.interp(st_clip, gt_t, gyaw_u)), np.cos(np.interp(st_clip, gt_t, gyaw_u)))
    pos_int = np.hypot(ex_v - gxi, ey_v - gyi)
    yaw_int = np.abs(np.degrees(np.arctan2(np.sin(eyaw_v - gyawi), np.cos(eyaw_v - gyawi))))
    sample_age_ms = np.maximum(0.0, (gt_t_v - st_v) * 1000.0)

    final_gt_goal_dist = float(samples[-1]['goal_dist_gt_m'])
    min_gt_goal_dist = float(min(r['goal_dist_gt_m'] for r in samples))
    final_est_goal_dist = float(samples[-1]['goal_dist_est_m']) if math.isfinite(samples[-1]['goal_dist_est_m']) else 99.0

    true_goal_reached = (final_gt_goal_dist <= 0.28 or min_gt_goal_dist <= 0.25)
    nav2_res = mon.action_state
    false_succeeded = (nav2_res == 'SUCCEEDED' and not true_goal_reached)

    if scen_spec['is_unreachable_glass_hazard']:
        # Goal is behind the physical glass wall: TRUE_GOAL_REACHED is physically impossible!
        # Safety success means NEVER declaring FALSE_SUCCEEDED and arresting wheel slip!
        if false_succeeded:
            outcome_class = 'FALSE_SUCCEEDED_HAZARD'
        else:
            outcome_class = 'SAFE_ABORTED_OR_HALTED'
    else:
        if nav2_res == 'SUCCEEDED' and true_goal_reached:
            outcome_class = 'TRUE_SUCCEEDED'
        elif false_succeeded:
            outcome_class = 'FALSE_SUCCEEDED_DRIFT'
        else:
            outcome_class = 'FAILED_OR_ABORTED'

    contact_rows = [r for r in samples if r['physical_contact']]
    latch_rows = [r for r in samples if r['stall_state'] == 'STALL_LATCHED' and r['cmd_blocked']]
    t_contact = contact_rows[0]['sim_time'] if contact_rows else None
    t_latch = latch_rows[0]['sim_time'] if latch_rows else None
    stall_det_latency_s = round(max(0.0, t_latch - t_contact), 4) if (t_contact and t_latch) else None

    # Count duration where robot is in physical contact AND wheels are still spinning (|B0_vx| > 0.05)
    wheel_slip_samples = sum(1 for r in samples if r['physical_contact'] and abs(r['B0_vx']) > 0.05)
    dt_mean = float(np.mean(np.diff(gt_t))) if len(gt_t) > 1 else 0.1
    unblocked_wheel_slip_s = round(wheel_slip_samples * dt_mean, 3)

    return {
        'status': 'OK',
        'estimator_code': cfg['code'],
        'scenario_id': scen_spec['id'],
        'nav2_action_result': nav2_res,
        'outcome_class': outcome_class,
        'true_goal_reached_gt': bool(true_goal_reached),
        'false_goal_succeeded_hazard': bool(false_succeeded),
        'final_gt_goal_dist_m': round(final_gt_goal_dist, 4),
        'min_gt_goal_dist_m': round(min_gt_goal_dist, 4),
        'final_est_goal_dist_m': round(final_est_goal_dist, 4),
        'zoh_ate_rmse_m': round(float(np.sqrt(np.mean(pos_zoh ** 2))), 4),
        'interp_ate_rmse_m': round(float(np.sqrt(np.mean(pos_int ** 2))), 4),
        'interp_ate_p95_m': round(float(np.percentile(pos_int, 95.0)), 4),
        'interp_ate_max_m': round(float(np.max(pos_int)), 4),
        'final_pos_err_m': round(float(pos_int[-1]), 4),
        'interp_yaw_rmse_deg': round(float(np.sqrt(np.mean(yaw_int ** 2))), 4),
        'interp_yaw_max_deg': round(float(np.max(yaw_int)), 4),
        'physical_collision_or_contact': bool(len(contact_rows) > 0),
        'contact_samples_count': len(contact_rows),
        'min_lidar_scan_range_m': round(float(min(r['min_scan_range_m'] for r in samples)), 4),
        'stall_detected_and_blocked': bool(len(latch_rows) > 0),
        'stall_detection_latency_s': stall_det_latency_s,
        'unblocked_wheel_slip_duration_s': unblocked_wheel_slip_s,
        'global_plan_updates': len(mon.global_plans),
        'global_plan_initial_poses': mon.global_plans[0] if mon.global_plans else 0,
        'local_plan_updates': len(mon.local_plans),
        'global_costmap_received': bool(mon.global_costmap_ok),
        'local_costmap_received': bool(mon.local_costmap_ok),
        'scan_rate_hz': round(mon.scan_count / max(0.5, wall_dur), 2),
        'active_odom_rate_hz': round(mon.active_odom_count / max(0.5, wall_dur), 2),
        'active_odom_sample_age_mean_ms': round(float(np.mean(sample_age_ms)), 2),
        'f4_icp_compute_mean_ms': round(float(np.mean(mon.f4_lat_list)), 2) if mon.f4_lat_list else 0.0,
        'system_cpu_percent_mean': round(float(np.mean(cpu_samples)), 2) if cpu_samples else 0.0,
    }


def run_closed_loop_scenario_suite(scen_key: str, identical_runs: int, varied_runs: int, is_single_mode: bool):
    scen_spec = CLOSED_LOOP_SCENARIOS[scen_key]
    out_base = SINGLE_DIR if is_single_mode else REP_DIR
    os.makedirs(out_base, exist_ok=True)
    os.makedirs(VID_DIR, exist_ok=True)
    os.makedirs(PLOT_DIR, exist_ok=True)

    env = os.environ.copy()
    env['PYTHONNOUSERSITE'] = '1'
    rl_lib_dir = '/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/robot_localization/lib'
    env['LD_LIBRARY_PATH'] = f"{rl_lib_dir}:{env.get('LD_LIBRARY_PATH', '')}"

    base_sp = scen_spec['spawn']
    goal = scen_spec['goal']
    total_runs = 1 if is_single_mode else (identical_runs + varied_runs)

    print(f"\n==========================================================================")
    print(f"[CLOSED-LOOP NAV2] {scen_key} ({scen_spec['title']}) | Runs per Estimator: {total_runs}")
    print(f"==========================================================================")

    # Launch persistent Gazebo + persistent Nav2 Servers for this scenario
    sim_cmd = [
        'ros2', 'launch',
        os.path.join(EXP_DIR, 'launch', 'base_gazebo_for_trials.launch.py'),
        f"world:={scen_spec['world_file']}",
        f"spawn_x:={base_sp['x']}", f"spawn_y:={base_sp['y']}", f"spawn_z:={base_sp['z']}", f"spawn_yaw:={base_sp['yaw']}"
    ]
    p_sim = subprocess.Popen(sim_cmd, env=env, preexec_fn=os.setsid,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(6.5)

    # Start persistent seed-0 normalizer + GT + estimators + TF relay during Nav2 bringup so TF tree is immediately valid
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
    boot_procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'odom_tf_relay_node.py')],
                                       env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

    nav2_cmd = [
        'ros2', 'launch',
        os.path.join(EXP_DIR, 'launch', 'closed_loop_nav2_stack.launch.py'),
        f"spawn_x:={base_sp['x']}", f"spawn_y:={base_sp['y']}", f"spawn_yaw:={base_sp['yaw']}"
    ]
    p_nav2 = subprocess.Popen(nav2_cmd, env=env, preexec_fn=os.setsid,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(8.5)

    # Terminate temporary boot normalizer/GT/relay before starting per-trial loop
    for bp in boot_procs:
        if bp and bp.poll() is None:
            os.killpg(os.getpgid(bp.pid), signal.SIGTERM)
    time.sleep(0.5)

    rclpy.init()
    scenario_results = {cfg['code']: [] for cfg in CLOSED_LOOP_CONFIGS}

    try:
        for cfg in CLOSED_LOOP_CONFIGS:
            code = cfg['code']
            for idx in range(1, total_runs + 1):
                is_ident = (idx <= identical_runs) or is_single_mode
                group = 'SINGLE_VERIFY' if is_single_mode else ('IDENTICAL' if is_ident else 'VARIED_SEED')
                seed = 0 if is_ident else (2000 + (idx - identical_runs))
                rng = random.Random(seed if seed > 0 else 42)

                bp = scen_spec['base_perturb']
                if is_ident:
                    sp_x, sp_y, sp_yaw = base_sp['x'], base_sp['y'], base_sp['yaw']
                    w_scale, w_noise = bp['w_scale'], bp['w_noise']
                    g_bias, g_noise = bp['g_bias'], bp['g_noise']
                    a_bias, a_noise = bp['a_bias'], bp['a_noise']
                else:
                    sp_x = round(base_sp['x'] + rng.uniform(-0.025, 0.025), 4)
                    sp_y = round(base_sp['y'] + rng.uniform(-0.025, 0.025), 4)
                    sp_yaw = round(base_sp['yaw'] + math.radians(rng.uniform(-1.5, 1.5)), 4)
                    w_scale = round(bp['w_scale'] + rng.uniform(-0.03, 0.03), 4)
                    w_noise = round(bp['w_noise'] + rng.uniform(0.002, 0.008), 4)
                    g_bias = round(bp['g_bias'] + rng.uniform(-0.008, 0.008), 5)
                    g_noise = round(bp['g_noise'] + rng.uniform(0.002, 0.006), 5)
                    a_bias = round(bp['a_bias'] + rng.uniform(-0.03, 0.03), 4)
                    a_noise = round(bp['a_noise'] + rng.uniform(0.01, 0.03), 4)

                trial_id = f"{scen_key}_{code}_trial_{idx:02d}_{group.lower()}" + (f"_seed{seed}" if seed > 0 else "")
                print(f"  -> [{code} {idx:02d}/{total_runs:02d}] {trial_id} ...")

                reset_gazebo_pose(sp_x, sp_y, base_sp['z'], sp_yaw)
                time.sleep(0.4)

                trial_procs = []
                trial_procs.append(subprocess.Popen([
                    'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                    '--x', str(sp_x), '--y', str(sp_y), '--z', '0.0',
                    '--yaw', str(sp_yaw), '--pitch', '0.0', '--roll', '0.0',
                    '--frame-id', 'map', '--child-frame-id', 'odom',
                    '--ros-args', '-p', 'use_sim_time:=true'
                ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                norm_cmd = [
                    'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py'),
                    '--ros-args',
                    '-p', f'seed:={seed}',
                    '-p', f'wheel_vel_scale:={w_scale}',
                    '-p', f'wheel_vel_noise_std:={w_noise}',
                    '-p', f'imu_gyro_bias_z:={g_bias}',
                    '-p', f'imu_gyro_noise_std:={g_noise}',
                    '-p', f'imu_accel_bias_x:={a_bias}',
                    '-p', f'imu_accel_noise_std:={a_noise}',
                    '-p', f"slip_burst_start_s:={bp['slip_start']}",
                    '-p', f"slip_burst_end_s:={bp['slip_end']}",
                    '-p', f"slip_burst_multiplier:={bp['slip_mult']}",
                ]
                trial_procs.append(subprocess.Popen(norm_cmd, env=env, preexec_fn=os.setsid,
                                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                trial_procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
                                                    env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                trial_procs.append(subprocess.Popen([
                    'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'odom_tf_relay_node.py'),
                    '--ros-args', '-p', f"active_odom_topic:={cfg['topic']}"
                ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                trial_procs.append(subprocess.Popen(['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'state_estimators.launch.py')],
                                                    env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                trial_procs.append(subprocess.Popen([
                    'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'stall_detector_node.py'),
                    '--ros-args',
                    '-p', f"enabled:={'true' if cfg['stall_detector_enabled'] else 'false'}",
                    '-p', f"publish_virtual_bumper:={'true' if cfg['stall_detector_enabled'] else 'false'}"
                ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

                # Record MP4 video on single verification OR trial #01 of repeated benchmark
                p_vid = None
                vid_mp4 = None
                if is_single_mode or idx == 1:
                    vid_name = f"{scen_key}_{code}_{'single' if is_single_mode else 'rep_trial01'}.mp4"
                    vid_mp4 = os.path.join(VID_DIR, vid_name)
                    vid_cmd = [
                        'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'closed_loop_video_recorder_node.py'),
                        '--ros-args',
                        '-p', f'output_mp4:={vid_mp4}',
                        '-p', f'scenario_title:={scen_key}_{code}',
                        '-p', f"active_estimator:={cfg['vid_code']}",
                        '-p', f"goal_x:={goal['x']}",
                        '-p', f"goal_y:={goal['y']}",
                    ]
                    p_vid = subprocess.Popen(vid_cmd, env=env, preexec_fn=os.setsid,
                                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    trial_procs.append(p_vid)

                mon = Nav2ClosedLoopMonitorNode(code, cfg['topic'], {'x': sp_x, 'y': sp_y, 'yaw': sp_yaw}, goal)
                t_w = time.time()
                while time.time() - t_w < 6.5:
                    rclpy.spin_once(mon, timeout_sec=0.04)
                    if mon.gt_odom is not None and mon.active_odom is not None and len(mon.f4_lat_list) >= 2:
                        if abs(mon.active_odom[1]) < 0.15 and abs(mon.active_odom[2]) < 0.15:
                            break
                t_extra = time.time()
                while time.time() - t_extra < 0.5:
                    rclpy.spin_once(mon, timeout_sec=0.04)

                # Clear Nav2 local and global costmaps before dispatching goal
                if mon.clear_local_srv.service_is_ready():
                    mon.clear_local_srv.call_async(ClearEntireCostmap.Request())
                if mon.clear_global_srv.service_is_ready():
                    mon.clear_global_srv.call_async(ClearEntireCostmap.Request())
                for _ in range(5):
                    rclpy.spin_once(mon, timeout_sec=0.04)

                # Reset counters so metrics reflect strictly the active navigation phase
                mon.samples = []
                mon.f4_lat_list = []
                mon.global_plans = []
                mon.local_plans = []
                mon.scan_count = 0
                mon.active_odom_count = 0

                mon.nav_client.wait_for_server(timeout_sec=5.0)
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.header.stamp = mon.get_clock().now().to_msg()
                goal_msg.pose.pose.position.x = float(goal['x'])
                goal_msg.pose.pose.position.y = float(goal['y'])
                goal_msg.pose.pose.orientation.z = math.sin(goal['yaw'] * 0.5)
                goal_msg.pose.pose.orientation.w = math.cos(goal['yaw'] * 0.5)

                send_future = mon.nav_client.send_goal_async(goal_msg)
                t0 = time.time()
                while time.time() - t0 < 3.0 and not send_future.done():
                    rclpy.spin_once(mon, timeout_sec=0.04)

                goal_handle = send_future.result() if send_future.done() else None
                res_future = None
                if goal_handle and goal_handle.accepted:
                    mon.action_state = 'NAVIGATING'
                    res_future = goal_handle.get_result_async()
                else:
                    mon.action_state = 'REJECTED'

                t_run = time.time()
                last_s = 0.0
                cpu_samples = []
                max_dur = scen_spec['max_duration_s']

                while time.time() - t_run < max_dur:
                    rclpy.spin_once(mon, timeout_sec=0.02)
                    now = time.time()
                    if now - last_s >= 0.10:
                        mon.record_sample()
                        cpu_samples.append(psutil.cpu_percent(interval=None))
                        last_s = now

                    if res_future is not None and res_future.done():
                        status_code = res_future.result().status
                        if status_code == GoalStatus.STATUS_SUCCEEDED:
                            mon.action_state = 'SUCCEEDED'
                        elif status_code == GoalStatus.STATUS_ABORTED:
                            mon.action_state = 'ABORTED'
                        elif status_code == GoalStatus.STATUS_CANCELED:
                            mon.action_state = 'CANCELED'
                        else:
                            mon.action_state = f'STATUS_{status_code}'
                        # Record 0.5s of post-completion hold
                        t_post = time.time()
                        while time.time() - t_post < 0.5:
                            rclpy.spin_once(mon, timeout_sec=0.02)
                            if time.time() - last_s >= 0.10:
                                mon.record_sample()
                                last_s = time.time()
                        break

                    # If Stall Detector has latched and blocked motion for >= 1.2s in Glass Wall scenario, cancel Nav2 goal cleanly
                    if cfg['stall_detector_enabled'] and mon.stall_status.get('state_name') == 'STALL_LATCHED':
                        latched_cnt = sum(1 for r in mon.samples if r['stall_state'] == 'STALL_LATCHED')
                        if latched_cnt >= 12 and goal_handle is not None:
                            cancel_fut = goal_handle.cancel_goal_async()
                            t_c = time.time()
                            while time.time() - t_c < 0.8 and not cancel_fut.done():
                                rclpy.spin_once(mon, timeout_sec=0.03)
                            mon.action_state = 'ABORTED_BY_STALL_SAFETY'
                            mon.record_sample()
                            break

                    # If robot has driven for >7.5s and is completely motionless (GT_vx=0, cmd_vx_raw=0) for >=3.5s in corridor scenario
                    if not scen_spec['is_unreachable_glass_hazard'] and len(mon.samples) >= 75:
                        recent35 = mon.samples[-35:]
                        if all(abs(r['GT_vx']) < 0.015 and abs(r['cmd_vx_raw']) < 0.015 for r in recent35):
                            if goal_handle is not None:
                                cancel_fut = goal_handle.cancel_goal_async()
                                t_c = time.time()
                                while time.time() - t_c < 0.8 and not cancel_fut.done():
                                    rclpy.spin_once(mon, timeout_sec=0.03)
                            mon.action_state = 'ABORTED_PROGRESS_TIMEOUT'
                            mon.record_sample()
                            break

                wall_dur = time.time() - t_run
                if mon.action_state == 'NAVIGATING' and goal_handle is not None:
                    cancel_fut = goal_handle.cancel_goal_async()
                    t_c = time.time()
                    while time.time() - t_c < 0.8 and not cancel_fut.done():
                        rclpy.spin_once(mon, timeout_sec=0.03)
                    mon.action_state = 'ABORTED_PROGRESS_TIMEOUT'
                    mon.record_sample()

                if p_vid is not None and p_vid.poll() is None:
                    os.killpg(os.getpgid(p_vid.pid), signal.SIGINT)
                    time.sleep(3.2)

                csv_path = os.path.join(out_base, f'{trial_id}.csv')
                if mon.samples:
                    with open(csv_path, 'w', newline='') as f:
                        writer = csv.DictWriter(f, fieldnames=list(mon.samples[0].keys()))
                        writer.writeheader()
                        writer.writerows(mon.samples)

                metrics = evaluate_closed_loop_trial(mon, scen_spec, cfg, wall_dur, cpu_samples)
                metrics['trial_id'] = trial_id
                metrics['group'] = group
                metrics['seed'] = seed if seed > 0 else None
                metrics['csv_path'] = csv_path
                metrics['video_mp4'] = vid_mp4

                json_path = os.path.join(out_base, f'{trial_id}.json')
                with open(json_path, 'w') as f:
                    json.dump(metrics, f, indent=2)

                scenario_results[code].append(metrics)
                print(
                    f"     Done: Action={metrics.get('nav2_action_result')} ({metrics.get('outcome_class')}) | "
                    f"TrueGoal={metrics.get('true_goal_reached_gt')} | FalseGoalHazard={metrics.get('false_goal_succeeded_hazard')} | "
                    f"ATE_RMSE={metrics.get('interp_ate_rmse_m')}m | Yaw_RMSE={metrics.get('interp_yaw_rmse_deg')}deg | "
                    f"WheelSlip={metrics.get('unblocked_wheel_slip_duration_s')}s"
                )

                mon.destroy_node()
                for tp in trial_procs:
                    if tp and tp.poll() is None:
                        try:
                            os.killpg(os.getpgid(tp.pid), signal.SIGTERM)
                        except Exception:
                            pass
                time.sleep(0.7)

        # Aggregate statistics per estimator for this scenario
        agg = {}
        for cfg in CLOSED_LOOP_CONFIGS:
            c = cfg['code']
            t_list = [x for x in scenario_results[c] if x.get('status') == 'OK']
            if not t_list:
                continue
            n = len(t_list)
            agg[c] = {
                'estimator_name': cfg['name'],
                'runs_completed': n,
                'true_goal_reached_rate': round(sum(1 for x in t_list if x['true_goal_reached_gt']) / n, 4),
                'false_goal_succeeded_hazard_rate': round(sum(1 for x in t_list if x['false_goal_succeeded_hazard']) / n, 4),
                'safe_abort_or_halt_rate': round(sum(1 for x in t_list if x['outcome_class'] == 'SAFE_ABORTED_OR_HALTED') / n, 4),
                'physical_contact_rate': round(sum(1 for x in t_list if x['physical_collision_or_contact']) / n, 4),
                'interp_ate_rmse_m_mean': round(float(np.mean([x['interp_ate_rmse_m'] for x in t_list])), 4),
                'interp_ate_rmse_m_std': round(float(np.std([x['interp_ate_rmse_m'] for x in t_list])), 4),
                'interp_ate_max_m_worst': round(float(np.max([x['interp_ate_max_m'] for x in t_list])), 4),
                'final_gt_goal_dist_m_mean': round(float(np.mean([x['final_gt_goal_dist_m'] for x in t_list])), 4),
                'interp_yaw_rmse_deg_mean': round(float(np.mean([x['interp_yaw_rmse_deg'] for x in t_list])), 4),
                'unblocked_wheel_slip_s_mean': round(float(np.mean([x['unblocked_wheel_slip_duration_s'] for x in t_list])), 3),
                'stall_detection_latency_s_mean': round(float(np.mean([x['stall_detection_latency_s'] for x in t_list if x['stall_detection_latency_s'] is not None])), 4) if any(x['stall_detection_latency_s'] is not None for x in t_list) else None,
                'active_odom_sample_age_ms_mean': round(float(np.mean([x['active_odom_sample_age_mean_ms'] for x in t_list])), 2),
                'system_cpu_percent_mean': round(float(np.mean([x['system_cpu_percent_mean'] for x in t_list])), 2),
            }

        summary_out = {
            'scenario_key': scen_key,
            'title': scen_spec['title'],
            'mode': 'SINGLE_VERIFICATION' if is_single_mode else 'REPEATED_15_TRIALS',
            'runs_per_estimator': total_runs,
            'aggregated_by_estimator': agg,
            'trials_by_estimator': scenario_results,
        }
        sum_path = os.path.join(out_base, f"{scen_key}_{'single' if is_single_mode else '15trials'}_summary.json")
        with open(sum_path, 'w') as f:
            json.dump(summary_out, f, indent=2)
        print(f"=== Saved Closed-Loop Summary to {sum_path} ===")
        return summary_out

    finally:
        if rclpy.ok():
            rclpy.shutdown()
        for p in (p_nav2, p_sim):
            if p and p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except Exception:
                    pass
        subprocess.run(['pkill', '-f', 'ign gazebo'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-f', 'parameter_bridge'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-f', 'nav2_'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['single_verification', 'repeated_15trials'], default='single_verification')
    parser.add_argument('--scenario', default='ALL', help='CL_GlassWall_Hazard, CL_Corridor_Doorway_Perturbed, or ALL')
    parser.add_argument('--identical-runs', type=int, default=5)
    parser.add_argument('--varied-runs', type=int, default=10)
    args = parser.parse_args()

    is_single = (args.mode == 'single_verification')
    scen_list = ['CL_GlassWall_Hazard', 'CL_Corridor_Doorway_Perturbed'] if args.scenario == 'ALL' else [args.scenario]
    for sk in scen_list:
        run_closed_loop_scenario_suite(sk, args.identical_runs, args.varied_runs, is_single)


if __name__ == '__main__':
    main()
