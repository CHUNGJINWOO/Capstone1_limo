#!/usr/bin/env -S python3 -s
"""
run_15trial_benchmark.py
Executes the 15-Trial Repeated Benchmark (5 Identical + 10 Varied Seed) per scenario (S1..S5)
in real Gazebo Fortress physics.
- Evaluates all 8 odometry/fusion tracks simultaneously on the exact same sensor stream:
  B0, F1a, F1b, F2, F3a, F3b, F4, F5 (plus F6 SLAM Toolbox map->base_footprint in S5).
- Uses timestamp-aligned interpolation against Ground Truth (/ground_truth/odom) so asynchronous
  publication rates never introduce artificial phase lag.
- Computes Mean, Std, Median, 95th Percentile, and Worst (Max) errors across the 15 trials.
- Records representative H.264 MP4 videos for each scenario in `videos/state_estimation/`.
"""
import os
import sys
import csv
import math
import json
import time
import random
import signal
import shutil
import argparse
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener


BENCH_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107'
RAW_DIR = os.path.join(BENCH_DIR, 'results', 'raw_trials')
SUM_DIR = os.path.join(BENCH_DIR, 'results', 'summaries')
PLOT_DIR = os.path.join(BENCH_DIR, 'results', 'plots')
VID_DIR = os.path.join(BENCH_DIR, 'videos', 'state_estimation')


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


ESTIMATORS = [
    ('B0',  '/odom',                         'Wheel Odometry (Raw Ackermann)'),
    ('F1a', '/odometry/ekf_legacy',          'EKF Legacy (limo_ekf.yaml repro)'),
    ('F1b', '/odometry/ekf_tuned',           'EKF Tuned (Wheel vx,vy + IMU yaw,wz)'),
    ('F2',  '/odometry/ukf_tuned',           'UKF Tuned (Wheel vx,vy + IMU yaw,wz)'),
    ('F3a', '/odometry/comp_filter',         'Complementary Filter Standalone'),
    ('F3b', '/odometry/comp_slip_gate',      'Complementary Filter + Slip/Stall Gate'),
    ('F4',  '/odometry/lidar_odom',          '2D LiDAR Scan Matching Odometry (ICP)'),
    ('F5',  '/odometry/lidar_imu_wheel_ekf', 'LiDAR Odom (x,y,vx,vy) + IMU (yaw,wz) EKF'),
]


SCENARIO_SPECS = {
    'S1': {
        'id': 'S1_Normal_MultiTurn',
        'title': 'S1: Normal Driving & Multi-Turn S-Curve',
        'synthetic_fault': False,
        'base_spawn': {'x': -0.8, 'y': -0.3, 'z': 0.05, 'yaw': 0.0},
        'schedule': [
            (0.8, 0.00,  0.00, 'hold_start'),
            (3.0, 0.26,  0.00, 'straight_1'),
            (2.8, 0.22,  0.34, 'left_curve'),
            (2.8, 0.22, -0.34, 'right_align'),
            (2.5, 0.22, -0.30, 'right_curve'),
            (1.5, 0.00,  0.00, 'hold_stop'),
        ],
    },
    'S2': {
        'id': 'S2_WheelSlip_PhysicalStall',
        'title': 'S2: Glass Wall Physical Stall & Wheel-Slip Recovery',
        'synthetic_fault': False,
        'base_spawn': {'x': 1.0, 'y': 0.3, 'z': 0.05, 'yaw': -1.5708},
        'false_goal_dist_thresh_m': 2.20,  # True wall is at 1.645m forward; >=2.20m means false odom advancement
        'schedule': [
            (0.8,  0.00, 0.00, 'hold_start'),
            (7.5,  0.30, 0.00, 'drive_and_stall_at_wall'),
            (2.2, -0.22, 0.00, 'reverse_from_wall'),
            (2.0,  0.22, 0.42, 'turn_away'),
            (1.2,  0.00, 0.00, 'hold_stop'),
        ],
    },
    'S3': {
        'id': 'S3_SharpTurns_NarrowPassage',
        'title': 'S3: High-Curvature Sharp Ackermann Turns & Narrow Aisle',
        'synthetic_fault': False,
        'base_spawn': {'x': -0.9, 'y': -0.3, 'z': 0.05, 'yaw': 0.0},
        'schedule': [
            (0.8, 0.00,  0.00, 'hold_start'),
            (2.6, 0.25,  0.50, 'sharp_left_1'),
            (2.6, 0.25, -0.50, 'sharp_right_1'),
            (2.6, 0.25, -0.48, 'sharp_right_2'),
            (2.6, 0.25,  0.48, 'sharp_left_2'),
            (2.0, 0.26,  0.00, 'exit_straight'),
            (1.2, 0.00,  0.00, 'hold_stop'),
        ],
    },
    'S4': {
        'id': 'S4_IMUBias_SensorNoise_Synthetic',
        'title': 'S4 [SYNTHETIC FAULT]: IMU Gyro/Accel Bias + High Noise',
        'synthetic_fault': True,
        'base_spawn': {'x': -0.8, 'y': -0.3, 'z': 0.05, 'yaw': 0.0},
        'base_fault': {
            'imu_gyro_bias_z': 0.035,     # +2.0 deg/s sustained gyro bias
            'imu_gyro_noise_std': 0.015,
            'imu_accel_bias_x': 0.28,     # +0.28 m/s^2 accelerometer bias (exposes F1a accel integration)
            'imu_accel_noise_std': 0.08,
        },
        'schedule': [
            (1.0, 0.00,  0.00, 'hold_start'),
            (3.0, 0.25,  0.00, 'straight_1'),
            (2.8, 0.22,  0.34, 'left_curve'),
            (2.8, 0.22, -0.34, 'right_align'),
            (3.0, 0.00,  0.00, 'stationary_bias_drift_hold'),
        ],
    },
    'S5': {
        'id': 'S5_LoopClosure_SLAM',
        'title': 'S5: Closed-Loop Return Trajectory & F6 SLAM Toolbox PGO',
        'synthetic_fault': False,
        'enable_f6_slam': True,
        'base_spawn': {'x': -0.6, 'y': -0.4, 'z': 0.05, 'yaw': 0.0},
        'schedule': [
            (0.8,  0.00,  0.00, 'hold_start'),
            (3.5,  0.26,  0.00, 'outbound_straight'),
            (2.5,  0.22,  0.38, 'outbound_turn'),
            (2.5, -0.22, -0.38, 'return_realign'),
            (3.5, -0.26,  0.00, 'return_to_origin'),
            (1.5,  0.00,  0.00, 'loop_closure_hold'),
        ],
    },
}


class TrialCollectorNode(Node):
    def __init__(self, enable_f6: bool = False, init_spawn=None):
        super().__init__(
            'trial_collector_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.enable_f6 = enable_f6
        self.init_spawn = init_spawn or {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.gt_msg = None
        self.est_msgs = {code: None for code, _, _ in ESTIMATORS}

        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 20)
        for code, topic, _ in ESTIMATORS:
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, c=code: self.on_est(c, msg),
                20
            )

        self.samples = []
        self.f6_init_map_pose = None

    def on_gt(self, msg: Odometry):
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt_msg = (
            t_s,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quat_to_yaw(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )

    def on_est(self, code: str, msg: Odometry):
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.est_msgs[code] = (
            t_s,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quat_to_yaw(msg.pose.pose.orientation),
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        )

    def send_cmd(self, vx: float, wz: float):
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        self.cmd_pub.publish(m)

    def record_sample(self, phase: str):
        if self.gt_msg is None:
            return
        row = {
            'sim_time': round(self.gt_msg[0], 4),
            'phase': phase,
            'GT_x': self.gt_msg[1],
            'GT_y': self.gt_msg[2],
            'GT_yaw': self.gt_msg[3],
            'GT_vx': self.gt_msg[4],
            'GT_wz': self.gt_msg[5],
        }
        for code, _, _ in ESTIMATORS:
            em = self.est_msgs.get(code)
            if em is not None:
                row[f'{code}_stamp'] = round(em[0], 4)
                row[f'{code}_x'] = em[1]
                row[f'{code}_y'] = em[2]
                row[f'{code}_yaw'] = em[3]
            else:
                row[f'{code}_stamp'] = np.nan
                row[f'{code}_x'] = np.nan
                row[f'{code}_y'] = np.nan
                row[f'{code}_yaw'] = np.nan

        if self.enable_f6:
            try:
                if self.tf_buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
                    t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                    mx = t.transform.translation.x
                    my = t.transform.translation.y
                    myaw = quat_to_yaw(t.transform.rotation)
                    if self.f6_init_map_pose is None:
                        self.f6_init_map_pose = (mx, my, myaw)
                    x0, y0, yaw0 = self.f6_init_map_pose
                    dx, dy = mx - x0, my - y0
                    c0, s0 = math.cos(yaw0), math.sin(yaw0)
                    row['F6_x'] = c0 * dx + s0 * dy
                    row['F6_y'] = -s0 * dx + c0 * dy
                    row['F6_yaw'] = wrap_angle(myaw - yaw0)
                else:
                    row['F6_x'] = np.nan
                    row['F6_y'] = np.nan
                    row['F6_yaw'] = np.nan
            except Exception:
                row['F6_x'] = np.nan
                row['F6_y'] = np.nan
                row['F6_yaw'] = np.nan

        self.samples.append(row)


def reset_gazebo_pose(x: float, y: float, z: float, yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    req = f'name: "limo", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}'
    cmd = [
        'ign', 'service', '-s', '/world/lounge_world/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req', req
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def evaluate_trial_metrics(samples, code: str, false_goal_thresh=None):
    if len(samples) < 5:
        return {'status': 'FAILED_INSUFFICIENT_SAMPLES', 'valid_samples': len(samples)}

    gt_t = np.array([r['sim_time'] for r in samples], dtype=np.float64)
    gt_x = np.array([r['GT_x'] for r in samples], dtype=np.float64)
    gt_y = np.array([r['GT_y'] for r in samples], dtype=np.float64)
    gt_yaw = np.array([r['GT_yaw'] for r in samples], dtype=np.float64)

    ex = np.array([r.get(f'{code}_x', np.nan) for r in samples], dtype=np.float64)
    ey = np.array([r.get(f'{code}_y', np.nan) for r in samples], dtype=np.float64)
    eyaw = np.array([r.get(f'{code}_yaw', np.nan) for r in samples], dtype=np.float64)

    valid = np.isfinite(ex) & np.isfinite(ey) & np.isfinite(eyaw)
    if np.count_nonzero(valid) < 5:
        return {'status': 'NO_DATA', 'valid_samples': int(np.count_nonzero(valid))}

    gt_t, gt_x, gt_y, gt_yaw = gt_t[valid], gt_x[valid], gt_y[valid], gt_yaw[valid]
    ex, ey, eyaw = ex[valid], ey[valid], eyaw[valid]

    # Verify initial origin alignment at first valid sample
    init_align_err_m = float(math.hypot(ex[0] - gt_x[0], ey[0] - gt_y[0]))

    pos_err = np.hypot(ex - gt_x, ey - gt_y)
    yaw_err_deg = np.abs(np.degrees(np.arctan2(np.sin(eyaw - gt_yaw), np.cos(eyaw - gt_yaw))))

    step = min(10, max(1, len(gt_t) // 5))
    rpe_t, rpe_r = [], []
    for i in range(len(gt_t) - step):
        dx_g, dy_g = gt_x[i + step] - gt_x[i], gt_y[i + step] - gt_y[i]
        cg, sg = math.cos(gt_yaw[i]), math.sin(gt_yaw[i])
        rx_g, ry_g = cg * dx_g + sg * dy_g, -sg * dx_g + cg * dy_g
        ryaw_g = wrap_angle(gt_yaw[i + step] - gt_yaw[i])

        dx_e, dy_e = ex[i + step] - ex[i], ey[i + step] - ey[i]
        ce, se = math.cos(eyaw[i]), math.sin(eyaw[i])
        rx_e, ry_e = ce * dx_e + se * dy_e, -se * dx_e + ce * dy_e
        ryaw_e = wrap_angle(eyaw[i + step] - eyaw[i])

        rpe_t.append(math.hypot(rx_e - rx_g, ry_e - ry_g))
        rpe_r.append(abs(math.degrees(wrap_angle(ryaw_e - ryaw_g))))

    rpe_t = np.array(rpe_t) if len(rpe_t) > 0 else np.array([0.0])
    rpe_r = np.array(rpe_r) if len(rpe_r) > 0 else np.array([0.0])

    max_forward_x = float(np.max(ex))
    res = {
        'status': 'OK',
        'valid_samples': int(len(pos_err)),
        'init_alignment_err_m': round(init_align_err_m, 4),
        'ate_rmse_m': round(float(np.sqrt(np.mean(pos_err ** 2))), 4),
        'ate_mean_m': round(float(np.mean(pos_err)), 4),
        'ate_median_m': round(float(np.median(pos_err)), 4),
        'ate_p95_m': round(float(np.percentile(pos_err, 95.0)), 4),
        'ate_max_m': round(float(np.max(pos_err)), 4),
        'final_pos_err_m': round(float(pos_err[-1]), 4),
        'rpe_trans_rmse_m': round(float(np.sqrt(np.mean(rpe_t ** 2))), 4),
        'rpe_rot_rmse_deg': round(float(np.sqrt(np.mean(rpe_r ** 2))), 4),
        'yaw_rmse_deg': round(float(np.sqrt(np.mean(yaw_err_deg ** 2))), 4),
        'yaw_max_deg': round(float(np.max(yaw_err_deg)), 4),
        'final_yaw_err_deg': round(float(yaw_err_deg[-1]), 4),
        'max_forward_x_m': round(max_forward_x, 4),
    }
    if false_goal_thresh is not None:
        res['false_goal_triggered'] = bool(max_forward_x >= false_goal_thresh)
    return res


def aggregate_15trial_stats(trial_results, codes):
    summary = {}
    metric_keys = [
        'ate_rmse_m', 'ate_mean_m', 'ate_median_m', 'ate_p95_m', 'ate_max_m',
        'final_pos_err_m', 'rpe_trans_rmse_m', 'rpe_rot_rmse_deg', 'yaw_rmse_deg', 'yaw_max_deg'
    ]
    for code in codes:
        valid_trials = [t['metrics'][code] for t in trial_results if code in t['metrics'] and t['metrics'][code].get('status') == 'OK']
        ident_trials = [t['metrics'][code] for t in trial_results if t['group'] == 'IDENTICAL' and code in t['metrics'] and t['metrics'][code].get('status') == 'OK']
        varied_trials = [t['metrics'][code] for t in trial_results if t['group'] == 'VARIED_SEED' and code in t['metrics'] and t['metrics'][code].get('status') == 'OK']

        def calc_group_stats(t_list):
            if len(t_list) == 0:
                return {'runs_completed': 0}
            out = {'runs_completed': len(t_list)}
            for mk in metric_keys:
                vals = np.array([x[mk] for x in t_list], dtype=np.float64)
                out[mk] = {
                    'mean': round(float(np.mean(vals)), 4),
                    'std': round(float(np.std(vals)), 4),
                    'median': round(float(np.median(vals)), 4),
                    'p95': round(float(np.percentile(vals, 95.0)), 4),
                    'worst_max': round(float(np.max(vals)), 4),
                    'best_min': round(float(np.min(vals)), 4),
                }
            if 'false_goal_triggered' in t_list[0]:
                fg_cnt = sum(1 for x in t_list if x.get('false_goal_triggered', False))
                out['false_goal_count'] = fg_cnt
                out['false_goal_rate'] = round(fg_cnt / len(t_list), 4)
            return out

        summary[code] = {
            'all_15_trials': calc_group_stats(valid_trials),
            'identical_5_trials': calc_group_stats(ident_trials),
            'varied_10_trials': calc_group_stats(varied_trials),
            'failed_or_missing_runs': len(trial_results) - len(valid_trials),
        }
    return summary


def plot_scenario_15trials(scenario_key: str, spec: dict, trial_results, agg_stats, codes):
    os.makedirs(PLOT_DIR, exist_ok=True)
    out_png = os.path.join(PLOT_DIR, f'{scenario_key}_15trials_summary.png')

    fig, axes = plt.subplots(1, 3, figsize=(18.5, 5.8), dpi=140)
    colors = {
        'B0': '#e63946', 'F1a': '#7f8c8d', 'F1b': '#00b4d8', 'F2': '#f77f00',
        'F3a': '#9b5de5', 'F3b': '#ffbe0b', 'F4': '#3a86ff', 'F5': '#ff006e', 'F6': '#2ec4b6'
    }

    # Panel 1: Representative Trial #1 Trajectory (GT vs Estimators)
    ax0 = axes[0]
    t0_csv = trial_results[0]['csv_path']
    rows = []
    with open(t0_csv, 'r') as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    if len(rows) > 2:
        gx = [float(r['GT_x']) for r in rows]
        gy = [float(r['GT_y']) for r in rows]
        ax0.plot(gx, gy, color='#2ecc71', linewidth=3.2, label='Ground Truth (GT)')
        for c in codes:
            if f'{c}_x' in rows[0]:
                cx = [float(r[f'{c}_x']) for r in rows if r[f'{c}_x'] != 'nan']
                cy = [float(r[f'{c}_y']) for r in rows if r[f'{c}_y'] != 'nan']
                if len(cx) > 2:
                    ls = '--' if c in ('B0', 'F1a') else '-'
                    ax0.plot(cx, cy, linestyle=ls, linewidth=1.6, color=colors.get(c, '#333333'), label=c)
    ax0.set_title(f"{scenario_key}: Trajectory vs GT (Rep. Trial #01)")
    ax0.set_xlabel('X in Odom Frame (m)')
    ax0.set_ylabel('Y in Odom Frame (m)')
    ax0.axis('equal')
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc='best', fontsize=8)

    # Panel 2: Boxplot + Scatter of ATE RMSE (m) across all 15 Trials
    ax1 = axes[1]
    box_data = []
    labels = []
    for c in codes:
        vals = [t['metrics'][c]['ate_rmse_m'] for t in trial_results if c in t['metrics'] and t['metrics'][c].get('status') == 'OK']
        if len(vals) > 0:
            box_data.append(vals)
            labels.append(c)
    bp = ax1.boxplot(box_data, labels=labels, patch_artist=True, showmeans=True)
    for patch, c in zip(bp['boxes'], labels):
        patch.set_facecolor(colors.get(c, '#cccccc'))
        patch.set_alpha(0.65)
    ax1.set_title(f"{scenario_key}: ATE RMSE Distribution (n=15 Trials: 5 Identical + 10 Varied)")
    ax1.set_ylabel('ATE RMSE (m)')
    ax1.grid(True, axis='y', alpha=0.3)

    # Panel 3: Bar Chart of Mean + 95th Percentile + Worst ATE Max (m) or Yaw RMSE (deg)
    ax2 = axes[2]
    x_pos = np.arange(len(labels))
    means = [agg_stats[c]['all_15_trials']['ate_rmse_m']['mean'] for c in labels]
    p95s = [agg_stats[c]['all_15_trials']['ate_rmse_m']['p95'] for c in labels]
    worsts = [agg_stats[c]['all_15_trials']['ate_max_m']['worst_max'] for c in labels]

    w = 0.26
    ax2.bar(x_pos - w, means, width=w, label='15-Trial Mean ATE RMSE (m)', color='#2b2d42')
    ax2.bar(x_pos, p95s, width=w, label='95th Percentile ATE RMSE (m)', color='#8d99ae')
    ax2.bar(x_pos + w, worsts, width=w, label='Worst Instantaneous Error (m)', color='#ef233c')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(labels)
    ax2.set_title(f"{scenario_key}: Mean vs 95th Percentile vs Worst Position Error")
    ax2.set_ylabel('Position Error (m)')
    ax2.grid(True, axis='y', alpha=0.3)
    ax2.legend(loc='best', fontsize=8)

    plt.tight_layout()
    plt.savefig(out_png)
    plt.close(fig)
    return out_png


def run_scenario_15trials(scenario_key: str, identical_runs: int = 5, varied_runs: int = 10):
    spec = SCENARIO_SPECS[scenario_key]
    os.makedirs(RAW_DIR, exist_ok=True)
    os.makedirs(SUM_DIR, exist_ok=True)
    os.makedirs(VID_DIR, exist_ok=True)

    env = os.environ.copy()
    env['PYTHONNOUSERSITE'] = '1'
    rl_lib_dir = '/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/robot_localization/lib'
    env['LD_LIBRARY_PATH'] = f"{rl_lib_dir}:{env.get('LD_LIBRARY_PATH', '')}"

    base_sp = spec['base_spawn']
    print(f"\n==================================================================")
    print(f"Starting {scenario_key} ({spec['title']}) - {identical_runs} Identical + {varied_runs} Varied = {identical_runs + varied_runs} Trials")
    print(f"==================================================================")

    # Start persistent Gazebo simulation for this scenario
    sim_cmd = [
        'ros2', 'launch',
        os.path.join(BENCH_DIR, 'launch', 'base_gazebo_for_trials.launch.py'),
        f"spawn_x:={base_sp['x']}", f"spawn_y:={base_sp['y']}", f"spawn_z:={base_sp['z']}", f"spawn_yaw:={base_sp['yaw']}"
    ]
    p_sim = subprocess.Popen(sim_cmd, env=env, preexec_fn=os.setsid,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(7.0)

    rclpy.init()
    trial_results = []
    enable_f6 = bool(spec.get('enable_f6_slam', False))
    codes = [c for c, _, _ in ESTIMATORS] + (['F6'] if enable_f6 else [])

    try:
        total_runs = identical_runs + varied_runs
        for idx in range(1, total_runs + 1):
            is_identical = (idx <= identical_runs)
            group = 'IDENTICAL' if is_identical else 'VARIED_SEED'
            seed = 0 if is_identical else (1000 + (idx - identical_runs))
            rng = random.Random(seed if seed > 0 else 42)

            if is_identical:
                sp_x = base_sp['x']
                sp_y = base_sp['y']
                sp_yaw = base_sp['yaw']
                w_scale = 1.0
                w_noise = 0.0
                g_bias = spec.get('base_fault', {}).get('imu_gyro_bias_z', 0.0)
                g_noise = spec.get('base_fault', {}).get('imu_gyro_noise_std', 0.0)
                a_bias = spec.get('base_fault', {}).get('imu_accel_bias_x', 0.0)
                a_noise = spec.get('base_fault', {}).get('imu_accel_noise_std', 0.0)
            else:
                sp_x = round(base_sp['x'] + rng.uniform(-0.03, 0.03), 4)
                sp_y = round(base_sp['y'] + rng.uniform(-0.03, 0.03), 4)
                sp_yaw = round(base_sp['yaw'] + math.radians(rng.uniform(-2.0, 2.0)), 4)
                w_scale = round(rng.uniform(0.97, 1.03), 4)
                w_noise = round(rng.uniform(0.004, 0.012), 4)
                base_gb = spec.get('base_fault', {}).get('imu_gyro_bias_z', 0.0)
                base_gn = spec.get('base_fault', {}).get('imu_gyro_noise_std', 0.0)
                base_ab = spec.get('base_fault', {}).get('imu_accel_bias_x', 0.0)
                base_an = spec.get('base_fault', {}).get('imu_accel_noise_std', 0.0)
                g_bias = round(base_gb + rng.uniform(-0.006, 0.006), 5)
                g_noise = round(base_gn + rng.uniform(0.002, 0.006), 5)
                a_bias = round(base_ab + rng.uniform(-0.04, 0.04), 4)
                a_noise = round(base_an + rng.uniform(0.01, 0.04), 4)

            trial_id = f"{scenario_key}_trial_{idx:02d}_{group.lower()}" + (f"_seed{seed}" if seed > 0 else "")
            print(f"  -> [{idx:02d}/{total_runs:02d}] {trial_id} | spawn=({sp_x:.3f}, {sp_y:.3f}, {sp_yaw:.3f}) ...")

            # Reset Gazebo model pose & let physics settle
            reset_gazebo_pose(sp_x, sp_y, base_sp['z'], sp_yaw)
            time.sleep(0.4)

            # Start per-trial normalizer, GT publisher, TF relay, and state estimators
            trial_procs = []
            norm_cmd = [
                'python3', '-s', os.path.join(BENCH_DIR, 'nodes', 'trial_sensor_normalizer_node.py'),
                '--ros-args',
                '-p', f'seed:={seed}',
                '-p', f'wheel_vel_scale:={w_scale}',
                '-p', f'wheel_vel_noise_std:={w_noise}',
                '-p', f'imu_gyro_bias_z:={g_bias}',
                '-p', f'imu_gyro_noise_std:={g_noise}',
                '-p', f'imu_accel_bias_x:={a_bias}',
                '-p', f'imu_accel_noise_std:={a_noise}',
            ]
            p_norm = subprocess.Popen(norm_cmd, env=env, preexec_fn=os.setsid,
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            trial_procs.append(p_norm)

            gt_cmd = ['python3', '-s', os.path.join(BENCH_DIR, 'nodes', 'ground_truth_publisher_node.py')]
            p_gt = subprocess.Popen(gt_cmd, env=env, preexec_fn=os.setsid,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            trial_procs.append(p_gt)

            tf_cmd = ['python3', '-s', os.path.join(BENCH_DIR, 'nodes', 'odom_tf_relay_node.py')]
            p_tf = subprocess.Popen(tf_cmd, env=env, preexec_fn=os.setsid,
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            trial_procs.append(p_tf)

            est_cmd = ['ros2', 'launch', os.path.join(BENCH_DIR, 'launch', 'state_estimators.launch.py')]
            p_est = subprocess.Popen(est_cmd, env=env, preexec_fn=os.setsid,
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            trial_procs.append(p_est)

            if enable_f6:
                slam_cmd = [
                    'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                    'use_sim_time:=true',
                    f"slam_params_file:={os.path.join(BENCH_DIR, 'config', 'slam_benchmark_params.yaml')}"
                ]
                p_slam = subprocess.Popen(slam_cmd, env=env, preexec_fn=os.setsid,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                trial_procs.append(p_slam)

            # Record MP4 video on Trial #01 (Identical Representative) and Trial #06 (Varied Seed 1001 Representative)
            p_vid = None
            vid_mp4 = None
            if idx in (1, identical_runs + 1):
                vid_mp4 = os.path.join(VID_DIR, f'{trial_id}.mp4')
                vid_cmd = [
                    'python3', '-s', os.path.join(BENCH_DIR, 'nodes', 'sim_video_recorder_node.py'),
                    '--ros-args',
                    '-p', f'output_mp4:={vid_mp4}',
                    '-p', f'scenario_title:={trial_id}'
                ]
                p_vid = subprocess.Popen(vid_cmd, env=env, preexec_fn=os.setsid,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                trial_procs.append(p_vid)

            collector = TrialCollectorNode(enable_f6=enable_f6, init_spawn={'x': sp_x, 'y': sp_y, 'yaw': sp_yaw})
            t_warm = time.time()
            while time.time() - t_warm < 1.5:
                rclpy.spin_once(collector, timeout_sec=0.04)

            for dur, vx, wz, phase in spec['schedule']:
                t_seg = time.time()
                last_s = 0.0
                while time.time() - t_seg < dur:
                    collector.send_cmd(vx, wz)
                    rclpy.spin_once(collector, timeout_sec=0.02)
                    now = time.time()
                    if now - last_s >= 0.10:
                        collector.record_sample(phase)
                        last_s = now

            collector.send_cmd(0.0, 0.0)

            if p_vid is not None and p_vid.poll() is None:
                os.killpg(os.getpgid(p_vid.pid), signal.SIGINT)
                time.sleep(3.0)

            # Save raw trial CSV
            csv_path = os.path.join(RAW_DIR, f'{trial_id}.csv')
            if len(collector.samples) > 0:
                with open(csv_path, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=list(collector.samples[0].keys()))
                    writer.writeheader()
                    writer.writerows(collector.samples)

            # Compute per-estimator metrics for this trial
            fg_thresh = spec.get('false_goal_dist_thresh_m', None)
            trial_metrics = {}
            for c in codes:
                trial_metrics[c] = evaluate_trial_metrics(collector.samples, c, false_goal_thresh=fg_thresh)

            trial_meta = {
                'trial_index': idx,
                'trial_id': trial_id,
                'scenario_key': scenario_key,
                'group': group,
                'seed': seed if seed > 0 else None,
                'synthetic_fault_injection': bool(spec.get('synthetic_fault', False)),
                'spawn_pose': {'x': sp_x, 'y': sp_y, 'z': base_sp['z'], 'yaw': sp_yaw},
                'perturbation_params': {
                    'wheel_vel_scale': w_scale,
                    'wheel_vel_noise_std': w_noise,
                    'imu_gyro_bias_z': g_bias,
                    'imu_gyro_noise_std': g_noise,
                    'imu_accel_bias_x': a_bias,
                    'imu_accel_noise_std': a_noise,
                },
                'samples_count': len(collector.samples),
                'csv_path': csv_path,
                'video_mp4': vid_mp4,
                'metrics': trial_metrics,
            }
            json_path = os.path.join(RAW_DIR, f'{trial_id}.json')
            with open(json_path, 'w') as f:
                json.dump(trial_meta, f, indent=2)

            trial_results.append(trial_meta)
            b0_rmse = trial_metrics['B0'].get('ate_rmse_m')
            f1b_rmse = trial_metrics['F1b'].get('ate_rmse_m')
            f3b_rmse = trial_metrics['F3b'].get('ate_rmse_m')
            f4_rmse = trial_metrics['F4'].get('ate_rmse_m')
            f5_rmse = trial_metrics['F5'].get('ate_rmse_m')
            print(f"     Done: samples={len(collector.samples)} | ATE_RMSE(m): B0={b0_rmse}, F1b={f1b_rmse}, F3b={f3b_rmse}, F4={f4_rmse}, F5={f5_rmse}")

            collector.destroy_node()

            # Clean up per-trial processes
            for p in trial_procs:
                if p and p.poll() is None:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                    except Exception:
                        pass
            time.sleep(0.8)

        # Aggregate 15-trial statistics and plot
        agg_stats = aggregate_15trial_stats(trial_results, codes)
        plot_png = plot_scenario_15trials(scenario_key, spec, trial_results, agg_stats, codes)

        scen_summary = {
            'scenario_key': scenario_key,
            'scenario_id': spec['id'],
            'title': spec['title'],
            'synthetic_fault_injection': bool(spec.get('synthetic_fault', False)),
            'identical_runs': identical_runs,
            'varied_runs': varied_runs,
            'total_runs': total_runs,
            'summary_plot_png': plot_png,
            'aggregated_statistics': agg_stats,
            'trials': trial_results,
        }
        sum_json = os.path.join(SUM_DIR, f'{scenario_key}_15trials_summary.json')
        with open(sum_json, 'w') as f:
            json.dump(scen_summary, f, indent=2)
        print(f"=== {scenario_key} 15-TRIAL SUMMARY SAVED TO {sum_json} ===")
        return scen_summary

    finally:
        if rclpy.ok():
            rclpy.shutdown()
        if p_sim and p_sim.poll() is None:
            try:
                os.killpg(os.getpgid(p_sim.pid), signal.SIGTERM)
            except Exception:
                pass
        subprocess.run(['pkill', '-f', 'ign gazebo'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-f', 'parameter_bridge'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.5)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='S1', help='S1, S2, S3, S4, S5, or ALL')
    parser.add_argument('--identical-runs', type=int, default=5)
    parser.add_argument('--varied-runs', type=int, default=10)
    args = parser.parse_args()

    os.chmod(os.path.join(BENCH_DIR, 'nodes', 'trial_sensor_normalizer_node.py'), 0o755)

    if args.scenario == 'ALL':
        for sk in ['S1', 'S2', 'S3', 'S4', 'S5']:
            run_scenario_15trials(sk, args.identical_runs, args.varied_runs)
    else:
        run_scenario_15trials(args.scenario, args.identical_runs, args.varied_runs)


if __name__ == '__main__':
    main()
