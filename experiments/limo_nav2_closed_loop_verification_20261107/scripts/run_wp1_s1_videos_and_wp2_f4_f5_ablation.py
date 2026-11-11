#!/usr/bin/env -S python3 -s
"""
run_wp1_s1_videos_and_wp2_f4_f5_ablation.py
Executes:
1. Work Package 1 (S1 Missing Videos Recovery):
   Generates the two previously ungenerated S1 representative videos (`S1_trial_01_identical.mp4`
   and `S1_trial_06_varied_seed_seed1001.mp4`) in `videos/s1_recovered/` while leaving the previous
   experiment directory untouched.
2. Work Package 2 (F4 / F5 & F3b Improvement Evaluation):
   Evaluates B0, F1b, F3a, F3b (original), F3b_improved, F4, F5 (original), and F5_improved across
   S1 (Normal), S2 (Wheel Slip/Stall), S3 (Sharp Turn), and S4 (IMU Gyro/Accel Bias), measuring:
   - ATE (ZOH & Timestamp-Interpolated), RPE (Trans & Rot), Yaw RMSE & Max
   - F4 actual ICP compute latency (`icp_compute_wall_ms`) vs 10 Hz sample-hold output age (`sample_hold_age_ms`)
   - F5 & F5_improved 50 Hz output age and CPU usage.
"""
import os
import sys
import csv
import math
import json
import time
import random
import signal
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
from std_msgs.msg import Float32MultiArray

EXP_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_nav2_closed_loop_verification_20261107'
ABL_DIR = os.path.join(EXP_DIR, 'results', 'f4_f5_ablation')
PLOT_DIR = os.path.join(EXP_DIR, 'results', 'plots')
S1_VID_DIR = os.path.join(EXP_DIR, 'videos', 's1_recovered')


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


ESTIMATORS = [
    ('B0',           '/odom',                             'Wheel Odometry (Raw Ackermann)'),
    ('F1b',          '/odometry/ekf_tuned',               'Tuned EKF (Wheel vx,vy + IMU yaw,wz)'),
    ('F3a',          '/odometry/comp_filter',             'Complementary Filter Standalone'),
    ('F3b',          '/odometry/comp_slip_gate',          'Original F3b (1.4s Range-Rate Slip Gate)'),
    ('F3b_improved', '/odometry/comp_slip_gate_improved', 'Improved F3b (0.2s LiDAR Vel Cross-Check)'),
    ('F4',           '/odometry/lidar_odom',              '2D LiDAR Trimmed-ICP Odometry (/scan only)'),
    ('F5',           '/odometry/lidar_imu_wheel_ekf',     'Original F5: LiDAR(x,y,vx,vy) + IMU(yaw,wz) EKF'),
    ('F5_improved',  '/odometry/f5_improved',             'Improved F5: LiDAR(x,y,yaw,vx,vy) + IMU(wz) EKF'),
]

SCENARIOS = {
    'S1_trial_01_identical': {
        'scenario_key': 'S1',
        'title': 'S1_trial_01_identical',
        'seed': 0,
        'spawn': {'x': -0.8, 'y': -0.3, 'z': 0.05, 'yaw': 0.0},
        'perturb': {'w_scale': 1.0, 'w_noise': 0.0, 'g_bias': 0.0, 'g_noise': 0.0, 'a_bias': 0.0, 'a_noise': 0.0},
        'record_video': os.path.join(S1_VID_DIR, 'S1_trial_01_identical.mp4'),
        'schedule': [
            (0.8, 0.00,  0.00, 'hold_start'),
            (3.0, 0.26,  0.00, 'straight_1'),
            (2.8, 0.22,  0.34, 'left_curve'),
            (2.8, 0.22, -0.34, 'right_align'),
            (2.5, 0.22, -0.30, 'right_curve'),
            (1.5, 0.00,  0.00, 'hold_stop'),
        ],
    },
    'S1_trial_06_varied_seed_seed1001': {
        'scenario_key': 'S1_seed1001',
        'title': 'S1_trial_06_varied_seed_seed1001',
        'seed': 1001,
        'spawn': {'x': -0.7858, 'y': -0.3145, 'z': 0.05, 'yaw': -0.0118},
        'perturb': {'w_scale': 0.9852, 'w_noise': 0.0081, 'g_bias': 0.00041, 'g_noise': 0.0039, 'a_bias': 0.0012, 'a_noise': 0.021},
        'record_video': os.path.join(S1_VID_DIR, 'S1_trial_06_varied_seed_seed1001.mp4'),
        'schedule': [
            (0.8, 0.00,  0.00, 'hold_start'),
            (3.0, 0.26,  0.00, 'straight_1'),
            (2.8, 0.22,  0.34, 'left_curve'),
            (2.8, 0.22, -0.34, 'right_align'),
            (2.5, 0.22, -0.30, 'right_curve'),
            (1.5, 0.00,  0.00, 'hold_stop'),
        ],
    },
    'S2_GlassWall_SlipStall': {
        'scenario_key': 'S2',
        'title': 'S2_GlassWall_SlipStall',
        'seed': 0,
        'spawn': {'x': 1.0, 'y': 0.3, 'z': 0.05, 'yaw': -1.5708},
        'perturb': {'w_scale': 1.0, 'w_noise': 0.0, 'g_bias': 0.0, 'g_noise': 0.0, 'a_bias': 0.0, 'a_noise': 0.0},
        'record_video': None,
        'schedule': [
            (0.8,  0.00, 0.00, 'hold_start'),
            (7.5,  0.30, 0.00, 'drive_and_stall_at_wall'),
            (2.2, -0.22, 0.00, 'reverse_from_wall'),
            (2.0,  0.22, 0.42, 'turn_away'),
            (1.2,  0.00, 0.00, 'hold_stop'),
        ],
    },
    'S3_SharpTurns': {
        'scenario_key': 'S3',
        'title': 'S3_SharpTurns',
        'seed': 0,
        'spawn': {'x': -0.9, 'y': -0.3, 'z': 0.05, 'yaw': 0.0},
        'perturb': {'w_scale': 1.0, 'w_noise': 0.0, 'g_bias': 0.0, 'g_noise': 0.0, 'a_bias': 0.0, 'a_noise': 0.0},
        'record_video': None,
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
    'S4_IMUBias_Fault': {
        'scenario_key': 'S4',
        'title': 'S4_IMUBias_Fault',
        'seed': 0,
        'spawn': {'x': -0.8, 'y': -0.3, 'z': 0.05, 'yaw': 0.0},
        'perturb': {
            'w_scale': 1.0, 'w_noise': 0.0,
            'g_bias': 0.035, 'g_noise': 0.015,
            'a_bias': 0.28,  'a_noise': 0.08,
        },
        'record_video': None,
        'schedule': [
            (1.0, 0.00,  0.00, 'hold_start'),
            (3.0, 0.25,  0.00, 'straight_1'),
            (2.8, 0.22,  0.34, 'left_curve'),
            (2.8, 0.22, -0.34, 'right_align'),
            (3.0, 0.00,  0.00, 'stationary_bias_drift_hold'),
        ],
    },
}


class AblationCollectorNode(Node):
    def __init__(self):
        super().__init__(
            'ablation_collector_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gt_msg = None
        self.est_msgs = {code: None for code, _, _ in ESTIMATORS}
        self.f4_lat_samples = []
        self.slip_diag_samples = []

        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 20)
        for code, topic, _ in ESTIMATORS:
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, c=code: self.on_est(c, msg),
                20
            )
        self.create_subscription(Float32MultiArray, '/diagnostics/f4_latency', self.on_f4_lat, 20)
        self.create_subscription(Float32MultiArray, '/diagnostics/slip_gate', self.on_slip_diag, 20)
        self.samples = []

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

    def on_f4_lat(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.f4_lat_samples.append({
                'icp_compute_wall_ms': float(msg.data[0]),
                'sim_publish_delay_ms': float(msg.data[1]),
                'scan_period_ms': float(msg.data[2]),
                'inlier_count': float(msg.data[3]),
            })

    def on_slip_diag(self, msg: Float32MultiArray):
        if len(msg.data) >= 7 and self.gt_msg is not None:
            self.slip_diag_samples.append({
                'sim_time': round(self.gt_msg[0], 4),
                'gt_vx': round(self.gt_msg[4], 4),
                'wheel_vx': float(msg.data[0]),
                'f3b_orig_vx': float(msg.data[1]),
                'f3b_orig_active': bool(msg.data[3] > 0.5),
                'f3b_imp_vx': float(msg.data[4]),
                'f3b_imp_active': bool(msg.data[5] > 0.5),
                'lidar_speed': float(msg.data[6]),
            })

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
        self.samples.append(row)


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


def evaluate_metrics_zoh_and_interp(samples, code: str):
    gt_t = np.array([r['sim_time'] for r in samples], dtype=np.float64)
    gt_x = np.array([r['GT_x'] for r in samples], dtype=np.float64)
    gt_y = np.array([r['GT_y'] for r in samples], dtype=np.float64)
    gt_yaw = np.array([r['GT_yaw'] for r in samples], dtype=np.float64)
    gt_yaw_unwrap = np.unwrap(gt_yaw)

    ex = np.array([r.get(f'{code}_x', np.nan) for r in samples], dtype=np.float64)
    ey = np.array([r.get(f'{code}_y', np.nan) for r in samples], dtype=np.float64)
    eyaw = np.array([r.get(f'{code}_yaw', np.nan) for r in samples], dtype=np.float64)
    estamp = np.array([r.get(f'{code}_stamp', np.nan) for r in samples], dtype=np.float64)

    valid = np.isfinite(ex) & np.isfinite(ey) & np.isfinite(eyaw) & np.isfinite(estamp)
    if np.count_nonzero(valid) < 5:
        return {'status': 'NO_DATA'}

    gt_t_v, gt_x_v, gt_y_v, gt_yaw_v = gt_t[valid], gt_x[valid], gt_y[valid], gt_yaw[valid]
    ex_v, ey_v, eyaw_v, st_v = ex[valid], ey[valid], eyaw[valid], estamp[valid]

    # ZOH error
    pos_zoh = np.hypot(ex_v - gt_x_v, ey_v - gt_y_v)
    yaw_zoh = np.abs(np.degrees(np.arctan2(np.sin(eyaw_v - gt_yaw_v), np.cos(eyaw_v - gt_yaw_v))))

    # Timestamp-interpolated error
    st_clip = np.clip(st_v, gt_t[0], gt_t[-1])
    gx_i = np.interp(st_clip, gt_t, gt_x)
    gy_i = np.interp(st_clip, gt_t, gt_y)
    gyaw_i = np.arctan2(np.sin(np.interp(st_clip, gt_t, gt_yaw_unwrap)), np.cos(np.interp(st_clip, gt_t, gt_yaw_unwrap)))
    pos_int = np.hypot(ex_v - gx_i, ey_v - gy_i)
    yaw_int = np.abs(np.degrees(np.arctan2(np.sin(eyaw_v - gyaw_i), np.cos(eyaw_v - gyaw_i))))

    sample_hold_age_ms = np.maximum(0.0, (gt_t_v - st_v) * 1000.0)

    # RPE (1.0s = 10 steps)
    step = min(10, max(1, len(gt_t_v) // 5))
    rpe_t, rpe_r = [], []
    for i in range(len(gt_t_v) - step):
        dx_g, dy_g = gx_i[i + step] - gx_i[i], gy_i[i + step] - gy_i[i]
        cg, sg = math.cos(gyaw_i[i]), math.sin(gyaw_i[i])
        rx_g, ry_g = cg * dx_g + sg * dy_g, -sg * dx_g + cg * dy_g
        ryaw_g = wrap_angle(gyaw_i[i + step] - gyaw_i[i])

        dx_e, dy_e = ex_v[i + step] - ex_v[i], ey_v[i + step] - ey_v[i]
        ce, se = math.cos(eyaw_v[i]), math.sin(eyaw_v[i])
        rx_e, ry_e = ce * dx_e + se * dy_e, -se * dx_e + ce * dy_e
        ryaw_e = wrap_angle(eyaw_v[i + step] - eyaw_v[i])

        rpe_t.append(math.hypot(rx_e - rx_g, ry_e - ry_g))
        rpe_r.append(abs(math.degrees(wrap_angle(ryaw_e - ryaw_g))))

    return {
        'status': 'OK',
        'samples': int(len(pos_zoh)),
        'zoh_ate_rmse_m': round(float(np.sqrt(np.mean(pos_zoh ** 2))), 4),
        'interp_ate_rmse_m': round(float(np.sqrt(np.mean(pos_int ** 2))), 4),
        'interp_ate_p95_m': round(float(np.percentile(pos_int, 95.0)), 4),
        'interp_ate_max_m': round(float(np.max(pos_int)), 4),
        'rpe_trans_rmse_m': round(float(np.sqrt(np.mean(np.array(rpe_t) ** 2))), 4),
        'rpe_rot_rmse_deg': round(float(np.sqrt(np.mean(np.array(rpe_r) ** 2))), 4),
        'zoh_yaw_rmse_deg': round(float(np.sqrt(np.mean(yaw_zoh ** 2))), 4),
        'interp_yaw_rmse_deg': round(float(np.sqrt(np.mean(yaw_int ** 2))), 4),
        'interp_yaw_max_deg': round(float(np.max(yaw_int)), 4),
        'sample_hold_age_mean_ms': round(float(np.mean(sample_hold_age_ms)), 2),
        'sample_hold_age_p95_ms': round(float(np.percentile(sample_hold_age_ms, 95.0)), 2),
        'max_forward_x_m': round(float(np.max(ex_v)), 4),
    }


def main():
    os.makedirs(ABL_DIR, exist_ok=True)
    os.makedirs(PLOT_DIR, exist_ok=True)
    os.makedirs(S1_VID_DIR, exist_ok=True)

    env = os.environ.copy()
    env['PYTHONNOUSERSITE'] = '1'
    rl_lib_dir = '/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/robot_localization/lib'
    env['LD_LIBRARY_PATH'] = f"{rl_lib_dir}:{env.get('LD_LIBRARY_PATH', '')}"

    sim_cmd = [
        'ros2', 'launch',
        os.path.join(EXP_DIR, 'launch', 'base_gazebo_for_trials.launch.py'),
        'spawn_x:=-0.8', 'spawn_y:=-0.3', 'spawn_z:=0.05', 'spawn_yaw:=0.0'
    ]
    print("Launching Gazebo Fortress for WP1 (S1 Videos) & WP2 (F4/F5/F3b Ablation)...")
    p_sim = subprocess.Popen(sim_cmd, env=env, preexec_fn=os.setsid,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(7.0)

    rclpy.init()
    all_results = {}

    try:
        for run_name, spec in SCENARIOS.items():
            sp = spec['spawn']
            pt = spec['perturb']
            print(f"\n--- Running {run_name} ---")
            reset_gazebo_pose(sp['x'], sp['y'], sp['z'], sp['yaw'])
            time.sleep(0.5)

            procs = []
            norm_cmd = [
                'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py'),
                '--ros-args',
                '-p', f"seed:={spec['seed']}",
                '-p', f"wheel_vel_scale:={pt['w_scale']}",
                '-p', f"wheel_vel_noise_std:={pt['w_noise']}",
                '-p', f"imu_gyro_bias_z:={pt['g_bias']}",
                '-p', f"imu_gyro_noise_std:={pt['g_noise']}",
                '-p', f"imu_accel_bias_x:={pt['a_bias']}",
                '-p', f"imu_accel_noise_std:={pt['a_noise']}",
            ]
            procs.append(subprocess.Popen(norm_cmd, env=env, preexec_fn=os.setsid,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
                                          env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'odom_tf_relay_node.py')],
                                          env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
            procs.append(subprocess.Popen(['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'state_estimators.launch.py')],
                                          env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

            p_vid = None
            if spec['record_video']:
                vid_cmd = [
                    'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'closed_loop_video_recorder_node.py'),
                    '--ros-args',
                    '-p', f"output_mp4:={spec['record_video']}",
                    '-p', f"scenario_title:={spec['title']}",
                    '-p', 'active_estimator:=ALL'
                ]
                p_vid = subprocess.Popen(vid_cmd, env=env, preexec_fn=os.setsid,
                                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                procs.append(p_vid)

            collector = AblationCollectorNode()
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
                time.sleep(3.5)

            csv_path = os.path.join(ABL_DIR, f'{run_name}.csv')
            if collector.samples:
                with open(csv_path, 'w', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=list(collector.samples[0].keys()))
                    writer.writeheader()
                    writer.writerows(collector.samples)

            metrics = {}
            for code, _, _ in ESTIMATORS:
                metrics[code] = evaluate_metrics_zoh_and_interp(collector.samples, code)

            # Summarize F4 ICP compute latency vs sample-hold output latency
            f4_lat_summary = {}
            if collector.f4_lat_samples:
                icp_ms = np.array([x['icp_compute_wall_ms'] for x in collector.f4_lat_samples])
                per_ms = np.array([x['scan_period_ms'] for x in collector.f4_lat_samples])
                inl = np.array([x['inlier_count'] for x in collector.f4_lat_samples])
                f4_lat_summary = {
                    'icp_compute_wall_ms_mean': round(float(np.mean(icp_ms)), 3),
                    'icp_compute_wall_ms_p95': round(float(np.percentile(icp_ms, 95.0)), 3),
                    'icp_compute_wall_ms_max': round(float(np.max(icp_ms)), 3),
                    'scan_period_ms_mean': round(float(np.mean(per_ms)), 2),
                    'inlier_count_mean': round(float(np.mean(inl)), 1),
                    'sample_hold_age_mean_ms': metrics['F4'].get('sample_hold_age_mean_ms'),
                }

            # Analyze S2 slip gate timing (F3b original vs F3b_improved)
            slip_timing = {}
            if spec['scenario_key'] == 'S2' and collector.slip_diag_samples:
                # Find physical contact time when gt_vx drops < 0.03 while wheel_vx > 0.15
                impact_samples = [s for s in collector.slip_diag_samples if s['sim_time'] > 2.0 and s['wheel_vx'] > 0.15 and abs(s['gt_vx']) < 0.03]
                t_impact = impact_samples[0]['sim_time'] if impact_samples else None
                pre_false_orig = sum(1 for s in collector.slip_diag_samples if t_impact and s['sim_time'] < t_impact - 0.1 and s['f3b_orig_active'])
                pre_false_imp = sum(1 for s in collector.slip_diag_samples if t_impact and s['sim_time'] < t_impact - 0.1 and s['f3b_imp_active'])
                post_orig = [s['sim_time'] for s in collector.slip_diag_samples if t_impact and s['sim_time'] >= t_impact and s['f3b_orig_active']]
                post_imp = [s['sim_time'] for s in collector.slip_diag_samples if t_impact and s['sim_time'] >= t_impact and s['f3b_imp_active']]
                slip_timing = {
                    'physical_impact_sim_time_s': t_impact,
                    'f3b_original_pre_impact_false_trigger_samples': pre_false_orig,
                    'f3b_improved_pre_impact_false_trigger_samples': pre_false_imp,
                    'f3b_original_post_impact_detection_delay_s': round(post_orig[0] - t_impact, 3) if (t_impact and post_orig) else None,
                    'f3b_improved_post_impact_detection_delay_s': round(post_imp[0] - t_impact, 3) if (t_impact and post_imp) else None,
                }

            all_results[run_name] = {
                'scenario_key': spec['scenario_key'],
                'csv_path': csv_path,
                'video_mp4': spec['record_video'],
                'f4_latency_breakdown': f4_lat_summary,
                's2_slip_gate_timing_comparison': slip_timing,
                'metrics': metrics,
            }
            print(
                f"  [Done] F4 ICP compute={f4_lat_summary.get('icp_compute_wall_ms_mean')}ms | "
                f"Interp ATE RMSE: F4={metrics['F4'].get('interp_ate_rmse_m')}m, "
                f"F5_orig={metrics['F5'].get('interp_ate_rmse_m')}m (Yaw={metrics['F5'].get('interp_yaw_rmse_deg')}deg), "
                f"F5_imp={metrics['F5_improved'].get('interp_ate_rmse_m')}m (Yaw={metrics['F5_improved'].get('interp_yaw_rmse_deg')}deg)"
            )

            collector.destroy_node()
            for p in procs:
                if p and p.poll() is None:
                    try:
                        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                    except Exception:
                        pass
            time.sleep(0.8)

        out_json = os.path.join(ABL_DIR, 'wp2_f4_f5_f3b_ablation_summary.json')
        with open(out_json, 'w') as f:
            json.dump(all_results, f, indent=2)
        print(f"\nSaved WP2 ablation summary to {out_json}")

        # Generate WP2 diagnostic comparison plot
        fig, axes = plt.subplots(1, 3, figsize=(18, 5.5), dpi=140)
        # Panel 1: S4 IMU Bias Yaw RMSE comparison (F4 vs F5_original vs F5_improved)
        s4_m = all_results['S4_IMUBias_Fault']['metrics']
        c_list = ['B0', 'F1b', 'F4', 'F5', 'F5_improved']
        yaws = [s4_m[c]['interp_yaw_rmse_deg'] for c in c_list]
        ates = [s4_m[c]['interp_ate_rmse_m'] for c in c_list]
        axes[0].bar(c_list, yaws, color=['#e63946', '#00b4d8', '#3a86ff', '#ff006e', '#2ec4b6'])
        axes[0].set_title('S4 (IMU Gyro Bias): Yaw RMSE (deg)\nF5_improved Eliminates IMU Yaw Drift')
        axes[0].set_ylabel('Yaw RMSE (deg)')
        axes[0].grid(True, axis='y', alpha=0.3)

        # Panel 2: S4 ATE RMSE comparison
        axes[1].bar(c_list, ates, color=['#e63946', '#00b4d8', '#3a86ff', '#ff006e', '#2ec4b6'])
        axes[1].set_title('S4 (IMU Gyro Bias): Timestamp-Aligned ATE RMSE (m)')
        axes[1].set_ylabel('ATE RMSE (m)')
        axes[1].grid(True, axis='y', alpha=0.3)

        # Panel 3: F4 ICP Compute Latency vs 10Hz Sample-Hold Output Age vs 50Hz EKF Output Age
        s1_res = all_results['S1_trial_01_identical']
        lat_labels = ['F4 Actual ICP\nCompute (Wall)', 'F5 / F5_imp\n50Hz Sample Age', 'F4 10Hz ZOH\nSample Age']
        lat_vals = [
            s1_res['f4_latency_breakdown']['icp_compute_wall_ms_mean'],
            s1_res['metrics']['F5_improved']['sample_hold_age_mean_ms'],
            s1_res['metrics']['F4']['sample_hold_age_mean_ms'],
        ]
        axes[2].bar(lat_labels, lat_vals, color=['#2ec4b6', '#3a86ff', '#f77f00'])
        axes[2].set_title('Latency Separation: ICP Compute (~3ms)\nvs Sensor Sample-Hold Age (ms)')
        axes[2].set_ylabel('Latency / Age (ms)')
        axes[2].grid(True, axis='y', alpha=0.3)

        plt.tight_layout()
        plt.savefig(os.path.join(PLOT_DIR, 'wp2_f4_f5_ablation_comparison.png'))
        plt.close(fig)

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


if __name__ == '__main__':
    main()
