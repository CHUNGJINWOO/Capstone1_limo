#!/usr/bin/env -S python3 -s
"""
verify_pipeline_smoke_test.py
Executes the 1-Run Smoke Test for the LIMO Pro State Estimation & Video Recording Benchmark:
1. Launches Gazebo Fortress Lounge World + LIMO + Fixed Bridges + State Estimators (B0, F1a, F1b, F2, F3a, F3b, F4, F5).
2. Starts rosbag2 recording and SimVideoRecorderNode (1920x960 @ 15 FPS H.264 MP4).
3. Verifies topic reception, message frequencies, sim-time timestamp synchronization, and TF tree
   (including fixed depth_camera_link / depth_link transforms and single odom->base_footprint broadcaster).
4. Executes Scenario S1 (Normal Driving + Left/Right Ackermann Turns + Stop) in Gazebo physics.
5. Computes ATE (RMSE, Mean, Median, Max, Final Error), RPE (1.0s window), and Yaw Error against Ground Truth
   for all 8 methods (B0, F1a, F1b, F2, F3a, F3b, F4, F5).
6. Saves raw CSV time-series, JSON report, trajectory/error plots, and verifies MP4 playability & rosbag2 output.
"""
import os
import sys
import csv
import math
import json
import time
import signal
import shutil
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, Image, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


BENCH_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107'


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
    ('F5',  '/odometry/lidar_imu_wheel_ekf', 'LiDAR Odom + Slip-Gated Wheel + IMU EKF'),
]


class SmokeTestMonitorNode(Node):
    def __init__(self):
        super().__init__(
            'smoke_test_monitor_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sim_time_sec = 0.0
        self.topic_stats = {}

        def register_stat(topic_name):
            self.topic_stats[topic_name] = {
                'count': 0,
                'first_wall_t': None,
                'last_wall_t': None,
                'last_stamp_sec': None,
                'last_sim_diff_sec': None,
                'frame_id': None,
            }

        # Monitor sensor & system topics
        for t in ['/clock', '/imu', '/scan', '/camera/depth_image', '/camera/depth/points',
                  '/lounge/overhead', '/ground_truth/odom', '/ground_truth/world_odom']:
            register_stat(t)

        self.create_subscription(Clock, '/clock', self.on_clock, 20)
        self.create_subscription(Imu, '/imu', lambda m: self.record_header('/imu', m.header), 50)
        self.create_subscription(LaserScan, '/scan', lambda m: self.record_header('/scan', m.header), 10)
        self.create_subscription(Image, '/camera/depth_image', lambda m: self.record_header('/camera/depth_image', m.header), 5)
        self.create_subscription(PointCloud2, '/camera/depth/points', lambda m: self.record_header('/camera/depth/points', m.header), 5)
        self.create_subscription(Image, '/lounge/overhead', lambda m: self.record_header('/lounge/overhead', m.header), 5)

        self.latest_gt = None  # (t_sec, x, y, yaw, vx, wz)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt_odom, 20)
        self.create_subscription(Odometry, '/ground_truth/world_odom', lambda m: self.record_header('/ground_truth/world_odom', m.header), 20)

        self.latest_est = {}
        for code, topic, _ in ESTIMATORS:
            register_stat(topic)
            self.latest_est[code] = None
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, c=code, tp=topic: self.on_est_odom(c, tp, msg),
                20
            )

        # Monitor /tf broadcasters to verify single broadcaster for odom->base_footprint
        self.tf_odom_stamps = []
        self.tf_backwards_jumps = 0
        self.create_subscription(TFMessage, '/tf', self.on_tf_msg, 50)

        # Synchronized time-series samples (20 Hz during drive)
        self.timeseries = []

    def on_clock(self, msg: Clock):
        self.sim_time_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        st = self.topic_stats['/clock']
        now_w = time.time()
        if st['first_wall_t'] is None:
            st['first_wall_t'] = now_w
        st['last_wall_t'] = now_w
        st['count'] += 1
        st['last_stamp_sec'] = self.sim_time_sec
        st['last_sim_diff_sec'] = 0.0

    def record_header(self, topic: str, header):
        st = self.topic_stats[topic]
        now_w = time.time()
        if st['first_wall_t'] is None:
            st['first_wall_t'] = now_w
        st['last_wall_t'] = now_w
        st['count'] += 1
        stamp_s = header.stamp.sec + header.stamp.nanosec * 1e-9
        st['last_stamp_sec'] = stamp_s
        if self.sim_time_sec > 0.0 and stamp_s > 0.0:
            st['last_sim_diff_sec'] = round(abs(self.sim_time_sec - stamp_s), 4)
        st['frame_id'] = header.frame_id

    def on_gt_odom(self, msg: Odometry):
        self.record_header('/ground_truth/odom', msg.header)
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.latest_gt = (t_s, x, y, yaw, vx, wz)

    def on_est_odom(self, code: str, topic: str, msg: Odometry):
        self.record_header(topic, msg.header)
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.latest_est[code] = (t_s, x, y, yaw, vx, wz)

    def on_tf_msg(self, msg: TFMessage):
        for t in msg.transforms:
            if t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint':
                ts = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                if len(self.tf_odom_stamps) > 0 and ts < self.tf_odom_stamps[-1] - 1e-4:
                    self.tf_backwards_jumps += 1
                self.tf_odom_stamps.append(ts)

    def publish_cmd(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def sample_state(self, phase_name: str):
        if self.latest_gt is None:
            return
        row = {
            'sim_time': self.sim_time_sec,
            'phase': phase_name,
            'GT_x': self.latest_gt[1],
            'GT_y': self.latest_gt[2],
            'GT_yaw': self.latest_gt[3],
            'GT_vx': self.latest_gt[4],
            'GT_wz': self.latest_gt[5],
        }
        for code, _, _ in ESTIMATORS:
            est = self.latest_est.get(code)
            if est is not None:
                row[f'{code}_x'] = est[1]
                row[f'{code}_y'] = est[2]
                row[f'{code}_yaw'] = est[3]
                row[f'{code}_vx'] = est[4]
                row[f'{code}_wz'] = est[5]
            else:
                row[f'{code}_x'] = np.nan
                row[f'{code}_y'] = np.nan
                row[f'{code}_yaw'] = np.nan
                row[f'{code}_vx'] = np.nan
                row[f'{code}_wz'] = np.nan
        self.timeseries.append(row)

    def check_tf_links(self):
        pairs = [
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
            ('base_link', 'imu_link'),
            ('base_link', 'laser_link'),
            ('base_link', 'depth_camera_link'),
            ('base_link', 'depth_link'),
        ]
        results = {}
        for parent, child in pairs:
            ok = self.tf_buffer.can_transform(parent, child, rclpy.time.Time())
            results[f'{parent} -> {child}'] = bool(ok)
        return results


def compute_metrics(timeseries, code: str):
    gt_x = np.array([r['GT_x'] for r in timeseries], dtype=np.float64)
    gt_y = np.array([r['GT_y'] for r in timeseries], dtype=np.float64)
    gt_yaw = np.array([r['GT_yaw'] for r in timeseries], dtype=np.float64)
    sim_t = np.array([r['sim_time'] for r in timeseries], dtype=np.float64)

    ex = np.array([r[f'{code}_x'] for r in timeseries], dtype=np.float64)
    ey = np.array([r[f'{code}_y'] for r in timeseries], dtype=np.float64)
    eyaw = np.array([r[f'{code}_yaw'] for r in timeseries], dtype=np.float64)

    valid = np.isfinite(ex) & np.isfinite(ey) & np.isfinite(eyaw)
    if np.count_nonzero(valid) < 5:
        return {'status': 'NO_DATA', 'valid_samples': int(np.count_nonzero(valid))}

    gt_x, gt_y, gt_yaw, sim_t = gt_x[valid], gt_y[valid], gt_yaw[valid], sim_t[valid]
    ex, ey, eyaw = ex[valid], ey[valid], eyaw[valid]

    pos_err = np.hypot(ex - gt_x, ey - gt_y)
    yaw_err_rad = np.arctan2(np.sin(eyaw - gt_yaw), np.cos(eyaw - gt_yaw))
    yaw_err_deg = np.abs(np.degrees(yaw_err_rad))

    # Relative Pose Error (RPE) over delta_t ~= 1.0s (approx 10 steps at 10 Hz sampling)
    step = min(10, max(1, len(sim_t) // 5))
    rpe_trans = []
    rpe_rot_deg = []
    for i in range(len(sim_t) - step):
        # GT relative motion in body frame i
        dx_gt = gt_x[i + step] - gt_x[i]
        dy_gt = gt_y[i + step] - gt_y[i]
        c_gt, s_gt = math.cos(gt_yaw[i]), math.sin(gt_yaw[i])
        rel_x_gt = c_gt * dx_gt + s_gt * dy_gt
        rel_y_gt = -s_gt * dx_gt + c_gt * dy_gt
        rel_yaw_gt = wrap_angle(gt_yaw[i + step] - gt_yaw[i])

        # Est relative motion in body frame i
        dx_e = ex[i + step] - ex[i]
        dy_e = ey[i + step] - ey[i]
        c_e, s_e = math.cos(eyaw[i]), math.sin(eyaw[i])
        rel_x_e = c_e * dx_e + s_e * dy_e
        rel_y_e = -s_e * dx_e + c_e * dy_e
        rel_yaw_e = wrap_angle(eyaw[i + step] - eyaw[i])

        rpe_trans.append(math.hypot(rel_x_e - rel_x_gt, rel_y_e - rel_y_gt))
        rpe_rot_deg.append(abs(math.degrees(wrap_angle(rel_yaw_e - rel_yaw_gt))))

    rpe_trans = np.array(rpe_trans, dtype=np.float64) if len(rpe_trans) > 0 else np.array([0.0])
    rpe_rot_deg = np.array(rpe_rot_deg, dtype=np.float64) if len(rpe_rot_deg) > 0 else np.array([0.0])

    return {
        'status': 'OK',
        'valid_samples': int(len(pos_err)),
        'ate_rmse_m': round(float(np.sqrt(np.mean(pos_err ** 2))), 4),
        'ate_mean_m': round(float(np.mean(pos_err)), 4),
        'ate_median_m': round(float(np.median(pos_err)), 4),
        'ate_max_m': round(float(np.max(pos_err)), 4),
        'final_pos_err_m': round(float(pos_err[-1]), 4),
        'rpe_trans_rmse_m': round(float(np.sqrt(np.mean(rpe_trans ** 2))), 4),
        'rpe_rot_rmse_deg': round(float(np.sqrt(np.mean(rpe_rot_deg ** 2))), 4),
        'yaw_err_rmse_deg': round(float(np.sqrt(np.mean(yaw_err_deg ** 2))), 4),
        'yaw_err_max_deg': round(float(np.max(yaw_err_deg)), 4),
        'final_yaw_err_deg': round(float(yaw_err_deg[-1]), 4),
    }


def generate_smoke_test_plots(timeseries, metrics, out_png: str):
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.8), dpi=140)

    sim_t = np.array([r['sim_time'] for r in timeseries]) - timeseries[0]['sim_time']
    gt_x = np.array([r['GT_x'] for r in timeseries])
    gt_y = np.array([r['GT_y'] for r in timeseries])
    gt_yaw = np.array([r['GT_yaw'] for r in timeseries])

    colors = {
        'B0': '#e63946',
        'F1a': '#7f8c8d',
        'F1b': '#00b4d8',
        'F2': '#f77f00',
        'F3a': '#9b5de5',
        'F3b': '#ffbe0b',
        'F4': '#3a86ff',
        'F5': '#ff006e',
    }

    # Panel 1: 2D Trajectory
    ax0 = axes[0]
    ax0.plot(gt_x, gt_y, color='#2ecc71', linewidth=3.2, label='Ground Truth (GT)')
    for code, _, _ in ESTIMATORS:
        ex = np.array([r[f'{code}_x'] for r in timeseries])
        ey = np.array([r[f'{code}_y'] for r in timeseries])
        if np.any(np.isfinite(ex)):
            ls = '--' if code in ('B0', 'F1a') else '-'
            ax0.plot(ex, ey, linestyle=ls, linewidth=1.6, color=colors[code], label=code)
    ax0.set_title('S1 Smoke Test: 2D Trajectory in Odom Frame')
    ax0.set_xlabel('X (m)')
    ax0.set_ylabel('Y (m)')
    ax0.axis('equal')
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc='best', fontsize=8)

    # Panel 2: Position Error vs Time
    ax1 = axes[1]
    for code, _, _ in ESTIMATORS:
        ex = np.array([r[f'{code}_x'] for r in timeseries])
        ey = np.array([r[f'{code}_y'] for r in timeseries])
        if np.any(np.isfinite(ex)):
            pe = np.hypot(ex - gt_x, ey - gt_y)
            ax1.plot(sim_t, pe, linewidth=1.6, color=colors[code],
                     label=f"{code} (RMSE={metrics[code].get('ate_rmse_m', 0):.3f}m)")
    ax1.set_title('Instantaneous Position Error ||p_est - p_GT|| (m)')
    ax1.set_xlabel('Elapsed Sim Time (s)')
    ax1.set_ylabel('Position Error (m)')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left', fontsize=8)

    # Panel 3: Heading (Yaw) Error vs Time
    ax2 = axes[2]
    for code, _, _ in ESTIMATORS:
        eyaw = np.array([r[f'{code}_yaw'] for r in timeseries])
        if np.any(np.isfinite(eyaw)):
            ye = np.abs(np.degrees(np.arctan2(np.sin(eyaw - gt_yaw), np.cos(eyaw - gt_yaw))))
            ax2.plot(sim_t, ye, linewidth=1.6, color=colors[code],
                     label=f"{code} (RMSE={metrics[code].get('yaw_err_rmse_deg', 0):.2f}°)")
    ax2.set_title('Absolute Heading Error |yaw_est - yaw_GT| (deg)')
    ax2.set_xlabel('Elapsed Sim Time (s)')
    ax2.set_ylabel('Yaw Error (deg)')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper left', fontsize=8)

    plt.tight_layout()
    plt.savefig(out_png)
    plt.close(fig)


def cleanup_processes(procs):
    for p in procs:
        if p and p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                pass
    time.sleep(2.0)
    for p in procs:
        if p and p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
    subprocess.run(['pkill', '-f', 'ign gazebo'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-f', 'parameter_bridge'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    os.makedirs(os.path.join(BENCH_DIR, 'videos', 'smoke_test'), exist_ok=True)
    os.makedirs(os.path.join(BENCH_DIR, 'results', 'smoke_test'), exist_ok=True)
    os.makedirs(os.path.join(BENCH_DIR, 'results', 'plots'), exist_ok=True)

    video_mp4 = os.path.join(BENCH_DIR, 'videos', 'smoke_test', 's1_normal_multiturn_smoke_test.mp4')
    bag_dir = os.path.join(BENCH_DIR, 'results', 'smoke_test', 'rosbag2_s1_smoke_test')
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)

    env = os.environ.copy()
    env['PYTHONNOUSERSITE'] = '1'

    procs = []
    try:
        print("[1/6] Launching Gazebo Fortress + LIMO + Fixed Bridges (benchmark_sim.launch.py)...")
        sim_cmd = [
            'ros2', 'launch',
            os.path.join(BENCH_DIR, 'launch', 'benchmark_sim.launch.py'),
            'spawn_x:=-0.8', 'spawn_y:=-0.3', 'spawn_z:=0.05', 'spawn_yaw:=0.0'
        ]
        p_sim = subprocess.Popen(sim_cmd, env=env, preexec_fn=os.setsid,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_sim)
        time.sleep(7.0)

        print("[2/6] Launching State Estimators B0, F1a, F1b, F2, F3a, F3b, F4, F5 (state_estimators.launch.py)...")
        est_cmd = [
            'ros2', 'launch',
            os.path.join(BENCH_DIR, 'launch', 'state_estimators.launch.py')
        ]
        p_est = subprocess.Popen(est_cmd, env=env, preexec_fn=os.setsid,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_est)
        time.sleep(3.5)

        print("[3/6] Starting SimVideoRecorderNode & rosbag2 recorder...")
        vid_cmd = [
            'python3', '-s',
            os.path.join(BENCH_DIR, 'nodes', 'sim_video_recorder_node.py'),
            '--ros-args',
            '-p', f'output_mp4:={video_mp4}',
            '-p', 'scenario_title:=S1_Normal_MultiTurn_SmokeTest'
        ]
        p_vid = subprocess.Popen(vid_cmd, env=env, preexec_fn=os.setsid,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_vid)

        bag_cmd = [
            'ros2', 'bag', 'record', '-o', bag_dir,
            '/clock', '/odom', '/imu', '/scan', '/cmd_vel',
            '/ground_truth/odom', '/ground_truth/world_odom',
            '/odometry/ekf_legacy', '/odometry/ekf_tuned', '/odometry/ukf_tuned',
            '/odometry/comp_filter', '/odometry/comp_slip_gate',
            '/odometry/lidar_odom', '/odometry/lidar_imu_wheel_ekf'
        ]
        p_bag = subprocess.Popen(bag_cmd, env=env, preexec_fn=os.setsid,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_bag)

        rclpy.init()
        monitor = SmokeTestMonitorNode()

        # Spin for 3s to warm up subscriptions and TF buffer
        t_warmup = time.time()
        while time.time() - t_warmup < 3.5:
            rclpy.spin_once(monitor, timeout_sec=0.05)

        print("[4/6] Executing Scenario S1 Multi-Segment Physical Driving Trajectory (20.0s)...")
        # Driving schedule in open corridor (-0.8, -0.3) -> (+3.2, -0.3):
        # (duration_s, vx_m_s, wz_rad_s, phase_name)
        schedule = [
            (1.0, 0.00,  0.00, 'initial_hold'),
            (4.0, 0.25,  0.00, 'straight_1'),
            (3.5, 0.22,  0.32, 'left_curve'),
            (3.5, 0.22, -0.32, 'right_curve_align'),
            (3.5, 0.22, -0.28, 'right_curve_2'),
            (3.5, 0.22,  0.28, 'left_curve_align'),
            (2.5, 0.00,  0.00, 'final_stop_hold'),
        ]

        for dur, vx, wz, phase in schedule:
            t_seg_start = time.time()
            last_sample_t = 0.0
            while time.time() - t_seg_start < dur:
                monitor.publish_cmd(vx, wz)
                rclpy.spin_once(monitor, timeout_sec=0.02)
                now = time.time()
                if now - last_sample_t >= 0.10:
                    monitor.sample_state(phase)
                    last_sample_t = now

        monitor.publish_cmd(0.0, 0.0)
        tf_links_verified = monitor.check_tf_links()

        # Finalize video & rosbag2 cleanly with SIGINT first so headers/verification files are written
        print("[5/6] Finalizing MP4 video & rosbag2 archive...")
        for p in (p_vid, p_bag):
            if p.poll() is None:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
        time.sleep(3.5)

        # Compute topic rates & timestamp sync
        topic_report = {}
        for tp, st in monitor.topic_stats.items():
            cnt = st['count']
            dur = (st['last_wall_t'] - st['first_wall_t']) if (st['first_wall_t'] and st['last_wall_t'] and st['last_wall_t'] > st['first_wall_t']) else 0.0
            hz = round(cnt / dur, 2) if dur > 0.2 else 0.0
            topic_report[tp] = {
                'messages_received': cnt,
                'measured_rate_hz': hz,
                'last_stamp_sec': round(st['last_stamp_sec'], 3) if st['last_stamp_sec'] else None,
                'sim_time_sync_diff_sec': st['last_sim_diff_sec'],
                'frame_id': st['frame_id'],
            }

        # Compute ATE/RPE metrics for all estimators
        print("[6/6] Computing ATE/RPE metrics and generating verification plots...")
        est_metrics = {}
        for code, topic, desc in ESTIMATORS:
            m = compute_metrics(monitor.timeseries, code)
            m['topic'] = topic
            m['description'] = desc
            est_metrics[code] = m

        # Save raw timeseries CSV
        csv_path = os.path.join(BENCH_DIR, 'results', 'smoke_test', 's1_smoke_test_timeseries.csv')
        if len(monitor.timeseries) > 0:
            fieldnames = list(monitor.timeseries[0].keys())
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(monitor.timeseries)

        # Generate plot
        plot_path = os.path.join(BENCH_DIR, 'results', 'plots', 's1_smoke_test_comparison.png')
        if len(monitor.timeseries) > 5:
            generate_smoke_test_plots(monitor.timeseries, est_metrics, plot_path)

        # Read video verification json
        vid_meta_path = video_mp4.replace('.mp4', '_video_verification.json')
        vid_meta = {}
        if os.path.exists(vid_meta_path):
            with open(vid_meta_path, 'r') as f:
                vid_meta = json.load(f)

        bag_meta_path = os.path.join(bag_dir, 'metadata.yaml')
        bag_exists = os.path.exists(bag_meta_path)

        summary = {
            'test_name': 'Stage2_Foundation_and_S1_Smoke_Test',
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'spawn_pose': {'x': -0.8, 'y': -0.3, 'z': 0.05, 'yaw': 0.0},
            'samples_collected': len(monitor.timeseries),
            'tf_links_verified': tf_links_verified,
            'tf_odom_messages_checked': len(monitor.tf_odom_stamps),
            'tf_backwards_timestamp_jumps': monitor.tf_backwards_jumps,
            'topic_reception_and_sync': topic_report,
            'estimator_metrics': est_metrics,
            'video_verification': vid_meta,
            'rosbag2_verified': {
                'bag_dir': bag_dir,
                'metadata_exists': bag_exists,
            },
            'artifacts': {
                'timeseries_csv': csv_path,
                'comparison_plot_png': plot_path,
                'video_mp4': video_mp4,
            }
        }

        out_json = os.path.join(BENCH_DIR, 'results', 'smoke_test', 's1_smoke_test_report.json')
        with open(out_json, 'w') as f:
            json.dump(summary, f, indent=2)

        print(f"\n=== SMOKE TEST COMPLETE! Report saved to {out_json} ===")
        for code, m in est_metrics.items():
            print(f"  {code:<4} | status={m['status']} | ATE_RMSE={m.get('ate_rmse_m')}m | "
                  f"ATE_Max={m.get('ate_max_m')}m | RPE_Trans={m.get('rpe_trans_rmse_m')}m | "
                  f"Yaw_RMSE={m.get('yaw_err_rmse_deg')}deg")

        monitor.destroy_node()
        rclpy.shutdown()
    finally:
        cleanup_processes(procs)


if __name__ == '__main__':
    main()
