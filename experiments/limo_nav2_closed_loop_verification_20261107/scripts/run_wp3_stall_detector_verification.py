#!/usr/bin/env -S python3 -s
"""
run_wp3_stall_detector_verification.py
Work Package 3: Independent Stall Detector & Safety Gate Verification (`nodes/stall_detector_node.py`).
Executes 3 dedicated closed-loop safety verification experiments in Gazebo Fortress:
1. Test 1 (Normal Multi-Turn Driving): Verifies 0 false triggers across straight + curved + S-turn motion.
2. Test 2 (Glass-Wall Impact -> Command Block -> Physical Stop -> Reverse Escape -> Restart):
   Measures exact impact-to-stall detection latency, verifies /cmd_vel blocking (cmd_vx_out = 0.0),
   verifies physical chassis AND wheel spin stop (preventing B0 wheel-slip runaway), verifies forward
   re-command remains blocked while latched, and verifies reverse escape unlatches back to NORMAL.
   Records MP4 video `videos/stall_detector/wp3_glass_wall_stall_stop_and_recovery.mp4`.
3. Test 3 (Sensor Dropout Fail-Safe): Simulates LiDAR odometry sensor dropout mid-drive while
   cmd_vx_raw = +0.25 m/s continues; verifies transition to FAILSAFE_SENSOR_TIMEOUT within ~0.45s
   and complete physical stop (never continuing blind driving when sensor state is uncertain).
"""
import os
import csv
import json
import math
import time
import signal
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

EXP_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_nav2_closed_loop_verification_20261107'
STALL_RES_DIR = os.path.join(EXP_DIR, 'results', 'stall_detector')
PLOT_DIR = os.path.join(EXP_DIR, 'results', 'plots')
VID_DIR = os.path.join(EXP_DIR, 'videos', 'stall_detector')


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class StallTestCollector(Node):
    def __init__(self):
        super().__init__(
            'stall_test_collector',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.pub_cmd_raw = self.create_publisher(Twist, '/cmd_vel_raw', 20)
        self.gt = None
        self.b0 = None
        self.f4 = None
        self.f3b_orig = None
        self.f3b_imp = None
        self.stall_status = None

        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 20)
        self.create_subscription(Odometry, '/odom', self.on_b0, 20)
        self.create_subscription(Odometry, '/odometry/lidar_odom', self.on_f4, 20)
        self.create_subscription(Odometry, '/odometry/comp_slip_gate', self.on_f3b_orig, 20)
        self.create_subscription(Odometry, '/odometry/comp_slip_gate_improved', self.on_f3b_imp, 20)
        self.create_subscription(String, '/stall_detector/status_json', self.on_status, 20)

        self.samples = []

    def on_gt(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt = (t, msg.pose.pose.position.x, msg.pose.pose.position.y, quat_to_yaw(msg.pose.pose.orientation), msg.twist.twist.linear.x)

    def on_b0(self, msg: Odometry):
        self.b0 = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x)

    def on_f4(self, msg: Odometry):
        self.f4 = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x)

    def on_f3b_orig(self, msg: Odometry):
        self.f3b_orig = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x)

    def on_f3b_imp(self, msg: Odometry):
        self.f3b_imp = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x)

    def on_status(self, msg: String):
        try:
            self.stall_status = json.loads(msg.data)
        except Exception:
            pass

    def send_cmd_raw(self, vx: float, wz: float):
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        self.pub_cmd_raw.publish(m)

    def record(self, phase: str):
        if self.gt is None or self.stall_status is None:
            return
        st = self.stall_status
        row = {
            'sim_time': round(self.gt[0], 4),
            'phase': phase,
            'GT_x': round(self.gt[1], 4),
            'GT_y': round(self.gt[2], 4),
            'GT_vx': round(self.gt[4], 4),
            'B0_x': round(self.b0[0], 4) if self.b0 else np.nan,
            'B0_vx': round(self.b0[2], 4) if self.b0 else np.nan,
            'F4_x': round(self.f4[0], 4) if self.f4 else np.nan,
            'F3b_orig_x': round(self.f3b_orig[0], 4) if self.f3b_orig else np.nan,
            'F3b_imp_x': round(self.f3b_imp[0], 4) if self.f3b_imp else np.nan,
            'state_code': st.get('state_code', 0),
            'state_name': st.get('state_name', 'NORMAL'),
            'cmd_blocked': bool(st.get('cmd_blocked', False)),
            'cmd_vx_raw': st.get('cmd_vx_raw', 0.0),
            'cmd_vx_out': st.get('cmd_vx_out', 0.0),
            'lidar_speed': st.get('lidar_speed', 0.0),
            'stall_events': st.get('stall_events', 0),
            'failsafe_events': st.get('failsafe_events', 0),
            'recovery_events': st.get('recovery_events', 0),
        }
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


def save_csv(path, samples):
    if not samples:
        return
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(samples[0].keys()))
        writer.writeheader()
        writer.writerows(samples)


def main():
    os.makedirs(STALL_RES_DIR, exist_ok=True)
    os.makedirs(PLOT_DIR, exist_ok=True)
    os.makedirs(VID_DIR, exist_ok=True)

    env = os.environ.copy()
    env['PYTHONNOUSERSITE'] = '1'
    rl_lib_dir = '/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/robot_localization/lib'
    env['LD_LIBRARY_PATH'] = f"{rl_lib_dir}:{env.get('LD_LIBRARY_PATH', '')}"

    sim_cmd = [
        'ros2', 'launch',
        os.path.join(EXP_DIR, 'launch', 'base_gazebo_for_trials.launch.py'),
        'spawn_x:=-0.8', 'spawn_y:=-0.3', 'spawn_z:=0.05', 'spawn_yaw:=0.0'
    ]
    print("Launching Gazebo Fortress for WP3 Independent Stall Detector Verification...")
    p_sim = subprocess.Popen(sim_cmd, env=env, preexec_fn=os.setsid,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(7.0)

    rclpy.init()
    report = {}

    try:
        # =========================================================================
        # TEST 1: Normal Multi-Turn S-Curve Driving (False-Trigger Check)
        # =========================================================================
        print("\n[WP3 Test 1] Normal Multi-Turn Driving (False-Trigger Check)...")
        reset_gazebo_pose(-0.8, -0.3, 0.05, 0.0)
        time.sleep(0.5)

        procs = []
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen(['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'state_estimators.launch.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'stall_detector_node.py'),
                                       '--ros-args', '-p', 'enabled:=true'],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

        col1 = StallTestCollector()
        t0 = time.time()
        while time.time() - t0 < 1.5:
            rclpy.spin_once(col1, timeout_sec=0.04)

        sched1 = [
            (0.8, 0.00,  0.00, 'hold_start'),
            (3.0, 0.26,  0.00, 'straight_1'),
            (2.8, 0.22,  0.34, 'left_curve'),
            (2.8, 0.22, -0.34, 'right_align'),
            (2.5, 0.22, -0.30, 'right_curve'),
            (1.0, 0.00,  0.00, 'hold_stop'),
        ]
        for dur, vx, wz, phase in sched1:
            ts = time.time()
            last_s = 0.0
            while time.time() - ts < dur:
                col1.send_cmd_raw(vx, wz)
                rclpy.spin_once(col1, timeout_sec=0.02)
                if time.time() - last_s >= 0.05:
                    col1.record(phase)
                    last_s = time.time()
        col1.send_cmd_raw(0.0, 0.0)

        csv1 = os.path.join(STALL_RES_DIR, 'wp3_test1_normal_driving.csv')
        save_csv(csv1, col1.samples)
        false_stalls = sum(1 for r in col1.samples if r['state_name'] != 'NORMAL' or r['cmd_blocked'])
        report['test1_normal_driving'] = {
            'samples': len(col1.samples),
            'false_trigger_samples': false_stalls,
            'stall_events': col1.samples[-1]['stall_events'] if col1.samples else 0,
            'failsafe_events': col1.samples[-1]['failsafe_events'] if col1.samples else 0,
            'false_positive_rate': 0.0 if false_stalls == 0 else round(false_stalls / len(col1.samples), 4),
            'verdict': 'PASS_ZERO_FALSE_TRIGGERS' if false_stalls == 0 else 'FAIL_FALSE_TRIGGER',
            'csv_path': csv1,
        }
        print(f"  -> Test 1 Done: samples={len(col1.samples)}, false_triggers={false_stalls}")
        col1.destroy_node()
        for p in procs:
            if p and p.poll() is None:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        time.sleep(0.8)

        # =========================================================================
        # TEST 2: Glass-Wall Impact -> Command Blocking -> Stop -> Reverse Recovery
        # =========================================================================
        print("\n[WP3 Test 2] Glass-Wall Impact, Command Blocking, Physical Stop, and Reverse Recovery...")
        reset_gazebo_pose(1.0, 0.3, 0.05, -1.5708)
        time.sleep(0.5)

        procs = []
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen(['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'state_estimators.launch.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'stall_detector_node.py'),
                                       '--ros-args', '-p', 'enabled:=true'],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

        vid2_mp4 = os.path.join(VID_DIR, 'wp3_glass_wall_stall_stop_and_recovery.mp4')
        p_vid2 = subprocess.Popen([
            'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'closed_loop_video_recorder_node.py'),
            '--ros-args',
            '-p', f'output_mp4:={vid2_mp4}',
            '-p', 'scenario_title:=WP3_Stall_Detector_Glass_Wall_Stop_and_Recovery',
            '-p', 'active_estimator:=F5_imp'
        ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_vid2)

        col2 = StallTestCollector()
        t0 = time.time()
        while time.time() - t0 < 1.5:
            rclpy.spin_once(col2, timeout_sec=0.04)

        sched2 = [
            (0.8,  0.00, 0.00, 'hold_start'),
            (6.8,  0.30, 0.00, 'drive_into_glass_wall'),
            (1.6,  0.25, 0.00, 'verify_forward_blocked_while_latched'),
            (2.4, -0.22, 0.00, 'reverse_escape_recovery'),
            (2.2,  0.22, 0.42, 'restart_forward_after_recovery'),
            (1.0,  0.00, 0.00, 'hold_stop'),
        ]
        for dur, vx, wz, phase in sched2:
            ts = time.time()
            last_s = 0.0
            while time.time() - ts < dur:
                col2.send_cmd_raw(vx, wz)
                rclpy.spin_once(col2, timeout_sec=0.02)
                if time.time() - last_s >= 0.05:
                    col2.record(phase)
                    last_s = time.time()
        col2.send_cmd_raw(0.0, 0.0)

        if p_vid2.poll() is None:
            os.killpg(os.getpgid(p_vid2.pid), signal.SIGINT)
            time.sleep(3.5)

        csv2 = os.path.join(STALL_RES_DIR, 'wp3_test2_glass_wall_stall_and_recovery.csv')
        save_csv(csv2, col2.samples)

        # Analyze impact time, latch time, physical stop, and recovery
        drive_rows = [r for r in col2.samples if r['phase'] in ('drive_into_glass_wall', 'verify_forward_blocked_while_latched')]
        impact_rows = [r for r in drive_rows if r['GT_x'] > 1.35 and abs(r['GT_vx']) < 0.03]
        latch_rows = [r for r in drive_rows if r['state_name'] == 'STALL_LATCHED' and r['cmd_blocked']]
        blocked_hold_rows = [r for r in col2.samples if r['phase'] == 'verify_forward_blocked_while_latched']
        rev_rows = [r for r in col2.samples if r['phase'] == 'reverse_escape_recovery']
        restart_rows = [r for r in col2.samples if r['phase'] == 'restart_forward_after_recovery']

        t_impact = impact_rows[0]['sim_time'] if impact_rows else None
        t_latch = latch_rows[0]['sim_time'] if latch_rows else None
        det_latency_s = round(t_latch - t_impact, 4) if (t_impact and t_latch) else None

        # Verify physical stop & wheel stop during verify_forward_blocked_while_latched
        max_gt_vx_blocked = max(abs(r['GT_vx']) for r in blocked_hold_rows) if blocked_hold_rows else None
        max_b0_vx_blocked = max(abs(r['B0_vx']) for r in blocked_hold_rows) if blocked_hold_rows else None
        max_cmd_out_blocked = max(abs(r['cmd_vx_out']) for r in blocked_hold_rows) if blocked_hold_rows else None
        max_b0_x_with_blocker = max(r['B0_x'] for r in col2.samples) if col2.samples else None

        # Verify recovery & restart
        rec_states = list({r['state_name'] for r in rev_rows})
        restart_normal = any(r['state_name'] == 'NORMAL' and r['GT_vx'] > 0.12 for r in restart_rows)

        report['test2_glass_wall_impact_and_recovery'] = {
            'samples': len(col2.samples),
            't_physical_impact_s': t_impact,
            't_stall_latched_s': t_latch,
            'stall_detection_latency_s': det_latency_s,
            'command_blocking_verified': (max_cmd_out_blocked == 0.0),
            'physical_chassis_stop_verified': (max_gt_vx_blocked is not None and max_gt_vx_blocked < 0.02),
            'wheel_spin_arrest_verified': (max_b0_vx_blocked is not None and max_b0_vx_blocked < 0.03),
            'max_b0_forward_x_with_stall_detector_m': round(float(max_b0_x_with_blocker), 4) if max_b0_x_with_blocker else None,
            'max_b0_forward_x_without_stall_detector_baseline_m': 2.2300,
            'reverse_recovery_states_observed': rec_states,
            'restart_forward_after_recovery_verified': restart_normal,
            'video_mp4': vid2_mp4,
            'csv_path': csv2,
        }
        print(
            f"  -> Test 2 Done: impact={t_impact}s, latch={t_latch}s (latency={det_latency_s}s) | "
            f"cmd_blocked={max_cmd_out_blocked == 0.0}, wheel_stopped={max_b0_vx_blocked < 0.03}, "
            f"B0_max_x={max_b0_x_with_blocker:.3f}m (vs 2.230m without blocker), restart={restart_normal}"
        )
        col2.destroy_node()
        for p in procs:
            if p and p.poll() is None:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        time.sleep(0.8)

        # =========================================================================
        # TEST 3: Sensor Dropout / Uncertain Sensor State Fail-Safe Verification
        # =========================================================================
        print("\n[WP3 Test 3] Sensor Dropout Fail-Safe Verification (Killing F4 mid-drive)...")
        reset_gazebo_pose(-0.8, -0.3, 0.05, 0.0)
        time.sleep(0.5)

        procs = []
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        p_f4 = subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'lidar_scan_matcher_odom_node.py')],
                                env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_f4)
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'stall_detector_node.py'),
                                       '--ros-args', '-p', 'enabled:=true', '-p', 'sensor_timeout_sec:=0.45'],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

        col3 = StallTestCollector()
        t0 = time.time()
        while time.time() - t0 < 1.5:
            rclpy.spin_once(col3, timeout_sec=0.04)

        # Phase A: Normal forward drive for 2.5s
        ts = time.time()
        last_s = 0.0
        while time.time() - ts < 2.5:
            col3.send_cmd_raw(0.24, 0.0)
            rclpy.spin_once(col3, timeout_sec=0.02)
            if time.time() - last_s >= 0.05:
                col3.record('normal_drive_before_dropout')
                last_s = time.time()

        # Kill F4 LiDAR odometry node to simulate sensor dropout while cmd_vx_raw = +0.24 m/s continues!
        t_dropout_sim = col3.samples[-1]['sim_time'] if col3.samples else 0.0
        os.killpg(os.getpgid(p_f4.pid), signal.SIGKILL)

        ts = time.time()
        while time.time() - ts < 2.5:
            col3.send_cmd_raw(0.24, 0.0)
            rclpy.spin_once(col3, timeout_sec=0.02)
            if time.time() - last_s >= 0.05:
                col3.record('sensor_dropout_active_cmd')
                last_s = time.time()

        col3.send_cmd_raw(0.0, 0.0)
        csv3 = os.path.join(STALL_RES_DIR, 'wp3_test3_sensor_dropout_failsafe.csv')
        save_csv(csv3, col3.samples)

        fs_rows = [r for r in col3.samples if r['state_name'] == 'FAILSAFE_SENSOR_TIMEOUT' and r['cmd_blocked']]
        t_fs = fs_rows[0]['sim_time'] if fs_rows else None
        fs_latency_s = round(t_fs - t_dropout_sim, 4) if (t_fs and t_dropout_sim) else None
        late_fs_rows = [r for r in fs_rows if (r['sim_time'] - t_fs) >= 0.4] if t_fs else []
        stopped_after_fs = all(abs(r['GT_vx']) < 0.02 and r['cmd_vx_out'] == 0.0 for r in late_fs_rows) if late_fs_rows else False

        report['test3_sensor_dropout_failsafe'] = {
            'samples': len(col3.samples),
            't_sensor_dropout_s': t_dropout_sim,
            't_failsafe_triggered_s': t_fs,
            'failsafe_trigger_latency_s': fs_latency_s,
            'cmd_blocked_on_uncertainty': bool(len(fs_rows) > 0),
            'physical_stop_confirmed': stopped_after_fs,
            'csv_path': csv3,
        }
        print(
            f"  -> Test 3 Done: dropout={t_dropout_sim}s -> failsafe={t_fs}s (latency={fs_latency_s}s) | "
            f"physical_stop_confirmed={stopped_after_fs}"
        )
        col3.destroy_node()
        for p in procs:
            if p and p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except Exception:
                    pass

        out_json = os.path.join(STALL_RES_DIR, 'wp3_stall_detector_verification_summary.json')
        with open(out_json, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"\nSaved WP3 Stall Detector summary to {out_json}")

        # Plot WP3 Test 2 & Test 3 Time-Series
        fig, axes = plt.subplots(1, 2, figsize=(15, 5.2), dpi=140)
        t2 = [r['sim_time'] for r in col2.samples]
        axes[0].plot(t2, [r['cmd_vx_raw'] for r in col2.samples], '--', color='#8d99ae', label='cmd_vx_raw (Requested)')
        axes[0].plot(t2, [r['cmd_vx_out'] for r in col2.samples], '-', color='#e63946', linewidth=2, label='cmd_vx_out (Stall-Gated)')
        axes[0].plot(t2, [r['GT_vx'] for r in col2.samples], '-', color='#2ecc71', linewidth=2, label='GT_vx (Actual Chassis)')
        axes[0].plot(t2, [r['B0_vx'] for r in col2.samples], ':', color='#3a86ff', linewidth=1.8, label='B0_vx (Wheel Encoder)')
        if t_impact:
            axes[0].axvline(t_impact, color='#ffbe0b', linestyle='--', label=f'Impact ({t_impact:.2f}s)')
        if t_latch:
            axes[0].axvline(t_latch, color='#ff006e', linestyle='-', label=f'Stall Latch ({t_latch:.2f}s)')
        axes[0].set_title('WP3 Test 2: Glass-Wall Impact -> Command Block -> Reverse Recovery')
        axes[0].set_xlabel('Simulation Time (s)')
        axes[0].set_ylabel('Linear Velocity (m/s)')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc='best', fontsize=8)

        t3 = [r['sim_time'] for r in col3.samples]
        axes[1].plot(t3, [r['cmd_vx_raw'] for r in col3.samples], '--', color='#8d99ae', label='cmd_vx_raw (Requested)')
        axes[1].plot(t3, [r['cmd_vx_out'] for r in col3.samples], '-', color='#e63946', linewidth=2, label='cmd_vx_out (Fail-Safe Gated)')
        axes[1].plot(t3, [r['GT_vx'] for r in col3.samples], '-', color='#2ecc71', linewidth=2, label='GT_vx (Actual Chassis)')
        if t_dropout_sim:
            axes[1].axvline(t_dropout_sim, color='#ffbe0b', linestyle='--', label=f'Sensor Dropout ({t_dropout_sim:.2f}s)')
        if t_fs:
            axes[1].axvline(t_fs, color='#ff006e', linestyle='-', label=f'Fail-Safe Stop ({t_fs:.2f}s)')
        axes[1].set_title('WP3 Test 3: Sensor Dropout -> FAILSAFE_SENSOR_TIMEOUT Stop')
        axes[1].set_xlabel('Simulation Time (s)')
        axes[1].set_ylabel('Linear Velocity (m/s)')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='best', fontsize=8)

        plt.tight_layout()
        plt.savefig(os.path.join(PLOT_DIR, 'wp3_stall_detector_verification.png'))
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
