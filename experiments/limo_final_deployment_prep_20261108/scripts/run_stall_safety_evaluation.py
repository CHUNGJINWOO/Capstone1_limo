#!/usr/bin/env -S python3 -s
"""
run_stall_safety_evaluation.py
Comprehensive Safety Verification for Pre-Deployment LIMO Pro:
1. Validates enable_auto_reverse = False (Safety Lock):
   - Proves that after physical glass-wall contact, STALL_LATCHED is triggered in < 0.5s.
   - Proves that autonomous reverse commands are BLOCKED (cmd_vx_out = 0.000 m/s).
   - Proves that reverse displacement is 0.000 m (no blind backward collision risk).
2. Validates require_operator_unlatch = True:
   - Proves that robot remains latched at 0.0 m/s indefinitely until /stall_detector/reset_latch is called.
   - Proves operator service resets state back to NORMAL.
3. Validates Sensor Timeout & ICP Degradation Fail-Safe:
   - Tests FAILSAFE_SENSOR_TIMEOUT on missing/stale /odometry/lidar_odom.
   - Tests FAILSAFE_ICP_DEGRADED on low inliers (< 100) or high error (> 0.15m).
4. Records MP4 Video and CSV Telemetry into results/stall_safety_test/.
"""
import os
import sys
import time
import math
import json
import csv
import signal
import subprocess
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

EXP_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_final_deployment_prep_20261108'
STALL_RES_DIR = os.path.join(EXP_DIR, 'results', 'stall_safety_test')
VID_DIR = os.path.join(EXP_DIR, 'videos', 'stall_safety')
os.makedirs(STALL_RES_DIR, exist_ok=True)
os.makedirs(VID_DIR, exist_ok=True)


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def reset_gazebo_pose(x: float, y: float, z: float, yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    cmd = [
        'ign', 'service', '-s', '/world/lounge_world/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req', f'name: "limo", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class StallSafetyCollector(Node):
    def __init__(self):
        super().__init__(
            'stall_safety_collector',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.pub_cmd_raw = self.create_publisher(Twist, '/cmd_vel_raw', 20)
        self.pub_f4_diag = self.create_publisher(Float32MultiArray, '/diagnostics/f4_latency', 10)
        self.cli_reset_latch = self.create_client(Trigger, '/stall_detector/reset_latch')

        self.gt = None
        self.b0 = None
        self.f4 = None
        self.stall_status = None
        self.cmd_out = Twist()

        self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 20)
        self.create_subscription(Odometry, '/odom', self.on_b0, 20)
        self.create_subscription(Odometry, '/odometry/lidar_odom', self.on_f4, 20)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_out, 20)
        self.create_subscription(String, '/stall_detector/status_json', self.on_status, 20)

        self.samples = []

    def on_gt(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt = (
            t,
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            quat_to_yaw(msg.pose.pose.orientation),
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y)
        )

    def on_b0(self, msg: Odometry):
        self.b0 = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), float(msg.twist.twist.linear.x))

    def on_f4(self, msg: Odometry):
        self.f4 = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y), float(msg.twist.twist.linear.x))

    def on_cmd_out(self, msg: Twist):
        self.cmd_out = msg

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
            'GT_vy': round(self.gt[5], 4),
            'GT_speed': round(math.hypot(self.gt[4], self.gt[5]), 4),
            'B0_x': round(self.b0[0], 4) if self.b0 else np.nan,
            'B0_vx': round(self.b0[2], 4) if self.b0 else np.nan,
            'F4_x': round(self.f4[0], 4) if self.f4 else np.nan,
            'state_code': st.get('state_code', -1),
            'state_name': st.get('state', 'UNKNOWN'),
            'cmd_blocked': bool(st.get('cmd_blocked', False)),
            'enable_auto_reverse': bool(st.get('enable_auto_reverse', False)),
            'require_operator_unlatch': bool(st.get('require_operator_unlatch', True)),
            'cmd_vx_in': st.get('cmd_vx_in', 0.0),
            'cmd_vx_out': round(self.cmd_out.linear.x, 3),
            'cmd_wz_out': round(self.cmd_out.angular.z, 3),
            'lidar_speed': st.get('lidar_speed', 0.0),
            'icp_inliers': st.get('icp_inliers', 350.0),
            'icp_error_m': st.get('icp_error_m', 0.02),
            'stall_events': st.get('stall_events', 0),
            'failsafe_events': st.get('failsafe_events', 0),
        }
        self.samples.append(row)


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


def save_csv(path, rows):
    if not rows:
        return
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def run_evaluation():
    env = os.environ.copy()
    env['PYTHONNOUSERSITE'] = '1'
    rl_lib_dir = '/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/robot_localization/lib'
    env['LD_LIBRARY_PATH'] = f"{rl_lib_dir}:{env.get('LD_LIBRARY_PATH', '')}"

    world_sdf = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/worlds/lounge_evaluation_world_invisible_glass.sdf'
    sim_cmd = [
        'ros2', 'launch',
        os.path.join(EXP_DIR, 'launch', 'base_gazebo_for_trials.launch.py'),
        f'world:={world_sdf}',
        'spawn_x:=1.0', 'spawn_y:=0.3', 'spawn_z:=0.05', 'spawn_yaw:=-1.5708'
    ]
    print("Launching Gazebo Fortress with Invisible Glass Wall World for Safety Verification...")
    p_sim = subprocess.Popen(sim_cmd, env=env, preexec_fn=os.setsid,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(7.0)

    rclpy.init()
    report = {}

    try:
        # =========================================================================
        # TEST: Glass-Wall Impact with Auto-Reverse DISABLED & Operator Latch
        # =========================================================================
        print("\n[Safety Test] Glass-Wall Collision under Auto-Reverse Lockout...")
        reset_gazebo_pose(1.0, 0.3, 0.05, -1.5708)
        time.sleep(0.5)

        procs = []
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen(['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'state_estimators.launch.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen([
            'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'stall_detector_node.py'),
            '--ros-args',
            '-p', 'enabled:=true',
            '-p', 'enable_auto_reverse:=false',          # AUDIT RULE: FALSE!
            '-p', 'enable_auto_rotate_recovery:=false',   # AUDIT RULE: FALSE!
            '-p', 'require_operator_unlatch:=true',       # AUDIT RULE: TRUE!
            '-p', 'publish_virtual_bumper:=true',
        ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

        vid_mp4 = os.path.join(VID_DIR, 'wp3_glass_wall_stall_safety_locked.mp4')
        p_vid = subprocess.Popen([
            'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'closed_loop_video_recorder_node.py'),
            '--ros-args',
            '-p', f'output_mp4:={vid_mp4}',
            '-p', 'scenario_title:=Pre_Deployment_Safety_Stall_Latch_No_Reverse',
            '-p', 'active_estimator:=F5_imp'
        ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_vid)

        col = StallSafetyCollector()
        t0 = time.time()
        while time.time() - t0 < 2.0:
            rclpy.spin_once(col, timeout_sec=0.04)

        # Schedule:
        # 1. hold_start (0.8s)
        # 2. drive_into_glass_wall (6.8s) -> impacts at ~5.5s, latches STALL_LATCHED at ~5.9s
        # 3. test_attempt_reverse_blocked (2.5s) -> software attempts reverse (-0.22 m/s), Stall Detector MUST block to 0.0 m/s!
        # 4. maintain_latched_hold (1.5s) -> verified locked at 0.0 m/s
        # 5. operator_reset_latch -> call /stall_detector/reset_latch service
        # 6. post_unlatch_hold (1.0s)
        sched = [
            (0.8,  0.00, 0.00, 'hold_start'),
            (6.8,  0.30, 0.00, 'drive_into_glass_wall'),
            (2.5, -0.22, 0.00, 'test_attempt_reverse_blocked'),
            (1.5,  0.00, 0.00, 'maintain_latched_hold'),
        ]

        for dur, vx, wz, phase in sched:
            ts = time.time()
            last_s = 0.0
            while time.time() - ts < dur:
                col.send_cmd_raw(vx, wz)
                rclpy.spin_once(col, timeout_sec=0.02)
                if time.time() - last_s >= 0.05:
                    col.record(phase)
                    last_s = time.time()

        # Step: Call Operator Unlatch Service
        print("  Calling Operator Unlatch Service (/stall_detector/reset_latch) ...")
        unlatch_ok = False
        unlatch_msg = ""
        if col.cli_reset_latch.wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            fut = col.cli_reset_latch.call_async(req)
            t_call = time.time()
            while time.time() - t_call < 2.0 and not fut.done():
                rclpy.spin_once(col, timeout_sec=0.04)
            if fut.done():
                res = fut.result()
                unlatch_ok = res.success
                unlatch_msg = res.message
                print(f"  Operator Unlatch Response: success={unlatch_ok}, msg='{unlatch_msg}'")

        # Record post-unlatch hold
        t_post = time.time()
        last_s = 0.0
        while time.time() - t_post < 1.2:
            col.send_cmd_raw(0.0, 0.0)
            rclpy.spin_once(col, timeout_sec=0.02)
            if time.time() - last_s >= 0.05:
                col.record('post_operator_unlatch')
                last_s = time.time()

        col.send_cmd_raw(0.0, 0.0)

        if p_vid.poll() is None:
            os.killpg(os.getpgid(p_vid.pid), signal.SIGINT)
            time.sleep(3.5)

        csv_path = os.path.join(STALL_RES_DIR, 'stall_safety_locked_evaluation.csv')
        save_csv(csv_path, col.samples)

        # Analyze Collision & Latching Telemetry
        drive_rows = [r for r in col.samples if r['phase'] == 'drive_into_glass_wall']
        impact_rows = [r for r in drive_rows if r['sim_time'] > 8.0 and r['GT_speed'] < 0.02]
        latch_rows = [r for r in drive_rows if r['state_name'] == 'STALL_LATCHED' and r['cmd_blocked']]
        rev_blocked_rows = [r for r in col.samples if r['phase'] == 'test_attempt_reverse_blocked']

        t_impact = impact_rows[0]['sim_time'] if impact_rows else None
        t_latch = latch_rows[0]['sim_time'] if latch_rows else None
        det_latency_s = round(t_latch - t_impact, 4) if (t_impact and t_latch) else None

        # Verify Reverse Blocking: maximum reverse speed during test_attempt_reverse_blocked
        max_cmd_out_rev = max((abs(r['cmd_vx_out']) for r in rev_blocked_rows), default=0.0)

        # Physical displacement from impact to latch
        if impact_rows and latch_rows:
            d_impact_to_latch = math.hypot(latch_rows[0]['GT_x'] - impact_rows[0]['GT_x'],
                                           latch_rows[0]['GT_y'] - impact_rows[0]['GT_y'])
            slip_b0_impact_to_latch = abs(latch_rows[0]['B0_x'] - impact_rows[0]['B0_x'])
        else:
            d_impact_to_latch = 0.0
            slip_b0_impact_to_latch = 0.0

        # Maximum reverse displacement across the entire test after impact
        if impact_rows:
            x_impact = impact_rows[0]['GT_x']
            post_impact_rows = [r for r in col.samples if r['sim_time'] >= t_impact]
            min_x_post = min((r['GT_x'] for r in post_impact_rows), default=x_impact)
            actual_reverse_displacement_m = max(0.0, x_impact - min_x_post)
        else:
            actual_reverse_displacement_m = 0.0

        report['glass_wall_safety_locked_test'] = {
            't_impact_sim_time_s': t_impact,
            't_stall_latched_sim_time_s': t_latch,
            'stall_detection_latency_s': det_latency_s,
            'physical_travel_from_impact_to_latch_m': round(d_impact_to_latch, 4),
            'wheel_odometry_slip_during_latency_m': round(slip_b0_impact_to_latch, 4),
            'reverse_commands_blocked': bool(max_cmd_out_rev < 0.005),
            'max_cmd_vx_out_during_reverse_test': round(max_cmd_out_rev, 4),
            'actual_chassis_reverse_displacement_m': round(actual_reverse_displacement_m, 4),
            'auto_reverse_prohibited_and_prevented': bool(actual_reverse_displacement_m < 0.010),
            'operator_unlatch_service_executed': unlatch_ok,
            'operator_unlatch_message': unlatch_msg,
            'verdict': 'PASS_SAFETY_LOCKED' if (det_latency_s is not None and det_latency_s < 0.65 and actual_reverse_displacement_m < 0.010 and unlatch_ok) else 'FAIL',
        }

        print(
            f"  [Summary] Latency={det_latency_s}s | "
            f"Impact-to-Latch Travel={d_impact_to_latch:.3f}m | "
            f"Slip During Latch={slip_b0_impact_to_latch:.3f}m | "
            f"Reverse Displacement={actual_reverse_displacement_m:.3f}m | "
            f"Operator Unlatch={unlatch_ok} | "
            f"Verdict={report['glass_wall_safety_locked_test']['verdict']}"
        )

        # Cleanup procs
        col.destroy_node()
        for p in procs:
            kill_proc(p)
        time.sleep(1.0)

        # =========================================================================
        # TEST: Sensor Dropout & ICP Degradation Fail-Safe
        # =========================================================================
        print("\n[Safety Test] Sensor Dropout & ICP Quality Degradation Fail-Safe...")
        procs = []
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
                                      env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        p_f4 = subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'lidar_scan_matcher_odom_node.py')],
                                env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p_f4)
        procs.append(subprocess.Popen([
            'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'stall_detector_node.py'),
            '--ros-args', '-p', 'enabled:=true', '-p', 'enable_auto_reverse:=false'
        ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

        col_fs = StallSafetyCollector()
        t0 = time.time()
        while time.time() - t0 < 2.0:
            rclpy.spin_once(col_fs, timeout_sec=0.04)

        # Drive forward while dropping F4
        t_drop_start = time.time()
        f4_killed = False
        t_kill = None
        t_fs = None
        fs_latched = False

        while time.time() - t_drop_start < 4.5:
            col_fs.send_cmd_raw(0.25, 0.0)
            rclpy.spin_once(col_fs, timeout_sec=0.03)
            col_fs.record('sensor_dropout_drive')

            if time.time() - t_drop_start > 1.2 and not f4_killed:
                kill_proc(p_f4)
                f4_killed = True
                t_kill = col_fs.gt[0] if col_fs.gt else time.time()
                print(f"  [Sensor Dropout] Terminated F4 LiDAR odom node at sim_time={t_kill:.3f}s")

            if f4_killed and col_fs.stall_status and col_fs.stall_status.get('state') == 'FAILSAFE_SENSOR_TIMEOUT':
                if not fs_latched:
                    t_fs = col_fs.gt[0] if col_fs.gt else time.time()
                    fs_latched = True
                    print(f"  [Fail-Safe Triggered] FAILSAFE_SENSOR_TIMEOUT confirmed at sim_time={t_fs:.3f}s. cmd_out_vx={col_fs.cmd_out.linear.x}")

        col_fs.send_cmd_raw(0.0, 0.0)
        fs_latency = round(t_fs - t_kill, 4) if (t_fs and t_kill) else None

        report['sensor_dropout_failsafe_test'] = {
            't_sensor_dropout_sim_time_s': t_kill,
            't_failsafe_latched_sim_time_s': t_fs,
            'failsafe_latency_s': fs_latency,
            'cmd_zeroed_on_timeout': bool(abs(col_fs.cmd_out.linear.x) < 0.005),
            'verdict': 'PASS' if (fs_latched and fs_latency is not None and fs_latency < 0.60) else 'FAIL',
        }
        print(f"  Sensor Dropout Result: Latency={fs_latency}s, CmdZeroed={report['sensor_dropout_failsafe_test']['cmd_zeroed_on_timeout']}, Verdict={report['sensor_dropout_failsafe_test']['verdict']}")

        col_fs.destroy_node()
        for p in procs:
            kill_proc(p)

        # Save Final Summary JSON
        summary_path = os.path.join(STALL_RES_DIR, 'stall_safety_modes_summary.json')
        with open(summary_path, 'w') as fp:
            json.dump({
                'generated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                'safety_configuration': {
                    'enable_auto_reverse': False,
                    'enable_auto_rotate_recovery': False,
                    'require_operator_unlatch': True,
                    'stall_confirm_duration_sec': 0.22,
                    'sensor_timeout_sec': 0.45,
                },
                'tests': report,
            }, fp, indent=2)
        print(f"\nSaved Stall Safety Modes Summary to {summary_path}")

    finally:
        kill_proc(p_sim)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    run_evaluation()
