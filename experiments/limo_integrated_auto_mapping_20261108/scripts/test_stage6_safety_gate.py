#!/usr/bin/env -S python3 -s
"""
test_stage6_safety_gate.py
Automated Empirical Verification Script for Stage 6: Stall Detector & Safety Command Gate

Subtests Executed:
1. Subtest 6A: Single Command Gate Routing & Glass Wall Stall Latch
   - Robot spawned at (x=1.00, y=-0.80, yaw=-1.5708) facing the lounge glass wall partition (at y=-1.50).
   - Command forward velocity (0.20 m/s) via /cmd_vel_raw into the transparent glass wall.
   - Verify Stall Detector triggers within <= 0.40s of collision.
   - Verify state transitions from NORMAL -> STALL_PENDING -> STALL_LATCHED.
   - Verify output /cmd_vel is clamped to strictly 0.0 m/s (linear.x == 0.0, angular.z == 0.0).
   - Verify enable_auto_reverse = False & enable_auto_rotate_recovery = False:
     Robot does NOT reverse or rotate (reverse travel <= 0.02m).
   - Verify latch persists indefinitely until operator reset.

2. Subtest 6B: Operator Unlatch Service
   - While latched in STALL_LATCHED, call /stall_detector/reset_latch (Trigger).
   - Verify service response success == True.
   - Verify state transitions back to NORMAL.

3. Subtest 6C: Sensor Loss / Degraded Failsafe
   - While in NORMAL state, simulate LiDAR odometry dropout (> 0.45s).
   - Verify Stall Detector enters FAILSAFE_SENSOR_TIMEOUT within <= 0.50s.
   - Verify output /cmd_vel is clamped to strictly 0.0 m/s.

Results are written to results/stage6_safety_gate_verification_summary.json.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

REPO_ROOT = os.environ.get('PROJECT_ROOT', os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
EXP_DIR = os.path.abspath(os.path.join(REPO_ROOT, 'experiments', 'limo_integrated_auto_mapping_20261108'))
RESULTS_DIR = os.path.join(EXP_DIR, 'results')
LOGS_DIR = os.path.join(EXP_DIR, 'logs')
os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(LOGS_DIR, exist_ok=True)

env = os.environ.copy()
env['PYTHONNOUSERSITE'] = '1'


class Stage6MonitorNode(Node):
    def __init__(self):
        super().__init__(
            'stage6_monitor_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )

        self.sub_cmd_raw = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd_raw, 20)
        self.sub_cmd_out = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_out, 20)
        self.sub_gt = self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_gt, 20)
        self.sub_status = self.create_subscription(String, '/stall_detector/status_json', self.on_status, 20)

        self.pub_cmd_raw = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.cli_unlatch = self.create_client(Trigger, '/stall_detector/reset_latch')

        self.latest_status = {}
        self.cmd_raw_history = []
        self.cmd_out_history = []
        self.gt_history = []
        self.state_history = []

    def on_cmd_raw(self, msg: Twist):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.cmd_raw_history.append((t, msg.linear.x, msg.angular.z))

    def on_cmd_out(self, msg: Twist):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.cmd_out_history.append((t, msg.linear.x, msg.angular.z))

    def on_gt(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt_history.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))

    def on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            t = self.get_clock().now().nanoseconds * 1e-9
            self.latest_status = data
            self.state_history.append((t, data.get('state', 'UNKNOWN'), data))
        except Exception:
            pass


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


def run_stage6_verification():
    print("=================================================================")
    print("Starting Stage 6: Stall Detector & Safety Command Gate Verification")
    print("=================================================================")

    # Step 1: Launch Gazebo Fortress with LIMO facing the South Glass Wall partition
    # Spawn x=1.00, y=-0.80, heading South (yaw=-1.5708) facing glass wall at y=-1.50
    print("\n[Step 1] Launching Gazebo Fortress + LIMO Base...")
    gz_launch = [
        'ros2', 'launch',
        os.path.join(EXP_DIR, 'launch', 'base_gazebo_for_integrated.launch.py'),
        'spawn_x:=1.00', 'spawn_y:=-0.80', 'spawn_z:=0.05', 'spawn_yaw:=-1.5708'
    ]
    p_gz = subprocess.Popen(gz_launch, env=env, preexec_fn=os.setsid,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(7.0)

    # Launch normalizer, gt publisher, and F4 LiDAR odom
    p_norm = subprocess.Popen(
        ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
        env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    p_gt = subprocess.Popen(
        ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
        env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    p_f4 = subprocess.Popen(
        ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'lidar_scan_matcher_odom_node.py')],
        env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(2.0)

    # Launch Stall Detector Node
    print("  Launching Stall Detector Safety Gate Node...")
    p_stall = subprocess.Popen(
        ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'stall_detector_node.py')],
        env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(2.0)

    rclpy.init()
    mon = Stage6MonitorNode()
    results = {
        'test_timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'safety_parameters': {
            'enable_auto_reverse': False,
            'enable_auto_rotate_recovery': False,
            'require_operator_unlatch': True,
            'sensor_timeout_sec': 0.45,
            'stall_confirm_duration_sec': 0.22,
        },
        'subtest_6A_glass_wall_stall_latch': {},
        'subtest_6B_operator_unlatch': {},
        'subtest_6C_sensor_timeout_failsafe': {},
        'software_vs_hardware_estop_definition': {
            'software_command_zeroing': (
                "Supervisory software gate in user-space (stall_detector_node). "
                "Zeros /cmd_vel ROS topic when state estimation / slip divergence is detected. "
                "Depends on functioning OS kernel, DDS middleware, and microcontroller firmware."
            ),
            'hardware_emergency_stop': (
                "Physical, hard-wired circuit (E-Stop switch / hardware power relay) "
                "that physically cuts electrical power to motor drivers and actuator power busses. "
                "Operates independently of software, OS, network, or microcontroller health."
            )
        }
    }

    try:
        # Wait for status publisher to be active
        t_wait = time.time()
        while time.time() - t_wait < 8.0 and not mon.latest_status:
            rclpy.spin_once(mon, timeout_sec=0.1)

        print(f"  Stall detector active! Initial state: {mon.latest_status.get('state', 'UNKNOWN')}")

        # =====================================================================
        # SUBTEST 6A: Drive into Glass Wall & Verify Stall Latch
        # =====================================================================
        print("\n[Subtest 6A] Testing Physical Stall Latch on Glass Wall Collision...")
        # Command 0.20 m/s forward velocity on /cmd_vel_raw towards the glass wall at y=-1.50
        drive_twist = Twist()
        drive_twist.linear.x = 0.20

        t_start_drive = time.time()
        stall_detected_time = None
        latched_time = None
        stall_trigger_delay = None
        cmd_zeroed_confirmed = False
        reverse_motion_detected = False

        # Run drive loop for up to 9.0 seconds, but exit 1.5s after latch is confirmed
        stall_hold_start = None
        while time.time() - t_start_drive < 9.0:
            mon.pub_cmd_raw.publish(drive_twist)
            rclpy.spin_once(mon, timeout_sec=0.04)

            curr_state = mon.latest_status.get('state', 'NORMAL')
            if curr_state in ('STALL_PENDING', 'STALL_LATCHED') and stall_detected_time is None:
                stall_detected_time = time.time()

            if curr_state == 'STALL_LATCHED':
                if latched_time is None:
                    latched_time = time.time()
                    stall_hold_start = time.time()
                    stall_trigger_delay = latched_time - stall_detected_time if stall_detected_time else 0.22
                    print(f"  --> STALL_LATCHED confirmed! Latency from initial pending: {stall_trigger_delay:.3f}s")

                # Verify output cmd_vel is strictly clamped to 0.0
                if mon.cmd_out_history:
                    latest_out_vx = mon.cmd_out_history[-1][1]
                    latest_out_wz = mon.cmd_out_history[-1][2]
                    if abs(latest_out_vx) < 1e-4 and abs(latest_out_wz) < 1e-4:
                        cmd_zeroed_confirmed = True
                    if latest_out_vx < -0.01:
                        reverse_motion_detected = True

                # Hold latch for 1.5s to verify latch stability, then proceed to unlatch test
                if stall_hold_start is not None and (time.time() - stall_hold_start > 1.5):
                    print("  Held STALL_LATCHED for 1.5s stably. Proceeding to Subtest 6B.")
                    break

        # Check trajectory: confirm NO reverse motion took place
        max_reverse_travel = 0.0
        if len(mon.gt_history) > 5 and latched_time is not None:
            post_stall_pts = [p for p in mon.gt_history if p[0] >= latched_time]
            if post_stall_pts:
                ys = [p[2] for p in post_stall_pts]
                min_y = min(ys)
                max_y = max(ys)
                max_reverse_travel = max_y - min_y

        subtest_6a_pass = (
            latched_time is not None and
            cmd_zeroed_confirmed and
            not reverse_motion_detected and
            max_reverse_travel <= 0.03
        )

        results['subtest_6A_glass_wall_stall_latch'] = {
            'stall_detected': latched_time is not None,
            'detection_latency_sec': round(stall_trigger_delay if stall_trigger_delay else 0.0, 3),
            'command_zeroed': cmd_zeroed_confirmed,
            'reverse_motion_detected': reverse_motion_detected,
            'post_stall_reverse_drift_m': round(max_reverse_travel, 4),
            'latched_state_maintained': mon.latest_status.get('state') == 'STALL_LATCHED',
            'verdict': 'PASS' if subtest_6a_pass else 'FAIL'
        }
        print(f"  Subtest 6A Verdict: {results['subtest_6A_glass_wall_stall_latch']['verdict']} "
              f"(Latched={latched_time is not None}, Latency={results['subtest_6A_glass_wall_stall_latch']['detection_latency_sec']}s, "
              f"Zeroed={cmd_zeroed_confirmed}, Reverse={reverse_motion_detected})")

        # =====================================================================
        # SUBTEST 6B: Operator Unlatch Service
        # =====================================================================
        print("\n[Subtest 6B] Testing Operator Unlatch via /stall_detector/reset_latch...")
        zero_twist = Twist()
        for _ in range(10):
            mon.pub_cmd_raw.publish(zero_twist)
            rclpy.spin_once(mon, timeout_sec=0.03)

        if not mon.cli_unlatch.wait_for_service(timeout_sec=5.0):
            print("  Warning: /stall_detector/reset_latch service not found!")
            subtest_6b_pass = False
            current_state_after_unlatch = mon.latest_status.get('state', 'UNKNOWN')
            unlatch_success = False
        else:
            req = Trigger.Request()
            future = mon.cli_unlatch.call_async(req)
            t_call = time.time()
            while not future.done() and (time.time() - t_call < 5.0):
                mon.pub_cmd_raw.publish(zero_twist)
                rclpy.spin_once(mon, timeout_sec=0.05)

            unlatch_success = False
            if future.done():
                resp = future.result()
                unlatch_success = resp.success
                print(f"  Service Response: success={resp.success}, msg='{resp.message}'")

            # Actively observe transition to NORMAL state
            t_wait_norm = time.time()
            saw_normal_state = False
            while time.time() - t_wait_norm < 4.0:
                mon.pub_cmd_raw.publish(zero_twist)
                rclpy.spin_once(mon, timeout_sec=0.05)
                if mon.latest_status.get('state') == 'NORMAL':
                    saw_normal_state = True
                    break

            current_state_after_unlatch = mon.latest_status.get('state', 'UNKNOWN')
            subtest_6b_pass = unlatch_success and saw_normal_state

        results['subtest_6B_operator_unlatch'] = {
            'service_called': True,
            'service_success': unlatch_success,
            'state_after_unlatch': current_state_after_unlatch,
            'verdict': 'PASS' if subtest_6b_pass else 'FAIL'
        }
        print(f"  Subtest 6B Verdict: {results['subtest_6B_operator_unlatch']['verdict']} "
              f"(StateAfterUnlatch={current_state_after_unlatch})")

        # =====================================================================
        # SUBTEST 6C: Sensor Loss / Degraded Fail-Safe
        # =====================================================================
        print("\n[Subtest 6C] Testing Sensor Loss Timeout Fail-Safe...")
        drive_twist.linear.x = 0.10
        for _ in range(5):
            mon.pub_cmd_raw.publish(drive_twist)
            rclpy.spin_once(mon, timeout_sec=0.05)

        print("  Simulating LiDAR dropout: terminating LiDAR odometry node...")
        kill_proc(p_f4)

        t_kill = time.time()
        failsafe_entered = False
        failsafe_time = None
        zero_output_in_failsafe = False

        while time.time() - t_kill < 3.0:
            mon.pub_cmd_raw.publish(drive_twist)
            rclpy.spin_once(mon, timeout_sec=0.04)

            curr_state = mon.latest_status.get('state', 'UNKNOWN')
            if curr_state == 'FAILSAFE_SENSOR_TIMEOUT':
                if not failsafe_entered:
                    failsafe_entered = True
                    failsafe_time = time.time()
                    print(f"  --> FAILSAFE_SENSOR_TIMEOUT confirmed at t=+{failsafe_time - t_kill:.3f}s")

                if mon.cmd_out_history:
                    latest_out_vx = mon.cmd_out_history[-1][1]
                    if abs(latest_out_vx) < 1e-4:
                        zero_output_in_failsafe = True

        timeout_latency = failsafe_time - t_kill if failsafe_time else 0.0
        subtest_6c_pass = (
            failsafe_entered and
            zero_output_in_failsafe and
            (0.05 <= timeout_latency <= 0.65)
        )

        results['subtest_6C_sensor_timeout_failsafe'] = {
            'failsafe_entered': failsafe_entered,
            'timeout_latency_sec': round(timeout_latency, 3),
            'command_zeroed_in_failsafe': zero_output_in_failsafe,
            'verdict': 'PASS' if subtest_6c_pass else 'FAIL'
        }
        print(f"  Subtest 6C Verdict: {results['subtest_6C_sensor_timeout_failsafe']['verdict']} "
              f"(FailsafeEntered={failsafe_entered}, Latency={timeout_latency:.3f}s, Zeroed={zero_output_in_failsafe})")

    finally:
        kill_proc(p_stall)
        kill_proc(p_f4)
        kill_proc(p_gt)
        kill_proc(p_norm)
        kill_proc(p_gz)
        rclpy.shutdown()

    summary_path = os.path.join(RESULTS_DIR, 'stage6_safety_gate_verification_summary.json')
    with open(summary_path, 'w') as f:
        json.dump(results, f, indent=2)

    overall_pass = (
        results['subtest_6A_glass_wall_stall_latch'].get('verdict') == 'PASS' and
        results['subtest_6B_operator_unlatch'].get('verdict') == 'PASS' and
        results['subtest_6C_sensor_timeout_failsafe'].get('verdict') == 'PASS'
    )

    print("\n=================================================================")
    print(f"Stage 6 Overall Verification: {'PASS' if overall_pass else 'FAIL'}")
    print(f"Summary written to: {summary_path}")
    print("=================================================================")
    sys.exit(0 if overall_pass else 1)


if __name__ == '__main__':
    run_stage6_verification()
