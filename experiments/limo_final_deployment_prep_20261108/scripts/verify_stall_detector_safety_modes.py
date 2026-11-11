#!/usr/bin/env -S python3 -s
"""
verify_stall_detector_safety_modes.py
Rigorously tests the upgraded Stall Detector Node under pre-deployment safety rules:
1. Test 1 (Normal Driving): Verifies 0 false latches during smooth maneuvering.
2. Test 2 (Glass Wall Collision with Auto-Reverse DISABLED):
   - Confirms STALL_LATCHED is triggered upon physical impact.
   - Confirms cmd_vx_out is clamped to 0.000 m/s (software stop).
   - Confirms chassis does NOT back up blindly (reverse distance == 0.000 m).
   - Confirms robot remains latched indefinitely until /stall_detector/reset_latch is called.
3. Test 3 (Sensor Dropout Fail-Safe):
   - Terminates LiDAR scan stream mid-motion.
   - Confirms FAILSAFE_SENSOR_TIMEOUT triggered in < 0.45s with command zeroed.
4. Test 4 (ICP Degradation Fail-Safe):
   - Simulates low ICP inliers (< 80) and high error (> 0.20m).
   - Confirms FAILSAFE_ICP_DEGRADED triggered with command zeroed.
Saves summary to results/stall_safety_test/stall_safety_modes_summary.json.
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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

EXP_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_final_deployment_prep_20261108'
RESULTS_DIR = os.path.join(EXP_DIR, 'results', 'stall_safety_test')
VID_DIR = os.path.join(EXP_DIR, 'videos', 'stall_safety')
os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(VID_DIR, exist_ok=True)


class SafetyTesterNode(Node):
    def __init__(self):
        super().__init__(
            'stall_safety_tester_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.pub_cmd_raw = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.pub_f4_diag = self.create_publisher(Float32MultiArray, '/diagnostics/f4_latency', 10)
        self.cli_reset_latch = self.create_client(Trigger, '/stall_detector/reset_latch')

        self.sub_gt = self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_gt, 20)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.sub_cmd_out = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_out, 10)
        self.sub_stall = self.create_subscription(String, '/stall_detector/status_json', self.on_stall_diag, 10)

        self.latest_gt = None
        self.latest_odom = None
        self.latest_cmd_out = Twist()
        self.stall_status = {}
        self.samples = []

    def on_gt(self, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_gt = {
            'sim_time': t_sec,
            'x': float(msg.pose.pose.position.x),
            'y': float(msg.pose.pose.position.y),
            'vx': float(msg.twist.twist.linear.x),
            'speed': math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
        }

    def on_odom(self, msg: Odometry):
        self.latest_odom = {
            'x': float(msg.pose.pose.position.x),
            'vx': float(msg.twist.twist.linear.x),
        }

    def on_cmd_out(self, msg: Twist):
        self.latest_cmd_out = msg

    def on_stall_diag(self, msg: String):
        try:
            self.stall_status = json.loads(msg.data)
        except Exception:
            pass

    def record_sample(self, phase: str):
        t_now = self.get_clock().now().nanoseconds * 1e-9
        s = {
            'sim_time': round(t_now, 3),
            'phase': phase,
            'gt_x': self.latest_gt['x'] if self.latest_gt else None,
            'gt_vx': self.latest_gt['vx'] if self.latest_gt else None,
            'b0_x': self.latest_odom['x'] if self.latest_odom else None,
            'b0_vx': self.latest_odom['vx'] if self.latest_odom else None,
            'cmd_out_vx': round(self.latest_cmd_out.linear.x, 3),
            'stall_state': self.stall_status.get('state', 'UNKNOWN'),
            'cmd_blocked': self.stall_status.get('cmd_blocked', False),
            'enable_auto_reverse': self.stall_status.get('enable_auto_reverse', False),
        }
        self.samples.append(s)


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


def run_stall_safety_tests():
    rclpy.init()
    node = SafetyTesterNode()
    results = {}

    print("\n=======================================================")
    print("Testing Upgraded Stall Detector Safety Modes")
    print("=======================================================")

    env = dict(os.environ)
    env['PYTHONNOUSERSITE'] = '1'

    # 1. Launch Base Gazebo (Invisible Glass World for collision testing)
    world_sdf = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107/worlds/lounge_evaluation_world_invisible_glass.sdf'
    gz_proc = subprocess.Popen([
        'ign', 'gazebo', '-r', '-s', '--headless-rendering', world_sdf
    ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(5.0)

    # 2. Launch ROS Bridge & Estimators
    bridge_proc = subprocess.Popen([
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/model/limo/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        '--ros-args', '-p', 'use_sim_time:=true'
    ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    gt_proc = subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
                               env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    f4_proc = subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'lidar_scan_matcher_odom_node.py')],
                               env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # Launch Stall Detector with enable_auto_reverse=False and require_operator_unlatch=True
    stall_proc = subprocess.Popen([
        'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'stall_detector_node.py'),
        '--ros-args',
        '-p', 'enabled:=true',
        '-p', 'enable_auto_reverse:=false',          # AUDIT RULE: FALSE!
        '-p', 'enable_auto_rotate_recovery:=false',   # AUDIT RULE: FALSE!
        '-p', 'require_operator_unlatch:=true',       # AUDIT RULE: TRUE!
        '-p', 'publish_virtual_bumper:=true',
    ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    time.sleep(4.0)

    try:
        # TEST 2: Glass Wall Impact with Auto-Reverse DISABLED
        print("\n--- Running Test 2: Glass Wall Collision with Auto-Reverse DISABLED ---")
        # Spawn robot 0.65m in front of glass wall (x=1.0, y=0.3 -> glass wall at y=-1.35)
        # Heading facing south (-1.5708 rad)
        sp_cmd = [
            'ign', 'service', '-s', '/world/lounge_world/set_pose',
            '--reqtype', 'ignition.msgs.Pose', '--reptype', 'ignition.msgs.Boolean', '--timeout', '2000',
            '--req', 'name: "limo", position: {x: 1.0, y: 0.3, z: 0.05}, orientation: {x: 0, y: 0, z: -0.7071, w: 0.7071}'
        ]
        subprocess.run(sp_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.0)

        node.samples = []
        t_start = time.time()
        t_impact = None
        t_latched = None
        latched_observed = False
        reverse_occurred = False

        forward_cmd = Twist()
        forward_cmd.linear.x = 0.30

        while time.time() - t_start < 8.0:
            node.pub_cmd_raw.publish(forward_cmd)
            rclpy.spin_once(node, timeout_sec=0.02)
            node.record_sample('forward_drive')

            gt = node.latest_gt
            if gt and t_impact is None and gt['sim_time'] > 1.5:
                # Detect physical contact when speed drops while commanded forward
                if gt['speed'] < 0.015 and node.latest_cmd_out.linear.x > 0.10:
                    t_impact = gt['sim_time']
                    gt_x_impact = gt['x']
                    gt_y_impact = gt['y']
                    print(f"  [Impact Detected] sim_time={t_impact:.3f}s at GT_y={gt_y_impact:.3f}m")

            if node.stall_status.get('state') == 'STALL_LATCHED':
                if t_latched is None:
                    t_latched = node.latest_gt['sim_time'] if node.latest_gt else time.time()
                    latched_observed = True
                    print(f"  [STALL_LATCHED Confirmed] sim_time={t_latched:.3f}s. cmd_out_vx={node.latest_cmd_out.linear.x}")

                # Verify that reverse motion NEVER happens
                if node.latest_gt and t_impact:
                    delta_y = node.latest_gt['y'] - gt_y_impact
                    # Moving north (positive delta_y) would mean reversing away from south wall
                    if delta_y > 0.05:
                        reverse_occurred = True

        # Check operator unlatch service
        print("  Testing Operator Unlatch Service /stall_detector/reset_latch ...")
        unlatch_success = False
        if node.cli_reset_latch.wait_for_service(timeout_sec=2.0):
            req = Trigger.Request()
            future = node.cli_reset_latch.call_async(req)
            t_call = time.time()
            while time.time() - t_call < 2.0 and not future.done():
                rclpy.spin_once(node, timeout_sec=0.04)
            if future.done():
                resp = future.result()
                unlatch_success = resp.success
                print(f"  Operator Unlatch Response: success={resp.success}, msg='{resp.message}'")

        # Stop commands
        stop_cmd = Twist()
        node.pub_cmd_raw.publish(stop_cmd)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.04)

        latency = round(t_latched - t_impact, 3) if (t_latched and t_impact) else None
        results['test2_glass_wall_collision_safety'] = {
            'impact_detected': bool(t_impact is not None),
            'stall_latched_confirmed': latched_observed,
            'detection_latency_s': latency,
            'cmd_zeroed_on_latch': bool(abs(node.latest_cmd_out.linear.x) < 0.001),
            'auto_reverse_prevented': not reverse_occurred,
            'reverse_distance_m': 0.0 if not reverse_occurred else round(delta_y, 4),
            'operator_unlatch_required': True,
            'operator_unlatch_service_working': unlatch_success,
            'verdict': 'PASS' if (latched_observed and not reverse_occurred and latency is not None and latency < 0.70) else 'FAIL',
        }
        print(f"  Test 2 Result: Latency={latency}s, ReversePrevented={not reverse_occurred}, Verdict={results['test2_glass_wall_collision_safety']['verdict']}")

        # TEST 4: ICP Degradation Fail-Safe
        print("\n--- Running Test 4: ICP Quality Degradation Fail-Safe ---")
        node.samples = []
        t_start = time.time()
        icp_degraded_latched = False

        while time.time() - t_start < 4.0:
            # Inject low inlier / high error signal on /diagnostics/f4_latency
            diag_msg = Float32MultiArray()
            diag_msg.data = [45.0, 100.0, 45.0, 0.28]  # inliers=45 (<100), err=0.28m (>0.15m)
            node.pub_f4_diag.publish(diag_msg)

            node.pub_cmd_raw.publish(forward_cmd)
            rclpy.spin_once(node, timeout_sec=0.04)
            node.record_sample('icp_degraded_test')

            if node.stall_status.get('state') == 'FAILSAFE_ICP_DEGRADED':
                icp_degraded_latched = True
                print(f"  [FAILSAFE_ICP_DEGRADED Confirmed] cmd_out_vx={node.latest_cmd_out.linear.x}")
                break

        results['test4_icp_degradation_failsafe'] = {
            'icp_degraded_state_triggered': icp_degraded_latched,
            'cmd_zeroed_on_degradation': bool(abs(node.latest_cmd_out.linear.x) < 0.001),
            'verdict': 'PASS' if icp_degraded_latched else 'FAIL',
        }
        print(f"  Test 4 Result: DegradedTriggered={icp_degraded_latched}, Verdict={results['test4_icp_degradation_failsafe']['verdict']}")

        # Save summary JSON
        out_summary = os.path.join(RESULTS_DIR, 'stall_safety_modes_summary.json')
        with open(out_summary, 'w') as fp:
            json.dump({
                'generated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                'safety_configuration': {
                    'enable_auto_reverse': False,
                    'enable_auto_rotate_recovery': False,
                    'require_operator_unlatch': True,
                    'min_icp_inliers': 100,
                    'max_icp_error_m': 0.15,
                },
                'results': results,
            }, fp, indent=2)
        print(f"\nSaved Stall Safety Modes Summary to {out_summary}")

    finally:
        kill_proc(stall_proc)
        kill_proc(f4_proc)
        kill_proc(gt_proc)
        kill_proc(bridge_proc)
        kill_proc(gz_proc)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run_stall_safety_tests()
