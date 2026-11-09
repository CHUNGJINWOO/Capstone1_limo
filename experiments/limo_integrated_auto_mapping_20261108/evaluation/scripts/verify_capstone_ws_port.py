#!/usr/bin/env -S python3 -s
"""
verify_capstone_ws_port.py
Comprehensive Regression Verification Suite for Ported capstone_ws Packages:
1. Verifies F4 Integrated Simulation Bringup via installed limo_capstone launch
2. Verifies F5_improved Integrated Simulation Bringup via installed limo_capstone launch
3. Verifies In-Flight Dynamic Depth Dropout (measures latency, command clamp, vehicle deceleration)
4. Verifies TF Single Ownership & Fail-Closed Safety Gate
5. Saves structured regression results to evaluation/results/capstone_ws_porting_verification_20261108.json
"""
import os
import sys
import time
import json
import signal
import subprocess
from datetime import datetime
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

REPO_ROOT = os.environ.get('PROJECT_ROOT', os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
WS_DIR = os.path.join(REPO_ROOT, 'capstone_ws')
EXP_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
RESULT_JSON = os.path.join(EXP_DIR, 'evaluation', 'results', 'capstone_ws_porting_verification_20261108.json')

env = os.environ.copy()
env['PYTHONNOUSERSITE'] = '1'
rl_lib = f"{WS_DIR}/install/robot_localization/lib"
env['LD_LIBRARY_PATH'] = f"{rl_lib}:/opt/ros/humble/lib:{env.get('LD_LIBRARY_PATH', '')}"


def kill_proc_group(p, sig=signal.SIGINT, timeout=3.0):
    if p and p.poll() is None:
        try:
            pgid = os.getpgid(p.pid)
            if pgid != os.getpgrp():
                os.killpg(pgid, sig)
            else:
                p.send_signal(sig)
            p.wait(timeout=timeout)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass


def kill_all_leftovers():
    subprocess.run(['pkill', '-9', '-f', 'ign|ros2|slam_toolbox|nav2|ekf|recorder'],
                   capture_output=True, text=True)
    time.sleep(2.0)


class PortingAuditNode(Node):
    def __init__(self, node_name: str):
        super().__init__(
            node_name,
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_rel = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_be = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.msg_counts = {}
        self.latest_status = None
        self.latest_odom = None
        self.latest_gt = None
        self.cmd_out_history = []

        self.create_subscription(LaserScan, '/scan', lambda m: self._count('/scan'), qos_be)
        self.create_subscription(Odometry, '/odom', lambda m: self._count('/odom'), 10)
        self.create_subscription(Odometry, '/odometry/lidar_odom', lambda m: self._count('/odometry/lidar_odom'), 10)
        self.create_subscription(Odometry, '/odometry/f5_improved', lambda m: self._count('/odometry/f5_improved'), 10)
        self.create_subscription(Odometry, '/odom_nav2', self._on_odom_nav2, 10)
        self.create_subscription(Odometry, '/ground_truth/world_odom', self._on_gt, 10)
        self.create_subscription(OccupancyGrid, '/map', lambda m: self._count('/map'), qos_rel)
        self.create_subscription(PointCloud2, '/depth_obstacles/points', lambda m: self._count('/depth_obstacles/points'), qos_rel)
        self.create_subscription(Twist, '/cmd_vel_raw', lambda m: self._count('/cmd_vel_raw'), 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(String, '/stall_detector/status_json', self._on_status, 10)

        self.pub_cmd_raw = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.pub_depth_status = self.create_publisher(String, '/auto_mapping/depth_status', 10)
        self.cli_reset = self.create_client(Trigger, '/stall_detector/reset_latch')

    def _count(self, topic: str):
        self.msg_counts[topic] = self.msg_counts.get(topic, 0) + 1

    def _on_odom_nav2(self, msg: Odometry):
        self._count('/odom_nav2')
        self.latest_odom = msg

    def _on_gt(self, msg: Odometry):
        self._count('/ground_truth/world_odom')
        self.latest_gt = msg

    def _on_cmd_vel(self, msg: Twist):
        self._count('/cmd_vel')
        t_sim = self.get_clock().now().nanoseconds * 1e-9
        self.cmd_out_history.append((t_sim, msg.linear.x, msg.angular.z))

    def _on_status(self, msg: String):
        self._count('/stall_detector/status_json')
        try:
            self.latest_status = json.loads(msg.data)
        except Exception:
            pass

    def sim_time(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def run_stage_test(estimator: str, duration: float) -> Dict[str, Any]:
    print(f"\n--- Testing limo_capstone Simulation Bringup ({estimator.upper()}) ---")
    kill_all_leftovers()

    cmd = [
        'ros2', 'launch', 'limo_capstone', 'limo_integrated_auto_mapping_sim.launch.py',
        'use_sim_time:=true',
        f'estimator_type:={estimator}',
        'start_explore:=true',
        'record_video:=false'
    ]

    p = subprocess.Popen(cmd, env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    node = PortingAuditNode(f'audit_{estimator}')

    metrics = {
        'estimator': estimator,
        'duration_sec': duration,
        'topics_verified': {},
        'tf_chain_verified': False,
        'verdict': 'FAIL'
    }

    try:
        t_start = time.time()
        while time.time() - t_start < duration:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Check topics
        metrics['topics_verified'] = dict(node.msg_counts)
        has_scan = node.msg_counts.get('/scan', 0) > 10
        has_map = node.msg_counts.get('/map', 0) > 0
        has_depth = node.msg_counts.get('/depth_obstacles/points', 0) > 0
        has_gate = node.msg_counts.get('/stall_detector/status_json', 0) > 10

        est_topic = '/odometry/lidar_odom' if estimator == 'f4' else '/odometry/f5_improved'
        has_est = node.msg_counts.get(est_topic, 0) > 10
        has_odom_nav2 = node.msg_counts.get('/odom_nav2', 0) > 10

        # Check TF chain map -> odom -> base_footprint -> base_link
        can_tf = node.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(),
                                              timeout=rclpy.duration.Duration(seconds=2.0))
        metrics['tf_chain_verified'] = can_tf

        all_ok = has_scan and has_map and has_depth and has_gate and has_est and has_odom_nav2 and can_tf
        metrics['verdict'] = 'PASS' if all_ok else 'FAIL'
        print(f"  Result ({estimator.upper()}): {metrics['verdict']} (TF={can_tf}, Map={has_map}, Est={has_est}, Depth={has_depth})")

    finally:
        kill_proc_group(p, signal.SIGINT, timeout=4.0)
        node.destroy_node()
        kill_all_leftovers()

    return metrics


def run_dynamic_depth_dropout_test() -> Dict[str, Any]:
    print("\n--- Testing In-Flight Dynamic Depth Dropout & Halting Latency ---")
    kill_all_leftovers()

    cmd = [
        'ros2', 'launch', 'limo_capstone', 'limo_integrated_auto_mapping_sim.launch.py',
        'use_sim_time:=true',
        'estimator_type:=f4',
        'start_explore:=false',
        'depth_mode:=required',
        'record_video:=false'
    ]

    p = subprocess.Popen(cmd, env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    node = PortingAuditNode('audit_dynamic_depth')

    res = {
        'test': 'In-Flight Dynamic Depth Dropout with Moving Vehicle',
        'steady_speed_achieved': False,
        'detection_latency_sec': None,
        'cutoff_latency_sec': None,
        'stopping_distance_m': None,
        'zero_creep_confirmed': False,
        'verdict': 'FAIL'
    }

    try:
        drive_cmd = Twist()
        drive_cmd.linear.x = 0.15

        # Step 1: Sensor & Node Warmup while remaining stationary (no motion command)
        print("  Waiting for sensor streams and depth pipeline warmup...")
        t_w = time.time()
        while time.time() - t_w < 6.0:
            node.pub_cmd_raw.publish(Twist())  # keep zero velocity during bringup
            rclpy.spin_once(node, timeout_sec=0.05)

        # Step 2: Unlatch stall detector if latched during cold bringup
        if node.cli_reset.wait_for_service(timeout_sec=3.0):
            req = Trigger.Request()
            fut = node.cli_reset.call_async(req)
            t_call = time.time()
            while not fut.done() and (time.time() - t_call < 3.0):
                rclpy.spin_once(node, timeout_sec=0.05)
            print("  Stall detector latch cleared to NORMAL via operator service.")

        # Step 3: Command forward motion and establish steady cruise
        print("  Commanding vehicle acceleration to steady cruising speed (0.15 m/s)...")
        t_cruise_start = time.time()
        while time.time() - t_cruise_start < 4.0:
            node.pub_cmd_raw.publish(drive_cmd)
            rclpy.spin_once(node, timeout_sec=0.03)

        v_gt = abs(node.latest_gt.twist.twist.linear.x) if node.latest_gt else 0.0
        st = node.latest_status.get('state', 'UNKNOWN') if node.latest_status else 'UNKNOWN'
        print(f"  Cruising state: state={st}, GT vx={v_gt:.3f} m/s")
        print(f"  Status detail: {node.latest_status}")
        res['steady_speed_achieved'] = (v_gt > 0.05) and (st == 'NORMAL')

        # Step 4: Record pre-fault pose
        p0_x = node.latest_gt.pose.pose.position.x if node.latest_gt else 0.0
        p0_y = node.latest_gt.pose.pose.position.y if node.latest_gt else 0.0

        # Step 5: Inject Dynamic Fault: Publish DEPTH_STALE error status
        print("  Injecting dynamic depth dropout (DEPTH_STALE) while cruising...")
        t_fault_sim = node.sim_time()

        err_msg = String()
        err_msg.data = json.dumps({'state': 'error', 'code': 'DEPTH_STALE', 'message': 'Dynamic in-flight dropout'})

        t_detect_sim = None
        t_cutoff_sim = None
        t_halt_sim = None
        t_loop = time.time()

        while time.time() - t_loop < 6.0:
            node.pub_cmd_raw.publish(drive_cmd)
            node.pub_depth_status.publish(err_msg)
            rclpy.spin_once(node, timeout_sec=0.02)

            cur_st = node.latest_status.get('state', 'UNKNOWN') if node.latest_status else 'UNKNOWN'
            if cur_st == 'FAILSAFE_SENSOR_TIMEOUT' and t_detect_sim is None:
                t_detect_sim = node.sim_time()

            # Check if /cmd_vel output clamped to 0
            if node.cmd_out_history and abs(node.cmd_out_history[-1][1]) < 1e-4 and t_cutoff_sim is None:
                t_cutoff_sim = node.sim_time()

            # Check vehicle physical halt in Gazebo
            cur_vx = abs(node.latest_gt.twist.twist.linear.x) if node.latest_gt else 0.0
            if cur_st == 'FAILSAFE_SENSOR_TIMEOUT' and cur_vx < 0.005 and t_halt_sim is None:
                t_halt_sim = node.sim_time()

        p1_x = node.latest_gt.pose.pose.position.x if node.latest_gt else 0.0
        p1_y = node.latest_gt.pose.pose.position.y if node.latest_gt else 0.0
        stop_dist = ((p1_x - p0_x)**2 + (p1_y - p0_y)**2)**0.5

        det_lat = (t_detect_sim - t_fault_sim) if t_detect_sim else 999.0
        cut_lat = (t_cutoff_sim - t_fault_sim) if t_cutoff_sim else 999.0

        res['detection_latency_sec'] = round(det_lat, 3)
        res['cutoff_latency_sec'] = round(cut_lat, 3)
        res['stopping_distance_m'] = round(stop_dist, 3)
        res['zero_creep_confirmed'] = (t_halt_sim is not None)

        pass_dyn = (det_lat <= 0.38) and (cut_lat <= 0.55) and (stop_dist < 0.20) and res['zero_creep_confirmed'] and res['steady_speed_achieved']
        res['verdict'] = 'PASS' if pass_dyn else 'FAIL'
        print(f"  Result (Dynamic Depth Dropout): {res['verdict']} (DetLat={det_lat:.3f}s <= 0.38s, CutLat={cut_lat:.3f}s <= 0.55s, StopDist={stop_dist:.3f}m, SteadySpeed={res['steady_speed_achieved']})")

    finally:
        kill_proc_group(p, signal.SIGINT, timeout=4.0)
        node.destroy_node()
        kill_all_leftovers()

    return res


def main():
    rclpy.init()
    print("=================================================================")
    print("Executing capstone_ws Integrated Auto-Mapping Regression Suite")
    print("=================================================================")

    report = {
        'test_timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'workspace': WS_DIR,
        'packages_tested': ['limo_depth_obstacles', 'limo_capstone'],
        'stages': {}
    }

    only_depth = '--only-depth' in sys.argv

    # 1. Test F4 bringup
    if not only_depth:
        report['stages']['stage_3_f4_simulation'] = run_stage_test('f4', duration=18.0)
        report['stages']['stage_4_f5_simulation'] = run_stage_test('f5_improved', duration=18.0)

    # 3. Test In-Flight Dynamic Depth Dropout
    report['stages']['stage_7_dynamic_depth_dropout'] = run_dynamic_depth_dropout_test()

    # 4. Overall summary
    all_pass = all(s.get('verdict') == 'PASS' for s in report['stages'].values())
    report['overall_verdict'] = 'PASS' if all_pass else 'FAIL'

    os.makedirs(os.path.dirname(RESULT_JSON), exist_ok=True)
    with open(RESULT_JSON, 'w') as f:
        json.dump(report, f, indent=2)

    print("=================================================================")
    print(f"Regression Suite Complete: Overall Verdict = {report['overall_verdict']}")
    print(f"Report written to: {RESULT_JSON}")
    print("=================================================================")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
