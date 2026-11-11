#!/usr/bin/env python3
"""
run_unpowered_trial.py
Systematic Unpowered Verification & Experiment Logger for LIMO Pro
Performs end-to-end audit for B0, F1b, F3b, F4, F5_improved:
- Starts launch process in clean isolated environment
- Records 10s rosbag with sensor & odometry streams
- Collects live system CPU/RAM and topic metrics
- Checks TF single-authority & safety gate locks
- Stores artifacts in /home/pcu/experiments/unpowered_trials/<EXP_ID>
"""
import argparse
import datetime
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


def quat_to_rpy(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class TrialMonitorNode(Node):
    def __init__(self, active_topic):
        super().__init__('trial_monitor_node')
        self.active_topic = active_topic
        self.counts = defaultdict(int)
        self.latest_msgs = {}

        sensor_qos = qos_profile_sensor_data
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=25
        )

        self.create_subscription(LaserScan, '/scan', lambda m: self._cb('/scan', m), sensor_qos)
        self.create_subscription(Imu, '/imu', lambda m: self._cb('/imu', m), sensor_qos)
        self.create_subscription(Odometry, '/odom', lambda m: self._cb('/odom', m), reliable_qos)
        self.create_subscription(Odometry, '/odom_nav2', lambda m: self._cb('/odom_nav2', m), reliable_qos)
        if self.active_topic not in ['/odom', '/odom_nav2']:
            self.create_subscription(Odometry, self.active_topic, lambda m: self._cb(self.active_topic, m), reliable_qos)
        self.create_subscription(Float32MultiArray, '/diagnostics/slip_gate', lambda m: self._cb('/diagnostics/slip_gate', m), 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _cb(self, topic, msg):
        self.counts[topic] += 1
        self.latest_msgs[topic] = msg


def get_system_resource_usage():
    try:
        # Get overall CPU percentage and Memory via /proc
        with open('/proc/meminfo', 'r') as f:
            lines = f.readlines()
        mem_total = 0
        mem_avail = 0
        for l in lines:
            if l.startswith('MemTotal:'):
                mem_total = int(l.split()[1])
            elif l.startswith('MemAvailable:'):
                mem_avail = int(l.split()[1])
        mem_used_mb = round((mem_total - mem_avail) / 1024.0, 1)

        # Get load averages
        load1, load5, load15 = os.getloadavg()
        return {
            'mem_used_mb': mem_used_mb,
            'mem_total_mb': round(mem_total / 1024.0, 1),
            'load_1min': load1,
            'load_5min': load5,
        }
    except Exception as e:
        return {'error': str(e)}


def cleanup_processes():
    targets = [
        'limo_base', 'ydlidar_ros2_driver', 'component_container',
        'static_transform_publisher', 'robot_state_publisher', 'joint_state_publisher',
        'depth_obstacle_filter_node', 'lidar_scan_matcher_odom_node',
        'odom_tf_relay_node', 'stall_detector_node', 'complementary_filter_node',
        'ekf_node', 'async_slam_toolbox_node', 'controller_server',
        'smoother_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'waypoint_follower', 'velocity_smoother',
        'lifecycle_manager', 'explore', 'rosbag2_recorder'
    ]
    for tgt in targets:
        subprocess.run(['pkill', '-f', tgt], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.0)


def run_single_trial(estimator_type, base_dir="/home/pcu/experiments/unpowered_trials"):
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    exp_id = f"EXP_{timestamp}_{estimator_type.upper()}_UNPOWERED"
    exp_dir = os.path.join(base_dir, exp_id)
    os.makedirs(exp_dir, exist_ok=True)
    bag_dir = os.path.join(exp_dir, "bag")
    log_file = os.path.join(exp_dir, "launch.log")

    topic_map = {
        'b0': '/odom',
        'f1b': '/odometry/ekf_tuned',
        'f3b': '/odometry/comp_slip_gate',
        'f4': '/odometry/lidar_odom',
        'f5_improved': '/odometry/f5_improved',
    }
    active_topic = topic_map.get(estimator_type, '/odom')

    print(f"\n=======================================================")
    print(f" STARTING UNPOWERED TRIAL: {exp_id}")
    print(f" Estimator Type: {estimator_type}")
    print(f" Active Odometry: {active_topic}")
    print(f" Output Dir: {exp_dir}")
    print(f"=======================================================")

    cleanup_processes()

    launch_cmd = [
        'ros2', 'launch', 'limo_capstone', 'limo_integrated_auto_mapping_real.launch.py',
        f'estimator_type:={estimator_type}',
        'use_depth_camera:=false',
        'allow_motor_drive:=false',
        'start_explore:=false',
        'use_sim_time:=false'
    ]

    report = {
        'experiment_id': exp_id,
        'estimator_type': estimator_type,
        'active_topic': active_topic,
        'timestamp': timestamp,
        'launch_command': ' '.join(launch_cmd),
        'status': 'UNVERIFIED',
        'reasons': [],
        'topic_rates_hz': {},
        'tf_status': {},
        'sensor_values': {},
        'safety_gate_status': {},
        'system_resources': {},
        'bag_info': {}
    }

    launch_log_fp = open(log_file, 'w')
    proc = subprocess.Popen(
        launch_cmd,
        stdout=launch_log_fp,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid
    )

    try:
        print("[1/5] Waiting 14s for hardware and Nav2 bringup...")
        time.sleep(14.0)

        if proc.poll() is not None:
            report['status'] = 'FAIL'
            report['reasons'].append(f"Launch process died prematurely with code {proc.returncode}")
            return report

        # Start 10s rosbag recording
        topics_to_record = [
            '/odom', '/imu', '/scan', '/odom_nav2',
            '/cmd_vel_raw', '/cmd_vel_locked', '/tf', '/tf_static'
        ]
        if active_topic not in topics_to_record:
            topics_to_record.append(active_topic)
        if estimator_type == 'f3b':
            topics_to_record.append('/diagnostics/slip_gate')

        print(f"[2/5] Starting 10s rosbag recording to {bag_dir}...")
        bag_cmd = ['timeout', '-s', 'INT', '10', 'ros2', 'bag', 'record', '-o', bag_dir] + topics_to_record
        bag_proc = subprocess.Popen(bag_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Spin monitor node to collect live metrics
        print("[3/5] Sampling live sensor rates and TF tree...")
        rclpy.init()
        monitor = TrialMonitorNode(active_topic)

        start_t = time.time()
        while time.time() - start_t < 9.0:
            rclpy.spin_once(monitor, timeout_sec=0.1)

        elapsed = time.time() - start_t
        for t, cnt in monitor.counts.items():
            report['topic_rates_hz'][t] = round(cnt / elapsed, 2)

        # Check TF
        now = rclpy.time.Time()
        frames_to_check = [
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
            ('base_link', 'laser_link'),
            ('map', 'odom')
        ]
        for parent, child in frames_to_check:
            try:
                tf = monitor.tf_buffer.lookup_transform(parent, child, now, timeout=Duration(seconds=2.0))
                report['tf_status'][f'{parent}->{child}'] = {
                    'ok': True,
                    'tx': round(tf.transform.translation.x, 4),
                    'ty': round(tf.transform.translation.y, 4),
                    'tz': round(tf.transform.translation.z, 4),
                }
            except Exception as e:
                report['tf_status'][f'{parent}->{child}'] = {'ok': False, 'error': str(e)}

        # Sample sensors
        if '/imu' in monitor.latest_msgs:
            im = monitor.latest_msgs['/imu']
            q = im.orientation
            norm_q = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            r, p, y = quat_to_rpy(q)
            report['sensor_values']['imu'] = {
                'quat_norm': round(norm_q, 5),
                'roll_deg': round(math.degrees(r), 2),
                'pitch_deg': round(math.degrees(p), 2),
                'yaw_deg': round(math.degrees(y), 2),
                'wz_rad_s': round(im.angular_velocity.z, 4),
                'ax_m_s2': round(im.linear_acceleration.x, 3)
            }

        if active_topic in monitor.latest_msgs:
            od = monitor.latest_msgs[active_topic]
            _, _, y_od = quat_to_rpy(od.pose.pose.orientation)
            report['sensor_values']['active_odom'] = {
                'x': round(od.pose.pose.position.x, 4),
                'y': round(od.pose.pose.position.y, 4),
                'yaw_deg': round(math.degrees(y_od), 2),
                'vx': round(od.twist.twist.linear.x, 4),
                'wz': round(od.twist.twist.angular.z, 4)
            }

        if '/diagnostics/slip_gate' in monitor.latest_msgs:
            sg = monitor.latest_msgs['/diagnostics/slip_gate']
            report['sensor_values']['slip_gate'] = {
                'vx': sg.data[0] if len(sg.data) > 0 else 0.0,
                'vx_gated': sg.data[1] if len(sg.data) > 1 else 0.0,
                'front_rate': sg.data[2] if len(sg.data) > 2 else 0.0,
                'slip_active': bool(sg.data[3] > 0.5) if len(sg.data) > 3 else False
            }

        # Check safety gate publishers
        pub_cmd_vel = monitor.count_publishers('/cmd_vel')
        pub_cmd_locked = monitor.count_publishers('/cmd_vel_locked')
        report['safety_gate_status'] = {
            'cmd_vel_publishers': pub_cmd_vel,
            'cmd_vel_locked_publishers': pub_cmd_locked,
            'motor_lock_active': (pub_cmd_vel == 0 and pub_cmd_locked >= 1)
        }

        # System resources
        report['system_resources'] = get_system_resource_usage()

        monitor.destroy_node()
        rclpy.shutdown()

        # Wait for bag recording to complete
        print("[4/5] Finalizing rosbag...")
        bag_proc.wait(timeout=5.0)

        # Inspect bag info
        try:
            bag_info_res = subprocess.run(['ros2', 'bag', 'info', bag_dir], capture_output=True, text=True, timeout=5.0)
            report['bag_info'] = {'stdout': bag_info_res.stdout, 'returncode': bag_info_res.returncode}
        except Exception as e:
            report['bag_info'] = {'error': str(e)}

        # Verification logic
        min_est_rate = 7.0 if estimator_type == 'f4' else 15.0
        pass_sensors = (
            report['topic_rates_hz'].get('/scan', 0) > 4.0
            and report['topic_rates_hz'].get('/imu', 0) > 20.0
            and report['topic_rates_hz'].get('/odom', 0) > 12.0
        )
        pass_estimator = (
            report['topic_rates_hz'].get(active_topic, 0) > min_est_rate
            and report['topic_rates_hz'].get('/odom_nav2', 0) > min_est_rate
        )
        pass_tf = report['tf_status'].get('odom->base_footprint', {}).get('ok', False)
        pass_safety = report['safety_gate_status']['motor_lock_active']

        if not pass_sensors:
            report['reasons'].append(f"Sensors below threshold: scan={report['topic_rates_hz'].get('/scan',0)}Hz, imu={report['topic_rates_hz'].get('/imu',0)}Hz, odom={report['topic_rates_hz'].get('/odom',0)}Hz")
        if not pass_estimator:
            report['reasons'].append(f"Estimator topic below threshold: {active_topic}={report['topic_rates_hz'].get(active_topic,0)}Hz, /odom_nav2={report['topic_rates_hz'].get('/odom_nav2',0)}Hz")
        if not pass_tf:
            report['reasons'].append("TF lookup failed for odom -> base_footprint")
        if not pass_safety:
            report['reasons'].append(f"Safety motor lock breached: cmd_vel_publishers={pub_cmd_vel}")

        if pass_sensors and pass_estimator and pass_tf and pass_safety:
            report['status'] = 'PASS'
        else:
            report['status'] = 'FAIL'

        print(f"[5/5] Trial Result: {report['status']}")

    finally:
        # Tear down launch process cleanly
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait()
        launch_log_fp.close()
        cleanup_processes()

    # Save metrics JSON
    with open(os.path.join(exp_dir, "metrics.json"), "w") as f:
        json.dump(report, f, indent=2)

    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run unpowered trial on LIMO Pro")
    parser.add_argument('--estimator', default='all', choices=['all', 'b0', 'f1b', 'f3b', 'f4', 'f5_improved'])
    args = parser.parse_args()

    estimators = ['b0', 'f1b', 'f3b', 'f4', 'f5_improved'] if args.estimator == 'all' else [args.estimator]
    all_reports = {}

    for est in estimators:
        rep = run_single_trial(est)
        all_reports[est] = rep
        time.sleep(3.0)

    print("\n=======================================================")
    print(" ALL UNPOWERED TRIALS COMPLETED")
    print("=======================================================")
    summary = {est: r['status'] for est, r in all_reports.items()}
    print(json.dumps(summary, indent=2))
    
    with open("/home/pcu/experiments/unpowered_trials/summary.json", "w") as f:
        json.dump(all_reports, f, indent=2)
