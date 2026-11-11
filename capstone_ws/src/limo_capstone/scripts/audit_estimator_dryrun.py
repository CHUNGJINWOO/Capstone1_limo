#!/usr/bin/env python3
"""
audit_estimator_dryrun.py
Physical Unpowered Bringup & Safety Audit Tool for LIMO Pro:
Launches a selected estimator with allow_motor_drive=false, samples topic rates,
validates TF tree, inspects sensor orientation and safety gate bindings,
and terminates cleanly.
"""
import argparse
import json
import math
import os
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
from sensor_msgs.msg import LaserScan, Imu, PointCloud2
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


class AuditAuditor(Node):
    def __init__(self, active_topic, check_depth=False):
        super().__init__('audit_auditor_node')
        self.active_topic = active_topic
        self.check_depth = check_depth
        self.counts = defaultdict(int)
        self.latest_msgs = {}

        # Compatible Sensor Data QoS (BEST_EFFORT) for hardware drivers
        sensor_qos = qos_profile_sensor_data
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', lambda m: self._cb('/scan', m), sensor_qos)
        self.create_subscription(Imu, '/imu', lambda m: self._cb('/imu', m), sensor_qos)
        self.create_subscription(Odometry, '/odom', lambda m: self._cb('/odom', m), reliable_qos)
        self.create_subscription(Odometry, '/odom_nav2', lambda m: self._cb('/odom_nav2', m), reliable_qos)
        self.create_subscription(Odometry, self.active_topic, lambda m: self._cb(self.active_topic, m), reliable_qos)
        self.create_subscription(Float32MultiArray, '/diagnostics/slip_gate', lambda m: self._cb('/diagnostics/slip_gate', m), 10)

        if self.check_depth:
            self.create_subscription(PointCloud2, '/camera/depth/points', lambda m: self._cb('/camera/depth/points', m), sensor_qos)
            self.create_subscription(PointCloud2, '/depth_obstacles/points', lambda m: self._cb('/depth_obstacles/points', m), sensor_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _cb(self, topic, msg):
        self.counts[topic] += 1
        self.latest_msgs[topic] = msg


def run_audit(estimator_type, use_depth_camera, sample_duration=6.0):
    topic_map = {
        'b0': '/odom',
        'f1b': '/odometry/ekf_tuned',
        'f3b': '/odometry/comp_slip_gate',
        'f4': '/odometry/lidar_odom',
        'f5_improved': '/odometry/f5_improved',
    }
    active_topic = topic_map.get(estimator_type, '/odom')
    depth_bool = (str(use_depth_camera).lower() == 'true')

    cmd = [
        'ros2', 'launch', 'limo_capstone', 'limo_integrated_auto_mapping_real.launch.py',
        f'estimator_type:={estimator_type}',
        f'use_depth_camera:={"true" if depth_bool else "false"}',
        'allow_motor_drive:=false',
        'start_explore:=false'
    ]

    print(f"Launching process: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )

    result = {
        'estimator_type': estimator_type,
        'use_depth_camera': depth_bool,
        'active_topic': active_topic,
        'bringup_success': False,
        'topic_rates_hz': {},
        'tf_status': {},
        'sensor_values': {},
        'safety_gate_status': {},
    }

    try:
        # Wait 18s for depth camera + Nav2 hardware bringup, 14s otherwise
        bringup_wait = 18.0 if depth_bool else 14.0
        time.sleep(bringup_wait)
        if proc.poll() is not None:
            result['error'] = f"Process terminated prematurely with code {proc.returncode}"
            return result

        rclpy.init()
        node = AuditAuditor(active_topic, check_depth=depth_bool)

        start_t = time.time()
        while time.time() - start_t < sample_duration:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Calculate rates
        elapsed = time.time() - start_t
        for t, cnt in node.counts.items():
            result['topic_rates_hz'][t] = round(cnt / elapsed, 2)

        # Check TF
        now = rclpy.time.Time()
        frames_to_check = [
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
            ('base_link', 'laser_link'),
            ('map', 'odom')
        ]
        if depth_bool:
            frames_to_check.append(('base_link', 'camera_color_optical_frame'))

        for parent, child in frames_to_check:
            try:
                tf = node.tf_buffer.lookup_transform(parent, child, now, timeout=Duration(seconds=2.0))
                result['tf_status'][f'{parent}->{child}'] = {
                    'ok': True,
                    'tx': round(tf.transform.translation.x, 4),
                    'ty': round(tf.transform.translation.y, 4),
                    'tz': round(tf.transform.translation.z, 4),
                }
            except Exception as e:
                result['tf_status'][f'{parent}->{child}'] = {'ok': False, 'error': str(e)}

        # Sample sensors
        if '/imu' in node.latest_msgs:
            im = node.latest_msgs['/imu']
            q = im.orientation
            norm_q = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            r, p, y = quat_to_rpy(q)
            result['sensor_values']['imu'] = {
                'quat_norm': round(norm_q, 5),
                'roll_deg': round(math.degrees(r), 2),
                'pitch_deg': round(math.degrees(p), 2),
                'yaw_deg': round(math.degrees(y), 2),
                'wz_rad_s': round(im.angular_velocity.z, 4),
                'ax_m_s2': round(im.linear_acceleration.x, 3)
            }

        if active_topic in node.latest_msgs:
            od = node.latest_msgs[active_topic]
            _, _, y_od = quat_to_rpy(od.pose.pose.orientation)
            result['sensor_values']['active_odom'] = {
                'x': round(od.pose.pose.position.x, 4),
                'y': round(od.pose.pose.position.y, 4),
                'yaw_deg': round(math.degrees(y_od), 2),
                'vx': round(od.twist.twist.linear.x, 4),
                'wz': round(od.twist.twist.angular.z, 4)
            }

        if '/diagnostics/slip_gate' in node.latest_msgs:
            sg = node.latest_msgs['/diagnostics/slip_gate']
            result['sensor_values']['slip_gate'] = {
                'vx': sg.data[0] if len(sg.data) > 0 else 0.0,
                'vx_gated': sg.data[1] if len(sg.data) > 1 else 0.0,
                'front_rate': sg.data[2] if len(sg.data) > 2 else 0.0,
                'slip_active': bool(sg.data[3] > 0.5) if len(sg.data) > 3 else False
            }

        # Check safety gate topics
        pub_cmd_vel = node.count_publishers('/cmd_vel')
        pub_cmd_locked = node.count_publishers('/cmd_vel_locked')
        result['safety_gate_status'] = {
            'cmd_vel_publishers': pub_cmd_vel,
            'cmd_vel_locked_publishers': pub_cmd_locked,
            'motor_lock_active': (pub_cmd_vel == 0 and pub_cmd_locked >= 1)
        }

        tf_ok = result['tf_status'].get('odom->base_footprint', {}).get('ok', False)
        if depth_bool:
            tf_ok = tf_ok and result['tf_status'].get('base_link->camera_color_optical_frame', {}).get('ok', False)

        result['bringup_success'] = (
            result['topic_rates_hz'].get('/scan', 0) > 4.0
            and result['topic_rates_hz'].get('/imu', 0) > 20.0
            and result['topic_rates_hz'].get('/odom', 0) > 12.0
            and result['topic_rates_hz'].get('/odom_nav2', 0) > 5.0
            and tf_ok
            and result['safety_gate_status']['motor_lock_active']
        )

        node.destroy_node()
        rclpy.shutdown()

    finally:
        # Graceful shutdown of launch tree
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=6.0)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait()
        # Clean up any orphan nodes
        subprocess.run(['pkill', '-f', 'limo|ydlidar|astra|ros2|nav2|slam_toolbox|tf2_ros'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2.0)

    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--estimator', default='f4', choices=['b0', 'f1b', 'f3b', 'f4', 'f5_improved'])
    parser.add_argument('--depth', default='false')
    args = parser.parse_args()

    res = run_audit(args.estimator, args.depth)
    print("=== AUDIT RESULT JSON START ===")
    print(json.dumps(res, indent=2))
    print("=== AUDIT RESULT JSON END ===")
