#!/usr/bin/env -S python3 -s
"""
complementary_filter_node.py
Ported from 2026-11-07 Benchmark for LIMO Pro Real-Robot Deployment:
Implements BOTH F3 variants simultaneously for direct, fair comparison:
- F3a (/odometry/comp_filter): Complementary Filter standalone (Wheel Odom + IMU only).
  Fuses high-frequency IMU gyro yaw rate (wz) and relative IMU orientation with wheel
  odometry linear velocity (vx, vy).
- F3b (/odometry/comp_slip_gate): Complementary Filter + Physics-Informed Slip/Stall Gate.
  Uses the exact same complementary heading filter as F3a, plus a Slip/Stall Gate that cross-checks
  wheel encoder velocity against forward/rear LiDAR range-rate and obstacle proximity.
  When wheels spin while the chassis is physically blocked (e.g. glass wall / chair rail stall),
  F3b suppresses translational integration to prevent phantom odometry advancement.
"""
import math
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class ComplementaryFilterNode(Node):
    def __init__(self):
        super().__init__(
            'complementary_filter_node',
            automatically_declare_parameters_from_overrides=True
        )
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        if not self.has_parameter('alpha_heading'):
            self.declare_parameter('alpha_heading', 0.98)
        if not self.has_parameter('slip_vel_threshold'):
            self.declare_parameter('slip_vel_threshold', 0.035)
        if not self.has_parameter('stall_range_rate_threshold'):
            self.declare_parameter('stall_range_rate_threshold', 0.018)
        if not self.has_parameter('contact_range_threshold'):
            self.declare_parameter('contact_range_threshold', 0.24)

        self.alpha = float(self.get_parameter('alpha_heading').value)
        self.slip_vel_thresh = float(self.get_parameter('slip_vel_threshold').value)
        self.stall_rate_thresh = float(self.get_parameter('stall_range_rate_threshold').value)
        self.contact_range_thresh = float(self.get_parameter('contact_range_threshold').value)

        # State for F3a (Complementary Filter only)
        self.x_f3a = 0.0
        self.y_f3a = 0.0
        self.yaw_f3a = 0.0

        # State for F3b (Complementary Filter + Slip/Stall Gate)
        self.x_f3b = 0.0
        self.y_f3b = 0.0
        self.yaw_f3b = 0.0

        # IMU state
        self.imu_yaw0 = None
        self.imu_rel_yaw = 0.0
        self.imu_wz = 0.0
        self.imu_ax = 0.0
        self.last_imu_t = None

        # Scan history for slip/stall gating: deque of (t_sec, front_median_r, rear_median_r, front_min_r)
        self.scan_history = deque(maxlen=15)
        self.slip_gate_active = False
        self.slip_scale = 1.0
        self.latest_front_rate = 0.0
        self.latest_front_min = 10.0

        self.last_odom_t = None

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

        self.pub_f3a = self.create_publisher(Odometry, '/odometry/comp_filter', 20)
        self.pub_f3b = self.create_publisher(Odometry, '/odometry/comp_slip_gate', 20)
        self.pub_diag = self.create_publisher(Float32MultiArray, '/diagnostics/slip_gate', 10)

        self.get_logger().info(
            "ComplementaryFilterNode initialized: publishing F3a (/odometry/comp_filter) "
            "and F3b (/odometry/comp_slip_gate)."
        )

    def on_imu(self, msg: Imu):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw_yaw = quat_to_yaw(msg.orientation)
        if self.imu_yaw0 is None:
            self.imu_yaw0 = raw_yaw
        self.imu_rel_yaw = wrap_angle(raw_yaw - self.imu_yaw0)
        self.imu_wz = msg.angular_velocity.z
        self.imu_ax = msg.linear_acceleration.x

        if self.last_imu_t is not None:
            dt = t_sec - self.last_imu_t
            if 0.0 < dt < 0.5:
                # Complementary filter update for heading
                gyro_pred_a = wrap_angle(self.yaw_f3a + self.imu_wz * dt)
                err_a = wrap_angle(self.imu_rel_yaw - gyro_pred_a)
                self.yaw_f3a = wrap_angle(gyro_pred_a + (1.0 - self.alpha) * err_a)

                gyro_pred_b = wrap_angle(self.yaw_f3b + self.imu_wz * dt)
                err_b = wrap_angle(self.imu_rel_yaw - gyro_pred_b)
                self.yaw_f3b = wrap_angle(gyro_pred_b + (1.0 - self.alpha) * err_b)

        self.last_imu_t = t_sec

    def on_scan(self, msg: LaserScan):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

        # Forward sector: [-20 deg, +20 deg]
        front_mask = (np.abs(angles) <= math.radians(20.0)) & np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        # Rear sector: [160 deg, 200 deg]
        rear_mask = (np.abs(np.abs(angles) - math.pi) <= math.radians(20.0)) & np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)

        front_med = float(np.median(ranges[front_mask])) if np.any(front_mask) else 10.0
        front_min = float(np.min(ranges[front_mask])) if np.any(front_mask) else 10.0
        rear_med = float(np.median(ranges[rear_mask])) if np.any(rear_mask) else 10.0
        self.latest_front_min = front_min

        self.scan_history.append((t_sec, front_med, rear_med, front_min))

    def compute_slip_gate_scale(self, t_sec: float, vx_wheel: float, wz_wheel: float) -> float:
        """
        Returns a gating scale in [0.0, 1.0] for wheel linear velocity.
        1.0 = normal traction; 0.0 = full stall/wheel-slip detected.
        """
        if abs(vx_wheel) < self.slip_vel_thresh:
            self.slip_gate_active = False
            return 1.0

        # Check 1: Direct obstacle contact in front while commanding forward velocity
        if vx_wheel > self.slip_vel_thresh and self.latest_front_min < self.contact_range_thresh:
            self.slip_gate_active = True
            return 0.0

        # Check 2: Range-rate consistency over recent ~0.25s scan history
        if len(self.scan_history) >= 3:
            t_old, f_old, r_old, _ = self.scan_history[0]
            t_new, f_new, r_new, _ = self.scan_history[-1]
            dt_scan = t_new - t_old
            if dt_scan >= 0.15 and (t_sec - t_new) <= 0.35:
                # Only use range rate if turning slowly and a surface is within 4.5m
                if abs(self.imu_wz) < 0.25:
                    if vx_wheel > 0.0 and f_new < 4.5 and f_old < 4.5:
                        front_rate = (f_old - f_new) / dt_scan  # positive when approaching wall
                        self.latest_front_rate = front_rate
                        if abs(front_rate) < self.stall_rate_thresh and abs(vx_wheel) > 0.05:
                            self.slip_gate_active = True
                            return 0.0
                    elif vx_wheel < 0.0 and r_new < 4.5 and r_old < 4.5:
                        rear_rate = (r_old - r_new) / dt_scan
                        self.latest_front_rate = -rear_rate
                        if abs(rear_rate) < self.stall_rate_thresh and abs(vx_wheel) > 0.05:
                            self.slip_gate_active = True
                            return 0.0

        self.slip_gate_active = False
        return 1.0

    def on_odom(self, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz_odom = msg.twist.twist.angular.z

        if self.last_odom_t is None:
            self.last_odom_t = t_sec
            return

        dt = t_sec - self.last_odom_t
        if dt <= 0.0 or dt > 0.5:
            self.last_odom_t = t_sec
            return
        self.last_odom_t = t_sec

        # If IMU hasn't arrived yet, fallback to odom orientation
        if self.imu_yaw0 is None:
            odom_yaw = quat_to_yaw(msg.pose.pose.orientation)
            self.yaw_f3a = odom_yaw
            self.yaw_f3b = odom_yaw
            wz_used = wz_odom
        else:
            wz_used = self.imu_wz

        # 1. Update F3a (Complementary Filter standalone - NO slip gate)
        c_a = math.cos(self.yaw_f3a)
        s_a = math.sin(self.yaw_f3a)
        self.x_f3a += (c_a * vx - s_a * vy) * dt
        self.y_f3a += (s_a * vx + c_a * vy) * dt

        msg_f3a = Odometry()
        msg_f3a.header.stamp = msg.header.stamp
        msg_f3a.header.frame_id = 'odom'
        msg_f3a.child_frame_id = 'base_footprint'
        msg_f3a.pose.pose.position.x = self.x_f3a
        msg_f3a.pose.pose.position.y = self.y_f3a
        msg_f3a.pose.pose.orientation = yaw_to_quat(self.yaw_f3a)
        msg_f3a.twist.twist.linear.x = vx
        msg_f3a.twist.twist.linear.y = vy
        msg_f3a.twist.twist.angular.z = wz_used
        msg_f3a.pose.covariance = msg.pose.covariance
        msg_f3a.twist.covariance = msg.twist.covariance
        self.pub_f3a.publish(msg_f3a)

        # 2. Update F3b (Complementary Filter + Slip/Stall Gate)
        self.slip_scale = self.compute_slip_gate_scale(t_sec, vx, wz_used)
        vx_gated = vx * self.slip_scale
        vy_gated = vy * self.slip_scale

        c_b = math.cos(self.yaw_f3b)
        s_b = math.sin(self.yaw_f3b)
        self.x_f3b += (c_b * vx_gated - s_b * vy_gated) * dt
        self.y_f3b += (s_b * vx_gated + c_b * vy_gated) * dt

        msg_f3b = Odometry()
        msg_f3b.header.stamp = msg.header.stamp
        msg_f3b.header.frame_id = 'odom'
        msg_f3b.child_frame_id = 'base_footprint'
        msg_f3b.pose.pose.position.x = self.x_f3b
        msg_f3b.pose.pose.position.y = self.y_f3b
        msg_f3b.pose.pose.orientation = yaw_to_quat(self.yaw_f3b)
        msg_f3b.twist.twist.linear.x = vx_gated
        msg_f3b.twist.twist.linear.y = vy_gated
        msg_f3b.twist.twist.angular.z = wz_used
        msg_f3b.pose.covariance = msg.pose.covariance
        msg_f3b.twist.covariance = msg.twist.covariance
        self.pub_f3b.publish(msg_f3b)

        diag = Float32MultiArray()
        diag.data = [float(vx), float(vx_gated), float(self.latest_front_rate), float(1.0 if self.slip_gate_active else 0.0)]
        self.pub_diag.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = ComplementaryFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
