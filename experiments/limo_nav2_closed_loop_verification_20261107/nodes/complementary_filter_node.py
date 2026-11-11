#!/usr/bin/env -S python3 -s
"""
complementary_filter_node.py
Preserves F3a and original F3b (from the 75-trial benchmark) AND adds F3b_improved as a separate comparison group:
1. F3a (/odometry/comp_filter): Complementary Filter standalone (Wheel Odom + IMU).
2. F3b (/odometry/comp_slip_gate): Original Complementary Filter + 15-scan (1.4s) range-rate Slip Gate
   (preserved strictly unmodified in logic to reproduce the 1.33s stall delay and pre-impact false trigger).
3. F3b_improved (/odometry/comp_slip_gate_improved): Improved Slip/Stall Gate using short-window (0.20s, 3-scan)
   LiDAR scan-matched velocity cross-check + bumper contact check + sensor timeout fail-safe.
"""
import math
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
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
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('alpha_heading', 0.98)
        self.declare_parameter('slip_vel_threshold', 0.035)
        self.declare_parameter('stall_range_rate_threshold', 0.018)
        self.declare_parameter('contact_range_threshold', 0.24)

        self.alpha = float(self.get_parameter('alpha_heading').value)
        self.slip_vel_thresh = float(self.get_parameter('slip_vel_threshold').value)
        self.stall_rate_thresh = float(self.get_parameter('stall_range_rate_threshold').value)
        self.contact_range_thresh = float(self.get_parameter('contact_range_threshold').value)

        # F3a state
        self.x_f3a = 0.0
        self.y_f3a = 0.0
        self.yaw_f3a = 0.0

        # F3b (original preserved) state
        self.x_f3b = 0.0
        self.y_f3b = 0.0
        self.yaw_f3b = 0.0

        # F3b_improved state
        self.x_f3b_imp = 0.0
        self.y_f3b_imp = 0.0
        self.yaw_f3b_imp = 0.0

        # IMU state
        self.imu_yaw0 = None
        self.imu_rel_yaw = 0.0
        self.imu_wz = 0.0
        self.imu_ax = 0.0
        self.last_imu_t = None

        # Original F3b scan history (maxlen=15 -> 1.4s window, preserved as-is)
        self.scan_history = deque(maxlen=15)
        self.slip_gate_active = False
        self.slip_scale = 1.0
        self.latest_front_rate = 0.0
        self.latest_front_min = 10.0
        self.last_scan_t = None

        # Improved F3b short-window F4 LiDAR pose history (maxlen=3 -> ~0.20s window)
        self.lidar_pose_history = deque(maxlen=3)
        self.imp_gate_active = False
        self.imp_slip_scale = 1.0
        self.imp_lidar_speed = 0.0

        self.last_odom_t = None

        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.on_imu, 50)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.sub_lidar_odom = self.create_subscription(Odometry, '/odometry/lidar_odom', self.on_lidar_odom, 20)

        self.pub_f3a = self.create_publisher(Odometry, '/odometry/comp_filter', 20)
        self.pub_f3b = self.create_publisher(Odometry, '/odometry/comp_slip_gate', 20)
        self.pub_f3b_imp = self.create_publisher(Odometry, '/odometry/comp_slip_gate_improved', 20)
        self.pub_diag = self.create_publisher(Float32MultiArray, '/diagnostics/slip_gate', 10)

        self.get_logger().info(
            "ComplementaryFilterNode initialized: F3a (/odometry/comp_filter), "
            "F3b original (/odometry/comp_slip_gate), F3b_improved (/odometry/comp_slip_gate_improved)."
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
                gyro_pred_a = wrap_angle(self.yaw_f3a + self.imu_wz * dt)
                err_a = wrap_angle(self.imu_rel_yaw - gyro_pred_a)
                self.yaw_f3a = wrap_angle(gyro_pred_a + (1.0 - self.alpha) * err_a)
                self.yaw_f3b = self.yaw_f3a
                self.yaw_f3b_imp = self.yaw_f3a

        self.last_imu_t = t_sec

    def on_lidar_odom(self, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lx = msg.pose.pose.position.x
        ly = msg.pose.pose.position.y
        lvx = msg.twist.twist.linear.x
        lvy = msg.twist.twist.linear.y
        self.lidar_pose_history.append((t_sec, lx, ly, math.hypot(lvx, lvy)))

    def on_scan(self, msg: LaserScan):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_scan_t = t_sec
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = len(ranges)
        if n == 0:
            return
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

        front_mask = (np.abs(angles) <= math.radians(20.0)) & np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        rear_mask = (np.abs(np.abs(angles) - math.pi) <= math.radians(20.0)) & np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)

        front_med = float(np.median(ranges[front_mask])) if np.any(front_mask) else 10.0
        front_min = float(np.min(ranges[front_mask])) if np.any(front_mask) else 10.0
        rear_med = float(np.median(ranges[rear_mask])) if np.any(rear_mask) else 10.0
        self.latest_front_min = front_min

        self.scan_history.append((t_sec, front_med, rear_med, front_min))

    def compute_original_f3b_scale(self, t_sec: float, vx_wheel: float) -> float:
        """Original F3b logic preserved verbatim from 75-trial benchmark."""
        if abs(vx_wheel) < self.slip_vel_thresh:
            self.slip_gate_active = False
            return 1.0

        if vx_wheel > self.slip_vel_thresh and self.latest_front_min < self.contact_range_thresh:
            self.slip_gate_active = True
            return 0.0

        if len(self.scan_history) >= 3:
            t_old, f_old, r_old, _ = self.scan_history[0]
            t_new, f_new, r_new, _ = self.scan_history[-1]
            dt_scan = t_new - t_old
            if dt_scan >= 0.15 and (t_sec - t_new) <= 0.35:
                if abs(self.imu_wz) < 0.25:
                    if vx_wheel > 0.0 and f_new < 4.5 and f_old < 4.5:
                        front_rate = (f_old - f_new) / dt_scan
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

    def compute_improved_f3b_scale(self, t_sec: float, vx_wheel: float) -> float:
        """
        Improved Slip/Stall Gate (F3b_improved):
        1. Fail-safe on sensor loss: if /scan or /odometry/lidar_odom is stale (> 0.45s), gate wheel integration.
        2. Short 0.20s (2-scan) LiDAR ICP displacement velocity comparison against wheel velocity:
           Eliminates the 1.33s deque lag and eliminates false triggers from furniture behind glass walls.
        """
        if abs(vx_wheel) < 0.04:
            self.imp_gate_active = False
            return 1.0

        # Fail-safe if scan is missing or stale (> 0.45s)
        if self.last_scan_t is None or (t_sec - self.last_scan_t) > 0.45:
            self.imp_gate_active = True
            return 0.0

        # Direct bumper proximity check
        if vx_wheel > 0.04 and self.latest_front_min < 0.22:
            self.imp_gate_active = True
            return 0.0

        # Short-window (2-3 scans, ~0.18-0.25s) LiDAR scan-matched velocity check
        if len(self.lidar_pose_history) >= 2:
            t0, x0, y0, _ = self.lidar_pose_history[0]
            t1, x1, y1, v_inst = self.lidar_pose_history[-1]
            dt_l = t1 - t0
            if (t_sec - t1) > 0.45:
                # LiDAR odometry stale -> fail-safe gate
                self.imp_gate_active = True
                return 0.0
            if dt_l >= 0.08:
                v_disp = math.hypot(x1 - x0, y1 - y0) / dt_l
                self.imp_lidar_speed = 0.5 * v_disp + 0.5 * v_inst
                if abs(vx_wheel) > 0.07 and self.imp_lidar_speed < 0.025:
                    self.imp_gate_active = True
                    return 0.0

        self.imp_gate_active = False
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

        if self.imu_yaw0 is None:
            odom_yaw = quat_to_yaw(msg.pose.pose.orientation)
            self.yaw_f3a = odom_yaw
            self.yaw_f3b = odom_yaw
            self.yaw_f3b_imp = odom_yaw
            wz_used = wz_odom
        else:
            wz_used = self.imu_wz

        c = math.cos(self.yaw_f3a)
        s = math.sin(self.yaw_f3a)

        # 1. F3a (Standalone)
        self.x_f3a += (c * vx - s * vy) * dt
        self.y_f3a += (s * vx + c * vy) * dt
        self.publish_odom(self.pub_f3a, msg, self.x_f3a, self.y_f3a, self.yaw_f3a, vx, vy, wz_used)

        # 2. F3b (Original preserved)
        self.slip_scale = self.compute_original_f3b_scale(t_sec, vx)
        vx_b = vx * self.slip_scale
        vy_b = vy * self.slip_scale
        self.x_f3b += (c * vx_b - s * vy_b) * dt
        self.y_f3b += (s * vx_b + c * vy_b) * dt
        self.publish_odom(self.pub_f3b, msg, self.x_f3b, self.y_f3b, self.yaw_f3b, vx_b, vy_b, wz_used)

        # 3. F3b_improved (Short-window LiDAR velocity cross-check + sensor fail-safe)
        self.imp_slip_scale = self.compute_improved_f3b_scale(t_sec, vx)
        vx_imp = vx * self.imp_slip_scale
        vy_imp = vy * self.imp_slip_scale
        self.x_f3b_imp += (c * vx_imp - s * vy_imp) * dt
        self.y_f3b_imp += (s * vx_imp + c * vy_imp) * dt
        self.publish_odom(self.pub_f3b_imp, msg, self.x_f3b_imp, self.y_f3b_imp, self.yaw_f3b_imp, vx_imp, vy_imp, wz_used)

        diag = Float32MultiArray()
        diag.data = [
            float(vx),
            float(vx_b),
            float(self.latest_front_rate),
            1.0 if self.slip_gate_active else 0.0,
            float(vx_imp),
            1.0 if self.imp_gate_active else 0.0,
            float(self.imp_lidar_speed),
        ]
        self.pub_diag.publish(diag)

    def publish_odom(self, pub, src_msg, x, y, yaw, vx, vy, wz):
        m = Odometry()
        m.header.stamp = src_msg.header.stamp
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_footprint'
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation = yaw_to_quat(yaw)
        m.twist.twist.linear.x = vx
        m.twist.twist.linear.y = vy
        m.twist.twist.angular.z = wz
        m.pose.covariance = src_msg.pose.covariance
        m.twist.covariance = src_msg.twist.covariance
        pub.publish(m)


def main():
    rclpy.init()
    node = ComplementaryFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
