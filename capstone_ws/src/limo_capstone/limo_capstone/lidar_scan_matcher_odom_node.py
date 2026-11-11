#!/usr/bin/env -S python3 -s
"""
lidar_scan_matcher_odom_node.py
Implements F4: 2D LiDAR Scan-Matching Odometry (/odometry/lidar_odom)
with explicit separation of:
1. Actual ICP algorithmic compute latency (`icp_compute_wall_ms`, measured via time.perf_counter())
2. Sim-time publication delay (`sim_publish_delay_ms`, clock.now() - scan.header.stamp)
3. Sensor scan period (`scan_period_ms`, ~100 ms for 10 Hz LiDAR)

Publishes:
- /odometry/lidar_odom (nav_msgs/Odometry)
- /diagnostics/f4_latency (std_msgs/Float32MultiArray):
  [icp_compute_wall_ms, sim_publish_delay_ms, scan_period_ms, inlier_count, vx, wz]
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32MultiArray


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def mat_to_se2(T: np.ndarray):
    x = float(T[0, 2])
    y = float(T[1, 2])
    yaw = math.atan2(float(T[1, 0]), float(T[0, 0]))
    return x, y, yaw


class LidarScanMatcherOdomNode(Node):
    def __init__(self):
        super().__init__(
            'lidar_scan_matcher_odom_node',
            automatically_declare_parameters_from_overrides=True
        )
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        if not self.has_parameter('laser_x_offset'):
            self.declare_parameter('laser_x_offset', 0.103)
        if not self.has_parameter('max_icp_iterations'):
            self.declare_parameter('max_icp_iterations', 18)
        if not self.has_parameter('max_correspondence_dist'):
            self.declare_parameter('max_correspondence_dist', 0.30)

        self.laser_x_offset = float(self.get_parameter('laser_x_offset').value)
        self.max_iter = int(self.get_parameter('max_icp_iterations').value)
        self.max_dist = float(self.get_parameter('max_correspondence_dist').value)

        self.keyframe_pts = None
        self.T_odom_keyframe = np.eye(3, dtype=np.float64)
        self.T_keyframe_curr = np.eye(3, dtype=np.float64)
        self.T_odom_curr = np.eye(3, dtype=np.float64)

        self.prev_t = None
        self.prev_pose = (0.0, 0.0, 0.0)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_inliers = 350.0

        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, '/odometry/lidar_odom', 20)
        self.pub_lat = self.create_publisher(Float32MultiArray, '/diagnostics/f4_latency', 20)

        self.get_logger().info("LidarScanMatcherOdomNode (F4) initialized with latency diagnostics.")

    def scan_to_points(self, msg: LaserScan) -> np.ndarray:
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        n = len(ranges)
        angles = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= max(0.15, msg.range_min)) & (ranges <= min(9.5, msg.range_max))
        r = ranges[valid]
        a = angles[valid]
        if len(r) < 20:
            return np.empty((0, 2), dtype=np.float64)
        px = r * np.cos(a) + self.laser_x_offset
        py = r * np.sin(a)
        return np.column_stack((px, py))

    def icp_2d(self, src_pts: np.ndarray, dst_pts: np.ndarray, init_T: np.ndarray) -> np.ndarray:
        T = init_T.copy()
        max_d2 = self.max_dist * self.max_dist
        inlier_cnt = 0

        for _ in range(self.max_iter):
            R = T[:2, :2]
            t = T[:2, 2]
            transformed = (src_pts @ R.T) + t

            diff = transformed[:, None, :] - dst_pts[None, :, :]
            dist2 = np.sum(diff * diff, axis=2)
            nn_idx = np.argmin(dist2, axis=1)
            min_d2 = dist2[np.arange(len(src_pts)), nn_idx]

            inliers = min_d2 < max_d2
            inlier_cnt = int(np.count_nonzero(inliers))
            if inlier_cnt < 12:
                break

            inlier_indices = np.where(inliers)[0]
            inlier_d2 = min_d2[inlier_indices]
            cutoff = np.percentile(inlier_d2, 85.0)
            keep = inlier_indices[inlier_d2 <= cutoff]
            if len(keep) < 10:
                keep = inlier_indices

            p = transformed[keep]
            q = dst_pts[nn_idx[keep]]

            p_mean = np.mean(p, axis=0)
            q_mean = np.mean(q, axis=0)
            p_centered = p - p_mean
            q_centered = q - q_mean

            H = p_centered.T @ q_centered
            U, _, Vt = np.linalg.svd(H)
            R_delta = Vt.T @ U.T
            if np.linalg.det(R_delta) < 0:
                Vt[1, :] *= -1
                R_delta = Vt.T @ U.T
            t_delta = q_mean - (R_delta @ p_mean)

            T_delta = np.eye(3, dtype=np.float64)
            T_delta[:2, :2] = R_delta
            T_delta[:2, 2] = t_delta
            T = T_delta @ T

            step_norm = math.hypot(t_delta[0], t_delta[1]) + abs(math.atan2(R_delta[1, 0], R_delta[0, 0]))
            if step_norm < 1e-4:
                break

        self.last_inliers = float(inlier_cnt)
        return T

    def on_scan(self, msg: LaserScan):
        t_wall_start = time.perf_counter()
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        pts = self.scan_to_points(msg)
        if len(pts) < 20:
            return

        if self.keyframe_pts is None:
            self.keyframe_pts = pts
            self.prev_t = t_sec
            self.last_inliers = float(len(pts))
            self.publish_odom_msg(msg.header.stamp, 0.0, 0.0, 0.0)
            return

        self.T_keyframe_curr = self.icp_2d(pts, self.keyframe_pts, self.T_keyframe_curr)
        self.T_odom_curr = self.T_odom_keyframe @ self.T_keyframe_curr

        dx_kf, dy_kf, dyaw_kf = mat_to_se2(self.T_keyframe_curr)
        if math.hypot(dx_kf, dy_kf) >= 0.12 or abs(dyaw_kf) >= 0.12:
            self.keyframe_pts = pts
            self.T_odom_keyframe = self.T_odom_curr.copy()
            self.T_keyframe_curr = np.eye(3, dtype=np.float64)

        x, y, yaw = mat_to_se2(self.T_odom_curr)

        scan_period_ms = 100.0
        if self.prev_t is not None:
            dt = t_sec - self.prev_t
            scan_period_ms = dt * 1000.0
            if 0.005 <= dt <= 0.5:
                dx = x - self.prev_pose[0]
                dy = y - self.prev_pose[1]
                dyaw = wrap_angle(yaw - self.prev_pose[2])
                vx_raw = (math.cos(yaw) * dx + math.sin(yaw) * dy) / dt
                vy_raw = (-math.sin(yaw) * dx + math.cos(yaw) * dy) / dt
                wz_raw = dyaw / dt
                self.vx = 0.55 * vx_raw + 0.45 * self.vx
                self.vy = 0.55 * vy_raw + 0.45 * self.vy
                self.wz = 0.55 * wz_raw + 0.45 * self.wz

        self.prev_t = t_sec
        self.prev_pose = (x, y, yaw)
        icp_compute_wall_ms = (time.perf_counter() - t_wall_start) * 1000.0

        self.publish_odom_msg(msg.header.stamp, x, y, yaw)

        now_sim = self.get_clock().now().nanoseconds * 1e-9
        sim_pub_delay_ms = max(0.0, (now_sim - t_sec) * 1000.0) if now_sim > 0.0 else 0.0

        lat_msg = Float32MultiArray()
        lat_msg.data = [
            float(icp_compute_wall_ms),
            float(sim_pub_delay_ms),
            float(scan_period_ms),
            float(self.last_inliers),
            float(self.vx),
            float(self.wz),
        ]
        self.pub_lat.publish(lat_msg)

    def publish_odom_msg(self, stamp, x: float, y: float, yaw: float):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(yaw)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz

        odom.pose.covariance[0] = 0.004
        odom.pose.covariance[7] = 0.004
        odom.pose.covariance[35] = 0.004
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.015
        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanMatcherOdomNode()
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
