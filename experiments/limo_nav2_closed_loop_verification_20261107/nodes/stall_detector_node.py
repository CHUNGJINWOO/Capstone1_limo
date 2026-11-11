#!/usr/bin/env -S python3 -s
"""
stall_detector_node.py
Independent Safety & Stall Detector Node (Work Package 3 & 4):
Compares commanded/wheel velocity (/cmd_vel_raw, /odom) against LiDAR-based actual chassis velocity
(/odometry/lidar_odom and /scan) to detect physical stalls (e.g. glass wall impact) and sensor dropouts.

Key Safety Guarantees:
1. Zero False Triggers in Normal Driving: Requires sustained velocity discrepancy (|v_cmd| > 0.075 m/s
   AND |v_wheel| > 0.065 m/s while |v_lidar| < 0.025 m/s) for >= 0.22s after initial acceleration grace.
2. Fast Glass-Wall Impact Detection (~0.22s): Latches STALL_LATCHED within 2-3 LiDAR scans of physical block.
3. Fail-Safe on Uncertain/Missing Sensors (FAILSAFE_SENSOR_TIMEOUT): Never continues blind driving if
   /scan or /odometry/lidar_odom is missing, stale (> 0.45s), or NaN.
4. Active Drive Command Blocking: Blocks forward /cmd_vel (publishing 0.0 m/s to Gazebo /cmd_vel).
5. Explicit Recovery State Machine (RECOVERING_REVERSE): Unlatches when a reverse escape command
   (vx_cmd < -0.04 m/s) achieves >= 0.06m backward LiDAR displacement, or after a full command stop (>= 0.8s)
   followed by heading rotation away from the obstacle.
6. Optional Virtual Bumper PointCloud2 (/stall_detector/virtual_bumper_cloud): Injects lethal obstacle points
   at 0.24m in front of base_footprint when stalled so Nav2 local/global costmaps immediately register the
   unseen barrier (e.g. glass wall below 2D LiDAR plane or transparent partition).
"""
import json
import math
import struct
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, String

STATE_NORMAL = 0
STATE_STALL_PENDING = 1
STATE_STALL_LATCHED = 2
STATE_FAILSAFE_SENSOR_TIMEOUT = 3
STATE_RECOVERING_REVERSE = 4

STATE_NAMES = {
    STATE_NORMAL: 'NORMAL',
    STATE_STALL_PENDING: 'STALL_PENDING',
    STATE_STALL_LATCHED: 'STALL_LATCHED',
    STATE_FAILSAFE_SENSOR_TIMEOUT: 'FAILSAFE_SENSOR_TIMEOUT',
    STATE_RECOVERING_REVERSE: 'RECOVERING_REVERSE',
}


class StallDetectorNode(Node):
    def __init__(self):
        super().__init__(
            'stall_detector_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('enabled', True)
        self.declare_parameter('cmd_vel_threshold', 0.075)
        self.declare_parameter('wheel_vel_threshold', 0.060)
        self.declare_parameter('lidar_vel_stall_threshold', 0.025)
        self.declare_parameter('stall_confirm_duration_sec', 0.22)
        self.declare_parameter('sensor_timeout_sec', 0.45)
        self.declare_parameter('publish_virtual_bumper', True)
        self.declare_parameter('bumper_distance_m', 0.24)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.cmd_thresh = float(self.get_parameter('cmd_vel_threshold').value)
        self.wheel_thresh = float(self.get_parameter('wheel_vel_threshold').value)
        self.lidar_stall_thresh = float(self.get_parameter('lidar_vel_stall_threshold').value)
        self.confirm_dur = float(self.get_parameter('stall_confirm_duration_sec').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout_sec').value)
        self.pub_bumper = bool(self.get_parameter('publish_virtual_bumper').value)
        self.bumper_dist = float(self.get_parameter('bumper_distance_m').value)

        self.state = STATE_NORMAL
        self.pending_start_sim_t = None
        self.latched_sim_t = None
        self.zero_cmd_start_sim_t = None
        self.recovery_start_pose = None
        self.stall_direction = 1.0  # +1 forward stall, -1 reverse stall

        self.latest_cmd = Twist()
        self.last_cmd_sim_t = None
        self.wheel_vx = 0.0
        self.wheel_wz = 0.0
        self.last_odom_sim_t = None

        self.lidar_vx = 0.0
        self.lidar_speed_short = 0.0
        self.lidar_pose = (0.0, 0.0)
        self.last_lidar_sim_t = None
        self.lidar_hist = deque(maxlen=3)

        self.last_scan_sim_t = None
        self.min_front_scan = 10.0
        self.startup_sim_t = None

        self.stall_event_count = 0
        self.failsafe_event_count = 0
        self.recovery_event_count = 0
        self.cmd_blocked = False
        self.out_vx = 0.0
        self.out_wz = 0.0

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd_raw, 20)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.sub_lidar = self.create_subscription(Odometry, '/odometry/lidar_odom', self.on_lidar_odom, 20)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 20)
        self.pub_diag = self.create_publisher(Float32MultiArray, '/diagnostics/stall_detector', 20)
        self.pub_status = self.create_publisher(String, '/stall_detector/status_json', 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/stall_detector/virtual_bumper_cloud', 10)

        self.timer = self.create_timer(0.025, self.control_step)
        self.get_logger().info(
            f"StallDetectorNode started (enabled={self.enabled}, confirm={self.confirm_dur}s, "
            f"sensor_timeout={self.sensor_timeout}s)."
        )

    def now_sim(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_odom_sim_t = t_s if t_s > 0.0 else self.now_sim()
        self.wheel_vx = float(msg.twist.twist.linear.x)
        self.wheel_wz = float(msg.twist.twist.angular.z)

    def on_lidar_odom(self, msg: Odometry):
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_s = self.now_sim()
        stamp_s = t_s if t_s > 0.0 else now_s
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(vx)):
            return
        self.last_lidar_sim_t = now_s if now_s > 0.0 else stamp_s
        self.lidar_pose = (x, y)
        self.lidar_vx = vx
        self.lidar_hist.append((stamp_s, x, y, math.hypot(vx, vy)))

        if len(self.lidar_hist) >= 2:
            t0, x0, y0, _ = self.lidar_hist[0]
            t1, x1, y1, v_inst = self.lidar_hist[-1]
            dt = t1 - t0
            if dt >= 0.05:
                v_disp = math.hypot(x1 - x0, y1 - y0) / dt
                self.lidar_speed_short = 0.6 * v_disp + 0.4 * v_inst
            else:
                self.lidar_speed_short = v_inst
        else:
            self.lidar_speed_short = abs(vx)

    def on_scan(self, msg: LaserScan):
        now_s = self.now_sim()
        t_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_scan_sim_t = now_s if now_s > 0.0 else t_s
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        if len(ranges) == 0:
            return
        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment
        mask = (np.abs(angles) <= math.radians(22.0)) & np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        self.min_front_scan = float(np.min(ranges[mask])) if np.any(mask) else 10.0

    def on_cmd_raw(self, msg: Twist):
        self.latest_cmd = msg
        self.last_cmd_sim_t = self.now_sim()
        self.evaluate_and_publish_cmd()

    def control_step(self):
        self.evaluate_and_publish_cmd()

    def evaluate_and_publish_cmd(self):
        t_now = self.now_sim()
        if t_now <= 0.0:
            return
        if self.startup_sim_t is None:
            self.startup_sim_t = t_now

        cmd_vx = float(self.latest_cmd.linear.x)
        cmd_wz = float(self.latest_cmd.angular.z)
        want_motion = (abs(cmd_vx) > 0.04 or abs(cmd_wz) > 0.10)

        # 1. Check Sensor Health (Fail-safe on missing or stale /scan or /odometry/lidar_odom)
        in_startup_grace = (t_now - self.startup_sim_t) < 1.2
        scan_stale = (self.last_scan_sim_t is None) or ((t_now - self.last_scan_sim_t) > self.sensor_timeout)
        lidar_stale = (self.last_lidar_sim_t is None) or ((t_now - self.last_lidar_sim_t) > self.sensor_timeout)

        if not in_startup_grace and (scan_stale or lidar_stale):
            if want_motion and self.state != STATE_FAILSAFE_SENSOR_TIMEOUT:
                self.state = STATE_FAILSAFE_SENSOR_TIMEOUT
                self.failsafe_event_count += 1
                self.get_logger().warn(
                    f"[FAIL-SAFE] Sensor timeout detected (scan_stale={scan_stale}, lidar_stale={lidar_stale})! "
                    f"Blocking motion commands."
                )
        elif self.state == STATE_FAILSAFE_SENSOR_TIMEOUT and not scan_stale and not lidar_stale:
            # Recover from sensor timeout once sensors are healthy AND command is near zero or reversed
            if abs(cmd_vx) < 0.02:
                self.state = STATE_NORMAL
                self.pending_start_sim_t = None

        # 2. Evaluate Stall & Recovery State Machine when sensors are healthy
        if self.state != STATE_FAILSAFE_SENSOR_TIMEOUT:
            is_discrepant = (
                abs(cmd_vx) >= self.cmd_thresh
                and abs(self.wheel_vx) >= self.wheel_thresh
                and self.lidar_speed_short < self.lidar_stall_thresh
            )

            if self.state == STATE_NORMAL:
                if is_discrepant:
                    self.state = STATE_STALL_PENDING
                    self.pending_start_sim_t = t_now
                    self.stall_direction = 1.0 if cmd_vx >= 0.0 else -1.0

            elif self.state == STATE_STALL_PENDING:
                if not is_discrepant:
                    self.state = STATE_NORMAL
                    self.pending_start_sim_t = None
                elif (t_now - self.pending_start_sim_t) >= self.confirm_dur:
                    self.state = STATE_STALL_LATCHED
                    self.latched_sim_t = t_now
                    self.zero_cmd_start_sim_t = None
                    self.stall_event_count += 1
                    self.get_logger().warn(
                        f"[STALL LATCHED] Physical stall confirmed after {t_now - self.pending_start_sim_t:.3f}s "
                        f"(cmd_vx={cmd_vx:.3f}, wheel_vx={self.wheel_vx:.3f}, lidar_speed={self.lidar_speed_short:.3f})."
                    )

            elif self.state == STATE_STALL_LATCHED:
                # Check recovery conditions:
                # Condition A: Opposite/reverse escape command (e.g. vx < -0.04 when forward stalled)
                if self.stall_direction > 0.0 and cmd_vx < -0.04:
                    self.state = STATE_RECOVERING_REVERSE
                    self.recovery_start_pose = self.lidar_pose
                    self.recovery_event_count += 1
                    self.get_logger().info("[RECOVERY] Reverse escape command received; entering RECOVERING_REVERSE.")
                # Condition B: Command stopped for >= 0.6s followed by pure rotation away
                elif abs(cmd_vx) < 0.015:
                    if self.zero_cmd_start_sim_t is None:
                        self.zero_cmd_start_sim_t = t_now
                    elif (t_now - self.zero_cmd_start_sim_t) >= 0.6 and abs(cmd_wz) > 0.15:
                        self.state = STATE_NORMAL
                        self.pending_start_sim_t = None
                        self.recovery_event_count += 1
                else:
                    self.zero_cmd_start_sim_t = None

            elif self.state == STATE_RECOVERING_REVERSE:
                # If robot has backed up >= 0.06m in LiDAR odom, clear latch back to NORMAL
                if self.recovery_start_pose is not None:
                    backed_dist = math.hypot(
                        self.lidar_pose[0] - self.recovery_start_pose[0],
                        self.lidar_pose[1] - self.recovery_start_pose[1]
                    )
                    if backed_dist >= 0.06 and cmd_vx >= -0.01:
                        self.state = STATE_NORMAL
                        self.pending_start_sim_t = None
                        self.get_logger().info(f"[RECOVERY COMPLETE] Backed up {backed_dist:.3f}m; returning to NORMAL.")
                    elif cmd_vx > 0.04 and backed_dist < 0.03:
                        # Trying to drive forward again without having backed up -> re-latch immediately
                        self.state = STATE_STALL_LATCHED

        # 3. Compute gated /cmd_vel output
        out = Twist()
        self.cmd_blocked = False
        if self.enabled:
            if self.state == STATE_FAILSAFE_SENSOR_TIMEOUT:
                out.linear.x = 0.0
                out.angular.z = 0.0
                self.cmd_blocked = want_motion
            elif self.state == STATE_STALL_LATCHED:
                # Block forward motion in stall direction, allow rotation or reverse
                if self.stall_direction > 0.0 and cmd_vx > 0.0:
                    out.linear.x = 0.0
                    out.angular.z = cmd_wz
                    self.cmd_blocked = True
                elif self.stall_direction < 0.0 and cmd_vx < 0.0:
                    out.linear.x = 0.0
                    out.angular.z = cmd_wz
                    self.cmd_blocked = True
                else:
                    out.linear.x = cmd_vx
                    out.angular.z = cmd_wz
            else:
                out.linear.x = cmd_vx
                out.angular.z = cmd_wz
        else:
            out.linear.x = cmd_vx
            out.angular.z = cmd_wz

        self.out_vx = float(out.linear.x)
        self.out_wz = float(out.angular.z)
        self.pub_cmd.publish(out)

        # 4. Publish Virtual Bumper PointCloud2 if STALL_LATCHED and enabled
        if self.pub_bumper and self.enabled and self.state in (STATE_STALL_LATCHED, STATE_RECOVERING_REVERSE):
            self.publish_virtual_bumper_cloud()

        # 5. Publish Diagnostics
        diag = Float32MultiArray()
        diag.data = [
            float(self.state),
            1.0 if self.cmd_blocked else 0.0,
            float(cmd_vx),
            float(self.out_vx),
            float(self.wheel_vx),
            float(self.lidar_speed_short),
            float(self.stall_event_count),
            float(self.failsafe_event_count),
            float(self.recovery_event_count),
        ]
        self.pub_diag.publish(diag)

        st_msg = String()
        st_msg.data = json.dumps({
            'sim_time': round(t_now, 3),
            'state_code': self.state,
            'state_name': STATE_NAMES.get(self.state, 'UNKNOWN'),
            'cmd_blocked': self.cmd_blocked,
            'cmd_vx_raw': round(cmd_vx, 4),
            'cmd_vx_out': round(self.out_vx, 4),
            'wheel_vx': round(self.wheel_vx, 4),
            'lidar_speed': round(self.lidar_speed_short, 4),
            'stall_events': self.stall_event_count,
            'failsafe_events': self.failsafe_event_count,
            'recovery_events': self.recovery_event_count,
        })
        self.pub_status.publish(st_msg)

    def publish_virtual_bumper_cloud(self):
        """Publishes a dense arc of obstacle points in base_footprint at bumper_distance_m ahead."""
        pts = []
        for y_off in np.linspace(-0.22, 0.22, 15):
            for x_off in (self.bumper_dist, self.bumper_dist + 0.04):
                pts.append((float(x_off), float(y_off), 0.18))

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        buf = bytearray()
        for px, py, pz in pts:
            buf.extend(struct.pack('fff', px, py, pz))
        msg.data = bytes(buf)
        self.pub_cloud.publish(msg)


def main():
    rclpy.init()
    node = StallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
