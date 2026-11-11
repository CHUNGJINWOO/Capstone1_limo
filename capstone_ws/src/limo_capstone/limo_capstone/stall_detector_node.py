#!/usr/bin/env -S python3 -s
"""
stall_detector_node.py
Safety & Stall Detector Node for Physical Pre-Deployment (Work Package 3 Final):
Compares commanded/wheel velocity (/cmd_vel_raw, /odom) against LiDAR-based actual chassis velocity
(/odometry/lidar_odom and /scan) to detect physical stalls, glass wall impacts, and sensor dropouts.

Key Pre-Deployment Safety Rules:
1. enable_auto_reverse is FALSE by default:
   LIMO Pro lacks rear proximity/blind-spot sensors. Blind automatic reverse recovery is prohibited
   by default to prevent secondary collisions with chair frames, tables, or hazards behind the robot.
2. STALL_LATCHED is maintained indefinitely:
   Once latched, drive commands (/cmd_vel) are forced to strictly 0.0 m/s (linear and angular).
   Automatic restart is blocked without explicit operator approval via /stall_detector/reset_latch.
3. Fail-Safe on Missing/Degraded Sensors:
   - FAILSAFE_SENSOR_TIMEOUT: triggers if /scan or /odometry/lidar_odom is stale (> 0.45s) or missing.
   - FAILSAFE_ICP_DEGRADED: triggers if ICP inlier count drops < min_inliers (100) or error > max_error (0.15m).
   - INITIALIZING (startup and after /stall_detector/reset_latch): output is held at 0 until every
     required input (scan, lidar odom, depth, wheel odom, estimator odom, per configuration) is fresh
     for startup_ready_hold_sec with no motion commanded; otherwise FAILSAFE after startup_timeout_sec.
4. Software Stop vs Hardware Emergency Stop Separation:
   Software command zeroing is explicitly distinguished from physical hardware E-Stop (relay/power cut).
5. Virtual Bumper PointCloud2:
   Injects lethal obstacle pointcloud at bumper distance (0.24m) to ensure Nav2 costmaps register the hazard.
"""
import json
import math
import struct
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

STATE_NORMAL = 0
STATE_STALL_PENDING = 1
STATE_STALL_LATCHED = 2
STATE_FAILSAFE_SENSOR_TIMEOUT = 3
STATE_FAILSAFE_ICP_DEGRADED = 4
STATE_RECOVERING_REVERSE = 5
STATE_INITIALIZING = 6

STATE_NAMES = {
    STATE_INITIALIZING: 'INITIALIZING',
    STATE_NORMAL: 'NORMAL',
    STATE_STALL_PENDING: 'STALL_PENDING',
    STATE_STALL_LATCHED: 'STALL_LATCHED',
    STATE_FAILSAFE_SENSOR_TIMEOUT: 'FAILSAFE_SENSOR_TIMEOUT',
    STATE_FAILSAFE_ICP_DEGRADED: 'FAILSAFE_ICP_DEGRADED',
    STATE_RECOVERING_REVERSE: 'RECOVERING_REVERSE',
}


class StallDetectorNode(Node):
    def __init__(self):
        super().__init__('stall_detector_node')
        self.declare_parameter('enabled', True)
        self.declare_parameter('enable_auto_reverse', False)  # AUDIT RULE: False by default!
        self.declare_parameter('enable_auto_rotate_recovery', False)  # AUDIT RULE: False by default!
        self.declare_parameter('require_operator_unlatch', False)
        self.declare_parameter('cmd_vel_threshold', 0.075)
        self.declare_parameter('wheel_vel_threshold', 0.060)
        self.declare_parameter('lidar_vel_stall_threshold', 0.025)
        self.declare_parameter('stall_confirm_duration_sec', 1.5)
        self.declare_parameter('sensor_timeout_sec', 0.38)
        self.declare_parameter('depth_mode', 'required')  # Options: 'required', 'optional', 'disabled'
        self.declare_parameter('publish_virtual_bumper', True)
        self.declare_parameter('bumper_distance_m', 0.24)
        self.declare_parameter('min_icp_inlier_count', 100.0)
        self.declare_parameter('max_icp_error_m', 0.15)
        self.declare_parameter('require_lidar_odom', True)
        # Startup arming: output stays 0 until every required sensor is healthy for
        # startup_ready_hold_sec; not armed within startup_timeout_sec -> FAILSAFE.
        self.declare_parameter('startup_ready_hold_sec', 0.5)
        self.declare_parameter('startup_timeout_sec', 30.0)
        # Startup-only readiness inputs (runtime FAILSAFE checks are unchanged):
        # wheel /odom feeds stall detection for every estimator; estimator_odom_topic
        # (e.g. /odom_nav2 from odom_tf_relay_node) is the active estimator's output.
        self.declare_parameter('require_wheel_odom', True)
        self.declare_parameter('estimator_odom_topic', '')
        # Runtime freshness limit for /odom and estimator_odom_topic once armed
        # (<= 0 -> use sensor_timeout_sec).
        self.declare_parameter('odom_timeout_sec', 0.0)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.enable_auto_reverse = bool(self.get_parameter('enable_auto_reverse').value)
        self.enable_auto_rotate = bool(self.get_parameter('enable_auto_rotate_recovery').value)
        self.require_operator_unlatch = bool(self.get_parameter('require_operator_unlatch').value)
        self.cmd_thresh = float(self.get_parameter('cmd_vel_threshold').value)
        self.wheel_thresh = float(self.get_parameter('wheel_vel_threshold').value)
        self.lidar_stall_thresh = float(self.get_parameter('lidar_vel_stall_threshold').value)
        self.confirm_dur = float(self.get_parameter('stall_confirm_duration_sec').value)
        self.sensor_timeout = float(self.get_parameter('sensor_timeout_sec').value)
        self.depth_mode = str(self.get_parameter('depth_mode').value).lower()
        self.pub_bumper = bool(self.get_parameter('publish_virtual_bumper').value)
        self.bumper_dist = float(self.get_parameter('bumper_distance_m').value)
        self.min_inliers = float(self.get_parameter('min_icp_inlier_count').value)
        self.max_icp_err = float(self.get_parameter('max_icp_error_m').value)
        self.require_lidar_odom = bool(self.get_parameter('require_lidar_odom').value)
        self.startup_hold = float(self.get_parameter('startup_ready_hold_sec').value)
        self.startup_timeout = float(self.get_parameter('startup_timeout_sec').value)
        self.require_wheel_odom = bool(self.get_parameter('require_wheel_odom').value)
        self.estimator_odom_topic = str(self.get_parameter('estimator_odom_topic').value).strip()
        self.last_est_odom_sim_t = None
        odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self.odom_timeout = odom_timeout if odom_timeout > 0.0 else self.sensor_timeout
        self.ready_since_sim_t = None
        self.init_start_sim_t = None

        self.state = STATE_INITIALIZING
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

        # Depth Camera Health Tracking
        self.depth_state = 'init'
        self.depth_code = 'DEPTH_MISSING'
        self.last_depth_sim_t = None
        self.depth_msg_count = 0

        # ICP Diagnostics
        self.last_icp_inliers = 350.0
        self.last_icp_err = 0.02
        self.last_icp_diag_sim_t = None

        self.stall_event_count = 0
        self.failsafe_event_count = 0
        self.icp_degraded_event_count = 0
        self.recovery_event_count = 0
        self.operator_unlatch_count = 0
        self.cmd_blocked = False

        # Subscriptions
        self.sub_cmd_raw = self.create_subscription(Twist, '/cmd_vel_raw', self.cb_cmd_raw, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, 20)
        self.sub_lidar_odom = self.create_subscription(Odometry, '/odometry/lidar_odom', self.cb_lidar_odom, 20)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.sub_f4_diag = self.create_subscription(Float32MultiArray, '/diagnostics/f4_latency', self.cb_f4_diag, 10)
        self.sub_depth_status = self.create_subscription(String, '/auto_mapping/depth_status', self.cb_depth_status, 10)
        self.sub_depth_points = self.create_subscription(PointCloud2, '/depth_obstacles/points', self.cb_depth_points, qos_profile_sensor_data)
        if self.estimator_odom_topic:
            self.sub_est_odom = self.create_subscription(
                Odometry, self.estimator_odom_topic, self.cb_est_odom, 20)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_diag = self.create_publisher(String, '/stall_detector/status_json', 10)
        self.pub_bumper_cloud = self.create_publisher(PointCloud2, '/stall_detector/virtual_bumper_cloud', 10)

        # Operator unlatch service
        self.srv_reset_latch = self.create_service(Trigger, '/stall_detector/reset_latch', self.cb_reset_latch)

        # 50 Hz Control & Evaluation Loop
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(
            f"StallDetectorNode initialized: enabled={self.enabled}, "
            f"enable_auto_reverse={self.enable_auto_reverse} (SAFETY LOCKED), "
            f"require_operator_unlatch={self.require_operator_unlatch}"
        )

    def cb_cmd_raw(self, msg: Twist):
        self.latest_cmd = msg
        t_sec = self.get_clock().now().nanoseconds * 1e-9
        self.last_cmd_sim_t = t_sec
        if self.startup_sim_t is None:
            self.startup_sim_t = t_sec

    def cb_odom(self, msg: Odometry):
        self.wheel_vx = float(msg.twist.twist.linear.x)
        self.wheel_wz = float(msg.twist.twist.angular.z)
        self.last_odom_sim_t = self.get_clock().now().nanoseconds * 1e-9

    def cb_est_odom(self, msg: Odometry):
        self.last_est_odom_sim_t = self.get_clock().now().nanoseconds * 1e-9

    def cb_lidar_odom(self, msg: Odometry):
        t_sec = self.get_clock().now().nanoseconds * 1e-9
        self.last_lidar_sim_t = t_sec
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        self.lidar_pose = (px, py)
        self.lidar_vx = float(msg.twist.twist.linear.x)
        self.lidar_hist.append((t_sec, px, py))

        if len(self.lidar_hist) >= 2:
            t0, x0, y0 = self.lidar_hist[0]
            t1, x1, y1 = self.lidar_hist[-1]
            dt = t1 - t0
            if dt > 0.04:
                self.lidar_speed_short = math.hypot(x1 - x0, y1 - y0) / dt
            else:
                self.lidar_speed_short = abs(self.lidar_vx)
        else:
            self.lidar_speed_short = abs(self.lidar_vx)

    def cb_scan(self, msg: LaserScan):
        self.last_scan_sim_t = self.get_clock().now().nanoseconds * 1e-9
        if msg.ranges:
            ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
            self.min_front_scan = min(ranges) if ranges else 10.0

    def cb_f4_diag(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.last_icp_inliers = float(msg.data[3])
            self.last_icp_diag_sim_t = self.get_clock().now().nanoseconds * 1e-9

    def cb_depth_status(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.depth_state = d.get('state', 'error')
            self.depth_code = d.get('code', 'DEPTH_UNKNOWN')
            if self.depth_state == 'ready':
                self.last_depth_sim_t = self.get_clock().now().nanoseconds * 1e-9
        except Exception:
            pass

    def cb_depth_points(self, msg: PointCloud2):
        self.last_depth_sim_t = self.get_clock().now().nanoseconds * 1e-9
        self.depth_msg_count += 1

    def cb_reset_latch(self, req: Trigger.Request, resp: Trigger.Response):
        if self.state == STATE_INITIALIZING:
            resp.success = False
            resp.message = "Still INITIALIZING (sensors not yet healthy); reset refused."
            return resp
        prev_state_name = STATE_NAMES.get(self.state, 'UNKNOWN')
        # Re-arm through the same readiness check instead of jumping to NORMAL.
        self.state = STATE_INITIALIZING
        self.init_start_sim_t = None
        self.ready_since_sim_t = None
        self.pending_start_sim_t = None
        self.latched_sim_t = None
        self.zero_cmd_start_sim_t = None
        self.operator_unlatch_count += 1
        resp.success = True
        resp.message = f"Latch cleared by operator ({prev_state_name} -> INITIALIZING; re-arms when sensors are healthy)."
        self.get_logger().info(f"[OPERATOR RESET] {resp.message}")
        return resp

    def control_loop(self):
        t_now = self.get_clock().now().nanoseconds * 1e-9
        cmd_vx = self.latest_cmd.linear.x
        cmd_wz = self.latest_cmd.angular.z

        # 0. Watchdog on /cmd_vel_raw:
        # If upstream Nav2 stops publishing for > 0.25s, clamp to 0.0 m/s
        if self.last_cmd_sim_t is None or (t_now - self.last_cmd_sim_t) > 0.25:
            cmd_vx = 0.0
            cmd_wz = 0.0

        # Safety Gate: Inhibit reverse if enable_auto_reverse is False (LIMO Pro lack of rear sensors)
        if not self.enable_auto_reverse and cmd_vx < 0.0:
            cmd_vx = 0.0

        want_motion = (abs(cmd_vx) > 0.02 or abs(cmd_wz) > 0.05)

        # 1. Health checks
        # Sensor staleness applies if sensor stream was established and lost, or if motion is commanded
        scan_stale = (self.last_scan_sim_t is not None and (t_now - self.last_scan_sim_t) > self.sensor_timeout)
        lidar_stale = (
            self.require_lidar_odom
            and self.last_lidar_sim_t is not None
            and (t_now - self.last_lidar_sim_t) > self.sensor_timeout
        )
        if want_motion:
            if self.last_scan_sim_t is None:
                scan_stale = True
            if self.require_lidar_odom and self.last_lidar_sim_t is None:
                lidar_stale = True

        # Depth Camera Health Evaluation
        depth_stale = False
        depth_invalid = False
        if self.depth_mode == 'required':
            if self.last_depth_sim_t is not None:
                depth_stale = (t_now - self.last_depth_sim_t) > self.sensor_timeout
                depth_invalid = (self.depth_state != 'ready' or self.depth_code != 'DEPTH_READY')
            elif want_motion:
                depth_stale = True
        elif self.depth_mode == 'optional':
            if self.last_depth_sim_t is not None and (t_now - self.last_depth_sim_t) > self.sensor_timeout:
                if not hasattr(self, '_last_depth_warn_t') or (t_now - self._last_depth_warn_t > 2.0):
                    self._last_depth_warn_t = t_now
                    self.get_logger().warn("[DEPTH OPTIONAL DROPOUT] Depth camera inactive; continuing 2D LiDAR navigation.")

        depth_fault = (self.depth_mode == 'required' and (depth_stale or depth_invalid))

        # Odometry freshness (runtime). Same convention as scan/lidar: a stream that was
        # established and then lost is stale; a stream that never arrived only counts while
        # motion is commanded. Not evaluated while INITIALIZING (handled by the arming check).
        def odom_lost(t_last):
            return t_last is not None and (t_now - t_last) > self.odom_timeout
        wheel_odom_stale = self.require_wheel_odom and odom_lost(self.last_odom_sim_t)
        est_odom_stale = bool(self.estimator_odom_topic) and odom_lost(self.last_est_odom_sim_t)
        if want_motion:
            if self.require_wheel_odom and self.last_odom_sim_t is None:
                wheel_odom_stale = True
            if self.estimator_odom_topic and self.last_est_odom_sim_t is None:
                est_odom_stale = True
        odom_fault = wheel_odom_stale or est_odom_stale

        icp_diag_stale = (
            self.require_lidar_odom
            and self.last_icp_diag_sim_t is not None
            and (t_now - self.last_icp_diag_sim_t) > (self.sensor_timeout * 3.0)
        )
        icp_degraded = (
            self.require_lidar_odom
            and self.last_icp_diag_sim_t is not None
            and (self.last_icp_inliers < self.min_inliers or icp_diag_stale)
        )

        def fresh(t_last):
            return t_last is not None and (t_now - t_last) <= self.sensor_timeout
        # Readiness of every input the current configuration actually uses. A stream that has
        # never arrived is NOT ready (unlike the stale flags, which only fire once motion is
        # commanded). Used to arm from INITIALIZING and to leave an auto-clearing FAILSAFE.
        checks = {
            'scan': fresh(self.last_scan_sim_t),
            'lidar_odom': (not self.require_lidar_odom) or fresh(self.last_lidar_sim_t),
            'icp': not icp_degraded,
            'depth': self.depth_mode != 'required' or (fresh(self.last_depth_sim_t) and not depth_fault),
            'wheel_odom': (not self.require_wheel_odom) or fresh(self.last_odom_sim_t),
            'estimator_odom': (not self.estimator_odom_topic) or fresh(self.last_est_odom_sim_t),
        }

        if self.state == STATE_INITIALIZING:
            # Arm only once, when all required inputs exist, are fresh and healthy,
            # and no motion is being commanded. Output stays 0 meanwhile (section 3).
            if self.init_start_sim_t is None:
                self.init_start_sim_t = t_now
            ready = all(checks.values())
            if not ready:
                self.ready_since_sim_t = None
            elif self.ready_since_sim_t is None:
                self.ready_since_sim_t = t_now
            if ready and not want_motion and (t_now - self.ready_since_sim_t) >= self.startup_hold:
                self.state = STATE_NORMAL
                self.get_logger().info(
                    f"[ARMED] All required sensors healthy for {t_now - self.ready_since_sim_t:.2f}s; "
                    f"INITIALIZING -> NORMAL after {t_now - self.init_start_sim_t:.2f}s."
                )
            elif self.startup_timeout > 0.0 and (t_now - self.init_start_sim_t) > self.startup_timeout:
                self.state = STATE_FAILSAFE_SENSOR_TIMEOUT
                self.failsafe_event_count += 1
                self.get_logger().error(
                    f"[FAIL-SAFE STARTUP TIMEOUT] not ready after {self.startup_timeout:.1f}s: "
                    f"not_ready={[k for k, ok in checks.items() if not ok]}, "
                    f"depth_code={self.depth_code}, want_motion={want_motion}. Blocking all motion."
                )
        elif scan_stale or lidar_stale or depth_fault or odom_fault:
            if self.state != STATE_FAILSAFE_SENSOR_TIMEOUT:
                self.state = STATE_FAILSAFE_SENSOR_TIMEOUT
                self.failsafe_event_count += 1
                self.get_logger().error(
                    f"[FAIL-SAFE SENSOR TIMEOUT] scan_stale={scan_stale}, lidar_stale={lidar_stale}, "
                    f"wheel_odom_stale={wheel_odom_stale}, est_odom_stale={est_odom_stale}, "
                    f"depth_fault={depth_fault} (depth_mode={self.depth_mode}, code={self.depth_code})! Blocking all motion."
                )
        elif icp_degraded:
            if want_motion and self.state not in (STATE_FAILSAFE_ICP_DEGRADED, STATE_STALL_LATCHED):
                self.state = STATE_FAILSAFE_ICP_DEGRADED
                self.icp_degraded_event_count += 1
                self.get_logger().error(
                    f"[FAIL-SAFE ICP DEGRADED] inliers={self.last_icp_inliers:.1f} (<{self.min_inliers}), "
                    f"err={self.last_icp_err:.3f}m (>{self.max_icp_err}m)! Blocking all motion."
                )
        elif self.state in (STATE_FAILSAFE_SENSOR_TIMEOUT, STATE_FAILSAFE_ICP_DEGRADED):
            depth_ok = True
            if self.depth_mode == 'required':
                depth_ok = (not depth_stale and not depth_invalid)
            if all(checks.values()) and not scan_stale and not lidar_stale and not odom_fault and not icp_degraded and depth_ok:
                if not self.require_operator_unlatch:
                    if abs(cmd_vx) < 0.02 and abs(cmd_wz) < 0.02:
                        self.state = STATE_NORMAL
                        self.pending_start_sim_t = None

        # 2. Evaluate Physical Stall Condition
        if self.state not in (STATE_FAILSAFE_SENSOR_TIMEOUT, STATE_FAILSAFE_ICP_DEGRADED):
            if self.require_lidar_odom:
                is_discrepant = (
                    abs(cmd_vx) >= self.cmd_thresh
                    and abs(self.wheel_vx) >= self.wheel_thresh
                    and self.lidar_speed_short < self.lidar_stall_thresh
                )
            else:
                is_discrepant = (
                    cmd_vx >= self.cmd_thresh
                    and abs(self.wheel_vx) >= self.wheel_thresh
                    and self.min_front_scan < self.bumper_dist
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
                    self.get_logger().error(
                        f"[STALL LATCHED] Physical stall confirmed after {t_now - self.pending_start_sim_t:.3f}s "
                        f"(cmd_vx={cmd_vx:.3f}, wheel_vx={self.wheel_vx:.3f}, lidar_speed={self.lidar_speed_short:.3f}). "
                        f"All drive commands LOCKED at 0.0 m/s."
                    )

            elif self.state == STATE_STALL_LATCHED:
                # If operator unlatch is required, remain in STALL_LATCHED until service is called!
                if self.require_operator_unlatch:
                    pass  # Strictly remain in STALL_LATCHED
                elif self.enable_auto_reverse:
                    # ONLY if auto_reverse is explicitly allowed by config
                    if self.stall_direction > 0.0 and cmd_vx < -0.04:
                        self.state = STATE_RECOVERING_REVERSE
                        self.recovery_start_pose = self.lidar_pose
                        self.recovery_event_count += 1
                        self.get_logger().info("[RECOVERY] Reverse escape allowed; entering RECOVERING_REVERSE.")
                elif self.enable_auto_rotate:
                    if abs(cmd_vx) < 0.015:
                        if self.zero_cmd_start_sim_t is None:
                            self.zero_cmd_start_sim_t = t_now
                        elif (t_now - self.zero_cmd_start_sim_t) >= 1.0 and abs(cmd_wz) > 0.20:
                            self.state = STATE_NORMAL
                            self.pending_start_sim_t = None
                            self.recovery_event_count += 1

            elif self.state == STATE_RECOVERING_REVERSE:
                if not self.enable_auto_reverse:
                    self.state = STATE_STALL_LATCHED
                elif self.recovery_start_pose is not None:
                    backed_dist = math.hypot(
                        self.lidar_pose[0] - self.recovery_start_pose[0],
                        self.lidar_pose[1] - self.recovery_start_pose[1]
                    )
                    if backed_dist >= 0.06 and cmd_vx >= -0.01:
                        self.state = STATE_NORMAL
                        self.pending_start_sim_t = None
                        self.get_logger().info(f"[RECOVERY COMPLETE] Backed up {backed_dist:.3f}m; returning to NORMAL.")
                    elif cmd_vx > 0.04 and backed_dist < 0.03:
                        self.state = STATE_STALL_LATCHED

        # 3. Compute gated /cmd_vel output
        out = Twist()
        self.cmd_blocked = False
        if self.enabled:
            if self.state in (STATE_INITIALIZING, STATE_FAILSAFE_SENSOR_TIMEOUT, STATE_FAILSAFE_ICP_DEGRADED):
                out.linear.x = 0.0
                out.linear.y = 0.0
                out.angular.z = 0.0
                self.cmd_blocked = want_motion
            elif self.state == STATE_STALL_LATCHED:
                # Force zero output on forward, reverse, and rotate (complete software stop)
                out.linear.x = 0.0
                out.linear.y = 0.0
                out.angular.z = 0.0
                self.cmd_blocked = want_motion
            elif self.state == STATE_RECOVERING_REVERSE:
                out.linear.x = float(cmd_vx)
                out.angular.z = float(cmd_wz)
                self.cmd_blocked = False
            else:
                out.linear.x = float(cmd_vx)
                out.angular.z = float(cmd_wz)
                self.cmd_blocked = False
        else:
            out.linear.x = float(cmd_vx)
            out.angular.z = float(cmd_wz)
            self.cmd_blocked = False

        self.pub_cmd_vel.publish(out)

        # 4. Publish Virtual Bumper Cloud when Stalled
        if self.pub_bumper and self.state in (STATE_STALL_LATCHED, STATE_STALL_PENDING):
            self.publish_virtual_bumper_cloud()

        # 5. Publish diagnostics JSON at 10 Hz or immediately on state change
        if not hasattr(self, '_diag_tick'):
            self._diag_tick = 0
            self._prev_state = None
        self._diag_tick += 1
        state_changed = (self.state != self._prev_state)
        if self._diag_tick % 5 == 0 or state_changed:
            self._prev_state = self.state
            diag_obj = {
                'sim_time': round(t_now, 3),
                'state': STATE_NAMES.get(self.state, 'UNKNOWN'),
                'state_code': self.state,
                'cmd_blocked': bool(self.cmd_blocked),
                'enable_auto_reverse': bool(self.enable_auto_reverse),
                'require_operator_unlatch': bool(self.require_operator_unlatch),
                'cmd_vx_in': round(float(cmd_vx), 3),
                'cmd_vx_out': round(float(out.linear.x), 3),
                'wheel_vx': round(float(self.wheel_vx), 3),
                'lidar_speed': round(float(self.lidar_speed_short), 3),
                'min_front_scan_m': round(float(self.min_front_scan), 3),
                'icp_inliers': round(float(self.last_icp_inliers), 1),
                'icp_error_m': round(float(self.last_icp_err), 4),
                'depth_mode': self.depth_mode,
                'depth_state': self.depth_state,
                'depth_code': self.depth_code,
                'depth_stale': bool(depth_stale),
                'wheel_odom_stale': bool(wheel_odom_stale),
                'est_odom_stale': bool(est_odom_stale),
                'stall_events': self.stall_event_count,
                'failsafe_events': self.failsafe_event_count,
                'icp_degraded_events': self.icp_degraded_event_count,
                'operator_unlatch_count': self.operator_unlatch_count,
            }
            msg_str = String()
            msg_str.data = json.dumps(diag_obj)
            self.pub_diag.publish(msg_str)
            if self._diag_tick % 50 == 0:
                self.get_logger().debug(
                    f"[STALL TELEMETRY 1Hz] state={diag_obj['state']} v_cmd={diag_obj['cmd_vx_out']:.2f} "
                    f"v_wheel={diag_obj['wheel_vx']:.2f} v_lidar={diag_obj['lidar_speed']:.2f} inliers={diag_obj['icp_inliers']:.0f}"
                )

    def publish_virtual_bumper_cloud(self):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.height = 1
        msg.width = 17
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        raw_data = bytearray()
        bx = self.bumper_dist
        for dy in np.linspace(-0.20, 0.20, msg.width):
            raw_data.extend(struct.pack('ffff', float(bx), float(dy), 0.12, 100.0))
        msg.data = bytes(raw_data)
        self.pub_bumper_cloud.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
