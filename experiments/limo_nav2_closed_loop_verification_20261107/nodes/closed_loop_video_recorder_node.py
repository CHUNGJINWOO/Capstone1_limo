#!/usr/bin/env -S python3 -s
"""
closed_loop_video_recorder_node.py
Synchronized Dual-Panel Simulation Video Recorder (1920x960 @ 15 FPS, H.264 MP4)
supporting both Multi-Estimator Evaluation AND Closed-Loop Nav2 + Stall Detector HUD:
- Left Panel (1280x960): Real Gazebo Fortress overhead camera stream (/lounge/overhead)
  overlaid with:
  - World-frame Ground Truth (Green) and Estimated trajectories (B0, F1b, F3b, F3b_imp, F4, F5, F5_imp)
  - Nav2 Global Path (/plan, bright cyan dotted/solid curve) & Local Path (/local_plan, yellow curve)
  - Nav2 Start & Goal markers in world frame
  - Live header banner showing SimTime, Active Estimator, Nav2 Action State, Stall Detector State,
    cmd_vx_raw vs gated cmd_vx_out, and minimum LiDAR range.
- Right Panel (640x960):
  - Top (640x460): 2D Metric Trajectory & Path Canvas (Odom/World frame)
  - Bottom (640x500): Live Quantitative Telemetry HUD (GT vs Active Estimator error, ATE RMSE,
    Yaw error, Stall Detector status badge, F4 ICP compute latency, and Nav2 status).
"""
import os
import json
import math
import time
import signal
import subprocess
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


ESTIMATOR_SPECS = [
    ('GT',       '/ground_truth/odom',                (0, 255, 0),     3),  # Bright Green
    ('B0',       '/odom',                             (60, 60, 255),   2),  # Red
    ('F1b',      '/odometry/ekf_tuned',               (255, 220, 0),   2),  # Cyan
    ('F3b',      '/odometry/comp_slip_gate',          (0, 180, 255),   1),  # Orange-Yellow
    ('F3b_imp',  '/odometry/comp_slip_gate_improved', (0, 255, 255),   2),  # Bright Yellow
    ('F4',       '/odometry/lidar_odom',              (255, 130, 40),  2),  # Blue
    ('F5',       '/odometry/lidar_imu_wheel_ekf',     (200, 80, 255),  2),  # Purple-Magenta
    ('F5_imp',   '/odometry/f5_improved',             (255, 255, 255), 2),  # White-Cyan highlight
]


class ClosedLoopVideoRecorderNode(Node):
    def __init__(self):
        super().__init__(
            'closed_loop_video_recorder_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('output_mp4', '/tmp/closed_loop.mp4')
        self.declare_parameter('scenario_title', 'Closed-Loop Nav2 Verification')
        self.declare_parameter('active_estimator', 'ALL')
        self.declare_parameter('goal_x', float('nan'))
        self.declare_parameter('goal_y', float('nan'))
        self.declare_parameter('fps', 15.0)

        self.output_mp4 = self.get_parameter('output_mp4').value
        self.scenario_title = self.get_parameter('scenario_title').value.replace('_', ' ')
        self.active_estimator = self.get_parameter('active_estimator').value
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.fps = float(self.get_parameter('fps').value)

        os.makedirs(os.path.dirname(os.path.abspath(self.output_mp4)), exist_ok=True)
        self.raw_mp4 = self.output_mp4.replace('.mp4', '_raw_mp4v.mp4')

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(self.raw_mp4, fourcc, self.fps, (1920, 960))
        self._finalized = False

        self.latest_overhead_bgr = None
        self.init_world_pose = None
        self.curr_world_gt = None

        self.est_data = {}
        for name, topic, color, thickness in ESTIMATOR_SPECS:
            self.est_data[name] = {
                'topic': topic,
                'color': color,
                'thickness': thickness,
                'pose': None,
                'traj': [],
                'sq_err_sum': 0.0,
                'err_count': 0,
                'pos_err': 0.0,
                'yaw_err_deg': 0.0,
                'msg_count': 0,
            }
            self.create_subscription(
                Odometry,
                topic,
                lambda msg, n=name: self.on_est_odom(n, msg),
                20
            )

        self.sub_world_gt = self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_world_gt, 20)
        self.sub_cam = self.create_subscription(Image, '/lounge/overhead', self.on_overhead_image, 5)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_scan, 5)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.sub_plan = self.create_subscription(NavPath, '/plan', self.on_global_plan, 5)
        self.sub_lplan = self.create_subscription(NavPath, '/local_plan', self.on_local_plan, 5)
        self.sub_stall = self.create_subscription(String, '/stall_detector/status_json', self.on_stall_status, 10)
        self.sub_nav_status = self.create_subscription(String, '/nav2_monitor/status_json', self.on_nav_status, 10)
        self.sub_f4_lat = self.create_subscription(Float32MultiArray, '/diagnostics/f4_latency', self.on_f4_lat, 10)

        self.min_scan_range = 10.0
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.global_plan_pts = []
        self.local_plan_pts = []
        self.stall_info = {
            'state_name': 'NORMAL',
            'cmd_blocked': False,
            'cmd_vx_raw': 0.0,
            'cmd_vx_out': 0.0,
            'wheel_vx': 0.0,
            'lidar_speed': 0.0,
        }
        self.nav_info = {
            'action_state': 'IDLE',
            'goal_dist_gt_m': 0.0,
            'goal_dist_est_m': 0.0,
            'physical_contact': False,
        }
        self.f4_icp_ms = 0.0
        self.frame_count = 0
        self.last_sim_t = 0.0

        self.timer = self.create_timer(1.0 / self.fps, self.record_frame)

    def on_world_gt(self, msg: Odometry):
        xw = msg.pose.pose.position.x
        yw = msg.pose.pose.position.y
        yaww = quat_to_yaw(msg.pose.pose.orientation)
        self.curr_world_gt = (xw, yw, yaww)
        if self.init_world_pose is None:
            self.init_world_pose = (xw, yw, yaww)

    def on_est_odom(self, name: str, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t_sec > 0.0:
            self.last_sim_t = max(self.last_sim_t, t_sec)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        d = self.est_data[name]
        d['pose'] = (x, y, yaw, vx, wz, t_sec)
        d['msg_count'] += 1
        if len(d['traj']) == 0 or math.hypot(x - d['traj'][-1][0], y - d['traj'][-1][1]) >= 0.02:
            d['traj'].append((x, y))

        if name != 'GT' and self.est_data['GT']['pose'] is not None:
            gtx, gty, gtyaw, _, _, _ = self.est_data['GT']['pose']
            pe = math.hypot(x - gtx, y - gty)
            ye = abs(math.degrees(wrap_angle(yaw - gtyaw)))
            d['pos_err'] = pe
            d['yaw_err_deg'] = ye
            d['sq_err_sum'] += pe * pe
            d['err_count'] += 1

    def on_overhead_image(self, msg: Image):
        if msg.encoding == 'rgb8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            self.latest_overhead_bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            self.latest_overhead_bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()

    def on_scan(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        valid = ranges[np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)]
        if len(valid) > 0:
            self.min_scan_range = float(np.min(valid))

    def on_cmd_vel(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z

    def on_global_plan(self, msg: NavPath):
        self.global_plan_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses[::2]]

    def on_local_plan(self, msg: NavPath):
        self.local_plan_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_stall_status(self, msg: String):
        try:
            self.stall_info = json.loads(msg.data)
        except Exception:
            pass

    def on_nav_status(self, msg: String):
        try:
            self.nav_info = json.loads(msg.data)
        except Exception:
            pass

    def on_f4_lat(self, msg: Float32MultiArray):
        if len(msg.data) >= 1:
            self.f4_icp_ms = float(msg.data[0])

    def odom_to_world(self, x_odom: float, y_odom: float):
        if self.init_world_pose is None:
            return x_odom, y_odom
        x0, y0, yaw0 = self.init_world_pose
        c = math.cos(yaw0)
        s = math.sin(yaw0)
        return x0 + c * x_odom - s * y_odom, y0 + s * x_odom + c * y_odom

    def world_to_overhead_px(self, xw: float, yw: float):
        scale = 104.36
        u = int(640.0 + xw * scale)
        v = int(480.0 - yw * scale)
        return u, v

    def record_frame(self):
        if self.latest_overhead_bgr is None and self.est_data['GT']['pose'] is None:
            return

        canvas = np.zeros((960, 1920, 3), dtype=np.uint8)
        if self.latest_overhead_bgr is not None:
            left = self.latest_overhead_bgr.copy()
            if left.shape[0] != 960 or left.shape[1] != 1280:
                left = cv2.resize(left, (1280, 960))
        else:
            left = np.full((960, 1280, 3), 35, dtype=np.uint8)

        # Draw Nav2 Global Plan (in map/world frame)
        if len(self.global_plan_pts) >= 2:
            g_px = []
            for (wx, wy) in self.global_plan_pts:
                u, v = self.world_to_overhead_px(wx, wy)
                if 0 <= u < 1280 and 0 <= v < 960:
                    g_px.append([u, v])
            if len(g_px) >= 2:
                cv2.polylines(left, [np.array(g_px, dtype=np.int32)], False, (255, 255, 0), 2, cv2.LINE_AA)

        # Draw Goal marker if specified
        if math.isfinite(self.goal_x) and math.isfinite(self.goal_y):
            gu, gv = self.world_to_overhead_px(self.goal_x, self.goal_y)
            if 0 <= gu < 1280 and 0 <= gv < 960:
                cv2.circle(left, (gu, gv), 12, (0, 215, 255), 2, cv2.LINE_AA)
                cv2.drawMarker(left, (gu, gv), (0, 215, 255), cv2.MARKER_CROSS, 16, 2)
                cv2.putText(left, "GOAL", (gu + 14, gv + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 215, 255), 2)

        # Draw trajectories
        draw_list = ['B0', 'F1b', 'F3b', 'F3b_imp', 'F4', 'F5', 'F5_imp', 'GT']
        if self.active_estimator not in ('ALL', '') and self.active_estimator in self.est_data:
            draw_list = [self.active_estimator, 'GT']

        for name in draw_list:
            d = self.est_data[name]
            pts = d['traj']
            if len(pts) >= 2:
                px_pts = []
                for (ox, oy) in pts:
                    xw, yw = self.odom_to_world(ox, oy)
                    u, v = self.world_to_overhead_px(xw, yw)
                    if 0 <= u < 1280 and 0 <= v < 960:
                        px_pts.append([u, v])
                if len(px_pts) >= 2:
                    thick = 3 if name in ('GT', self.active_estimator) else d['thickness']
                    cv2.polylines(left, [np.array(px_pts, dtype=np.int32)], False, d['color'], thick, cv2.LINE_AA)
            if d['pose'] is not None:
                ox, oy, _, _, _, _ = d['pose']
                xw, yw = self.odom_to_world(ox, oy)
                u, v = self.world_to_overhead_px(xw, yw)
                if 0 <= u < 1280 and 0 <= v < 960:
                    cv2.circle(left, (u, v), 6 if name in ('GT', self.active_estimator) else 4, d['color'], -1)

        # Header overlay on left panel
        cv2.rectangle(left, (10, 10), (1050, 118), (18, 18, 24), -1)
        cv2.putText(left, f"GAZEBO FORTRESS LIVE | {self.scenario_title}",
                    (22, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        st_name = self.stall_info.get('state_name', 'NORMAL')
        act_state = self.nav_info.get('action_state', 'OPEN_LOOP')
        cv2.putText(left, f"SimTime: {self.last_sim_t:.2f}s | ActiveEst: {self.active_estimator} | Nav2: {act_state} | StallDet: {st_name}",
                    (22, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (170, 240, 255), 1)
        cv2.putText(left, f"cmd_raw: {self.stall_info.get('cmd_vx_raw', self.cmd_vx):.2f}m/s -> cmd_out: {self.cmd_vx:.2f}m/s | MinScan: {self.min_scan_range:.2f}m | F4 ICP: {self.f4_icp_ms:.1f}ms",
                    (22, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 220, 200), 1)
        canvas[:, :1280] = left

        # Right Panel (640x960)
        right = np.full((960, 640, 3), 24, dtype=np.uint8)
        plot_box = np.full((430, 600, 3), 36, dtype=np.uint8)
        all_x, all_y = [0.0], [0.0]
        for name in draw_list:
            for (ox, oy) in self.est_data[name]['traj']:
                all_x.append(ox)
                all_y.append(oy)
        min_x, max_x = min(all_x) - 0.5, max(all_x) + 0.5
        min_y, max_y = min(all_y) - 0.5, max(all_y) + 0.5
        span = max(max_x - min_x, max_y - min_y, 2.5)
        cx_m = 0.5 * (min_x + max_x)
        cy_m = 0.5 * (min_y + max_y)

        def odom_to_plot(ox, oy):
            px = int(300 + ((ox - cx_m) / span) * 520)
            py = int(215 - ((oy - cy_m) / span) * 360)
            return px, py

        for g in np.arange(-10.0, 10.5, 0.5):
            gx0, gy0 = odom_to_plot(g, cy_m - span)
            gx1, gy1 = odom_to_plot(g, cy_m + span)
            if 0 <= gx0 < 600:
                cv2.line(plot_box, (gx0, 0), (gx1, 429), (50, 50, 50), 1)
            hx0, hy0 = odom_to_plot(cx_m - span, g)
            hx1, hy1 = odom_to_plot(cx_m + span, g)
            if 0 <= hy0 < 430:
                cv2.line(plot_box, (0, hy0), (599, hy1), (50, 50, 50), 1)

        for name in draw_list:
            d = self.est_data[name]
            if len(d['traj']) >= 2:
                p_arr = np.array([odom_to_plot(ox, oy) for (ox, oy) in d['traj']], dtype=np.int32)
                cv2.polylines(plot_box, [p_arr], False, d['color'], 2, cv2.LINE_AA)

        cv2.putText(plot_box, "2D Odom Frame Trajectory (GT=Green vs Estimators)",
                    (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (235, 235, 235), 1)
        right[15:445, 20:620] = plot_box

        # Bottom Telemetry Table
        y0 = 470
        cv2.putText(right, "LIVE STATE ESTIMATION & CLOSED-LOOP SAFETY HUD",
                    (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)
        y0 += 28
        cv2.putText(right, "Track     PosErr(m)   RMSE(m)   YawErr(deg)   Msgs",
                    (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 210, 240), 1)
        y0 += 10
        cv2.line(right, (20, y0), (620, y0), (90, 90, 90), 1)

        for name, _, color, _ in ESTIMATOR_SPECS:
            y0 += 26
            d = self.est_data[name]
            if name == 'GT':
                if d['pose'] is not None:
                    gx, gy, gyaw, gvx, _, _ = d['pose']
                    txt = f"GT(Ref)   x={gx:+5.2f}m  y={gy:+5.2f}m  yaw={math.degrees(gyaw):+6.1f}d  v={gvx:+4.2f}"
                else:
                    txt = "GT(Ref)   waiting..."
                cv2.putText(right, txt, (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1)
            else:
                rmse = math.sqrt(d['sq_err_sum'] / max(1, d['err_count']))
                marker = ">>" if name == self.active_estimator else "  "
                txt = f"{marker}{name:<7s} {d['pos_err']:7.3f}     {rmse:7.3f}     {d['yaw_err_deg']:7.2f}      {d['msg_count']:4d}"
                cv2.putText(right, txt, (20, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 2 if name == self.active_estimator else 1)

        # Stall Detector & Nav2 Status Badge
        y0 += 36
        blocked = bool(self.stall_info.get('cmd_blocked', False))
        badge_col = (0, 0, 210) if blocked or st_name != 'NORMAL' else (30, 130, 40)
        cv2.rectangle(right, (20, y0), (620, y0 + 82), badge_col, -1)
        cv2.putText(right, f"STALL DETECTOR: {st_name} | CMD_BLOCKED={blocked}",
                    (32, y0 + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 255), 2)
        cv2.putText(right, f"Wheel vx={self.stall_info.get('wheel_vx', 0.0):+.2f}m/s vs LiDAR speed={self.stall_info.get('lidar_speed', 0.0):.2f}m/s",
                    (32, y0 + 54), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (240, 240, 240), 1)
        cv2.putText(right, f"Nav2 Action: {act_state} | Contact={self.nav_info.get('physical_contact', False)}",
                    (32, y0 + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 235, 180), 1)

        canvas[:, 1280:] = right
        self.writer.write(canvas)
        self.frame_count += 1

    def finalize_video(self):
        if self._finalized:
            return
        self._finalized = True
        try:
            self.writer.release()
        except Exception:
            pass
        if os.path.exists(self.raw_mp4) and os.path.getsize(self.raw_mp4) > 1024:
            cmd = [
                'ffmpeg', '-y', '-loglevel', 'error',
                '-i', self.raw_mp4,
                '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
                '-preset', 'fast', '-crf', '22',
                self.output_mp4
            ]
            try:
                subprocess.run(cmd, check=False)
            except FileNotFoundError:
                os.replace(self.raw_mp4, self.output_mp4)
                return
            if os.path.exists(self.output_mp4) and os.path.getsize(self.output_mp4) > 1024:
                try:
                    os.remove(self.raw_mp4)
                except OSError:
                    pass
            elif os.path.exists(self.raw_mp4):
                os.replace(self.raw_mp4, self.output_mp4)


def main():
    rclpy.init()
    node = ClosedLoopVideoRecorderNode()

    def sig_handler(signum, frame):
        node.finalize_video()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.finalize_video()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
