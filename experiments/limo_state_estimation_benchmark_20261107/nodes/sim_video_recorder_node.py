#!/usr/bin/env -S python3 -s
"""
sim_video_recorder_node.py
Synchronized Dual-Panel Simulation Video Recorder (1920x960 @ 15 FPS, H.264 MP4).
- Left Panel (1280x960): Real Gazebo Fortress overhead camera stream (/lounge/overhead)
  with world-frame Ground Truth & Estimated trajectories overlaid.
- Right Panel (640x960):
  - Top (640x480): Metric 2D trajectory comparison canvas (GT vs B0, F1a, F1b, F2, F3a, F3b, F4, F5).
  - Bottom (640x480): Live quantitative telemetry HUD (instantaneous position error, yaw error,
    running RMSE, velocities, IMU yaw rate, LiDAR min range, and Slip/Stall Gate badge).
"""
import os
import json
import math
import time
import subprocess
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, Imu, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


ESTIMATOR_SPECS = [
    ('GT',  '/ground_truth/odom',          (0, 255, 0),     3),  # Bright Green
    ('B0',  '/odom',                       (60, 60, 255),   2),  # Red
    ('F1a', '/odometry/ekf_legacy',        (160, 160, 160), 1),  # Gray
    ('F1b', '/odometry/ekf_tuned',         (255, 220, 0),   2),  # Cyan
    ('F2',  '/odometry/ukf_tuned',         (0, 165, 255),   2),  # Orange
    ('F3a', '/odometry/comp_filter',       (255, 100, 180), 1),  # Purple-Pink
    ('F3b', '/odometry/comp_slip_gate',    (0, 255, 255),   2),  # Yellow
    ('F4',  '/odometry/lidar_odom',        (255, 120, 40),  2),  # Blue
    ('F5',  '/odometry/lidar_imu_wheel_ekf', (255, 0, 255), 2),  # Magenta
]


class SimVideoRecorderNode(Node):
    def __init__(self):
        super().__init__(
            'sim_video_recorder_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('output_mp4', '/tmp/smoke_test.mp4')
        self.declare_parameter('scenario_title', 'S1_Normal_MultiTurn_SmokeTest')
        self.declare_parameter('fps', 15.0)

        self.output_mp4 = self.get_parameter('output_mp4').value
        self.scenario_title = self.get_parameter('scenario_title').value.replace('_', ' ')
        self.fps = float(self.get_parameter('fps').value)

        os.makedirs(os.path.dirname(os.path.abspath(self.output_mp4)), exist_ok=True)
        self.raw_mp4 = self.output_mp4.replace('.mp4', '_raw_mp4v.mp4')

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(self.raw_mp4, fourcc, self.fps, (1920, 960))

        self.latest_overhead_bgr = None
        self.latest_overhead_stamp = 0.0
        self.init_world_pose = None  # (xw0, yw0, yaww0)

        # Store latest state & trajectory per estimator: name -> dict
        self.est_data = {}
        for name, topic, color, thickness in ESTIMATOR_SPECS:
            self.est_data[name] = {
                'topic': topic,
                'color': color,
                'thickness': thickness,
                'pose': None,       # (x, y, yaw, vx, wz, t_sec)
                'traj': [],         # list of (x, y)
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

        self.sub_world_gt = self.create_subscription(
            Odometry, '/ground_truth/world_odom', self.on_world_gt, 20
        )
        self.sub_cam = self.create_subscription(
            Image, '/lounge/overhead', self.on_overhead_image, 5
        )
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.on_scan, 5
        )
        self.sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self.on_cmd_vel, 10
        )
        self.sub_slip = self.create_subscription(
            Float32MultiArray, '/diagnostics/slip_gate', self.on_slip_diag, 10
        )

        self.min_scan_range = 10.0
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.slip_active = False
        self.gated_vx = 0.0
        self.frame_count = 0
        self.first_sim_t = None
        self.last_sim_t = 0.0

        # Record frame at fixed wall timer matching FPS (15 Hz)
        self.timer = self.create_timer(1.0 / self.fps, self.record_frame)
        self.get_logger().info(f"SimVideoRecorderNode recording to {self.output_mp4} at {self.fps} FPS.")

    def on_world_gt(self, msg: Odometry):
        if self.init_world_pose is None:
            xw = msg.pose.pose.position.x
            yw = msg.pose.pose.position.y
            yaww = quat_to_yaw(msg.pose.pose.orientation)
            self.init_world_pose = (xw, yw, yaww)

    def on_est_odom(self, name: str, msg: Odometry):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.first_sim_t is None and t_sec > 0.0:
            self.first_sim_t = t_sec
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
            self.latest_overhead_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        elif msg.encoding == 'bgr8':
            self.latest_overhead_bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3)).copy()
            self.latest_overhead_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_scan(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        valid = ranges[np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)]
        if len(valid) > 0:
            self.min_scan_range = float(np.min(valid))

    def on_cmd_vel(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_wz = msg.angular.z

    def on_slip_diag(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.gated_vx = float(msg.data[1])
            self.slip_active = bool(msg.data[3] > 0.5)

    def odom_to_world(self, x_odom: float, y_odom: float):
        if self.init_world_pose is None:
            return x_odom, y_odom
        x0, y0, yaw0 = self.init_world_pose
        c = math.cos(yaw0)
        s = math.sin(yaw0)
        xw = x0 + c * x_odom - s * y_odom
        yw = y0 + s * x_odom + c * y_odom
        return xw, yw

    def world_to_overhead_px(self, xw: float, yw: float):
        """
        Overhead camera at (0, 0, 8.5), rpy=(0, pi/2, pi/2), width=1280, height=960, hfov=1.25 rad.
        fx = 640 / tan(0.625) = 887.06 px -> scale at Z=0 is 887.06 / 8.5 = 104.36 px/m.
        With rpy=(0, pi/2, pi/2), camera image u (+right) maps to +X_world, and v (+down) maps to -Y_world.
        """
        scale = 104.36
        u = int(640.0 + xw * scale)
        v = int(480.0 - yw * scale)
        return u, v

    def record_frame(self):
        if self.latest_overhead_bgr is None and self.est_data['GT']['pose'] is None:
            return

        canvas = np.zeros((960, 1920, 3), dtype=np.uint8)

        # 1. Left Panel (1280x960): Gazebo Overhead Camera + World Trajectories
        if self.latest_overhead_bgr is not None:
            left = self.latest_overhead_bgr.copy()
            if left.shape[0] != 960 or left.shape[1] != 1280:
                left = cv2.resize(left, (1280, 960))
        else:
            left = np.full((960, 1280, 3), 35, dtype=np.uint8)
            cv2.putText(left, "Waiting for /lounge/overhead camera...", (320, 480),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2)

        # Draw world-projected trajectories onto overhead image
        for name in ['B0', 'F1b', 'F2', 'F3b', 'F4', 'F5', 'GT']:
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
                    cv2.polylines(left, [np.array(px_pts, dtype=np.int32)], False, d['color'], d['thickness'], cv2.LINE_AA)

            if d['pose'] is not None:
                ox, oy, _, _, _, _ = d['pose']
                xw, yw = self.odom_to_world(ox, oy)
                u, v = self.world_to_overhead_px(xw, yw)
                if 0 <= u < 1280 and 0 <= v < 960:
                    cv2.circle(left, (u, v), 5 if name == 'GT' else 4, d['color'], -1)

        # Header banner on left panel
        cv2.rectangle(left, (10, 10), (830, 88), (20, 20, 20), -1)
        cv2.putText(left, f"GAZEBO FORTRESS LIVE OVERHEAD VIEW | {self.scenario_title}",
                    (22, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2)
        cv2.putText(left, f"SimTime: {self.last_sim_t:.2f}s | Frame: {self.frame_count} | MinLiDAR: {self.min_scan_range:.2f}m | cmd_vx: {self.cmd_vx:.2f}m/s",
                    (22, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (180, 235, 255), 1)

        if self.slip_active:
            cv2.rectangle(left, (850, 10), (1265, 88), (0, 0, 180), -1)
            cv2.putText(left, "WHEEL SLIP / STALL GATED!", (868, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.80, (255, 255, 255), 2)

        canvas[:, :1280] = left

        # 2. Right Top Panel (640x480): Metric Odom-Frame Trajectory Plot
        plot_box = np.full((480, 640, 3), 24, dtype=np.uint8)
        cv2.putText(plot_box, "ODOM-FRAME TRAJECTORY COMPARISON (METERS)", (18, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (230, 230, 230), 2)

        # Determine dynamic bounding box across trajectories
        all_x = [0.0, 2.0, -0.5]
        all_y = [0.0, 1.5, -1.5]
        for name, _, _, _ in ESTIMATOR_SPECS:
            for (px, py) in self.est_data[name]['traj']:
                all_x.append(px)
                all_y.append(py)
        min_x, max_x = min(all_x) - 0.4, max(all_x) + 0.4
        min_y, max_y = min(all_y) - 0.4, max(all_y) + 0.4
        span = max(max_x - min_x, max_y - min_y, 2.0)
        cx_m = 0.5 * (min_x + max_x)
        cy_m = 0.5 * (min_y + max_y)
        px_per_m = 380.0 / span

        def odom_to_plot_px(ox, oy):
            pu = int(320 + (ox - cx_m) * px_per_m)
            pv = int(255 - (oy - cy_m) * px_per_m)
            return pu, pv

        # Draw 0.5m grid lines
        for gx in np.arange(math.floor(cx_m - span), math.ceil(cx_m + span) + 0.5, 0.5):
            u0, _ = odom_to_plot_px(gx, cy_m)
            if 20 <= u0 <= 620:
                cv2.line(plot_box, (u0, 45), (u0, 455), (45, 45, 45), 1)
        for gy in np.arange(math.floor(cy_m - span), math.ceil(cy_m + span) + 0.5, 0.5):
            _, v0 = odom_to_plot_px(cx_m, gy)
            if 45 <= v0 <= 455:
                cv2.line(plot_box, (20, v0), (620, v0), (45, 45, 45), 1)

        # Origin crosshair
        ou, ov = odom_to_plot_px(0.0, 0.0)
        if 20 <= ou <= 620 and 45 <= ov <= 455:
            cv2.drawMarker(plot_box, (ou, ov), (120, 120, 120), cv2.MARKER_CROSS, 14, 1)

        for name, _, color, thickness in ESTIMATOR_SPECS:
            pts = self.est_data[name]['traj']
            if len(pts) >= 2:
                p_arr = []
                for (ox, oy) in pts:
                    pu, pv = odom_to_plot_px(ox, oy)
                    pu = max(20, min(620, pu))
                    pv = max(45, min(455, pv))
                    p_arr.append([pu, pv])
                cv2.polylines(plot_box, [np.array(p_arr, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)
            if self.est_data[name]['pose'] is not None:
                ox, oy, _, _, _, _ = self.est_data[name]['pose']
                pu, pv = odom_to_plot_px(ox, oy)
                if 20 <= pu <= 620 and 45 <= pv <= 455:
                    cv2.circle(plot_box, (pu, pv), 5 if name == 'GT' else 3, color, -1)

        canvas[:480, 1280:] = plot_box

        # 3. Right Bottom Panel (640x480): Quantitative Telemetry HUD Table
        hud = np.full((480, 640, 3), 16, dtype=np.uint8)
        cv2.line(hud, (0, 0), (640, 0), (100, 100, 100), 2)
        cv2.putText(hud, "STATE ESTIMATION ACCURACY vs GROUND TRUTH", (18, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 2)

        headers = "Method    X(m)   Y(m)  Yaw(d) | PosErr(m) RMSE(m) YawErr"
        cv2.putText(hud, headers, (14, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.47, (180, 200, 230), 1)
        cv2.line(hud, (14, 70), (626, 70), (80, 80, 80), 1)

        row_y = 98
        for name, _, color, _ in ESTIMATOR_SPECS:
            d = self.est_data[name]
            cv2.rectangle(hud, (14, row_y - 12), (28, row_y + 2), color, -1)
            if d['pose'] is None:
                line = f"{name:<6}   [WAITING FOR MESSAGES...]"
                cv2.putText(hud, line, (34, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (130, 130, 130), 1)
            else:
                ox, oy, oyaw, _, _, _ = d['pose']
                yaw_d = math.degrees(oyaw)
                if name == 'GT':
                    line = f"{name:<5}  {ox:6.2f} {oy:6.2f} {yaw_d:6.1f} |   [REFERENCE GROUND TRUTH]"
                else:
                    rmse = math.sqrt(d['sq_err_sum'] / max(1, d['err_count']))
                    line = f"{name:<5}  {ox:6.2f} {oy:6.2f} {yaw_d:6.1f} |   {d['pos_err']:6.3f}   {rmse:6.3f}  {d['yaw_err_deg']:5.1f}d"
                cv2.putText(hud, line, (34, row_y), cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1)
            row_y += 34

        # Footer sensor summary
        cv2.line(hud, (14, 412), (626, 412), (80, 80, 80), 1)
        gt_v = self.est_data['GT']['pose'][3] if self.est_data['GT']['pose'] else 0.0
        b0_v = self.est_data['B0']['pose'][3] if self.est_data['B0']['pose'] else 0.0
        cv2.putText(hud, f"v_GT: {gt_v:.2f} m/s | v_Wheel(B0): {b0_v:.2f} m/s | v_SlipGated(F3b): {self.gated_vx:.2f} m/s",
                    (16, 438), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (210, 230, 210), 1)
        cv2.putText(hud, "Note: GT is strictly isolated from all estimators (used for evaluation only).",
                    (16, 464), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (150, 150, 150), 1)

        canvas[480:, 1280:] = hud

        self.writer.write(canvas)
        self.frame_count += 1

    def finalize_video(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None

        # Transcode to H.264 baseline MP4 for universal playback
        if os.path.exists(self.raw_mp4) and os.path.getsize(self.raw_mp4) > 0:
            cmd = [
                'gst-launch-1.0', '-q',
                'filesrc', f'location={self.raw_mp4}',
                '!', 'qtdemux', '!', 'avdec_mpeg4', '!', 'videoconvert',
                '!', 'x264enc', 'bitrate=4500', 'speed-preset=superfast',
                '!', 'video/x-h264,profile=baseline',
                '!', 'mp4mux', '!', 'filesink', f'location={self.output_mp4}'
            ]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if res.returncode == 0 and os.path.exists(self.output_mp4) and os.path.getsize(self.output_mp4) > 1000:
                os.remove(self.raw_mp4)
            else:
                os.replace(self.raw_mp4, self.output_mp4)

        # Verify playability with cv2.VideoCapture and extract sample keyframes
        cap = cv2.VideoCapture(self.output_mp4)
        decoded_frames = 0
        non_black_frames = 0
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps_read = float(cap.get(cv2.CAP_PROP_FPS))

        sample_frame_path = self.output_mp4.replace('.mp4', '_mid_frame.png')
        target_mid = max(1, self.frame_count // 2)

        while True:
            ret, frm = cap.read()
            if not ret:
                break
            decoded_frames += 1
            if np.mean(frm) > 5.0:
                non_black_frames += 1
            if decoded_frames == target_mid:
                cv2.imwrite(sample_frame_path, frm)
        cap.release()

        meta = {
            'output_mp4': self.output_mp4,
            'file_size_bytes': os.path.getsize(self.output_mp4) if os.path.exists(self.output_mp4) else 0,
            'written_frames': self.frame_count,
            'decoded_frames_verified': decoded_frames,
            'non_black_frames_verified': non_black_frames,
            'resolution': f'{width}x{height}',
            'fps': fps_read,
            'sim_time_start': self.first_sim_t,
            'sim_time_end': self.last_sim_t,
            'sim_duration_s': (self.last_sim_t - (self.first_sim_t or 0.0)) if self.first_sim_t else 0.0,
            'sample_keyframe_png': sample_frame_path if os.path.exists(sample_frame_path) else None,
            'playable': bool(decoded_frames > 0 and non_black_frames > 0 and width == 1920 and height == 960),
        }
        meta_path = self.output_mp4.replace('.mp4', '_video_verification.json')
        with open(meta_path, 'w') as f:
            json.dump(meta, f, indent=2)
        self.get_logger().info(f"Video verification saved to {meta_path}: {json.dumps(meta)}")


def main():
    rclpy.init()
    node = SimVideoRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize_video()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
