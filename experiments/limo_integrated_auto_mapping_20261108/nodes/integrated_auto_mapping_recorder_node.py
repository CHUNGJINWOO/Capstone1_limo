#!/usr/bin/env -S python3 -s
"""
integrated_auto_mapping_recorder_node.py
Synchronized Dual-Panel Video Recorder (1920x960 @ 15 FPS, H.264 MP4)
for LIMO Pro Master Integrated Auto-Mapping Verification:
- Left Panel (1280x960): Gazebo Fortress overhead camera stream (/lounge/overhead)
  overlaid with:
  - Ground Truth trajectory (Green)
  - Selected Estimator trajectory (F4 Cyan / F5 Magenta)
  - Nav2 Global Path (/plan)
  - Dynamic HUD overlay: SimTime, Estimator, SLAM cells, Depth Obstacle points, Stall Gate State.
- Right Panel (640x960):
  - Top (640x460): 2D Metric Trajectory Map & Costmap Coverage Canvas
  - Bottom (640x500): Real-Time Quantitative Integration Telemetry HUD
"""
import os
import json
import math
import time
import subprocess
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class IntegratedAutoMappingRecorderNode(Node):
    def __init__(self):
        super().__init__(
            'integrated_auto_mapping_recorder_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('output_mp4', '/tmp/auto_mapping.mp4')
        self.declare_parameter('active_estimator', 'f4')
        self.declare_parameter('fps', 15.0)

        self.output_mp4 = self.get_parameter('output_mp4').value
        self.active_estimator = self.get_parameter('active_estimator').value
        self.fps = float(self.get_parameter('fps').value)

        os.makedirs(os.path.dirname(os.path.abspath(self.output_mp4)), exist_ok=True)
        self.raw_mp4 = self.output_mp4.replace('.mp4', '_raw.mp4')

        self.frame_count = 0
        self.sync_jsonl_path = self.output_mp4.replace('.mp4', '_frame_sync.jsonl')
        self.sync_file = open(self.sync_jsonl_path, 'w')

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(self.raw_mp4, fourcc, self.fps, (1920, 960))
        self._finalized = False

        self.latest_overhead = None
        self.gt_traj = []
        self.est_traj = []
        self.latest_gt = None
        self.latest_est = None

        self.latest_plan = []
        self.map_cell_count = 0
        self.depth_obstacle_count = 0
        self.stall_state = 'NORMAL'
        self.cmd_raw_vx = 0.0
        self.cmd_out_vx = 0.0
        self.distance_traveled = 0.0
        self.ate_rmse = 0.0
        self.ate_errors = []

        qos_cam = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                  reliability=ReliabilityPolicy.RELIABLE)

        self.sub_cam = self.create_subscription(Image, '/lounge/overhead', self.on_cam, qos_cam)
        self.sub_gt = self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_gt, 10)

        est_topic = '/odometry/lidar_odom' if self.active_estimator == 'f4' else '/odometry/f5_improved'
        self.sub_est = self.create_subscription(Odometry, est_topic, self.on_est, 10)

        self.sub_plan = self.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.on_map, 5)
        self.sub_depth = self.create_subscription(PointCloud2, '/depth_obstacles/points', self.on_depth, qos_reliable)
        self.sub_cmd_raw = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd_raw, 10)
        self.sub_cmd_out = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_out, 10)
        self.sub_stall = self.create_subscription(String, '/stall_detector/status_json', self.on_stall, 10)

        # 15 FPS frame assembly timer
        self.timer = self.create_timer(1.0 / self.fps, self.render_frame)

    def on_cam(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            img = arr.reshape((msg.height, msg.width, 3))
            self.latest_overhead = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        except Exception:
            pass

    def on_gt(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        self.latest_gt = (px, py, quat_to_yaw(msg.pose.pose.orientation))
        if not self.gt_traj or math.hypot(px - self.gt_traj[-1][0], py - self.gt_traj[-1][1]) > 0.02:
            if self.gt_traj:
                self.distance_traveled += math.hypot(px - self.gt_traj[-1][0], py - self.gt_traj[-1][1])
            self.gt_traj.append((px, py))

    def on_est(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        self.latest_est = (px, py, quat_to_yaw(msg.pose.pose.orientation))
        if not self.est_traj or math.hypot(px - self.est_traj[-1][0], py - self.est_traj[-1][1]) > 0.02:
            self.est_traj.append((px, py))

        if self.latest_gt is not None:
            err = math.hypot(px - self.latest_gt[0], py - self.latest_gt[1])
            self.ate_errors.append(err)
            self.ate_rmse = math.sqrt(float(np.mean(np.square(self.ate_errors))))

    def on_plan(self, msg: NavPath):
        self.latest_plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_map(self, msg: OccupancyGrid):
        data = np.array(msg.data, dtype=np.int8)
        self.map_cell_count = int(np.sum(data > 0))

    def on_depth(self, msg: PointCloud2):
        self.depth_obstacle_count = msg.width * msg.height

    def on_cmd_raw(self, msg: Twist):
        self.cmd_raw_vx = msg.linear.x

    def on_cmd_out(self, msg: Twist):
        self.cmd_out_vx = msg.linear.x

    def on_stall(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.stall_state = d.get('state', 'UNKNOWN')
        except Exception:
            pass

    def world_to_overhead_pixel(self, x: float, y: float):
        # Overhead camera calibration for lounge_evaluation_world
        # Bounds: X in [-3.5, 6.0], Y in [-4.5, 4.5] mapped to 1280x960
        u = int(1280.0 * (x - (-3.5)) / (6.0 - (-3.5)))
        v = int(960.0 * (1.0 - (y - (-4.5)) / (4.5 - (-4.5))))
        return (max(0, min(1279, u)), max(0, min(959, v)))

    def world_to_metric_canvas_pixel(self, x: float, y: float):
        # Right-top metric panel: X in [-2.5, 5.0], Y in [-3.5, 3.5] mapped to 640x460
        u = int(640.0 * (x - (-2.5)) / (5.0 - (-2.5)))
        v = int(460.0 * (1.0 - (y - (-3.5)) / (3.5 - (-3.5))))
        return (max(0, min(639, u)), max(0, min(459, v)))

    def render_frame(self):
        if self._finalized or self.latest_overhead is None:
            return

        left_img = cv2.resize(self.latest_overhead, (1280, 960))

        # Overlay GT Path (Green)
        if len(self.gt_traj) > 1:
            pts = np.array([self.world_to_overhead_pixel(x, y) for x, y in self.gt_traj], dtype=np.int32)
            cv2.polylines(left_img, [pts], isClosed=False, color=(0, 255, 0), thickness=3)

        # Overlay Estimator Path (Cyan for F4, Magenta for F5)
        est_color = (255, 255, 0) if self.active_estimator == 'f4' else (255, 0, 255)
        if len(self.est_traj) > 1:
            pts = np.array([self.world_to_overhead_pixel(x, y) for x, y in self.est_traj], dtype=np.int32)
            cv2.polylines(left_img, [pts], isClosed=False, color=est_color, thickness=2)

        # Overlay Nav2 Plan (Bright Yellow)
        if len(self.latest_plan) > 1:
            pts = np.array([self.world_to_overhead_pixel(x, y) for x, y in self.latest_plan], dtype=np.int32)
            cv2.polylines(left_img, [pts], isClosed=False, color=(0, 255, 255), thickness=2)

        # Current Robot Position Marker
        if self.latest_gt:
            cx, cy = self.world_to_overhead_pixel(self.latest_gt[0], self.latest_gt[1])
            cv2.circle(left_img, (cx, cy), 8, (0, 0, 255), -1)
            cv2.circle(left_img, (cx, cy), 12, (255, 255, 255), 2)

        # Header banner on left panel
        cv2.rectangle(left_img, (0, 0), (1280, 50), (20, 20, 20), -1)
        sim_t = self.get_clock().now().nanoseconds * 1e-9
        header_text = (
            f"LIMO PRO INTEGRATED AUTO-MAPPING  |  SimTime: {sim_t:6.2f}s  |  "
            f"Estimator: {self.active_estimator.upper()}  |  SLAM Cells: {self.map_cell_count}  |  "
            f"Stall Gate: {self.stall_state}"
        )
        cv2.putText(left_img, header_text, (20, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 255, 255), 2)

        # Right Panel Construction (640x960)
        right_img = np.zeros((960, 640, 3), dtype=np.uint8)
        right_img[:] = (30, 30, 30)

        # Top Metric Canvas (640x460)
        metric_canvas = np.zeros((460, 640, 3), dtype=np.uint8)
        metric_canvas[:] = (20, 24, 28)
        # Grid lines
        for gx in np.arange(-2.0, 5.0, 1.0):
            p0 = self.world_to_metric_canvas_pixel(gx, -3.5)
            p1 = self.world_to_metric_canvas_pixel(gx, 3.5)
            cv2.line(metric_canvas, p0, p1, (45, 50, 55), 1)
        for gy in np.arange(-3.0, 3.5, 1.0):
            p0 = self.world_to_metric_canvas_pixel(-2.5, gy)
            p1 = self.world_to_metric_canvas_pixel(5.0, gy)
            cv2.line(metric_canvas, p0, p1, (45, 50, 55), 1)

        cv2.putText(metric_canvas, "2D TOP-DOWN METRIC TRAJECTORY", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)

        # Plot GT in metric canvas
        if len(self.gt_traj) > 1:
            pts = np.array([self.world_to_metric_canvas_pixel(x, y) for x, y in self.gt_traj], dtype=np.int32)
            cv2.polylines(metric_canvas, [pts], isClosed=False, color=(0, 255, 0), thickness=2)
        # Plot Est in metric canvas
        if len(self.est_traj) > 1:
            pts = np.array([self.world_to_metric_canvas_pixel(x, y) for x, y in self.est_traj], dtype=np.int32)
            cv2.polylines(metric_canvas, [pts], isClosed=False, color=est_color, thickness=2)

        right_img[0:460, 0:640] = metric_canvas
        cv2.line(right_img, (0, 460), (640, 460), (80, 80, 80), 2)

        # Bottom Telemetry HUD (640x500)
        hud_y = 490
        cv2.putText(right_img, "SYSTEM TELEMETRY & INTEGRATION HUD", (20, hud_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)

        lines = [
            ("Selected Estimator:", f"{self.active_estimator.upper()} ({'LiDAR ICP Scan Matcher' if self.active_estimator=='f4' else 'LiDAR+IMU EKF'})", (255, 255, 255)),
            ("SLAM Mapping:", "SLAM Toolbox (Online Asynchronous)", (200, 255, 200)),
            ("Occupied Map Cells:", f"{self.map_cell_count} cells", (0, 255, 0) if self.map_cell_count > 50 else (180, 180, 180)),
            ("Autonomous Explorer:", "explore_lite (Frontier Pursuit)", (255, 255, 200)),
            ("3D Obstacle Avoidance:", f"Orbbec DaBai ({self.depth_obstacle_count} pts)", (0, 255, 255)),
            ("Stall Safety Gate:", f"{self.stall_state} (AUTO-REV LOCKED)", (0, 255, 0) if self.stall_state=='NORMAL' else (0, 0, 255)),
            ("Command Routing:", f"Raw={self.cmd_raw_vx:+.2f} m/s -> Out={self.cmd_out_vx:+.2f} m/s", (255, 255, 255)),
            ("Traveled Distance:", f"{self.distance_traveled:6.3f} m", (255, 255, 100)),
            ("ATE RMSE to GT:", f"{self.ate_rmse:6.4f} m", (100, 255, 100) if self.ate_rmse < 0.15 else (0, 165, 255)),
            ("Hardware Safety Dist.:", "Supervisory Gate vs Physical E-Stop", (180, 180, 180))
        ]

        for i, (label, val, col) in enumerate(lines):
            y_pos = hud_y + 35 + i * 38
            cv2.putText(right_img, label, (25, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (180, 180, 180), 1)
            cv2.putText(right_img, val, (250, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.52, col, 2)

        # Composite Side-by-Side: 1280 + 640 = 1920 x 960
        combined = np.hstack([left_img, right_img])
        self.writer.write(combined)
        self.frame_count += 1

        try:
            sync_entry = {
                'frame_index': self.frame_count,
                'sim_time': round(sim_t, 3),
                'wall_time': round(time.time(), 3),
                'stall_state': self.stall_state,
                'cmd_vx_out': round(float(self.cmd_out_vx), 3),
                'cmd_vx_raw': round(float(self.cmd_raw_vx), 3),
                'gt_pose': [round(float(v), 3) for v in self.latest_gt] if self.latest_gt else None,
                'est_pose': [round(float(v), 3) for v in self.latest_est] if self.latest_est else None,
                'depth_points': self.depth_obstacle_count,
                'map_cells': self.map_cell_count
            }
            self.sync_file.write(json.dumps(sync_entry) + '\n')
            self.sync_file.flush()
        except Exception:
            pass

    def finalize_video(self):
        if self._finalized:
            return
        self._finalized = True
        self.writer.release()
        try:
            if hasattr(self, 'sync_file') and not self.sync_file.closed:
                self.sync_file.close()
                self.get_logger().info(f"Frame synchronization metadata saved at: {self.sync_jsonl_path}")
        except Exception:
            pass
        import shutil
        if self.raw_mp4 != self.output_mp4 and os.path.exists(self.raw_mp4):
            if shutil.which('ffmpeg'):
                ffmpeg_cmd = [
                    'ffmpeg', '-y',
                    '-i', self.raw_mp4,
                    '-c:v', 'libx264',
                    '-pix_fmt', 'yuv420p',
                    '-preset', 'fast',
                    '-crf', '22',
                    self.output_mp4
                ]
                try:
                    subprocess.run(ffmpeg_cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    self.get_logger().info(f"H.264 Video ready at: {self.output_mp4}")
                    if os.path.exists(self.raw_mp4):
                        os.remove(self.raw_mp4)
                except Exception as e:
                    self.get_logger().warn(f"FFmpeg transcode failed ({e}), falling back to direct MP4")
                    os.replace(self.raw_mp4, self.output_mp4)
            else:
                os.replace(self.raw_mp4, self.output_mp4)
                self.get_logger().info(f"MP4 Video ready at: {self.output_mp4}")


def main():
    rclpy.init()
    node = IntegratedAutoMappingRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize_video()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
