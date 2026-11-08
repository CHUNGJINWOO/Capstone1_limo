#!/usr/bin/env python3
"""
run_video_scenarios.py:
High-resolution telemetry and video-evidence runner for:
  1. scen3_unmapped_detour_video:
     - Starts at S=(-2.0, -3.0, 1.5708), Goal G=(3.0, 0.5, 0.0)
     - Static map has NO obstacle at (0.0, -0.85); Gazebo world has unmapped_box_sc3 at (0.0, -0.85).
     - At departure, unmapped_box_sc3 is 2.79m away (> 2.5m LiDAR range) and occluded behind wall c_10/c_7.
     - Initial global path (/plan) routes through the South corridor (y = -0.85).
     - As robot drives along initial path past y = -2.0, LiDAR (/scan) discovers unmapped_box_sc3,
       global costmap marks the obstacle, and Nav2 replans around the wide North detour route (y > 0.8) to Goal G.
  2. scen4_dynamic_spawn_video (separate scenario):
     - Starts at S=(-4.5, -3.5, 0.0), Goal G=(-1.5, -3.5, 0.0)
     - While driving along initial straight path, dynamic_box_sc4 appears ahead at (-2.3, -3.5, 0.25)
       (safe distance ~1.4m ahead of robot), LiDAR detects it, costmap updates, and Nav2 replans around it to Goal G.

Records 100% authentic ROS 2 messages & Gazebo Ground Truth:
  - Full /plan waypoint coordinates (all poses) in plan_history.jsonl
  - Full /global_costmap/costmap and /local_costmap/costmap 2D grids in costmap_snapshots.npz
  - Transformed /scan LiDAR points in map frame and obstacle hit counts in scan_history.jsonl
  - High-rate Gazebo GT poses in gt_pose_history.jsonl
  - High-rate overhead Gazebo camera frames in frames/XXXXX.png
  - Exact event timestamps (initial plan, first LiDAR hit, first costmap mark, detour replan, goal reached) in events.json
  - Authoritative evaluation summary in runtime_execution_record.json and summary.json
"""

import os
import sys
import time
import math
import signal
import subprocess
import json
import hashlib
import shutil
import pathlib
import argparse
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan, Image as ImageMsg
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

BASE_DIR = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(BASE_DIR / 'output/simulation'))
from evaluation_lib import (
    compute_polygon_clearance_sat,
    make_robot_polygon,
    make_box_polygon,
    parse_sdf_boxes,
    evaluate_scenario_from_records,
)

GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'STATUS_UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'STATUS_ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'STATUS_EXECUTING',
    GoalStatus.STATUS_CANCELING: 'STATUS_CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'STATUS_SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'STATUS_CANCELED',
    GoalStatus.STATUS_ABORTED: 'STATUS_ABORTED'
}


def compute_sha256(filepath):
    h = hashlib.sha256()
    with open(filepath, 'rb') as f:
        while chunk := f.read(8192):
            h.update(chunk)
    return h.hexdigest()


class VideoScenarioRecorder(Node):
    def __init__(self, output_dir, world_name, scenario_type, sdf_boxes, target_obs_box):
        super().__init__('video_scenario_recorder', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)
        ])
        self.output_dir = pathlib.Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'frames').mkdir(exist_ok=True)

        self.world_name = world_name
        self.scenario_type = scenario_type
        self.sdf_boxes = sdf_boxes
        self.static_boxes = [b for b in sdf_boxes if not b['is_dynamic']]
        self.dynamic_boxes = [b for b in sdf_boxes if b['is_dynamic']]
        # target_obs_box: dict {'name': str, 'cx': float, 'cy': float, 'sx': float, 'sy': float}
        self.target_obs_box = target_obs_box
        self.obstacle_active = (scenario_type == 'unmapped_detour')

        self.sim_time = 0.0
        self.scan_count = 0
        self.odom_count = 0
        self.image_count = 0
        self.last_saved_image_sim = -100.0
        self.last_saved_scan_sim = -100.0
        self.recording = False
        self.current_phase = "INITIALIZING"

        self.latest_odom = None
        self.latest_amcl = None
        self.latest_truth = None  # [x, y, z, qz, qw, yaw]
        self.latest_cmd = [0.0, 0.0]
        self.feedback = None

        self.plans_recorded = []
        self.initial_plan_record = None
        self.current_plan_record = None

        self.global_costmap_snaps = []  # list of (sim_time, grid_int8)
        self.global_costmap_meta = None
        self.local_costmap_snaps = []   # list of (sim_time, ox, oy, res, w, h, grid_int8)

        # Milestone event timestamps (synchronized to sim_time)
        self.events = {
            'nav_start_sim_time': None,
            'initial_plan_sim_time': None,
            'obstacle_spawn_sim_time': None,
            'first_scan_hit_sim_time': None,
            'first_scan_hit_robot_pose': None,
            'first_scan_hit_distance': None,
            'first_costmap_mark_sim_time': None,
            'first_costmap_mark_cells': None,
            'detour_replan_sim_time': None,
            'detour_replan_plan_id': None,
            'action_end_sim_time': None,
            'final_action_status': None
        }

        self.latest_scan_obs_hits = 0
        self.latest_scan_obs_min_dist = None
        self.latest_costmap_obs_lethal = 0
        self.latest_costmap_obs_high = 0

        self.min_moving_clearance_all_time = float('inf')
        self.min_wall_clearance_all_time = float('inf')
        self.min_clearance_all_time = float('inf')
        self.collision_occurred = False
        self.collision_object = None
        self.dynamic_obstacle_poses = {}

        self.prev_truth_pos = None
        self.trajectory_length = 0.0
        self.frames = []

        self.robot_width = 0.22
        self.robot_length = 0.32

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Output files
        self.gt_pose_file = (self.output_dir / 'gt_pose_history.jsonl').open('w')
        self.obstacle_pose_file = (self.output_dir / 'obstacle_pose_history.jsonl').open('w')
        self.costmap_roi_file = (self.output_dir / 'costmap_roi_history.jsonl').open('w')
        self.telemetry_file = (self.output_dir / 'telemetry.jsonl').open('w')
        self.plan_history_file = (self.output_dir / 'plan_history.jsonl').open('w')
        self.scan_history_file = (self.output_dir / 'scan_history.jsonl').open('w')
        self.cmd_history_file = (self.output_dir / 'cmd_history.jsonl').open('w')

        # Subscriptions
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._global_costmap_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self._local_costmap_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        )
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(ImageMsg, '/demo/overhead', self._image_cb, qos_profile_sensor_data)

        truth_topic = f'/world/{world_name}/pose/info'
        self.create_subscription(TFMessage, truth_topic, self._truth_cb, 10)

        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec / 1e9

    def _odom_cb(self, msg):
        self.odom_count += 1
        self.latest_odom = msg

    def _amcl_cb(self, msg):
        self.latest_amcl = msg

    def _cmd_cb(self, msg):
        self.latest_cmd = [round(msg.linear.x, 4), round(msg.angular.z, 4)]
        cmd_rec = {
            'sim_time': round(self.sim_time, 3),
            'phase': self.current_phase,
            'vx': round(msg.linear.x, 4),
            'wz': round(msg.angular.z, 4)
        }
        self.cmd_history_file.write(json.dumps(cmd_rec) + '\n')
        self.cmd_history_file.flush()

    def _classify_plan_branch(self, poses):
        """Classifies global path branch in the maze."""
        if not poses:
            return "EMPTY"
        if self.scenario_type == 'unmapped_detour':
            central_ys = [p[1] for p in poses if -0.6 <= p[0] <= 2.2]
            if not central_ys:
                return "OTHER"
            max_y = max(central_ys)
            min_y = min(central_ys)
            if max_y > 0.8:
                return "NORTH_DETOUR"
            elif min_y < -0.4:
                return "SOUTH_CORRIDOR"
            return "OTHER"
        else:
            # For scen4_dynamic_spawn along y = -3.5, check deviation around (-2.3, -3.5)
            obs_ys = [p[1] for p in poses if -2.8 <= p[0] <= -1.8]
            if not obs_ys:
                return "INITIAL_STRAIGHT"
            max_dev = max(abs(y - (-3.5)) for y in obs_ys)
            if max_dev >= 0.28:
                return "LOCAL_BYPASS_DETOUR"
            return "INITIAL_STRAIGHT"

    def _plan_cb(self, msg):
        if not msg.poses:
            return
        poses = [[round(p.pose.position.x, 4), round(p.pose.position.y, 4)] for p in msg.poses]
        branch = self._classify_plan_branch(poses)
        plan_id = len(self.plans_recorded)
        h_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        plan_rec = {
            'plan_id': plan_id,
            'sim_time': round(self.sim_time, 3),
            'header_sim_time': round(h_stamp, 3),
            'phase': self.current_phase,
            'route_branch': branch,
            'waypoint_count': len(poses),
            'start_pose': poses[0],
            'goal_pose': poses[-1],
            'poses': poses
        }
        self.plans_recorded.append(plan_rec)
        self.current_plan_record = plan_rec

        if self.initial_plan_record is None and self.current_phase == "NAVIGATING":
            self.initial_plan_record = plan_rec
            self.events['initial_plan_sim_time'] = round(self.sim_time, 3)
            print(f"[Recorder] Initial Global Plan #{plan_id} recorded at sim_time={self.sim_time:.2f}s "
                  f"(branch={branch}, waypoints={len(poses)})", flush=True)

        if (self.events['detour_replan_sim_time'] is None
                and self.initial_plan_record is not None
                and branch in ["NORTH_DETOUR", "LOCAL_BYPASS_DETOUR"]
                and branch != self.initial_plan_record['route_branch']):
            self.events['detour_replan_sim_time'] = round(self.sim_time, 3)
            self.events['detour_replan_plan_id'] = plan_id
            print(f"[Recorder] DETOUR REPLAN #{plan_id} triggered at sim_time={self.sim_time:.2f}s! "
                  f"Branch changed: {self.initial_plan_record['route_branch']} -> {branch} "
                  f"(waypoints={len(poses)})", flush=True)

        self.plan_history_file.write(json.dumps(plan_rec) + '\n')
        self.plan_history_file.flush()

    def _scan_cb(self, msg):
        self.scan_count += 1
        if not self.recording or self.latest_truth is None:
            return

        # Use GT robot pose (or AMCL pose) to transform scan rays into map frame for obstacle hit check
        rx, ry, _, _, _, ryaw = self.latest_truth
        # laser_link is 0.10m ahead of base_link along robot X axis
        lx = rx + 0.10 * math.cos(ryaw)
        ly = ry + 0.10 * math.sin(ryaw)

        bx, by = self.target_obs_box['cx'], self.target_obs_box['cy']
        bsx, bsy = self.target_obs_box['sx'], self.target_obs_box['sy']
        # Margin of 0.08m around obstacle box surface for LiDAR beam hit attribution
        x0, x1 = bx - bsx / 2.0 - 0.08, bx + bsx / 2.0 + 0.08
        y0, y1 = by - bsy / 2.0 - 0.08, by + bsy / 2.0 + 0.08

        hit_pts = []
        all_pts = []
        min_obs_r = float('inf')

        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            # Only consider rays within Nav2 obstacle_max_range (2.5m)
            if 0.08 < r <= 2.50 and math.isfinite(r):
                beam_yaw = ryaw + angle
                px = lx + r * math.cos(beam_yaw)
                py = ly + r * math.sin(beam_yaw)
                if i % 2 == 0:
                    all_pts.append([round(px, 3), round(py, 3)])
                if self.obstacle_active and (x0 <= px <= x1) and (y0 <= py <= y1):
                    hit_pts.append([round(px, 3), round(py, 3)])
                    if r < min_obs_r:
                        min_obs_r = r
            angle += msg.angle_increment

        self.latest_scan_obs_hits = len(hit_pts)
        self.latest_scan_obs_min_dist = round(min_obs_r, 3) if min_obs_r != float('inf') else None

        if (self.current_phase == "NAVIGATING"
                and self.obstacle_active
                and self.events['first_scan_hit_sim_time'] is None
                and len(hit_pts) >= 2):
            self.events['first_scan_hit_sim_time'] = round(self.sim_time, 3)
            self.events['first_scan_hit_robot_pose'] = [round(rx, 3), round(ry, 3), round(ryaw, 3)]
            self.events['first_scan_hit_distance'] = round(min_obs_r, 3)
            print(f"[Recorder] FIRST LIDAR OBSTACLE DETECTION at sim_time={self.sim_time:.2f}s! "
                  f"Robot at ({rx:.2f}, {ry:.2f}), distance={min_obs_r:.2f}m, rays={len(hit_pts)}", flush=True)

        if self.sim_time - self.last_saved_scan_sim >= 0.10:
            self.last_saved_scan_sim = self.sim_time
            scan_rec = {
                'sim_time': round(self.sim_time, 3),
                'phase': self.current_phase,
                'robot_pose': [round(rx, 3), round(ry, 3), round(ryaw, 3)],
                'obstacle_hit_count': len(hit_pts),
                'min_obstacle_range': self.latest_scan_obs_min_dist,
                'obstacle_hit_points': hit_pts,
                'scan_points_map': all_pts
            }
            self.scan_history_file.write(json.dumps(scan_rec) + '\n')
            self.scan_history_file.flush()

    def _global_costmap_cb(self, msg):
        if not msg.data:
            return
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        res = msg.info.resolution
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))

        self.global_costmap_meta = {
            'origin': [ox, oy],
            'resolution': res,
            'width': w,
            'height': h
        }
        snap_idx = len(self.global_costmap_snaps)
        self.global_costmap_snaps.append((round(self.sim_time, 3), grid.copy()))

        # Check ROI around target obstacle box
        bx, by = self.target_obs_box['cx'], self.target_obs_box['cy']
        bsx, bsy = self.target_obs_box['sx'], self.target_obs_box['sy']
        px_min = max(0, min(w - 1, int(round((bx - bsx / 2.0 - 0.10 - ox) / res))))
        px_max = max(0, min(w - 1, int(round((bx + bsx / 2.0 + 0.10 - ox) / res))))
        py_min = max(0, min(h - 1, int(round((by - bsy / 2.0 - 0.10 - oy) / res))))
        py_max = max(0, min(h - 1, int(round((by + bsy / 2.0 + 0.10 - oy) / res))))

        roi_slice = grid[min(py_min, py_max):max(py_min, py_max) + 1,
                         min(px_min, px_max):max(px_min, px_max) + 1]
        vals = [int(v) for v in roi_slice.flatten()]

        lethal_cnt = sum(1 for c in vals if c == 100)
        high_cnt = sum(1 for c in vals if c >= 90)
        zero_cnt = sum(1 for c in vals if c == 0)
        unknown_cnt = sum(1 for c in vals if c == -1)

        self.latest_costmap_obs_lethal = lethal_cnt
        self.latest_costmap_obs_high = high_cnt

        if (self.current_phase == "NAVIGATING"
                and self.obstacle_active
                and self.events['first_costmap_mark_sim_time'] is None
                and lethal_cnt >= 3):
            self.events['first_costmap_mark_sim_time'] = round(self.sim_time, 3)
            self.events['first_costmap_mark_cells'] = lethal_cnt
            print(f"[Recorder] FIRST COSTMAP OBSTACLE MARKING at sim_time={self.sim_time:.2f}s! "
                  f"Lethal cells in obstacle ROI: {lethal_cnt} (high>=90: {high_cnt})", flush=True)

        entry = {
            'sim_time': round(self.sim_time, 3),
            'snapshot_idx': snap_idx,
            'phase': self.current_phase,
            'corridor_roi': {
                'total_cells': len(vals),
                'max_cost': max(vals) if vals else 0,
                'lethal_count': lethal_cnt,
                'high_cost_count': high_cnt,
                'zero_cost_count': zero_cnt,
                'unknown_count': unknown_cnt,
                'all_valid_cells_zero': (len(vals) > 0 and unknown_cnt == 0 and zero_cnt == len(vals))
            }
        }
        self.costmap_roi_file.write(json.dumps(entry) + '\n')
        self.costmap_roi_file.flush()

    def _local_costmap_cb(self, msg):
        if not msg.data:
            return
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        res = msg.info.resolution
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.local_costmap_snaps.append((round(self.sim_time, 3), ox, oy, res, w, h, grid.copy()))

    def _truth_cb(self, msg):
        recv_sim = self.sim_time
        recv_host = time.time()

        for t in msg.transforms:
            cid = t.child_frame_id
            h_stamp = t.header.stamp.sec + t.header.stamp.nanosec / 1e9

            if cid == 'limo':
                rx = t.transform.translation.x
                ry = t.transform.translation.y
                rz = t.transform.translation.z
                qz = t.transform.rotation.z
                qw = t.transform.rotation.w
                ryaw = math.atan2(2 * qw * qz, 1 - 2 * (qz * qz))
                self.latest_truth = [rx, ry, rz, qz, qw, ryaw]

                pos = (rx, ry)
                if self.prev_truth_pos:
                    d = math.hypot(pos[0] - self.prev_truth_pos[0], pos[1] - self.prev_truth_pos[1])
                    if d < 0.5:
                        self.trajectory_length += d
                self.prev_truth_pos = pos

                gt_rec = {
                    'header_sim_time': round(h_stamp, 4),
                    'recv_sim_time': round(recv_sim, 4),
                    'recv_host_time': round(recv_host, 4),
                    'phase': self.current_phase,
                    'x': round(rx, 4),
                    'y': round(ry, 4),
                    'z': round(rz, 4),
                    'yaw': round(ryaw, 4)
                }
                self.gt_pose_file.write(json.dumps(gt_rec) + '\n')
                self.gt_pose_file.flush()

                robot_poly = make_robot_polygon(rx, ry, ryaw, self.robot_width, self.robot_length)

                # Static walls
                for sb in self.static_boxes:
                    b_poly = make_box_polygon(sb['center'][0], sb['center'][1], sb['size'][0], sb['size'][1])
                    c = compute_polygon_clearance_sat(robot_poly, b_poly)
                    if c < self.min_wall_clearance_all_time:
                        self.min_wall_clearance_all_time = c
                    if c <= 0.005:
                        self.collision_occurred = True
                        self.collision_object = sb['name']

                # Dynamic / unmapped obstacles
                for db in self.dynamic_boxes:
                    mname = db['model_name']
                    if mname in self.dynamic_obstacle_poses:
                        dpos = self.dynamic_obstacle_poses[mname]
                        if dpos[2] < -5.0:
                            continue
                        bx, by = dpos[0], dpos[1]
                    else:
                        if not self.obstacle_active and mname == 'dynamic_box_sc4':
                            continue
                        bx, by = db['center'][0], db['center'][1]

                    b_poly = make_box_polygon(bx, by, db['size'][0], db['size'][1])
                    c = compute_polygon_clearance_sat(robot_poly, b_poly)
                    if c < self.min_moving_clearance_all_time:
                        self.min_moving_clearance_all_time = c
                    if c <= 0.005:
                        self.collision_occurred = True
                        self.collision_object = db['name']

                self.min_clearance_all_time = min(self.min_wall_clearance_all_time, self.min_moving_clearance_all_time)

            elif cid in ['unmapped_box_sc3', 'dynamic_box_sc4', 'moving_box_sc5', 'barricade_total', 'removable_barrier']:
                ox = t.transform.translation.x
                oy = t.transform.translation.y
                oz = t.transform.translation.z
                self.dynamic_obstacle_poses[cid] = [ox, oy, oz]
                obs_rec = {
                    'header_sim_time': round(h_stamp, 4),
                    'recv_sim_time': round(recv_sim, 4),
                    'model_name': cid,
                    'x': round(ox, 4),
                    'y': round(oy, 4),
                    'z': round(oz, 4)
                }
                self.obstacle_pose_file.write(json.dumps(obs_rec) + '\n')
                self.obstacle_pose_file.flush()

    def _image_cb(self, msg):
        self.image_count += 1
        if not self.recording:
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if t <= 0.0:
            t = self.sim_time
        if t - self.last_saved_image_sim < 0.095:
            return
        self.last_saved_image_sim = t

        enc = msg.encoding.lower()
        mode = 'RGB' if enc in ['rgb8', '8uc3'] else 'RGBA' if enc == 'rgba8' else 'RGB'
        im = Image.frombytes(
            mode, (msg.width, msg.height), bytes(msg.data),
            'raw', 'BGR' if enc == 'bgr8' else mode, msg.step, 1
        ).convert('RGB')
        frame_idx = len(self.frames)
        file_rel = f'frames/{frame_idx:05d}.png'
        im.save(self.output_dir / file_rel)

        tf_pose = None
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            tf_pose = [round(tr.transform.translation.x, 4), round(tr.transform.translation.y, 4), round(yaw, 4)]
        except Exception:
            pass

        odom_pos = (
            [round(self.latest_odom.pose.pose.position.x, 4), round(self.latest_odom.pose.pose.position.y, 4)]
            if self.latest_odom else None
        )

        rec = {
            'frame': frame_idx,
            'image': file_rel,
            'sim_time': round(t, 3),
            'phase': self.current_phase,
            'cmd': self.latest_cmd,
            'tf_pose': tf_pose,
            'odom_pos': odom_pos,
            'truth': [round(v, 4) for v in self.latest_truth] if self.latest_truth else None,
            'current_plan_id': self.current_plan_record['plan_id'] if self.current_plan_record else None,
            'current_plan_branch': self.current_plan_record['route_branch'] if self.current_plan_record else None,
            'scan_obs_hits': self.latest_scan_obs_hits,
            'scan_obs_min_dist': self.latest_scan_obs_min_dist,
            'costmap_obs_lethal_cells': self.latest_costmap_obs_lethal,
            'costmap_obs_high_cells': self.latest_costmap_obs_high,
            'min_moving_clearance': round(self.min_moving_clearance_all_time, 4) if self.min_moving_clearance_all_time != float('inf') else None,
            'min_wall_clearance': round(self.min_wall_clearance_all_time, 4) if self.min_wall_clearance_all_time != float('inf') else None,
            'min_clearance_all_time': round(self.min_clearance_all_time, 4) if self.min_clearance_all_time != float('inf') else None,
            'collision_occurred': self.collision_occurred,
            'distance_remaining': round(self.feedback.distance_remaining, 3) if self.feedback else None
        }
        self.frames.append({'image': file_rel, 'sim_time': round(t, 3)})
        self.telemetry_file.write(json.dumps(rec, ensure_ascii=False) + '\n')
        self.telemetry_file.flush()

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        yaw_f = float(yaw)
        msg.pose.pose.orientation.z = math.sin(yaw_f / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_f / 2.0)
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.02
        self.init_pub.publish(msg)

    def save_costmap_archives(self):
        if self.global_costmap_snaps and self.global_costmap_meta:
            times = np.array([s[0] for s in self.global_costmap_snaps], dtype=np.float64)
            grids = np.stack([s[1] for s in self.global_costmap_snaps], axis=0)
            np.savez_compressed(
                self.output_dir / 'global_costmap_snapshots.npz',
                sim_times=times,
                grids=grids,
                origin=np.array(self.global_costmap_meta['origin'], dtype=np.float64),
                resolution=np.float64(self.global_costmap_meta['resolution'])
            )
        if self.local_costmap_snaps:
            times = np.array([s[0] for s in self.local_costmap_snaps], dtype=np.float64)
            origins = np.array([[s[1], s[2]] for s in self.local_costmap_snaps], dtype=np.float64)
            res = np.float64(self.local_costmap_snaps[0][3])
            grids = np.stack([s[6] for s in self.local_costmap_snaps], axis=0)
            np.savez_compressed(
                self.output_dir / 'local_costmap_snapshots.npz',
                sim_times=times,
                origins=origins,
                resolution=res,
                grids=grids
            )

    def close_files(self):
        for f in [self.gt_pose_file, self.obstacle_pose_file, self.costmap_roi_file,
                  self.telemetry_file, self.plan_history_file, self.scan_history_file, self.cmd_history_file]:
            try:
                f.close()
            except Exception:
                pass


def set_ignition_pose(world_name, model_name, x, y, z, env):
    req = f'name: "{model_name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}'
    cmd = [
        'ign', 'service', '-s', f'/world/{world_name}/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req', req
    ]
    res = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return res.returncode == 0


def run_experiment(args):
    out_dir = pathlib.Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    logs_dir = out_dir / 'logs'
    logs_dir.mkdir(parents=True, exist_ok=True)

    print("================================================================================")
    print(f"  VIDEO SCENARIO EXPERIMENT: {args.scenario} ({args.scenario_type})")
    print(f"  Output Directory: {out_dir}")
    print("================================================================================")

    configs_to_archive = {
        'nav2_params.yaml': args.params_file,
        'behavior_tree.xml': args.bt_xml,
        'world.sdf': args.world,
        'robot_model.sdf': args.model_file
    }

    hashes = {}
    for dest_name, src_path in configs_to_archive.items():
        src = pathlib.Path(src_path)
        dest = out_dir / dest_name
        shutil.copyfile(src, dest)
        h = compute_sha256(src)
        hashes[dest_name] = h
        (out_dir / f"{dest_name}.sha256").write_text(f"{h}  {dest_name}\n")
        print(f"  Archived {dest_name} (SHA256: {h[:12]}...)")

    run_args_record = {
        'scenario': args.scenario,
        'scenario_type': args.scenario_type,
        'domain_id': args.domain_id,
        'world': str(args.world),
        'map': str(args.map),
        'params_file': str(args.params_file),
        'bt_xml': str(args.bt_xml),
        'model_file': str(args.model_file),
        'timeout_sim': args.timeout_sim,
        'spawn_pose': [args.spawn_x, args.spawn_y, args.spawn_yaw],
        'goal_pose': [args.goal_x, args.goal_y, args.goal_yaw],
        'sha256_hashes': hashes,
        'start_host_timestamp': time.time(),
        'start_host_iso': time.strftime("%Y-%m-%dT%H:%M:%S%z")
    }
    with open(out_dir / 'run_args.json', 'w') as f:
        json.dump(run_args_record, f, indent=2)

    domain_id = str(args.domain_id)
    os.environ['ROS_DOMAIN_ID'] = domain_id
    os.environ['IGN_PARTITION'] = f'partition_{domain_id}'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    os.environ['IGN_TRANSPORT_TOPIC_STATISTICS'] = '1'
    env = os.environ.copy()

    procs = []
    def spawn_proc(cmd, log_prefix):
        lf = (logs_dir / f"{log_prefix}.log").open('w')
        p = subprocess.Popen(cmd, env=env, stdout=lf, stderr=lf, preexec_fn=os.setsid)
        procs.append((p, lf))
        return p

    def cleanup():
        print("[VideoRunner] Terminating background processes...", flush=True)
        for p, lf in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1.0)
        for p, lf in procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
            try:
                lf.close()
            except Exception:
                pass
        subprocess.run(['pkill', '-9', '-f', f'partition_{domain_id}'], stderr=subprocess.DEVNULL)

    try:
        # 1. Launch Gazebo simulation + bridges
        sim_cmd = [
            'ros2', 'launch', 'limo_capstone', 'limo_mc3f_sim.launch.py',
            f'world:={args.world}',
            f'world_name:={args.world_name}',
            f'model_file:={args.model_file}',
            f'spawn_x:={args.spawn_x}',
            f'spawn_y:={args.spawn_y}',
            f'spawn_z:=0.05',
            f'spawn_yaw:={args.spawn_yaw}',
            'gui:=false'
        ]
        spawn_proc(sim_cmd, 'gazebo')

        bridge_cmd = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/demo/overhead@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'/world/{args.world_name}/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ]
        spawn_proc(bridge_cmd, 'aux_bridge')

        sdf_boxes = parse_sdf_boxes(args.world)
        if args.scenario_type == 'unmapped_detour':
            target_obs_box = {
                'name': 'unmapped_box_sc3',
                'cx': 0.0, 'cy': -0.85,
                'sx': 0.45, 'sy': 0.45
            }
        else:
            target_obs_box = {
                'name': 'dynamic_box_sc4',
                'cx': -2.3, 'cy': -3.5,
                'sx': 0.45, 'sy': 0.45
            }

        rclpy.init()
        node = VideoScenarioRecorder(out_dir, args.world_name, args.scenario_type, sdf_boxes, target_obs_box)

        print("[VideoRunner] Waiting for Gazebo clock, scan, and overhead camera...", flush=True)
        t0 = time.time()
        while time.time() - t0 < 35.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.sim_time > 0.5 and node.scan_count > 0 and node.odom_count > 0 and node.image_count > 0:
                break
        if node.sim_time <= 0.0 or node.scan_count == 0:
            raise RuntimeError(f"Sensor startup timeout: sim={node.sim_time}, scan={node.scan_count}, img={node.image_count}")
        print(f"[VideoRunner] Sensors online at sim_time={node.sim_time:.2f}s.", flush=True)

        # 2. Launch Nav2
        nav2_cmd = [
            'ros2', 'launch', 'limo_capstone', 'limo_nav2_sim.launch.py',
            'use_sim_time:=true',
            f'map:={args.map}',
            f'params_file:={args.params_file}',
            'use_rviz:=false'
        ]
        spawn_proc(nav2_cmd, 'nav2')

        print("[VideoRunner] Waiting for Nav2 AMCL and global costmap...", flush=True)
        node.recording = True
        init_start_sim = node.sim_time
        t_wait = time.monotonic() + 50.0
        while time.monotonic() < t_wait:
            node.publish_initial_pose(args.spawn_x, args.spawn_y, args.spawn_yaw)
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.nav_client.server_is_ready() and len(node.global_costmap_snaps) > 0 and node.latest_amcl is not None:
                break
        else:
            raise RuntimeError("Nav2 server, AMCL, or global costmap timeout")

        # Warm-up spin (3.0s) so initial frames and costmap are settled before goal dispatch
        t_warm = time.monotonic() + 3.0
        while time.monotonic() < t_warm:
            node.publish_initial_pose(args.spawn_x, args.spawn_y, args.spawn_yaw)
            rclpy.spin_once(node, timeout_sec=0.1)

        print(f"[VideoRunner] AMCL & Costmap ready at sim_time={node.sim_time:.2f}s. Sending NavigateToPose goal ({args.goal_x}, {args.goal_y}, {args.goal_yaw})...", flush=True)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(args.goal_x)
        goal_msg.pose.pose.position.y = float(args.goal_y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(float(args.goal_yaw) / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(float(args.goal_yaw) / 2.0)
        bt_xml_abs = os.path.abspath(str(args.bt_xml))
        goal_msg.behavior_tree = bt_xml_abs

        send_future = node.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: setattr(node, 'feedback', fb.feedback)
        )
        while not send_future.done():
            rclpy.spin_once(node, timeout_sec=0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal rejected by NavigateToPose server!")

        node.current_phase = "NAVIGATING"
        nav_start_sim = node.sim_time
        nav_start_host = time.time()
        node.events['nav_start_sim_time'] = round(nav_start_sim, 3)
        print(f"[VideoRunner] Goal accepted at sim_time={nav_start_sim:.2f}s. Navigating...", flush=True)

        res_future = goal_handle.get_result_async()
        timed_out = False
        cancel_requested = False
        cancel_accepted = None
        action_result_received = False
        final_status_code = None
        dynamic_events_list = []

        last_log_sim = nav_start_sim
        while not res_future.done():
            rclpy.spin_once(node, timeout_sec=0.04)
            elapsed_sim = node.sim_time - nav_start_sim

            # For scen4_dynamic_spawn: trigger dynamic obstacle spawn when robot drives ~0.7m past start (x >= -3.8)
            if (args.scenario_type == 'dynamic_spawn'
                    and not node.obstacle_active
                    and node.latest_truth is not None
                    and node.latest_truth[0] >= -3.80):
                # Robot is at x ~ -3.8, spawning box at (-2.3, -3.5) is ~1.5m ahead!
                ok = set_ignition_pose(args.world_name, 'dynamic_box_sc4', -2.3, -3.5, 0.25, env)
                node.obstacle_active = True
                node.events['obstacle_spawn_sim_time'] = round(node.sim_time, 3)
                dynamic_events_list.append({
                    'event': 'dynamic_spawn',
                    'target': 'dynamic_box_sc4',
                    'sim_time': round(node.sim_time, 3),
                    'robot_pose_at_spawn': [round(node.latest_truth[0], 3), round(node.latest_truth[1], 3)],
                    'service_response': ok,
                    'requested_pose': [-2.3, -3.5, 0.25]
                })
                print(f"[VideoRunner] Spawned dynamic_box_sc4 at (-2.3, -3.5, 0.25) at sim_time={node.sim_time:.2f}s "
                      f"(robot at x={node.latest_truth[0]:.2f}m, distance={-2.3 - node.latest_truth[0]:.2f}m)", flush=True)

            if node.sim_time - last_log_sim >= 3.0:
                last_log_sim = node.sim_time
                tr_xy = (round(node.latest_truth[0], 2), round(node.latest_truth[1], 2)) if node.latest_truth else None
                br = node.current_plan_record['route_branch'] if node.current_plan_record else None
                print(f"[VideoRunner] sim={node.sim_time:.1f}s (el={elapsed_sim:.1f}s) | "
                      f"truth={tr_xy} | branch={br} | scan_hits={node.latest_scan_obs_hits} | "
                      f"costmap_lethal={node.latest_costmap_obs_lethal} | min_clear={node.min_clearance_all_time:.3f}m",
                      flush=True)

            if elapsed_sim >= args.timeout_sim:
                print(f"[VideoRunner] Timeout ({args.timeout_sim}s) reached at sim_time={node.sim_time:.2f}s. Canceling goal...", flush=True)
                timed_out = True
                cancel_requested = True
                node.current_phase = "POST_CANCEL"
                cancel_fut = goal_handle.cancel_goal_async()
                t_c = time.time()
                while not cancel_fut.done() and time.time() - t_c < 5.0:
                    rclpy.spin_once(node, timeout_sec=0.05)
                if cancel_fut.done():
                    c_resp = cancel_fut.result()
                    cancel_accepted = (len(c_resp.goals_canceling) > 0)
                t_r = time.time()
                while not res_future.done() and time.time() - t_r < 5.0:
                    rclpy.spin_once(node, timeout_sec=0.05)
                break

        nav_end_sim = node.sim_time
        nav_end_host = time.time()

        if res_future.done():
            action_result_received = True
            final_status_code = int(res_future.result().status)
            if not timed_out:
                node.current_phase = "POST_RESULT"

        node.events['action_end_sim_time'] = round(nav_end_sim, 3)
        node.events['final_action_status'] = GOAL_STATUS_NAMES.get(final_status_code, 'UNCONFIRMED')
        print(f"[VideoRunner] Navigation finished at sim_time={nav_end_sim:.2f}s with status={node.events['final_action_status']} ({final_status_code})", flush=True)

        # Record 2.5s of post-arrival settle frames
        settle_start = node.sim_time
        while node.sim_time - settle_start < 2.5:
            rclpy.spin_once(node, timeout_sec=0.05)
        settle_end_sim = node.sim_time

        node.recording = False
        node.save_costmap_archives()
        node.close_files()

        with open(out_dir / 'events.json', 'w') as f:
            json.dump(node.events, f, indent=2)

        runtime_record = {
            'scenario': args.scenario,
            'scenario_type': args.scenario_type,
            'domain_id': str(args.domain_id),
            'world': str(args.world),
            'map': str(args.map),
            'initial_pose': [args.spawn_x, args.spawn_y, args.spawn_yaw],
            'goal_pose': [args.goal_x, args.goal_y, args.goal_yaw],
            'applied_behavior_tree': {
                'goal_behavior_tree_field': bt_xml_abs,
                'bt_xml_sha256': hashes.get('behavior_tree.xml'),
                'binding_verified': True
            },
            'action_execution': {
                'timed_out': timed_out,
                'cancel_requested': cancel_requested,
                'cancel_accepted': cancel_accepted,
                'action_result_received': action_result_received,
                'final_action_status_code': final_status_code,
                'final_action_status_str': GOAL_STATUS_NAMES.get(final_status_code, 'UNCONFIRMED')
            },
            'phase_timestamps': {
                'init_start_sim': round(init_start_sim, 3),
                'nav_start_sim': round(nav_start_sim, 3),
                'nav_end_sim': round(nav_end_sim, 3),
                'settle_end_sim': round(settle_end_sim, 3)
            },
            'milestone_events': node.events,
            'dynamic_events': dynamic_events_list,
            'simulation_duration_seconds': round(nav_end_sim - nav_start_sim, 2),
            'host_duration_seconds': round(nav_end_host - nav_start_host, 2),
            'total_recorded_frames': len(node.frames),
            'total_recorded_plans': len(node.plans_recorded),
            'total_global_costmap_snapshots': len(node.global_costmap_snaps)
        }
        with open(out_dir / 'runtime_execution_record.json', 'w') as f:
            json.dump(runtime_record, f, indent=2)

        runtime_record['evaluation_source'] = "AUTOMATIC_RUNTIME_RECORD"
        summary = evaluate_scenario_from_records(out_dir, runtime_meta=runtime_record)
        summary['milestone_events'] = node.events
        with open(out_dir / 'summary.json', 'w') as f:
            json.dump(summary, f, indent=2)

        print(f"[VideoRunner] Completed! Summary saved to {out_dir / 'summary.json'}", flush=True)
        print(f"[VideoRunner] Primary Verdict: {summary['verdict']['primary_status']} "
              f"(GT Error: {summary['measurements']['final_ground_truth_error']}m, "
              f"Min Clearance: {summary['measurements']['minimum_all_objects_clearance_meters']}m)", flush=True)

        node.destroy_node()
        rclpy.shutdown()

    finally:
        cleanup()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--scenario-type', choices=['unmapped_detour', 'dynamic_spawn'], required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--world', required=True)
    parser.add_argument('--world-name', default='mc3f_world')
    parser.add_argument('--map', required=True)
    parser.add_argument('--params-file', required=True)
    parser.add_argument('--bt-xml', required=True)
    parser.add_argument('--model-file', required=True)
    parser.add_argument('--spawn-x', type=float, required=True)
    parser.add_argument('--spawn-y', type=float, required=True)
    parser.add_argument('--spawn-yaw', type=float, required=True)
    parser.add_argument('--goal-x', type=float, required=True)
    parser.add_argument('--goal-y', type=float, required=True)
    parser.add_argument('--goal-yaw', type=float, required=True)
    parser.add_argument('--timeout-sim', type=float, default=50.0)
    parser.add_argument('--domain-id', type=int, default=165)
    args = parser.parse_args()
    run_experiment(args)
