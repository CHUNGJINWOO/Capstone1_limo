#!/usr/bin/env python3
"""
sim_rigorous_runner.py:
Rigorous simulation validator with strict phase separation and empirical data persistence:
  1. Phases: INITIALIZING -> NAVIGATING -> POST_RESULT / POST_CANCEL
  2. Standstill measured strictly in NAVIGATING phase linked to obstacle events.
  3. Ground truth header timestamps, receive timestamps, and data gap detection.
  4. Raw data history persistence (gt_pose_history.jsonl, obstacle_pose_history.jsonl, costmap_roi_history.jsonl).
  5. SAT 2D polygon clearance calculation.
  6. Costmap clearing: distinguishes 'no cost >= 50' vs 'all valid cells == 0', rejects unknown (-1), requires hold >= 2.0s, formats latency below observation resolution.
  7. Exact action_msgs/msg/GoalStatus usage.
"""

import os
import sys
import time
import math
import signal
import subprocess
import json
import pathlib
import argparse
import re
from PIL import Image
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Image as ImageMsg, LaserScan
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
import tf2_ros

# Standard ROS 2 GoalStatus names mapping
GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'STATUS_UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'STATUS_ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'STATUS_EXECUTING',
    GoalStatus.STATUS_CANCELING: 'STATUS_CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'STATUS_SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'STATUS_CANCELED',
    GoalStatus.STATUS_ABORTED: 'STATUS_ABORTED'
}

# Import authoritative evaluation functions from shared evaluation_lib
from evaluation_lib import (
    point_to_segment_distance,
    compute_polygon_clearance_sat,
    make_robot_polygon,
    make_box_polygon,
    compute_obb_aabb_clearance,
    analyze_physical_standstill,
    analyze_costmap_clearing,
)


class RigorousValidator(Node):
    def __init__(self, output_dir, world_name, obstacles, monitor_roi=False):
        super().__init__('rigorous_validator', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)
        ])
        self.output_dir = pathlib.Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'frames').mkdir(exist_ok=True)

        self.world_name = world_name
        self.obstacles = obstacles  # List of [cx, cy, sx, sy, name]
        self.monitor_roi = monitor_roi

        self.sim_time = 0.0
        self.scan_count = 0
        self.odom_count = 0
        self.image_count = 0
        self.last_saved_image_sim = -100.0
        self.recording = False
        self.status = "INIT"
        self.current_phase = "INITIALIZING"  # INITIALIZING, NAVIGATING, POST_RESULT, POST_CANCEL
        self.frames = []
        self.current_cmd = [0.0, 0.0]
        self.latest_odom = None
        self.latest_amcl = None
        self.latest_truth = None  # [x, y, z, qz, qw]
        self.latest_plan = []
        self.latest_costmap = None
        self.feedback = None

        # Rigorous metrics tracking
        self.min_clearance_all_time = float('inf')
        self.collision_occurred = False
        self.collision_object = None
        self.gt_pose_history = []
        self.monitored_cost_history = []
        self.dynamic_obstacle_poses = {}   # model_name: [x, y, z]
        self.initial_free_cells = None
        self.final_free_cells = None
        self.trajectory_length = 0.0
        self.prev_truth_pos = None

        # Robot dimensions
        self.robot_width = 0.22
        self.robot_length = 0.32

        # Raw history log files for complete reproducibility
        self.gt_pose_file = (self.output_dir / 'gt_pose_history.jsonl').open('w')
        self.obstacle_pose_file = (self.output_dir / 'obstacle_pose_history.jsonl').open('w')
        self.costmap_roi_file = (self.output_dir / 'costmap_roi_history.jsonl').open('w')
        self.telemetry_file = (self.output_dir / 'telemetry.jsonl').open('w')

        # Subscriptions
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._costmap_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._slam_map_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(ImageMsg, '/demo/overhead', self._image_cb, qos_profile_sensor_data)

        # Ground truth pose topic
        truth_topic = f'/world/{world_name}/pose/info'
        self.create_subscription(TFMessage, truth_topic, self._truth_cb, 10)

        # Publishers & Action Clients
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _clock_cb(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec / 1e9

    def _scan_cb(self, msg):
        self.scan_count += 1

    def _odom_cb(self, msg):
        self.odom_count += 1
        self.latest_odom = msg

    def _amcl_cb(self, msg):
        self.latest_amcl = msg

    def _plan_cb(self, msg):
        self.latest_plan = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]

    def _costmap_cb(self, msg):
        self.latest_costmap = msg
        if not self.monitor_roi or not msg.data:
            return

        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        res = msg.info.resolution
        w, h = msg.info.width, msg.info.height
        data = msg.data

        def get_region_costs(x_min, x_max, y_min, y_max):
            px_min = max(0, min(w - 1, int(round((x_min - ox) / res))))
            px_max = max(0, min(w - 1, int(round((x_max - ox) / res))))
            py_min = max(0, min(h - 1, int(round((y_min - oy) / res))))
            py_max = max(0, min(h - 1, int(round((y_max - oy) / res))))
            vals = []
            for py in range(min(py_min, py_max), max(py_min, py_max) + 1):
                for px in range(min(px_min, px_max), max(px_min, px_max) + 1):
                    vals.append(int(data[py * w + px]))
            return vals

        # Barrier ROI in Scenario 7: Center (0.5, -3.5), size 0.60 x 1.60
        front_vals = get_region_costs(0.15, 0.25, -4.0, -3.0)
        interior_vals = get_region_costs(0.35, 0.65, -4.0, -3.0)
        inflation_vals = get_region_costs(-0.10, 0.15, -4.2, -2.8)
        all_roi_vals = get_region_costs(0.15, 0.65, -4.0, -3.0)

        barrier_pose = self.dynamic_obstacle_poses.get('removable_barrier', None)

        unknown_cnt = sum(1 for c in all_roi_vals if c == -1)
        zero_cnt = sum(1 for c in all_roi_vals if c == 0)
        high_cnt = sum(1 for c in all_roi_vals if c >= 50)
        lethal_cnt = sum(1 for c in all_roi_vals if c == 100)
        all_zero = (len(all_roi_vals) > 0 and unknown_cnt == 0 and zero_cnt == len(all_roi_vals))

        entry = {
            'sim_time': round(self.sim_time, 3),
            'header_stamp': round(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9, 3),
            'phase': self.current_phase,
            'barrier_gt_translation': barrier_pose,
            'front_surface': {
                'max_cost': max(front_vals) if front_vals else 0,
                'mean_cost': round(float(np.mean([v for v in front_vals if v >= 0])), 1) if front_vals else 0.0,
                'marked_count': sum(1 for v in front_vals if v >= 50)
            },
            'interior': {
                'max_cost': max(interior_vals) if interior_vals else 0,
                'mean_cost': round(float(np.mean([v for v in interior_vals if v >= 0])), 1) if interior_vals else 0.0,
                'marked_count': sum(1 for v in interior_vals if v >= 50)
            },
            'all_roi': {
                'total_cells': len(all_roi_vals),
                'unknown_count': unknown_cnt,
                'zero_cost_count': zero_cnt,
                'high_cost_count': high_cnt,
                'lethal_count': lethal_cnt,
                'all_valid_cells_zero': all_zero
            }
        }
        self.monitored_cost_history.append(entry)
        self.costmap_roi_file.write(json.dumps(entry) + '\n')
        self.costmap_roi_file.flush()

    def _slam_map_cb(self, msg):
        data = np.array(msg.data)
        free_count = int((data == 0).sum())
        if self.initial_free_cells is None and free_count > 0:
            self.initial_free_cells = free_count
        self.final_free_cells = free_count

    def _cmd_cb(self, msg):
        self.current_cmd = [msg.linear.x, msg.angular.z]

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

                self.latest_truth = [rx, ry, rz, qz, qw]

                pos = (rx, ry)
                if self.prev_truth_pos:
                    d = math.hypot(pos[0] - self.prev_truth_pos[0], pos[1] - self.prev_truth_pos[1])
                    if d < 0.5:
                        self.trajectory_length += d
                self.prev_truth_pos = pos

                # Persist raw GT pose record
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
                self.gt_pose_history.append(gt_rec)
                self.gt_pose_file.write(json.dumps(gt_rec) + '\n')
                self.gt_pose_file.flush()

                # SAT Polygon-to-Polygon Continuous Clearance Calculation
                robot_poly = make_robot_polygon(rx, ry, ryaw, self.robot_width, self.robot_length)
                frame_min_clear = float('inf')

                for obs in self.obstacles:
                    bx, by, bw, bl, name = obs
                    mname = name.split('::')[0]
                    if mname in self.dynamic_obstacle_poses:
                        dpos = self.dynamic_obstacle_poses[mname]
                        if dpos[2] < -5.0:  # underground -> inactive/cleared
                            continue
                        bx, by = dpos[0], dpos[1]

                    box_poly = make_box_polygon(bx, by, bw, bl)
                    c = compute_polygon_clearance_sat(robot_poly, box_poly)
                    if c < frame_min_clear:
                        frame_min_clear = c
                    if c <= 0.005:  # 5mm geometric non-intersection threshold
                        self.collision_occurred = True
                        self.collision_object = name

                if frame_min_clear < self.min_clearance_all_time:
                    self.min_clearance_all_time = frame_min_clear

            elif cid in ['dynamic_box_sc4', 'moving_box_sc5', 'barricade_total', 'removable_barrier', 'unmapped_box_sc3']:
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
        if t - self.last_saved_image_sim < 0.095:
            return
        self.last_saved_image_sim = t

        enc = msg.encoding.lower()
        mode = 'RGB' if enc in ['rgb8', '8uc3'] else 'RGBA' if enc == 'rgba8' else 'RGB'
        im = Image.frombytes(mode, (msg.width, msg.height), bytes(msg.data), 'raw', 'BGR' if enc == 'bgr8' else mode, msg.step, 1).convert('RGB')
        frame_idx = len(self.frames)
        file_rel = f'frames/{frame_idx:05d}.png'
        im.save(self.output_dir / file_rel)

        tf_pose = None
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            tf_pose = [tr.transform.translation.x, tr.transform.translation.y, yaw]
        except Exception:
            pass

        odom_pos = [self.latest_odom.pose.pose.position.x, self.latest_odom.pose.pose.position.y] if self.latest_odom else None

        rec = {
            'frame': frame_idx,
            'image': file_rel,
            'sim_time': round(t, 3),
            'phase': self.current_phase,
            'cmd': self.current_cmd,
            'tf_pose': tf_pose,
            'odom_pos': odom_pos,
            'truth': self.latest_truth,
            'min_clearance_all_time': float(self.min_clearance_all_time),
            'collision_occurred': self.collision_occurred,
            'distance_remaining': self.feedback.distance_remaining if self.feedback else None
        }
        self.frames.append({'image': file_rel, 'sim_time': t})
        self.telemetry_file.write(json.dumps(rec, ensure_ascii=False) + '\n')
        self.telemetry_file.flush()

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.05
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.03
        self.init_pub.publish(msg)

    def close(self):
        self.gt_pose_file.close()
        self.obstacle_pose_file.close()
        self.costmap_roi_file.close()
        self.telemetry_file.close()

def parse_walls_from_sdf(sdf_path):
    """Parses all collision box geometry [cx, cy, sx, sy, name] from SDF file"""
    boxes = []
    text = pathlib.Path(sdf_path).read_text()
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', text, re.DOTALL):
        mname = m.group(1)
        if mname in ['ground_plane', 'demo_camera']:
            continue
        mbody = m.group(2)
        mp_match = re.search(r'^\s*<pose>([-\d\.\s]+)</pose>', mbody, re.MULTILINE)
        mx, my = 0.0, 0.0
        if mp_match:
            vals = [float(v) for v in mp_match.group(1).split()]
            mx, my = vals[0], vals[1]

        for c in re.finditer(r'<collision name="([^"]+)">(.*?)</collision>', mbody, re.DOTALL):
            cname = c.group(1)
            cbody = c.group(2)
            cp_match = re.search(r'<pose>([-\d\.\s]+)</pose>', cbody)
            cx, cy = mx, my
            if cp_match:
                vals = [float(v) for v in cp_match.group(1).split()]
                cx += vals[0]
                cy += vals[1]
            sz_match = re.search(r'<size>([-\d\.\s]+)</size>', cbody)
            if sz_match:
                sz_vals = [float(v) for v in sz_match.group(1).split()]
                sx, sy = sz_vals[0], sz_vals[1]
                boxes.append((cx, cy, sx, sy, f'{mname}::{cname}'))
    return boxes

def run_rigorous_test(args):
    out_dir = pathlib.Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    logs_dir = out_dir / 'logs'
    logs_dir.mkdir(exist_ok=True)

    domain_id = str(args.domain_id)
    os.environ['ROS_DOMAIN_ID'] = domain_id
    os.environ['IGN_PARTITION'] = f'partition_{domain_id}'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

    env = os.environ.copy()
    children = []
    log_files = []

    def spawn_process(cmd, name):
        log_f = (logs_dir / f'{name}.log').open('w')
        log_files.append(log_f)
        p = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        children.append(p)
        return p

    obstacles = parse_walls_from_sdf(args.world)
    monitor_roi = (args.event_type == 'clear_obstacle')

    # Result Summary Structure
    result_summary = {
        'scenario': args.scenario,
        'domain_id': domain_id,
        'world': args.world,
        'map': args.map,
        'initial_pose': [args.spawn_x, args.spawn_y, args.spawn_yaw],
        'goal_pose': [args.goal_x, args.goal_y, args.goal_yaw],
        'criteria': {
            'goal_tolerance_strict': 0.15,
            'goal_tolerance_relaxed': 0.40,
            'collision_clearance_threshold': 0.005,
            'safe_stop_speed_threshold': 0.015,
            'safe_stop_duration_threshold': 3.0,
            'safe_stop_clearance_threshold': 0.10
        },
        'measurements': {
            'action_execution': {
                'timed_out': False,
                'cancel_requested': False,
                'cancel_accepted': None,
                'action_result_received': False,
                'final_action_status_code': None,
                'final_action_status_str': 'UNCONFIRMED'
            },
            'final_ground_truth_pose': None,
            'final_ground_truth_error': None,
            'minimum_obstacle_clearance_meters': None,
            'clearance_type': 'SAMPLED_2D_OBB_AABB_GEOMETRIC_NON_INTERSECTION',
            'collision_detected': False,
            'collision_object': None,
            'phase_timestamps': {
                'init_start_sim': None,
                'nav_start_sim': None,
                'nav_end_sim': None,
                'settle_end_sim': None
            },
            'standstill_analysis': {
                'navigating_max_continuous_stop_sec': 0.0,
                'navigating_post_obstacle_stop_sec': 0.0,
                'initial_wait_stop_sec': 0.0,
                'post_result_stop_sec': 0.0,
                'post_cancel_stop_sec': 0.0,
                'telemetry_quality': 'DEGRADED',
                'large_gaps_count': 0,
                'samples_count': 0
            },
            'simulation_duration_seconds': None,
            'host_duration_seconds': None,
            'total_recorded_frames': 0,
            'dynamic_events': [],
            'costmap_clearing_verification': {
                'roi_monitoring_enabled': monitor_roi,
                'pre_removal_front_max_cost': None,
                'pre_removal_interior_max_cost': None,
                'physical_removal_confirmed': False,
                'post_removal_all_valid_cells_zero': False,
                'post_removal_unknown_cells_detected': True,
                'clearing_hold_duration_seconds': 0.0,
                'clearing_hold_verified': False,
                'clearing_latency_seconds_raw': None,
                'clearing_latency_display': None,
                'clearing_verified': False
            },
            'exploration_metrics': {
                'initial_free_cells': None,
                'final_free_cells': None,
                'free_cells_increase': None,
                'trajectory_length_meters': None
            }
        },
        'verdict': {
            'goal_reached_strict': 'NOT_PASSED',
            'goal_reached_relaxed': 'NOT_PASSED',
            'collision_free_verified': 'NOT_PASSED',
            'safe_stop_verified': 'NOT_APPLICABLE',
            'clearing_verified': 'NOT_MEASURED',
            'exploration_completed': 'NOT_PASSED',
            'primary_status': 'FAILED',
            'rationale': ''
        }
    }

    rclpy.init()
    node = RigorousValidator(
        output_dir=out_dir,
        world_name=args.world_name,
        obstacles=obstacles,
        monitor_roi=monitor_roi
    )

    try:
        # 1. Launch Gazebo Simulation
        gz_cmd = [
            'ros2', 'launch', 'limo_capstone', 'limo_mc3f_sim.launch.py',
            f'world:={args.world}',
            f'model_file:={args.model_file}',
            'headless:=true',
            'rviz:=false',
            'use_sim_time:=true',
            f'spawn_x:={args.spawn_x}',
            f'spawn_y:={args.spawn_y}',
            f'spawn_z:=0.05',
            f'spawn_yaw:={args.spawn_yaw}'
        ]
        spawn_process(gz_cmd, 'gazebo')

        # 2. Bridge camera and truth pose
        truth_topic = f'/world/{args.world_name}/pose/info'
        bridge_cmd = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/demo/overhead@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'{truth_topic}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ]
        spawn_process(bridge_cmd, 'video_bridge')

        print(f"[RigorousRunner] Waiting for clock, sensors, and Gazebo world ({len(obstacles)} obstacle elements)...", flush=True)
        t_wait = time.monotonic() + 45
        while time.monotonic() < t_wait:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.scan_count > 0 and node.odom_count > 0 and node.image_count > 0 and node.sim_time > 0.5:
                break
        else:
            raise RuntimeError(f"Sensor timeout: scan={node.scan_count}, odom={node.odom_count}, img={node.image_count}, sim={node.sim_time}")

        result_summary['measurements']['phase_timestamps']['init_start_sim'] = round(node.sim_time, 3)
        print(f"[RigorousRunner] Phase INITIALIZING active at sim_time={node.sim_time:.2f}s.", flush=True)

        if args.mode == 'nav2':
            nav2_cmd = [
                'ros2', 'launch', 'limo_capstone', 'limo_nav2_sim.launch.py',
                f'map:={args.map}',
                f'params_file:={args.params_file}',
                'use_sim_time:=true',
                'autostart:=true'
            ]
            spawn_process(nav2_cmd, 'nav2')

            print("[RigorousRunner] Waiting for Nav2 AMCL and Costmap...", flush=True)
            t_wait = time.monotonic() + 50
            while time.monotonic() < t_wait:
                node.publish_initial_pose(args.spawn_x, args.spawn_y, args.spawn_yaw)
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.nav_client.server_is_ready() and node.latest_costmap is not None and node.latest_amcl is not None:
                    break
            else:
                raise RuntimeError("Nav2 server or AMCL initialization timeout")

            # Warm-up spin
            t_warm = time.monotonic() + 3.0
            while time.monotonic() < t_warm:
                node.publish_initial_pose(args.spawn_x, args.spawn_y, args.spawn_yaw)
                rclpy.spin_once(node, timeout_sec=0.1)

            # Send Goal -> Transition to NAVIGATING
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(args.goal_x)
            goal_msg.pose.pose.position.y = float(args.goal_y)
            yaw = float(args.goal_yaw)
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

            print(f"[RigorousRunner] Sending goal ({args.goal_x}, {args.goal_y}, yaw={args.goal_yaw})", flush=True)
            send_goal_future = node.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda f: setattr(node, 'feedback', f.feedback)
            )
            rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=10.0)
            goal_handle = send_goal_future.result()

            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("Goal rejected by Nav2")

            # Goal accepted: Enter NAVIGATING Phase!
            node.recording = True
            node.status = "NAVIGATING"
            node.current_phase = "NAVIGATING"
            result_summary['measurements']['phase_timestamps']['nav_start_sim'] = round(node.sim_time, 3)
            print(f"[RigorousRunner] Phase NAVIGATING started at sim_time={node.sim_time:.2f}s.", flush=True)

            result_future = goal_handle.get_result_async()
            start_host = time.monotonic()
            start_sim = node.sim_time
            max_sim_dur = float(args.timeout_sim)
            last_log_t = start_host

            event_done = False
            last_event_t = 0.0
            timed_out = False
            obstacle_event_time = None
            phys_clear_time = None

            while (time.monotonic() - start_host < 180) and (node.sim_time - start_sim < max_sim_dur):
                rclpy.spin_once(node, timeout_sec=0.05)
                elapsed_sim = node.sim_time - start_sim

                # Dynamic event execution & verification
                if args.event_type == 'dynamic_spawn' and not event_done and elapsed_sim >= 3.5:
                    event_done = True
                    obstacle_event_time = node.sim_time
                    cmd_ign = [
                        'ign', 'service', '-s', f'/world/{args.world_name}/set_pose',
                        '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '2000',
                        '--req', 'name: "dynamic_box_sc4", position: {x: -2.3, y: -3.5, z: 0.25}'
                    ]
                    res = subprocess.run(cmd_ign, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    srv_success = "data: true" in res.stdout
                    event_info = {
                        'event': 'dynamic_spawn',
                        'target': 'dynamic_box_sc4',
                        'sim_time': round(node.sim_time, 3),
                        'service_response': srv_success,
                        'requested_pose': [-2.3, -3.5, 0.25]
                    }
                    result_summary['measurements']['dynamic_events'].append(event_info)
                    print(f"[Rigorous EVENT] Dynamic obstacle appeared: srv_ok={srv_success} at sim_time={node.sim_time:.2f}s", flush=True)

                elif args.event_type == 'moving_obstacle' and (node.sim_time - last_event_t >= 0.1):
                    if obstacle_event_time is None:
                        obstacle_event_time = node.sim_time
                    last_event_t = node.sim_time
                    cur_y = min(0.6, -1.6 + 0.14 * elapsed_sim)
                    cmd_ign = [
                        'ign', 'service', '-s', f'/world/{args.world_name}/set_pose',
                        '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '500',
                        '--req', f'name: "moving_box_sc5", position: {{x: 0.0, y: {cur_y:.3f}, z: 0.25}}'
                    ]
                    subprocess.run(cmd_ign, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                elif args.event_type == 'clear_obstacle' and not event_done and elapsed_sim >= 7.0:
                    event_done = True
                    phys_clear_time = node.sim_time
                    cmd_ign = [
                        'ign', 'service', '-s', f'/world/{args.world_name}/set_pose',
                        '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '2000',
                        '--req', 'name: "removable_barrier", position: {x: 0.5, y: -3.5, z: -10.0}'
                    ]
                    res = subprocess.run(cmd_ign, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                    srv_success = "data: true" in res.stdout
                    event_info = {
                        'event': 'clear_obstacle',
                        'target': 'removable_barrier',
                        'sim_time': round(node.sim_time, 3),
                        'service_response': srv_success,
                        'requested_pose': [0.5, -3.5, -10.0]
                    }
                    result_summary['measurements']['dynamic_events'].append(event_info)
                    print(f"[Rigorous EVENT] Obstacle cleared: srv_ok={srv_success} at sim_time={node.sim_time:.2f}s", flush=True)

                if result_future.done():
                    break

                if time.monotonic() - last_log_t > 5.0:
                    d_rem = node.feedback.distance_remaining if node.feedback else -1.0
                    print(f"[RigorousRunner] [{node.current_phase}] sim_time={elapsed_sim:.1f}s, dist_rem={d_rem:.2f}m, min_clearance={node.min_clearance_all_time:.3f}m", flush=True)
                    last_log_t = time.monotonic()

            dur_sim = node.sim_time - start_sim
            dur_host = time.monotonic() - start_host
            result_summary['measurements']['phase_timestamps']['nav_end_sim'] = round(node.sim_time, 3)

            # Phase transition after NAVIGATING
            if result_future.done():
                node.current_phase = "POST_RESULT"
                print(f"[RigorousRunner] Action finished before timeout. Phase POST_RESULT active at sim_time={node.sim_time:.2f}s.", flush=True)
            else:
                timed_out = True
                node.current_phase = "POST_CANCEL"
                result_summary['measurements']['action_execution']['timed_out'] = True
                result_summary['measurements']['action_execution']['cancel_requested'] = True
                print(f"[RigorousRunner] Timeout reached ({args.timeout_sim}s). Phase POST_CANCEL active, sending cancel...", flush=True)

                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
                cancel_res = cancel_future.result()
                if cancel_res is not None:
                    result_summary['measurements']['action_execution']['cancel_accepted'] = (cancel_res.return_code == 0)

                # Wait up to 5s for action server to report terminal status
                rclpy.spin_until_future_complete(node, result_future, timeout_sec=5.0)

            # Settle period (3.0s) strictly recorded in POST_RESULT / POST_CANCEL
            t_settle = time.monotonic() + 3.0
            while time.monotonic() < t_settle:
                rclpy.spin_once(node, timeout_sec=0.1)

            result_summary['measurements']['phase_timestamps']['settle_end_sim'] = round(node.sim_time, 3)

            # Record exact GoalStatus from action_msgs definition
            if result_future.done():
                status_code = result_future.result().status
                result_summary['measurements']['action_execution']['action_result_received'] = True
                result_summary['measurements']['action_execution']['final_action_status_code'] = status_code
                result_summary['measurements']['action_execution']['final_action_status_str'] = GOAL_STATUS_NAMES.get(status_code, f'UNKNOWN_{status_code}')
            else:
                result_summary['measurements']['action_execution']['action_result_received'] = False
                result_summary['measurements']['action_execution']['final_action_status_code'] = None
                result_summary['measurements']['action_execution']['final_action_status_str'] = 'UNCONFIRMED'

            # Physical Ground Truth position & error
            if node.latest_truth:
                tx, ty = node.latest_truth[0], node.latest_truth[1]
                gt_err = math.hypot(tx - args.goal_x, ty - args.goal_y)
                result_summary['measurements']['final_ground_truth_pose'] = [round(tx, 4), round(ty, 4)]
                result_summary['measurements']['final_ground_truth_error'] = round(gt_err, 4)
            else:
                gt_err = float('inf')

            # Physical standstill analysis strictly separated by phase
            standstill_res = analyze_physical_standstill(
                node.gt_pose_history,
                obstacle_event_time=obstacle_event_time,
                speed_thresh=result_summary['criteria']['safe_stop_speed_threshold']
            )
            result_summary['measurements']['standstill_analysis'] = standstill_res
            result_summary['measurements']['minimum_obstacle_clearance_meters'] = round(float(node.min_clearance_all_time), 4)
            result_summary['measurements']['collision_detected'] = bool(node.collision_occurred)
            result_summary['measurements']['collision_object'] = node.collision_object
            result_summary['measurements']['simulation_duration_seconds'] = round(dur_sim, 2)
            result_summary['measurements']['host_duration_seconds'] = round(dur_host, 2)
            result_summary['measurements']['total_recorded_frames'] = len(node.frames)

            # Costmap Multi-Cell ROI Clearing Analysis using shared evaluation_lib
            if monitor_roi and node.monitored_cost_history and phys_clear_time is not None:
                clearing_res = analyze_costmap_clearing(
                    node.monitored_cost_history,
                    phys_clear_time=phys_clear_time,
                    required_hold_duration=2.0,
                    max_obs_gap=3.0,
                    obs_resolution=2.0
                )
                result_summary['measurements']['costmap_clearing_verification'] = clearing_res
                result_summary['verdict']['clearing_verified'] = 'PASSED' if clearing_res['clearing_verified'] else 'NOT_PASSED'


            # 4 Independent Verdict Criteria Evaluation
            status_code = result_summary['measurements']['action_execution']['final_action_status_code']
            nav2_succ = (status_code == GoalStatus.STATUS_SUCCEEDED)

            gt_strict_pass = (nav2_succ and gt_err <= result_summary['criteria']['goal_tolerance_strict'])
            gt_relaxed_pass = (nav2_succ and gt_err <= result_summary['criteria']['goal_tolerance_relaxed'])
            coll_free_pass = (not node.collision_occurred) and (node.min_clearance_all_time > result_summary['criteria']['collision_clearance_threshold'])

            result_summary['verdict']['goal_reached_strict'] = 'PASSED' if gt_strict_pass else 'NOT_PASSED'
            result_summary['verdict']['goal_reached_relaxed'] = 'PASSED' if gt_relaxed_pass else 'NOT_PASSED'
            result_summary['verdict']['collision_free_verified'] = 'PASSED' if coll_free_pass else 'NOT_PASSED'

            # Safe stop evaluation:
            # Applicable ONLY to obstacle interception scenarios (dynamic spawn, moving obstacle, complete blockage)
            is_obstacle_scenario = (args.event_type in ['dynamic_spawn', 'moving_obstacle', 'block_path'])
            if is_obstacle_scenario:
                nav_stop_sec = standstill_res['navigating_max_continuous_stop_sec']
                safe_stop_pass = (
                    coll_free_pass and
                    (nav_stop_sec >= result_summary['criteria']['safe_stop_duration_threshold']) and
                    (node.min_clearance_all_time >= result_summary['criteria']['safe_stop_clearance_threshold'])
                )
                result_summary['verdict']['safe_stop_verified'] = 'PASSED' if safe_stop_pass else 'NOT_PASSED'
            else:
                result_summary['verdict']['safe_stop_verified'] = 'NOT_APPLICABLE'

            # Primary Verdict Determination
            if gt_strict_pass and coll_free_pass:
                result_summary['verdict']['primary_status'] = 'SUCCESS_STRICT'
                result_summary['verdict']['rationale'] = f'Goal reached with strict tolerance (GT error: {gt_err:.3f}m <= 0.15m), non-intersecting 2D clearance: {node.min_clearance_all_time:.3f}m.'
            elif gt_relaxed_pass and coll_free_pass:
                result_summary['verdict']['primary_status'] = 'SUCCESS_RELAXED_ONLY'
                result_summary['verdict']['rationale'] = f'Goal reached only under relaxed tolerance (GT error: {gt_err:.3f}m <= 0.40m), non-intersecting 2D clearance: {node.min_clearance_all_time:.3f}m.'
            elif args.event_type == 'block_path':
                if result_summary['verdict']['safe_stop_verified'] == 'PASSED':
                    result_summary['verdict']['primary_status'] = 'SAFE_STOP_PASSED'
                    result_summary['verdict']['rationale'] = f'Corridor fully blocked. Robot safely stopped during NAVIGATING phase (clearance: {node.min_clearance_all_time:.3f}m >= 0.10m), standstill for {standstill_res["navigating_max_continuous_stop_sec"]:.1f}s.'
                else:
                    result_summary['verdict']['primary_status'] = 'SAFE_STOP_NOT_PASSED'
                    result_summary['verdict']['rationale'] = f'Corridor fully blocked. Safe stop failed: clearance={node.min_clearance_all_time:.3f}m (thresh 0.10m), autonomous navigating stop={standstill_res["navigating_max_continuous_stop_sec"]:.1f}s (thresh 3.0s).'
            elif timed_out:
                result_summary['verdict']['primary_status'] = 'TIMEOUT_CANCELLED'
                result_summary['verdict']['rationale'] = f'Goal incomplete within timeout ({args.timeout_sim}s). Action state: {result_summary["measurements"]["action_execution"]["final_action_status_str"]}. Final GT error: {gt_err:.3f}m.'
            else:
                result_summary['verdict']['primary_status'] = 'FAILED'
                result_summary['verdict']['rationale'] = f'Navigation failed (status: {result_summary["measurements"]["action_execution"]["final_action_status_str"]}, GT error: {gt_err:.3f}m, collision={node.collision_occurred}).'

        elif args.mode == 'auto_mapping':
            print("[RigorousRunner] Starting Exploration without prior map (auto_mapping)...", flush=True)
            mapping_cmd = [
                'ros2', 'launch', 'limo_capstone', 'limo_auto_mapping_sim.launch.py',
                f'nav2_params_file:={args.params_file}',
                'use_sim_time:=true'
            ]
            spawn_process(mapping_cmd, 'auto_mapping')

            node.recording = True
            node.status = "EXPLORING"
            node.current_phase = "NAVIGATING"
            result_summary['measurements']['phase_timestamps']['nav_start_sim'] = round(node.sim_time, 3)

            start_host = time.monotonic()
            start_sim = node.sim_time
            max_sim_dur = float(args.timeout_sim)
            last_log_t = start_host

            while (time.monotonic() - start_host < 180) and (node.sim_time - start_sim < max_sim_dur):
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.monotonic() - last_log_t > 5.0:
                    truth_str = f"({node.latest_truth[0]:.2f}, {node.latest_truth[1]:.2f})" if node.latest_truth else "N/A"
                    free_cnt = node.final_free_cells or 0
                    print(f"[RigorousRunner] Exploring: sim_time={node.sim_time - start_sim:.1f}s, truth={truth_str}, free_cells={free_cnt}, traj={node.trajectory_length:.2f}m", flush=True)
                    last_log_t = time.monotonic()

            dur_sim = node.sim_time - start_sim
            dur_host = time.monotonic() - start_host
            result_summary['measurements']['phase_timestamps']['nav_end_sim'] = round(node.sim_time, 3)

            # Save generated map
            map_save_path = out_dir / 'exploration_result_map'
            save_cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(map_save_path), '--ros-args', '-p', 'use_sim_time:=true']
            subprocess.run(save_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)

            standstill_res = analyze_physical_standstill(node.gt_pose_history)
            result_summary['measurements']['standstill_analysis'] = standstill_res
            result_summary['measurements']['minimum_obstacle_clearance_meters'] = round(float(node.min_clearance_all_time), 4)
            result_summary['measurements']['collision_detected'] = bool(node.collision_occurred)
            result_summary['measurements']['collision_object'] = node.collision_object
            result_summary['measurements']['simulation_duration_seconds'] = round(dur_sim, 2)
            result_summary['measurements']['host_duration_seconds'] = round(dur_host, 2)
            result_summary['measurements']['total_recorded_frames'] = len(node.frames)

            init_c = node.initial_free_cells or 0
            final_c = node.final_free_cells or 0
            inc_c = max(0, final_c - init_c)
            result_summary['measurements']['exploration_metrics'] = {
                'initial_free_cells': init_c,
                'final_free_cells': final_c,
                'free_cells_increase': inc_c,
                'trajectory_length_meters': round(node.trajectory_length, 2)
            }

            coll_free_pass = (not node.collision_occurred) and (node.min_clearance_all_time > 0.005)
            result_summary['verdict']['collision_free_verified'] = 'PASSED' if coll_free_pass else 'NOT_PASSED'
            result_summary['verdict']['safe_stop_verified'] = 'NOT_APPLICABLE'
            result_summary['verdict']['exploration_completed'] = 'NOT_PASSED'
            result_summary['verdict']['primary_status'] = 'EXPLORATION_PARTIAL_TIMELIMIT'
            result_summary['verdict']['rationale'] = f'Exploration executed for {dur_sim:.1f}s time limit. Known free cells increased by {inc_c} (+{(inc_c/(init_c+1e-5))*100:.1f}%), traveled {node.trajectory_length:.2f}m non-intersecting (clearance: {node.min_clearance_all_time:.3f}m). Frontiers still remained when stopped.'

    except Exception as e:
        result_summary['verdict']['primary_status'] = 'ERROR'
        result_summary['verdict']['rationale'] = repr(e)
        print(f"[RigorousRunner] ERROR: {e}", flush=True)

    finally:
        print("[RigorousRunner] Tearing down processes...", flush=True)
        owned = set()
        for logfile in logs_dir.glob("*.log"):
            for pid in re.findall(r"process started with pid \[(\d+)\]", logfile.read_text(errors="replace")):
                owned.add(int(pid))
        for pid in owned:
            try: os.kill(pid, signal.SIGINT)
            except ProcessLookupError: pass
        for p in reversed(children):
            if p.poll() is None:
                try: os.killpg(p.pid, signal.SIGINT)
                except ProcessLookupError: pass
        for p in reversed(children):
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try: os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError: pass
        for pid in owned:
            try: os.kill(pid, signal.SIGKILL)
            except ProcessLookupError: pass
        for f in log_files:
            f.close()

        node.close()
        node.destroy_node()
        rclpy.shutdown()

        summary_file = out_dir / 'summary.json'
        summary_file.write_text(json.dumps(result_summary, ensure_ascii=False, indent=2))
        print(f"[RigorousRunner] Finished. Rigorous summary saved to {summary_file}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--output_dir', required=True)
    parser.add_argument('--world', required=True)
    parser.add_argument('--world_name', default='mc3f_world')
    parser.add_argument('--map', required=True)
    parser.add_argument('--model_file', required=True)
    parser.add_argument('--params_file', required=True)
    parser.add_argument('--spawn_x', type=float, default=0.0)
    parser.add_argument('--spawn_y', type=float, default=0.0)
    parser.add_argument('--spawn_yaw', type=float, default=0.0)
    parser.add_argument('--goal_x', type=float, default=1.5)
    parser.add_argument('--goal_y', type=float, default=0.0)
    parser.add_argument('--goal_yaw', type=float, default=0.0)
    parser.add_argument('--domain_id', type=int, default=88)
    parser.add_argument('--timeout_sim', type=float, default=50.0)
    parser.add_argument('--mode', choices=['nav2', 'auto_mapping'], default='nav2')
    parser.add_argument('--event_type', choices=['none', 'dynamic_spawn', 'moving_obstacle', 'block_path', 'clear_obstacle'], default='none')
    parser.add_argument('--monitor_x', type=float, default=None)
    parser.add_argument('--monitor_y', type=float, default=None)
    args = parser.parse_args()
    run_rigorous_test(args)
