#!/usr/bin/env python3
"""
run_crossing_experiments.py:
Scientific test runner for moving obstacle scenarios:
  1. Scenario A (clean_crossing): Obstacle traverses corridor and completely clears/disappears.
  2. Scenario B (corridor_remain): Obstacle traverses into corridor center and remains blocking.
Strictly archives copies and SHA-256 hashes of Nav2 params, BT XML, world SDF, robot model SDF, and execution args.
Instruments root-cause isolation across:
  - Step 1: Costmap clearing in corridor ROI
  - Step 2: Global planner path generation
  - Step 3: Local controller / RPP command execution
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

BASE_DIR = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(BASE_DIR / 'output/simulation'))
from evaluation_lib import (
    point_to_segment_distance,
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

class CrossingExperimentValidator(Node):
    def __init__(self, output_dir, world_name, scenario_type, sdf_boxes):
        super().__init__('crossing_experiment_validator', parameter_overrides=[
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

        self.sim_time = 0.0
        self.scan_count = 0
        self.odom_count = 0
        self.image_count = 0
        self.last_saved_image_sim = -100.0
        self.current_phase = "INITIALIZING"

        self.latest_odom = None
        self.latest_amcl = None
        self.latest_truth = None
        self.latest_plan = []
        self.latest_costmap = None
        self.latest_cmd = [0.0, 0.0]
        self.feedback = None

        self.min_moving_clearance_all_time = float('inf')
        self.min_wall_clearance_all_time = float('inf')
        self.min_clearance_all_time = float('inf')
        self.collision_occurred = False
        self.collision_object = None
        self.dynamic_obstacle_poses = {}

        self.prev_truth_pos = None
        self.trajectory_length = 0.0

        # Robot dimensions
        self.robot_width = 0.22
        self.robot_length = 0.32

        # Files
        self.gt_pose_file = (self.output_dir / 'gt_pose_history.jsonl').open('w')
        self.obstacle_pose_file = (self.output_dir / 'obstacle_pose_history.jsonl').open('w')
        self.costmap_roi_file = (self.output_dir / 'costmap_roi_history.jsonl').open('w')
        self.telemetry_file = (self.output_dir / 'telemetry.jsonl').open('w')
        self.plan_history_file = (self.output_dir / 'plan_history.jsonl').open('w')
        self.cmd_history_file = (self.output_dir / 'cmd_history.jsonl').open('w')

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
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(ImageMsg, '/demo/overhead', self._image_cb, qos_profile_sensor_data)

        truth_topic = f'/world/{world_name}/pose/info'
        self.create_subscription(TFMessage, truth_topic, self._truth_cb, 10)

        # Publishers & Action Clients
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

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
        plan_rec = {
            'sim_time': round(self.sim_time, 3),
            'phase': self.current_phase,
            'waypoint_count': len(msg.poses),
            'start_pose': [round(msg.poses[0].pose.position.x, 3), round(msg.poses[0].pose.position.y, 3)] if msg.poses else None,
            'goal_pose': [round(msg.poses[-1].pose.position.x, 3), round(msg.poses[-1].pose.position.y, 3)] if msg.poses else None
        }
        self.plan_history_file.write(json.dumps(plan_rec) + '\n')
        self.plan_history_file.flush()

    def _cmd_cb(self, msg):
        self.latest_cmd = [msg.linear.x, msg.angular.z]
        cmd_rec = {
            'sim_time': round(self.sim_time, 3),
            'phase': self.current_phase,
            'vx': round(msg.linear.x, 4),
            'wz': round(msg.angular.z, 4)
        }
        self.cmd_history_file.write(json.dumps(cmd_rec) + '\n')
        self.cmd_history_file.flush()

    def _costmap_cb(self, msg):
        self.latest_costmap = msg
        if not msg.data:
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

        # South Corridor ROI: x in [-0.25, 0.25], y in [-1.30, -0.40]
        corridor_vals = get_region_costs(-0.25, 0.25, -1.30, -0.40)
        # Center cell line: x in [-0.10, 0.10], y in [-1.00, -0.70]
        center_vals = get_region_costs(-0.10, 0.10, -1.00, -0.70)

        box_pose = self.dynamic_obstacle_poses.get('moving_box_sc5', None)

        unknown_cnt = sum(1 for c in corridor_vals if c == -1)
        zero_cnt = sum(1 for c in corridor_vals if c == 0)
        lethal_cnt = sum(1 for c in corridor_vals if c == 100)
        high_cnt = sum(1 for c in corridor_vals if c >= 50)

        all_zero = (len(corridor_vals) > 0 and unknown_cnt == 0 and zero_cnt == len(corridor_vals))

        entry = {
            'sim_time': round(self.sim_time, 3),
            'phase': self.current_phase,
            'obstacle_pose': box_pose,
            'corridor_roi': {
                'total_cells': len(corridor_vals),
                'max_cost': max(corridor_vals) if corridor_vals else 0,
                'mean_cost': round(float(np.mean([v for v in corridor_vals if v >= 0])), 1) if corridor_vals else 0.0,
                'lethal_count': lethal_cnt,
                'high_cost_count': high_cnt,
                'zero_cost_count': zero_cnt,
                'unknown_count': unknown_cnt,
                'all_valid_cells_zero': all_zero
            },
            'corridor_center': {
                'max_cost': max(center_vals) if center_vals else 0,
                'lethal_count': sum(1 for c in center_vals if c == 100)
            }
        }
        self.costmap_roi_file.write(json.dumps(entry) + '\n')
        self.costmap_roi_file.flush()

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

                # 1. Static walls
                for sb in self.static_boxes:
                    b_poly = make_box_polygon(sb['center'][0], sb['center'][1], sb['size'][0], sb['size'][1])
                    c = compute_polygon_clearance_sat(robot_poly, b_poly)
                    if c < self.min_wall_clearance_all_time:
                        self.min_wall_clearance_all_time = c
                    if c <= 0.005:
                        self.collision_occurred = True
                        self.collision_object = sb['name']

                # 2. Dynamic obstacles
                for db in self.dynamic_boxes:
                    mname = db['model_name']
                    if mname in self.dynamic_obstacle_poses:
                        dpos = self.dynamic_obstacle_poses[mname]
                        if dpos[2] < -5.0:  # Physically removed
                            continue
                        bx, by = dpos[0], dpos[1]
                    else:
                        bx, by = db['center'][0], db['center'][1]

                    b_poly = make_box_polygon(bx, by, db['size'][0], db['size'][1])
                    c = compute_polygon_clearance_sat(robot_poly, b_poly)
                    if c < self.min_moving_clearance_all_time:
                        self.min_moving_clearance_all_time = c
                    if c <= 0.005:
                        self.collision_occurred = True
                        self.collision_object = db['name']

                self.min_clearance_all_time = min(self.min_wall_clearance_all_time, self.min_moving_clearance_all_time)

            elif cid in ['moving_box_sc5', 'barricade_total', 'removable_barrier']:
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
        if self.sim_time - self.last_saved_image_sim >= 0.25:
            self.last_saved_image_sim = self.sim_time
            # Save frame metadata
            telem = {
                'sim_time': round(self.sim_time, 3),
                'phase': self.current_phase,
                'min_moving_clearance': round(self.min_moving_clearance_all_time, 4) if self.min_moving_clearance_all_time != float('inf') else None,
                'min_wall_clearance': round(self.min_wall_clearance_all_time, 4) if self.min_wall_clearance_all_time != float('inf') else None,
                'min_clearance': round(self.min_clearance_all_time, 4) if self.min_clearance_all_time != float('inf') else None,
                'collision': self.collision_occurred,
                'cmd_vel': self.latest_cmd
            }
            self.telemetry_file.write(json.dumps(telem) + '\n')
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
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.02
        self.init_pub.publish(msg)

    def close_files(self):
        for f in [self.gt_pose_file, self.obstacle_pose_file, self.costmap_roi_file,
                  self.telemetry_file, self.plan_history_file, self.cmd_history_file]:
            try:
                f.close()
            except Exception:
                pass


def parse_maze_obstacles(sdf_path):
    import xml.etree.ElementTree as ET
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    obstacles = []
    for coll in root.findall('.//collision'):
        name = coll.get('name')
        pose_elem = coll.find('pose')
        size_elem = coll.find('.//size')
        if pose_elem is None or size_elem is None:
            continue
        pv = [float(v) for v in pose_elem.text.split()]
        sv = [float(v) for v in size_elem.text.split()]
        if len(pv) >= 3 and len(sv) >= 2:
            obstacles.append([pv[0], pv[1], sv[0], sv[1], name])
    return obstacles


def run_experiment(args):
    out_dir = pathlib.Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    logs_dir = out_dir / 'logs'
    logs_dir.mkdir(parents=True, exist_ok=True)

    print("================================================================================")
    print(f"  CROSSING EXPERIMENT: {args.scenario}")
    print(f"  Output Directory: {out_dir}")
    print("================================================================================")

    # 1. Archive copies of config files and calculate SHA-256 hashes
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

    # 2. Setup environment
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
        print("[CrossingRunner] Terminating background processes...", flush=True)
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
            lf.close()
        subprocess.run(['pkill', '-9', '-f', f'partition_{domain_id}'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(['pkill', '-9', '-f', 'ign gazebo'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    sdf_boxes = parse_sdf_boxes(args.world)

    rclpy.init()
    node = CrossingExperimentValidator(
        output_dir=out_dir,
        world_name=args.world_name,
        scenario_type=args.scenario,
        sdf_boxes=sdf_boxes
    )

    try:
        # Launch Gazebo
        gz_cmd = [
            'ros2', 'launch', 'limo_capstone', 'limo_mc3f_sim.launch.py',
            f'world:={args.world}',
            f'model_file:={args.model_file}',
            'headless:=true',
            'rviz:=false',
            'use_sim_time:=true',
            f'spawn_x:={args.spawn_x}',
            f'spawn_y:={args.spawn_y}',
            'spawn_z:=0.05',
            f'spawn_yaw:={args.spawn_yaw}'
        ]
        spawn_proc(gz_cmd, 'gazebo')

        # Bridge
        truth_topic = f'/world/{args.world_name}/pose/info'
        bridge_cmd = [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/demo/overhead@sensor_msgs/msg/Image[ignition.msgs.Image',
            f'{truth_topic}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ]
        spawn_proc(bridge_cmd, 'video_bridge')

        print("[CrossingRunner] Waiting for Gazebo clock and sensors...", flush=True)
        t_w = time.monotonic() + 45
        while time.monotonic() < t_w:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.scan_count > 0 and node.odom_count > 0 and node.sim_time > 0.5:
                break
        else:
            raise RuntimeError(f"Gazebo sensor timeout: scan={node.scan_count}, odom={node.odom_count}, img={node.image_count}, sim={node.sim_time}")

        print(f"[CrossingRunner] Gazebo active at sim_time={node.sim_time:.2f}s", flush=True)

        # Launch Nav2
        nav2_cmd = [
            'ros2', 'launch', 'limo_capstone', 'limo_nav2_sim.launch.py',
            f'map:={args.map}',
            f'params_file:={args.params_file}',
            'use_sim_time:=true',
            'autostart:=true'
        ]
        spawn_proc(nav2_cmd, 'nav2')

        print("[CrossingRunner] Waiting for Nav2 AMCL and Costmap...", flush=True)
        t_w = time.monotonic() + 50
        while time.monotonic() < t_w:
            node.publish_initial_pose(args.spawn_x, args.spawn_y, args.spawn_yaw)
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.nav_client.server_is_ready() and node.latest_costmap is not None and node.latest_amcl is not None:
                break
        else:
            raise RuntimeError("Nav2 server or costmap timeout")

        # Warm-up spin
        t_warm = time.monotonic() + 3.0
        while time.monotonic() < t_warm:
            node.publish_initial_pose(args.spawn_x, args.spawn_y, args.spawn_yaw)
            rclpy.spin_once(node, timeout_sec=0.1)

        # Send Navigation Goal with explicitly bound Behavior Tree XML
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(args.goal_x)
        goal_msg.pose.pose.position.y = float(args.goal_y)
        yaw = float(args.goal_yaw)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        bt_xml_abs = str(pathlib.Path(args.bt_xml).resolve())
        if not os.path.exists(bt_xml_abs):
            raise FileNotFoundError(f"Target Behavior Tree XML not found: {bt_xml_abs}")
        goal_msg.behavior_tree = bt_xml_abs
        bt_hash = compute_sha256(bt_xml_abs)
        print(f"[CrossingRunner] BOUND BEHAVIOR TREE TO GOAL: {bt_xml_abs} (SHA256: {bt_hash})", flush=True)

        print(f"[CrossingRunner] Sending goal ({args.goal_x}, {args.goal_y}) with BT: {pathlib.Path(bt_xml_abs).name}", flush=True)
        send_goal_future = node.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda f: setattr(node, 'feedback', f.feedback)
        )
        rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=10.0)
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            raise RuntimeError("Nav2 goal rejected!")

        node.current_phase = "NAVIGATING"
        start_sim = node.sim_time
        start_host = time.monotonic()
        result_future = goal_handle.get_result_async()

        # Dynamic execution loop
        last_event_t = 0.0
        obstacle_cleared_physically = False
        obstacle_clear_sim_time = None
        box_y = -1.6

        while (time.monotonic() - start_host < 180) and (node.sim_time - start_sim < args.timeout_sim):
            rclpy.spin_once(node, timeout_sec=0.05)
            elapsed_sim = node.sim_time - start_sim

            if args.scenario == 'clean_crossing':
                # Move box north across corridor, then remove completely
                if (node.sim_time - last_event_t >= 0.1) and not obstacle_cleared_physically:
                    last_event_t = node.sim_time
                    box_y = -1.6 + 0.14 * elapsed_sim

                    if box_y < -0.10:
                        # Traversing corridor
                        cmd_ign = [
                            'ign', 'service', '-s', f'/world/{args.world_name}/set_pose',
                            '--reqtype', 'ignition.msgs.Pose',
                            '--reptype', 'ignition.msgs.Boolean',
                            '--timeout', '500',
                            '--req', f'name: "moving_box_sc5", position: {{x: 0.0, y: {box_y:.3f}, z: 0.25}}'
                        ]
                        subprocess.run(cmd_ign, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    else:
                        # Box has fully cleared the corridor -> remove to z = -10.0!
                        obstacle_cleared_physically = True
                        obstacle_clear_sim_time = node.sim_time
                        cmd_ign = [
                            'ign', 'service', '-s', f'/world/{args.world_name}/set_pose',
                            '--reqtype', 'ignition.msgs.Pose',
                            '--reptype', 'ignition.msgs.Boolean',
                            '--timeout', '2000',
                            '--req', 'name: "moving_box_sc5", position: {x: 0.0, y: 0.0, z: -10.0}'
                        ]
                        res = subprocess.run(cmd_ign, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                        print(f"\n[CrossingRunner EVENT] Obstacle physically cleared corridor & removed to z=-10 at sim_time={node.sim_time:.2f}s (srv_ok={'data: true' in res.stdout})", flush=True)

            elif args.scenario == 'corridor_remain':
                # Move box into center of corridor (y = -0.85) and leave it there
                if node.sim_time - last_event_t >= 0.1:
                    last_event_t = node.sim_time
                    box_y = min(-0.85, -1.6 + 0.14 * elapsed_sim)
                    cmd_ign = [
                        'ign', 'service', '-s', f'/world/{args.world_name}/set_pose',
                        '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '500',
                        '--req', f'name: "moving_box_sc5", position: {{x: 0.0, y: {box_y:.3f}, z: 0.25}}'
                    ]
                    subprocess.run(cmd_ign, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            if result_future.done():
                break

        dur_sim = node.sim_time - start_sim
        dur_host = time.monotonic() - start_host

        timed_out = not result_future.done()
        cancel_requested = False
        cancel_accepted = None
        action_res_received = False
        final_status_code = None
        final_status_str = "UNCONFIRMED"

        if timed_out:
            print(f"[CrossingRunner] Timeout reached ({args.timeout_sim}s). Cancelling goal...", flush=True)
            node.current_phase = "POST_CANCEL"
            cancel_requested = True
            cf = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cf, timeout_sec=5.0)
            if cf.done():
                cres = cf.result()
                cancel_accepted = (cres.return_code == 0)
            t_settle = time.monotonic() + 3.0
            while time.monotonic() < t_settle:
                rclpy.spin_once(node, timeout_sec=0.1)
                if result_future.done():
                    break
        else:
            node.current_phase = "POST_RESULT"
            t_settle = time.monotonic() + 2.0
            while time.monotonic() < t_settle:
                rclpy.spin_once(node, timeout_sec=0.1)

        if result_future.done():
            action_res_received = True
            final_status_code = result_future.result().status
            final_status_str = GOAL_STATUS_NAMES.get(final_status_code, f"CODE_{final_status_code}")

        print(f"[CrossingRunner] Action finished: code={final_status_code} ({final_status_str})", flush=True)

        # Construct authoritative runtime execution record directly from run state
        runtime_execution_record = {
            'evaluation_source': 'AUTOMATIC_RUNTIME_RECORD',
            'scenario': args.scenario,
            'domain_id': args.domain_id,
            'world': str(args.world),
            'map': str(args.map),
            'params_file': str(args.params_file),
            'bt_xml': bt_xml_abs,
            'applied_behavior_tree': {
                'path': bt_xml_abs,
                'sha256': bt_hash,
                'name': pathlib.Path(bt_xml_abs).name
            },
            'model_file': str(args.model_file),
            'initial_pose': [float(args.spawn_x), float(args.spawn_y), float(args.spawn_yaw)],
            'goal_pose': [float(args.goal_x), float(args.goal_y), float(args.goal_yaw)],
            'simulation_duration_seconds': round(dur_sim, 3),
            'host_duration_seconds': round(dur_host, 3),
            'action_execution': {
                'timed_out': timed_out,
                'cancel_requested': cancel_requested,
                'cancel_accepted': cancel_accepted,
                'action_result_received': action_res_received,
                'final_action_status_code': final_status_code,
                'final_action_status_str': final_status_str
            },
            'event_timestamps': {
                'start_sim_time': round(start_sim, 3),
                'action_end_sim_time': round(node.sim_time, 3),
                'obstacle_cleared_sim_time': round(obstacle_clear_sim_time, 3) if obstacle_clear_sim_time else None
            },
            'dynamic_events': [
                {
                    'event': 'moving_obstacle',
                    'sim_time': 10.0,
                    'target_y': -0.10 if args.scenario == 'clean_crossing' else -0.85
                }
            ]
        }
        if obstacle_cleared_physically and obstacle_clear_sim_time is not None:
            runtime_execution_record['dynamic_events'].append({
                'event': 'clear_obstacle',
                'sim_time': round(obstacle_clear_sim_time, 3)
            })

        # Save authoritative runtime record to raw disk file
        with open(out_dir / 'runtime_execution_record.json', 'w') as f:
            json.dump(runtime_execution_record, f, indent=2)

    finally:
        node.close_files()
        cleanup()
        rclpy.shutdown()

    # 3. Post-run Authoritative Evaluation via evaluation_lib
    print("\n[CrossingRunner] Running authoritative evaluation from raw logs with runtime metadata...", flush=True)
    eval_result = evaluate_scenario_from_records(out_dir, runtime_meta=runtime_execution_record)

    # Save reevaluation_summary.json
    with open(out_dir / 'reevaluation_summary.json', 'w') as f:
        json.dump(eval_result, f, indent=2)

    # Save summary.json
    with open(out_dir / 'summary.json', 'w') as f:
        json.dump(eval_result, f, indent=2)

    # Print Key Diagnostics
    m = eval_result['measurements']
    v = eval_result['verdict']
    act = m['action_execution']
    ss = m['standstill_analysis']

    print("--------------------------------------------------------------------------------")
    print(f"  Final Action Status: {act.get('final_action_status_str')} (code {act.get('final_action_status_code')})")
    print(f"  GT Final Error: {m.get('final_ground_truth_error')} m")
    print(f"  Moving Box SAT Min Clearance: {m.get('minimum_moving_obstacle_clearance_meters')} m")
    print(f"  Wall SAT Min Clearance: {m.get('minimum_static_wall_clearance_meters')} m")
    print(f"  All Objects SAT Min Clearance: {m.get('minimum_all_objects_clearance_meters')} m")
    print(f"  Navigating Standstill: {ss.get('navigating_max_continuous_stop_sec'):.2f} s")
    print(f"  Primary Status: {v.get('primary_status')}")
    print(f"  Rationale: {v.get('rationale')}")
    print("--------------------------------------------------------------------------------\n")

    return eval_result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Moving Obstacle Crossing Experiment Runner")
    parser.add_argument('--scenario', required=True, choices=['clean_crossing', 'corridor_remain'], help="Scenario type")
    parser.add_argument('--domain-id', default='162', help="ROS Domain ID")
    parser.add_argument('--world-name', default='mc3f_world', help="Gazebo world name")
    parser.add_argument('--world', default=str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_sc5_moving.sdf'), help="World SDF path")
    parser.add_argument('--map', default=str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_map.yaml'), help="Map YAML path")
    parser.add_argument('--params-file', default=str(BASE_DIR / 'capstone_ws/src/limo_capstone/config/nav2_sim_ackermann_params.yaml'), help="Nav2 params path")
    parser.add_argument('--bt-xml', default=str(BASE_DIR / 'capstone_ws/src/limo_capstone/config/navigate_to_pose_ackermann.xml'), help="BT XML path")
    parser.add_argument('--model-file', default=str(BASE_DIR / 'output/simulation/2026-10-24/inputs/limo_contact_check.sdf'), help="Robot model SDF path")
    parser.add_argument('--output-dir', required=True, help="Output directory for results")
    parser.add_argument('--spawn-x', default='-2.0', help="Spawn X")
    parser.add_argument('--spawn-y', default='0.0', help="Spawn Y")
    parser.add_argument('--spawn-yaw', default='0.0', help="Spawn Yaw")
    parser.add_argument('--goal-x', default='2.5', help="Goal X")
    parser.add_argument('--goal-y', default='0.0', help="Goal Y")
    parser.add_argument('--goal-yaw', default='0.0', help="Goal Yaw")
    parser.add_argument('--timeout-sim', type=float, default=55.0, help="Simulation timeout in seconds")

    args = parser.parse_args()
    run_experiment(args)
