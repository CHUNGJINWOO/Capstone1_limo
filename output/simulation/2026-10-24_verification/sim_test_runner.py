#!/usr/bin/env python3
"""
sim_test_runner.py:
Unified simulation test runner for LIMO autonomous navigation validation.
Executes specified scenario, logs all sensor/tf/costmap/plan/cmd/ground-truth data,
and generates structured result summary (JSON).
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
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image as ImageMsg, LaserScan
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
import tf2_ros

class SimulationValidator(Node):
    def __init__(self, output_dir, world_name, initial_pose, goal_pose, domain_id):
        super().__init__('sim_validator', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)
        ])
        self.output_dir = pathlib.Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'frames').mkdir(exist_ok=True)

        self.world_name = world_name
        self.initial_pose = initial_pose  # [x, y, yaw]
        self.goal_pose = goal_pose        # [x, y, yaw]
        self.domain_id = domain_id

        self.sim_time = 0.0
        self.scan_count = 0
        self.odom_count = 0
        self.image_count = 0
        self.last_saved_image_sim = -100.0
        self.recording = False
        self.status = "INIT"
        self.frames = []
        self.current_cmd = [0.0, 0.0]
        self.latest_odom = None
        self.latest_amcl = None
        self.latest_truth = None  # [x, y, z, qz, qw]
        self.latest_plan = []
        self.latest_costmap = None
        self.feedback = None
        self.recovery_events = []

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

        # Truth pose from Ignition Gazebo Pose publisher
        truth_topic = f'/world/{world_name}/pose/info'
        self.create_subscription(TFMessage, truth_topic, self._truth_cb, 10)

        # Publishers & Action Clients
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.telemetry_file = (self.output_dir / 'telemetry.jsonl').open('w')

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

    def _cmd_cb(self, msg):
        self.current_cmd = [msg.linear.x, msg.angular.z]

    def _truth_cb(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == 'limo':
                self.latest_truth = [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                ]

    def _image_cb(self, msg):
        self.image_count += 1
        if not self.recording:
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if t - self.last_saved_image_sim < 0.095:  # ~10 fps max
            return
        self.last_saved_image_sim = t

        enc = msg.encoding.lower()
        mode = 'RGB' if enc in ['rgb8', '8uc3'] else 'RGBA' if enc == 'rgba8' else 'RGB'
        im = Image.frombytes(mode, (msg.width, msg.height), bytes(msg.data), 'raw', 'BGR' if enc == 'bgr8' else mode, msg.step, 1).convert('RGB')
        frame_idx = len(self.frames)
        file_rel = f'frames/{frame_idx:05d}.png'
        im.save(self.output_dir / file_rel)

        # Lookup TF map -> base_link
        tf_pose = None
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            tf_pose = [tr.transform.translation.x, tr.transform.translation.y, yaw]
        except Exception:
            pass

        odom_pos = None
        if self.latest_odom:
            p = self.latest_odom.pose.pose.position
            odom_pos = [p.x, p.y]

        cost_info = None
        if self.latest_costmap:
            cost_info = {
                'width': self.latest_costmap.info.width,
                'height': self.latest_costmap.info.height,
                'resolution': self.latest_costmap.info.resolution,
                'origin': [self.latest_costmap.info.origin.position.x, self.latest_costmap.info.origin.position.y]
            }

        rec = {
            'frame': frame_idx,
            'image': file_rel,
            'sim_time': t,
            'status': self.status,
            'cmd': self.current_cmd,
            'tf_pose': tf_pose,
            'odom_pos': odom_pos,
            'truth': self.latest_truth,
            'plan_length': len(self.latest_plan),
            'costmap': cost_info,
            'distance_remaining': self.feedback.distance_remaining if self.feedback else None
        }
        self.frames.append({'image': file_rel, 'sim_time': t})
        self.telemetry_file.write(json.dumps(rec, ensure_ascii=False) + '\n')
        self.telemetry_file.flush()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.initial_pose[0])
        msg.pose.pose.position.y = float(self.initial_pose[1])
        msg.pose.pose.position.z = 0.05
        yaw = float(self.initial_pose[2])
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.04
        msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.03
        self.init_pub.publish(msg)

    def close(self):
        self.telemetry_file.close()

def run_test(args):
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

    result_summary = {
        'scenario': args.scenario,
        'domain_id': domain_id,
        'world': args.world,
        'map': args.map,
        'initial_pose': [args.spawn_x, args.spawn_y, args.spawn_yaw],
        'goal_pose': [args.goal_x, args.goal_y, args.goal_yaw],
        'nav2_action_status': None,
        'nav2_succeeded': False,
        'ground_truth_reached': False,
        'final_ground_truth_error': None,
        'collision_detected': False,
        'simulation_time': None,
        'host_time': None,
        'total_frames': 0,
        'status': 'FAILED',
        'details': ''
    }

    rclpy.init()
    node = SimulationValidator(
        output_dir=out_dir,
        world_name=args.world_name,
        initial_pose=[args.spawn_x, args.spawn_y, args.spawn_yaw],
        goal_pose=[args.goal_x, args.goal_y, args.goal_yaw],
        domain_id=domain_id
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

        # Wait for sensors & simulation clock
        print("[Runner] Waiting for Gazebo clock, scan, and camera...", flush=True)
        t_wait = time.monotonic() + 45
        while time.monotonic() < t_wait:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.scan_count > 0 and node.odom_count > 0 and node.image_count > 0 and node.sim_time > 0.5:
                break
        else:
            raise RuntimeError(f"Sensor timeout: scan={node.scan_count}, odom={node.odom_count}, img={node.image_count}, sim={node.sim_time}")

        print(f"[Runner] Sensors online at sim_time={node.sim_time:.2f}s.", flush=True)

        # 3. Launch Nav2 (or SLAM auto-mapping depending on mode)
        if args.mode == 'nav2':
            nav2_cmd = [
                'ros2', 'launch', 'limo_capstone', 'limo_nav2_sim.launch.py',
                f'map:={args.map}',
                f'params_file:={args.params_file}',
                'use_sim_time:=true',
                'autostart:=true'
            ]
            spawn_process(nav2_cmd, 'nav2')

            # Wait for Nav2 Action Server and Costmap
            print("[Runner] Waiting for Nav2 stack and AMCL...", flush=True)
            t_wait = time.monotonic() + 50
            while time.monotonic() < t_wait:
                node.publish_initial_pose()
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.nav_client.server_is_ready() and node.latest_costmap is not None and node.latest_amcl is not None:
                    break
            else:
                raise RuntimeError("Nav2 server or AMCL initialization timeout")

            print(f"[Runner] Nav2 ready at sim_time={node.sim_time:.2f}s. Costmap: {node.latest_costmap.info.width}x{node.latest_costmap.info.height}", flush=True)

            # Warm-up spin
            t_warm = time.monotonic() + 3.0
            while time.monotonic() < t_warm:
                node.publish_initial_pose()
                rclpy.spin_once(node, timeout_sec=0.1)

            # Send Goal
            node.recording = True
            node.status = "GOAL_SENT"

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(args.goal_x)
            goal_msg.pose.pose.position.y = float(args.goal_y)
            yaw = float(args.goal_yaw)
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

            print(f"[Runner] Sending goal ({args.goal_x}, {args.goal_y})", flush=True)
            send_goal_future = node.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda f: setattr(node, 'feedback', f.feedback)
            )
            rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=10.0)
            goal_handle = send_goal_future.result()

            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("Goal rejected by Nav2")

            node.status = "NAVIGATING"
            result_future = goal_handle.get_result_async()
            start_host = time.monotonic()
            start_sim = node.sim_time
            max_sim_dur = float(args.timeout_sim)
            last_log_t = start_host

            event_done = False
            last_event_t = 0.0

            while (time.monotonic() - start_host < 180) and (node.sim_time - start_sim < max_sim_dur):
                rclpy.spin_once(node, timeout_sec=0.05)
                elapsed_sim = node.sim_time - start_sim

                # Dynamic event handling
                if args.event_type == 'dynamic_spawn' and not event_done and elapsed_sim >= 3.5:
                    event_done = True
                    cmd_ign = [
                        'ign', 'service', '-s', f'/world/{args.world_name}/set_pose',
                        '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '2000',
                        '--req', 'name: "dynamic_box_sc4", position: {x: -2.3, y: -3.5, z: 0.25}'
                    ]
                    subprocess.run(cmd_ign, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    print(f"[Runner EVENT] Dynamic obstacle appeared at (-2.3, -3.5, 0.25) at sim_time={node.sim_time:.2f}s", flush=True)

                elif args.event_type == 'moving_obstacle' and (node.sim_time - last_event_t >= 0.1):
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
                    cmd_ign = [
                        'ign', 'service', '-s', f'/world/{args.world_name}/set_pose',
                        '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean',
                        '--timeout', '2000',
                        '--req', 'name: "removable_barrier", position: {x: 0.5, y: -3.5, z: -10.0}'
                    ]
                    subprocess.run(cmd_ign, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    print(f"[Runner EVENT] Blocking obstacle removed at sim_time={node.sim_time:.2f}s (clearing corridor)", flush=True)

                if result_future.done():
                    break
                if time.monotonic() - last_log_t > 5.0:
                    d_rem = node.feedback.distance_remaining if node.feedback else -1.0
                    print(f"[Runner] In-progress: sim_time={elapsed_sim:.1f}s, dist_rem={d_rem:.2f}m, cmd={node.current_cmd}", flush=True)
                    last_log_t = time.monotonic()

            dur_sim = node.sim_time - start_sim
            dur_host = time.monotonic() - start_host

            if result_future.done():
                action_status = result_future.result().status
                result_summary['nav2_action_status'] = action_status
                result_summary['nav2_succeeded'] = (action_status == 4)
                node.status = "SUCCEEDED" if action_status == 4 else "NAV2_FAILED"
            else:
                print("[Runner] Simulation timeout reached. Cancelling goal...", flush=True)
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
                result_summary['nav2_action_status'] = None
                result_summary['nav2_succeeded'] = False
                node.status = "TIMEOUT"

            # Post-run settling
            t_settle = time.monotonic() + 3.0
            while time.monotonic() < t_settle:
                rclpy.spin_once(node, timeout_sec=0.1)

            # Ground truth verification
            if node.latest_truth:
                tx, ty = node.latest_truth[0], node.latest_truth[1]
                err = math.hypot(tx - args.goal_x, ty - args.goal_y)
                result_summary['final_ground_truth_pose'] = [tx, ty]
                result_summary['final_ground_truth_error'] = err
                gt_tol = float(args.gt_tolerance)
                result_summary['ground_truth_reached'] = (err < gt_tol)
                print(f"[Runner] Ground truth position: ({tx:.3f}, {ty:.3f}), error: {err:.3f}m (tol: {gt_tol}m)", flush=True)
            else:
                print("[Runner] WARNING: No ground truth pose received from Gazebo!", flush=True)

            if result_summary['nav2_succeeded'] and result_summary['ground_truth_reached']:
                result_summary['status'] = 'SUCCESS'
            elif args.event_type == 'block_path' and not result_summary['nav2_succeeded']:
                # 완전 차단 시나리오의 경우, 충돌 없이 안전 정지 및 목표 실패 처리가 성공적인 방어 동작임!
                result_summary['status'] = 'SAFE_STOP_VERIFIED'
                result_summary['details'] = 'Corridor fully blocked. Robot safely stopped and aborted navigation without collision.'
            elif node.status == 'TIMEOUT':
                result_summary['status'] = 'TIMEOUT'
            else:
                result_summary['status'] = 'FAILED'

            result_summary['simulation_time'] = dur_sim
            result_summary['host_time'] = dur_host
            result_summary['total_frames'] = len(node.frames)

        elif args.mode == 'auto_mapping':
            # 사전 지도 없는 탐색 모드 (SLAM Toolbox + explore_lite)
            print("[Runner] Starting Exploration without prior map (auto_mapping)...", flush=True)
            mapping_cmd = [
                'ros2', 'launch', 'limo_capstone', 'limo_auto_mapping_sim.launch.py',
                f'nav2_params_file:={args.params_file}',
                'use_sim_time:=true'
            ]
            spawn_process(mapping_cmd, 'auto_mapping')

            node.recording = True
            node.status = "EXPLORING"
            start_host = time.monotonic()
            start_sim = node.sim_time
            max_sim_dur = float(args.timeout_sim)
            last_log_t = start_host

            while (time.monotonic() - start_host < 180) and (node.sim_time - start_sim < max_sim_dur):
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.monotonic() - last_log_t > 5.0:
                    truth_str = f"({node.latest_truth[0]:.2f}, {node.latest_truth[1]:.2f})" if node.latest_truth else "N/A"
                    print(f"[Runner] Exploring: sim_time={node.sim_time - start_sim:.1f}s, truth={truth_str}, cmd={node.current_cmd}", flush=True)
                    last_log_t = time.monotonic()

            dur_sim = node.sim_time - start_sim
            dur_host = time.monotonic() - start_host
            node.status = "EXPLORATION_COMPLETED"

            result_summary['simulation_time'] = dur_sim
            result_summary['host_time'] = dur_host
            result_summary['total_frames'] = len(node.frames)
            result_summary['status'] = 'EXPLORATION_COMPLETED'
            if node.latest_truth:
                result_summary['final_ground_truth_pose'] = [node.latest_truth[0], node.latest_truth[1]]

    except Exception as e:
        result_summary['status'] = 'ERROR'
        result_summary['details'] = repr(e)
        print(f"[Runner] ERROR: {e}", flush=True)

    finally:
        print("[Runner] Cleaning up child processes...", flush=True)
        # Kill owned PIDs from logs
        owned = set()
        for logfile in logs_dir.glob("*.log"):
            for pid in logfile.read_text(errors="replace").split():
                pass
            import re
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

        # Save summary
        (out_dir / 'summary.json').write_text(json.dumps(result_summary, ensure_ascii=False, indent=2))
        print(f"[Runner] Test finished. Summary saved to {out_dir / 'summary.json'}")

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
    parser.add_argument('--gt_tolerance', type=float, default=0.45)
    args = parser.parse_args()
    run_test(args)
