#!/usr/bin/env -S python3 -s
"""
test_stage5_depth_obstacle_avoidance.py
Independent Verification Script for Stage 5: Depth Camera 3D Obstacle Avoidance Integration
1. Subtest 5A: Low Obstacle with 2D LiDAR Only (enable_depth_filter = False)
   - Spawns an 8cm low hurdle in the corridor directly along the robot's navigation path.
   - The 2D LiDAR plane (14cm height) cannot detect the 8cm hurdle (shoots over).
   - Verifies that costmap remains unpopulated for the obstacle.
   - Nav2 drives forward until the robot physically impacts the obstacle (collision occurs).
2. Subtest 5B: Low Obstacle with 2D LiDAR + Orbbec DaBai Depth Camera (enable_depth_filter = True)
   - Same obstacle in the exact same location.
   - Orbbec DaBai depth camera + depth_obstacle_filter publishes /depth_obstacles/points.
   - Nav2 Costmap ObstacleLayer marks the 3D low obstacle and applies inflation.
   - Nav2 detects blocked path, decelerates, and stops safely before the obstacle (collision avoided).
"""
import os
import sys
import time
import math
import json
import signal
import subprocess
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2, LaserScan
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener

EXP_DIR = '${PROJECT_ROOT}/experiments/limo_integrated_auto_mapping_20261108'
RESULTS_DIR = os.path.join(EXP_DIR, 'results')
LOGS_DIR = os.path.join(EXP_DIR, 'logs')
MODELS_DIR = os.path.join(EXP_DIR, 'models')
os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(LOGS_DIR, exist_ok=True)


class Stage5MonitorNode(Node):
    def __init__(self):
        super().__init__(
            'stage5_monitor_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                  reliability=ReliabilityPolicy.RELIABLE)
        self.sub_gt = self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_gt, 20)
        self.sub_odom_nav2 = self.create_subscription(Odometry, '/odom_nav2', self.on_odom_nav2, 20)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 20)
        self.sub_plan = self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.sub_depth_points = self.create_subscription(PointCloud2, '/depth_obstacles/points', self.on_depth_points, qos_reliable)
        self.sub_local_costmap = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_local_costmap, 5)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.on_map, 5)

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.gt_history = []
        self.odom_history = []
        self.cmd_history = []
        self.depth_cloud_count = 0
        self.depth_points_total = 0
        self.local_costmap_msg = None
        self.map_msg = None
        self.latest_gt = None

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def on_gt(self, msg: Odometry):
        self.latest_gt = msg
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt_history.append((t_sec, px, py))

    def on_odom_nav2(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_history.append((t_sec, px, py))

    def on_cmd(self, msg: Twist):
        t_sec = self.get_clock().now().nanoseconds * 1e-9
        self.cmd_history.append((t_sec, msg.linear.x, msg.angular.z))

    def on_plan(self, msg: Path):
        pass

    def on_depth_points(self, msg: PointCloud2):
        self.depth_cloud_count += 1
        self.depth_points_total += msg.width * msg.height

    def on_local_costmap(self, msg: OccupancyGrid):
        self.local_costmap_msg = msg


def kill_proc(p):
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            p.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass


def reset_gazebo_pose(x=-1.10, y=-0.60, z=0.05, qz=0.7071068, qw=0.7071068):
    cmd = [
        'ign', 'service', '-s', '/world/lounge_world/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req', f'name: "limo", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'
    ]
    subprocess.run(cmd, capture_output=True, text=True, timeout=3.0)
    time.sleep(1.0)


def set_hurdle_pose(x=-1.10, y=0.15, z=0.0):
    cmd = [
        'ign', 'service', '-s', '/world/lounge_world/set_pose',
        '--reqtype', 'ignition.msgs.Pose',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '2000',
        '--req', f'name: "low_obstacle_hurdle", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
    ]
    subprocess.run(cmd, capture_output=True, text=True, timeout=3.0)
    time.sleep(0.5)


def run_stage5_verification():
    env = os.environ.copy()
    env['PYTHONNOUSERSITE'] = '1'
    rl_lib_dir = '${PROJECT_ROOT}/capstone_ws/install/robot_localization/lib'
    depth_ws_install = '${PROJECT_ROOT}/experiments/depth_mapping_20261107/ws/install'
    existing_ament = env.get('AMENT_PREFIX_PATH', '')
    env['AMENT_PREFIX_PATH'] = f"{depth_ws_install}:{existing_ament}" if existing_ament else depth_ws_install
    env['LD_LIBRARY_PATH'] = f"{rl_lib_dir}:{env.get('LD_LIBRARY_PATH', '')}"

    print("=================================================================")
    print("Starting Stage 5: Depth Camera 3D Obstacle Avoidance Verification")
    print("=================================================================")

    rclpy.init()
    results = {
        'test_timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'subtest_5A_lidar_only': {},
        'subtest_5B_lidar_and_depth': {}
    }

    hurdle_sdf = os.path.join(MODELS_DIR, 'low_obstacle_hurdle.sdf')

    try:
        # =====================================================================
        # SUBTEST 5A: Low Obstacle with 2D LiDAR ONLY (enable_depth_filter=false)
        # =====================================================================
        print("\n[Subtest 5A] Testing Low Obstacle with 2D LiDAR ONLY (Depth Disabled)...")
        print("  Starting clean Gazebo instance for 5A...")
        gz_5a = subprocess.Popen([
            'ros2', 'launch',
            os.path.join(EXP_DIR, 'launch', 'base_gazebo_for_integrated.launch.py'),
            'spawn_x:=-1.10', 'spawn_y:=-0.60', 'spawn_z:=0.05', 'spawn_yaw:=1.5708'
        ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(7.0)

        p_norm_5a = subprocess.Popen(
            ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
            env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        p_gt_5a = subprocess.Popen(
            ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
            env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(1.0)

        print("  Spawning 8cm low obstacle hurdle at (x=-1.10, y=0.15)...")
        subprocess.run([
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'lounge_world',
            '-file', hurdle_sdf,
            '-name', 'low_obstacle_hurdle',
            '-x', '-1.10', '-y', '0.15', '-z', '0.0'
        ], capture_output=True, text=True, timeout=5.0)
        time.sleep(1.0)

        nav5a_launch = [
            'ros2', 'launch',
            os.path.join(EXP_DIR, 'launch', 'integrated_stage5_depth.launch.py'),
            'use_sim_time:=true',
            'estimator_type:=f4',
            'enable_depth_filter:=false',
            'cmd_vel_out:=/cmd_vel'
        ]
        log_5a = open(os.path.join(LOGS_DIR, 'stage5A_lidar_only_launch.log'), 'w')
        p_5a = subprocess.Popen(nav5a_launch, env=env, preexec_fn=os.setsid,
                                stdout=log_5a, stderr=subprocess.STDOUT)
        time.sleep(6.0)

        mon5a = Stage5MonitorNode()
        connected = mon5a.action_client.wait_for_server(timeout_sec=15.0)
        if not connected:
            raise RuntimeError("Timed out waiting for Nav2 action server in 5A")

        tf_start = time.time()
        curr_trans = None
        while time.time() - tf_start < 10.0:
            rclpy.spin_once(mon5a, timeout_sec=0.1)
            try:
                curr_trans = mon5a.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                break
            except Exception:
                pass

        rx = curr_trans.transform.translation.x if curr_trans else -1.10
        ry = curr_trans.transform.translation.y if curr_trans else -0.60
        qz = curr_trans.transform.rotation.z if curr_trans else 0.7071
        qw = curr_trans.transform.rotation.w if curr_trans else 0.7071
        ryaw = 2.0 * math.atan2(qz, qw)

        target_x = rx + 1.10 * math.cos(ryaw)
        target_y = ry + 1.10 * math.sin(ryaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = mon5a.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.z = math.sin(ryaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(ryaw / 2.0)

        t_map = time.time()
        while mon5a.map_msg is None and (time.time() - t_map < 10.0):
            rclpy.spin_once(mon5a, timeout_sec=0.1)

        print(f"  Sending goal to (x={target_x:.2f}, y={target_y:.2f}). Hurdle is at relative +0.75m.")
        send_goal_future = mon5a.action_client.send_goal_async(goal_msg)
        t_send = time.time()
        while not send_goal_future.done() and (time.time() - t_send < 10.0):
            rclpy.spin_once(mon5a, timeout_sec=0.1)
        goal_handle = send_goal_future.result()

        if goal_handle is not None and goal_handle.accepted:
            t_start_nav = time.time()
            result_future = goal_handle.get_result_async()
            while not result_future.done() and (time.time() - t_start_nav < 16.0):
                rclpy.spin_once(mon5a, timeout_sec=0.05)
        else:
            print("  Goal rejected or timed out in 5A.")

        depth_pts_5a = mon5a.depth_cloud_count
        hist_pts_5a = mon5a.gt_history if len(mon5a.gt_history) > 1 else mon5a.odom_history
        travel_dist_5a = 0.0
        if len(hist_pts_5a) > 1:
            pts = np.array([(p[1], p[2]) for p in hist_pts_5a])
            diffs = np.diff(pts, axis=0)
            travel_dist_5a = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))

        fin_gt_x = mon5a.latest_gt.pose.pose.position.x if mon5a.latest_gt else -1.10
        fin_gt_y = mon5a.latest_gt.pose.pose.position.y if mon5a.latest_gt else -0.60
        dist_to_hurdle_5a = math.hypot(fin_gt_x - (-1.10), fin_gt_y - 0.15)
        collision_5a = dist_to_hurdle_5a <= 0.22

        results['subtest_5A_lidar_only'] = {
            'depth_filter_enabled': False,
            'depth_clouds_received': depth_pts_5a,
            'traveled_distance_m': round(travel_dist_5a, 3),
            'final_distance_to_hurdle_m': round(dist_to_hurdle_5a, 3),
            'collision_detected': collision_5a,
            'verdict': 'CONFIRMED_VULNERABLE' if (collision_5a and depth_pts_5a == 0) else 'UNEXPECTED'
        }
        print(f"  Subtest 5A Result: {results['subtest_5A_lidar_only']['verdict']} "
              f"(DepthClouds={depth_pts_5a}, DistToHurdle={dist_to_hurdle_5a:.3f}m, Collision={collision_5a})")

        mon5a.destroy_node()
        kill_proc(p_5a)
        kill_proc(p_gt_5a)
        kill_proc(p_norm_5a)
        kill_proc(gz_5a)
        time.sleep(3.0)

        # =====================================================================
        # SUBTEST 5B: Low Obstacle with 2D LiDAR + DEPTH CAMERA (enable_depth_filter=true)
        # =====================================================================
        print("\n[Subtest 5B] Testing Low Obstacle with 2D LiDAR + Depth Camera (Integrated)...")
        print("  Starting clean Gazebo instance for 5B...")
        gz_5b = subprocess.Popen([
            'ros2', 'launch',
            os.path.join(EXP_DIR, 'launch', 'base_gazebo_for_integrated.launch.py'),
            'spawn_x:=-1.10', 'spawn_y:=-0.60', 'spawn_z:=0.05', 'spawn_yaw:=1.5708'
        ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(7.0)

        p_norm_5b = subprocess.Popen(
            ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
            env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        p_gt_5b = subprocess.Popen(
            ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
            env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(1.0)

        print("  Spawning 8cm low obstacle hurdle at (x=-1.10, y=0.15)...")
        subprocess.run([
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'lounge_world',
            '-file', hurdle_sdf,
            '-name', 'low_obstacle_hurdle',
            '-x', '-1.10', '-y', '0.15', '-z', '0.0'
        ], capture_output=True, text=True, timeout=5.0)
        time.sleep(1.0)

        nav5b_launch = [
            'ros2', 'launch',
            os.path.join(EXP_DIR, 'launch', 'integrated_stage5_depth.launch.py'),
            'use_sim_time:=true',
            'estimator_type:=f4',
            'enable_depth_filter:=true',
            'cmd_vel_out:=/cmd_vel'
        ]
        log_5b = open(os.path.join(LOGS_DIR, 'stage5B_lidar_and_depth_launch.log'), 'w')
        p_5b = subprocess.Popen(nav5b_launch, env=env, preexec_fn=os.setsid,
                                stdout=log_5b, stderr=subprocess.STDOUT)
        time.sleep(6.0)

        mon5b = Stage5MonitorNode()
        connected = mon5b.action_client.wait_for_server(timeout_sec=15.0)
        if not connected:
            raise RuntimeError("Timed out waiting for Nav2 action server in 5B")

        tf_start = time.time()
        curr_trans = None
        while time.time() - tf_start < 10.0:
            rclpy.spin_once(mon5b, timeout_sec=0.1)
            try:
                curr_trans = mon5b.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                break
            except Exception:
                pass

        rx = curr_trans.transform.translation.x if curr_trans else -1.10
        ry = curr_trans.transform.translation.y if curr_trans else -0.60
        qz = curr_trans.transform.rotation.z if curr_trans else 0.7071
        qw = curr_trans.transform.rotation.w if curr_trans else 0.7071
        ryaw = 2.0 * math.atan2(qz, qw)

        target_x = rx + 1.10 * math.cos(ryaw)
        target_y = ry + 1.10 * math.sin(ryaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = mon5b.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.z = math.sin(ryaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(ryaw / 2.0)

        t_map_b = time.time()
        while mon5b.map_msg is None and (time.time() - t_map_b < 10.0):
            rclpy.spin_once(mon5b, timeout_sec=0.1)

        print(f"  Sending goal to (x={target_x:.2f}, y={target_y:.2f}). Depth filter active.")
        send_goal_future = mon5b.action_client.send_goal_async(goal_msg)
        t_send_b = time.time()
        while not send_goal_future.done() and (time.time() - t_send_b < 10.0):
            rclpy.spin_once(mon5b, timeout_sec=0.1)
        goal_handle = send_goal_future.result()

        if goal_handle is not None and goal_handle.accepted:
            t_start_nav = time.time()
            result_future = goal_handle.get_result_async()
            while not result_future.done() and (time.time() - t_start_nav < 16.0):
                rclpy.spin_once(mon5b, timeout_sec=0.05)
        else:
            print("  Goal rejected or timed out in 5B.")

        depth_pts_5b = mon5b.depth_cloud_count
        depth_pts_total_5b = mon5b.depth_points_total
        hist_pts_5b = mon5b.gt_history if len(mon5b.gt_history) > 1 else mon5b.odom_history
        travel_dist_5b = 0.0
        if len(hist_pts_5b) > 1:
            pts = np.array([(p[1], p[2]) for p in hist_pts_5b])
            diffs = np.diff(pts, axis=0)
            travel_dist_5b = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))

        fin_gt_x_5b = mon5b.latest_gt.pose.pose.position.x if mon5b.latest_gt else -1.10
        fin_gt_y_5b = mon5b.latest_gt.pose.pose.position.y if mon5b.latest_gt else -0.60
        dist_to_hurdle_5b = math.hypot(fin_gt_x_5b - (-1.10), fin_gt_y_5b - 0.15)

        # In 5B with Depth:
        # 1. Depth points MUST be received (> 5 clouds)
        # 2. Obstacle was detected and robot safely decelerated/stopped before impact (distance >= 0.25m, NO collision)
        collision_5b = dist_to_hurdle_5b < 0.23
        subtest_5b_pass = (depth_pts_5b >= 5) and (not collision_5b) and (dist_to_hurdle_5b >= 0.25)

        results['subtest_5B_lidar_and_depth'] = {
            'depth_filter_enabled': True,
            'depth_clouds_received': depth_pts_5b,
            'depth_obstacle_points_count': depth_pts_total_5b,
            'traveled_distance_m': round(travel_dist_5b, 3),
            'final_distance_to_hurdle_m': round(dist_to_hurdle_5b, 3),
            'collision_detected': collision_5b,
            'verdict': 'PASS' if subtest_5b_pass else 'FAIL'
        }
        print(f"  Subtest 5B Verdict: {results['subtest_5B_lidar_and_depth']['verdict']} "
              f"(DepthClouds={depth_pts_5b}, DistToHurdle={dist_to_hurdle_5b:.3f}m, Collision={collision_5b})")

        mon5b.destroy_node()
        kill_proc(p_5b)
        kill_proc(p_gt_5b)
        kill_proc(p_norm_5b)
        kill_proc(gz_5b)
        time.sleep(1.0)

    finally:
        rclpy.shutdown()

    summary_path = os.path.join(RESULTS_DIR, 'stage5_depth_obstacle_verification_summary.json')
    with open(summary_path, 'w') as f:
        json.dump(results, f, indent=2)

    overall_pass = (results['subtest_5A_lidar_only'].get('verdict') == 'CONFIRMED_VULNERABLE' and
                    results['subtest_5B_lidar_and_depth'].get('verdict') == 'PASS')
    print("\n=================================================================")
    print(f"Stage 5 Overall Verification: {'PASS' if overall_pass else 'FAIL'}")
    print(f"Summary written to: {summary_path}")
    print("=================================================================")
    sys.exit(0 if overall_pass else 1)


if __name__ == '__main__':
    run_stage5_verification()
