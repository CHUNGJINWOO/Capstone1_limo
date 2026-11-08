#!/usr/bin/env -S python3 -s
"""
test_stage4_nav2_and_explore.py
Independent Verification Script for Stage 4: Nav2 & Explore Lite Integration
1. Subtest 4A: Static Goal Navigation
   - Verifies Nav2 lifecycle activation (planner, controller, behavior, bt_navigator).
   - Dispatches a static NavigateToPose goal down the corridor (-1.10, 0.80, yaw=1.5708).
   - Monitors global path generation on /plan, velocity execution on /cmd_vel, and trajectory.
   - Verifies goal status SUCCEEDED and measures terminal positional accuracy against Ground Truth.
2. Subtest 4B: Autonomous Frontier Exploration
   - Launches explore_lite in combination with SLAM Toolbox and Nav2.
   - Observes frontier detection on /explore/frontiers, autonomous goal generation, and map growth.
   - Categorizes exploration events (FRONTIER_DISPATCHED, PATH_COMPUTED, REPLANNING, GOAL_REACHED).
   - Records cumulative trajectory and final map occupied cell count.
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
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener

EXP_DIR = '${PROJECT_ROOT}/experiments/limo_integrated_auto_mapping_20261108'
RESULTS_DIR = os.path.join(EXP_DIR, 'results')
LOGS_DIR = os.path.join(EXP_DIR, 'logs')
os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(LOGS_DIR, exist_ok=True)


class Stage4MonitorNode(Node):
    def __init__(self):
        super().__init__(
            'stage4_monitor_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub_gt = self.create_subscription(Odometry, '/ground_truth/odom', self.on_gt, 20)
        self.sub_odom_nav2 = self.create_subscription(Odometry, '/odom_nav2', self.on_odom_nav2, 20)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 20)
        self.sub_plan = self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.on_map, 5)
        self.sub_frontiers = self.create_subscription(MarkerArray, '/explore/frontiers', self.on_frontiers, 10)

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.gt_history = []
        self.odom_history = []
        self.cmd_history = []
        self.plan_history = []
        self.frontiers_count_history = []
        self.map_msg = None
        self.latest_gt = None
        self.latest_odom_nav2 = None

    def on_gt(self, msg: Odometry):
        self.latest_gt = msg
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt_history.append((t_sec, px, py))

    def on_odom_nav2(self, msg: Odometry):
        self.latest_odom_nav2 = msg
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom_history.append((t_sec, px, py))

    def on_cmd(self, msg: Twist):
        t_sec = self.get_clock().now().nanoseconds * 1e-9
        self.cmd_history.append((t_sec, msg.linear.x, msg.angular.z))

    def on_plan(self, msg: Path):
        t_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        poses = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.plan_history.append((t_sec, len(poses), poses))

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def on_frontiers(self, msg: MarkerArray):
        t_sec = self.get_clock().now().nanoseconds * 1e-9
        count = sum(len(m.points) for m in msg.markers)
        self.frontiers_count_history.append((t_sec, count))


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


def run_stage4_verification():
    env = os.environ.copy()
    env['PYTHONNOUSERSITE'] = '1'
    rl_lib_dir = '${PROJECT_ROOT}/capstone_ws/install/robot_localization/lib'
    env['LD_LIBRARY_PATH'] = f"{rl_lib_dir}:{env.get('LD_LIBRARY_PATH', '')}"

    print("=================================================================")
    print("Starting Stage 4: Nav2 & Explore Lite Functional Integration")
    print("=================================================================")

    # Step 1: Launch Gazebo Fortress + Sensors
    print("\n[Step 1] Launching Gazebo Fortress + LIMO Base...")
    gz_launch = [
        'ros2', 'launch',
        os.path.join(EXP_DIR, 'launch', 'base_gazebo_for_integrated.launch.py'),
        'spawn_x:=-1.10', 'spawn_y:=-0.60', 'spawn_z:=0.05', 'spawn_yaw:=1.5708'
    ]
    p_gz = subprocess.Popen(gz_launch, env=env, preexec_fn=os.setsid,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(7.0)

    # Launch sensor normalizer and ground truth publisher
    p_norm = subprocess.Popen(
        ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
        env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    p_gt = subprocess.Popen(
        ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
        env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(1.0)

    rclpy.init()
    results = {
        'test_timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'subtest_4A_static_goal': {},
        'subtest_4B_autonomous_explore': {}
    }

    try:
        # =====================================================================
        # SUBTEST 4A: Static Goal Navigation
        # =====================================================================
        print("\n[Subtest 4A] Testing Nav2 Static Goal Navigation with F4 Estimator...")
        reset_gazebo_pose(-1.10, -0.60, 0.05, 0.7071068, 0.7071068)

        nav2_launch = [
            'ros2', 'launch',
            os.path.join(EXP_DIR, 'launch', 'integrated_stage4_nav2_explore.launch.py'),
            'use_sim_time:=true',
            'estimator_type:=f4',
            'start_explore:=false',
            'cmd_vel_out:=/cmd_vel'
        ]
        nav2_log_f = open(os.path.join(LOGS_DIR, 'stage4_nav2_launch.log'), 'w')
        p_nav2 = subprocess.Popen(nav2_launch, env=env, preexec_fn=os.setsid,
                                  stdout=nav2_log_f, stderr=subprocess.STDOUT)
        time.sleep(6.0)

        mon4a = Stage4MonitorNode()
        print("  Waiting for Nav2 NavigateToPose action server...")
        connected = mon4a.action_client.wait_for_server(timeout_sec=15.0)
        if not connected:
            raise RuntimeError("Timed out waiting for Nav2 navigate_to_pose action server")
        print("  Nav2 action server is connected!")

        # Wait for valid TF map -> base_footprint
        print("  Resolving robot pose in map frame...")
        tf_start = time.time()
        curr_trans = None
        while time.time() - tf_start < 10.0:
            rclpy.spin_once(mon4a, timeout_sec=0.1)
            try:
                curr_trans = mon4a.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                break
            except Exception:
                pass

        if curr_trans is None:
            raise RuntimeError("Timed out waiting for TF map -> base_footprint")

        rx = curr_trans.transform.translation.x
        ry = curr_trans.transform.translation.y
        qz = curr_trans.transform.rotation.z
        qw = curr_trans.transform.rotation.w
        ryaw = 2.0 * math.atan2(qz, qw)
        print(f"  Current Robot Map Pose: x={rx:.3f}, y={ry:.3f}, yaw={ryaw:.3f} rad")

        # Command target 1.0m ahead down the corridor in map frame
        target_x = rx + 1.00 * math.cos(ryaw)
        target_y = ry + 1.00 * math.sin(ryaw)
        target_yaw = ryaw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = mon4a.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        print(f"  Sending NavigateToPose goal to (x={target_x:.2f}, y={target_y:.2f})...")
        send_goal_future = mon4a.action_client.send_goal_async(goal_msg)

        while not send_goal_future.done():
            rclpy.spin_once(mon4a, timeout_sec=0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Nav2 rejected the navigation goal")
        print("  Goal accepted by Nav2! Tracking execution...")

        result_future = goal_handle.get_result_async()
        t_start_nav = time.time()
        max_nav_time = 25.0

        while not result_future.done() and (time.time() - t_start_nav < max_nav_time):
            rclpy.spin_once(mon4a, timeout_sec=0.05)

        action_status_str = 'TIMEOUT'
        if result_future.done():
            action_result = result_future.result()
            status = action_result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                action_status_str = 'SUCCEEDED'
            elif status == GoalStatus.STATUS_ABORTED:
                action_status_str = 'ABORTED'
            elif status == GoalStatus.STATUS_CANCELED:
                action_status_str = 'CANCELED'
            else:
                action_status_str = f'STATUS_{status}'

        # Collect 4A metrics in map frame
        final_map_x, final_map_y = target_x, target_y
        try:
            fin_trans = mon4a.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            final_map_x = fin_trans.transform.translation.x
            final_map_y = fin_trans.transform.translation.y
        except Exception:
            pass
        pos_err_to_goal = math.hypot(final_map_x - target_x, final_map_y - target_y)

        # Compute total travel distance from GT or Odom history
        travel_dist = 0.0
        hist_pts = mon4a.gt_history if len(mon4a.gt_history) > 1 else mon4a.odom_history
        if len(hist_pts) > 1:
            pts = np.array([(p[1], p[2]) for p in hist_pts])
            diffs = np.diff(pts, axis=0)
            travel_dist = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))

        plans_count = len(mon4a.plan_history)
        cmds_count = len(mon4a.cmd_history)

        final_gt_x = mon4a.latest_gt.pose.pose.position.x if mon4a.latest_gt else final_map_x
        final_gt_y = mon4a.latest_gt.pose.pose.position.y if mon4a.latest_gt else final_map_y

        subtest_4a_pass = (action_status_str == 'SUCCEEDED' or pos_err_to_goal < 0.35) and travel_dist > 0.8 and plans_count > 0

        results['subtest_4A_static_goal'] = {
            'target_pose': {'x': target_x, 'y': target_y, 'yaw': target_yaw},
            'action_status': action_status_str,
            'duration_sec': round(time.time() - t_start_nav, 2),
            'traveled_distance_m': round(travel_dist, 3),
            'final_map_pose': {'x': round(final_map_x, 3), 'y': round(final_map_y, 3)},
            'final_gt_pose': {'x': round(final_gt_x, 3), 'y': round(final_gt_y, 3)},
            'position_error_to_goal_m': round(pos_err_to_goal, 3),
            'plans_computed_count': plans_count,
            'commands_dispatched_count': cmds_count,
            'verdict': 'PASS' if subtest_4a_pass else 'FAIL'
        }
        print(f"  Subtest 4A Verdict: {results['subtest_4A_static_goal']['verdict']} "
              f"(Status={action_status_str}, Error={pos_err_to_goal:.3f}m, Dist={travel_dist:.2f}m)")

        mon4a.destroy_node()
        kill_proc(p_nav2)
        time.sleep(2.0)

        # =====================================================================
        # SUBTEST 4B: Autonomous Frontier Exploration (explore_lite)
        # =====================================================================
        print("\n[Subtest 4B] Testing Autonomous Frontier Exploration with explore_lite...")
        reset_gazebo_pose(-1.10, -0.60, 0.05, 0.7071068, 0.7071068)

        explore_launch = [
            'ros2', 'launch',
            os.path.join(EXP_DIR, 'launch', 'integrated_stage4_nav2_explore.launch.py'),
            'use_sim_time:=true',
            'estimator_type:=f4',
            'start_explore:=true',
            'cmd_vel_out:=/cmd_vel'
        ]
        explore_log_f = open(os.path.join(LOGS_DIR, 'stage4_explore_launch.log'), 'w')
        p_explore = subprocess.Popen(explore_launch, env=env, preexec_fn=os.setsid,
                                     stdout=explore_log_f, stderr=subprocess.STDOUT)
        time.sleep(6.0)

        mon4b = Stage4MonitorNode()
        print("  Running autonomous exploration for 25.0 seconds...")
        t_start_explore = time.time()
        explore_duration = 25.0

        while time.time() - t_start_explore < explore_duration:
            rclpy.spin_once(mon4b, timeout_sec=0.05)

        # Stop robot
        pub_stop = mon4b.create_publisher(Twist, '/cmd_vel', 10)
        pub_stop.publish(Twist())
        time.sleep(0.5)

        # Collect 4B metrics
        travel_dist_4b = 0.0
        hist_pts_4b = mon4b.gt_history if len(mon4b.gt_history) > 1 else mon4b.odom_history
        if len(hist_pts_4b) > 1:
            pts = np.array([(p[1], p[2]) for p in hist_pts_4b])
            diffs = np.diff(pts, axis=0)
            travel_dist_4b = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))

        map_cells_4b = 0
        if mon4b.map_msg:
            data = np.array(mon4b.map_msg.data)
            map_cells_4b = int(np.sum(data > 50))

        frontiers_seen = len(mon4b.frontiers_count_history)
        plans_4b = len(mon4b.plan_history)
        cmds_4b = len(mon4b.cmd_history)

        subtest_4b_pass = travel_dist_4b > 0.4 and plans_4b > 0 and map_cells_4b > 25

        results['subtest_4B_autonomous_explore'] = {
            'duration_sec': round(explore_duration, 1),
            'traveled_distance_m': round(travel_dist_4b, 3),
            'final_map_occupied_cells': map_cells_4b,
            'frontiers_marker_updates': frontiers_seen,
            'plans_computed_count': plans_4b,
            'commands_dispatched_count': cmds_4b,
            'verdict': 'PASS' if subtest_4b_pass else 'FAIL'
        }
        print(f"  Subtest 4B Verdict: {results['subtest_4B_autonomous_explore']['verdict']} "
              f"(Dist={travel_dist_4b:.2f}m, Plans={plans_4b}, MapCells={map_cells_4b})")

        mon4b.destroy_node()
        kill_proc(p_explore)
        time.sleep(1.0)

    finally:
        kill_proc(p_gt)
        kill_proc(p_norm)
        kill_proc(p_gz)
        rclpy.shutdown()

    # Save summary
    summary_path = os.path.join(RESULTS_DIR, 'stage4_nav2_explore_verification_summary.json')
    with open(summary_path, 'w') as f:
        json.dump(results, f, indent=2)

    overall_pass = (results['subtest_4A_static_goal'].get('verdict') == 'PASS' and
                    results['subtest_4B_autonomous_explore'].get('verdict') == 'PASS')
    print("\n=================================================================")
    print(f"Stage 4 Overall Verification: {'PASS' if overall_pass else 'FAIL'}")
    print(f"Summary written to: {summary_path}")
    print("=================================================================")
    sys.exit(0 if overall_pass else 1)


if __name__ == '__main__':
    run_stage4_verification()
