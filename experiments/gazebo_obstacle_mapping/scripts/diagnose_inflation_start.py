#!/usr/bin/env python3
"""
diagnose_inflation_start.py:
Comprehensive diagnostic script to investigate why the robot does not move
when started in the inflation zone in Gazebo.
Collects:
1. Robot center and surrounding costmap values (global and local)
2. Selected frontier and Nav2 goal
3. Path planning result (/compute_path_to_pose) from actual robot pose
4. Nav2 NavigateToPose goal acceptance, failure, cancel result
5. cmd_vel messages and actual odometry displacement
6. AutoMapping termination reason
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from limo_mapping_interfaces.action import AutoMapping

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping')
LOGS_DIR = EXP_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)
DIAG_RAW_LOG = LOGS_DIR / 'diagnose_inflation_start.raw.log'
DIAG_JSON = LOGS_DIR / 'diagnose_inflation_start.json'
WORLD_SDF = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf')


def stop_process(proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=2)
        except Exception:
            pass


class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('inflation_diagnostics_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.global_costmap = None
        self.local_costmap = None
        self.cmd_vel_msgs = []
        self.odom_start = None
        self.odom_current = None
        self.feedbacks = []
        self.action_status = None
        self.action_result = None

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_global_costmap, 1)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_local_costmap, 1)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.auto_client = ActionClient(self, AutoMapping, '/auto_mapping')

    def on_global_costmap(self, msg):
        self.global_costmap = msg

    def on_local_costmap(self, msg):
        self.local_costmap = msg

    def on_cmd_vel(self, msg):
        self.cmd_vel_msgs.append({
            'time': time.monotonic(),
            'vx': msg.linear.x,
            'vy': msg.linear.y,
            'wz': msg.angular.z
        })

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.odom_start is None:
            self.odom_start = (p.x, p.y)
        self.odom_current = (p.x, p.y)

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y)
        except Exception as e:
            return None

    def sample_costmap_window(self, costmap_msg, center_x, center_y, half_cells=4):
        if costmap_msg is None:
            return None
        info = costmap_msg.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        gx = int(round((center_x - ox) / res))
        gy = int(round((center_y - oy) / res))
        
        window = []
        for dy in range(-half_cells, half_cells + 1):
            row = []
            for dx in range(-half_cells, half_cells + 1):
                cx, cy = gx + dx, gy + dy
                if 0 <= cx < info.width and 0 <= cy < info.height:
                    val = costmap_msg.data[cy * info.width + cx]
                    row.append(val)
                else:
                    row.append(None)
            window.append(row)
        center_cost = window[half_cells][half_cells]
        return {
            'grid_coord': [gx, gy],
            'world_coord': [center_x, center_y],
            'resolution': res,
            'center_cost': center_cost,
            'window_9x9': window
        }


def run_diagnostics():
    spawn_x = -7.75
    spawn_y = 0.0
    spawn_yaw = 0.0
    print(f"=== STARTING INFLATION DIAGNOSTICS AT ({spawn_x}, {spawn_y}) ===", flush=True)

    sim_proc = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD_SDF}", f"spawn_x:={spawn_x}", f"spawn_y:={spawn_y}", f"spawn_yaw:={spawn_yaw}",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(10)

    mapping_log_path = LOGS_DIR / 'diagnose_mapping_server.log'
    map_fd = open(mapping_log_path, 'w')
    mapping_proc = subprocess.Popen([
        "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
        "use_sim_time:=true"
    ], stdout=map_fd, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(12)

    rclpy.init()
    diag = DiagnosticNode()

    # Step 1: Wait for costmaps and TF
    print("Waiting for TF and costmaps...", flush=True)
    t0 = time.monotonic()
    robot_pose = None
    while time.monotonic() - t0 < 20.0:
        rclpy.spin_once(diag, timeout_sec=0.2)
        robot_pose = diag.get_robot_pose()
        if robot_pose and diag.global_costmap and diag.local_costmap:
            break

    print(f"Robot pose in map: {robot_pose}", flush=True)
    global_window = diag.sample_costmap_window(diag.global_costmap, robot_pose[0], robot_pose[1])
    local_window = diag.sample_costmap_window(diag.local_costmap, robot_pose[0], robot_pose[1])
    print(f"Global costmap center cost: {global_window['center_cost'] if global_window else 'None'}", flush=True)
    print(f"Local costmap center cost:  {local_window['center_cost'] if local_window else 'None'}", flush=True)

    # Step 2: Test ComputePathToPose directly to a known open target
    print("Testing /compute_path_to_pose directly to (-5.0, 0.0)...", flush=True)
    path_plan_result = {'accepted': False, 'status': None, 'path_len': 0, 'error': ''}
    if diag.path_client.wait_for_server(timeout_sec=10.0):
        req = ComputePathToPose.Goal()
        req.goal = PoseStamped()
        req.goal.header.frame_id = 'map'
        req.goal.header.stamp = diag.get_clock().now().to_msg()
        req.goal.pose.position.x = -5.0
        req.goal.pose.position.y = 0.0
        req.goal.pose.orientation.w = 1.0
        req.use_start = False
        req.planner_id = 'GridBased'

        future = diag.path_client.send_goal_async(req)
        t_path = time.monotonic()
        handle = None
        while time.monotonic() - t_path < 10.0:
            rclpy.spin_once(diag, timeout_sec=0.2)
            if future.done() and handle is None:
                handle = future.result()
                path_plan_result['accepted'] = handle.accepted
                if handle.accepted:
                    res_future = handle.get_result_async()
                    while time.monotonic() - t_path < 10.0:
                        rclpy.spin_once(diag, timeout_sec=0.2)
                        if res_future.done():
                            res = res_future.result()
                            path_plan_result['status'] = res.status
                            path_plan_result['path_len'] = len(res.result.path.poses)
                            break
                break
    print(f"Direct path plan result: {path_plan_result}", flush=True)

    # Step 3: Run AutoMapping Action Goal and capture explorer interaction
    print("Sending /auto_mapping Action Goal...", flush=True)
    action_diag = {
        'goal_accepted': False,
        'terminal_status': None,
        'terminal_message': '',
        'feedbacks': []
    }
    if diag.auto_client.wait_for_server(timeout_sec=15.0):
        goal_msg = AutoMapping.Goal()
        goal_msg.output_prefix = str(EXP_DIR / 'maps/diag_inflation_run')
        goal_msg.max_duration_sec = 25.0

        def on_fb(fb_msg):
            diag.feedbacks.append({
                'phase': fb_msg.feedback.phase,
                'elapsed': fb_msg.feedback.elapsed_sec,
                'retries': fb_msg.feedback.retry_count
            })

        send_fut = diag.auto_client.send_goal_async(goal_msg, feedback_callback=on_fb)
        t_goal = time.monotonic()
        goal_h = None
        while time.monotonic() - t_goal < 25.0:
            rclpy.spin_once(diag, timeout_sec=0.2)
            if send_fut.done() and goal_h is None:
                goal_h = send_fut.result()
                action_diag['goal_accepted'] = goal_h.accepted
                if goal_h.accepted:
                    def on_res(fut):
                        r = fut.result()
                        action_diag['terminal_status'] = r.status
                        action_diag['terminal_message'] = r.result.message
                    goal_h.get_result_async().add_done_callback(on_res)
            if action_diag['terminal_status'] is not None:
                break

    dist_traveled = 0.0
    if diag.odom_start and diag.odom_current:
        dist_traveled = math.hypot(diag.odom_current[0] - diag.odom_start[0],
                                   diag.odom_current[1] - diag.odom_start[1])

    stop_process(mapping_proc)
    stop_process(sim_proc)
    map_fd.close()
    diag.destroy_node()
    rclpy.shutdown()

    # Read mapping log
    mapping_log_text = mapping_log_path.read_text()
    
    diagnostic_summary = {
        'spawn_pose': [spawn_x, spawn_y, spawn_yaw],
        'actual_robot_pose_map': robot_pose,
        'global_costmap_window': global_window,
        'local_costmap_window': local_window,
        'direct_compute_path_result': path_plan_result,
        'action_diagnostics': action_diag,
        'feedbacks_count': len(diag.feedbacks),
        'feedbacks_sample': diag.feedbacks[:10],
        'cmd_vel_count': len(diag.cmd_vel_msgs),
        'cmd_vel_samples': diag.cmd_vel_msgs[:10],
        'distance_traveled_m': dist_traveled,
        'mapping_server_log_tail': mapping_log_text.splitlines()[-50:]
    }

    with open(DIAG_JSON, 'w') as f:
        json.dump(diagnostic_summary, f, indent=2)

    with open(DIAG_RAW_LOG, 'w') as f:
        f.write(f"=== INFLATION DIAGNOSTIC RUN REPORT ===\n")
        f.write(f"Robot Map Pose: {robot_pose}\n")
        f.write(f"Global Cost Center: {global_window['center_cost'] if global_window else 'None'}\n")
        f.write(f"Local Cost Center:  {local_window['center_cost'] if local_window else 'None'}\n")
        f.write(f"Direct ComputePathToPose: {path_plan_result}\n")
        f.write(f"Action Status: {action_diag['terminal_status']}, Msg: {action_diag['terminal_message']}\n")
        f.write(f"CmdVel Msgs: {len(diag.cmd_vel_msgs)}\n")
        f.write(f"Displacement: {dist_traveled:.4f}m\n")

    print("\n=== DIAGNOSTICS COMPLETE ===", flush=True)
    print(f"Robot Map Pose: {robot_pose}", flush=True)
    print(f"Global Cost Center: {global_window['center_cost'] if global_window else 'None'}", flush=True)
    print(f"Local Cost Center:  {local_window['center_cost'] if local_window else 'None'}", flush=True)
    print(f"Direct ComputePathToPose: {path_plan_result}", flush=True)
    print(f"Action Status: {action_diag['terminal_status']}, Msg: {action_diag['terminal_message']}", flush=True)
    print(f"CmdVel Msgs: {len(diag.cmd_vel_msgs)}, Displacement: {dist_traveled:.4f}m", flush=True)


if __name__ == '__main__':
    run_diagnostics()
