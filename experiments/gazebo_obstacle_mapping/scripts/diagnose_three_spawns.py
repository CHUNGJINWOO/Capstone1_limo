#!/usr/bin/env python3
"""
diagnose_three_spawns.py:
Tests three distinct start locations relative to the west wall (at x = -7.80):
1. x = -7.75: Inscribed zone (dist = 0.05m < robot_radius 0.19m, cost == 99)
2. x = -7.58: Inflation decay zone (robot_radius 0.19m < dist = 0.22m < inflation_radius 0.30m, 0 < cost < 99)
3. x = -7.40: Free zone (dist = 0.40m > inflation_radius 0.30m, cost == 0)

Collects for each:
- Costmap values (center and surrounding)
- ComputePathToPose result (direct Navfn plan to open space)
- AutoMapping action execution: Nav2 goal, cmd_vel, feedbacks, displacement, result status
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
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import ComputePathToPose
from limo_mapping_interfaces.action import AutoMapping

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping')
LOGS_DIR = EXP_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)
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
        super().__init__('diag_node_' + uuid.uuid4().hex[:6])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.global_costmap = None
        self.cmd_vel_count = 0
        self.odom_start = None
        self.odom_current = None
        self.feedbacks = []

        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, 1)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        self.path_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.auto_client = ActionClient(self, AutoMapping, '/auto_mapping')

    def on_costmap(self, msg):
        self.global_costmap = msg

    def on_cmd_vel(self, msg):
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.cmd_vel_count += 1

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
        except Exception:
            return None


def run_single_diagnostic(spawn_x, label):
    print(f"\n=======================================================", flush=True)
    print(f"DIAGNOSTIC TEST: {label} (spawn_x={spawn_x}, wall_x=-7.80, dist={abs(spawn_x - (-7.80)):.2f}m)", flush=True)
    print(f"=======================================================", flush=True)

    sim_proc = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD_SDF}", f"spawn_x:={spawn_x}", "spawn_y:=0.0", "spawn_yaw:=0.0",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(10)

    log_path = LOGS_DIR / f'diag_{label}_mapping.log'
    log_fd = open(log_path, 'w')
    mapping_proc = subprocess.Popen([
        "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
        "use_sim_time:=true"
    ], stdout=log_fd, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(12)

    diag = DiagnosticNode()

    # Wait for costmap and TF
    t0 = time.monotonic()
    robot_pose = None
    while time.monotonic() - t0 < 15.0:
        rclpy.spin_once(diag, timeout_sec=0.2)
        robot_pose = diag.get_robot_pose()
        if robot_pose and diag.global_costmap:
            break

    # Sample cost at robot center
    center_cost = None
    if diag.global_costmap and robot_pose:
        info = diag.global_costmap.info
        gx = int(round((robot_pose[0] - info.origin.position.x) / info.resolution))
        gy = int(round((robot_pose[1] - info.origin.position.y) / info.resolution))
        if 0 <= gx < info.width and 0 <= gy < info.height:
            center_cost = diag.global_costmap.data[gy * info.width + gx]

    print(f"[{label}] Robot map pose: {robot_pose}, Cost at center: {center_cost}", flush=True)

    # Direct ComputePathToPose from robot pose to (1.5, 0.0) in front of robot in map frame
    path_plan_status = None
    path_len = 0
    if diag.path_client.wait_for_server(timeout_sec=8.0) and robot_pose:
        req = ComputePathToPose.Goal()
        req.goal = PoseStamped()
        req.goal.header.frame_id = 'map'
        req.goal.header.stamp = diag.get_clock().now().to_msg()
        req.goal.pose.position.x = 1.5
        req.goal.pose.position.y = 0.0
        req.goal.pose.orientation.w = 1.0
        req.use_start = False
        req.planner_id = 'GridBased'

        future = diag.path_client.send_goal_async(req)
        t_p = time.monotonic()
        h = None
        while time.monotonic() - t_p < 6.0:
            rclpy.spin_once(diag, timeout_sec=0.2)
            if future.done() and h is None:
                h = future.result()
                if h.accepted:
                    res_fut = h.get_result_async()
                    while time.monotonic() - t_p < 6.0:
                        rclpy.spin_once(diag, timeout_sec=0.2)
                        if res_fut.done():
                            res = res_fut.result()
                            path_plan_status = res.status
                            path_len = len(res.result.path.poses)
                            break
                break

    print(f"[{label}] ComputePathToPose result: status={path_plan_status}, path_len={path_len}", flush=True)

    # Send AutoMapping goal for 20s
    action_res_status = None
    action_res_msg = ""
    if diag.auto_client.wait_for_server(timeout_sec=10.0):
        goal = AutoMapping.Goal()
        goal.output_prefix = str(EXP_DIR / f'maps/diag_{label}')
        goal.max_duration_sec = 20.0

        def on_fb(fb):
            diag.feedbacks.append(fb.feedback.phase)

        fut = diag.auto_client.send_goal_async(goal, feedback_callback=on_fb)
        t_a = time.monotonic()
        gh = None
        while time.monotonic() - t_a < 20.0:
            rclpy.spin_once(diag, timeout_sec=0.2)
            if fut.done() and gh is None:
                gh = fut.result()
                if gh.accepted:
                    def on_r(rfut):
                        nonlocal action_res_status, action_res_msg
                        r = rfut.result()
                        action_res_status = r.status
                        action_res_msg = r.result.message
                    gh.get_result_async().add_done_callback(on_r)
            if action_res_status is not None:
                break

    dist_traveled = 0.0
    if diag.odom_start and diag.odom_current:
        dist_traveled = math.hypot(diag.odom_current[0] - diag.odom_start[0],
                                   diag.odom_current[1] - diag.odom_start[1])

    print(f"[{label}] AutoMapping Action Status: {action_res_status}, Msg: {action_res_msg}", flush=True)
    print(f"[{label}] CmdVel active msgs: {diag.cmd_vel_count}, Dist traveled: {dist_traveled:.4f}m, Feedbacks: {len(diag.feedbacks)}", flush=True)

    stop_process(mapping_proc)
    stop_process(sim_proc)
    log_fd.close()
    diag.destroy_node()

    return {
        'label': label,
        'spawn_x': spawn_x,
        'dist_to_wall_m': abs(spawn_x - (-7.80)),
        'robot_map_pose': robot_pose,
        'center_cost': center_cost,
        'compute_path_status': path_plan_status,
        'compute_path_len': path_len,
        'action_status': action_res_status,
        'action_msg': action_res_msg,
        'cmd_vel_count': diag.cmd_vel_count,
        'dist_traveled_m': dist_traveled,
        'feedbacks_count': len(diag.feedbacks)
    }


def main():
    rclpy.init()
    results = []
    
    # 1. Inscribed spawn
    r1 = run_single_diagnostic(-7.75, "inscribed_zone")
    results.append(r1)

    # 2. True Inflation spawn
    r2 = run_single_diagnostic(-7.58, "inflation_decay_zone")
    results.append(r2)

    # 3. Free spawn
    r3 = run_single_diagnostic(-7.40, "free_zone")
    results.append(r3)

    rclpy.shutdown()

    out_file = LOGS_DIR / 'diagnose_three_spawns.json'
    with open(out_file, 'w') as f:
        json.dump(results, f, indent=2)

    print("\n=======================================================", flush=True)
    print(f"THREE-SPAWN COMPARISON COMPLETE. Saved to {out_file}", flush=True)
    for r in results:
        print(f"  {r['label']} (dist={r['dist_to_wall_m']:.2f}m): Cost={r['center_cost']}, PathLen={r['compute_path_len']}, DistTraveled={r['dist_traveled_m']:.3f}m, ActionStatus={r['action_status']}", flush=True)
    print("=======================================================", flush=True)


if __name__ == '__main__':
    main()
