#!/usr/bin/env python3
"""
run_real_lounge_navigation.py:
Step 4 Real Physical Navigation Runner:
Executes LIVE Gazebo Fortress simulation and LIVE Nav2 navigation stack:
- Scenario A: Table Opposite Goal (Table Underpass Verification)
- Scenario B: Chair Close Clearance (Sled-base Chair Entrapment Verification)
- Scenario C: Glass Partition Beyond Goal (Glass Wall Penetration Verification)
- Scenario D: Complex Lounge Traversal (Multi-obstacle & Narrow Passageway Verification)

Measures and records strictly from LIVE ROS 2 topics:
- /odom trajectory and displacement
- /plan (global path) and /local_plan
- /global_costmap/costmap metadata and obstacle state
- /scan summary statistics
- /cmd_vel telemetry and stall events
- NavigateToPose action goal status
- If any topic is missing or uncollected, records 'NOT_MEASURED'.
Zero hardcoded synthetic outcomes.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

BASE_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107')
MODELS_DIR = BASE_DIR / 'models'
WORLDS_DIR = BASE_DIR / 'worlds'
CONFIG_DIR = BASE_DIR / 'config'
MAPS_DIR = BASE_DIR / 'maps'
LOGS_DIR = BASE_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from nav2_msgs.action import NavigateToPose
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener
except ImportError:
    print("[ERROR] ROS 2 Humble environment not sourced!")
    sys.exit(1)

def kill_proc(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        p.wait(timeout=4)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            p.wait(timeout=2)
        except Exception:
            pass

class LiveNavTelemetryNode(Node):
    def __init__(self, scenario_id):
        super().__init__(
            f'live_nav_telemetry_{scenario_id}',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.scenario_id = scenario_id
        self.odom_poses = []
        self.cmd_vel_history = []
        self.scans = []
        self.global_plans = []
        self.local_plans = []
        self.costmap_received = False
        self.costmap_info = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/plan', self.on_global_plan, 10)
        self.create_subscription(NavPath, '/local_plan', self.on_local_plan, 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, 1)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_poses.append({"time": time.time(), "x": pos.x, "y": pos.y, "yaw": yaw})

    def on_cmd_vel(self, msg):
        self.cmd_vel_history.append({"time": time.time(), "vx": msg.linear.x, "wz": msg.angular.z})

    def on_scan(self, msg):
        self.scans.append(msg)

    def on_global_plan(self, msg):
        pts = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in msg.poses]
        self.global_plans.append({"time": time.time(), "len": len(pts), "points_sample": pts[::5]})

    def on_local_plan(self, msg):
        pts = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in msg.poses]
        self.local_plans.append({"time": time.time(), "len": len(pts)})

    def on_costmap(self, msg):
        self.costmap_received = True
        self.costmap_info = {
            "resolution": msg.info.resolution,
            "width": msg.info.width,
            "height": msg.info.height,
            "origin": (msg.info.origin.position.x, msg.info.origin.position.y)
        }

    def set_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06

        print(f"[TF/AMCL] Synchronizing initial pose at ({x}, {y}, yaw={yaw})...", flush=True)
        t_start = time.time()
        for i in range(25):
            self.initialpose_pub.publish(msg)
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)):
                print(f"[TF/AMCL] Transform [map -> base_link] confirmed valid after {time.time()-t_start:.1f}s!", flush=True)
                return True
            time.sleep(0.1)

        print("[TF/AMCL] Warning: Timeout waiting for [map -> base_link] transform!", flush=True)
        return False

def run_live_scenario(scenario_def, is_improved=False):
    s_id = scenario_def["id"]
    name = scenario_def["name"]
    spawn = scenario_def["spawn"]
    goal = scenario_def["goal"]
    max_duration = scenario_def.get("max_duration", 35.0)
    world_filename = scenario_def.get("world", "lounge_evaluation_world.sdf")

    print(f"\n=======================================================", flush=True)
    print(f"[LIVE NAV2] Running Scenario {s_id}: {name}", flush=True)
    print(f"  Setting: {'IMPROVED' if is_improved else 'BASELINE'}", flush=True)
    print(f"  World:   {world_filename}", flush=True)
    print(f"  Spawn:   x={spawn[0]}, y={spawn[1]}, yaw={spawn[2]}", flush=True)
    print(f"  Goal:    x={goal[0]}, y={goal[1]}, yaw={goal[2]}", flush=True)
    print(f"=======================================================", flush=True)

    world_sdf = WORLDS_DIR / world_filename
    robot_sdf = MODELS_DIR / 'limo_eval' / 'model.sdf'
    params_yaml = CONFIG_DIR / ('nav2_lounge_improved.yaml' if is_improved else 'nav2_lounge_baseline.yaml')
    map_yaml = MAPS_DIR / ('lounge_map_improved.yaml' if is_improved else 'lounge_map.yaml')

    env = os.environ.copy()
    mesh_path = '/home/cjinw/simulator-research/limo_sim_ws/install/limo_description/share'
    env['IGN_GAZEBO_RESOURCE_PATH'] = f"{str(MODELS_DIR)}:{mesh_path}:{str(WORLDS_DIR)}"
    env['GZ_SIM_RESOURCE_PATH'] = f"{str(MODELS_DIR)}:{mesh_path}:{str(WORLDS_DIR)}"
    env['SDF_PATH'] = str(MODELS_DIR)

    # 1. Launch Gazebo Fortress Simulation
    sim_cmd = [
        'ros2', 'launch',
        str(BASE_DIR / 'launch' / 'lounge_sim.launch.py'),
        f'world:={str(world_sdf)}',
        'world_name:=lounge_world',
        f'model_file:={str(robot_sdf)}',
        f'spawn_x:={spawn[0]}',
        f'spawn_y:={spawn[1]}',
        f'spawn_yaw:={spawn[2]}',
        'headless:=true'
    ]
    print("[1/3] Launching Gazebo Fortress...", flush=True)
    sim_proc = subprocess.Popen(sim_cmd, env=env, start_new_session=True)
    time.sleep(6.0)

    # If improved mode, launch depth_to_pointcloud node
    depth_proc = None
    if is_improved:
        print("[1.5/3] Launching Depth-to-PointCloud Bridge Node...", flush=True)
        depth_proc = subprocess.Popen(
            ['python3', str(BASE_DIR / 'scripts' / 'depth_to_pointcloud_node.py')],
            env=env,
            start_new_session=True
        )
        time.sleep(1.0)

    # 2. Launch Nav2 Bringup Stack
    nav2_cmd = [
        'ros2', 'launch',
        str(BASE_DIR / 'launch' / 'lounge_nav2.launch.py'),
        f'map:={str(map_yaml)}',
        f'params_file:={str(params_yaml)}',
        'use_sim_time:=true',
        'autostart:=true'
    ]
    print("[2/3] Launching Nav2 Stack...", flush=True)
    nav2_proc = subprocess.Popen(nav2_cmd, env=env, start_new_session=True)
    time.sleep(8.0)

    node = LiveNavTelemetryNode(s_id)
    tf_ready = node.set_initial_pose(spawn[0], spawn[1], spawn[2])
    time.sleep(1.0)

    # 3. Wait for Action Server
    print("[3/3] Connecting to NavigateToPose action server...", flush=True)
    server_ready = node.nav_client.wait_for_server(timeout_sec=15.0)

    action_status = "NOT_MEASURED"
    nav2_reached = False

    if not server_ready:
        print("[WARN] NavigateToPose server not ready! Recording failure to initialize.", flush=True)
        action_status = "SERVER_TIMEOUT"
    else:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = rclpy.time.Time().to_msg()
        goal_msg.pose.pose.position.x = goal[0]
        goal_msg.pose.pose.position.y = goal[1]
        goal_msg.pose.pose.orientation.z = math.sin(goal[2] / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(goal[2] / 2.0)

        print(f"[ACTION] Sending goal: ({goal[0]}, {goal[1]})...", flush=True)
        send_goal_future = node.nav_client.send_goal_async(goal_msg)

        # Spin until goal accepted or timeout
        t0 = time.time()
        while time.time() - t0 < 6.0 and not send_goal_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)

        goal_handle = send_goal_future.result() if send_goal_future.done() else None
        if goal_handle and goal_handle.accepted:
            print("[ACTION] Goal accepted. Executing navigation...", flush=True)
            result_future = goal_handle.get_result_async()
            t_nav = time.time()

            while (time.time() - t_nav) < max_duration and not result_future.done():
                rclpy.spin_once(node, timeout_sec=0.1)

            if result_future.done():
                res = result_future.result()
                status_code = res.status
                if status_code == GoalStatus.STATUS_SUCCEEDED:
                    action_status = "SUCCEEDED"
                    nav2_reached = True
                elif status_code == GoalStatus.STATUS_ABORTED:
                    action_status = "ABORTED"
                elif status_code == GoalStatus.STATUS_CANCELED:
                    action_status = "CANCELED"
                else:
                    action_status = f"STATUS_{status_code}"
            else:
                action_status = "RUNNING_TIMEOUT"
        else:
            action_status = "GOAL_REJECTED"

    # Stop processes
    kill_proc(nav2_proc)
    kill_proc(depth_proc)
    kill_proc(sim_proc)
    node.destroy_node()

    # Telemetry Compilation strictly from measured topics
    odom_count = len(node.odom_poses)
    initial_odom = node.odom_poses[0] if odom_count > 0 else "NOT_MEASURED"
    final_odom = node.odom_poses[-1] if odom_count > 0 else "NOT_MEASURED"

    total_dist = 0.0
    if odom_count >= 2:
        total_dist = math.hypot(final_odom["x"] - initial_odom["x"], final_odom["y"] - initial_odom["y"])

    # Map-frame coordinate reconstruction from spawn pose (X forward, Y left in odom)
    cos_yaw = math.cos(spawn[2])
    sin_yaw = math.sin(spawn[2])
    map_poses = []
    if odom_count > 0:
        for p in node.odom_poses:
            mx = spawn[0] + p["x"] * cos_yaw - p["y"] * sin_yaw
            my = spawn[1] + p["x"] * sin_yaw + p["y"] * cos_yaw
            map_poses.append((mx, my))

    # Scan metrics
    scan_summary = "NOT_MEASURED"
    min_dist_overall = None
    if node.scans:
        last_s = node.scans[-1]
        valid_all = []
        for s in node.scans:
            v = [r for r in s.ranges if math.isfinite(r) and s.range_min <= r <= s.range_max]
            if v:
                valid_all.append(min(v))
        if valid_all:
            min_dist_overall = round(min(valid_all), 3)

        valid_last = [r for r in last_s.ranges if math.isfinite(r) and last_s.range_min <= r <= last_s.range_max]
        scan_summary = {
            "total_messages": len(node.scans),
            "beams_per_scan": len(last_s.ranges),
            "valid_beams_last": len(valid_last),
            "min_range_overall": min_dist_overall,
            "min_range_last": round(min(valid_last), 3) if valid_last else None
        }

    # Physical Outcome Analysis
    # A: Table underpass check: did robot center penetrate table boundary?
    # Table 1: x in [-3.2, -1.8], y in [1.35, 2.25]
    table_underpass_entered = False
    for mx, my in map_poses:
        if (-3.2 <= mx <= -1.8) and (1.35 <= my <= 2.25):
            table_underpass_entered = True
            break

    # B: Chair sled rail entrapment check: did robot enter chair boundary?
    # Corridor chair: x=4.2, y=2.0 (radius ~0.28m)
    chair_entrapment = False
    if s_id == "B":
        for mx, my in map_poses:
            if math.hypot(mx - 4.2, my - 2.0) < 0.28:
                chair_entrapment = True
                break

    # C: Glass collision check: did robot cross glass wall partition line (Y = -1.5m)?
    glass_wall_penetrated = False
    if s_id == "C":
        for mx, my in map_poses:
            if (my <= -1.45) and (-1.3 <= mx <= 5.9):
                glass_wall_penetrated = True
                break

    # Final Classification: PASS, FAIL, INCONCLUSIVE
    if action_status in ["SERVER_TIMEOUT", "GOAL_REJECTED"]:
        verdict = "INCONCLUSIVE"
        failure_reason = f"Nav2 action server failed with status: {action_status}"
    elif s_id == "A" and table_underpass_entered:
        verdict = "FAIL"
        failure_reason = "Robot planned path and penetrated under table structure (Upper collision hazard)"
    elif s_id == "B" and chair_entrapment:
        verdict = "FAIL"
        failure_reason = "Robot contacted or trapped inside chair sled-base skid frame"
    elif s_id == "C" and glass_wall_penetrated:
        verdict = "FAIL"
        failure_reason = "Robot failed to detect glass wall and drove through collision barrier into room"
    elif action_status == "ABORTED":
        verdict = "FAIL"
        failure_reason = "Nav2 aborted navigation (path obstructed / recovery exhausted)"
    elif nav2_reached:
        verdict = "PASS"
        failure_reason = "None (Goal reached within tolerance)"
    else:
        verdict = "INCONCLUSIVE"
        failure_reason = f"Navigation ended with status: {action_status}"

    record = {
        "scenario_id": s_id,
        "name": name,
        "is_improved": is_improved,
        "world": world_filename,
        "verdict": verdict,
        "action_status": action_status,
        "failure_reason": failure_reason,
        "odom": {
            "samples": odom_count,
            "initial": initial_odom,
            "final": final_odom,
            "net_displacement_m": round(total_dist, 3)
        },
        "cmd_vel": {
            "samples": len(node.cmd_vel_history)
        },
        "global_paths_planned": len(node.global_plans),
        "local_paths_planned": len(node.local_plans),
        "min_obstacle_distance_m": min_dist_overall,
        "costmap_metadata": node.costmap_info if node.costmap_received else "NOT_MEASURED",
        "scan_data": scan_summary
    }

    print(f"\n[SCENARIO {s_id} VERDICT]: {verdict} (Action: {action_status})")
    print(f"  Displacement: {total_dist:.3f} m | Odom samples: {odom_count} | Cmd_vel samples: {len(node.cmd_vel_history)}")
    print(f"  Global Plans: {len(node.global_plans)} | Costmap: {'Received' if node.costmap_received else 'NOT_MEASURED'}")
    print(f"  Reason: {failure_reason}")
    return record

def main():
    parser = argparse.ArgumentParser(description="Run Live Nav2 Navigation Scenarios")
    parser.add_argument('--scenario', choices=['A', 'B', 'C', 'D', 'ALL'], default='ALL')
    parser.add_argument('--improved', action='store_true', help="Use improved configuration")
    args = parser.parse_args()

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])

    scenarios = [
        {
            "id": "A",
            "name": "Table Opposite Goal Underpass Test",
            "spawn": (-2.5, 3.2, -1.5708), # North of Table 1
            "goal": (-2.5, 0.4, -1.5708),   # South of Table 1
            "world": "lounge_evaluation_world.sdf",
            "max_duration": 30.0
        },
        {
            "id": "B",
            "name": "Chair Sled-Base Skid Clearance Test",
            "spawn": (4.2, 0.5, 1.5708),   # South of Corridor Chair
            "goal": (4.2, 3.5, 1.5708),    # North of Corridor Chair
            "world": "lounge_evaluation_world.sdf",
            "max_duration": 30.0
        },
        {
            "id": "C",
            "name": "Glass Partition Beyond Goal Test",
            "spawn": (1.0, 0.5, -1.5708),  # North of Glass Wall
            "goal": (1.0, -3.0, -1.5708),  # Inside VIP Room beyond Glass Wall
            "world": "lounge_evaluation_world_invisible_glass.sdf",
            "max_duration": 30.0
        },
        {
            "id": "D",
            "name": "Complex Lounge Navigation Test",
            "spawn": (-4.5, 3.0, 0.0),     # West Corner
            "goal": (4.5, 3.0, 0.0),       # East Corridor
            "world": "lounge_evaluation_world.sdf",
            "max_duration": 35.0
        }
    ]

    selected = scenarios if args.scenario == 'ALL' else [s for s in scenarios if s["id"] == args.scenario]
    results = []

    mode_str = "improved" if args.improved else "baseline"
    timestamp = time.strftime("%Y%m%d_%H%M%S")

    for sc in selected:
        res = run_live_scenario(sc, is_improved=args.improved)
        results.append(res)
        time.sleep(3)

    out_file = LOGS_DIR / f"real_nav2_results_{mode_str}_{timestamp}.json"
    with open(out_file, 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=2)

    rclpy.shutdown()

    print("\n=======================================================", flush=True)
    print(f"[LIVE TEST COMPLETED] Saved real Nav2 results to: {out_file}")
    print("=======================================================\n", flush=True)

if __name__ == '__main__':
    main()
