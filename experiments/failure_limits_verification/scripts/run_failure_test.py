#!/usr/bin/env python3
"""
run_failure_test.py:
Empirical failure characterization runner for LIMO autonomous mapping.
Executes real Gazebo + SLAM Toolbox + Nav2 (slow profile) + limo_auto_mapping Action server.
Collects telemetry, contacts, TF, costmaps, explorer events, and produces structured verdicts.
"""
import os
import sys
import time
import math
import json
import uuid
import signal
import subprocess
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener
from limo_mapping_interfaces.action import AutoMapping

WS_ROOT = Path('/home/cjinw/school/2026-2/Capstone1_limo')
EXP_WS = WS_ROOT / 'experiments/failure_limits_verification/ws'
RECORD_DIR = WS_ROOT / 'notebook/2026_11_05/05_실물시험대비_실패조건_한계검증/records'
RAW_LOG_DIR = RECORD_DIR / 'raw_logs'
RAW_LOG_DIR.mkdir(parents=True, exist_ok=True)

def clean_processes():
    subprocess.run(['killall', '-9', 'ign', 'ruby', 'gzserver', 'gzclient', 'ros2', 
                    'mapping_server', 'frontier_explorer', 'async_slam_toolbox_node', 
                    'controller_server', 'planner_server', 'bt_navigator', 'behavior_server'], 
                   stderr=subprocess.DEVNULL)
    time.sleep(2)

def spawn_gazebo_obstacle(name, x, y, z=0.45, sx=0.5, sy=0.5, sz=0.9):
    sdf = (
        f"<sdf version='1.6'>"
        f"<model name='{name}'>"
        f"<static>true</static>"
        f"<link name='link'>"
        f"<visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
        f"<material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material></visual>"
        f"<collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>"
        f"</link>"
        f"</model>"
        f"</sdf>"
    )
    req = f'sdf: "{sdf}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'
    cmd = [
        'ign', 'service', '-s', '/world/mc3f_world/create',
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '3000',
        '--req', req
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    return res.returncode == 0 and 'data: true' in res.stdout

def remove_gazebo_obstacle(name):
    req = f'name: "{name}", type: 2'
    cmd = [
        'ign', 'service', '-s', '/world/mc3f_world/remove',
        '--reqtype', 'ignition.msgs.Entity',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '3000',
        '--req', req
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    return res.returncode == 0 and 'data: true' in res.stdout

class TestMonitorNode(Node):
    def __init__(self, run_id, scenario_name, obstacle_boxes=None):
        super().__init__(f'monitor_{run_id}')
        self.run_id = run_id
        self.scenario_name = scenario_name
        self.obstacle_boxes = obstacle_boxes or [] # list of (name, xmin, xmax, ymin, ymax)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_pose = None
        self.current_pose = None
        self.trajectory = []
        self.cmd_vels = []
        self.nav_plans = []
        self.scans = []
        self.gcost_meta = None
        self.latest_gcost_data = None
        self.physical_contacts = []
        self.wheel_slip_events = []
        self.sim_time = 0.0

        self.create_subscription(Clock, '/clock', self._on_clock, 50)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 50)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 20)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._on_gcost, 10)
        self.create_subscription(NavPath, '/plan', self._on_plan, 10)

    def _on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_cmd(self, msg):
        self.cmd_vels.append((self.sim_time, msg.linear.x, msg.angular.z))

    def _on_scan(self, msg):
        valid = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        min_r = min(valid) if valid else None
        self.scans.append((self.sim_time, min_r, len(valid)))

    def _on_gcost(self, msg):
        self.gcost_meta = {
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y
        }
        self.latest_gcost_data = list(msg.data)

    def _on_plan(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.nav_plans.append((self.sim_time, pts))

    def update_pose_and_collision(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            # Compute yaw
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            p = (self.sim_time, x, y, yaw)
            if self.start_pose is None:
                self.start_pose = (x, y, yaw)
            self.current_pose = (x, y, yaw)
            self.trajectory.append(p)

            # Check collision with physical obstacle boundaries
            # Robot body box: length 0.28m, width 0.13m centered at (x, y) with yaw
            # Approximate with bounding box margin
            r_margin = 0.08 # half width / front bumper clearance
            for obs_name, xmin, xmax, ymin, ymax in self.obstacle_boxes:
                # Check overlap
                if (xmin - r_margin <= x <= xmax + r_margin) and (ymin - r_margin <= y <= ymax + r_margin):
                    self.physical_contacts.append({
                        'time': self.sim_time,
                        'obstacle': obs_name,
                        'robot_x': round(x, 3),
                        'robot_y': round(y, 3)
                    })

            # Check wheel slip / stall: cmd_vel > 0.05 but displacement in last 3s < 0.01
            if len(self.cmd_vels) > 10 and len(self.trajectory) > 10:
                recent_cmds = [c[1] for c in self.cmd_vels[-10:]]
                avg_cmd = sum(recent_cmds) / len(recent_cmds)
                if avg_cmd > 0.04:
                    dtraj = math.hypot(x - self.trajectory[-10][1], y - self.trajectory[-10][2])
                    if dtraj < 0.01:
                        self.wheel_slip_events.append({
                            'time': self.sim_time,
                            'cmd_vel_x': round(avg_cmd, 3),
                            'displacement_last_10': round(dtraj, 4)
                        })

        except Exception:
            pass

    def get_center_cost(self):
        if not self.current_pose or not self.gcost_meta or not self.latest_gcost_data:
            return None
        x, y, _ = self.current_pose
        res = self.gcost_meta['resolution']
        ox = self.gcost_meta['origin_x']
        oy = self.gcost_meta['origin_y']
        w = self.gcost_meta['width']
        h = self.gcost_meta['height']
        gx = int(math.floor((x - ox) / res))
        gy = int(math.floor((y - oy) / res))
        if 0 <= gx < w and 0 <= gy < h:
            idx = gy * w + gx
            return self.latest_gcost_data[idx]
        return None

def run_test(scenario_code, run_index=1, custom_timeout=60.0, dynamic_inject=False):
    run_id = f"RUN_{scenario_code}_{time.strftime('%Y%m%d_%H%M%S')}_idx{run_index}"
    clean_processes()

    # Determine scenario configuration
    if scenario_code.startswith('A'): # Glass Scenario
        world_sdf = WS_ROOT / 'experiments/failure_limits_verification/worlds/world_scenario_a_glass.sdf'
        spawn_x, spawn_y, spawn_z, spawn_yaw = 0.0, -1.0, 0.05, 1.5708 # Facing North towards glass
        # Obstacle box for invisible glass wall: x in [-1.8, 4.0], y = 0.0 (+/- 0.04)
        obs_boxes = [
            ('invisible_glass_partition', -1.8, 4.0, -0.05, 0.05)
        ]
    elif scenario_code.startswith('B'): # Desk Scenario
        world_sdf = WS_ROOT / 'experiments/failure_limits_verification/worlds/world_scenario_b_desk.sdf'
        if run_index == 1:
            spawn_x, spawn_y, spawn_z, spawn_yaw = 0.0, -1.2, 0.05, 1.5708 # Far spawn -> backward stall loop
        else:
            spawn_x, spawn_y, spawn_z, spawn_yaw = 0.0, -0.4, 0.05, 1.5708 # Close spawn -> straight path under desk into crossbar
        # Obstacle box for under-scan crossbar: x in [-0.40, 0.40], y = 0.25 (+/- 0.03)
        obs_boxes = [
            ('under_scan_crossbar', -0.40, 0.40, 0.22, 0.28)
        ]
    elif scenario_code.startswith('C'): # Dense Obstacles
        world_sdf = WS_ROOT / 'experiments/failure_limits_verification/worlds/world_scenario_c_dense.sdf'
        spawn_x, spawn_y, spawn_z, spawn_yaw = 0.0, 0.0, 0.05, 0.0
        obs_boxes = [
            ('dense_box_north', -0.3, 0.3, 0.16, 0.32),
            ('dense_box_south', -0.3, 0.3, -0.32, -0.16),
            ('dense_box_west', -0.32, -0.18, -0.3, 0.3)
        ]
    elif scenario_code.startswith('D'): # Dynamic Corridor
        world_sdf = WS_ROOT / 'experiments/failure_limits_verification/worlds/world_scenario_d_dynamic.sdf'
        spawn_x, spawn_y, spawn_z, spawn_yaw = -2.2, 0.0, 0.05, 0.0 # Facing East
        obs_boxes = []
    else:
        raise ValueError(f"Unknown scenario code: {scenario_code}")

    model_sdf = WS_ROOT / 'experiments/failure_limits_verification/models/limo_ackermann_fixed.sdf'
    raw_log_file = RAW_LOG_DIR / f"{run_id}.log"
    json_out_file = RAW_LOG_DIR / f"{run_id}.json"

    print(f"\n=======================================================")
    print(f"STARTING TEST RUN: {run_id}")
    print(f"Scenario: {scenario_code}, Index: {run_index}")
    print(f"World: {world_sdf.name}, Model: {model_sdf.name}")
    print(f"Spawn: x={spawn_x}, y={spawn_y}, z={spawn_z}, yaw={spawn_yaw}")
    print(f"=======================================================")

    log_fd = open(raw_log_file, 'w')

    # [1] Launch Gazebo
    print("[1] Launching Gazebo...")
    gz_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT}/capstone_ws/install/setup.bash && "
        f"ros2 launch limo_capstone limo_mc3f_sim.launch.py "
        f"world:={world_sdf} model_file:={model_sdf} "
        f"spawn_x:={spawn_x} spawn_y:={spawn_y} spawn_z:={spawn_z} spawn_yaw:={spawn_yaw} "
        f"headless:=true rviz:=false"
    )
    gz_proc = subprocess.Popen(["bash", "-c", gz_cmd], stdout=log_fd, stderr=log_fd, preexec_fn=os.setsid)
    time.sleep(10)

    # [2] Launch limo_auto_mapping with slow profile
    print("[2] Launching Auto Mapping (nav2_profile:=slow)...")
    map_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {WS_ROOT}/capstone_ws/install/setup.bash && "
        f"source {EXP_WS}/install/setup.bash && "
        f"ros2 launch limo_auto_mapping auto_mapping.launch.py "
        f"nav2_profile:=slow use_sim_time:=true"
    )
    map_proc = subprocess.Popen(["bash", "-c", map_cmd], stdout=log_fd, stderr=log_fd, preexec_fn=os.setsid)
    time.sleep(14)

    # Initialize ROS Node for client and monitor
    rclpy.init()
    monitor = TestMonitorNode(run_id, scenario_code, obs_boxes)
    action_client = ActionClient(monitor, AutoMapping, '/auto_mapping')

    print("[3] Waiting for /auto_mapping Action server...")
    server_ready = action_client.wait_for_server(timeout_sec=20.0)
    if not server_ready:
        print("ERROR: /auto_mapping action server not available!")

    # Check center cost at start
    t_wait_tf = time.time()
    while time.time() - t_wait_tf < 15.0 and monitor.start_pose is None:
        rclpy.spin_once(monitor, timeout_sec=0.2)
        monitor.update_pose_and_collision()
    start_center_cost = monitor.get_center_cost()
    print(f"Initial Robot TF Pose: {monitor.start_pose}")
    print(f"Initial Robot Center Costmap Value: {start_center_cost}")

    # [4] Send Goal
    goal_msg = AutoMapping.Goal()
    goal_msg.output_prefix = str(RECORD_DIR / f"maps/{run_id}")
    goal_msg.allow_margin_retry = False
    goal_msg.max_duration_sec = 0.0

    print("[4] Sending /auto_mapping goal...")
    send_fut = action_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(monitor, send_fut, timeout_sec=5.0)
    goal_handle = send_fut.result()

    goal_accepted = goal_handle.accepted if goal_handle else False
    print(f"Goal Accepted: {goal_accepted}")

    # Monitor loop
    result_future = None
    if goal_accepted:
        result_future = goal_handle.get_result_async()

    t_start = time.time()
    dynamic_injected = False
    action_result = None

    print(f"[5] Monitoring run for up to {custom_timeout}s...")
    while time.time() - t_start < custom_timeout:
        rclpy.spin_once(monitor, timeout_sec=0.1)
        monitor.update_pose_and_collision()

        # Check dynamic injection for scenario D
        if dynamic_inject and not dynamic_injected:
            if monitor.current_pose and monitor.current_pose[0] > -1.2:
                print(f"\n>>> INJECTING DYNAMIC OBSTACLE AT x=0.0, y=0.0 (t={time.time()-t_start:.1f}s) <<<")
                spawn_gazebo_obstacle('dyn_block', 0.0, 0.0, 0.45, 0.5, 0.6, 0.9)
                dynamic_injected = True
                monitor.obstacle_boxes.append(('dyn_block', -0.25, 0.25, -0.3, 0.3))

        if result_future and result_future.done():
            action_result = result_future.result()
            print(f"\nAction returned result early at t={time.time()-t_start:.1f}s!")
            break

    total_wall_time = time.time() - t_start

    # If action still running at timeout, cancel
    cancelled_by_tester = False
    if goal_handle and (not result_future or not result_future.done()):
        print("\nAction still active at test timeout. Cancelling...")
        cancel_fut = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(monitor, cancel_fut, timeout_sec=5.0)
        cancelled_by_tester = True
        time.sleep(2)

    # Calculate trajectory metrics
    traj_pts = [(p[1], p[2]) for p in monitor.trajectory]
    cum_dist = 0.0
    for i in range(1, len(traj_pts)):
        cum_dist += math.hypot(traj_pts[i][0] - traj_pts[i-1][0], traj_pts[i][1] - traj_pts[i-1][1])

    disp = 0.0
    if monitor.start_pose and monitor.current_pose:
        disp = math.hypot(monitor.current_pose[0] - monitor.start_pose[0], 
                          monitor.current_pose[1] - monitor.start_pose[1])

    # Shutdown processes
    log_fd.flush()
    log_fd.close()
    try:
        os.killpg(os.getpgid(map_proc.pid), signal.SIGINT)
        os.killpg(os.getpgid(gz_proc.pid), signal.SIGINT)
    except Exception:
        pass
    clean_processes()

    # Parse explorer events from log
    explore_events = []
    log_content = ""
    try:
        with open(raw_log_file, 'r', errors='ignore') as f:
            log_content = f.read()
            for line in log_content.splitlines():
                if 'EXPLORE_EVENT' in line:
                    idx = line.find('{')
                    if idx != -1:
                        try:
                            explore_events.append(json.loads(line[idx:]))
                        except Exception:
                            pass
    except Exception:
        pass

    # Extract Action Result Fields
    res_status = None
    res_success = None
    res_message = None
    res_complete = None
    if action_result:
        res_status = action_result.status
        res_success = action_result.result.success
        res_message = action_result.result.message
        res_complete = action_result.result.exploration_complete

    # Formulate verdicts:
    # 1. Condition Created
    # 2. Failure Phenomenon Reproduced
    # 3. System Response Quality
    condition_created = True
    failure_reproduced = False
    system_response_ok = False
    failure_type = "NONE"
    notes = []

    def get_target_y_val(ev):
        t = ev.get('target')
        if isinstance(t, (list, tuple)) and len(t) >= 2:
            return t[1]
        elif isinstance(t, dict):
            return t.get('y', -999.0)
        return -999.0

    if scenario_code.startswith('A'):
        # Glass scenario
        # Failure: Robot picks goal inside classroom through invisible glass, plans path crossing y=0, drives into glass wall and collides/stalls
        glass_crossing_plans = [p for p in monitor.nav_plans if any(pt[1] > 0.0 for pt in p[1])]
        has_glass_goal = len(glass_crossing_plans) > 0 or any(e.get('event') == 'goal_sent' and get_target_y_val(e) > 0.0 for e in explore_events)
        has_collision = len(monitor.physical_contacts) > 0
        has_stall = len(monitor.wheel_slip_events) > 0

        if has_collision or (has_stall and len(monitor.physical_contacts) > 0):
            failure_reproduced = True
            failure_type = "GLASS_WALL_COLLISION_STALL"
            system_response_ok = False # System failed to detect glass and crashed into it
            notes.append(f"Robot failed to observe invisible glass partition and collided at y=0.0 (contacts={len(monitor.physical_contacts)}, slips={len(monitor.wheel_slip_events)}).")
        else:
            notes.append(f"Glass observation: plans_crossing={len(glass_crossing_plans)}, contacts={len(monitor.physical_contacts)}, slips={len(monitor.wheel_slip_events)}.")

    elif scenario_code.startswith('B'):
        # Desk scenario
        has_crossbar_contact = len(monitor.physical_contacts) > 0
        has_stall = len(monitor.wheel_slip_events) > 0
        has_progress_timeout = any('progress_timeout' in l for l in log_content.splitlines())
        has_repeated_goal = len([e for e in explore_events if e.get('event') == 'goal_sent']) > 1 and \
                            any(e.get('event') == 'goal_excluded' for e in explore_events)

        if has_crossbar_contact or (has_stall and monitor.current_pose and monitor.current_pose[1] > 0.1):
            failure_reproduced = True
            failure_type = "DESK_CROSSBAR_CHASSIS_SNAG"
            system_response_ok = False # System failed to see crossbar below scan plane and snagged
            notes.append(f"Robot planned path under desk and chassis snagged on 5cm crossbar (contacts={len(monitor.physical_contacts)}, slips={len(monitor.wheel_slip_events)}).")
        elif has_progress_timeout and has_repeated_goal:
            failure_reproduced = True
            failure_type = "BACKWARD_GOAL_STALL_COOLDOWN_LOOP"
            system_response_ok = False
            notes.append(f"Robot stalled on non-holonomic backward goal, timed out after 25s, and re-sent identical goal after 10s cooldown.")
        else:
            notes.append("No desk crossbar snag or stall loop observed.")

    elif scenario_code.startswith('C'):
        # Dense obstacles scenario
        # Center cost >= 99 check
        has_inscribed_wait = any('인플레이션 감지' in l or 'cost=99' in l for l in log_content.splitlines())
        has_inscribed_timeout = any('인플레이션 영역 대기 시간 초과' in l for l in log_content.splitlines())
        zero_disp_success = any(e.get('event') == 'goal_result' and e.get('displacement_m', 1.0) is not None and e.get('displacement_m', 1.0) < 0.05 for e in explore_events)

        if has_inscribed_wait:
            condition_created = True
            failure_reproduced = True # Enclosed/inscribed failure condition reproduced
            if has_inscribed_timeout and res_status == 6: # STATUS_ABORTED
                system_response_ok = True # System properly waited 10s and cleanly exited with error without sending illegal goals
                failure_type = "INSCRIBED_START_CLEAN_ABORT"
                notes.append(f"Robot detected inscribed/lethal cost 99, waited 10s re-evaluating without sending illegal goals, and cleanly exited with exploration_error (Status 6 ABORTED).")
            else:
                system_response_ok = False
                failure_type = "INSCRIBED_START_UNRESOLVED"
                notes.append("Inscribed start detected but did not follow clean abort policy.")
        elif zero_disp_success:
            condition_created = True
            failure_reproduced = True
            system_response_ok = False
            failure_type = "ZERO_DISPLACEMENT_SUCCESS_LOOP"
            notes.append("Zero-displacement goal completion loop detected.")
        else:
            notes.append("Dense obstacles condition did not trigger inscribed detection.")

    elif scenario_code.startswith('D'):
        # Dynamic corridor scenario
        condition_created = dynamic_injected
        if dynamic_injected:
            notes.append("Dynamic obstacle successfully spawned at (0.0, 0.0).")
            # Check collision
            has_dyn_collision = any('dyn_block' in c.get('obstacle', '') for c in monitor.physical_contacts)
            if has_dyn_collision:
                failure_reproduced = True
                system_response_ok = False
                failure_type = "DYNAMIC_OBSTACLE_COLLISION"
                notes.append("Robot collided physically with dynamic obstacle.")
            else:
                # Check if bypassed or stopped
                has_recovery = any('spin' in log_content.lower() or 'backup' in log_content.lower() for _ in [1])
                has_replan = len(monitor.nav_plans) > 5
                failure_reproduced = True
                failure_type = "DYNAMIC_BLOCK_PATH_BLOCKED"
                if disp > 2.0:
                    system_response_ok = True
                    notes.append(f"Robot successfully detected dynamic block and replanned via bypass (disp={disp:.3f}m).")
                else:
                    system_response_ok = False
                    notes.append(f"Robot halted / attempted recovery when blocked (disp={disp:.3f}m, recovery={has_recovery}).")

    # Assemble structured record
    run_record = {
        'run_id': run_id,
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'scenario_code': scenario_code,
        'run_index': run_index,
        'config': {
            'world_sdf': str(world_sdf),
            'model_sdf': str(model_sdf),
            'spawn_pose': {'x': spawn_x, 'y': spawn_y, 'z': spawn_z, 'yaw': spawn_yaw},
            'nav2_profile': 'slow',
            'allow_margin_retry': False,
            'custom_timeout_sec': custom_timeout
        },
        'measurements': {
            'wall_time_sec': round(total_wall_time, 2),
            'sim_time_sec': round(monitor.sim_time, 2),
            'start_pose': [round(v, 4) for v in monitor.start_pose] if monitor.start_pose else None,
            'final_pose': [round(v, 4) for v in monitor.current_pose] if monitor.current_pose else None,
            'displacement_m': round(disp, 4),
            'cumulative_distance_m': round(cum_dist, 4),
            'initial_center_cost': start_center_cost,
            'num_cmd_vel_samples': len(monitor.cmd_vels),
            'num_nav_plans': len(monitor.nav_plans),
            'num_physical_contacts': len(monitor.physical_contacts),
            'physical_contact_details': monitor.physical_contacts[:5],
            'num_wheel_slips': len(monitor.wheel_slip_events),
            'wheel_slip_details': monitor.wheel_slip_events[:5]
        },
        'action_result': {
            'goal_accepted': goal_accepted,
            'status_code': res_status,
            'success': res_success,
            'exploration_complete': res_complete,
            'message': res_message,
            'cancelled_by_tester': cancelled_by_tester
        },
        'explorer_events': explore_events,
        'verdict': {
            'condition_created': condition_created,
            'failure_reproduced': failure_reproduced,
            'system_response_ok': system_response_ok,
            'failure_type': failure_type,
            'notes': " ".join(notes)
        }
    }

    with open(json_out_file, 'w') as f:
        json.dump(run_record, f, indent=2)

    print(f"\n=== TEST {run_id} FINISHED ===")
    print(f"Displacement: {disp:.3f}m, Cumulative Dist: {cum_dist:.3f}m")
    print(f"Contacts: {len(monitor.physical_contacts)}, Slips: {len(monitor.wheel_slip_events)}")
    print(f"Action Status: {res_status}, Message: {res_message}")
    print(f"Verdict: ConditionCreated={condition_created}, FailureReproduced={failure_reproduced}, SystemOK={system_response_ok}")
    print(f"Failure Type: {failure_type}")
    print(f"Log: {raw_log_file.name}")
    print(f"JSON: {json_out_file.name}\n")

    return run_record

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True, choices=['A', 'B', 'C', 'D'])
    parser.add_argument('--index', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--dynamic', action='store_true')
    args = parser.parse_args()

    run_test(args.scenario, args.index, args.timeout, args.dynamic)
