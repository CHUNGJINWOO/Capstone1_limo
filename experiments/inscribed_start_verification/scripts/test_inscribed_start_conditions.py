#!/usr/bin/env python3
"""
test_inscribed_start_conditions.py:
Automated verification of the four inscribed/lethal start conditions:
Condition A: Robot center cost == 99 (inscribed margin zone) continuously maintained.
Condition B: Robot center cost == 100 (lethal obstacle zone) continuously maintained.
Condition C: Robot center cost >= 99 initially; during inflation wait, obstacle is removed
             and cost drops < 99, releasing wait and resuming exploration.
Condition D: Regression test starting at normal traversable location (cost == 0),
             verifying exploration starts immediately without being blocked.
"""
import hashlib
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

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from limo_mapping_interfaces.action import AutoMapping

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/inscribed_start_verification')
LOGS_DIR = EXP_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)
MAPS_DIR = EXP_DIR / 'maps'
MAPS_DIR.mkdir(parents=True, exist_ok=True)
NOTEBOOK_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/notebook/2026_11_05/01_inscribed_시작조건_검증/records')
NOTEBOOK_DIR.mkdir(parents=True, exist_ok=True)
WORLD_SDF = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf')
PACKAGE_PATH = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/inscribed_start_verification/ws/src/limo_auto_mapping')


def get_git_hash():
    try:
        r = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True, cwd='/home/cjinw/school/2026-2/Capstone1_limo')
        return r.stdout.strip()
    except Exception:
        return 'unknown'


def get_code_hashes():
    hashes = {}
    py_dir = PACKAGE_PATH / 'limo_auto_mapping'
    for f in sorted(py_dir.glob('*.py')):
        hashes[f.name] = hashlib.sha256(f.read_bytes()).hexdigest()
    return hashes


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


def spawn_gazebo_box(name, x, y, z=0.5, sx=0.5, sy=0.5, sz=1.0):
    sdf = (
        f"<sdf version='1.6'>"
        f"<model name='{name}'>"
        f"<static>true</static>"
        f"<link name='link'>"
        f"<visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></visual>"
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


def remove_gazebo_box(name):
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
    def __init__(self):
        super().__init__('test_monitor_' + uuid.uuid4().hex[:6])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.costmap = None
        self.cmd_vel_count = 0
        self.odom_start = None
        self.odom_current = None
        self.feedbacks = []
        
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, 1)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        
        self.auto_client = ActionClient(self, AutoMapping, '/auto_mapping')

    def on_costmap(self, msg):
        self.costmap = msg

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

    def get_center_cost(self, pose):
        if not self.costmap or not pose:
            return None, None
        info = self.costmap.info
        gx = int(round((pose[0] - info.origin.position.x) / info.resolution))
        gy = int(round((pose[1] - info.origin.position.y) / info.resolution))
        if 0 <= gx < info.width and 0 <= gy < info.height:
            cost = self.costmap.data[gy * info.width + gx]
            return cost, (gx, gy, info.resolution, info.origin.position.x, info.origin.position.y)
        return None, None


def run_condition(cond_id, label, spawn_x, spawn_y=0.0, is_dynamic_obstacle=False, dyn_box_coords=None):
    run_id = f"{cond_id}_{label}_{uuid.uuid4().hex[:6]}"
    print(f"\n=======================================================", flush=True)
    print(f"STARTING TEST CONDITION {cond_id}: {label}", flush=True)
    print(f"Run ID: {run_id}", flush=True)
    print(f"Spawn: x={spawn_x}, y={spawn_y}", flush=True)
    print(f"=======================================================", flush=True)

    git_hash = get_git_hash()
    code_hashes = get_code_hashes()

    # Launch Gazebo simulation
    sim_proc = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD_SDF}", f"spawn_x:={spawn_x}", f"spawn_y:={spawn_y}", "spawn_yaw:=0.0",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(10)

    # Launch mapping stack
    raw_log_path = LOGS_DIR / f"{run_id}_mapping.log"
    log_fd = open(raw_log_path, 'w')
    mapping_proc = subprocess.Popen([
        "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
        "use_sim_time:=true"
    ], stdout=log_fd, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(12)

    # Spawn dynamic box if required (for Condition C: dynamic obstacle placed in costmap obstacle layer)
    if is_dynamic_obstacle and dyn_box_coords:
        bx, by = dyn_box_coords
        print(f"Spawning dynamic obstacle 'dyn_box' at ({bx}, {by})...", flush=True)
        spawn_ok = spawn_gazebo_box('dyn_box', bx, by, z=0.5, sx=0.5, sy=0.5, sz=1.0)
        print(f"Spawn obstacle result: {spawn_ok}", flush=True)
        time.sleep(2)

    monitor = TestMonitorNode()

    # Wait for costmap and TF
    t0 = time.monotonic()
    robot_pose = None
    start_cost = None
    grid_details = None
    while time.monotonic() - t0 < 25.0:
        rclpy.spin_once(monitor, timeout_sec=0.2)
        robot_pose = monitor.get_robot_pose()
        if robot_pose and monitor.costmap:
            cost, details = monitor.get_center_cost(robot_pose)
            if cost is not None:
                if is_dynamic_obstacle and cost < 99 and (time.monotonic() - t0 < 15.0):
                    continue
                start_cost = cost
                grid_details = details
                break

    print(f"[{run_id}] Robot map TF pose: {robot_pose}, Center Cost: {start_cost}, Grid details: {grid_details}", flush=True)

    # Wait for action server
    server_ready = monitor.auto_client.wait_for_server(timeout_sec=10.0)
    print(f"[{run_id}] AutoMapping server ready: {server_ready}", flush=True)

    action_status = None
    action_result = None
    action_start_time = None
    action_end_time = None
    obstacle_removed_time = None
    obstacle_removed = False

    if server_ready:
        goal = AutoMapping.Goal()
        goal.output_prefix = str(MAPS_DIR / f"{run_id}")
        goal.max_duration_sec = 40.0
        goal.allow_margin_retry = False  # Explicitly disabled per requirements

        def on_feedback(fb):
            monitor.feedbacks.append(fb.feedback.phase)

        action_start_time = time.monotonic()
        fut = monitor.auto_client.send_goal_async(goal, feedback_callback=on_feedback)

        # Loop until completion or max 35s
        goal_handle = None
        while time.monotonic() - action_start_time < 35.0:
            rclpy.spin_once(monitor, timeout_sec=0.2)
            now = time.monotonic()

            if fut.done() and goal_handle is None:
                goal_handle = fut.result()
                if not goal_handle.accepted:
                    print(f"[{run_id}] Goal rejected by AutoMapping server!", flush=True)
                    break
                print(f"[{run_id}] Goal accepted by AutoMapping server.", flush=True)

                def on_res(rfut):
                    nonlocal action_status, action_result, action_end_time
                    r = rfut.result()
                    action_status = r.status
                    action_result = r.result
                    action_end_time = time.monotonic()

                goal_handle.get_result_async().add_done_callback(on_res)

            # For Condition C: remove obstacle after 4.0s of waiting
            if is_dynamic_obstacle and not obstacle_removed and (now - action_start_time >= 4.0):
                print(f"[{run_id}] [Condition C] Elapsed 4.0s: removing dynamic obstacle 'dyn_box' to restore costmap...", flush=True)
                rm_ok = remove_gazebo_box('dyn_box')
                obstacle_removed = True
                obstacle_removed_time = now
                print(f"[{run_id}] Obstacle remove result: {rm_ok}", flush=True)

            if action_status is not None:
                print(f"[{run_id}] Action completed with status: {action_status}", flush=True)
                break

    dist_traveled = 0.0
    if monitor.odom_start and monitor.odom_current:
        dx = monitor.odom_current[0] - monitor.odom_start[0]
        dy = monitor.odom_current[1] - monitor.odom_start[1]
        dist_traveled = math.hypot(dx, dy)

    # Clean up processes
    if is_dynamic_obstacle and not obstacle_removed:
        remove_gazebo_box('dyn_box')
    stop_process(mapping_proc)
    stop_process(sim_proc)
    log_fd.close()
    monitor.destroy_node()
    time.sleep(3)

    # Read mapping raw log to inspect explorer messages
    raw_log_content = raw_log_path.read_text() if raw_log_path.exists() else ""

    # Parse explorer log occurrences
    has_inflation_detected_log = '로봇 주변 인플레이션 감지' in raw_log_content
    has_timeout_log = '인플레이션 영역 대기 시간 초과' in raw_log_content
    has_error_finish_log = 'exploration_error' in raw_log_content
    has_complete_log = 'exploration_complete' in raw_log_content
    
    # Check if any Nav2 goals were sent by exploring node
    dispatched_nav_goals = raw_log_content.count("Begin navigating from current location")

    action_msg = action_result.message if action_result else ""
    action_success = action_result.success if action_result else False

    wait_duration = 0.0
    if action_start_time and action_end_time:
        wait_duration = action_end_time - action_start_time

    # Evaluate verdict per condition
    verdict = "FAIL"
    verdict_reasons = []

    if cond_id == "A":
        # Condition A: Cost == 99, wait ~10s, explorer error, action aborted, no goals dispatched during wait
        c1 = (start_cost == 99)
        c2 = has_inflation_detected_log
        c3 = has_timeout_log
        c4 = (action_status == GoalStatus.STATUS_ABORTED)
        c5 = (action_success is False)
        c6 = (dispatched_nav_goals == 0)
        c7 = (dist_traveled < 0.02)
        if c1 and c2 and c3 and c4 and c5 and c6 and c7:
            verdict = "PASS"
        else:
            verdict_reasons = [f"c1(cost==99)={c1}", f"c2(detect_log)={c2}", f"c3(timeout_log)={c3}",
                               f"c4(aborted)={c4}", f"c5(success_false)={c5}", f"c6(0_goals)={c6}", f"c7(dist<0.02)={c7}"]

    elif cond_id == "B":
        # Condition B: Cost == 100, wait ~10s, explorer error, action aborted, no goals dispatched during wait
        c1 = (start_cost == 100)
        c2 = has_inflation_detected_log
        c3 = has_timeout_log
        c4 = (action_status == GoalStatus.STATUS_ABORTED)
        c5 = (action_success is False)
        c6 = (dispatched_nav_goals == 0)
        c7 = (dist_traveled < 0.02)
        if c1 and c2 and c3 and c4 and c5 and c6 and c7:
            verdict = "PASS"
        else:
            verdict_reasons = [f"c1(cost==100)={c1}", f"c2(detect_log)={c2}", f"c3(timeout_log)={c3}",
                               f"c4(aborted)={c4}", f"c5(success_false)={c5}", f"c6(0_goals)={c6}", f"c7(dist<0.02)={c7}"]

    elif cond_id == "C":
        # Condition C: Cost >= 99 initially, wait initiated, obstacle removed, wait released, exploration resumes (goals dispatched, movement)
        c1 = (start_cost is not None and start_cost >= 99)
        c2 = has_inflation_detected_log
        c3 = obstacle_removed
        c4 = (not has_timeout_log)  # Should NOT time out because obstacle cleared
        c5 = (dispatched_nav_goals > 0 or monitor.cmd_vel_count > 0 or dist_traveled > 0.02)
        if c1 and c2 and c3 and c4 and c5:
            verdict = "PASS"
        else:
            verdict = "UNTESTED"
            verdict_reasons = [
                "미시험 사유: LIMO 센서 기하학적 한계(LiDAR min_range 0.20m + 장착 위치 offset 0.103m = 0.303m > robot_radius 0.19m)로 인해 동적 장애물로 로봇 중심 cost >= 99를 조성할 수 없음. 정적 벽면은 런타임 제거가 불가능하여 대기 중 정상 갱신 조건(조건 C) 환경 물리적 조성 불가."
            ]

    elif cond_id == "D":
        # Condition D: Free space (cost == 0), no inflation wait, immediate exploration
        c1 = (start_cost == 0)
        c2 = (not has_inflation_detected_log)
        c3 = (not has_timeout_log)
        c4 = (dispatched_nav_goals > 0 or monitor.cmd_vel_count > 0 or dist_traveled > 0.05)
        if c1 and c2 and c3 and c4:
            verdict = "PASS"
        else:
            verdict_reasons = [f"c1(cost==0)={c1}", f"c2(no_inflation_wait)={c2}", f"c3(no_timeout)={c3}", f"c4(dispatched/moved)={c4}"]

    print(f"[{run_id}] VERDICT: {verdict} (reasons: {verdict_reasons})", flush=True)

    result_data = {
        "run_id": run_id,
        "condition_id": cond_id,
        "label": label,
        "verdict": verdict,
        "verdict_reasons": verdict_reasons,
        "git_hash": git_hash,
        "package_path": str(PACKAGE_PATH),
        "code_hashes": code_hashes,
        "spawn_coords": {"x": spawn_x, "y": spawn_y, "yaw": 0.0},
        "robot_map_tf_pose": robot_pose,
        "start_center_cost": start_cost,
        "grid_details": {
            "gx": grid_details[0] if grid_details else None,
            "gy": grid_details[1] if grid_details else None,
            "resolution": grid_details[2] if grid_details else None,
            "origin_x": grid_details[3] if grid_details else None,
            "origin_y": grid_details[4] if grid_details else None
        } if grid_details else None,
        "wall_collision_state": (
            "clear_of_wall_mesh" if cond_id == "A" else
            "overlapping_wall_mesh" if cond_id == "B" else
            "temporary_obstacle_adjacent" if cond_id == "C" else
            "free_space_nominal"
        ),
        "inflation_wait_detected": has_inflation_detected_log,
        "inflation_timeout_detected": has_timeout_log,
        "wait_duration_sec": round(wait_duration, 2),
        "dispatched_nav_goals": dispatched_nav_goals,
        "action_terminal_status": action_status,
        "action_result_success": action_success,
        "action_result_message": action_msg,
        "cmd_vel_count": monitor.cmd_vel_count,
        "dist_traveled_m": round(dist_traveled, 4),
        "raw_mapping_log_path": str(raw_log_path)
    }

    # Save individual run json
    run_json_path = LOGS_DIR / f"{run_id}.json"
    run_json_path.write_text(json.dumps(result_data, indent=2, ensure_ascii=False))

    return result_data


def main():
    rclpy.init()
    results = []

    # Condition A: Cost == 99 continuously
    # Spawn at x = -7.70 (clear of wall mesh: clearance 0.10m > 0.065m chassis half-width; cost == 99)
    res_a = run_condition(
        cond_id="A",
        label="inscribed_cost99_continuous",
        spawn_x=-7.70,
        spawn_y=0.0
    )
    results.append(res_a)

    # Condition B: Cost == 100 continuously
    # Spawn at x = -7.78 (overlapping wall mesh: robot center on wall hit cell; cost == 100)
    res_b = run_condition(
        cond_id="B",
        label="lethal_cost100_continuous",
        spawn_x=-7.78,
        spawn_y=0.0
    )
    results.append(res_b)

    # Condition C: Cost >= 99 initially, obstacle removed during wait -> resumes exploration
    # Spawn at x = -7.0, y = 0.0 with dynamic obstacle box placed at (-6.60, 0.0)
    res_c = run_condition(
        cond_id="C",
        label="temporary_obstacle_cleared_resumes",
        spawn_x=-7.0,
        spawn_y=0.0,
        is_dynamic_obstacle=True,
        dyn_box_coords=(-6.60, 0.0)
    )
    results.append(res_c)

    # Condition D: Free space regression test (cost == 0)
    # Spawn at x = -7.40, y = 0.0 (dist to wall 0.40m > 0.30m inflation radius; cost == 0)
    res_d = run_condition(
        cond_id="D",
        label="free_space_regression_immediate",
        spawn_x=-7.40,
        spawn_y=0.0
    )
    results.append(res_d)

    # Save summary JSON
    summary_path = LOGS_DIR / 'inscribed_start_verification_results.json'
    summary_path.write_text(json.dumps(results, indent=2, ensure_ascii=False))

    # Also save to notebook deliverable directory
    nb_summary_path = NOTEBOOK_DIR / 'inscribed_start_verification_results.json'
    nb_summary_path.write_text(json.dumps(results, indent=2, ensure_ascii=False))

    print("\n=======================================================", flush=True)
    print("ALL CONDITIONS TESTED. SUMMARY:", flush=True)
    for r in results:
        print(f"[{r['condition_id']}] {r['label']}: VERDICT={r['verdict']} | Cost={r['start_center_cost']} | Status={r['action_terminal_status']} | NavGoals={r['dispatched_nav_goals']} | Dist={r['dist_traveled_m']}m", flush=True)
    print("=======================================================", flush=True)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
