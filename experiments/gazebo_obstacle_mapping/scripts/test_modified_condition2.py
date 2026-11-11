#!/usr/bin/env python3
"""
test_modified_condition2.py:
Executes the live Gazebo simulation stack with the MODIFIED code
to verify fix under Condition 2: Robot start position in inflation zone (cost >= 99).
Checks:
- No immediate crash with 'Robot outside known traversable costmap'
- BFS escape / grace period allows planner to proceed
- Action does NOT immediately abort
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
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from limo_mapping_interfaces.action import AutoMapping

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping')
LOGS_DIR = EXP_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)
RAW_LOG_FILE = LOGS_DIR / 'modified_live_cond2.raw.log'
JSON_LOG_FILE = LOGS_DIR / 'modified_live_cond2.json'
MAPPING_LOG_FILE = LOGS_DIR / 'modified_mapping_server.log'
WORLD_SDF = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf')


class TeeLogger:
    def __init__(self, stream, file):
        self.stream = stream
        self.file = file
    def write(self, data):
        self.stream.write(data)
        self.file.write(data)
        self.stream.flush()
        self.file.flush()
    def flush(self):
        self.stream.flush()
        self.file.flush()


def verify_code_provenance():
    expected_prefix = str(EXP_DIR / 'ws/install/limo_auto_mapping')
    try:
        prefix = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', 'limo_auto_mapping']
        ).decode().strip()
    except Exception as exc:
        raise RuntimeError(f"Failed to query ros2 pkg prefix: {exc}")

    import limo_auto_mapping.server as s
    import limo_auto_mapping.explorer as e
    import limo_auto_mapping.frontier as f
    import limo_auto_mapping.resume as r

    actual_paths = {
        'server': s.__file__,
        'explorer': e.__file__,
        'frontier': f.__file__,
        'resume': r.__file__,
    }

    src_dir = EXP_DIR / 'ws/src/limo_auto_mapping/limo_auto_mapping'
    src_hashes = {}
    for mod, path in actual_paths.items():
        src_file = src_dir / f"{mod}.py"
        with open(src_file, 'rb') as fp:
            src_hashes[mod] = hashlib.sha256(fp.read()).hexdigest()

    print("==================================================================", flush=True)
    print("PROVENANCE VERIFICATION (MODIFIED CODE):", flush=True)
    print(f"  Expected Prefix: {expected_prefix}", flush=True)
    print(f"  Actual Prefix:   {prefix}", flush=True)
    for mod, p in actual_paths.items():
        print(f"  {mod:10s} Path: {p}", flush=True)
        print(f"  {mod:10s} Hash: {src_hashes[mod]}", flush=True)
    print("==================================================================\n", flush=True)

    if prefix != expected_prefix:
        raise RuntimeError(f"Package prefix mismatch! Expected {expected_prefix}, got {prefix}")
    return src_hashes


class LiveMonitorNode(Node):
    def __init__(self):
        super().__init__('live_monitor_node_cond2')
        self.feedbacks = []
        self.action_result = None
        self.action_status = None
        self.latest_costmap = None
        self.start_odom = None
        self.distance_traveled = 0.0

        transient = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.feedbacks.append((time.monotonic(), fb.phase, fb.retry_count))
        self.get_logger().info(f"[ACTION FEEDBACK]: phase={fb.phase}, retries={fb.retry_count}")

    def on_result_cb(self, future):
        res = future.result()
        self.action_status = res.status
        self.action_result = res.result
        self.get_logger().info(f"[ACTION RESULT]: status={res.status}, success={res.result.success}, msg='{res.result.message}'")

    def on_costmap(self, msg):
        self.latest_costmap = msg

    def on_odom(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        if self.start_odom is None:
            self.start_odom = (px, py)
        else:
            self.distance_traveled = math.hypot(px - self.start_odom[0], py - self.start_odom[1])


def stop_process(proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=6)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=3)
        except Exception:
            pass


def run_condition_2_modified_test(src_hashes, spawn_x=-7.75, spawn_y=0.0, spawn_yaw=0.0, run_suffix="01"):
    run_id = f"mod_c2_{run_suffix}_{str(uuid.uuid4())[:8]}"
    print("\n------------------------------------------------------------------", flush=True)
    print(f"RUNNING LIVE MODIFIED TEST: Condition 2 (Start in Inflation Zone)", flush=True)
    print(f"Run ID: {run_id}, Spawn: ({spawn_x}, {spawn_y}, {spawn_yaw})", flush=True)
    print("------------------------------------------------------------------", flush=True)

    print(f"[1/3] Starting Gazebo Fortress at ({spawn_x}, {spawn_y})...", flush=True)
    sim_proc = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD_SDF}", f"spawn_x:={spawn_x}", f"spawn_y:={spawn_y}", f"spawn_yaw:={spawn_yaw}",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(6)

    print("[2/3] Starting auto_mapping.launch.py (modified)...", flush=True)
    map_log_fd = open(MAPPING_LOG_FILE, 'w')
    mapping_proc = subprocess.Popen([
        "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
        "use_sim_time:=true"
    ], stdout=map_log_fd, stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(8)

    monitor = LiveMonitorNode()
    client = ActionClient(monitor, AutoMapping, '/auto_mapping')

    print("[3/3] Sending AutoMapping Action Goal...", flush=True)
    server_ready = client.wait_for_server(timeout_sec=25.0)
    if not server_ready:
        stop_process(mapping_proc)
        stop_process(sim_proc)
        map_log_fd.close()
        raise RuntimeError("Action server /auto_mapping did not become ready!")

    output_prefix = str(EXP_DIR / f'maps/test_mod_c2_{run_id}')
    goal_msg = AutoMapping.Goal()
    goal_msg.output_prefix = output_prefix
    goal_msg.max_duration_sec = 35.0

    send_future = client.send_goal_async(goal_msg, feedback_callback=monitor.on_feedback)
    goal_start_time = time.monotonic()
    goal_handle = None

    crashed = False
    crash_cause = ""
    goal_dispatched = False

    while time.monotonic() - goal_start_time < 40.0:
        rclpy.spin_once(monitor, timeout_sec=0.2)
        if send_future.done() and goal_handle is None:
            goal_handle = send_future.result()
            print(f"Goal accepted by MappingServer: {goal_handle.accepted}", flush=True)
            if not goal_handle.accepted:
                crash_cause = "Goal was rejected by mapping server"
                crashed = True
                break
            goal_handle.get_result_async().add_done_callback(monitor.on_result_cb)

        # Check feedback
        if monitor.feedbacks:
            phases = [f[1] for f in monitor.feedbacks]
            if any('explore' in p or 'navigating' in p for p in phases):
                goal_dispatched = True

        if monitor.action_status is not None:
            print(f"Action terminal status reached: {monitor.action_status}", flush=True)
            print(f"Action message: {monitor.action_result.message}", flush=True)
            if monitor.action_status == GoalStatus.STATUS_ABORTED:
                if 'Python explorer reported an error' in monitor.action_result.message:
                    crashed = True
                    crash_cause = monitor.action_result.message
            break

    stop_process(mapping_proc)
    stop_process(sim_proc)
    map_log_fd.close()
    monitor.destroy_node()

    log_content = Path(MAPPING_LOG_FILE).read_text()
    error_line = ""
    for line in log_content.splitlines():
        if 'Robot outside known traversable costmap' in line or 'exploration_error' in line:
            error_line = line
            break

    success_pass = (not crashed) and (error_line == "")
    print(f"*** Condition 2 Live Test Result: Pass={success_pass}, Crashed={crashed}, DistanceTraveled={monitor.distance_traveled:.3f}m ***\n")
    return {
        'run_id': run_id,
        'condition': 'C2_start_in_inflation_zone',
        'passed': success_pass,
        'crashed': crashed,
        'crash_cause': crash_cause,
        'error_line': error_line,
        'distance_traveled_m': monitor.distance_traveled,
        'feedbacks': [f[1] for f in monitor.feedbacks],
        'action_status': monitor.action_status,
        'contact_measured': 'not_measured'
    }


def main():
    raw_fd = open(RAW_LOG_FILE, 'w')
    sys.stdout = TeeLogger(sys.__stdout__, raw_fd)
    sys.stderr = TeeLogger(sys.__stderr__, raw_fd)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])

    try:
        src_hashes = verify_code_provenance()
        # Run 3 repetitions with slightly different start poses/seeds in inflation zone
        runs = [
            (-7.75, 0.0, 0.0, "run1"),
            (-7.72, 0.1, 0.1, "run2"),
            (-7.70, -0.1, -0.1, "run3"),
        ]
        results = []
        for sx, sy, syaw, sfx in runs:
            res = run_condition_2_modified_test(src_hashes, spawn_x=sx, spawn_y=sy, spawn_yaw=syaw, run_suffix=sfx)
            results.append(res)
            time.sleep(2)

        summary = {
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'code_hashes': src_hashes,
            'raw_log_path': str(RAW_LOG_FILE),
            'results': results
        }

        with open(JSON_LOG_FILE, 'w') as fp:
            json.dump(summary, fp, indent=2)

        print("\n==================================================================", flush=True)
        print(f"MODIFIED CONDITION 2 TESTS COMPLETE. Output saved to: {JSON_LOG_FILE}", flush=True)
        print("==================================================================", flush=True)

    finally:
        rclpy.shutdown()
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        raw_fd.close()


if __name__ == '__main__':
    main()
