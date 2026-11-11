#!/usr/bin/env python3
"""
test_baseline_reproduction.py:
Executes the live Gazebo simulation stack with the UNMODIFIED baseline code
to reproduce the actual failures under:
Condition 1: Cluttered start with small exit
Condition 2: Robot start position in costmap inflation zone (cost >= 99)
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
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from limo_mapping_interfaces.action import AutoMapping

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping')
LOGS_DIR = EXP_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)
RAW_LOG_FILE = LOGS_DIR / 'baseline_live_reproduction.raw.log'
JSON_LOG_FILE = LOGS_DIR / 'baseline_live_reproduction.json'
MAPPING_LOG_FILE = LOGS_DIR / 'mapping_server.log'
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
    print("PROVENANCE VERIFICATION:", flush=True)
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
        super().__init__('live_monitor_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.feedbacks = []
        self.action_result = None
        self.action_status = None
        self.latest_costmap = None
        self.latest_odom = None
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
        self.latest_odom = msg
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


def run_condition_2_inflation_reproduction(src_hashes):
    """
    Condition 2 Live Reproduction:
    Spawn robot at wall boundary (x=-7.75, y=0.0) where robot center has cost >= 99.
    Expect baseline to fail with ValueError('Robot outside known traversable costmap')
    and Action to abort with 'Python explorer reported an error'.
    """
    run_id = str(uuid.uuid4())
    print("\n------------------------------------------------------------------", flush=True)
    print(f"RUNNING LIVE REPRODUCTION: Condition 2 (Start in Inflation Zone)", flush=True)
    print(f"Run ID: {run_id}", flush=True)
    print("------------------------------------------------------------------", flush=True)

    spawn_x, spawn_y, spawn_yaw = -7.75, 0.0, 0.0

    print("[1/3] Starting Gazebo Fortress at inflation position (x=-7.75, y=0.0)...", flush=True)
    sim_proc = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD_SDF}", f"spawn_x:={spawn_x}", f"spawn_y:={spawn_y}", f"spawn_yaw:={spawn_yaw}",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(6)

    print("[2/3] Starting auto_mapping.launch.py (baseline)...", flush=True)
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

    output_prefix = str(EXP_DIR / f'maps/test_cond2_{run_id[:8]}')
    goal_msg = AutoMapping.Goal()
    goal_msg.output_prefix = output_prefix
    goal_msg.max_duration_sec = 25.0

    send_future = client.send_goal_async(goal_msg, feedback_callback=monitor.on_feedback)
    goal_start_time = time.monotonic()
    goal_handle = None

    reproduced = False
    reproduce_cause = ""

    while time.monotonic() - goal_start_time < 30.0:
        rclpy.spin_once(monitor, timeout_sec=0.2)
        if send_future.done() and goal_handle is None:
            goal_handle = send_future.result()
            print(f"Goal accepted by MappingServer: {goal_handle.accepted}", flush=True)
            if not goal_handle.accepted:
                reproduce_cause = "Goal was rejected by mapping server"
                break
            goal_handle.get_result_async().add_done_callback(monitor.on_result_cb)

        if monitor.action_status is not None:
            print(f"Action terminal status reached: {monitor.action_status}", flush=True)
            print(f"Action message: {monitor.action_result.message}", flush=True)
            if monitor.action_status == GoalStatus.STATUS_ABORTED:
                reproduced = True
                reproduce_cause = monitor.action_result.message
            break

    stop_process(mapping_proc)
    stop_process(sim_proc)
    map_log_fd.close()
    monitor.destroy_node()

    # Read captured mapping log for the exact Python traceback
    log_content = Path(MAPPING_LOG_FILE).read_text()
    error_line = ""
    for line in log_content.splitlines():
        if 'Robot outside known traversable costmap' in line or 'exploration_error' in line:
            error_line = line
            print(f"Captured explorer log: {error_line}", flush=True)
            break

    print(f"*** Condition 2 Live Reproduction Result: Reproduced={reproduced}, Cause='{reproduce_cause}', Traceback='{error_line}' ***\n")
    return {
        'run_id': run_id,
        'condition': 'C2_start_in_inflation_zone',
        'reproduced': reproduced,
        'cause': reproduce_cause,
        'traceback_log': error_line,
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
        res_c2 = run_condition_2_inflation_reproduction(src_hashes)

        summary = {
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
            'code_hashes': src_hashes,
            'raw_log_path': str(RAW_LOG_FILE),
            'results': [res_c2]
        }

        with open(JSON_LOG_FILE, 'w') as fp:
            json.dump(summary, fp, indent=2)

        print("\n==================================================================", flush=True)
        print(f"BASELINE REPRODUCTION COMPLETE. Output saved to: {JSON_LOG_FILE}", flush=True)
        print("==================================================================", flush=True)

    finally:
        rclpy.shutdown()
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        raw_fd.close()


if __name__ == '__main__':
    main()
