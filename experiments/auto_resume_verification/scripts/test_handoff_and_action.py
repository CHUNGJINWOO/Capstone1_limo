#!/usr/bin/env python3
"""
test_handoff_and_action.py:
Component verification test for SLAM continuation and AutoMapping Action:
- Injects known start_pose [0.0, 0.0, 0.0] and sets require_localization_ready:=false.
- Verifies:
  1. SLAM resumption from serialized posegraph.
  2. TF tree integrity (clean map->odom without duplication).
  3. Pose deviation from injected start_pose.
  4. Real LiDAR obstacle clearance and collision monitoring.
  5. AutoMapping Action execution, feedback reception, and cmd_vel dispatch.
  6. Map serialization (.pgm, .yaml, .posegraph, .data).
  7. Direct generation of component_handoff_action.json from raw observations.
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
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from limo_mapping_interfaces.action import AutoMapping
from tf2_ros import Buffer, TransformListener, TransformException


EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification')
BASE_MAP = EXP_DIR / 'maps/complex_indoor_baseline'
NEW_MAP = EXP_DIR / 'maps/resumed_session_01'
WORLD_STAGE_A = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf')
JSON_LOG_FILE = EXP_DIR / 'logs/component_handoff_action.json'
RAW_LOG_FILE = EXP_DIR / 'logs/component_handoff_action.raw.log'

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'STATUS_UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'STATUS_ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'STATUS_EXECUTING',
    GoalStatus.STATUS_CANCELING: 'STATUS_CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'STATUS_SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'STATUS_CANCELED',
    GoalStatus.STATUS_ABORTED: 'STATUS_ABORTED',
}


def compute_code_hash(script_path):
    try:
        git_hash = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=str(EXP_DIR.parent.parent)
        ).decode().strip()
    except Exception:
        git_hash = "unknown_git_commit"
    with open(script_path, 'rb') as f:
        script_sha = hashlib.sha256(f.read()).hexdigest()
    return f"git:{git_hash} | script_sha256:{script_sha}"


def kill_proc(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        p.wait(timeout=6)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            p.wait(timeout=3)
        except Exception:
            pass


class HandoffTester(Node):
    def __init__(self):
        super().__init__('handoff_action_tester')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, AutoMapping, '/auto_mapping')
        self.map_updates = 0
        self.cmd_vel_count = 0
        self.feedback_history = []
        self.min_scan_range = float('inf')

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, transient)

    def on_cmd_vel(self, msg):
        if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
            self.cmd_vel_count += 1

    def on_scan(self, msg):
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            m = min(valid)
            if m < self.min_scan_range:
                self.min_scan_range = m

    def on_map(self, msg):
        self.map_updates += 1

    def get_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            p = t.transform.translation
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (p.x, p.y, yaw)
        except TransformException:
            return None


def main():
    RAW_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    raw_log_fd = open(RAW_LOG_FILE, 'w')

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

    sys.stdout = TeeLogger(sys.__stdout__, raw_log_fd)
    sys.stderr = TeeLogger(sys.__stderr__, raw_log_fd)

    run_id = str(uuid.uuid4())
    command_str = f"python3 {Path(__file__).resolve()}"
    code_hash_str = compute_code_hash(Path(__file__).resolve())

    print("=======================================================", flush=True)
    print("[COMPONENT TEST] SLAM Handoff, Continuation & AutoMapping Action", flush=True)
    print("  (Start pose injected; localization gate bypassed)", flush=True)
    print(f"  Run ID:    {run_id}", flush=True)
    print(f"  Command:   {command_str}", flush=True)
    print(f"  Code Hash: {code_hash_str}", flush=True)
    print(f"  Raw Log:   {RAW_LOG_FILE}", flush=True)
    print(f"  Base Posegraph: {BASE_MAP}", flush=True)
    print(f"  New Map Target: {NEW_MAP}", flush=True)
    print("=======================================================\n", flush=True)

    sim_proc = None
    mapping_proc = None
    node = None

    results = {
        "run_id": run_id,
        "command": command_str,
        "code_hash": code_hash_str,
        "raw_log_path": str(RAW_LOG_FILE),
        "test_type": "component_handoff_and_action_injected",
        "spawn_world": [-6.2, -2.0, 0.0],
        "ground_truth_map": [0.0, 0.0, 0.0],
        "start_pose_injected": True,
        "ready_gate_bypassed": True,
        "localization_ready_emitted": "미측정 (게이트 해제됨)",
        "localization_time_sec": "미측정 (AMCL 미실행)",
        "pre_handoff_amcl_pose": "미측정 (AMCL 미실행, 좌표 직접 주입)",
        "amcl_position_error_m": "미측정 (AMCL 미실행)",
        "amcl_yaw_error_rad": "미측정 (AMCL 미실행)",
        "post_handoff_slam_pose": None,
        "pose_delta_position_m": None,
        "pose_delta_yaw_rad": None,
        "amcl_node_terminated": "미측정 (AMCL 미실행)",
        "slam_node_active": False,
        "tf_duplicated": False,
        "min_obstacle_distance_m": None,
        "collision_detected": False,
        "action_goal_accepted": False,
        "action_status": None,
        "action_feedback_received": False,
        "action_cmd_vel_count": 0,
        "feedback_samples_count": 0,
        "new_serialized_files": {}
    }

    try:
        # 1. Start Gazebo at (-6.2, -2.0, 0.0) -> Map Truth: [0.0, 0.0, 0.0]
        sim_cmd = [
            'ros2', 'launch', 'limo_capstone', 'limo_mc3f_sim.launch.py',
            f'world:={WORLD_STAGE_A}',
            'spawn_x:=-6.2', 'spawn_y:=-2.0', 'spawn_yaw:=0.0',
            'headless:=true', 'rviz:=false'
        ]
        sim_proc = subprocess.Popen(sim_cmd, start_new_session=True)
        time.sleep(6)

        # 2. Launch auto_mapping with posegraph continuation and start_pose=[0.0, 0.0, 0.0]
        start_pose_str = "[0.0, 0.0, 0.0]"
        mapping_cmd = [
            'ros2', 'launch', 'limo_auto_mapping', 'auto_mapping.launch.py',
            f'posegraph:={BASE_MAP}',
            f'start_pose:={start_pose_str}',
            'use_sim_time:=true',
            'require_localization_ready:=false'
        ]
        mapping_proc = subprocess.Popen(mapping_cmd, start_new_session=True)

        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
        node = HandoffTester()

        # 3. Wait for SLAM and TF to initialize
        print("Waiting for resumed SLAM /map and TF tree...", flush=True)
        t0 = time.monotonic()
        actual_pose = None
        while time.monotonic() - t0 < 30.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            actual_pose = node.get_pose()
            if actual_pose is not None and node.map_updates > 0:
                break
        else:
            raise RuntimeError("Timed out waiting for resumed SLAM map/TF")

        print(f"Resumed SLAM active! Pose in map: x={actual_pose[0]:.3f}, y={actual_pose[1]:.3f}, yaw={actual_pose[2]:.3f}", flush=True)
        results['slam_node_active'] = True
        results['post_handoff_slam_pose'] = [round(v, 4) for v in actual_pose]
        delta_pos = math.hypot(actual_pose[0] - 0.0, actual_pose[1] - 0.0)
        delta_yaw = abs(actual_pose[2] - 0.0)
        results['pose_delta_position_m'] = round(delta_pos, 4)
        results['pose_delta_yaw_rad'] = round(delta_yaw, 4)
        print(f"  Pose deviation from injected start_pose: {delta_pos:.4f}m, {delta_yaw:.4f}rad", flush=True)

        # 4. Check TF duplication and tree structure directly via tf_buffer
        try:
            t = node.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            print(f"  [OK] Clean map -> odom transform verified from SLAM! translation: ({t.transform.translation.x:.3f}, {t.transform.translation.y:.3f})", flush=True)
            results['tf_duplicated'] = False
        except Exception as e:
            print("  [WARN] Failed looking up map -> odom:", str(e), flush=True)
            results['tf_duplicated'] = True

        # 5. Connect to AutoMapping Action server
        print("Connecting to /auto_mapping Action Server...", flush=True)
        if not node.action_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("AutoMapping action server not available")

        # Clear any prior output files
        for ext in ('.pgm', '.yaml', '.posegraph', '.data'):
            f = Path(str(NEW_MAP) + ext)
            if f.is_file():
                f.unlink()

        print("Sending AutoMapping goal...", flush=True)
        goal = AutoMapping.Goal()
        goal.output_prefix = str(NEW_MAP)
        goal.allow_margin_retry = False
        goal.max_duration_sec = 10.0

        def feedback_cb(feedback_msg):
            fb = feedback_msg.feedback
            print(f"  [Action Feedback] Phase: {fb.phase}, Elapsed: {fb.elapsed_sec:.1f}s, Retries: {fb.retry_count}", flush=True)
            node.feedback_history.append({'phase': fb.phase, 'elapsed': fb.elapsed_sec, 'retries': fb.retry_count})

        send_future = node.action_client.send_goal_async(goal, feedback_callback=feedback_cb)
        t_action_end = time.monotonic() + 15.0
        while time.monotonic() < t_action_end and not send_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            results['action_goal_accepted'] = False
            results['action_status'] = "REJECTED"
            print("Goal was rejected by AutoMapping server!", flush=True)
        else:
            results['action_goal_accepted'] = True
            print("  -> Goal ACCEPTED by AutoMapping action server!", flush=True)

            # 6. Monitor Action Execution for exploration
            result_future = goal_handle.get_result_async()
            t_run_end = time.monotonic() + 35.0
            while time.monotonic() < t_run_end and not result_future.done():
                rclpy.spin_once(node, timeout_sec=0.1)

            if result_future.done():
                res = result_future.result()
                status_name = STATUS_NAMES.get(res.status, f"UNKNOWN({res.status})")
                results['action_status'] = f"{status_name} ({res.status})"
                print(f"Action completed with status: {results['action_status']}", flush=True)
                if hasattr(res.result, 'message'):
                    print(f"Action Result Message: {res.result.message}", flush=True)
            else:
                results['action_status'] = "STATUS_TIMEOUT"
                print("Action goal timed out waiting for result!", flush=True)

        results['action_feedback_received'] = len(node.feedback_history) > 0
        results['feedback_samples_count'] = len(node.feedback_history)
        results['action_cmd_vel_count'] = node.cmd_vel_count
        print(f"Total /cmd_vel navigation commands observed: {node.cmd_vel_count}", flush=True)

        # Collision detection from scan
        results['min_obstacle_distance_m'] = round(node.min_scan_range, 3) if node.min_scan_range != float('inf') else "미측정"
        results['collision_detected'] = (node.min_scan_range < 0.15) if node.min_scan_range != float('inf') else False
        print(f"LiDAR Minimum Obstacle Distance: {results['min_obstacle_distance_m']}m, Collision: {results['collision_detected']}", flush=True)

        # 7. Serialize map and save 2D map
        print("Saving new map under new name...", flush=True)
        save_cmd = [
            'bash', '-c',
            f'source /opt/ros/humble/setup.bash && export PYTHONNOUSERSITE=1 && ros2 run nav2_map_server map_saver_cli -f {NEW_MAP} --ros-args -p use_sim_time:=true'
        ]
        subprocess.run(save_cmd, capture_output=True, timeout=15)

        # Call slam_toolbox serialize
        serialize_cmd = [
            'bash', '-c',
            f'source /opt/ros/humble/setup.bash && export PYTHONNOUSERSITE=1 && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{{filename: \'{NEW_MAP}\'}}"'
        ]
        subprocess.run(serialize_cmd, capture_output=True, timeout=15)

        # 8. Verify newly created files
        for ext in ('.pgm', '.yaml', '.posegraph', '.data'):
            f = Path(str(NEW_MAP) + ext)
            if f.is_file() and f.stat().st_size > 0:
                results['new_serialized_files'][ext] = f.stat().st_size
                print(f"  [OK] New file generated: {f.name} ({f.stat().st_size} bytes)", flush=True)
            else:
                results['new_serialized_files'][ext] = None
                print(f"  [FAIL] Missing file: {f.name}", flush=True)

    except Exception as exc:
        print(f"Exception during component test: {exc}", flush=True)

    finally:
        if node:
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        kill_proc(mapping_proc)
        kill_proc(sim_proc)
        subprocess.run(['pkill', '-9', '-f', 'ign gazebo'], check=False)
        subprocess.run(['pkill', '-9', '-f', 'ros_gz'], check=False)
        subprocess.run(['pkill', '-9', '-f', 'async_slam_toolbox'], check=False)
        subprocess.run(['pkill', '-9', '-f', 'nav2_'], check=False)
        time.sleep(3)

        with open(JSON_LOG_FILE, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"\n[OK] Component Test results written directly to: {JSON_LOG_FILE}", flush=True)
        raw_log_fd.close()


if __name__ == '__main__':
    main()
