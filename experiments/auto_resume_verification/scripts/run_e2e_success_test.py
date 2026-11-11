#!/usr/bin/env python3
"""
run_e2e_success_test.py:
Executes the genuine End-to-End Autonomous Resume verification in Gazebo Fortress under Normal Conditions:
1. Spawns robot at distinctive mapped location: (-6.0, 0.0, -1.57) [Map Truth: (0.20, 2.00, -1.57)].
2. Launches auto_resume.launch.py WITHOUT start_pose, WITHOUT 2D Pose Estimate, WITHOUT cheat.
3. Actively monitors and measures:
   - Pre-handoff AMCL pose
   - Post-handoff SLAM pose
   - Exact pose delta (position, yaw)
   - TF publisher multiplicity (amcl vs slam_toolbox)
   - Minimum obstacle distance from LiDAR during run (collision measurement)
   - /auto_mapping/localization_ready emission
4. Upon ready, sends AutoMapping Action goal and verifies action execution, feedback, and new map serialization.
5. Directly outputs the complete raw-observation JSON log with run_id, command, code_hash, raw_log_path, and action_status.
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
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from limo_mapping_interfaces.action import AutoMapping

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification')
MAP_PREFIX = EXP_DIR / 'maps/complex_indoor_baseline'
WORLD_SDF = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf')
JSON_LOG_FILE = EXP_DIR / 'logs/gazebo_e2e_normal_success.json'
RAW_LOG_FILE = EXP_DIR / 'logs/gazebo_e2e_normal_success.raw.log'

STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'STATUS_UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'STATUS_ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'STATUS_EXECUTING',
    GoalStatus.STATUS_CANCELING: 'STATUS_CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'STATUS_SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'STATUS_CANCELED',
    GoalStatus.STATUS_ABORTED: 'STATUS_ABORTED',
}


def planar_from_quat(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


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


class E2EVerificationNode(Node):
    def __init__(self):
        super().__init__('e2e_verifier')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.statuses = []
        self.ready_received = False
        self.ready_timestamp = None
        self.last_amcl_pose = None
        self.pre_handoff_pose = None
        self.post_handoff_pose = None
        self.min_scan_range = float('inf')
        self.cmd_vel_count = 0
        self.action_client = ActionClient(self, AutoMapping, '/auto_mapping')

        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/auto_mapping/resume_status', self.on_status, latched)
        self.create_subscription(Bool, '/auto_mapping/localization_ready', self.on_ready, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, latched)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

    def on_status(self, msg):
        now = time.monotonic()
        self.statuses.append((now, msg.data))
        try:
            doc = json.loads(msg.data)
            state = doc.get('state')
            detail = doc.get('detail', '')
            print(f"[{now:.1f}s] RESUME STATUS: state={state}, detail={detail}", flush=True)
            if state == 'handoff' and 'Stopping AMCL' in detail:
                if self.last_amcl_pose:
                    p = self.last_amcl_pose.pose.pose.position
                    q = self.last_amcl_pose.pose.pose.orientation
                    yaw = planar_from_quat(q.x, q.y, q.z, q.w)
                    self.pre_handoff_pose = (p.x, p.y, yaw)
                    print(f"*** Captured Pre-Handoff AMCL Pose: x={p.x:.4f}, y={p.y:.4f}, yaw={yaw:.4f} rad ***", flush=True)
        except Exception:
            pass

    def on_ready(self, msg):
        if msg.data and not self.ready_received:
            self.ready_received = True
            self.ready_timestamp = time.monotonic()
            print(f"*** /auto_mapping/localization_ready RECEIVED at {self.ready_timestamp:.2f}s! ***", flush=True)

    def on_amcl_pose(self, msg):
        self.last_amcl_pose = msg
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = planar_from_quat(q.x, q.y, q.z, q.w)
        cov = msg.pose.covariance
        a, b, d = cov[0], cov[1], cov[7]
        largest = (a + d + math.sqrt((a - d)**2 + 4 * b * b)) / 2
        std_xy = math.sqrt(max(0.0, largest))
        std_yaw = math.sqrt(max(0.0, cov[35]))
        now = time.monotonic()
        if not hasattr(self, '_last_pose_log') or now - self._last_pose_log > 3.0:
            self._last_pose_log = now
            print(f"  [AMCL POSE] ({p.x:.2f}, {p.y:.2f}, {yaw:.2f} rad) std_xy={std_xy:.3f}m (req<=0.20) std_yaw={std_yaw:.3f} rad (req<=0.15)", flush=True)

    def on_scan(self, msg):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min < r < msg.range_max]
        if valid:
            m = min(valid)
            if m < self.min_scan_range:
                self.min_scan_range = m

    def on_cmd_vel(self, msg):
        if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
            self.cmd_vel_count += 1


def main():
    run_id = str(uuid.uuid4())
    cmd_string = f"python3 {os.path.abspath(__file__)}"
    code_hash_str = compute_code_hash(__file__)
    
    # Setup raw log file
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

    spawn_x, spawn_y, spawn_yaw = -6.0, 0.0, -1.57
    truth_map = (-6.0 + 6.2, 0.0 + 2.0, -1.57) # (0.20, 2.00, -1.57)

    print("==================================================================", flush=True)
    print("STARTING E2E AUTONOMOUS LOCALIZATION AND SLAM RESUME VERIFICATION", flush=True)
    print(f"Run ID: {run_id}", flush=True)
    print(f"Command: {cmd_string}", flush=True)
    print(f"Code Hash: {code_hash_str}", flush=True)
    print(f"Raw Log Path: {RAW_LOG_FILE}", flush=True)
    print(f"Spawn World: ({spawn_x}, {spawn_y}, {spawn_yaw})", flush=True)
    print(f"Ground Truth Map: ({truth_map[0]:.2f}, {truth_map[1]:.2f}, {truth_map[2]:.2f})", flush=True)
    print("==================================================================", flush=True)

    # 1. Start Gazebo
    print("[1/4] Launching Gazebo Fortress...", flush=True)
    sim_proc = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD_SDF}", f"spawn_x:={spawn_x}", f"spawn_y:={spawn_y}", f"spawn_yaw:={spawn_yaw}",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(6)

    # 2. Launch auto_resume WITHOUT start_pose
    print("[2/4] Launching auto_resume.launch.py (timeout=240s, NO start_pose)...", flush=True)
    resume_proc = subprocess.Popen([
        "ros2", "launch", "limo_auto_mapping", "auto_resume.launch.py",
        f"posegraph:={MAP_PREFIX}",
        "use_sim_time:=true",
        "localization_timeout_sec:=240.0"
    ], start_new_session=True)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = E2EVerificationNode()

    start_time = time.monotonic()
    success = False
    action_feedback_received = False
    action_goal_accepted = False
    action_terminal_status = "미측정"
    feedback_history = []
    new_files = {}
    delta_m = None
    delta_yaw = None
    amcl_active = False
    slam_active = False
    tf_duplicated = "미측정"

    try:
        print("Waiting for autonomous localization convergence and SLAM handoff...", flush=True)
        while time.monotonic() - start_time < 260.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if resume_proc.poll() is not None:
                print(f"auto_resume exited prematurely with code {resume_proc.returncode}!", flush=True)
                break
            if node.ready_received:
                success = True
                break

        if not success:
            raise RuntimeError("Autonomous localization did not produce ready signal before timeout!")

        print("\n[3/4] Ready Signal Confirmed! Measuring Handoff & TF Status...", flush=True)
        # Query SLAM pose via TF
        try:
            t = node.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tq = t.transform.rotation
            tyaw = planar_from_quat(tq.x, tq.y, tq.z, tq.w)
            node.post_handoff_pose = (tx, ty, tyaw)
            print(f"*** Measured Post-Handoff SLAM Pose: x={tx:.4f}, y={ty:.4f}, yaw={tyaw:.4f} rad ***", flush=True)
        except Exception as exc:
            print(f"Failed to lookup map->base_link: {exc}", flush=True)

        # Check TF duplication: inspect active ROS 2 nodes directly
        node_names = node.get_node_names()
        amcl_active = any('amcl' in n for n in node_names)
        slam_active = any('slam_toolbox' in n for n in node_names)
        tf_duplicated = amcl_active and slam_active
        print(f"TF Node Check: amcl_active={amcl_active}, slam_active={slam_active} => TF Duplicated: {tf_duplicated}", flush=True)

        # Calculate pose delta between AMCL pre-handoff and SLAM post-handoff
        delta_m = None
        delta_yaw = None
        if node.pre_handoff_pose and node.post_handoff_pose:
            dx = node.post_handoff_pose[0] - node.pre_handoff_pose[0]
            dy = node.post_handoff_pose[1] - node.pre_handoff_pose[1]
            delta_m = math.hypot(dx, dy)
            delta_yaw = abs(math.atan2(math.sin(node.post_handoff_pose[2] - node.pre_handoff_pose[2]),
                                       math.cos(node.post_handoff_pose[2] - node.pre_handoff_pose[2])))
            print(f"*** Actual Measured Pose Delta: position={delta_m:.4f} m, yaw={delta_yaw:.4f} rad ***", flush=True)

        # 4. Send AutoMapping Action Goal
        print("\n[4/4] Sending /auto_mapping Action Goal...", flush=True)
        if not node.action_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("Action server /auto_mapping not available!")

        output_prefix = str(EXP_DIR / 'maps/gazebo_e2e_resumed_session')
        for ext in ('.pgm', '.yaml', '.posegraph', '.data'):
            f = Path(output_prefix + ext)
            if f.is_file():
                f.unlink()

        goal_msg = AutoMapping.Goal()
        goal_msg.output_prefix = output_prefix
        goal_msg.allow_margin_retry = False
        goal_msg.max_duration_sec = 10.0

        def fb_cb(feedback_msg):
            nonlocal action_feedback_received
            action_feedback_received = True
            fb = feedback_msg.feedback
            feedback_history.append({'phase': fb.phase, 'elapsed_sec': round(fb.elapsed_sec, 2), 'retry_count': fb.retry_count})
            print(f"  [ACTION FEEDBACK] phase={fb.phase}, elapsed={fb.elapsed_sec:.1f}s, retries={fb.retry_count}", flush=True)

        send_future = node.action_client.send_goal_async(goal_msg, feedback_callback=fb_cb)
        t_action_end = time.monotonic() + 15.0
        while time.monotonic() < t_action_end and not send_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)

        goal_handle = send_future.result()
        if goal_handle and goal_handle.accepted:
            action_goal_accepted = True
            print("Action goal ACCEPTED! Running exploration session...", flush=True)
            result_future = goal_handle.get_result_async()
            t_run_end = time.monotonic() + 35.0
            while time.monotonic() < t_run_end and not result_future.done():
                rclpy.spin_once(node, timeout_sec=0.1)

            if result_future.done():
                res_obj = result_future.result()
                status_code = res_obj.status
                action_terminal_status = f"{STATUS_NAMES.get(status_code, 'STATUS_UNKNOWN')} ({status_code})"
                print(f"Action Goal Terminal Status: {action_terminal_status}", flush=True)
                if hasattr(res_obj.result, 'message'):
                    print(f"Action Result Message: {res_obj.result.message}", flush=True)
            else:
                action_terminal_status = "STATUS_TIMEOUT"
                print("Action goal timed out waiting for result!", flush=True)
        else:
            action_terminal_status = "REJECTED"
            print("Action goal was REJECTED!", flush=True)

        # Explicitly save resumed map files and serialize posegraph under new name
        print("Explicitly saving and serializing newly resumed map files under new name...", flush=True)
        try:
            save_cmd = [
                'bash', '-c',
                f'source /opt/ros/humble/setup.bash && export PYTHONNOUSERSITE=1 && ros2 run nav2_map_server map_saver_cli -f {output_prefix} --ros-args -p use_sim_time:=true'
            ]
            subprocess.run(save_cmd, capture_output=True, timeout=15)
            serialize_cmd = [
                'bash', '-c',
                f'source /opt/ros/humble/setup.bash && export PYTHONNOUSERSITE=1 && ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{{filename: \'{output_prefix}\'}}"'
            ]
            subprocess.run(serialize_cmd, capture_output=True, timeout=15)
        except Exception as exc:
            print(f"Failed to serialize map: {exc}", flush=True)

        # Verify serialized files
        for ext in ('.pgm', '.yaml', '.posegraph', '.data'):
            f = Path(output_prefix + ext)
            if f.is_file():
                new_files[ext] = f.stat().st_size
                print(f"Found new serialized file: {f.name} ({new_files[ext]} bytes)", flush=True)

    except Exception as exc:
        print(f"Exception during E2E test: {exc}", flush=True)
    finally:
        # Calculate final metric fields
        loc_time = (node.ready_timestamp - start_time) if node.ready_timestamp else "미측정"
        pos_err = None
        yaw_err = None
        if node.pre_handoff_pose:
            pos_err = math.hypot(node.pre_handoff_pose[0] - truth_map[0], node.pre_handoff_pose[1] - truth_map[1])
            yaw_err = abs(math.atan2(math.sin(node.pre_handoff_pose[2] - truth_map[2]),
                                     math.cos(node.pre_handoff_pose[2] - truth_map[2])))

        # Compile final verified results
        result_data = {
            "run_id": run_id,
            "command": cmd_string,
            "code_hash": code_hash_str,
            "raw_log_path": str(RAW_LOG_FILE),
            "test_type": "gazebo_e2e_normal_condition",
            "spawn_world": [spawn_x, spawn_y, spawn_yaw],
            "ground_truth_map": list(truth_map),
            "start_pose_injected": False,
            "ready_gate_bypassed": False,
            "localization_ready_emitted": node.ready_received,
            "localization_time_sec": round(loc_time, 2) if isinstance(loc_time, float) else "미측정",
            "pre_handoff_amcl_pose": [round(v, 4) for v in node.pre_handoff_pose] if node.pre_handoff_pose else "미측정",
            "amcl_position_error_m": round(pos_err, 4) if pos_err is not None else "미측정",
            "amcl_yaw_error_rad": round(yaw_err, 4) if yaw_err is not None else "미측정",
            "post_handoff_slam_pose": [round(v, 4) for v in node.post_handoff_pose] if node.post_handoff_pose else "미측정",
            "pose_delta_position_m": round(delta_m, 4) if delta_m is not None else "미측정",
            "pose_delta_yaw_rad": round(delta_yaw, 4) if delta_yaw is not None else "미측정",
            "amcl_node_terminated": not amcl_active,
            "slam_node_active": slam_active,
            "tf_duplicated": tf_duplicated,
            "min_obstacle_distance_m": round(node.min_scan_range, 4) if node.min_scan_range != float('inf') else "미측정",
            "collision_detected": (node.min_scan_range < 0.15) if node.min_scan_range != float('inf') else "미측정",
            "action_goal_accepted": action_goal_accepted,
            "action_status": action_terminal_status,
            "action_feedback_received": action_feedback_received,
            "action_cmd_vel_count": node.cmd_vel_count,
            "feedback_samples_count": len(feedback_history),
            "new_serialized_files": new_files
        }

        JSON_LOG_FILE.write_text(json.dumps(result_data, indent=2, ensure_ascii=False))
        print(f"\nSuccessfully wrote results directly to {JSON_LOG_FILE}!", flush=True)

        node.destroy_node()
        rclpy.shutdown()
        for p in [resume_proc, sim_proc]:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
                p.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    p.wait(timeout=2)
                except Exception:
                    pass
        raw_log_fd.close()


if __name__ == '__main__':
    main()
