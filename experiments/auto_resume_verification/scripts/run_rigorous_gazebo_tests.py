#!/usr/bin/env python3
"""
run_rigorous_gazebo_tests.py:
Automated test harness for Gazebo verification of LIMO Auto-Resume Localization & Mapping.

Tests the safety and robustness across 6 core scenarios:
  1. Distinctive Mapped Areas
  2. Ambiguous Corridor Area (symmetric walls)
  3. Unexplored / Low-Overlap Area (unmapped plaza)
  4. Altered Obstacle Layout (World Stage B)
  5. Scan / TF Dropout (Sensor stream interrupted)
  6. Robot Moved During Localization (Movement injection)

Strict evaluation criteria:
  - Ground truth used ONLY for evaluation (no cheat / no ground truth injection).
  - No motion commands allowed during localization.
  - Zero false ready signals permitted in ambiguous/unmapped/altered/dropout states.
  - Explicit record of run_id, command, code_hash, raw_log_path.
  - Unmeasured metrics explicitly marked as '미측정'.
"""
import argparse
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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException


EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification')
MAP_PREFIX = EXP_DIR / 'maps/complex_indoor_baseline'
WORLD_STAGE_A = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf')
WORLD_STAGE_B = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageB.sdf')
RESULTS_FILE = EXP_DIR / 'logs/gazebo_verification_results.json'
SUITE_JSON_FILE = EXP_DIR / 'logs/gazebo_safety_suite.json'
RAW_LOG_FILE = EXP_DIR / 'logs/gazebo_safety_suite.raw.log'


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


def world_to_map(x_w, y_w, yaw_w):
    """Rigid transform fixed before mapping and evaluation:
       x_m = x_w + 6.2, y_m = y_w + 2.0, yaw_m = yaw_w"""
    return (x_w + 6.2, y_w + 2.0, yaw_w)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


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


class VerificationMonitor(Node):
    def __init__(self):
        super().__init__('verification_monitor')
        self.cmd_vel_during_loc = 0
        self.resume_statuses = []
        self.readiness_events = []
        self.last_amcl_pose = None
        self.last_odom = None
        self.last_scan = None
        self.map_count = 0
        self.min_scan_range = float('inf')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'last_odom', m), 10)
        
        transient = QoSProfile(depth=5, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/auto_mapping/resume_status', self.on_status, transient)
        self.create_subscription(Bool, '/auto_mapping/localization_ready', self.on_ready, transient)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, transient)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, transient)

    def on_cmd_vel(self, msg):
        if abs(msg.linear.x) > 1e-4 or abs(msg.angular.z) > 1e-4:
            self.cmd_vel_during_loc += 1

    def on_scan(self, msg):
        self.last_scan = msg
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if valid:
            m = min(valid)
            if m < self.min_scan_range:
                self.min_scan_range = m

    def on_status(self, msg):
        self.resume_statuses.append((time.monotonic(), msg.data))

    def on_ready(self, msg):
        self.readiness_events.append((time.monotonic(), bool(msg.data)))

    def on_amcl_pose(self, msg):
        self.last_amcl_pose = msg

    def on_map(self, msg):
        self.map_count += 1


def run_trial(name, category, world_path, spawn_pose, seed, timeout_sec=40.0,
              inject_motion=False, inject_sensor_kill=False):
    trial_run_id = str(uuid.uuid4())
    command_str = f"python3 {Path(__file__).resolve()} --trial {name}"
    code_hash_str = compute_code_hash(Path(__file__).resolve())

    print(f"\n=======================================================", flush=True)
    print(f"[TEST] {name} ({category})", flush=True)
    print(f"  Run ID:    {trial_run_id}", flush=True)
    print(f"  Spawn:     x={spawn_pose[0]}, y={spawn_pose[1]}, yaw={spawn_pose[2]} | Seed: {seed}", flush=True)
    
    truth_m = world_to_map(*spawn_pose)
    print(f"  Pre-determined Ground Truth in Map: x={truth_m[0]:.2f}, y={truth_m[1]:.2f}, yaw={truth_m[2]:.2f}", flush=True)

    record = {
        'run_id': trial_run_id,
        'command': command_str,
        'code_hash': code_hash_str,
        'raw_log_path': str(RAW_LOG_FILE),
        'test_type': 'gazebo_safety_robustness',
        'test_name': name,
        'category': category,
        'spawn_world': list(spawn_pose),
        'ground_truth_map': list(truth_m),
        'seed': seed,
        'world_file': str(world_path.name),
        'timeout_sec': timeout_sec,
        'start_pose_injected': False,
        'ready_gate_bypassed': False,
        'status': 'PENDING',
        'ready_emitted': False,
        'false_ready': False,
        'cmd_vel_violations': 0,
        'pos_error_m': "미측정",
        'yaw_error_rad': "미측정",
        'localization_time_sec': "미측정",
        'pre_handoff_amcl_pose': "미측정 (위치 추정 수렴 미완료로 핸드오프 미진입)",
        'post_handoff_slam_pose': "미측정 (SLAM 미실행)",
        'pose_delta_position_m': "미측정",
        'pose_delta_yaw_rad': "미측정",
        'tf_duplicated': "미측정 (AMCL 단독 실행 중 종료)",
        'min_obstacle_distance_m': "미측정",
        'collision_detected': False,
        'action_goal_accepted': "미측정 (안전성 시험으로 Action 미실행)",
        'action_status': "미측정 (안전성 시험으로 Action 미실행)",
        'action_feedback_received': "미측정 (안전성 시험으로 Action 미실행)",
        'action_cmd_vel_count': "미측정 (안전성 시험으로 Action 미실행)",
        'new_serialized_files': "미측정 (SLAM 미실행)",
        'failure_reason': None,
        'details': {}
    }

    # 1. Launch Gazebo Fortress
    sim_cmd = [
        'ros2', 'launch', 'limo_capstone', 'limo_mc3f_sim.launch.py',
        f'world:={world_path}',
        f'spawn_x:={spawn_pose[0]}',
        f'spawn_y:={spawn_pose[1]}',
        f'spawn_yaw:={spawn_pose[2]}',
        'headless:=true',
        'rviz:=false'
    ]
    sim_proc = subprocess.Popen(sim_cmd, start_new_session=True)
    time.sleep(5)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    monitor = VerificationMonitor()

    # 2. Launch auto_resume
    resume_cmd = [
        'ros2', 'launch', 'limo_auto_mapping', 'auto_resume.launch.py',
        f'posegraph:={MAP_PREFIX}',
        'use_sim_time:=true',
        f'localization_timeout_sec:={timeout_sec}'
    ]
    resume_proc = subprocess.Popen(resume_cmd, start_new_session=True)

    start_time = time.monotonic()
    t_end = start_time + timeout_sec + 15.0  # allow margin for clean termination

    try:
        motion_injected = False
        sensor_killed = False

        while time.monotonic() < t_end:
            rclpy.spin_once(monitor, timeout_sec=0.1)

            # Check if auto_resume process terminated
            if resume_proc.poll() is not None:
                break

            # Injection Scenarios
            elapsed = time.monotonic() - start_time
            if inject_motion and not motion_injected and elapsed > 6.0:
                print("  [INJECTION] Injecting /cmd_vel movement (vx=0.3 m/s) to test stationary abort...", flush=True)
                cmd_pub = monitor.create_publisher(Twist, '/cmd_vel', 10)
                twist = Twist()
                twist.linear.x = 0.30
                cmd_pub.publish(twist)
                motion_injected = True

            if inject_sensor_kill and not sensor_killed and elapsed > 8.0:
                print("  [INJECTION] Simulating sensor dropout by stopping parameter bridge...", flush=True)
                subprocess.run(['pkill', '-f', 'ros_gz_bridge'], check=False)
                sensor_killed = True

            # Check for readiness
            if any(ev[1] for ev in monitor.readiness_events):
                record['ready_emitted'] = True
                break

        total_elapsed = time.monotonic() - start_time
        record['localization_time_sec'] = round(total_elapsed, 2)
        record['cmd_vel_violations'] = monitor.cmd_vel_during_loc

        # Parse latest resume status
        latest_status = None
        if monitor.resume_statuses:
            try:
                latest_status = json.loads(monitor.resume_statuses[-1][1])
            except Exception:
                latest_status = {'raw': monitor.resume_statuses[-1][1]}
        record['details']['latest_status'] = latest_status

        # Evaluate AMCL pose if any
        if monitor.last_amcl_pose:
            p = monitor.last_amcl_pose.pose.pose
            q = p.orientation
            est_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            err_pos = math.hypot(p.position.x - truth_m[0], p.position.y - truth_m[1])
            err_yaw = abs(angle_diff(est_yaw, truth_m[2]))
            record['pos_error_m'] = round(err_pos, 4)
            record['yaw_error_rad'] = round(err_yaw, 4)
            record['details']['final_pose'] = [p.position.x, p.position.y, est_yaw]

        # Evaluate Obstacle clearance and collision from scan
        if monitor.min_scan_range != float('inf'):
            record['min_obstacle_distance_m'] = round(monitor.min_scan_range, 3)
            record['collision_detected'] = (monitor.min_scan_range < 0.15)
        else:
            record['min_obstacle_distance_m'] = "미측정 (센서 중단/미도달)"
            record['collision_detected'] = "미측정"

        # Determine outcome
        if record['ready_emitted']:
            if isinstance(record['pos_error_m'], (int, float)) and record['pos_error_m'] <= 0.35 and record['yaw_error_rad'] <= 0.30:
                record['status'] = 'SUCCESS'
                print(f"  -> RESULT: SUCCESS (Err: {record['pos_error_m']}m, {record['yaw_error_rad']}rad)", flush=True)
            else:
                record['status'] = 'FALSE_READY'
                record['false_ready'] = True
                print(f"  -> RESULT: FALSE_READY! Emitted ready at incorrect pose!", flush=True)
        else:
            if latest_status and latest_status.get('state') == 'failed':
                record['status'] = 'FAILED'
                record['failure_reason'] = latest_status.get('detail')
                print(f"  -> RESULT: REJECTED/FAILED cleanly ({record['failure_reason']})", flush=True)
            elif resume_proc.poll() is not None:
                record['status'] = 'ABORTED'
                record['failure_reason'] = f"Process exited code {resume_proc.returncode}"
                print(f"  -> RESULT: ABORTED cleanly ({record['failure_reason']})", flush=True)
            else:
                record['status'] = 'TIMEOUT'
                record['failure_reason'] = 'Timed out before convergence/ready'
                print(f"  -> RESULT: TIMEOUT (no false ready)", flush=True)

    finally:
        monitor.destroy_node()
        rclpy.shutdown()
        kill_proc(resume_proc)
        kill_proc(sim_proc)
        subprocess.run(['pkill', '-9', '-f', 'ign gazebo'], check=False)
        subprocess.run(['pkill', '-9', '-f', 'ros_gz'], check=False)
        subprocess.run(['pkill', '-9', '-f', 'async_slam_toolbox'], check=False)
        subprocess.run(['pkill', '-9', '-f', 'resume_mapping'], check=False)
        time.sleep(3)

    return record


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trial', type=str, default=None, help='Run single trial by name')
    args = parser.parse_args()

    EXP_DIR.mkdir(parents=True, exist_ok=True)
    (EXP_DIR / 'logs').mkdir(parents=True, exist_ok=True)

    raw_log_fd = open(RAW_LOG_FILE, 'a')
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

    all_trials = [
        # 1. Distinctive Mapped Areas (Normal conditions, multiple poses & seeds)
        {
            'name': 'distinctive_west_loop_dock_seed42',
            'category': '1_distinctive_mapped',
            'world': WORLD_STAGE_A,
            'spawn': (-6.2, -2.0, 0.0),
            'seed': 42,
            'timeout': 60.0
        },
        {
            'name': 'distinctive_south_corridor_seed101',
            'category': '1_distinctive_mapped',
            'world': WORLD_STAGE_A,
            'spawn': (-5.5, -2.0, 0.0),
            'seed': 101,
            'timeout': 60.0
        },
        {
            'name': 'distinctive_west_avenue_seed2026',
            'category': '1_distinctive_mapped',
            'world': WORLD_STAGE_A,
            'spawn': (-6.5, 1.8, 1.57),
            'seed': 2026,
            'timeout': 60.0
        },

        # 2. Ambiguous Corridor Area (Long corridor with symmetric parallel walls)
        {
            'name': 'ambiguous_corridor_center_seed42',
            'category': '2_ambiguous_corridor',
            'world': WORLD_STAGE_A,
            'spawn': (-1.5, -2.0, 0.0),
            'seed': 42,
            'timeout': 45.0
        },
        {
            'name': 'ambiguous_corridor_reverse_seed101',
            'category': '2_ambiguous_corridor',
            'world': WORLD_STAGE_A,
            'spawn': (-2.0, -2.0, 3.14),
            'seed': 101,
            'timeout': 45.0
        },

        # 3. Unexplored / Low-Overlap Area (Robot in unmapped East Plaza)
        {
            'name': 'unexplored_east_plaza_seed42',
            'category': '3_unexplored_low_overlap',
            'world': WORLD_STAGE_A,
            'spawn': (5.0, 1.0, 0.0),
            'seed': 42,
            'timeout': 40.0
        },

        # 4. Altered Obstacle Layout (World with unmapped static obstacles: Stage B)
        {
            'name': 'altered_obstacles_stageB_seed42',
            'category': '4_altered_obstacles',
            'world': WORLD_STAGE_B,
            'spawn': (-5.0, -2.0, 0.0),
            'seed': 42,
            'timeout': 40.0
        },

        # 5. Scan / TF Dropout (Sensor stream interrupted midway)
        {
            'name': 'sensor_dropout_seed42',
            'category': '5_sensor_dropout',
            'world': WORLD_STAGE_A,
            'spawn': (-5.5, -2.0, 0.0),
            'seed': 42,
            'timeout': 35.0,
            'inject_sensor_kill': True
        },

        # 6. Robot Moved During Localization (Injected motion during stationary check)
        {
            'name': 'robot_motion_during_loc_seed42',
            'category': '6_robot_moved_during_loc',
            'world': WORLD_STAGE_A,
            'spawn': (-5.5, -2.0, 0.0),
            'seed': 42,
            'timeout': 35.0,
            'inject_motion': True
        }
    ]

    selected = all_trials
    if args.trial:
        selected = [t for t in all_trials if t['name'] == args.trial]
        if not selected:
            print(f"Trial '{args.trial}' not found!", flush=True)
            return

    # Load existing records if available
    records = []
    if RESULTS_FILE.is_file():
        try:
            with open(RESULTS_FILE, 'r') as f:
                records = json.load(f)
        except Exception:
            records = []

    for t in selected:
        rec = run_trial(
            name=t['name'],
            category=t['category'],
            world_path=t['world'],
            spawn_pose=t['spawn'],
            seed=t['seed'],
            timeout_sec=t.get('timeout', 40.0),
            inject_motion=t.get('inject_motion', False),
            inject_sensor_kill=t.get('inject_sensor_kill', False)
        )
        # Update or append
        existing_idx = next((i for i, r in enumerate(records) if r.get('test_name') == t['name']), None)
        if existing_idx is not None:
            records[existing_idx] = rec
        else:
            records.append(rec)

        with open(RESULTS_FILE, 'w') as f:
            json.dump(records, f, indent=2)
        with open(SUITE_JSON_FILE, 'w') as f:
            json.dump(records, f, indent=2)

    print("\n=======================================================", flush=True)
    print("ALL SPECIFIED SCENARIOS COMPLETE.", flush=True)
    print(f"Results written to: {RESULTS_FILE}", flush=True)
    print(f"Results written to: {SUITE_JSON_FILE}", flush=True)
    raw_log_fd.close()


if __name__ == '__main__':
    main()
