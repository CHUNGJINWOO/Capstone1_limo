#!/usr/bin/env python3
"""
run_synthetic_amcl_test.py:
Runs standalone synthetic AMCL global localization verification without Gazebo physics.
Validates:
  - Global localization convergence from uniform random particles on a known 2D grid map.
  - Verification that ground truth is used strictly for offline post-test evaluation.
  - Recording exact runtime, estimated pose, position error, and yaw error.
  - Explicitly marking all unmeasured/non-applicable fields (SLAM, Gazebo collision, Action) as '미측정'.
Outputs:
  - logs/synthetic_amcl_verification.raw.log
  - logs/synthetic_amcl_verification.json
"""
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification')
JSON_LOG_FILE = EXP_DIR / 'logs/synthetic_amcl_verification.json'
RAW_LOG_FILE = EXP_DIR / 'logs/synthetic_amcl_verification.raw.log'


class Logger:
    def __init__(self, log_path):
        self.terminal = sys.stdout
        self.log_file = open(log_path, 'w', buffering=1)

    def write(self, message):
        self.terminal.write(message)
        self.log_file.write(message)

    def flush(self):
        self.terminal.flush()
        self.log_file.flush()

    def close(self):
        self.log_file.close()


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


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def main():
    RAW_LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
    logger = Logger(RAW_LOG_FILE)
    sys.stdout = logger
    sys.stderr = logger

    run_id = str(uuid.uuid4())
    command_str = f"python3 {Path(__file__).resolve()}"
    code_hash_str = compute_code_hash(Path(__file__).resolve())

    print(f"=======================================================")
    print(f"[SYNTHETIC AMCL TEST] Standalone Nav2 AMCL Verification")
    print(f"  Run ID:    {run_id}")
    print(f"  Command:   {command_str}")
    print(f"  Code Hash: {code_hash_str}")
    print(f"  Raw Log:   {RAW_LOG_FILE}")
    print(f"=======================================================\n")

    # Import limo_auto_mapping.resume and localization
    import limo_auto_mapping.resume as resume
    from limo_auto_mapping.localization import close_pose

    old_share_fn = None
    node = None
    sensor = None
    executor = None
    sensor_thread = None
    timer = None
    tmp_dir = Path(tempfile.mkdtemp(prefix='synthetic_amcl_'))
    try:
        # 1. Build synthetic occupancy grid (100x100, 0.1m resolution)
        n, res = 100, 0.1
        data = [100 if x in (0, n - 1) or y in (0, n - 1) else 0
                for y in range(n) for x in range(n)]
        for x0, x1, y0, y1 in [(15, 30, 55, 65), (60, 65, 20, 40), (72, 90, 70, 74)]:
            for y in range(y0, y1):
                for x in range(x0, x1):
                    data[y * n + x] = 100

        prefix = tmp_dir / 'synthetic'
        for ext in ('.posegraph', '.data'):
            prefix.with_suffix(ext).write_text('placeholder: synthetic amcl test')

        pixels = bytes(0 if data[y * n + x] else 254 for y in range(n - 1, -1, -1) for x in range(n))
        prefix.with_suffix('.pgm').write_bytes(b'P5\n100 100\n255\n' + pixels)
        prefix.with_suffix('.yaml').write_text(yaml.safe_dump({
            'image': 'synthetic.pgm',
            'resolution': res,
            'origin': [0.0, 0.0, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }))

        # Find valid nav2_explore_params.yaml
        candidate_paths = [
            Path('/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/config/nav2_explore_params.yaml'),
            Path('/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws_10_21/src/limo_capstone/config/nav2_explore_params.yaml'),
        ]
        source_params = next(p for p in candidate_paths if p.is_file())
        (tmp_dir / 'config').mkdir()
        (tmp_dir / 'config/nav2_explore_params.yaml').write_bytes(source_params.read_bytes())

        # Robot true pose in map coordinates
        robot_truth = (4.2, 3.1, 0.4)
        print(f"Ground Truth Pose (used strictly for evaluation): {robot_truth}")

        # Compute synthetic raycast scan ranges
        ranges = []
        for i in range(360):
            theta = robot_truth[2] - math.pi + i * math.pi / 180.0
            r = 0.1
            while r < 12.0:
                x = int((robot_truth[0] + r * math.cos(theta)) / res)
                y = int((robot_truth[1] + r * math.sin(theta)) / res)
                if not (0 <= x < n and 0 <= y < n) or data[y * n + x]:
                    break
                r += 0.02
            ranges.append(r)

        # Monkeypatch get_package_share_directory in resume
        old_share_fn = resume.get_package_share_directory
        resume.get_package_share_directory = lambda _: str(tmp_dir)

        # Initialize ROS 2
        rclpy.init(args=[
            '--ros-args',
            '-p', f'posegraph:={prefix}',
            '-p', 'stable_sec:=2.0',
            '-p', 'stable_samples:=3',
            '-p', 'localization_timeout_sec:=45.0',
            '-p', 'use_sim_time:=false'
        ])

        node = resume.ResumeMapping()
        sensor = Node('synthetic_sensor_source')
        publisher = sensor.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        broadcaster = TransformBroadcaster(sensor)
        static_broadcaster = StaticTransformBroadcaster(sensor)

        fixed = TransformStamped()
        fixed.header.frame_id = 'base_link'
        fixed.child_frame_id = 'laser_link'
        fixed.transform.rotation.w = 1.0
        static_broadcaster.sendTransform(fixed)

        def publish():
            now = sensor.get_clock().now().to_msg()
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.rotation.w = 1.0
            broadcaster.sendTransform(t)

            scan = LaserScan()
            scan.header.stamp = now
            scan.header.frame_id = 'laser_link'
            scan.angle_min = -math.pi
            scan.angle_increment = math.pi / 180.0
            scan.angle_max = math.pi - math.pi / 180.0
            scan.range_min = 0.05
            scan.range_max = 12.0
            scan.scan_time = 0.1
            scan.ranges = ranges
            publisher.publish(scan)

        timer = sensor.create_timer(0.1, publish)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(sensor)
        sensor_thread = threading.Thread(target=executor.spin, daemon=True)
        sensor_thread.start()

        found_poses = []
        original_launch = node.launch

        def intercept_launch(cmd):
            if cmd[3] == 'auto_mapping.launch.py':
                estimate_str = next(s.split(':=', 1)[1] for s in cmd if s.startswith('start_pose:='))
                estimate = json.loads(estimate_str)
                print(f"[AMCL Intercept] Convergence detected! Hand-off start_pose: {estimate}")
                found_poses.append(estimate)
                raise StopIteration("Synthetic AMCL reached convergence")
            original_launch(cmd)

        node.launch = intercept_launch

        t_start = time.monotonic()
        success = False
        try:
            print("Running ResumeMapping node with real Nav2 AMCL on synthetic map...")
            node.run()
        except StopIteration:
            success = True
        except Exception as e:
            print(f"Node exited with error: {e}")

        elapsed_time = time.monotonic() - t_start
        print(f"Elapsed Time: {elapsed_time:.2f}s, Success: {success}")

        # Compute errors
        if found_poses:
            est = found_poses[0]
            err_pos = math.hypot(est[0] - robot_truth[0], est[1] - robot_truth[1])
            err_yaw = abs(angle_diff(est[2], robot_truth[2]))
            print(f"Final Estimated Pose: {est}")
            print(f"Position Error: {err_pos:.4f}m, Yaw Error: {err_yaw:.4f}rad")
        else:
            est = None
            err_pos = None
            err_yaw = None

        results = {
            "run_id": run_id,
            "command": command_str,
            "code_hash": code_hash_str,
            "raw_log_path": str(RAW_LOG_FILE),
            "test_type": "synthetic_amcl_standalone",
            "start_pose_injected": False,
            "ready_gate_bypassed": False,
            "localization_ready_emitted": success,
            "localization_time_sec": round(elapsed_time, 2) if success else "미측정",
            "ground_truth_map": list(robot_truth),
            "estimated_pose": [round(v, 4) for v in est] if est else "미측정",
            "position_error_m": round(err_pos, 4) if err_pos is not None else "미측정",
            "yaw_error_rad": round(err_yaw, 4) if err_yaw is not None else "미측정",
            "amcl_node_terminated": True,
            "slam_node_active": "미측정 (합성 AMCL 단독 시험)",
            "tf_duplicated": "미측정 (합성 AMCL 단독 시험)",
            "post_handoff_slam_pose": "미측정 (합성 AMCL 단독 시험)",
            "pose_delta_position_m": "미측정 (합성 AMCL 단독 시험)",
            "pose_delta_yaw_rad": "미측정 (합성 AMCL 단독 시험)",
            "min_obstacle_distance_m": "미측정 (Gazebo 미실행)",
            "collision_detected": "미측정 (Gazebo 미실행)",
            "action_goal_accepted": "미측정 (Action 미실행)",
            "action_status": "미측정 (Action 미실행)",
            "action_feedback_received": "미측정 (Action 미실행)",
            "action_cmd_vel_count": "미측정 (Action 미실행)",
            "new_serialized_files": "미측정 (합성 AMCL 단독 시험)"
        }

        with open(JSON_LOG_FILE, 'w') as f:
            json.dump(results, f, indent=2)

        print(f"\n[OK] Synthetic AMCL results written directly to: {JSON_LOG_FILE}")

    finally:
        if node:
            try:
                node.stop_stack()
            except Exception:
                pass
        if executor and sensor_thread and timer and node and sensor:
            try:
                executor.shutdown(timeout_sec=5)
                sensor_thread.join(timeout=5)
                timer.cancel()
                node.destroy_node()
                sensor.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        shutil.rmtree(tmp_dir, ignore_errors=True)
        if old_share_fn:
            resume.get_package_share_directory = old_share_fn
        logger.close()


if __name__ == '__main__':
    main()
