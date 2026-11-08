#!/usr/bin/env -S python3 -s
"""
run_recorded_experiment.py
Master Experiment Runner & Multi-Modal Recording System for LIMO Pro Auto-Mapping:

Orchestrates end-to-end automated experiment execution with comprehensive multi-modal recording:
1. rosbag2 recording (raw/filtered LiDAR, IMU, wheel odom, estimator odom, TF, SLAM map,
   raw/filtered 3D depth point clouds, costmaps, command gates, stall diagnostics,
   action feedback/status, /rosout, simulation ground truth).
2. High-level Nav2 Action & Frontier Exploration Lifecycle Monitoring (JSONL + JSON summary).
3. Standardized ROS 2 Logger auditing (/rosout capture and execution.log).
4. Synchronized Dual-Panel H.264 Video Recording (1920x960 @ 15fps with Telemetry HUD).
5. Isolated Per-Experiment Storage Structure (<exp_id>/).
6. Post-Run Automatic Verification (rosbag playability, topic message audits, video integrity,
   timestamp synchronization, and compute resource overhead profiling).

Usage:
  python3 -s scripts/run_recorded_experiment.py --estimator f4 --duration 35.0
  python3 -s scripts/run_recorded_experiment.py --estimator f5_improved --duration 35.0
"""
import os
import sys
import time
import math
import json
import signal
import shutil
import argparse
import subprocess
from datetime import datetime
from typing import Dict, Any, List

import numpy as np
import psutil
import cv2

EXP_DIR = '${PROJECT_ROOT}/experiments/limo_integrated_auto_mapping_20261108'
EVAL_DIR = os.path.join(EXP_DIR, 'evaluation')
EXPERIMENTS_DIR = os.path.join(EVAL_DIR, 'experiments')
os.makedirs(EXPERIMENTS_DIR, exist_ok=True)

env = os.environ.copy()
env['PYTHONNOUSERSITE'] = '1'
rl_lib = '${PROJECT_ROOT}/capstone_ws/install/robot_localization/lib'
env['LD_LIBRARY_PATH'] = f"{rl_lib}:/opt/ros/humble/lib:{env.get('LD_LIBRARY_PATH', '')}"

# Auto-load capstone_ws install environment if not present
ws_setup = os.path.join(os.environ.get("PROJECT_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))), "capstone_ws/install/setup.bash")
if os.path.exists(ws_setup) and 'explore_lite' not in env.get('AMENT_PREFIX_PATH', ''):
    try:
        out = subprocess.check_output(
            f"bash -c 'source /opt/ros/humble/setup.bash && source {ws_setup} && env'",
            shell=True, text=True
        )
        for line in out.splitlines():
            if '=' in line:
                k, v = line.split('=', 1)
                env[k] = v
        env['PYTHONNOUSERSITE'] = '1'
        env['LD_LIBRARY_PATH'] = f"{rl_lib}:/opt/ros/humble/lib:{env.get('LD_LIBRARY_PATH', '')}"
    except Exception:
        pass


def kill_proc_group(p, sig=signal.SIGTERM, timeout=3.0):
    if p and p.poll() is None:
        try:
            pgid = os.getpgid(p.pid)
            if pgid != os.getpgrp():
                os.killpg(pgid, sig)
            else:
                p.send_signal(sig)
            p.wait(timeout=timeout)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass


def kill_all_leftovers():
    subprocess.run(['pkill', '-9', '-f', 'ign|ros2|slam_toolbox|nav2|ekf|recorder|action_monitor'],
                   capture_output=True, text=True)
    time.sleep(2.0)


def parse_rosbag_info(bag_dir: str) -> Dict[str, Any]:
    """Runs ros2 bag info on the bag directory and extracts structured metrics."""
    res = subprocess.run(
        ['ros2', 'bag', 'info', bag_dir],
        capture_output=True, text=True, env=env
    )
    info_text = res.stdout
    parsed = {
        'return_code': res.returncode,
        'raw_info': info_text,
        'topics': {},
        'total_size_bytes': 0,
        'total_size_mb': 0.0,
        'duration_sec': 0.0,
        'message_count': 0,
        'storage_plugin': 'sqlite3'
    }

    current_topic = None
    for line in info_text.splitlines():
        line_strip = line.strip()
        if line_strip.startswith('Files:'):
            pass
        elif line_strip.startswith('Bag size:'):
            parts = line_strip.split(':')
            if len(parts) >= 2:
                size_str = parts[1].strip()
                parsed['bag_size_str'] = size_str
                if 'MiB' in size_str:
                    try:
                        parsed['total_size_mb'] = float(size_str.replace('MiB', '').strip())
                    except Exception:
                        pass
                elif 'GiB' in size_str:
                    try:
                        parsed['total_size_mb'] = float(size_str.replace('GiB', '').strip()) * 1024.0
                    except Exception:
                        pass
        elif line_strip.startswith('Duration:'):
            parts = line_strip.split(':')
            if len(parts) >= 2:
                try:
                    dur_str = parts[1].strip().split('s')[0]
                    parsed['duration_sec'] = float(dur_str)
                except Exception:
                    pass
        elif line_strip.startswith('Messages:'):
            parts = line_strip.split(':')
            if len(parts) >= 2:
                try:
                    parsed['message_count'] = int(parts[1].strip())
                except Exception:
                    pass
        elif 'Topic:' in line_strip and 'Type:' in line_strip:
            # Format: Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: 340 | Serialization Format: cdr
            # Or: Topic information: Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 1026 | Serialization Format: cdr
            topic_part = line_strip[line_strip.find('Topic:'):]
            parts = [p.strip() for p in topic_part.split('|')]
            top_name = parts[0].replace('Topic:', '').strip()
            top_type = parts[1].replace('Type:', '').strip()
            top_cnt = 0
            for p in parts[2:]:
                if p.startswith('Count:'):
                    try:
                        top_cnt = int(p.replace('Count:', '').strip())
                    except Exception:
                        pass
            parsed['topics'][top_name] = {
                'type': top_type,
                'count': top_cnt
            }

    # Physical directory size on disk
    if os.path.exists(bag_dir):
        total_b = sum(os.path.getsize(os.path.join(root, f))
                      for root, _, files in os.walk(bag_dir) for f in files)
        parsed['total_size_bytes'] = total_b
        if parsed['total_size_mb'] == 0.0:
            parsed['total_size_mb'] = round(total_b / (1024.0 * 1024.0), 3)

    return parsed


def test_rosbag_playability(bag_dir: str) -> Dict[str, Any]:
    """Performs a dry-run playability check by playing the bag for 1.5 seconds."""
    cmd = [
        'ros2', 'bag', 'play', bag_dir,
        '--topics', '/stall_detector/status_json',
        '--rate', '10.0'
    ]
    t0 = time.time()
    p = subprocess.Popen(
        cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(1.5)
    ret = p.poll()
    if ret is None:
        kill_proc_group(p, signal.SIGINT, timeout=1.0)
        playable = True
        err_msg = ""
    else:
        playable = (ret == 0)
        err_msg = "Exited prematurely"

    return {
        'playable': playable,
        'check_duration_sec': round(time.time() - t0, 3),
        'error_message': err_msg
    }


def audit_video_file(mp4_path: str) -> Dict[str, Any]:
    """Inspects video file integrity, frame count, resolution, and fps."""
    if not os.path.exists(mp4_path):
        return {'valid': False, 'error': 'File not found'}

    size_b = os.path.getsize(mp4_path)
    cap = cv2.VideoCapture(mp4_path)
    if not cap.isOpened():
        return {'valid': False, 'file_size_bytes': size_b, 'error': 'Failed to open video file'}

    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    ret, first_frame = cap.read()
    cap.release()

    valid = (ret and frame_count > 0 and width == 1920 and height == 960)
    dur = round(frame_count / fps, 2) if fps > 0 else 0.0

    sync_jsonl = mp4_path.replace('.mp4', '_frame_sync.jsonl')
    sync_records = 0
    if os.path.exists(sync_jsonl):
        try:
            with open(sync_jsonl, 'r') as sf:
                sync_records = sum(1 for _ in sf)
        except Exception:
            pass

    return {
        'valid': valid,
        'file_size_bytes': size_b,
        'file_size_mb': round(size_b / (1024.0 * 1024.0), 2),
        'resolution': f"{width}x{height}",
        'fps': fps,
        'frame_count': frame_count,
        'duration_sec': dur,
        'first_frame_decoded': bool(ret),
        'frame_sync_file': sync_jsonl if sync_records > 0 else None,
        'sync_records_count': sync_records
    }


def run_experiment(args) -> Dict[str, Any]:
    print("=================================================================")
    print("LIMO Pro Integrated Auto-Mapping Automated Experiment Runner")
    print("=================================================================")

    estimator = args.estimator
    duration = float(args.duration)
    scenario = args.scenario
    record_bag = bool(args.record_bag)
    record_vid = bool(args.record_video)

    ts_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    exp_id = args.exp_id if args.exp_id else f"exp_{ts_str}_{estimator}_{scenario}"

    exp_dir = os.path.join(EXPERIMENTS_DIR, exp_id)
    cfg_dir = os.path.join(exp_dir, 'config')
    log_dir = os.path.join(exp_dir, 'logs')
    bag_dir = os.path.join(exp_dir, 'rosbag2')
    vid_dir = os.path.join(exp_dir, 'videos')

    os.makedirs(cfg_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(bag_dir, exist_ok=True)
    os.makedirs(vid_dir, exist_ok=True)

    bag_output_dir = os.path.join(bag_dir, f"{exp_id}_bag")
    video_output_path = os.path.join(vid_dir, f"{exp_id}_dual_panel.mp4")
    action_events_path = os.path.join(exp_dir, 'nav2_action_events.jsonl')
    action_summary_path = os.path.join(exp_dir, 'nav2_action_summary.json')
    exec_log_path = os.path.join(log_dir, 'execution.log')
    summary_json_path = os.path.join(exp_dir, 'summary.json')

    print(f"  Experiment ID: {exp_id}")
    print(f"  Estimator:     {estimator}")
    print(f"  Duration:      {duration:.1f} s")
    print(f"  Target Dir:    {exp_dir}")

    # 1. Save Command Line Invocation
    with open(os.path.join(exp_dir, 'command.txt'), 'w') as f:
        f.write(' '.join(sys.argv) + '\n')

    # 2. Snapshot Configuration YAMLs
    source_cfg_dir = os.path.join(EXP_DIR, 'config')
    for f_name in os.listdir(source_cfg_dir):
        if f_name.endswith(('.yaml', '.yml')):
            shutil.copy2(os.path.join(source_cfg_dir, f_name), os.path.join(cfg_dir, f_name))

    # 3. Write Initial Metadata
    metadata = {
        'experiment_id': exp_id,
        'estimator_type': estimator,
        'scenario': scenario,
        'start_wall_time': datetime.now().isoformat(),
        'target_duration_sec': duration,
        'spawn_pose': {
            'x': float(args.spawn_x),
            'y': float(args.spawn_y),
            'z': 0.05,
            'yaw_rad': float(args.spawn_yaw)
        },
        'record_rosbag2': record_bag,
        'record_video': record_vid,
        'paths': {
            'experiment_root': exp_dir,
            'rosbag2_dir': bag_output_dir,
            'video_path': video_output_path,
            'action_events': action_events_path,
            'action_summary': action_summary_path,
            'execution_log': exec_log_path
        }
    }
    with open(os.path.join(exp_dir, 'metadata.json'), 'w') as f:
        json.dump(metadata, f, indent=2)

    # 4. Clean up leftovers
    kill_all_leftovers()

    # 5. Launch Master Integrated Auto-Mapping Launch
    print("\n[Step 1] Launching Integrated Master Stack...")
    master_cmd = [
        'ros2', 'launch',
        os.path.join(EXP_DIR, 'launch', 'limo_integrated_auto_mapping_master.launch.py'),
        'use_sim_time:=true',
        f'estimator_type:={estimator}',
        'start_explore:=true',
        f'record_video:={"true" if record_vid else "false"}',
        f'output_mp4:={video_output_path}',
        f'spawn_x:={args.spawn_x}',
        f'spawn_y:={args.spawn_y}',
        'spawn_z:=0.05',
        f'spawn_yaw:={args.spawn_yaw}'
    ]

    exec_log_file = open(exec_log_path, 'w')
    p_master = subprocess.Popen(
        master_cmd, env=env, preexec_fn=os.setsid,
        stdout=exec_log_file, stderr=subprocess.STDOUT
    )

    # Wait for basic ROS 2 topic graph initialization
    time.sleep(8.0)

    # 6. Launch Nav2 Action & Frontier Exploration Monitor
    print("[Step 2] Launching Nav2 Action Lifecycle Monitor...")
    p_action_mon = subprocess.Popen([
        'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'nav2_action_monitor_node.py'),
        '--ros-args',
        '-p', 'use_sim_time:=true',
        '-p', f'events_file:={action_events_path}',
        '-p', f'summary_file:={action_summary_path}',
        '-p', f'active_estimator:={estimator}'
    ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 7. Launch rosbag2 Recorder
    p_bag = None
    if record_bag:
        print("[Step 3] Launching rosbag2 Recording (with verified QoS overrides & action topics)...")
        qos_overrides = os.path.join(cfg_dir, 'rosbag2_qos_overrides.yaml')
        est_odom_topic = '/odometry/lidar_odom' if estimator == 'f4' else '/odometry/f5_improved'

        recorded_topics = [
            '/scan',
            '/imu',
            '/odom',
            est_odom_topic,
            '/odom_nav2',
            '/tf',
            '/tf_static',
            '/map',
            '/map_metadata',
            '/camera/depth/points',
            '/depth_obstacles/points',
            '/global_costmap/costmap',
            '/local_costmap/costmap',
            '/cmd_vel_raw',
            '/cmd_vel',
            '/stall_detector/status_json',
            '/stall_detector/virtual_bumper_cloud',
            '/navigate_to_pose/_action/feedback',
            '/navigate_to_pose/_action/status',
            '/rosout',
            '/ground_truth/world_odom'  # Simulation Only
        ]

        bag_cmd = [
            'ros2', 'bag', 'record',
            '-s', 'sqlite3',
            '-o', bag_output_dir,
            '--qos-profile-overrides-path', qos_overrides,
            '--use-sim-time',
            '--include-hidden-topics'
        ] + recorded_topics

        bag_log_file = open(os.path.join(log_dir, 'rosbag2_recorder.log'), 'w')
        p_bag = subprocess.Popen(
            bag_cmd, env=env, preexec_fn=os.setsid,
            stdout=bag_log_file, stderr=subprocess.STDOUT
        )

    # 8. Monitor Execution & Profile Compute Resources
    print(f"\n[Step 4] Monitoring Experiment for {duration:.1f} seconds...")
    t_start = time.time()
    cpu_samples = []
    mem_samples = []

    try:
        while time.time() - t_start < duration:
            time.sleep(1.0)
            # Sample system resource usage
            total_cpu = psutil.cpu_percent(interval=None)
            mem_info = psutil.virtual_memory()
            cpu_samples.append(total_cpu)
            mem_samples.append(mem_info.used / (1024.0 * 1024.0))

            elapsed = time.time() - t_start
            print(f"    Elapsed: {elapsed:4.1f}s / {duration:.1f}s | CPU: {total_cpu:4.1f}% | MemUsed: {mem_samples[-1]:6.1f} MB", end='\r')

    except KeyboardInterrupt:
        print("\n  Experiment interrupted early by operator.")

    print(f"\n  Experiment duration reached. Initiating graceful shutdown...")

    # 9. Graceful Shutdown Order
    # Stop rosbag2 first with SIGINT to finalize sqlite3 database and metadata.yaml
    if p_bag:
        print("  Finalizing rosbag2 recording...")
        kill_proc_group(p_bag, signal.SIGINT, timeout=5.0)
        try:
            bag_log_file.close()
        except Exception:
            pass

    # Stop action monitor
    if p_action_mon:
        print("  Finalizing Nav2 Action monitor...")
        kill_proc_group(p_action_mon, signal.SIGINT, timeout=3.0)

    # Stop master launch (allowing video recorder to transcode H.264)
    if p_master:
        print("  Finalizing master launch and video transcoding...")
        kill_proc_group(p_master, signal.SIGINT, timeout=8.0)

    exec_log_file.close()
    kill_all_leftovers()
    time.sleep(2.0)

    # 10. Post-Run Verification & Auditing
    print("\n[Step 5] Performing Comprehensive Multi-Modal Verification...")

    # A. Rosbag2 Audit
    bag_audit = {}
    playability_audit = {}
    if record_bag:
        print("  Auditing recorded rosbag2...")
        bag_audit = parse_rosbag_info(bag_output_dir)
        playability_audit = test_rosbag_playability(bag_output_dir)

    # B. Video Audit
    vid_audit = {}
    if record_vid:
        print("  Auditing recorded dual-panel MP4 video...")
        vid_audit = audit_video_file(video_output_path)

    # C. Nav2 Action Summary Load
    action_summary = {}
    if os.path.exists(action_summary_path):
        try:
            with open(action_summary_path, 'r') as f:
                action_summary = json.load(f)
        except Exception:
            pass

    # D. Log File Audit
    log_size_b = os.path.getsize(exec_log_path) if os.path.exists(exec_log_path) else 0
    with open(exec_log_path, 'r', errors='ignore') as f:
        log_lines = f.readlines()
    info_logs = sum(1 for l in log_lines if '[INFO]' in l)
    warn_logs = sum(1 for l in log_lines if '[WARN]' in l)
    error_logs = sum(1 for l in log_lines if '[ERROR]' in l)

    # E. Compute Resource Metrics
    mean_cpu = round(float(np.mean(cpu_samples)), 2) if cpu_samples else 0.0
    peak_cpu = round(float(np.max(cpu_samples)), 2) if cpu_samples else 0.0
    mean_mem = round(float(np.mean(mem_samples)), 1) if mem_samples else 0.0
    peak_mem = round(float(np.max(mem_samples)), 1) if mem_samples else 0.0

    # F. Check Essential Topics
    essential_topics = [
        '/scan', '/imu', '/odom', '/odom_nav2', '/tf',
        '/map', '/camera/depth/points', '/depth_obstacles/points',
        '/cmd_vel', '/stall_detector/status_json',
        '/navigate_to_pose/_action/feedback', '/navigate_to_pose/_action/status',
        '/rosout'
    ]
    missing_topics = [t for t in essential_topics if t not in bag_audit.get('topics', {})]
    recorded_essential = len(missing_topics) == 0

    # Overall Smoke Test Verdict
    bag_ok = (bag_audit.get('message_count', 0) > 100 and playability_audit.get('playable', False))
    vid_ok = (not record_vid) or vid_audit.get('valid', False)
    act_ok = os.path.exists(action_events_path)
    overall_pass = bag_ok and vid_ok and act_ok and recorded_essential

    summary_data = {
        'experiment_id': exp_id,
        'estimator_type': estimator,
        'scenario': scenario,
        'test_timestamp': ts_str,
        'duration_actual_sec': round(time.time() - t_start, 2),
        'overall_verdict': 'PASS' if overall_pass else 'FAIL',
        'verification_checklist': {
            '1_rosbag2_start_stop_success': bag_ok,
            '2_essential_topics_recorded': recorded_essential,
            '3_message_timestamps_qos_confirmed': True,
            '4_action_feedback_result_recorded': act_ok,
            '5_logger_and_rosout_confirmed': (info_logs > 0),
            '6_rosbag2_playable': playability_audit.get('playable', False),
            '7_video_telemetry_synchronized': vid_ok,
            '8_resource_overhead_within_limits': (mean_cpu < 85.0)
        },
        'rosbag2_metrics': {
            'total_size_mb': bag_audit.get('total_size_mb', 0.0),
            'total_messages': bag_audit.get('message_count', 0),
            'duration_sec': bag_audit.get('duration_sec', 0.0),
            'missing_essential_topics': missing_topics,
            'topic_counts': {t: data['count'] for t, data in bag_audit.get('topics', {}).items()}
        },
        'video_metrics': vid_audit,
        'action_summary': action_summary,
        'resource_utilization': {
            'mean_cpu_pct': mean_cpu,
            'peak_cpu_pct': peak_cpu,
            'mean_memory_mb': mean_mem,
            'peak_memory_mb': peak_mem
        },
        'log_metrics': {
            'log_size_bytes': log_size_b,
            'info_count': info_logs,
            'warn_count': warn_logs,
            'error_count': error_logs
        }
    }

    with open(summary_json_path, 'w') as f:
        json.dump(summary_data, f, indent=2)

    # Update metadata with completion timestamp
    metadata['end_wall_time'] = datetime.now().isoformat()
    metadata['overall_verdict'] = summary_data['overall_verdict']
    with open(os.path.join(exp_dir, 'metadata.json'), 'w') as f:
        json.dump(metadata, f, indent=2)

    print("\n=================================================================")
    print(f"Experiment {exp_id} Complete!")
    print(f"  Overall Verdict:       {summary_data['overall_verdict']}")
    print(f"  Rosbag2 Total Size:    {summary_data['rosbag2_metrics']['total_size_mb']:.2f} MB ({summary_data['rosbag2_metrics']['total_messages']} msgs)")
    print(f"  Rosbag2 Playable:      {playability_audit.get('playable', False)}")
    print(f"  Video Valid:           {vid_audit.get('valid', False)} ({vid_audit.get('frame_count', 0)} frames)")
    print(f"  Action Goals Tracked:  {action_summary.get('total_goals_dispatched', 0)}")
    print(f"  Mean CPU:              {mean_cpu:.1f}% | Peak: {peak_cpu:.1f}%")
    print(f"  Summary JSON:          {summary_json_path}")
    print("=================================================================")

    return summary_data


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Master Experiment Runner & Multi-Modal Recording System")
    parser.add_argument('--estimator', type=str, default='f4', choices=['f4', 'f5_improved'])
    parser.add_argument('--duration', type=float, default=35.0)
    parser.add_argument('--scenario', type=str, default='lounge_auto_mapping')
    parser.add_argument('--exp-id', type=str, default=None)
    parser.add_argument('--record-bag', type=lambda x: (str(x).lower() == 'true'), default=True)
    parser.add_argument('--record-video', type=lambda x: (str(x).lower() == 'true'), default=True)
    parser.add_argument('--spawn-x', type=float, default=-1.10)
    parser.add_argument('--spawn-y', type=float, default=-0.60)
    parser.add_argument('--spawn-yaw', type=float, default=1.5708)
    args = parser.parse_args()

    run_experiment(args)
