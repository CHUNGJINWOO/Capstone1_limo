#!/usr/bin/env -S python3 -s
"""
run_long_term_mapping_evaluation.py
Comprehensive Long-Term Autonomous Mapping Evaluation (F4 vs F5_improved)
Duration: 80s per trial under identical environment conditions.

Evaluates:
- Traveled distance (m)
- Map coverage: occupied grid cells, free cells, total explored area (m^2)
- Map quality: wall thickness / sharpness standard deviation (sigma_wall)
- Stall events & safety gate states
- CPU & memory consumption (psutil)
- State estimation latency & publish frequency
- Video recording: dual-panel MP4 generation and integrity verification
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
import psutil
import numpy as np

import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

REPO_ROOT = os.environ.get('PROJECT_ROOT', os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..')))
EXP_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
EVAL_DIR = os.path.join(EXP_DIR, 'evaluation')
RESULTS_DIR = os.path.join(EVAL_DIR, 'results')
VIDEOS_DIR = os.path.join(EVAL_DIR, 'videos')
LOGS_DIR = os.path.join(EVAL_DIR, 'logs')

os.makedirs(RESULTS_DIR, exist_ok=True)
VIDEOS_DIR_EXIST = os.makedirs(VIDEOS_DIR, exist_ok=True)
LOGS_DIR_EXIST = os.makedirs(LOGS_DIR, exist_ok=True)

env = os.environ.copy()
env['PYTHONNOUSERSITE'] = '1'
rl_lib = os.path.join(REPO_ROOT, 'capstone_ws', 'install', 'robot_localization', 'lib')
env['LD_LIBRARY_PATH'] = f"{rl_lib}:{env.get('LD_LIBRARY_PATH', '')}"


class LongTermMappingMonitor(Node):
    def __init__(self, estimator_name):
        super().__init__(
            f'long_term_monitor_{estimator_name}',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.estimator_name = estimator_name
        qos_reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                  reliability=ReliabilityPolicy.RELIABLE)

        self.sub_gt = self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_gt, 20)
        self.sub_odom_nav2 = self.create_subscription(Odometry, '/odom_nav2', self.on_odom_nav2, 20)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.on_map, 5)
        self.sub_plan = self.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.sub_depth = self.create_subscription(PointCloud2, '/depth_obstacles/points', self.on_depth, qos_reliable)
        self.sub_cmd_raw = self.create_subscription(Twist, '/cmd_vel_raw', self.on_cmd_raw, 20)
        self.sub_cmd_out = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_out, 20)
        self.sub_stall = self.create_subscription(String, '/stall_detector/status_json', self.on_stall, 10)

        self.gt_history = []
        self.odom_history = []
        self.map_occupied_cells = 0
        self.map_free_cells = 0
        self.map_resolution = 0.05
        self.map_data = None
        self.map_info = None
        self.plans_received = 0
        self.depth_clouds_count = 0
        self.depth_points_total = 0
        self.cmd_raw_count = 0
        self.cmd_out_count = 0
        self.stall_events = 0
        self.latest_stall_state = 'NORMAL'
        self.odom_latencies = []

    def on_gt(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt_history.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))

    def on_odom_nav2(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_t = self.get_clock().now().nanoseconds * 1e-9
        if now_t > t:
            self.odom_latencies.append(now_t - t)
        self.odom_history.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))

    def on_map(self, msg: OccupancyGrid):
        arr = np.array(msg.data, dtype=np.int8)
        self.map_occupied_cells = int(np.sum(arr > 0))
        self.map_free_cells = int(np.sum(arr == 0))
        self.map_resolution = msg.info.resolution
        self.map_data = arr.reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def on_plan(self, msg: NavPath):
        if len(msg.poses) > 0:
            self.plans_received += 1

    def on_depth(self, msg: PointCloud2):
        self.depth_clouds_count += 1
        self.depth_points_total += msg.width * msg.height

    def on_cmd_raw(self, msg: Twist):
        self.cmd_raw_count += 1

    def on_cmd_out(self, msg: Twist):
        self.cmd_out_count += 1

    def on_stall(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.latest_stall_state = d.get('state', 'UNKNOWN')
            self.stall_events = d.get('stall_events', 0)
        except Exception:
            pass


def kill_proc(p):
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
            p.wait(timeout=4.0)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                p.wait(timeout=2.0)
            except Exception:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:
                    pass


def kill_all_leftovers():
    subprocess.run(['pkill', '-9', '-f', 'ign|ros2|slam_toolbox|nav2|ekf|recorder'],
                   capture_output=True, text=True)
    time.sleep(2.0)


def verify_mp4_integrity(mp4_path: str) -> dict:
    if not os.path.exists(mp4_path):
        return {'valid': False, 'error': 'File not found'}

    size_bytes = os.path.getsize(mp4_path)
    if size_bytes < 50000:
        return {'valid': False, 'error': f'File too small ({size_bytes} bytes)'}

    cap = cv2.VideoCapture(mp4_path)
    if not cap.isOpened():
        return {'valid': False, 'error': 'Could not open video with OpenCV'}

    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    duration_sec = frame_count / fps if fps > 0 else 0.0
    cap.release()

    return {
        'valid': (frame_count >= 150 and width == 1920 and height == 960),
        'file_size_bytes': size_bytes,
        'frame_count': frame_count,
        'fps': round(fps, 2),
        'resolution': f"{width}x{height}",
        'duration_sec': round(duration_sec, 2)
    }


def compute_wall_sharpness(map_data, map_info):
    """
    Computes wall sharpness metric sigma_wall.
    Finds occupied cells along the corridor wall (nominal x ~ -1.35m)
    and computes the standard deviation of x coordinates.
    Lower sigma means sharper wall with less ghosting/doubling.
    """
    if map_data is None or map_info is None:
        return 0.0, 0

    origin_x = map_info.origin.position.x
    origin_y = map_info.origin.position.y
    res = map_info.resolution

    # Find cells with occupancy > 50
    occ_indices = np.argwhere(map_data > 50)
    if len(occ_indices) == 0:
        return 0.0, 0

    # Convert to world coordinates
    world_y = origin_y + occ_indices[:, 0] * res
    world_x = origin_x + occ_indices[:, 1] * res

    # 1. Main dividing wall at y ~ -1.50m (x in [-3.0, 1.0])
    mask_y = (world_y >= -1.65) & (world_y <= -1.35) & (world_x >= -3.0) & (world_x <= 1.0)
    wall_y = world_y[mask_y]

    # 2. Vertical corridor wall/post at x ~ -1.35m (y in [-1.5, 1.5])
    mask_x = (world_x >= -1.55) & (world_x <= -1.15) & (world_y >= -1.5) & (world_y <= 1.5)
    wall_x = world_x[mask_x]

    sigmas = []
    total_pts = 0
    if len(wall_y) >= 5:
        sigmas.append(float(np.std(wall_y)))
        total_pts += len(wall_y)
    if len(wall_x) >= 5:
        sigmas.append(float(np.std(wall_x)))
        total_pts += len(wall_x)

    if not sigmas:
        return 0.0, 0

    return round(float(np.mean(sigmas)), 4), total_pts


def collect_cpu_memory_stats(pids):
    cpu_usages = []
    mem_usages = []
    for pid in pids:
        try:
            proc = psutil.Process(pid)
            cpu_usages.append(proc.cpu_percent(interval=None))
            mem_usages.append(proc.memory_info().rss / (1024 * 1024))
            for child in proc.children(recursive=True):
                try:
                    cpu_usages.append(child.cpu_percent(interval=None))
                    mem_usages.append(child.memory_info().rss / (1024 * 1024))
                except Exception:
                    pass
        except Exception:
            pass
    return sum(cpu_usages), sum(mem_usages)


def run_single_trial(estimator_type: str, duration_sec: float = 80.0) -> dict:
    print(f"\n=================================================================")
    print(f"Executing Long-Term Exploration Trial: Estimator={estimator_type.upper()}")
    print(f"Duration: {duration_sec}s | Spawn: (-1.10, -0.60, Yaw=+1.5708 rad)")
    print(f"=================================================================")

    kill_all_leftovers()

    video_filename = f"long_term_mapping_{estimator_type}.mp4"
    output_video_path = os.path.join(VIDEOS_DIR, video_filename)
    trial_log_path = os.path.join(LOGS_DIR, f"long_term_mapping_{estimator_type}_launch.log")

    cmd = [
        'ros2', 'launch',
        os.path.join(EXP_DIR, 'launch', 'limo_integrated_auto_mapping_master.launch.py'),
        'use_sim_time:=true',
        f'estimator_type:={estimator_type}',
        'start_explore:=true',
        'record_video:=true',
        f'output_mp4:={output_video_path}',
        'spawn_x:=-1.10',
        'spawn_y:=-0.60',
        'spawn_z:=0.05',
        'spawn_yaw:=1.5708'
    ]

    log_f = open(trial_log_path, 'w')
    p_master = subprocess.Popen(cmd, env=env, preexec_fn=os.setsid,
                                stdout=log_f, stderr=subprocess.STDOUT)

    print(f"  Launched system with PID {p_master.pid}. Initializing stack (10s)...")
    time.sleep(10.0)

    rclpy.init()
    monitor = LongTermMappingMonitor(estimator_type)

    cpu_samples = []
    mem_samples = []
    root_pid = p_master.pid

    t_start = time.time()
    last_print_sec = -1
    print(f"  Starting live telemetry logging for {duration_sec}s...")

    while time.time() - t_start < duration_sec:
        rclpy.spin_once(monitor, timeout_sec=0.05)
        elapsed = time.time() - t_start
        cur_sec = int(elapsed)

        if cur_sec % 2 == 0 and cur_sec != last_print_sec and cur_sec > 0:
            cpu_val, mem_val = collect_cpu_memory_stats([root_pid])
            cpu_samples.append(cpu_val)
            mem_samples.append(mem_val)

        if cur_sec % 10 == 0 and cur_sec != last_print_sec and cur_sec > 0:
            last_print_sec = cur_sec
            free_area = monitor.map_free_cells * (monitor.map_resolution ** 2)
            print(f"  [{elapsed:4.1f}s / {duration_sec:.0f}s] "
                  f"OccCells={monitor.map_occupied_cells} | "
                  f"FreeArea={free_area:.2f}m² | "
                  f"Plans={monitor.plans_received} | "
                  f"GateState={monitor.latest_stall_state} | "
                  f"Stalls={monitor.stall_events}")

    print("\n  Execution completed. Terminating trial & finalizing video...")
    try:
        os.killpg(os.getpgid(p_master.pid), signal.SIGINT)
        p_master.wait(timeout=6.0)
    except Exception:
        kill_proc(p_master)

    rclpy.shutdown()
    log_f.close()
    time.sleep(2.0)

    # Process raw video fallback
    raw_video = output_video_path.replace('.mp4', '_raw.mp4')
    if os.path.exists(raw_video) and (not os.path.exists(output_video_path) or os.path.getsize(output_video_path) == 0):
        import shutil
        if shutil.which('ffmpeg'):
            try:
                subprocess.run([
                    'ffmpeg', '-y', '-i', raw_video,
                    '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-preset', 'fast', '-crf', '22',
                    output_video_path
                ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                shutil.copyfile(raw_video, output_video_path)
        else:
            shutil.copyfile(raw_video, output_video_path)

    # Calculate metrics
    travel_dist = 0.0
    if len(monitor.gt_history) > 1:
        pts = np.array([(p[1], p[2]) for p in monitor.gt_history])
        diffs = np.diff(pts, axis=0)
        travel_dist = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))

    sigma_wall, wall_points_count = compute_wall_sharpness(monitor.map_data, monitor.map_info)
    free_space_area_m2 = monitor.map_free_cells * (monitor.map_resolution ** 2)
    total_explored_area_m2 = (monitor.map_occupied_cells + monitor.map_free_cells) * (monitor.map_resolution ** 2)

    video_audit = verify_mp4_integrity(output_video_path)
    mean_cpu = float(np.mean(cpu_samples)) if cpu_samples else 0.0
    peak_cpu = float(np.max(cpu_samples)) if cpu_samples else 0.0
    mean_mem = float(np.mean(mem_samples)) if mem_samples else 0.0
    peak_mem = float(np.max(mem_samples)) if mem_samples else 0.0

    mean_latency = float(np.mean(monitor.odom_latencies) * 1000.0) if monitor.odom_latencies else 0.0
    max_latency = float(np.max(monitor.odom_latencies) * 1000.0) if monitor.odom_latencies else 0.0

    print(f"\n  [Results for {estimator_type.upper()}]")
    print(f"    Traveled Distance: {travel_dist:.3f} m")
    print(f"    Occupied Grid Cells: {monitor.map_occupied_cells}")
    print(f"    Free Space Explored: {free_space_area_m2:.2f} m²")
    print(f"    Wall Sharpness (sigma_wall): {sigma_wall:.4f} m ({wall_points_count} points)")
    print(f"    Plans Computed: {monitor.plans_received}")
    print(f"    Stall Events: {monitor.stall_events}")
    print(f"    Mean / Peak CPU: {mean_cpu:.1f}% / {peak_cpu:.1f}%")
    print(f"    Mean Latency: {mean_latency:.1f} ms (Max: {max_latency:.1f} ms)")
    print(f"    Video: {video_audit.get('resolution')} @ {video_audit.get('fps')}fps, "
          f"{video_audit.get('frame_count')} frames, Valid={video_audit.get('valid')}")

    kill_all_leftovers()

    return {
        'estimator_type': estimator_type,
        'trial_duration_sec': duration_sec,
        'traveled_distance_m': round(travel_dist, 3),
        'map_metrics': {
            'occupied_cells_count': monitor.map_occupied_cells,
            'free_cells_count': monitor.map_free_cells,
            'free_space_area_m2': round(free_space_area_m2, 3),
            'total_explored_area_m2': round(total_explored_area_m2, 3),
            'wall_sharpness_sigma_m': sigma_wall,
            'wall_points_evaluated': wall_points_count,
            'grid_resolution_m': monitor.map_resolution
        },
        'system_performance': {
            'mean_cpu_percent': round(mean_cpu, 1),
            'peak_cpu_percent': round(peak_cpu, 1),
            'mean_memory_mb': round(mean_mem, 1),
            'peak_memory_mb': round(peak_mem, 1),
            'mean_estimation_latency_ms': round(mean_latency, 2),
            'max_estimation_latency_ms': round(max_latency, 2)
        },
        'navigation_and_safety': {
            'plans_computed_count': monitor.plans_received,
            'depth_clouds_processed': monitor.depth_clouds_count,
            'commands_dispatched_raw': monitor.cmd_raw_count,
            'commands_dispatched_out': monitor.cmd_out_count,
            'stall_events_count': monitor.stall_events,
            'terminal_safety_state': monitor.latest_stall_state
        },
        'video_audit': video_audit,
        'video_file_path': output_video_path
    }


def main():
    print("=================================================================")
    print("LIMO Pro Extended Auto-Mapping Evaluation: F4 vs F5_improved")
    print("=================================================================")

    trial_f4 = run_single_trial('f4', duration_sec=80.0)
    time.sleep(3.0)
    trial_f5 = run_single_trial('f5_improved', duration_sec=80.0)

    # Comparative evaluation
    comparison = {
        'evaluation_timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'environment': 'Lounge World (Fortress)',
        'duration_sec_per_trial': 80.0,
        'trial_f4': trial_f4,
        'trial_f5_improved': trial_f5,
        'comparison_summary': {
            'distance_f4_vs_f5': {
                'f4_m': trial_f4['traveled_distance_m'],
                'f5_m': trial_f5['traveled_distance_m'],
                'difference_m': round(trial_f4['traveled_distance_m'] - trial_f5['traveled_distance_m'], 3)
            },
            'map_coverage_f4_vs_f5': {
                'f4_occupied_cells': trial_f4['map_metrics']['occupied_cells_count'],
                'f5_occupied_cells': trial_f5['map_metrics']['occupied_cells_count'],
                'f4_free_area_m2': trial_f4['map_metrics']['free_space_area_m2'],
                'f5_free_area_m2': trial_f5['map_metrics']['free_space_area_m2']
            },
            'wall_sharpness_f4_vs_f5': {
                'f4_sigma_wall_m': trial_f4['map_metrics']['wall_sharpness_sigma_m'],
                'f5_sigma_wall_m': trial_f5['map_metrics']['wall_sharpness_sigma_m'],
                'interpretation': (
                    "F4 scan matching provides direct scan-to-scan consistency, while "
                    "F5_improved fuses wheel/IMU + scan matching in EKF with slight filtering lag."
                )
            },
            'cpu_consumption_f4_vs_f5': {
                'f4_mean_cpu_pct': trial_f4['system_performance']['mean_cpu_percent'],
                'f5_mean_cpu_pct': trial_f5['system_performance']['mean_cpu_percent'],
                'f4_peak_cpu_pct': trial_f4['system_performance']['peak_cpu_percent'],
                'f5_peak_cpu_pct': trial_f5['system_performance']['peak_cpu_percent']
            },
            'estimation_latency_f4_vs_f5': {
                'f4_mean_latency_ms': trial_f4['system_performance']['mean_estimation_latency_ms'],
                'f5_mean_latency_ms': trial_f5['system_performance']['mean_estimation_latency_ms']
            },
            'stalls_and_safety_f4_vs_f5': {
                'f4_stalls': trial_f4['navigation_and_safety']['stall_events_count'],
                'f5_stalls': trial_f5['navigation_and_safety']['stall_events_count'],
                'f4_terminal_state': trial_f4['navigation_and_safety']['terminal_safety_state'],
                'f5_terminal_state': trial_f5['navigation_and_safety']['terminal_safety_state']
            }
        }
    }

    out_file = os.path.join(RESULTS_DIR, 'long_term_mapping_f4_f5_comparison.json')
    with open(out_file, 'w') as f:
        json.dump(comparison, f, indent=2)

    print("\n=================================================================")
    print("Long-Term Mapping Evaluation (F4 vs F5_improved) Finished!")
    print(f"Summary saved to: {out_file}")
    print("=================================================================")


if __name__ == '__main__':
    main()
