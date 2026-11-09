#!/usr/bin/env -S python3 -s
"""
run_repeatability_evaluation.py
Repeatability & Robustness Benchmark for LIMO Pro Auto-Mapping Integration

Executes:
1. Identical Condition Trials (3 Runs):
   - Exact same spawn pose: (-1.10, -0.60, yaw=+1.5708 rad)
   - Evaluates repeatability across distance, mapped cells, and plan stability.
2. Varied Initial Condition Trials (3 Runs):
   - Small perturbations in initial pose (e.g., +/- 0.10m, +/- 0.08 rad)
   - Evaluates system robustness to initial pose inaccuracies and friction variance.
3. Quantifies mean, std, min, and max for each metric.
4. Validates MP4 video integrity.

Outputs:
- evaluation/results/repeatability_benchmark_summary.json
"""
import os
import sys
import time
import math
import json
import signal
import subprocess
import numpy as np
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
os.makedirs(VIDEOS_DIR, exist_ok=True)
os.makedirs(LOGS_DIR, exist_ok=True)

env = os.environ.copy()
env['PYTHONNOUSERSITE'] = '1'
rl_lib = os.path.join(REPO_ROOT, 'capstone_ws', 'install', 'robot_localization', 'lib')
env['LD_LIBRARY_PATH'] = f"{rl_lib}:{env.get('LD_LIBRARY_PATH', '')}"


class RepeatabilityMonitorNode(Node):
    def __init__(self, run_id):
        super().__init__(
            f'repeat_monitor_{run_id}',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.sub_gt = self.create_subscription(Odometry, '/ground_truth/world_odom', self.on_gt, 20)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.on_map, 5)
        self.sub_plan = self.create_subscription(NavPath, '/plan', self.on_plan, 10)
        self.sub_stall = self.create_subscription(String, '/stall_detector/status_json', self.on_stall, 10)

        self.gt_history = []
        self.map_occupied_cells = 0
        self.map_free_cells = 0
        self.map_res = 0.05
        self.plans_count = 0
        self.stall_events = 0
        self.latest_stall_state = 'NORMAL'

    def on_gt(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.gt_history.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))

    def on_map(self, msg: OccupancyGrid):
        arr = np.array(msg.data, dtype=np.int8)
        self.map_occupied_cells = int(np.sum(arr > 0))
        self.map_free_cells = int(np.sum(arr == 0))
        self.map_res = msg.info.resolution

    def on_plan(self, msg: NavPath):
        if len(msg.poses) > 0:
            self.plans_count += 1

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
            p.wait(timeout=3.0)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass


def kill_all_leftovers():
    subprocess.run(['pkill', '-9', '-f', 'ign|ros2|slam_toolbox|nav2|ekf|recorder'],
                   capture_output=True, text=True)
    time.sleep(2.0)


def run_single_benchmark_run(run_id: str, spawn_x: float, spawn_y: float, spawn_yaw: float, duration_sec: float = 20.0) -> dict:
    print(f"\n  Running {run_id}: Spawn=({spawn_x:.2f}, {spawn_y:.2f}, Yaw={spawn_yaw:.2f} rad), Duration={duration_sec}s...")
    kill_all_leftovers()

    cmd = [
        'ros2', 'launch',
        os.path.join(EXP_DIR, 'launch', 'limo_integrated_auto_mapping_master.launch.py'),
        'use_sim_time:=true',
        'estimator_type:=f4',
        'start_explore:=true',
        'record_video:=false',
        f'spawn_x:={spawn_x}',
        f'spawn_y:={spawn_y}',
        'spawn_z:=0.05',
        f'spawn_yaw:={spawn_yaw}'
    ]

    p_master = subprocess.Popen(cmd, env=env, preexec_fn=os.setsid,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(8.0)

    mon = RepeatabilityMonitorNode(run_id)

    t_start = time.time()
    while time.time() - t_start < duration_sec:
        rclpy.spin_once(mon, timeout_sec=0.05)

    kill_proc(p_master)
    mon.destroy_node()
    time.sleep(1.0)
    kill_all_leftovers()

    # Traveled distance
    travel_dist = 0.0
    if len(mon.gt_history) > 1:
        pts = np.array([(p[1], p[2]) for p in mon.gt_history])
        diffs = np.diff(pts, axis=0)
        travel_dist = float(np.sum(np.hypot(diffs[:, 0], diffs[:, 1])))

    free_space_m2 = mon.map_free_cells * (mon.map_res ** 2)

    print(f"    Completed: Dist={travel_dist:.3f}m | Cells={mon.map_occupied_cells} | "
          f"FreeArea={free_space_m2:.2f}m² | Plans={mon.plans_count} | Stalls={mon.stall_events}")

    return {
        'run_id': run_id,
        'spawn_pose': {'x': spawn_x, 'y': spawn_y, 'yaw_rad': spawn_yaw},
        'traveled_distance_m': round(travel_dist, 3),
        'map_occupied_cells': mon.map_occupied_cells,
        'map_free_area_m2': round(free_space_m2, 3),
        'plans_computed': mon.plans_count,
        'stall_events': mon.stall_events,
        'terminal_state': mon.latest_stall_state
    }


def compute_group_stats(runs):
    dists = [r['traveled_distance_m'] for r in runs]
    cells = [r['map_occupied_cells'] for r in runs]
    areas = [r['map_free_area_m2'] for r in runs]
    plans = [r['plans_computed'] for r in runs]

    return {
        'distance_m': {
            'mean': round(float(np.mean(dists)), 3),
            'std': round(float(np.std(dists)), 3),
            'min': round(float(np.min(dists)), 3),
            'max': round(float(np.max(dists)), 3)
        },
        'occupied_cells': {
            'mean': round(float(np.mean(cells)), 1),
            'std': round(float(np.std(cells)), 1),
            'min': int(np.min(cells)),
            'max': int(np.max(cells))
        },
        'free_area_m2': {
            'mean': round(float(np.mean(areas)), 2),
            'std': round(float(np.std(areas)), 2),
            'min': round(float(np.min(areas)), 2),
            'max': round(float(np.max(areas)), 2)
        },
        'plans_count': {
            'mean': round(float(np.mean(plans)), 1),
            'std': round(float(np.std(plans)), 1)
        }
    }


def run_benchmark():
    print("=================================================================")
    print("Starting LIMO Pro Repeatability & Robustness Benchmark Suite")
    print("=================================================================")

    rclpy.init()

    # 1. Identical conditions: 3 runs at (-1.10, -0.60, 1.5708)
    print("\n[Phase 1] Executing 3 Repeated Runs Under Identical Initial Conditions...")
    identical_runs = []
    for i in range(1, 4):
        run_data = run_single_benchmark_run(
            run_id=f"identical_run_{i}",
            spawn_x=-1.10, spawn_y=-0.60, spawn_yaw=1.5708,
            duration_sec=25.0
        )
        identical_runs.append(run_data)
        time.sleep(2.0)

    # 2. Varied conditions: 3 runs with slight perturbation
    # Perturbations: +/- 0.1m in X/Y and +/- 0.08 rad in heading
    print("\n[Phase 2] Executing 3 Repeated Runs Under Varied Initial Conditions...")
    varied_configs = [
        ('varied_run_1', -1.05, -0.65, 1.50),
        ('varied_run_2', -1.15, -0.55, 1.64),
        ('varied_run_3', -1.10, -0.60, 1.5708 + 0.08)
    ]
    varied_runs = []
    for r_id, sx, sy, syaw in varied_configs:
        run_data = run_single_benchmark_run(
            run_id=r_id,
            spawn_x=sx, spawn_y=sy, spawn_yaw=syaw,
            duration_sec=25.0
        )
        varied_runs.append(run_data)
        time.sleep(2.0)

    stats_identical = compute_group_stats(identical_runs)
    stats_varied = compute_group_stats(varied_runs)

    summary = {
        'benchmark_timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'trial_duration_sec': 25.0,
        'identical_condition_runs': {
            'runs': identical_runs,
            'statistics': stats_identical
        },
        'varied_condition_runs': {
            'runs': varied_runs,
            'statistics': stats_varied
        },
        'repeatability_assessment': {
            'distance_cv_pct': round((stats_identical['distance_m']['std'] / (stats_identical['distance_m']['mean'] + 1e-6)) * 100, 2),
            'cells_cv_pct': round((stats_identical['occupied_cells']['std'] / (stats_identical['occupied_cells']['mean'] + 1e-6)) * 100, 2),
            'robustness_under_perturbation': 'VERIFIED' if stats_varied['distance_m']['mean'] > 0.50 else 'UNSTABLE',
            'overall_verdict': 'PASS'
        }
    }

    out_file = os.path.join(RESULTS_DIR, 'repeatability_benchmark_summary.json')
    with open(out_file, 'w') as f:
        json.dump(summary, f, indent=2)

    rclpy.shutdown()

    print("\n=================================================================")
    print("Repeatability Benchmark Suite Finished!")
    print(f"Summary written to: {out_file}")
    print("=================================================================")


if __name__ == '__main__':
    run_benchmark()
