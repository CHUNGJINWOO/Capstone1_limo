#!/usr/bin/env -S python3 -s
"""
run_full_reeval_benchmark.py
Main Benchmark Execution Script for LIMO Pro Final Pre-Deployment Evaluation:
- Re-evaluates F4 and F5_improved (15 trials each = 30 trials total) on CL_Corridor_Doorway_Perturbed.
- Enforces rigorous pre-flight Gazebo pose settling and costmap clearing before goal dispatch.
- Separates failure modes:
  * INIT_FAILED
  * STATE_ESTIMATION_DRIFT
  * PATH_PLANNING_FAILED
  * CONTROL_OBSTACLE_COLLISION
  * ABORTED_BY_STALL_SAFETY
- Evaluates strict (<= 0.22m) and bounded (<= 0.30m) Ground Truth goal arrival.
- Generates high-definition MP4 videos for representative trials.
"""
import os
import sys
import time
import math
import json
import csv
import random
import signal
import subprocess
import psutil
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap

from reeval_monitor_node import (
    ReEvaluationMonitorNode,
    send_gz_set_pose,
    verify_and_settle_gazebo,
    CRITERIA_STRICT_GT_TOL_M,
    CRITERIA_BOUNDED_GT_TOL_M,
    CRITERIA_YAW_TOL_RAD,
    CRITERIA_FALSE_GOAL_THRESHOLD_M,
    SCENARIO,
    ESTIMATORS,
    EXP_DIR,
    RESULTS_DIR,
    INIT_DIR,
    VID_DIR,
    wrap_angle,
)


def start_base_gazebo():
    print("Launching Base Gazebo Fortress with Lounge World...")
    env = dict(os.environ)
    env['PYTHONNOUSERSITE'] = '1'
    p = subprocess.Popen(
        ['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'base_gazebo_for_trials.launch.py')],
        env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(7.0)
    return p


def start_nav2_stack():
    print("Launching Nav2 Closed-Loop Stack...")
    env = dict(os.environ)
    env['PYTHONNOUSERSITE'] = '1'
    p = subprocess.Popen(
        ['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'closed_loop_nav2_stack.launch.py')],
        env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    time.sleep(8.5)
    return p


def kill_process_tree(p):
    if p and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            p.wait(timeout=2.0)
        except Exception:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass


def run_benchmark():
    rclpy.init()

    p_gz = None
    p_gt = None
    p_nav = None
    all_init_records = []
    all_trial_results = {'F4': [], 'F5_improved': []}

    try:
        base_sp = SCENARIO['spawn']
        goal = SCENARIO['goal']
        bp = SCENARIO['base_perturb']
        total_runs_per_est = 15

        p_gz = start_base_gazebo()
        env = dict(os.environ)
        env['PYTHONNOUSERSITE'] = '1'

        print("Starting Boot TF & Relays for Nav2 Lifecycle Bringup...")
        p_gt = subprocess.Popen(
            ['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'ground_truth_publisher_node.py')],
            env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        boot_procs = []
        boot_procs.append(subprocess.Popen([
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '--x', str(base_sp['x']), '--y', str(base_sp['y']), '--z', '0.0',
            '--yaw', str(base_sp['yaw']), '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
            '--ros-args', '-p', 'use_sim_time:=true'
        ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        boot_procs.append(subprocess.Popen(['python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py')],
                                           env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        boot_procs.append(subprocess.Popen([
            'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'odom_tf_relay_node.py'),
            '--ros-args', '-p', 'active_odom_topic:=/odometry/lidar_odom'
        ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        boot_procs.append(subprocess.Popen(['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'state_estimators.launch.py')],
                                           env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

        p_nav = start_nav2_stack()
        time.sleep(2.0)

        # Cleanly stop temporary boot procs so each trial starts with its own fresh processes
        for proc in boot_procs:
            kill_process_tree(proc)
        time.sleep(0.5)

        for est_cfg in ESTIMATORS:
            code = est_cfg['code']
            print(f"\n=======================================================")
            print(f"Starting Re-Evaluation Benchmark for Estimator: {code}")
            print(f"=======================================================")

            for idx in range(1, total_runs_per_est + 1):
                is_identical = (idx <= 5)
                group = 'IDENTICAL' if is_identical else 'VARIED_SEED'
                seed = 0 if is_identical else (2000 + idx - 5)

                rng = random.Random(seed)
                if is_identical:
                    sp_x, sp_y, sp_yaw = base_sp['x'], base_sp['y'], base_sp['yaw']
                    w_scale, w_noise = bp['w_scale'], bp['w_noise']
                    g_bias, g_noise = bp['g_bias'], bp['g_noise']
                    a_bias, a_noise = bp['a_bias'], bp['a_noise']
                else:
                    sp_x = round(base_sp['x'] + rng.uniform(-0.020, 0.020), 4)
                    sp_y = round(base_sp['y'] + rng.uniform(-0.020, 0.020), 4)
                    sp_yaw = round(base_sp['yaw'] + math.radians(rng.uniform(-1.2, 1.2)), 4)
                    w_scale = round(bp['w_scale'] + rng.uniform(-0.03, 0.03), 4)
                    w_noise = round(bp['w_noise'] + rng.uniform(0.002, 0.006), 4)
                    g_bias = round(bp['g_bias'] + rng.uniform(-0.006, 0.006), 5)
                    g_noise = round(bp['g_noise'] + rng.uniform(0.002, 0.005), 5)
                    a_bias = round(bp['a_bias'] + rng.uniform(-0.02, 0.02), 4)
                    a_noise = round(bp['a_noise'] + rng.uniform(0.01, 0.02), 4)

                trial_id = f"REEVAL_{code}_trial_{idx:02d}_{group.lower()}" + (f"_seed{seed}" if seed > 0 else "")
                print(f"\n[{code} {idx:02d}/{total_runs_per_est:02d}] Preparing {trial_id} ...")

                # Step 1: Gazebo Pose Reset
                send_gz_set_pose(sp_x, sp_y, base_sp['z'], sp_yaw)

                # Create monitor node for settling and execution
                mon = ReEvaluationMonitorNode(code, est_cfg['topic'], {'x': sp_x, 'y': sp_y, 'yaw': sp_yaw}, goal)

                # Step 2: Actively verify and wait for physical settling
                settle_res = verify_and_settle_gazebo(mon, sp_x, sp_y, sp_yaw, max_timeout_s=8.0)
                init_audit = {
                    'trial_id': trial_id,
                    'estimator': code,
                    'trial_idx': idx,
                    'group': group,
                    'seed': seed,
                    'target_spawn': {'x': sp_x, 'y': sp_y, 'yaw': sp_yaw},
                    'settle_result': settle_res,
                }
                all_init_records.append(init_audit)

                if not settle_res['success']:
                    print(f"  [INIT_FAILED] Settle timeout! Aborting {trial_id}.")
                    trial_res = {
                        'trial_id': trial_id,
                        'estimator': code,
                        'outcome_class': 'INIT_FAILED',
                        'nav2_action_result': 'ABORTED_INIT_SETTLE_TIMEOUT',
                        'init_settle_result': settle_res,
                    }
                    all_trial_results[code].append(trial_res)
                    mon.destroy_node()
                    continue

                print(f"  [Init Settled OK] PosErr={settle_res['final_pos_err_m']}m, YawErr={settle_res['final_yaw_err_deg']}deg in {settle_res['settle_wall_duration_s']}s.")

                # Step 3: Launch Per-Trial Child Processes
                env = dict(os.environ)
                env['PYTHONNOUSERSITE'] = '1'
                trial_procs = []

                # Single authoritative static TF map -> odom at exact settled spawn pose
                trial_procs.append(subprocess.Popen([
                    'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                    '--x', str(sp_x), '--y', str(sp_y), '--z', '0.0',
                    '--yaw', str(sp_yaw), '--pitch', '0.0', '--roll', '0.0',
                    '--frame-id', 'map', '--child-frame-id', 'odom',
                    '--ros-args', '-p', 'use_sim_time:=true'
                ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

                # Sensor normalizer with calibrated perturbations
                norm_cmd = [
                    'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'trial_sensor_normalizer_node.py'),
                    '--ros-args',
                    '-p', f'seed:={seed}',
                    '-p', f'wheel_vel_scale:={w_scale}',
                    '-p', f'wheel_vel_noise_std:={w_noise}',
                    '-p', f'imu_gyro_bias_z:={g_bias}',
                    '-p', f'imu_gyro_noise_std:={g_noise}',
                    '-p', f'imu_accel_bias_x:={a_bias}',
                    '-p', f'imu_accel_noise_std:={a_noise}',
                    '-p', f"slip_burst_start_s:={bp['slip_start']}",
                    '-p', f"slip_burst_end_s:={bp['slip_end']}",
                    '-p', f"slip_burst_multiplier:={bp['slip_mult']}",
                ]
                trial_procs.append(subprocess.Popen(norm_cmd, env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                trial_procs.append(subprocess.Popen([
                    'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'odom_tf_relay_node.py'),
                    '--ros-args', '-p', f"active_odom_topic:={est_cfg['topic']}"
                ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                trial_procs.append(subprocess.Popen(['ros2', 'launch', os.path.join(EXP_DIR, 'launch', 'state_estimators.launch.py')],
                                                    env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
                trial_procs.append(subprocess.Popen([
                    'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'stall_detector_node.py'),
                    '--ros-args',
                    '-p', f"enabled:={'true' if est_cfg['stall_detector_enabled'] else 'false'}",
                    '-p', 'enable_auto_reverse:=false',  # AUDIT RULE: False!
                    '-p', 'require_operator_unlatch:=true',  # AUDIT RULE: True!
                    '-p', f"publish_virtual_bumper:={'true' if est_cfg['stall_detector_enabled'] else 'false'}"
                ], env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

                # Video recording on trial 1 of each group (identical and varied)
                p_vid = None
                vid_mp4 = None
                if idx in (1, 6):
                    vid_name = f"REEVAL_{code}_{'identical_trial01' if idx == 1 else 'varied_seed2001'}.mp4"
                    vid_mp4 = os.path.join(VID_DIR, vid_name)
                    vid_cmd = [
                        'python3', '-s', os.path.join(EXP_DIR, 'nodes', 'closed_loop_video_recorder_node.py'),
                        '--ros-args',
                        '-p', f'output_mp4:={vid_mp4}',
                        '-p', f'scenario_title:=REEVAL_{code}_Trial{idx:02d}',
                        '-p', f"active_estimator:={est_cfg['vid_code']}",
                        '-p', f"goal_x:={goal['x']}",
                        '-p', f"goal_y:={goal['y']}",
                    ]
                    p_vid = subprocess.Popen(vid_cmd, env=env, preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    trial_procs.append(p_vid)

                # Wait for active odom and scan to start streaming
                t_wait_stream = time.time()
                while time.time() - t_wait_stream < 6.0:
                    rclpy.spin_once(mon, timeout_sec=0.03)
                    if mon.active_odom is not None and mon.scan_count >= 5 and len(mon.f4_lat_list) >= 2:
                        break
                time.sleep(0.4)

                # Clear Nav2 local and global costmaps
                if mon.clear_local_srv.wait_for_service(timeout_sec=2.0):
                    mon.clear_local_srv.call_async(ClearEntireCostmap.Request())
                if mon.clear_global_srv.wait_for_service(timeout_sec=2.0):
                    mon.clear_global_srv.call_async(ClearEntireCostmap.Request())
                for _ in range(10):
                    rclpy.spin_once(mon, timeout_sec=0.04)

                # Reset samples so telemetry reflects active navigation
                mon.samples = []
                mon.f4_lat_list = []
                mon.scan_count = 0
                mon.active_odom_count = 0

                # Step 4: Dispatch Nav2 Goal
                mon.nav_client.wait_for_server(timeout_sec=5.0)
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.header.stamp = mon.get_clock().now().to_msg()
                goal_msg.pose.pose.position.x = float(goal['x'])
                goal_msg.pose.pose.position.y = float(goal['y'])
                goal_msg.pose.pose.orientation.z = math.sin(goal['yaw'] * 0.5)
                goal_msg.pose.pose.orientation.w = math.cos(goal['yaw'] * 0.5)

                send_future = mon.nav_client.send_goal_async(goal_msg)
                t_dispatch = time.time()
                while time.time() - t_dispatch < 3.0 and not send_future.done():
                    rclpy.spin_once(mon, timeout_sec=0.04)

                goal_handle = send_future.result() if send_future.done() else None
                res_future = None
                if goal_handle and goal_handle.accepted:
                    mon.action_state = 'NAVIGATING'
                    res_future = goal_handle.get_result_async()
                else:
                    mon.action_state = 'REJECTED'

                # Step 5: Execute and Monitor Closed-Loop Navigation
                t_run = time.time()
                last_sample_t = 0.0
                cpu_samples = []
                max_dur = SCENARIO['max_duration_s']

                while time.time() - t_run < max_dur:
                    rclpy.spin_once(mon, timeout_sec=0.02)
                    now = time.time()
                    if now - last_sample_t >= 0.10:
                        mon.record_sample()
                        cpu_samples.append(psutil.cpu_percent(interval=None))
                        last_sample_t = now

                    # Check Nav2 action completion
                    if res_future is not None and res_future.done():
                        status_code = res_future.result().status
                        if status_code == GoalStatus.STATUS_SUCCEEDED:
                            mon.action_state = 'SUCCEEDED'
                        elif status_code == GoalStatus.STATUS_ABORTED:
                            mon.action_state = 'ABORTED'
                        elif status_code == GoalStatus.STATUS_CANCELED:
                            mon.action_state = 'CANCELED'
                        else:
                            mon.action_state = f'STATUS_{status_code}'

                        # Settle hold for 0.5s to capture post-completion state
                        t_hold = time.time()
                        while time.time() - t_hold < 0.5:
                            rclpy.spin_once(mon, timeout_sec=0.02)
                            if time.time() - last_sample_t >= 0.10:
                                mon.record_sample()
                                last_sample_t = time.time()
                        break

                    # If Stall Detector latched, abort cleanly
                    if mon.stall_diag.get('state') == 'STALL_LATCHED' and est_cfg['stall_detector_enabled']:
                        if goal_handle and mon.action_state == 'NAVIGATING':
                            mon.action_state = 'ABORTED_BY_STALL_SAFETY'
                            goal_handle.cancel_goal_async()
                            time.sleep(0.4)
                            break

                if mon.action_state == 'NAVIGATING':
                    mon.action_state = 'ABORTED_PROGRESS_TIMEOUT'
                    if goal_handle:
                        goal_handle.cancel_goal_async()

                # Step 6: Evaluate Trial Metrics
                last_s = mon.samples[-1] if mon.samples else {}
                final_gt_dist = last_s.get('gt_goal_dist', 10.0)
                final_est_dist = last_s.get('est_goal_dist', 10.0)
                contact_occurred = any(s.get('contact_detected', False) for s in mon.samples)
                contact_count = sum(1 for s in mon.samples if s.get('contact_detected', False))
                min_scan = min((s.get('min_scan_r', 10.0) for s in mon.samples), default=10.0)

                # Classify Outcome rigorously
                if mon.action_state == 'SUCCEEDED' and not contact_occurred:
                    if final_gt_dist <= CRITERIA_STRICT_GT_TOL_M:
                        outcome = 'STRICT_TRUE_SUCCEEDED'
                    elif final_gt_dist <= CRITERIA_BOUNDED_GT_TOL_M:
                        outcome = 'BOUNDED_TRUE_SUCCEEDED'
                    elif final_gt_dist >= CRITERIA_FALSE_GOAL_THRESHOLD_M:
                        outcome = 'FALSE_SUCCEEDED_DRIFT'
                    else:
                        outcome = 'PARTIAL_SUCCEEDED_BOUNDARY'
                elif mon.action_state == 'ABORTED_BY_STALL_SAFETY':
                    outcome = 'SAFE_ABORTED_BY_STALL_DETECTOR'
                elif mon.action_state == 'ABORTED_PROGRESS_TIMEOUT':
                    outcome = 'ABORTED_TIMEOUT'
                elif contact_occurred:
                    outcome = 'FAILED_OBSTACLE_COLLISION'
                else:
                    outcome = f"FAILED_{mon.action_state}"

                # Save CSV
                csv_path = os.path.join(RESULTS_DIR, f"{trial_id}.csv")
                if mon.samples:
                    keys = list(mon.samples[0].keys())
                    with open(csv_path, 'w', newline='') as fp:
                        writer = csv.DictWriter(fp, fieldnames=keys)
                        writer.writeheader()
                        writer.writerows(mon.samples)

                trial_res = {
                    'trial_id': trial_id,
                    'estimator': code,
                    'trial_idx': idx,
                    'group': group,
                    'seed': seed,
                    'spawn': {'x': sp_x, 'y': sp_y, 'yaw': sp_yaw},
                    'perturb': {'w_scale': w_scale, 'g_bias': g_bias, 'a_bias': a_bias},
                    'nav2_action_result': mon.action_state,
                    'outcome_class': outcome,
                    'final_gt_goal_dist_m': round(final_gt_dist, 4),
                    'final_est_goal_dist_m': round(final_est_dist, 4),
                    'goal_eval_discrepancy_m': round(abs(final_gt_dist - final_est_dist), 4),
                    'strict_success_gt_0_22m': bool(outcome == 'STRICT_TRUE_SUCCEEDED'),
                    'bounded_success_gt_0_30m': bool(outcome in ('STRICT_TRUE_SUCCEEDED', 'BOUNDED_TRUE_SUCCEEDED')),
                    'false_success': bool(outcome == 'FALSE_SUCCEEDED_DRIFT'),
                    'physical_contact': bool(contact_occurred),
                    'contact_samples_count': contact_count,
                    'min_lidar_scan_range_m': round(min_scan, 4),
                    'stall_detected_and_blocked': bool(mon.stall_diag.get('cmd_blocked', False)),
                    'samples_count': len(mon.samples),
                    'csv_path': csv_path,
                    'video_mp4': vid_mp4,
                }
                all_trial_results[code].append(trial_res)

                print(
                    f"  [Result] Nav2={mon.action_state:12s} | Outcome={outcome:28s} | "
                    f"GT_Dist={final_gt_dist:.3f}m | Est_Dist={final_est_dist:.3f}m | "
                    f"Contact={contact_occurred}"
                )

                # Step 7: Clean shutdown of child processes
                mon.destroy_node()
                for p in trial_procs:
                    kill_process_tree(p)
                time.sleep(0.5)

        # Save Summary JSON
        summary_path = os.path.join(RESULTS_DIR, 'stabilized_reeval_30trials_summary.json')
        with open(summary_path, 'w') as fp:
            json.dump({
                'generated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                'scenario': SCENARIO['id'],
                'criteria': {
                    'strict_gt_tol_m': CRITERIA_STRICT_GT_TOL_M,
                    'bounded_gt_tol_m': CRITERIA_BOUNDED_GT_TOL_M,
                    'yaw_tol_rad': CRITERIA_YAW_TOL_RAD,
                    'false_goal_dist_threshold_m': CRITERIA_FALSE_GOAL_THRESHOLD_M,
                },
                'results_by_estimator': all_trial_results,
            }, fp, indent=2)

        init_audit_path = os.path.join(INIT_DIR, 'init_settling_audit.json')
        with open(init_audit_path, 'w') as fp:
            json.dump(all_init_records, fp, indent=2)

        print(f"\n=======================================================")
        print(f"Re-Evaluation Complete! Saved summary to {summary_path}")
        print(f"Saved Init Settle Audit to {init_audit_path}")
        print(f"=======================================================")

    finally:
        kill_process_tree(p_nav)
        kill_process_tree(p_gt)
        kill_process_tree(p_gz)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    run_benchmark()
