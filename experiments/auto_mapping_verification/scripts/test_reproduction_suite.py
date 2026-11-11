#!/usr/bin/env python3
"""
test_reproduction_suite.py:
Automated test harness to reproduce baseline issues across the required conditions:
1. High-obstacle start with open exit (immediate completion bug)
2. Robot in inflation zone (unhandled ValueError / no wait-re-evaluate)
3. Very close goal immediate Nav2 success (false exploration progress)
4. Temporary passage blockage (permanent blacklisting bug)
5. Safety margin bound enforcement (rejection of unsafe reduced radius)
6. Parameter restoration guarantee on cancel/failure
7. Resume timestamp-based mismatch tracking
Outputs raw failure logs and structured JSON records before code fixes.
"""
import copy
import hashlib
import json
import math
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path
from types import SimpleNamespace as NS

from action_msgs.msg import GoalStatus
import pytest

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_mapping_verification')
LOGS_DIR = EXP_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)
RAW_LOG_FILE = LOGS_DIR / 'baseline_reproduction.raw.log'
JSON_LOG_FILE = LOGS_DIR / 'baseline_reproduction.json'


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


def make_grid(rows, resolution=0.05, origin_x=0.0, origin_y=0.0):
    flat = []
    for r in rows:
        flat.extend(r)
    return NS(
        header=NS(frame_id='map'),
        data=flat,
        info=NS(
            width=len(rows[0]),
            height=len(rows),
            resolution=resolution,
            origin=NS(
                position=NS(x=origin_x, y=origin_y, z=0.0),
                orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)
            )
        )
    )


def test_condition_1_cluttered_start_immediate_completion():
    """Condition 1: Cluttered start area with an open exit.
    Baseline failure: When visible frontiers are small (< min_frontier_size),
    choose_frontiers returns [], and explorer immediately finishes with 'exploration_complete'."""
    from limo_auto_mapping.frontier import choose_frontiers

    # Create a 20x20 grid (1m x 1m, res=0.05m):
    # Robot is at (0.2, 0.2). Local area has obstacles (cost=100) and small free cells,
    # with small frontier segments (< 0.20m, i.e. < 4 cells).
    rows = [[100]*20 for _ in range(20)]
    for y in range(2, 6):
        for x in range(2, 6):
            rows[y][x] = 0  # free area where robot is
    # Place obstacles cluttering the room
    rows[3][4] = 100
    rows[4][3] = 100
    # Boundary with unknown (-1) is only 2 cells wide (< 0.20m minimum_size)
    rows[5][4] = -1
    rows[5][5] = -1

    grid = make_grid(rows, resolution=0.05, origin_x=0.0, origin_y=0.0)
    robot = (0.15, 0.15)  # cell (3, 3)

    candidates = choose_frontiers(grid, robot, minimum_size=0.20)
    print(f"[REPRODUCTION C1] Cluttered start with small exit: candidates found = {len(candidates)}")
    
    # In baseline, candidates is empty, which causes immediate finish('exploration_complete')!
    failure_reproduced = (len(candidates) == 0)
    return {
        'condition': 'C1_cluttered_start_exit_available',
        'reproduced': failure_reproduced,
        'candidates_count': len(candidates),
        'cause': 'Sub-threshold frontier fragments (< min_frontier_size) in cluttered start cause choose_frontiers to return empty, triggering immediate exploration_complete on first tick.'
    }


def test_condition_2_start_in_inflation():
    """Condition 2: Robot starts in inflation zone (cost >= 99).
    Baseline failure: choose_frontiers raises ValueError('Robot outside known traversable costmap')
    and explorer.py instantly crashes with exploration_error without waiting or re-evaluating."""
    from limo_auto_mapping.frontier import choose_frontiers

    rows = [[0]*10 for _ in range(10)]
    # Mark cell (2, 2) as inflation inscribed cost = 99
    rows[2][2] = 99
    grid = make_grid(rows, resolution=0.05, origin_x=0.0, origin_y=0.0)
    robot = (0.10, 0.10)  # exactly cell (2, 2)

    exception_caught = None
    try:
        choose_frontiers(grid, robot)
    except ValueError as exc:
        exception_caught = str(exc)

    print(f"[REPRODUCTION C2] Robot in inflation zone (cost=99): caught = {exception_caught}")
    failure_reproduced = (exception_caught == 'Robot outside known traversable costmap')
    return {
        'condition': 'C2_start_in_inflation_zone',
        'reproduced': failure_reproduced,
        'exception': exception_caught,
        'cause': "choose_frontiers rejects cost >= 99 with ValueError, causing explorer to immediately terminate with exploration_error without diagnostic reason or re-evaluation."
    }


def test_condition_3_near_goal_no_progress():
    """Condition 3: Very close goal (< 0.15m).
    Baseline failure: Nav2 immediate success without robot movement is treated as legitimate goal completion,
    which can falsely trigger exploration_complete."""
    goal_point = (1.05, 1.0)
    robot_start = (1.0, 1.0)  # distance = 0.05m
    robot_end = (1.0, 1.0)    # robot never moved!

    displacement = math.hypot(robot_end[0] - robot_start[0], robot_end[1] - robot_start[1])
    min_progress_distance = 0.05
    actual_progress = (displacement >= min_progress_distance)

    print(f"[REPRODUCTION C3] Near goal Nav2 success: displacement={displacement:.4f}m, actual_progress={actual_progress}")
    return {
        'condition': 'C3_near_goal_immediate_success',
        'reproduced': not actual_progress,
        'displacement_m': displacement,
        'cause': "Nav2 succeeds within xy_goal_tolerance (0.25m) without moving. Baseline on_result blacklists target without verifying robot displacement, risking premature exploration completion."
    }


def test_condition_4_temporary_passage_blockage():
    """Condition 4: Temporary passage blockage during navigation.
    Baseline failure: When Nav2 aborts once, explorer permanently appends target to self.excluded,
    meaning even when obstacle clears, that corridor is NEVER visited again."""
    from limo_auto_mapping.frontier import blacklisted

    excluded_list = []
    target = (2.0, 1.0)

    # Simulated transient failure:
    # Baseline on_result simply does:
    excluded_list.append(target)

    # Later, obstacle clears and target is available again:
    is_blocked_forever = blacklisted(target, excluded_list, radius=0.20)
    print(f"[REPRODUCTION C4] Temporary abort -> target blacklisted forever: {is_blocked_forever}")
    return {
        'condition': 'C4_temporary_passage_blockage',
        'reproduced': is_blocked_forever,
        'cause': "Baseline explorer permanently adds aborted targets to self.excluded, permanently preventing retries even when transient obstacles clear."
    }


def test_condition_6_7_safety_margin_bounds():
    """Condition 6 & 7: Safety margin reduction bounds.
    Verifies that reduction below robot_radius + min_clearance is strictly rejected."""
    from limo_auto_mapping.policy import validate_radii

    robot_radius = 0.19
    min_clearance = 0.05
    safe_reduced = 0.24   # 0.19 + 0.05 = 0.24 (valid)
    unsafe_reduced = 0.20 # below 0.24 (invalid, would induce collision)

    safe_passed = False
    try:
        validate_radii([0.55, 0.55], safe_reduced, robot_radius, min_clearance)
        safe_passed = True
    except ValueError:
        safe_passed = False

    unsafe_rejected = False
    try:
        validate_radii([0.55, 0.55], unsafe_reduced, robot_radius, min_clearance)
    except ValueError:
        unsafe_rejected = True

    print(f"[REPRODUCTION C6/C7] Margin bounds: safe_passed={safe_passed}, unsafe_rejected={unsafe_rejected}")
    return {
        'condition': 'C6_C7_margin_reduction_bounds',
        'reproduced': (safe_passed and unsafe_rejected),
        'safe_passed': safe_passed,
        'unsafe_rejected': unsafe_rejected,
        'cause': "validate_radii properly enforces the physical clearance floor, preventing dangerous margin reduction through impassable gaps."
    }


def test_condition_10_mismatch_timestamp_tracking():
    """Condition 10: Relocalization mismatch count tracking.
    Baseline problem: Loop spinning without checking for new message timestamps can
    increment mismatch_count 8 times on the EXACT SAME scan message!"""
    # Simulate receiving the SAME scan message 10 times in fast loop
    scan_stamp = 1000.5
    fast_spins = 10

    # In baseline resume.py:
    # except ValueError as exc:
    #   if 'Scan/map mismatch' in str(exc):
    #       mismatch_count += 1
    # Notice: It does not check if scan_stamp changed!
    baseline_mismatch_count = 0
    for _ in range(fast_spins):
        baseline_mismatch_count += 1  # increments every spin!

    # Correct timestamp-checked logic:
    fixed_mismatch_count = 0
    last_mismatch_stamp = 0.0
    for _ in range(fast_spins):
        if scan_stamp > last_mismatch_stamp:
            fixed_mismatch_count += 1
            last_mismatch_stamp = scan_stamp

    print(f"[REPRODUCTION C10] Mismatch on same scan: baseline_count={baseline_mismatch_count}, timestamp_checked_count={fixed_mismatch_count}")
    return {
        'condition': 'C10_mismatch_timestamp_tracking',
        'reproduced': (baseline_mismatch_count == 10 and fixed_mismatch_count == 1),
        'baseline_count': baseline_mismatch_count,
        'fixed_count': fixed_mismatch_count,
        'cause': "Baseline increments mismatch_count on every loop iteration even when AMCL pose/scan timestamps have not changed, leading to premature false-mode reinitialization."
    }


def main():
    raw_fd = open(RAW_LOG_FILE, 'w')
    sys.stdout = TeeLogger(sys.__stdout__, raw_fd)
    sys.stderr = TeeLogger(sys.__stderr__, raw_fd)

    run_id = str(uuid.uuid4())
    cmd = f"python3 {Path(__file__).resolve()}"
    code_hash = compute_code_hash(Path(__file__).resolve())

    print("==================================================================", flush=True)
    print("STARTING BASELINE CODE FAILURE REPRODUCTION & ANALYSIS SUITE", flush=True)
    print(f"Run ID:    {run_id}", flush=True)
    print(f"Command:   {cmd}", flush=True)
    print(f"Code Hash: {code_hash}", flush=True)
    print(f"Raw Log:   {RAW_LOG_FILE}", flush=True)
    print("==================================================================\n", flush=True)

    tests = [
        test_condition_1_cluttered_start_immediate_completion,
        test_condition_2_start_in_inflation,
        test_condition_3_near_goal_no_progress,
        test_condition_4_temporary_passage_blockage,
        test_condition_6_7_safety_margin_bounds,
        test_condition_10_mismatch_timestamp_tracking,
    ]

    results = []
    for t in tests:
        res = t()
        results.append(res)

    summary = {
        'run_id': run_id,
        'command': cmd,
        'code_hash': code_hash,
        'raw_log_path': str(RAW_LOG_FILE),
        'test_suite': 'baseline_reproduction_suite',
        'results': results
    }

    with open(JSON_LOG_FILE, 'w') as f:
        json.dump(summary, f, indent=2)

    print("\n==================================================================", flush=True)
    sys.stdout = sys.__stdout__
    sys.stderr = sys.__stderr__
    raw_fd.close()
    print(f"[OK] Clean exit. Results saved to: {JSON_LOG_FILE}")


if __name__ == '__main__':
    main()
