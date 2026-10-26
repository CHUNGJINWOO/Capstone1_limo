#!/usr/bin/env python3
"""
run_all_rigorous_scenarios.py:
Sequential orchestrator for the 8 Rigorous Test Scenarios.
Ensures clean shutdown between runs, domain isolation, and comprehensive logging.
"""

import os
import sys
import time
import subprocess
import pathlib
import json

BASE_DIR = pathlib.Path(__file__).resolve().parents[3]
OUT_DIR = BASE_DIR / 'output/simulation/2026-10-24_rigorous'
RUNNER_SCRIPT = OUT_DIR / 'sim_rigorous_runner.py'

SCENARIOS = [
    {
        'id': 'scen1_rigorous_baseline',
        'domain_id': 125,
        'world': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_world.sdf'),
        'map': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_map.yaml'),
        'model_file': str(BASE_DIR / 'output/simulation/2026-10-24/inputs/limo_contact_check.sdf'),
        'params_file': str(BASE_DIR / 'capstone_ws/src/limo_capstone/config/nav2_sim_ackermann_params.yaml'),
        'spawn_x': -4.5, 'spawn_y': -3.5, 'spawn_yaw': 0.0,
        'goal_x': -2.0, 'goal_y': -1.0, 'goal_yaw': 1.57,
        'timeout_sim': 35.0,
        'mode': 'nav2',
        'event_type': 'none',
        'monitor_x': None, 'monitor_y': None
    },
    {
        'id': 'scen2_rigorous_static',
        'domain_id': 126,
        'world': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_world.sdf'),
        'map': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_map.yaml'),
        'model_file': str(BASE_DIR / 'output/simulation/2026-10-24/inputs/limo_contact_check.sdf'),
        'params_file': str(BASE_DIR / 'capstone_ws/src/limo_capstone/config/nav2_sim_ackermann_params.yaml'),
        'spawn_x': -2.0, 'spawn_y': 0.0, 'spawn_yaw': 0.0,
        'goal_x': 2.5, 'goal_y': 1.4, 'goal_yaw': 0.0,
        'timeout_sim': 40.0,
        'mode': 'nav2',
        'event_type': 'none',
        'monitor_x': None, 'monitor_y': None
    },
    {
        'id': 'scen3_rigorous_unmapped',
        'domain_id': 127,
        'world': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_sc3_unmapped.sdf'),
        'map': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_map.yaml'),
        'model_file': str(BASE_DIR / 'output/simulation/2026-10-24/inputs/limo_contact_check.sdf'),
        'params_file': str(BASE_DIR / 'capstone_ws/src/limo_capstone/config/nav2_sim_ackermann_params.yaml'),
        'spawn_x': -2.0, 'spawn_y': 0.0, 'spawn_yaw': 0.0,
        'goal_x': 3.0, 'goal_y': 0.5, 'goal_yaw': 0.0,
        'timeout_sim': 45.0,
        'mode': 'nav2',
        'event_type': 'none',
        'monitor_x': None, 'monitor_y': None
    },
    {
        'id': 'scen4_rigorous_dynamic',
        'domain_id': 128,
        'world': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_sc4_dynamic.sdf'),
        'map': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_map.yaml'),
        'model_file': str(BASE_DIR / 'output/simulation/2026-10-24/inputs/limo_contact_check.sdf'),
        'params_file': str(BASE_DIR / 'capstone_ws/src/limo_capstone/config/nav2_sim_ackermann_params.yaml'),
        'spawn_x': -4.5, 'spawn_y': -3.5, 'spawn_yaw': 0.0,
        'goal_x': -1.5, 'goal_y': -3.5, 'goal_yaw': 0.0,
        'timeout_sim': 35.0,
        'mode': 'nav2',
        'event_type': 'dynamic_spawn',
        'monitor_x': None, 'monitor_y': None
    },
    {
        'id': 'scen5_rigorous_moving',
        'domain_id': 129,
        'world': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_sc5_moving.sdf'),
        'map': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_map.yaml'),
        'model_file': str(BASE_DIR / 'output/simulation/2026-10-24/inputs/limo_contact_check.sdf'),
        'params_file': str(BASE_DIR / 'capstone_ws/src/limo_capstone/config/nav2_sim_ackermann_params.yaml'),
        'spawn_x': -2.0, 'spawn_y': 0.0, 'spawn_yaw': 0.0,
        'goal_x': 2.5, 'goal_y': 0.0, 'goal_yaw': 0.0,
        'timeout_sim': 40.0,
        'mode': 'nav2',
        'event_type': 'moving_obstacle',
        'monitor_x': None, 'monitor_y': None
    },
    {
        'id': 'scen6_rigorous_blocked',
        'domain_id': 130,
        'world': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_sc6_blocked.sdf'),
        'map': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_map.yaml'),
        'model_file': str(BASE_DIR / 'output/simulation/2026-10-24/inputs/limo_contact_check.sdf'),
        'params_file': str(BASE_DIR / 'capstone_ws/src/limo_capstone/config/nav2_sim_ackermann_params.yaml'),
        'spawn_x': -2.0, 'spawn_y': -3.5, 'spawn_yaw': 0.0,
        'goal_x': 4.0, 'goal_y': -3.5, 'goal_yaw': 0.0,
        'timeout_sim': 30.0,
        'mode': 'nav2',
        'event_type': 'block_path',
        'monitor_x': None, 'monitor_y': None
    },
    {
        'id': 'scen7_rigorous_clearing',
        'domain_id': 131,
        'world': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_sc7_clearing.sdf'),
        'map': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_map.yaml'),
        'model_file': str(BASE_DIR / 'output/simulation/2026-10-24/inputs/limo_contact_check.sdf'),
        'params_file': str(BASE_DIR / 'capstone_ws/src/limo_capstone/config/nav2_sim_ackermann_params.yaml'),
        'spawn_x': -2.0, 'spawn_y': -3.5, 'spawn_yaw': 0.0,
        'goal_x': 4.0, 'goal_y': -3.5, 'goal_yaw': 0.0,
        'timeout_sim': 40.0,
        'mode': 'nav2',
        'event_type': 'clear_obstacle',
        'monitor_x': 0.5, 'monitor_y': -3.5
    },
    {
        'id': 'scen8_rigorous_exploration',
        'domain_id': 132,
        'world': str(BASE_DIR / 'output/simulation/2026-10-24_verification/maze/maze_world.sdf'),
        'map': 'NONE',
        'model_file': str(BASE_DIR / 'output/simulation/2026-10-24/inputs/limo_contact_check.sdf'),
        'params_file': str(BASE_DIR / 'capstone_ws/src/limo_capstone/config/nav2_explore_params.yaml'),
        'spawn_x': -4.5, 'spawn_y': -3.5, 'spawn_yaw': 0.0,
        'goal_x': 0.0, 'goal_y': 0.0, 'goal_yaw': 0.0,
        'timeout_sim': 50.0,
        'mode': 'auto_mapping',
        'event_type': 'none',
        'monitor_x': None, 'monitor_y': None
    }
]

def run_scenario(sc):
    print(f"\n==================================================", flush=True)
    print(f"  STARTING SCENARIO: {sc['id']}", flush=True)
    print(f"==================================================", flush=True)

    target_dir = OUT_DIR / sc['id']
    target_dir.mkdir(parents=True, exist_ok=True)

    cmd = [
        'python3', str(RUNNER_SCRIPT),
        '--scenario', sc['id'],
        '--output_dir', str(target_dir),
        '--world', sc['world'],
        '--world_name', 'mc3f_world',
        '--map', sc['map'],
        '--model_file', sc['model_file'],
        '--params_file', sc['params_file'],
        '--spawn_x', str(sc['spawn_x']),
        '--spawn_y', str(sc['spawn_y']),
        '--spawn_yaw', str(sc['spawn_yaw']),
        '--goal_x', str(sc['goal_x']),
        '--goal_y', str(sc['goal_y']),
        '--goal_yaw', str(sc['goal_yaw']),
        '--domain_id', str(sc['domain_id']),
        '--timeout_sim', str(sc['timeout_sim']),
        '--mode', sc['mode'],
        '--event_type', sc['event_type']
    ]
    if sc['monitor_x'] is not None:
        cmd.extend(['--monitor_x', str(sc['monitor_x']), '--monitor_y', str(sc['monitor_y'])])

    bash_wrapper = f"source /opt/ros/humble/setup.bash && source {BASE_DIR}/capstone_ws/install/setup.bash && " + " ".join(cmd)

    start_t = time.time()
    res = subprocess.run(['bash', '-c', bash_wrapper], cwd=str(BASE_DIR))
    elapsed = time.time() - start_t
    print(f"\n[Orchestrator] Completed {sc['id']} in {elapsed:.1f}s, exit code: {res.returncode}", flush=True)
    time.sleep(3.0)

def main():
    skip_existing = '--skip-existing' in sys.argv
    args = [a for a in sys.argv[1:] if a != '--skip-existing']
    target_scen = args[0] if args else None

    for sc in SCENARIOS:
        if target_scen and sc['id'] != target_scen:
            continue
        target_summary = OUT_DIR / sc['id'] / 'summary.json'
        if skip_existing and target_summary.exists() and target_summary.stat().st_size > 100:
            print(f"[Orchestrator] Skipping {sc['id']} (already completed).", flush=True)
            continue
        run_scenario(sc)

if __name__ == '__main__':
    main()
