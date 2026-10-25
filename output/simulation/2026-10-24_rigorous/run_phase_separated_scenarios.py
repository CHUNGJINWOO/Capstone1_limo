#!/usr/bin/env python3
"""
run_phase_separated_scenarios.py:
Runs scenarios 4, 5, 6, 7 in a dedicated output directory:
  /home/cjinw/school/2026-2/Capstone1_limo/output/simulation/2026-10-24_phase_separated
Preserves previous results in 2026-10-24_rigorous intact.
"""

import os
import sys
import time
import subprocess
import pathlib

BASE_DIR = pathlib.Path(__file__).resolve().parents[3]
OUT_DIR = BASE_DIR / 'output/simulation/2026-10-24_phase_separated'
RUNNER_SCRIPT = BASE_DIR / 'output/simulation/2026-10-24_rigorous/sim_rigorous_runner.py'

TARGET_SCENARIOS = [
    {
        'id': 'scen4_dynamic_spawn',
        'domain_id': 140,
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
        'id': 'scen5_moving_obstacle',
        'domain_id': 141,
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
        'id': 'scen6_blocked_safe_stop',
        'domain_id': 142,
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
        'id': 'scen7_clearing_resume',
        'domain_id': 143,
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
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    target_scen = sys.argv[1] if len(sys.argv) > 1 else None
    for sc in TARGET_SCENARIOS:
        if target_scen and sc['id'] != target_scen:
            continue
        run_scenario(sc)

if __name__ == '__main__':
    main()
