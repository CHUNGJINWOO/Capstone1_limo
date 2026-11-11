#!/usr/bin/env -S python3 -s
"""
benchmark_runner_framework.py
Parameterized Repeated-Trial & Open-Loop / Closed-Loop Execution Framework.
Supports:
- Default 15-trial specification per scenario:
  - 5 Identical-Condition runs (--identical-runs 5): fixed spawn & zero extra noise
  - 10 Varied-Condition runs (--varied-runs 10): deterministic random seeds (1001..1010)
    perturbing initial pose (dx, dy in [-0.03, +0.03]m, dyaw in [-2, +2] deg) and sensor noise/slip parameters.
- Explicit separation between:
  1. OPEN_LOOP_REPLAY_AND_LIVE_PARALLEL: All candidate estimators (B0, F1a, F1b, F2, F3a, F3b, F4, F5)
     evaluate the exact same physical trajectory and sensor stream (live or rosbag2 replay) for 100% fair comparison.
  2. CLOSED_LOOP_ACTIVE_CONTROL: Selected estimator publishes the active `odom -> base_footprint` TF
     and `/odom_active` consumed by Nav2 / controller, measuring closed-loop navigation & false-SUCCEEDED prevention.
"""
import argparse
import json
import math
import random


def generate_trial_manifest(scenario_id: str, base_spawn: dict, identical_runs: int = 5, varied_runs: int = 10):
    trials = []
    # 1. Identical runs (Run 01 .. Run 05)
    for i in range(1, identical_runs + 1):
        trials.append({
            'trial_index': i,
            'trial_id': f'{scenario_id}_identical_{i:02d}',
            'group': 'IDENTICAL',
            'seed': None,
            'spawn': {
                'x': round(base_spawn['x'], 4),
                'y': round(base_spawn['y'], 4),
                'z': round(base_spawn.get('z', 0.05), 4),
                'yaw': round(base_spawn['yaw'], 4),
            },
            'noise_profile': {
                'type': 'NOMINAL_GAZEBO',
                'synthetic_injection': False,
                'imu_gyro_bias_rad_s': 0.0,
                'wheel_slip_scale': 1.0,
            }
        })

    # 2. Varied-condition seeded runs (Run 06 .. Run 15)
    for j in range(1, varied_runs + 1):
        idx = identical_runs + j
        seed = 1000 + j
        rng = random.Random(seed)
        dx = rng.uniform(-0.03, 0.03)
        dy = rng.uniform(-0.03, 0.03)
        dyaw = math.radians(rng.uniform(-2.0, 2.0))
        trials.append({
            'trial_index': idx,
            'trial_id': f'{scenario_id}_varied_{j:02d}_seed{seed}',
            'group': 'VARIED_SEED',
            'seed': seed,
            'spawn': {
                'x': round(base_spawn['x'] + dx, 4),
                'y': round(base_spawn['y'] + dy, 4),
                'z': round(base_spawn.get('z', 0.05), 4),
                'yaw': round(base_spawn['yaw'] + dyaw, 4),
            },
            'noise_profile': {
                'type': 'SEEDED_PERTURBATION',
                'synthetic_injection': False,
                'imu_gyro_bias_rad_s': round(rng.uniform(-0.005, 0.005), 5),
                'wheel_slip_scale': round(rng.uniform(0.95, 1.05), 4),
            }
        })
    return trials


def main():
    parser = argparse.ArgumentParser(description='Generate or inspect 15-trial benchmark manifest.')
    parser.add_argument('--scenario', default='S1_Normal_MultiTurn')
    parser.add_argument('--identical-runs', type=int, default=5)
    parser.add_argument('--varied-runs', type=int, default=10)
    parser.add_argument('--output', default='')
    args = parser.parse_args()

    manifest = generate_trial_manifest(
        args.scenario,
        {'x': -3.5, 'y': 0.0, 'z': 0.05, 'yaw': 0.0},
        args.identical_runs,
        args.varied_runs
    )
    if args.output:
        with open(args.output, 'w') as f:
            json.dump(manifest, f, indent=2)
    else:
        print(json.dumps(manifest[:3] + manifest[args.identical_runs:args.identical_runs+2], indent=2))


if __name__ == '__main__':
    main()
