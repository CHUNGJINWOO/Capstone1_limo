#!/usr/bin/env -S python3 -s
"""
recompute_75trial_zoh_vs_interp.py
Reads the preserved 75 trial CSVs/JSONs from `limo_state_estimation_benchmark_20261107` (READ-ONLY)
and computes side-by-side:
1. Original Zero-Order-Hold (ZOH) statistics (sampled at 10 Hz collector loop time)
2. Timestamp-Interpolated statistics (GT linearly interpolated at each estimator's exact message header.stamp)
Saves the full side-by-side comparison to `results/previous_75trial_zoh_vs_interpolated_recomputation.json`.
"""
import os
import csv
import glob
import json
import math
import numpy as np

OLD_BENCH_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107'
NEW_EXP_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_nav2_closed_loop_verification_20261107'


def wrap_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))


def compute_trial_both_modes(csv_path, codes):
    with open(csv_path, 'r') as f:
        rows = list(csv.DictReader(f))

    gt_t = np.array([float(r['sim_time']) for r in rows], dtype=np.float64)
    gt_x = np.array([float(r['GT_x']) for r in rows], dtype=np.float64)
    gt_y = np.array([float(r['GT_y']) for r in rows], dtype=np.float64)
    gt_yaw = np.array([float(r['GT_yaw']) for r in rows], dtype=np.float64)
    gt_yaw_unwrap = np.unwrap(gt_yaw)

    out = {}
    for c in codes:
        if f'{c}_x' not in rows[0]:
            continue
        ex = np.array([float(r[f'{c}_x']) if r[f'{c}_x'] != '' else np.nan for r in rows], dtype=np.float64)
        ey = np.array([float(r[f'{c}_y']) if r[f'{c}_y'] != '' else np.nan for r in rows], dtype=np.float64)
        eyaw = np.array([float(r[f'{c}_yaw']) if r[f'{c}_yaw'] != '' else np.nan for r in rows], dtype=np.float64)
        has_stamp = f'{c}_stamp' in rows[0]
        estamp = np.array([float(r[f'{c}_stamp']) if (has_stamp and r[f'{c}_stamp'] != '') else np.nan for r in rows], dtype=np.float64)

        valid = np.isfinite(ex) & np.isfinite(ey) & np.isfinite(eyaw)
        if np.count_nonzero(valid) < 5:
            continue

        # 1. ZOH mode (evaluated directly against row's GT_t)
        pos_zoh = np.hypot(ex[valid] - gt_x[valid], ey[valid] - gt_y[valid])
        yaw_zoh = np.abs(np.degrees(wrap_angle(eyaw[valid] - gt_yaw[valid])))

        # 2. Timestamp-Interpolated mode (GT linearly interpolated at estamp)
        if has_stamp and np.all(np.isfinite(estamp[valid])):
            st = estamp[valid]
            st_clipped = np.clip(st, gt_t[0], gt_t[-1])
            gx_i = np.interp(st_clipped, gt_t, gt_x)
            gy_i = np.interp(st_clipped, gt_t, gt_y)
            gyaw_i = wrap_angle(np.interp(st_clipped, gt_t, gt_yaw_unwrap))
            pos_interp = np.hypot(ex[valid] - gx_i, ey[valid] - gy_i)
            yaw_interp = np.abs(np.degrees(wrap_angle(eyaw[valid] - gyaw_i)))
            sample_hold_lag_ms = float(np.mean(np.maximum(0.0, gt_t[valid] - st)) * 1000.0)
        else:
            pos_interp = pos_zoh
            yaw_interp = yaw_zoh
            sample_hold_lag_ms = 0.0

        out[c] = {
            'zoh_ate_rmse_m': float(np.sqrt(np.mean(pos_zoh ** 2))),
            'interp_ate_rmse_m': float(np.sqrt(np.mean(pos_interp ** 2))),
            'zoh_ate_p95_m': float(np.percentile(pos_zoh, 95.0)),
            'interp_ate_p95_m': float(np.percentile(pos_interp, 95.0)),
            'zoh_ate_max_m': float(np.max(pos_zoh)),
            'interp_ate_max_m': float(np.max(pos_interp)),
            'zoh_yaw_rmse_deg': float(np.sqrt(np.mean(yaw_zoh ** 2))),
            'interp_yaw_rmse_deg': float(np.sqrt(np.mean(yaw_interp ** 2))),
            'mean_sample_hold_lag_ms': round(sample_hold_lag_ms, 2),
        }
    return out


def main():
    codes_all = ['B0', 'F1a', 'F1b', 'F2', 'F3a', 'F3b', 'F4', 'F5', 'F6']
    estimator_corrected_names = {
        'B0': 'Wheel Odometry (Raw Ackermann)',
        'F1a': 'Legacy EKF (limo_ekf.yaml reproduction - double yaw + raw ax)',
        'F1b': 'Tuned EKF (Wheel vx,vy + IMU yaw,wz)',
        'F2': 'Tuned UKF (Wheel vx,vy + IMU yaw,wz)',
        'F3a': 'Complementary Filter Standalone (Wheel vx,vy + IMU yaw,wz)',
        'F3b': 'Complementary Filter + Range-Rate Slip/Stall Gate (1.4s deque window)',
        'F4': '2D LiDAR Trimmed-ICP Scan Matching Odometry (/scan only)',
        'F5': 'LiDAR Odometry (F4: x,y,vx,vy) + IMU (yaw,wz) EKF [CORRECTED NAME: No Wheel/F3b input]',
        'F6': 'SLAM Toolbox Scan-to-Map Localization [S5 Loop Closure: UNVERIFIED]',
    }

    csv_files = sorted(glob.glob(os.path.join(OLD_BENCH_DIR, 'results', 'raw_trials', 'S*_trial_*.csv')))
    json_files = sorted(glob.glob(os.path.join(OLD_BENCH_DIR, 'results', 'raw_trials', 'S*_trial_*.json')))
    video_files = sorted(glob.glob(os.path.join(OLD_BENCH_DIR, 'videos', '**', '*.mp4'), recursive=True))

    report = {
        'preserved_benchmark_dir': OLD_BENCH_DIR,
        'preserved_counts': {
            'raw_trial_csvs': len(csv_files),
            'raw_trial_jsons': len(json_files),
            'mp4_videos': len(video_files),
        },
        'estimator_corrected_nomenclature': estimator_corrected_names,
        'audit_corrections': {
            'F5_true_inputs': 'odom0=/odometry/lidar_odom (x, y, vx, vy) + imu0=/imu (yaw, wz). Wheel odometry (/odom) and F3b (/odometry/comp_slip_gate) are NOT used in F5.',
            'F6_S5_loop_closure_status': 'UNVERIFIED in previous S5 (2.7m out-and-back trajectory had <10 pose-graph chain nodes; accuracy came from local scan-to-map matching, not pose-graph loop closure).',
            'lounge_ABCD_videos_classification': 'Ground Truth-guided manual waypoint tracking tests (using /ground_truth/world_odom closed-loop proportional controller), NOT sensor-driven Nav2 obstacle avoidance.',
            'missing_S1_videos_in_old_dir': [
                'S1_trial_01_identical.mp4',
                'S1_trial_06_varied_seed_seed1001.mp4'
            ],
        },
        'scenarios': {}
    }

    for sk in ['S1', 'S2', 'S3', 'S4', 'S5']:
        sk_csvs = sorted(glob.glob(os.path.join(OLD_BENCH_DIR, 'results', 'raw_trials', f'{sk}_trial_*.csv')))
        per_trial = [compute_trial_both_modes(p, codes_all) for p in sk_csvs]
        scen_stats = {}
        for c in codes_all:
            c_list = [t[c] for t in per_trial if c in t]
            if not c_list:
                continue
            scen_stats[c] = {
                'trials_count': len(c_list),
                'zoh_ate_rmse_m': {
                    'mean': round(float(np.mean([x['zoh_ate_rmse_m'] for x in c_list])), 4),
                    'std': round(float(np.std([x['zoh_ate_rmse_m'] for x in c_list])), 4),
                    'p95': round(float(np.percentile([x['zoh_ate_rmse_m'] for x in c_list], 95.0)), 4),
                    'worst_max': round(float(np.max([x['zoh_ate_max_m'] for x in c_list])), 4),
                },
                'timestamp_interpolated_ate_rmse_m': {
                    'mean': round(float(np.mean([x['interp_ate_rmse_m'] for x in c_list])), 4),
                    'std': round(float(np.std([x['interp_ate_rmse_m'] for x in c_list])), 4),
                    'p95': round(float(np.percentile([x['interp_ate_rmse_m'] for x in c_list], 95.0)), 4),
                    'worst_max': round(float(np.max([x['interp_ate_max_m'] for x in c_list])), 4),
                },
                'zoh_yaw_rmse_deg': {
                    'mean': round(float(np.mean([x['zoh_yaw_rmse_deg'] for x in c_list])), 4),
                    'std': round(float(np.std([x['zoh_yaw_rmse_deg'] for x in c_list])), 4),
                },
                'timestamp_interpolated_yaw_rmse_deg': {
                    'mean': round(float(np.mean([x['interp_yaw_rmse_deg'] for x in c_list])), 4),
                    'std': round(float(np.std([x['interp_yaw_rmse_deg'] for x in c_list])), 4),
                },
                'mean_sample_hold_lag_ms': round(float(np.mean([x['mean_sample_hold_lag_ms'] for x in c_list])), 2),
            }
        report['scenarios'][sk] = scen_stats

    out_json = os.path.join(NEW_EXP_DIR, 'results', 'previous_75trial_zoh_vs_interpolated_recomputation.json')
    with open(out_json, 'w') as f:
        json.dump(report, f, indent=2)
    print(f"Saved side-by-side ZOH vs Interpolated report to {out_json}")


if __name__ == '__main__':
    main()
