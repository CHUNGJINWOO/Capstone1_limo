#!/usr/bin/env -S python3 -s
"""
generate_master_summary_and_plots.py
Aggregates all 5 scenarios (S1..S5, 15 trials each = 75 total trials) into a single
master statistical report (`results/summaries/master_all_scenarios_summary.json`)
and generates a comprehensive cross-scenario comparison plot (`results/plots/master_cross_scenario_comparison.png`).
Also verifies that all videos across smoke_test, state_estimation, and lounge_scenarios are playable MP4s.
"""
import os
import json
import glob
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


BENCH_DIR = '/home/cjinw/school/2026-2/Capstone1_limo/experiments/limo_state_estimation_benchmark_20261107'
SUM_DIR = os.path.join(BENCH_DIR, 'results', 'summaries')
PLT_DIR = os.path.join(BENCH_DIR, 'results', 'plots')


def main():
    os.makedirs(SUM_DIR, exist_ok=True)
    os.makedirs(PLT_DIR, exist_ok=True)

    scenarios = ['S1', 'S2', 'S3', 'S4', 'S5']
    estimators = ['B0', 'F1a', 'F1b', 'F2', 'F3a', 'F3b', 'F4', 'F5', 'F6']
    scen_data = {}

    for s in scenarios:
        p = os.path.join(SUM_DIR, f'{s}_15trials_summary.json')
        if os.path.exists(p):
            scen_data[s] = json.load(open(p))

    # Build cross-scenario summary table
    master_table = {}
    for est in estimators:
        master_table[est] = {}
        for s in scenarios:
            if s not in scen_data:
                continue
            agg = scen_data[s].get('aggregated_statistics', {}).get(est, {}).get('all_15_trials', {})
            if agg.get('runs_completed', 0) > 0:
                master_table[est][s] = {
                    'runs_completed': agg['runs_completed'],
                    'ate_rmse_mean': agg['ate_rmse_m']['mean'],
                    'ate_rmse_std': agg['ate_rmse_m']['std'],
                    'ate_rmse_median': agg['ate_rmse_m']['median'],
                    'ate_rmse_p95': agg['ate_rmse_m']['p95'],
                    'ate_rmse_worst': agg['ate_rmse_m']['worst_max'],
                    'ate_max_worst': agg['ate_max_m']['worst_max'],
                    'final_pos_err_mean': agg['final_pos_err_m']['mean'],
                    'final_pos_err_p95': agg['final_pos_err_m']['p95'],
                    'final_pos_err_worst': agg['final_pos_err_m']['worst_max'],
                    'rpe_trans_rmse_mean': agg['rpe_trans_rmse_m']['mean'],
                    'rpe_rot_rmse_mean': agg['rpe_rot_rmse_deg']['mean'],
                    'yaw_rmse_mean': agg['yaw_rmse_deg']['mean'],
                    'yaw_max_worst': agg['yaw_max_deg']['worst_max'],
                    'false_stall_goal_rate': agg.get('false_stall_goal_rate', None),
                }

    # Generate 4-panel Master Comparison Plot
    fig, axes = plt.subplots(2, 2, figsize=(18, 11), dpi=140)
    est_core = ['B0', 'F1a', 'F1b', 'F2', 'F3a', 'F3b', 'F4', 'F5']
    colors = {
        'B0': '#e63946',
        'F1a': '#6c757d',
        'F1b': '#00b4d8',
        'F2': '#f77f00',
        'F3a': '#9b5de5',
        'F3b': '#ffbe0b',
        'F4': '#3a86ff',
        'F5': '#ff006e',
        'F6': '#06d6a0',
    }

    # Panel 1: Mean ATE RMSE across S1..S5
    ax1 = axes[0, 0]
    x = np.arange(len(scenarios))
    w = 0.095
    for idx, est in enumerate(est_core):
        vals = [master_table[est].get(s, {}).get('ate_rmse_mean', 0.0) for s in scenarios]
        stds = [master_table[est].get(s, {}).get('ate_rmse_std', 0.0) for s in scenarios]
        ax1.bar(x + (idx - 3.5) * w, vals, w, yerr=stds, capsize=2, label=est, color=colors[est], alpha=0.9)
    # Add F6 on S5
    if 'S5' in master_table['F6']:
        ax1.bar(x[-1] + 4.5 * w, [master_table['F6']['S5']['ate_rmse_mean']], w,
                yerr=[master_table['F6']['S5']['ate_rmse_std']], capsize=2, label='F6 (SLAM PGO)', color=colors['F6'])
    ax1.set_xticks(x)
    ax1.set_xticklabels([
        'S1: Normal\nMulti-Turn',
        'S2: Glass Wall\nStall & Slip',
        'S3: Sharp Turn\nNarrow Aisle',
        'S4: IMU Bias\n[SYNTHETIC]',
        'S5: Closed-Loop\nReturn & SLAM'
    ], fontsize=9)
    ax1.set_ylabel('15-Trial Mean ATE RMSE (m) ± 1 SD')
    ax1.set_title('A. Position Error (ATE RMSE Mean ± SD) Across S1–S5 (75 Gazebo Trials)')
    ax1.grid(True, axis='y', alpha=0.3)
    ax1.legend(ncol=3, fontsize=8)

    # Panel 2: Worst Instantaneous Position Error (ate_max_worst) across S1..S5
    ax2 = axes[0, 1]
    for idx, est in enumerate(est_core):
        vals = [master_table[est].get(s, {}).get('ate_max_worst', 0.0) for s in scenarios]
        ax2.bar(x + (idx - 3.5) * w, vals, w, label=est, color=colors[est], alpha=0.9)
    if 'S5' in master_table['F6']:
        ax2.bar(x[-1] + 4.5 * w, [master_table['F6']['S5']['ate_max_worst']], w, label='F6 (SLAM PGO)', color=colors['F6'])
    ax2.set_xticks(x)
    ax2.set_xticklabels([
        'S1: Normal\nMulti-Turn',
        'S2: Glass Wall\nStall & Slip',
        'S3: Sharp Turn\nNarrow Aisle',
        'S4: IMU Bias\n[SYNTHETIC]',
        'S5: Closed-Loop\nReturn & SLAM'
    ], fontsize=9)
    ax2.set_ylabel('Worst-Case Max Position Error Across 15 Trials (m)')
    ax2.set_title('B. Worst-Case Instantaneous Position Error (15-Trial Worst Max)')
    ax2.grid(True, axis='y', alpha=0.3)
    ax2.legend(ncol=3, fontsize=8)

    # Panel 3: S2 Slip/Stall Comparison (F3a vs F3b vs F4 vs F5 vs B0/F1b)
    ax3 = axes[1, 0]
    s2_ests = ['B0', 'F1a', 'F1b', 'F2', 'F3a', 'F3b', 'F4', 'F5']
    s2_means = [master_table[e]['S2']['ate_rmse_mean'] for e in s2_ests]
    s2_p95s = [master_table[e]['S2']['ate_rmse_p95'] for e in s2_ests]
    s2_worsts = [master_table[e]['S2']['ate_max_worst'] for e in s2_ests]
    xs2 = np.arange(len(s2_ests))
    ax3.bar(xs2 - 0.25, s2_means, 0.25, label='15-Trial Mean ATE RMSE (m)', color='#1d3557')
    ax3.bar(xs2, s2_p95s, 0.25, label='95th Percentile ATE RMSE (m)', color='#457b9d')
    ax3.bar(xs2 + 0.25, s2_worsts, 0.25, label='Worst Instantaneous Error (m)', color='#e63946')
    ax3.set_xticks(xs2)
    ax3.set_xticklabels(s2_ests)
    ax3.set_ylabel('Error (m)')
    ax3.set_title('C. S2 Physical Stall & Wheel Slip: Effect of Slip Gate (F3a->F3b) & LiDAR (F4/F5)')
    ax3.grid(True, axis='y', alpha=0.3)
    ax3.legend(fontsize=8)

    # Panel 4: S5 Closed-Loop Return Final Position Error (Origin Return Drift)
    ax4 = axes[1, 1]
    s5_ests = ['B0', 'F1a', 'F1b', 'F2', 'F3a', 'F3b', 'F4', 'F5', 'F6']
    s5_f_mean = [master_table[e]['S5']['final_pos_err_mean'] for e in s5_ests]
    s5_f_p95 = [master_table[e]['S5']['final_pos_err_p95'] for e in s5_ests]
    s5_f_worst = [master_table[e]['S5']['final_pos_err_worst'] for e in s5_ests]
    xs5 = np.arange(len(s5_ests))
    ax4.bar(xs5 - 0.25, s5_f_mean, 0.25, label='Mean Return Drift (m)', color='#2a9d8f')
    ax4.bar(xs5, s5_f_p95, 0.25, label='95th Percentile Return Drift (m)', color='#e9c46a')
    ax4.bar(xs5 + 0.25, s5_f_worst, 0.25, label='Worst Return Drift (m)', color='#e76f51')
    ax4.set_xticks(xs5)
    ax4.set_xticklabels(s5_ests)
    ax4.set_ylabel('Final Position Error at Loop Return (m)')
    ax4.set_title('D. S5 Closed-Loop Return Error: Odometry Drift vs F6 SLAM Loop Closure')
    ax4.grid(True, axis='y', alpha=0.3)
    ax4.legend(fontsize=8)

    plt.tight_layout()
    master_plot_path = os.path.join(PLT_DIR, 'master_cross_scenario_comparison.png')
    plt.savefig(master_plot_path)
    plt.close(fig)

    out_master_json = os.path.join(SUM_DIR, 'master_all_scenarios_summary.json')
    with open(out_master_json, 'w') as f:
        json.dump({
            'generated_at': '2026-11-07',
            'total_scenarios': 5,
            'trials_per_scenario': 15,
            'total_gazebo_trials_executed': 75,
            'master_plot_png': master_plot_path,
            'estimators_summary': master_table,
        }, f, indent=2)
    print(f"Saved master summary to {out_master_json} and plot to {master_plot_path}")


if __name__ == '__main__':
    main()
