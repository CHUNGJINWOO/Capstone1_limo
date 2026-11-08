#!/usr/bin/env python3
"""
reevaluate_scenarios.py:
Authoritative re-evaluator for LIMO simulation raw records.
Preserves existing summary.json intact and outputs reevaluation_summary.json in each scenario directory.
Regenerates:
  - Measurements (Action execution, GT pose error, SAT clearance, standstill by phase, costmap clearing)
  - Telemetry quality & header stamp 0.0 reception clock approximation metadata
  - Independent verdicts (Strict Goal, Relaxed Goal, Collision-Free, Safe Stop, Clearing, Exploration)
  - Cohesive primary status and rationale
"""

import os
import sys
import json
import pathlib

BASE_DIR = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(BASE_DIR / 'output/simulation'))
from evaluation_lib import evaluate_scenario_from_records

TARGET_DIRS = [
    BASE_DIR / 'output/simulation/2026-10-24_phase_separated',
    BASE_DIR / 'output/simulation/2026-10-24_rigorous',
    BASE_DIR / 'output/simulation/2026-10-24_investigation'
]

def reevaluate_all():
    print("================================================================================")
    print("  AUTHORITATIVE RE-EVALUATION OF EXISTING SIMULATION RAW RECORDS")
    print("================================================================================\n")

    results_table = []

    for base_p in TARGET_DIRS:
        if not base_p.exists():
            continue
        print(f">>> Inspecting Directory: {base_p.name}")
        for sdir in sorted(base_p.iterdir()):
            if not sdir.is_dir():
                continue
            gt_file = sdir / 'gt_pose_history.jsonl'
            telem_file = sdir / 'telemetry.jsonl'
            if not (gt_file.exists() or telem_file.exists()):
                continue

            reeval = evaluate_scenario_from_records(sdir)

            # Save to separate reevaluation_summary.json (preserving summary.json!)
            out_file = sdir / 'reevaluation_summary.json'
            with open(out_file, 'w') as f:
                json.dump(reeval, f, indent=2)

            m = reeval['measurements']
            v = reeval['verdict']
            act = m['action_execution']
            ss = m['standstill_analysis']
            clr = m['costmap_clearing_verification']

            print(f"  [{sdir.name}] -> {out_file.name}")
            print(f"    Action Status: {act.get('final_action_status_str')} (code {act.get('final_action_status_code')})")
            print(f"    GT Error: {m.get('final_ground_truth_error'):.4f} m, SAT Clearance: {m.get('minimum_obstacle_clearance_meters'):.4f} m")
            print(f"    Standstill: Nav={ss.get('navigating_max_continuous_stop_sec')}s, Init={ss.get('initial_wait_stop_sec')}s, PostCancel={ss.get('post_cancel_stop_sec')}s")
            print(f"    Quality: {ss.get('telemetry_quality')}, Source: {ss.get('telemetry_source')}")
            if clr.get('roi_monitoring_enabled'):
                print(f"    Clearing: TotalROI={clr.get('total_roi_cells')}, Hold={clr.get('clearing_hold_duration_seconds')}s, Latency={clr.get('clearing_latency_display')}, Verified={clr.get('clearing_verified')}")
            print(f"    Verdicts: Strict={v.get('goal_reached_strict')}, Relaxed={v.get('goal_reached_relaxed')}, CollFree={v.get('collision_free_verified')}, SafeStop={v.get('safe_stop_verified')}, Clearing={v.get('clearing_verified')}")
            print(f"    Primary Status: {v.get('primary_status')}")
            print(f"    Rationale: {v.get('rationale')}\n")

            if 'phase_separated' in str(base_p):
                results_table.append({
                    'id': sdir.name,
                    'status': act.get('final_action_status_str'),
                    'gt_err': m.get('final_ground_truth_error'),
                    'clearance': m.get('minimum_obstacle_clearance_meters'),
                    'nav_stop': ss.get('navigating_max_continuous_stop_sec'),
                    'init_stop': ss.get('initial_wait_stop_sec'),
                    'post_cancel_stop': ss.get('post_cancel_stop_sec'),
                    'sim_dur': m.get('simulation_duration_seconds'),
                    'primary': v.get('primary_status'),
                    'verdicts': v,
                    'clr': clr
                })

    return results_table

if __name__ == '__main__':
    reevaluate_all()
