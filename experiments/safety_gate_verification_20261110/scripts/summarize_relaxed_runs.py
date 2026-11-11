#!/usr/bin/env python3
"""usage: summarize_relaxed_runs.py RUNS_DIR run... ; prints one comparable block per run (explore_relaxed experiments)."""
import json, os, re, subprocess, sys
root = sys.argv[1]
here = os.path.dirname(os.path.abspath(__file__))
for r in sys.argv[2:]:
    d = f'{root}/{r}'
    s = json.load(open(f'{d}/summary.json')); lc = s['log_counts']
    q = None
    try:
        subprocess.run(['python3', f'{here}/analyze_map_quality.py', d], capture_output=True, check=True)
        q = json.load(open(f'{d}/map_quality.json'))
    except Exception:
        pass
    log = open(f'{d}/launch.log', errors='ignore').read()
    n_unreach = len(re.findall(r'not reachable [(]', log))
    stop = re.findall(r'(No frontiers found, stopping|All frontiers traversed/tried out, stopping)', log)[:1]
    print(f"== {r}: stop={s['stop_reason']} sim={s['sim_s']} path={s['gt_path_m']} m | gate {s['gate_counters']} pubs={s['cmd_vel_publishers']}")
    print(f"   goals_sent={lc['goals_sent (Begin navigating)']} ctrl_no_progress={lc['controller_failed_to_make_progress']} planner_fail={lc['planner_no_valid_path']} recoveries={lc['behavior_recovery_run']} "
          f"explore_stop={stop} not_reachable_lines={n_unreach} waited={len(re.findall('waiting instead of stopping', log))} "
          f"FAIL-SAFE={len(re.findall('FAIL-SAFE', log))} STALL={len(re.findall('STALL LATCHED', log))}")
    if q:
        print(f"   map: interior_known={q['interior_known_fraction']} unknown_in_room={q['unknown_inside_total_m2']} m2 clusters={[(c['area_m2'], c['centroid_world']) for c in q['unknown_inside_clusters_ge_0p25m2'][:2]]} wall_cov={q['outer_wall_mean_coverage']} wall_offset={q['outer_wall_mean_abs_offset_m']}")
    print(f"   slam_err={s.get('slam_err_xy_m')} rtf_cpu_sys={s.get('cpu_system_pct')}")
