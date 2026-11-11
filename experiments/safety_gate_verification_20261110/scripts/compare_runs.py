#!/usr/bin/env python3
"""Aggregate every auto_mapping/<run>/ (summary.json, timeseries.csv, launch.log, map_quality.json)
into one comparison table. Run analyze_map_quality.py on each run first.
usage: compare_runs.py AUTO_MAPPING_DIR [run_name ...]   -> prints markdown, writes comparison.md/json"""
import csv, json, os, re, sys

root = sys.argv[1]
runs = sys.argv[2:] or sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)) and not d.startswith('_'))
rows = []
for r in runs:
    d = os.path.join(root, r)
    try:
        s = json.load(open(os.path.join(d, 'summary.json')))
    except Exception:
        continue
    q = {}
    try:
        q = json.load(open(os.path.join(d, 'map_quality.json')))
    except Exception:
        pass
    ts = list(csv.DictReader(open(os.path.join(d, 'timeseries.csv'))))
    rtf_overall = None
    if len(ts) > 2:
        w0, s0 = float(ts[1]['wall_s']), float(ts[1]['sim_s']); w1, s1 = float(ts[-1]['wall_s']), float(ts[-1]['sim_s'])
        rtf_overall = (s1 - s0) / (w1 - w0) if w1 > w0 else None
    rtf5 = [float(x['rtf_5s']) for x in ts if x['rtf_5s']]
    log = open(os.path.join(d, 'launch.log'), errors='ignore').read()
    fs = re.findall(r'\[FAIL-SAFE[^\]]*\] ([^!]*)!', log)
    st = re.findall(r'\[STALL LATCHED\] ([^.]*)\.', log)
    lc = s['log_counts']
    # time at which the robot last moved >2 mm (sim s) to separate "ended" from "stuck"
    last_move = 0.0; prev = None
    for x in ts:
        if x['gt_x'] == '':
            continue
        p = (float(x['gt_x']), float(x['gt_y']))
        if prev is None or abs(p[0] - prev[0]) + abs(p[1] - prev[1]) > 0.01:
            last_move = float(x['sim_s'])
        prev = p
    rows.append(dict(
        run=r, drive='ackermann' if r.startswith('ack') else 'differential', stop=s['stop_reason'], wall_s=s['wall_s'], sim_s=s['sim_s'],
        rtf_overall=round(rtf_overall, 3) if rtf_overall else None, rtf5_min=round(min(rtf5), 3) if rtf5 else None,
        last_motion_sim_s=last_move, gt_path_m=s['gt_path_m'],
        known_m2=round(s['map']['known'], 1) if s.get('map') else None,
        interior_known_frac=q.get('interior_known_fraction'), wall_cov=q.get('outer_wall_mean_coverage'),
        wall_abs_offset_m=q.get('outer_wall_mean_abs_offset_m'), wall_std_m=q.get('outer_wall_mean_std_m'),
        unknown_inside_m2=q.get('unknown_inside_total_m2'),
        slam_err_mean_m=(s.get('slam_err_xy_m') or {}).get('mean'), slam_err_max_m=(s.get('slam_err_xy_m') or {}).get('max'),
        goals_sent=lc['goals_sent (Begin navigating)'], preemptions=lc['goal_preemptions'],
        ctrl_no_progress=lc['controller_failed_to_make_progress'], planner_fail=lc['planner_no_valid_path'],
        collision_ahead=lc['collision_ahead'], recoveries=lc['behavior_recovery_run'],
        gate_failsafe=s['gate_counters'].get('failsafe_events'), gate_stall=s['gate_counters'].get('stall_events'),
        gate_last_state=s['gate_last_flags'].get('state'), failsafe_cause=fs[:1], stall_cause=st[:1],
        stall_pending_events=sum(1 for t, x in s['gate_transitions'] if x == 'STALL_PENDING'),
        gate_blocked_out_frac=None, cmd_vel_publishers=s['cmd_vel_publishers'],
        min_scan_m=s['min_scan_m'], explore_stop_msg=lc['explore_no_frontiers_stop'],
        cpu_gazebo=(s['cpu_pct_of_one_core']['gazebo'] or {}).get('mean'), cpu_nav2=(s['cpu_pct_of_one_core']['nav2'] or {}).get('mean'),
        cpu_slam=(s['cpu_pct_of_one_core']['slam_toolbox'] or {}).get('mean'), cpu_gate=(s['cpu_pct_of_one_core']['safety_gate'] or {}).get('mean'),
        cpu_sys_mean=(s.get('cpu_system_pct') or {}).get('mean'),
        gap_max_sim={k: v['max'] for k, v in s['stream_gaps_sim'].items()},
        gap_over_0p38={k: v['over_0p38'] for k, v in s['stream_gaps_sim'].items()},
    ))

json.dump(rows, open(os.path.join(root, 'comparison.json'), 'w'), indent=1)
cols = ['run', 'stop', 'sim_s', 'rtf_overall', 'last_motion_sim_s', 'gt_path_m', 'known_m2', 'interior_known_frac', 'wall_cov', 'wall_abs_offset_m',
        'slam_err_mean_m', 'goals_sent', 'ctrl_no_progress', 'planner_fail', 'recoveries', 'gate_failsafe', 'gate_stall', 'stall_pending_events', 'cpu_sys_mean']
md = ['| ' + ' | '.join(cols) + ' |', '|' + '---|' * len(cols)]
for r in rows:
    md.append('| ' + ' | '.join(str(r[c]) for c in cols) + ' |')
md.append('')
for r in rows:
    md.append(f"- {r['run']}: failsafe_cause={r['failsafe_cause']} stall_cause={r['stall_cause']} min_scan={r['min_scan_m']} "
              f"gaps>0.38s={ {k: v for k, v in r['gap_over_0p38'].items() if v} } cpu(gazebo/nav2/slam/gate)={r['cpu_gazebo']}/{r['cpu_nav2']}/{r['cpu_slam']}/{r['cpu_gate']}")
open(os.path.join(root, 'comparison.md'), 'w').write('\n'.join(md) + '\n')
print('\n'.join(md))
