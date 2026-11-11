#!/usr/bin/env python3
"""usage: summarize_stage.py latency BEFORE_TAG AFTER_TAG | nav TAG... | bench TAG..."""
import csv, json, os, sys
import numpy as np
E = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
mode = sys.argv[1]
if mode == 'latency':
    for lab, t in (('before', sys.argv[2]), ('after ', sys.argv[3])):
        d = json.load(open(f'{E}/depth_analysis/{t}/depth_latency.json'))
        a = np.array([float(r['status_age']) for r in csv.DictReader(open(f'{E}/depth_analysis/{t}/status_ages.csv')) if float(r['wall_s']) > 5])
        print(lab, t, 'rtf %.3f' % d['rtf_overall'], 'drop %.3f' % d['frames']['drop_fraction'],
              'status age p95 %.3f p99 %.3f max %.3f margin %.3f' % (np.percentile(a, 95), np.percentile(a, 99), a.max(), 0.38 - a.max()),
              '| cpu filter %.0f gazebo %.0f navstack %.0f sys %.1f' % (d['cpu_summary']['depth_filter']['mean'], d['cpu_summary']['gazebo']['mean'], d['cpu_summary']['nav2']['mean'], d['system_cpu_pct']['mean']),
              '| events', [e[1] for e in d['status_code_events']])
elif mode == 'nav':
    for t in sys.argv[2:]:
        print('==', t); print(open(f'{E}/sim_settings_20261111/nav/{t}/result.txt').read())
elif mode == 'bench':
    keys = ('marked_z_min_all', 'marked_points_total_all', 'local_cells_eq100', 'local_cells_ge99', 'global_cells_ge99', 'corridor_marked_points', 'corridor_marked_z',
            'corridor_local_cells_ge99', 'corridor_global_cells_ge99', 'marked_points_near_rail', 'local_cost_on_rail_line', 'global_cost_on_rail_line')
    for t in sys.argv[2:]:
        p = json.load(open(f'{E}/sim_settings_20261111/bench/{t}/probe.json'))
        print(t, {k: p.get(k) for k in keys if p.get(k) is not None})
