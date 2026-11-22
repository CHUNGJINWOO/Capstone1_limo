"""Aggregate run_depth_targets.py results: one row per run, then a per (config, layer, target) summary.
usage: python3 aggregate.py <results root> [run dir names...]   (reads <root>/<name>/run/*/run.json; writes nothing)"""
import json, sys
from pathlib import Path

root = Path(sys.argv[1]); names = sys.argv[2:] or sorted(p.name for p in root.iterdir() if (p / 'run').is_dir())
rows = []
for nm in names:
    for f in sorted((root / nm / 'run').glob('*/run.json')):
        d = json.loads(f.read_text()); st = d.get('stationary') or {}
        rows.append(dict(dir=nm, run=d['run'], target=d['target'], config=d['config'], layer=d.get('depth_layer'), result=d.get('result'),
                         nav2=d.get('nav2'), clear=d.get('min_clear_m'), blocked=d.get('blocked_s'), raw=st.get('raw'), filt=st.get('filter'),
                         lidar=st.get('lidar'), loc=st.get('local_costmap_lethal'), glob=st.get('global_costmap_lethal'),
                         first_filter=d.get('first_filter_hit_range_m'), first_lethal=d.get('first_lethal_range_m'), gap=d.get('filter_gap_max_s'),
                         resets=len(d.get('pre_test_resets') or []), gate_fault=d.get('gate_faulted_during_test'), stop_fault=d.get('stop_after_fault_s'),
                         rtf=d.get('rtf_stationary'), load=(d.get('load_avg') or [None])[0]))
cols = ['dir', 'run', 'result', 'nav2', 'clear', 'blocked', 'raw', 'filt', 'lidar', 'loc', 'glob', 'first_filter', 'first_lethal', 'gap', 'resets', 'gate_fault', 'stop_fault', 'rtf', 'load']
print('| ' + ' | '.join(cols) + ' |'); print('|' + '---|' * len(cols))
for r in rows: print('| ' + ' | '.join('' if r[c] is None else str(r[c]) for c in cols) + ' |')
print()
groups = {}
for r in rows: groups.setdefault((r['config'], r['layer'], r['target']), []).append(r)
print('| config | layer | target | results | min clearance (m) | stationary raw/filter/lidar/local lethal (first run) |'); print('|---|---|---|---|---|---|')
for (c, l, t), rs in sorted(groups.items(), key=lambda k: (k[0][2], k[0][0], str(k[0][1]))):
    res = ', '.join(sorted({x['result'] for x in rs}, key=str)) + f' ({len(rs)})'
    cl = [x['clear'] for x in rs if x['clear'] is not None]; r0 = rs[0]
    print(f"| {c} | {l} | {t} | {res}: " + '/'.join(str(x['result']) for x in rs) + f" | {min(cl) if cl else ''} | {r0['raw']}/{r0['filt']}/{r0['lidar']}/{r0['loc']} |")
