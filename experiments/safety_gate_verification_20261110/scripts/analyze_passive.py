#!/usr/bin/env python3
"""usage: analyze_passive.py RUN_DIR  -- summarize relaxed_passive.jsonl (cap0 vs original, cap60 vs Nav2) around the stop."""
import json, re, sys
import numpy as np
d = sys.argv[1]
rows = [json.loads(l) for l in open(f'{d}/relaxed_passive.jsonl')]
c0 = [r for r in rows if r['who'] == 'cap0']; c60 = [r for r in rows if r['who'] == 'cap60']
print(f'cap0 msgs {len(c0)}, cap60 msgs {len(c60)}')
# 1) cap0 (dry run, default parameters) vs the original explore_lite marker count
pairs = [(r['frontiers_kept'], r['original_marker_count']) for r in c0 if r.get('original_marker_count') is not None and (r.get('original_marker_age') or 9) < 1.2]
if pairs:
    eq = sum(1 for a, b in pairs if a == b)
    print(f'cap0 frontier count == original marker count in {eq}/{len(pairs)} paired samples (age < 1.2 s)')
    diffs = [(a, b) for a, b in pairs if a != b][:6]
    print('  examples of mismatch (relaxed cap0, original):', diffs)
# 2) log: when did the original stop?
log = open(f'{d}/launch.log', errors='ignore').read()
stop = re.search(r'\[explore_node\]: No frontiers found, stopping', log)
print('original explore stop message present:', bool(stop))
# 3) cap60 at the end of the run (after the original stopped)
last = c60[-1] if c60 else None
if last:
    print('last cap60 message:', {k: last[k] for k in ('sim_t', 'frontiers_found', 'frontiers_kept', 'reachable', 'reach_checked', 'bfs_visited', 'bfs_traversed_nonzero', 'bfs_max_cost', 'start_dist_m', 'chosen')})
# 4) Nav2 vs costmap flags over the whole run
tot = ok = agree = 0; cm_ok_nav_fail = []; cm_fail_nav_ok = 0
for r in c60:
    for c in r.get('nav2_check', []):
        if c['nav2'] in ('no_goal', 'timeout'): continue
        tot += 1; n2 = c['nav2'] == 'ok'; ok += n2; agree += (n2 == bool(c.get('costmap_flag')))
        if c.get('costmap_flag') and not n2: cm_ok_nav_fail.append((r['sim_t'], c['m']))
        if (not c.get('costmap_flag')) and n2: cm_fail_nav_ok += 1
print(f'Nav2 planner checked {tot} candidates: ok {ok}; costmap-flag agrees with Nav2 in {agree}/{tot}; costmap says reachable but Nav2 fails: {len(cm_ok_nav_fail)} (e.g. {cm_ok_nav_fail[:4]}); costmap says unreachable but Nav2 ok: {cm_fail_nav_ok}')
# 5) timeline of cap0 vs cap60 candidate counts
print('sim_t  cap0.kept cap60.kept cap60.reachable  cap60 bfs_traversed_nonzero')
c60i = {round(r['sim_t']): r for r in c60}
for r in c0[::12][:40]:
    q = c60i.get(round(r['sim_t'])) or {}
    print(f"{r['sim_t']:7.1f} {r['frontiers_kept']:4d} {q.get('frontiers_kept','-')!s:>6} {q.get('reachable','-')!s:>6} {q.get('bfs_traversed_nonzero','-')!s:>8}")
