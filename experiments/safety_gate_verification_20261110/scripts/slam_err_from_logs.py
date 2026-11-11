#!/usr/bin/env python3
"""Re-analysis of existing run data only (no new simulation). For each run dir given:
- fit sim_time<->wall_time from 'Message Filter dropping ... at time X' warnings (sim stamp + wall stamp of the same line)
- extract Nav2 'Begin navigating from current location (x, y)' (SLAM map-frame pose, ~every 2 s), convert with the run's align
  to world and compare with ground truth (timeseries.csv, 5 s) linearly interpolated.
usage: slam_err_from_logs.py RUNS_DIR run... """
import csv, json, math, re, sys
import numpy as np
root = sys.argv[1]
for r in sys.argv[2:]:
    d = f'{root}/{r}'
    s = json.load(open(f'{d}/summary.json')); al = s['align']
    log = open(f'{d}/launch.log', errors='ignore').read().splitlines()
    pairs = []; nav = []
    for ln in log:
        m = re.search(r'\[(\d+\.\d+)\].*dropping message: frame .* at time ([\d.]+)', ln)
        if m: pairs.append((float(m[1]), float(m[2])))
        m = re.search(r'\[bt_navigator\]: Begin navigating from current location \(([-\d.]+), ([-\d.]+)\)', ln)
        if m:
            w = re.search(r'\[(\d+\.\d+)\]', ln)
            nav.append((float(w[1]), float(m[1]), float(m[2])))
    P = np.array(pairs); a, b = np.polyfit(P[:, 0], P[:, 1], 1)
    resid = P[:, 1] - (a * P[:, 0] + b)
    rows = list(csv.DictReader(open(f'{d}/timeseries.csv')))
    T = np.array([float(x['sim_s']) for x in rows]); GX = np.array([float(x['gt_x']) for x in rows]); GY = np.array([float(x['gt_y']) for x in rows])
    c, sn = math.cos(al['theta']), math.sin(al['theta'])
    out = []
    for w, mx, my in nav:
        t = a * w + b
        if t < T[0] or t > T[-1]: continue
        wx = c * mx - sn * my + al['tx']; wy = sn * mx + c * my + al['ty']
        gx = np.interp(t, T, GX); gy = np.interp(t, T, GY)
        out.append((round(t, 1), round(wx, 2), round(wy, 2), round(float(gx), 2), round(float(gy), 2), round(math.hypot(wx - gx, wy - gy), 2)))
    print(f'== {r}: sim/wall fit slope {a:.3f} (RTF), rms resid {np.sqrt((resid**2).mean()):.2f}s, n_fit {len(P)}, nav samples {len(out)}')
    for o in out:
        if o[0] <= 110: print('  t=%6.1f slam_world=(%5.2f,%5.2f) gt_interp=(%5.2f,%5.2f) err=%.2f' % o)
    json.dump(out, open(f'{sys.argv[0].rsplit("/",1)[0]}/../post_analysis_20261111/slam_error/{r}_nav_pose_vs_gt.json', 'w'))
