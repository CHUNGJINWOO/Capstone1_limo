#!/usr/bin/env python3
"""usage: summarize_estimators.py RUNS_DIR  -> one comparable table (per scenario, per estimator)"""
import sys, json, math, glob, os
import numpy as np
root = sys.argv[1]; rows_out = []
for d in sorted(glob.glob(f'{root}/*__*')):
    if not os.path.exists(f'{d}/probe.json'): continue
    est, sc = os.path.basename(d).split('__'); J = json.load(open(f'{d}/probe.json')); R = J['rows']
    gt = np.array([r['gt'] for r in R]); g0 = gt[0]; c, s = math.cos(-g0[2]), math.sin(-g0[2])
    body = lambda p: np.array([c * (p[0] - g0[0]) - s * (p[1] - g0[1]), s * (p[0] - g0[0]) + c * (p[1] - g0[1])])
    tr = np.array([body(g) for g in gt]); trav = np.hypot(*tr.T)
    def series(k): a = np.array([r[k][:2] for r in R]); return a - a[0]
    act, f4, wh = series('active'), series('lidar'), series('wheel')
    err = np.hypot(*(act - tr).T); on = np.where(err > 0.05)[0]
    onset = f'{trav[on[0]]:.2f}m@{R[on[0]]["t"]:.1f}s' if len(on) else 'none'
    first = lambda st: next((f'{r["t"]:.1f}s' for r in R if r['gate'] == st), '-')
    inl = [r['f4_inl'] for r in R if r['f4_inl'] is not None]
    yawe = math.degrees(np.array([r['active'][2] for r in R])[-1] - R[0]['active'][2])
    rows_out.append((sc, est, trav[-1], np.hypot(*wh[-1]), np.hypot(*f4[-1]), np.hypot(*act[-1]), err[-1], err.max(), onset, first('STALL_PENDING'), first('STALL_LATCHED'), min(inl) if inl else float('nan'), float(np.mean(inl)) if inl else float('nan'), yawe))
print('scenario  estimator      GT_m  wheel_m  F4_m  active_m | active_err_end/max_m  onset(>5cm)  1st_PENDING  LATCHED  | F4 inliers min/mean | active yaw drift deg')
for r in sorted(rows_out):
    print('%-9s %-13s %5.3f  %5.3f  %5.3f  %5.3f | %5.3f / %5.3f   %-12s %-8s %-8s | %4.0f / %4.0f | %+5.1f' % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11], r[12], r[13]))
