#!/usr/bin/env python3
"""Offline: can a LiDAR-estimation failure be told apart from a physical block with signals the gate does not use today?
Over the interval [latch-1.5 s, latch] (ctrl: last 1.5 s of drive) compare, for each recorded run (estimator f4 = gate observer):
  GT travel | wheel travel | F4 travel | scan-derived forward travel from raw ranges (median range change of rays within +-10 deg of the heading and of the opposite
  direction; independent of ICP) | virtual-bumper clearance (min front range)
usage: stall_discriminator.py RUNS_DIR"""
import sys, json, math
import numpy as np
root = sys.argv[1]
for sc in ('wall', 'wall2', 'block', 'ctrl'):
    d = f'{root}/f4__{sc}'; R = json.load(open(f'{d}/probe.json'))['rows']; Z = np.load(f'{d}/probe_scans.npz'); rg = Z['ranges']; a0 = float(Z['angle_min']); da = float(Z['angle_inc'])
    ang = a0 + np.arange(rg.shape[1]) * da
    fr = (np.abs(ang) < math.radians(10)); bk = (np.abs(np.abs(ang) - math.pi) < math.radians(10))
    def med(i, m): r = rg[i][m]; r = r[np.isfinite(r) & (r > 0.15) & (r < 9.5)]; return float(np.median(r)) if len(r) else float('nan')
    t = np.array([r['t'] for r in R]); lat = next((r['t'] for r in R if r['gate'] == 'STALL_LATCHED'), None)
    t1 = lat if lat is not None else t[-1] if sc != 'ctrl' else t[min(len(t) - 1, int(np.argmax(np.array([r['gt'][1] for r in R]) - R[0]['gt'][1] > 1.7)))]
    i1 = int(np.argmin(abs(t - t1))); i0 = int(np.argmin(abs(t - (t1 - 1.5))))
    dist = lambda k: math.hypot(R[i1][k][0] - R[i0][k][0], R[i1][k][1] - R[i0][k][1])
    d_front = med(i0, fr) - med(i1, fr); d_back = med(i1, bk) - med(i0, bk); sc_trav = np.nanmean([d_front, d_back])
    print(f'{sc:6s} window {t[i0]:.1f}-{t[i1]:.1f}s gate_at_end={R[i1]["gate"]:14s} | GT {dist("gt"):.3f}  wheel {dist("wheel"):.3f}  F4 {dist("lidar"):.3f}  scan-derived {sc_trav:+.3f} (front {d_front:+.3f}, back {d_back:+.3f}) m | min range {np.nanmin(rg[i1][np.isfinite(rg[i1])]):.2f} m, front median {med(i1, fr):.2f} m')
