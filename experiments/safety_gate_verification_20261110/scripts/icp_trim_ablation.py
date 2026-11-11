#!/usr/bin/env python3
"""Offline ablation: which part of F4's point-to-point ICP causes the wall-parallel failure? Copy of F4's icp_2d with the trimming percentile and
correspondence gate as arguments (repo code untouched). Replays full probe sequences with F4's keyframe logic.  usage: icp_trim_ablation.py RUN_DIR..."""
import sys, json, math
import numpy as np
def se2(p):
    c, s = math.cos(p[2]), math.sin(p[2]); T = np.eye(3); T[:2, :2] = [[c, -s], [s, c]]; T[:2, 2] = p[:2]; return T
def mat2(T): return np.array([T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])])
def icp(src, dst, init, pct=85.0, gate=0.30, iters=18):
    T = init.copy()
    for _ in range(iters):
        tp = src @ T[:2, :2].T + T[:2, 2]; d2 = ((tp[:, None, :] - dst[None, :, :]) ** 2).sum(2); nn = d2.argmin(1); md = d2[np.arange(len(src)), nn]
        idx = np.where(md < gate ** 2)[0]
        if len(idx) < 12: break
        keep = idx[md[idx] <= np.percentile(md[idx], pct)] if pct < 100 else idx
        if len(keep) < 10: keep = idx
        p = tp[keep]; q = dst[nn[keep]]; pm, qm = p.mean(0), q.mean(0); U, _, Vt = np.linalg.svd((p - pm).T @ (q - qm)); Rd = Vt.T @ U.T
        if np.linalg.det(Rd) < 0: Vt[1] *= -1; Rd = Vt.T @ U.T
        td = qm - Rd @ pm; D = np.eye(3); D[:2, :2] = Rd; D[:2, 2] = td; T = D @ T
        if math.hypot(*td) + abs(math.atan2(Rd[1, 0], Rd[0, 0])) < 1e-4: break
    return T
for run in sys.argv[1:]:
    R = json.load(open(f'{run}/probe.json'))['rows']; Z = np.load(f'{run}/probe_scans.npz'); a0 = float(Z['angle_min']); da = float(Z['angle_inc'])
    P = []
    for r in Z['ranges']:
        ang = a0 + np.arange(len(r)) * da; v = np.isfinite(r) & (r >= 0.15) & (r <= 9.5); P.append(np.column_stack((r[v] * np.cos(ang[v]) + 0.103, r[v] * np.sin(ang[v]))))
    gt = np.array([x['gt'] for x in R]); g0 = gt[0]; c, s = math.cos(-g0[2]), math.sin(-g0[2])
    true = np.array([[c * (g[0] - g0[0]) - s * (g[1] - g0[1]), s * (g[0] - g0[0]) + c * (g[1] - g0[1])] for g in gt])
    print(f'== {run}  (GT travel {np.hypot(*true[-1]):.3f} m)')
    for name, pct, gate in (('F4 as is: trim 85%, gate 0.30', 85.0, 0.30), ('no trimming, gate 0.30', 100.0, 0.30), ('trim 85%, gate 0.60', 85.0, 0.60), ('no trimming, gate 0.60', 100.0, 0.60), ('no trimming, gate 1.00', 100.0, 1.00)):
        kf = P[0]; Tok = np.eye(3); Tkc = np.eye(3); est = [np.zeros(2)]
        for i in range(1, len(P)):
            Tkc = icp(P[i], kf, Tkc, pct, gate); Tc = Tok @ Tkc; est.append(mat2(Tc)[:2])
            if math.hypot(Tkc[0, 2], Tkc[1, 2]) >= 0.12 or abs(math.atan2(Tkc[1, 0], Tkc[0, 0])) >= 0.12: kf = P[i]; Tok = Tc.copy(); Tkc = np.eye(3)
        est = np.array(est); print(f'   {name:34s} travel {np.hypot(*est[-1]):.3f} m | final err {np.hypot(*(est[-1]-true[-1])):.3f} m')
