#!/usr/bin/env python3
"""Offline what-if on recorded probe scans (NOT applied anywhere): replay F4 (repo code) vs candidate improvements on the same scans.
 V0 F4 as is | V1 F4 + wheel-odom delta as ICP initial guess | V2 point-to-line ICP (init = previous) | V3 point-to-line + wheel init
usage: icp_variants_offline.py RUN_DIR [RUN_DIR...]"""
import os, sys, json, math, types
import numpy as np
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'capstone_ws', 'src', 'limo_capstone')))
from limo_capstone.lidar_scan_matcher_odom_node import LidarScanMatcherOdomNode as F4
obj = types.SimpleNamespace(laser_x_offset=0.103, max_iter=18, max_dist=0.30, last_inliers=0.0)
def se2(p):
    c, s = math.cos(p[2]), math.sin(p[2]); T = np.eye(3); T[:2, :2] = [[c, -s], [s, c]]; T[:2, 2] = p[:2]; return T
def mat2(T): return np.array([T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])])
def normals(pts, k=4):
    n = np.zeros_like(pts)
    for i in range(len(pts)):
        lo, hi = max(0, i - k), min(len(pts), i + k + 1); q = pts[lo:hi] - pts[lo:hi].mean(0); n[i] = np.linalg.eigh(q.T @ q)[1][:, 0]
    return n
def icp_p2l(src, dst, nd, init, iters=18, maxd=0.30):
    T = init.copy()
    for _ in range(iters):
        tp = src @ T[:2, :2].T + T[:2, 2]
        d2 = ((tp[:, None, :] - dst[None, :, :]) ** 2).sum(2); nn = d2.argmin(1); md = d2[np.arange(len(src)), nn]
        ok = md < maxd ** 2
        if ok.sum() < 12: break
        idx = np.where(ok)[0]; cut = np.percentile(md[idx], 85); idx = idx[md[idx] <= cut]
        p = tp[idx]; q = dst[nn[idx]]; n = nd[nn[idx]]; r = ((p - q) * n).sum(1)
        J = np.column_stack((n[:, 0], n[:, 1], p[:, 0] * n[:, 1] - p[:, 1] * n[:, 0]))
        H = J.T @ J + 1e-6 * np.eye(3); dx = -np.linalg.solve(H, J.T @ r)
        T = se2(dx) @ T
        if np.abs(dx).max() < 1e-5: break
    return T
for run in sys.argv[1:]:
    R = json.load(open(f'{run}/probe.json'))['rows']; Z = np.load(f'{run}/probe_scans.npz'); rg = Z['ranges']; a0 = float(Z['angle_min']); da = float(Z['angle_inc'])
    P = []
    for r in rg:
        ang = a0 + np.arange(len(r)) * da; v = np.isfinite(r) & (r >= 0.15) & (r <= 9.5); P.append(np.column_stack((r[v] * np.cos(ang[v]) + 0.103, r[v] * np.sin(ang[v]))))
    gt = np.array([x['gt'] for x in R]); w = np.array([x['wheel'] for x in R]); g0 = gt[0]
    c, s = math.cos(-g0[2]), math.sin(-g0[2])
    true = np.array([[c * (g[0] - g0[0]) - s * (g[1] - g0[1]), s * (g[0] - g0[0]) + c * (g[1] - g0[1])] for g in gt])
    def wheel_delta(i):   # body-frame motion between scan i-1 and i from wheel odom
        return mat2(np.linalg.inv(se2(w[i - 1])) @ se2(w[i]))
    out = []
    for name, use_w, p2l in (('V0 F4 (as is)', False, False), ('V1 F4 + wheel init', True, False), ('V2 point-to-line', False, True), ('V3 point-to-line + wheel init', True, True)):
        kf = P[0]; knd = normals(kf) if p2l else None; Tok = np.eye(3); Tkc = np.eye(3); est = [np.zeros(2)]
        for i in range(1, len(P)):
            init = (Tkc @ se2(wheel_delta(i))) if use_w else Tkc
            Tkc = icp_p2l(P[i], kf, knd, init) if p2l else F4.icp_2d(obj, P[i], kf, init)
            Tc = Tok @ Tkc; est.append(mat2(Tc)[:2])
            if math.hypot(Tkc[0, 2], Tkc[1, 2]) >= 0.12 or abs(math.atan2(Tkc[1, 0], Tkc[0, 0])) >= 0.12:
                kf = P[i]; knd = normals(kf) if p2l else None; Tok = Tc.copy(); Tkc = np.eye(3)
        est = np.array(est); e = np.hypot(*(est - true).T)
        out.append(f'   {name:30s} travel {np.hypot(*est[-1]):.3f} m (GT {np.hypot(*true[-1]):.3f}) | final err {e[-1]:.3f} m, max {e.max():.3f} m')
    print(f'== {run}'); print('\n'.join(out))
