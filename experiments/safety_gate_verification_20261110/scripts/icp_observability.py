#!/usr/bin/env python3
"""Offline analysis of recorded probe scans (no simulation). Uses the REAL F4 code (icp_2d / scan_to_points imported from the repo source).
 1. replay of F4 (same keyframe logic) on the recorded scans vs recorded live /odometry/lidar_odom and GT
 2. geometry: point normals (local PCA), point-to-line Hessian at the true relative pose -> eigenvalues / weakest direction
 3. sticky-minimum test: F4's point-to-point ICP started at the TRUE displacement - does it stay or slide back to ~0?
 4. cost profile: trimmed point-to-point cost vs displacement along the heading for a scan pair with true displacement D.
usage: icp_observability.py RUN_DIR [pair_dist=0.25]"""
import os, sys, json, math, types
import numpy as np
sys.path.insert(0, os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'capstone_ws', 'src', 'limo_capstone')))
import rclpy  # noqa  (module import only)
from limo_capstone.lidar_scan_matcher_odom_node import LidarScanMatcherOdomNode as F4
run = sys.argv[1]; pair_d = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
R = json.load(open(f'{run}/probe.json'))['rows']; Z = np.load(f'{run}/probe_scans.npz')
ranges = Z['ranges']; a0 = float(Z['angle_min']); da = float(Z['angle_inc']); N = len(ranges)
obj = types.SimpleNamespace(laser_x_offset=0.103, max_iter=18, max_dist=0.30, last_inliers=0.0)
def pts_of(r):
    ang = a0 + np.arange(len(r)) * da
    v = np.isfinite(r) & (r >= 0.15) & (r <= 9.5)
    return np.column_stack((r[v] * np.cos(ang[v]) + 0.103, r[v] * np.sin(ang[v]))), ang[v]
P = [pts_of(r) for r in ranges]
gt = np.array([x['gt'] for x in R]); g0 = gt[0]
def body(i):  # GT pose of scan i expressed in the start body frame
    c, s = math.cos(-g0[2]), math.sin(-g0[2]); d = gt[i, :2] - g0[:2]
    return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], gt[i, 2] - g0[2]])
def se2(p):
    c, s = math.cos(p[2]), math.sin(p[2]); T = np.eye(3); T[:2, :2] = [[c, -s], [s, c]]; T[:2, 2] = p[:2]; return T
def mat2(T): return np.array([T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])])
# --- 1. replay of F4
Tok = np.eye(3); Tkc = np.eye(3); kf = P[0][0]; rep = [np.zeros(3)]
for i in range(1, N):
    Tkc = F4.icp_2d(obj, P[i][0], kf, Tkc); Tc = Tok @ Tkc; rep.append(mat2(Tc))
    if math.hypot(Tkc[0, 2], Tkc[1, 2]) >= 0.12 or abs(math.atan2(Tkc[1, 0], Tkc[0, 0])) >= 0.12:
        kf = P[i][0]; Tok = Tc.copy(); Tkc = np.eye(3)
rep = np.array(rep); live = np.array([x['lidar'] for x in R]); live = live - live[0]
true = np.array([body(i) for i in range(N)])
print(f'[1] n_scans={N}  GT travel {np.hypot(*true[-1,:2]):.3f} m | live F4 {np.hypot(*live[-1,:2]):.3f} m | offline replay of F4 {np.hypot(*rep[-1,:2]):.3f} m | replay-vs-live end diff {np.hypot(*(rep[-1,:2]-live[-1,:2])):.3f} m')
ex = np.hypot(*(true[:, :2] - rep[:, :2]).T); onset = next((i for i, e in enumerate(ex) if e > 0.05), None)
print(f'    replay error vs GT: first >5 cm at scan {onset} (GT travel then {np.hypot(*true[onset,:2]) if onset is not None else float("nan"):.3f} m), max {ex.max():.3f} m')
# --- 2. geometry / Hessian at start scan
def normals(pts, k=5):
    n = np.zeros_like(pts)
    for i in range(len(pts)):
        lo, hi = max(0, i - k), min(len(pts), i + k + 1); q = pts[lo:hi] - pts[lo:hi].mean(0)
        w, v = np.linalg.eigh(q.T @ q); n[i] = v[:, 0]
    return n
pts = P[0][0]; nrm = normals(pts)
J = np.column_stack((nrm[:, 0], nrm[:, 1], pts[:, 0] * nrm[:, 1] - pts[:, 1] * nrm[:, 0])); H = J.T @ J
ev, evec = np.linalg.eigh(H[:2, :2])
print(f'[2] points {len(pts)} | translation Hessian eig {ev[0]:.1f} (dir {evec[:,0].round(2)}) / {ev[1]:.1f} (dir {evec[:,1].round(2)}), cond {ev[1]/max(ev[0],1e-9):.1f}')
print(f'    Sum n_x^2 (heading axis) {H[0,0]:.1f} vs Sum n_y^2 (lateral) {H[1,1]:.1f}; points with |n_x|>0.7: {(abs(nrm[:,0])>0.7).sum()} of {len(pts)} ({(abs(nrm[:,0])>0.7).mean()*100:.0f}%)')
# --- pick scan pair with true displacement ~ pair_d
trav = np.hypot(true[:, 0], true[:, 1]); cand = [(i, j) for i in range(N) for j in range(i + 1, N) if abs(np.hypot(*(true[j, :2] - true[i, :2])) - pair_d) < 0.02]
if cand:
    i, j = cand[0]; rel = mat2(np.linalg.inv(se2(true[i])) @ se2(true[j])); D = rel[0]
    print(f'    pair scans ({i},{j}): true relative motion in sensor-body frame x={rel[0]:.3f} y={rel[1]:.3f} yaw={rel[2]:.3f}')
    # --- 3. sticky-minimum
    for name, init in (('identity', np.eye(3)), ('TRUE displacement', se2(rel)), ('half of true', se2(rel * np.array([0.5, 0.5, 0.5])))):
        T = F4.icp_2d(obj, P[j][0], P[i][0], init.copy()); e = mat2(T)
        print(f'[3] init={name:18s} -> ICP x={e[0]:+.3f} y={e[1]:+.3f} yaw={e[2]:+.3f}  (error vs truth dx {e[0]-rel[0]:+.3f})  inliers {obj.last_inliers:.0f}')
    # --- 4. cost profile along heading (trimmed 85% mean squared NN distance, corr. gate 0.30)
    src = P[j][0]; dst = P[i][0]
    print('[4] cost vs candidate dx (true dx=%.3f), y,yaw fixed at truth:' % rel[0])
    line = []
    for dx in np.arange(-0.1, rel[0] + 0.25, 0.05):
        T = se2(np.array([dx, rel[1], rel[2]])); tp = src @ T[:2, :2].T + T[:2, 2]
        d2 = ((tp[:, None, :] - dst[None, :, :]) ** 2).sum(2).min(1); d2 = d2[d2 < 0.09]; k = d2 <= np.percentile(d2, 85); line.append((round(float(dx), 2), round(float(d2[k].mean()) * 1e4, 2), int(len(d2))))
    print('    (dx, trimmed-mean d^2 x1e4 [m^2], inliers):', line)
else:
    print('    no scan pair with the requested displacement (robot latched earlier)')
# --- 3b. recovery of a true displacement from identity init vs iteration budget (F4 uses 18 iterations, init = previous keyframe-relative pose)
print('[3b] recovered fraction of the true forward displacement, ICP started at identity (F4 code; iterations 18 [F4] / 100 / 1000):')
for dd in (0.02, 0.05, 0.10, 0.20):
    pr = [(i, j) for i in range(N) for j in range(i + 1, N) if abs(np.hypot(*(true[j, :2] - true[i, :2])) - dd) < 0.006]
    if not pr: continue
    i, j = pr[0]; rel = mat2(np.linalg.inv(se2(true[i])) @ se2(true[j])); res = []
    for it in (18, 100, 1000):
        obj.max_iter = it; T = F4.icp_2d(obj, P[j][0], P[i][0], np.eye(3)); res.append(round(mat2(T)[0] / rel[0], 2) if abs(rel[0]) > 1e-6 else float('nan'))
    obj.max_iter = 18
    print(f'     true dx {rel[0]:.3f} m (scans {i},{j}): recovered fraction {res[0]} / {res[1]} / {res[2]}')
