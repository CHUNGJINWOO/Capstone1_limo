#!/usr/bin/env python3
"""How faithful is 'static SLAM map + inflation' (no obstacle layer) vs the REAL /global_costmap/costmap? Uses the 4 paired snapshots
(slam_N / costmap_N) recorded by frontier_tracer in diff_trace3_attempt3. Writes reconstructed dumps and prints agreement."""
import math, sys, numpy as np
d = 'frontier_analysis/diff_trace3_attempt3/snapshots/'; out = 'post_analysis_20261111/stop_type/'
RR, IR, CS, res = 0.18, 0.28, 7.5, 0.05
def recon(m):
    H, W = m.shape; lethal = m >= 65; unk = m < 0; rad = int(math.ceil(IR / res)); dist = np.full((H, W), 9.0)
    for dy in range(-rad, rad + 1):
        for dx in range(-rad, rad + 1):
            dd = math.hypot(dx, dy) * res
            if dd > IR: continue
            sh = np.zeros_like(lethal); y0, y1 = max(0, dy), H + min(0, dy); x0, x1 = max(0, dx), W + min(0, dx)
            sh[y0:y1, x0:x1] = lethal[y0 - dy:y1 - dy, x0 - dx:x1 - dx]; dist = np.where(sh, np.minimum(dist, dd), dist)
    cost = np.zeros((H, W), dtype=np.int16); inf = dist <= IR
    cost[inf] = np.where(dist[inf] <= RR, 253, (252 * np.exp(-CS * (dist[inf] - RR))).astype(int)); cost[lethal] = 254
    occ = np.where(cost == 0, 0, np.where(cost == 253, 99, np.where(cost == 254, 100, 1 + (97 * (cost - 1)) // 251))); occ[unk] = -1
    return occ
for n, t, rob in ((1, 50, (-1.19, 1.01)), (2, 51, None), (3, 52, None), (4, 53, (-1.23, 0.97))):
    s = np.load(f'{d}slam_{n}_{t}.npy').astype(np.int16); c = np.load(f'{d}costmap_{n}_{t}.npy').astype(np.int16); r = recon(s)
    both = (c >= 0) & (r >= 0)
    z_real, z_rec = (c == 0), (r == 0)
    print(f'snap{n}: unknown real={int((c<0).sum())} recon={int((r<0).sum())} | cost-0 cells real={int(z_real.sum())} recon={int(z_rec.sum())} | '
          f'cost-0 agreement (IoU) {float((z_real&z_rec).sum())/max(1,float((z_real|z_rec).sum())):.3f} | cells where real>0 but recon==0: {int((both&(c>0)&(r==0)).sum())}, real==0 but recon>0: {int((both&(c==0)&(r>0)).sum())} | '
          f'real>=99: {int((c>=99).sum())} recon>=99: {int((r>=99).sum())}')
