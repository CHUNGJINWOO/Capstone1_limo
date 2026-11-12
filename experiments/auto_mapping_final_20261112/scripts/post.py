#!/usr/bin/env python3
"""Post-process one run: map image, driven path overlay, coverage numbers, unexplored interior regions.
usage: post.py RUN_DIR      (needs the monitor outputs final_map.npy/json + timeseries.csv and the saver outputs)
Coverage definition (same as analyze_map_quality.py): interior = inner faces of the 4 outer walls of lounge_evaluation_world.sdf
(x in [-5.9, 5.9], y in [-4.4, 4.4] = 103.84 m2); 'known' = SLAM cell that is free (<25) or occupied (>=65); coverage = known interior area / interior area.
Furniture is not subtracted, so cells under furniture count as unexplored."""
import csv, json, math, os, subprocess, sys
import numpy as np
from PIL import Image, ImageDraw
run = sys.argv[1]
here = os.path.dirname(os.path.abspath(__file__))
sg = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(here))), 'experiments', 'safety_gate_verification_20261110', 'scripts')
sg = os.path.normpath(os.path.join(here, '..', '..', 'safety_gate_verification_20261110', 'scripts'))
subprocess.run([sys.executable, os.path.join(sg, 'analyze_map_quality.py'), run], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
a = np.load(os.path.join(run, 'final_map.npy')); meta = json.load(open(os.path.join(run, 'final_map.json')))
res = meta['resolution']; ox, oy = meta['origin']; al = meta['align_world_from_map']
H, W = a.shape
occ = a >= 65; free = (a >= 0) & (a < 25); known = occ | free
# --- saved map file cross-check (nav2 map_saver trinary pgm: 254 free, 0 occupied, 205 unknown)
pgm = os.path.join(run, 'limo_sim_map.pgm'); saved = None
if os.path.exists(pgm):
    p = np.array(Image.open(pgm)); saved = {'size': list(p.shape[::-1]), 'known_cells': int(((p == 254) | (p == 0)).sum()),
                                            'monitor_final_map_known_cells': int(known.sum())}
    Image.fromarray(p).resize((p.shape[1] * 4, p.shape[0] * 4), Image.NEAREST).save(os.path.join(run, 'map.png'))
# --- path overlay (ground-truth path from the monitor timeseries, world -> map frame)
rows = list(csv.DictReader(open(os.path.join(run, 'timeseries.csv'))))
c, s = math.cos(al['theta']), math.sin(al['theta'])
bagpath = os.path.join(run, 'path_map_frame.csv')
if os.path.exists(bagpath):   # path in the SLAM map frame, from the rosbag (map->odom + odom->base_footprint)
    pts = [(float(r['x']), float(r['y'])) for r in csv.DictReader(open(bagpath))]
    def px(x, y): return (x - ox) / res, (H - 1) - (y - oy) / res
else:                          # fallback: coarse ground-truth path from the monitor timeseries (world -> map)
    pts = [(float(r['gt_x']), float(r['gt_y'])) for r in rows if r['gt_x'] not in ('', 'nan')]
    def px(x, y):
        dx, dy = x - al['tx'], y - al['ty']; mx_, my_ = c * dx + s * dy, -s * dx + c * dy
        return (mx_ - ox) / res, (H - 1) - (my_ - oy) / res
S = 6
img = np.full((H, W, 3), 128, np.uint8); img[free] = (255, 255, 255); img[occ] = (0, 0, 0)
im = Image.fromarray(img).resize((W * S, H * S), Image.NEAREST); d = ImageDraw.Draw(im)
line = [(px(x, y)[0] * S + S / 2, px(x, y)[1] * S + S / 2) for x, y in pts]
gtp = os.path.join(run, 'gt_world_path.csv'); gt_line = []
if line:
    for (x, y), col in ((line[0], (0, 170, 0)), (line[-1], (30, 60, 220))): d.ellipse((x - 7, y - 7, x + 7, y + 7), fill=col)
path_m = sum(h for h in (math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1)) if h < 0.5)
# --- coverage and unexplored interior regions (world frame)
jj, ii = np.mgrid[0:H, 0:W]
_defer = True
mx = ox + (ii + 0.5) * res; my = oy + (jj + 0.5) * res
wx = c * mx - s * my + al['tx']; wy = s * mx + c * my + al['ty']
X0, X1, Y0, Y1 = -5.9, 5.9, -4.4, 4.4
def wall_residuals(wx_, wy_, mask):
    """signed distance of occupied cells near the 4 outer walls to their wall line (cells within 0.45 m of a wall)."""
    px_, py_ = wx_[mask], wy_[mask]
    d = np.stack([px_ - X1, px_ - X0, py_ - Y1, py_ - Y0])          # east, west, north, south
    k = np.argmin(np.abs(d), axis=0); r = d[k, np.arange(d.shape[1])]
    keep = (np.abs(r) < 0.45) & (px_ > X0 - 0.5) & (px_ < X1 + 0.5) & (py_ > Y0 - 0.5) & (py_ < Y1 + 0.5)
    return r[keep], k[keep], px_[keep], py_[keep]
def stats(r): return {'mean_abs_m': round(float(np.mean(np.abs(r))), 3), 'p95_abs_m': round(float(np.percentile(np.abs(r), 95)), 3), 'n': int(len(r))}
r0, k0, _, _ = wall_residuals(wx, wy, occ)
best = None
for dth in np.arange(-0.12, 0.12001, 0.0025):
    cd, sd = math.cos(dth), math.sin(dth)
    ux, uy = cd * wx - sd * wy, sd * wx + cd * wy
    r, k, _, _ = wall_residuals(ux, uy, occ)
    # translation is separable: x walls (k 0,1) fix dx, y walls (k 2,3) fix dy; use the median signed residual
    dx = -np.median(r[k < 2]) if (k < 2).any() else 0.0; dy = -np.median(r[k >= 2]) if (k >= 2).any() else 0.0
    r2, k2, _, _ = wall_residuals(ux + dx, uy + dy, occ)
    sc = float(np.mean(np.abs(r2)))
    if best is None or sc < best[0]: best = (sc, dth, dx, dy, r2)
_, dth, dx, dy, r_ref = best
cd, sd = math.cos(dth), math.sin(dth)
wx_ref, wy_ref = cd * wx - sd * wy + dx, sd * wx + cd * wy + dy
refine = {'delta_theta_deg': round(math.degrees(dth), 2), 'dx_m': round(float(dx), 3), 'dy_m': round(float(dy), 3),
          'outer_wall_residual_with_start_pose_alignment': stats(r0), 'outer_wall_residual_after_wall_fit_alignment': stats(r_ref)}
inside = (wx > X0) & (wx < X1) & (wy > Y0) & (wy < Y1)
inside_ref = (wx_ref > X0) & (wx_ref < X1) & (wy_ref > Y0) & (wy_ref < Y1)
cell = res * res
unk = inside_ref & ~known     # unexplored interior, wall-fit alignment (see alignment_refinement)
lab = np.zeros(a.shape, int); n = 0; comps = []
for j0, i0 in zip(*np.nonzero(unk)):
    if lab[j0, i0]: continue
    n += 1; stack = [(j0, i0)]; lab[j0, i0] = n; cells = []
    while stack:
        j, i = stack.pop(); cells.append((j, i))
        for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            jn, in_ = j + dj, i + di
            if 0 <= jn < H and 0 <= in_ < W and unk[jn, in_] and not lab[jn, in_]:
                lab[jn, in_] = n; stack.append((jn, in_))
    if len(cells) * cell >= 0.3:
        xs = [wx_ref[j, i] for j, i in cells]; ys = [wy_ref[j, i] for j, i in cells]
        comps.append({'area_m2': round(len(cells) * cell, 2), 'x_range': [round(float(min(xs)), 2), round(float(max(xs)), 2)], 'y_range': [round(float(min(ys)), 2), round(float(max(ys)), 2)]})
comps.sort(key=lambda r: -r['area_m2'])
q = json.load(open(os.path.join(run, 'map_quality.json'))) if os.path.exists(os.path.join(run, 'map_quality.json')) else {}
summ = json.load(open(os.path.join(run, 'summary.json')))
out = {'definition': __doc__.split('\n')[2:5], 'interior_area_m2': round((X1 - X0) * (Y1 - Y0), 2),
       'known_in_interior_m2': round(float((known & inside).sum() * cell), 2), 'free_in_interior_m2': round(float((free & inside).sum() * cell), 2),
       'occupied_in_interior_m2': round(float((occ & inside).sum() * cell), 2),
       'coverage_fraction': round(float((known & inside).sum() * cell / ((X1 - X0) * (Y1 - Y0))), 3),
       'known_total_m2': round(float(known.sum() * cell), 2),
       'alignment_refinement': refine, 'coverage_fraction_with_wall_fit_alignment': round(float((known & inside_ref).sum() * cell / ((X1 - X0) * (Y1 - Y0))), 3), 'unexplored_interior_regions_ge_0.3m2': comps[:8],
       'path_length_m_from_gt_timeseries': round(path_m, 2), 'monitor_gt_path_m': summ.get('gt_path_m'), 'stop_reason': summ.get('stop_reason'),
       'sim_s': summ.get('sim_s'), 'wall_s': summ.get('wall_s'), 'saved_map_file': saved, 'wall_coverage_from_map_quality': [(w['wall'], w['along_coverage']) for w in q.get('walls', [])]}
json.dump(out, open(os.path.join(run, 'coverage.json'), 'w'), indent=1)
print(json.dumps({k: out[k] for k in ('interior_area_m2', 'known_in_interior_m2', 'coverage_fraction', 'coverage_fraction_with_wall_fit_alignment', 'alignment_refinement', 'path_length_m_from_gt_timeseries', 'stop_reason', 'sim_s', 'saved_map_file')}, indent=1))
print('unexplored regions:', comps[:5])
# --- overlay both paths now that the wall-fit alignment is known
def world_to_map_ref(x, y):
    cd_, sd_ = math.cos(dth), math.sin(dth)
    ux, uy = cd_ * (x - dx) + sd_ * (y - dy), -sd_ * (x - dx) + cd_ * (y - dy)      # undo the wall-fit delta
    ux, uy = ux - al['tx'], uy - al['ty']; return c * ux + s * uy, -s * ux + c * uy  # undo the start-pose alignment
if len(line) > 1: d.line(line, fill=(220, 30, 30), width=2)
gt_len = None
if os.path.exists(gtp):
    g = [(float(r['x']), float(r['y'])) for r in csv.DictReader(open(gtp))]
    gl = []
    for x, y in g:
        mx_, my_ = world_to_map_ref(x, y); gl.append(((mx_ - ox) / res * S + S / 2, ((H - 1) - (my_ - oy) / res) * S + S / 2))
    if len(gl) > 1: d.line(gl, fill=(20, 90, 230), width=3)
    gt_len = round(sum(math.hypot(g[i + 1][0] - g[i][0], g[i + 1][1] - g[i][1]) for i in range(len(g) - 1)), 2)
    for (x, y), col in ((gl[0], (0, 170, 0)), (gl[-1], (255, 140, 0))): d.ellipse((x - 7, y - 7, x + 7, y + 7), fill=col)
im.save(os.path.join(run, 'path_overlay.png'))
out['ground_truth_path_length_m_gazebo'] = gt_len
json.dump(out, open(os.path.join(run, 'coverage.json'), 'w'), indent=1)
print('ground-truth path length (Gazebo world, 20 Hz):', gt_len, 'm')
