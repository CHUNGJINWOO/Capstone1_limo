#!/usr/bin/env python3
"""Offline map-quality evaluation of one auto-mapping run against the known lounge geometry.

Reference geometry is read from capstone_ws/src/limo_capstone/worlds/lounge_evaluation_world.sdf
(outer walls, centre y=+-4.5 / x=+-6.0, thickness 0.2; glass wall y=-1.5). Furniture is NOT part of the
reference, so occupied cells away from the wall lines are reported but not scored as errors.

usage: analyze_map_quality.py RUN_DIR      (needs final_map.npy/.json written by auto_mapping_monitor.py)
Writes RUN_DIR/map_quality.json and RUN_DIR/map_overlay.png (if matplotlib is available).
"""
import json, math, os, sys
import numpy as np

run = sys.argv[1]
a = np.load(os.path.join(run, 'final_map.npy'))
meta = json.load(open(os.path.join(run, 'final_map.json')))
res = meta['resolution']; ox, oy = meta['origin']
al = meta.get('align_world_from_map')
if al is None:
    sys.exit('no alignment recorded (robot pose pair missing)')
H, W = a.shape
jj, ii = np.mgrid[0:H, 0:W]
mx = ox + (ii + 0.5) * res; my = oy + (jj + 0.5) * res
c, s = math.cos(al['theta']), math.sin(al['theta'])
wx = c * mx - s * my + al['tx']; wy = s * mx + c * my + al['ty']
occ = a >= 65; free = (a >= 0) & (a < 25); known = occ | free

X0, X1, Y0, Y1 = -5.9, 5.9, -4.4, 4.4            # inner faces of the outer walls
inside = (wx > X0) & (wx < X1) & (wy > Y0) & (wy < Y1)
interior_area = (X1 - X0) * (Y1 - Y0)
cell = res * res
out = dict(
    run=os.path.basename(run.rstrip('/')),
    resolution_m=res, interior_area_m2=round(interior_area, 2),
    known_in_interior_m2=round(float((known & inside).sum() * cell), 2),
    free_in_interior_m2=round(float((free & inside).sum() * cell), 2),
    occupied_in_interior_m2=round(float((occ & inside).sum() * cell), 2),
    interior_known_fraction=round(float((known & inside).sum() * cell / interior_area), 3),
    known_outside_interior_m2=round(float((known & ~inside).sum() * cell), 2),
    note_outside='known cells outside the outer walls = leaked rays (e.g. through glass) or wall thickness',
)

# ---- wall-line accuracy ---------------------------------------------------------------
def wall_stats(name, axis, pos, lo, hi, inward):
    """axis 'x': wall is the line x=pos spanning y in [lo,hi]; inward=+1/-1 is the interior side."""
    band = 0.35
    if axis == 'x':
        perp = wx - pos; along = wy
    else:
        perp = wy - pos; along = wx
    sel = occ & (np.abs(perp) < band) & (along > lo) & (along < hi)
    n = int(sel.sum())
    if n == 0:
        return dict(wall=name, occupied_cells_in_band=0, along_coverage=0.0)
    off = perp[sel] * inward          # >0 = displaced toward the interior
    bins = np.arange(lo, hi + 0.1, 0.1)
    hit = np.histogram(along[sel], bins=bins)[0] > 0
    return dict(wall=name, occupied_cells_in_band=n, offset_mean_m=round(float(off.mean()), 3),
                offset_std_m=round(float(off.std()), 3), offset_p95_abs_m=round(float(np.percentile(np.abs(off), 95)), 3),
                along_coverage=round(float(hit.mean()), 3))

walls = [
    wall_stats('north (y=4.4)', 'y', Y1, X0, X1, -1),
    wall_stats('south (y=-4.4)', 'y', Y0, X0, X1, +1),
    wall_stats('west (x=-5.9)', 'x', X0, Y0, Y1, +1),
    wall_stats('east (x=5.9)', 'x', X1, Y0, Y1, -1),
]
# glass wall centre line (y=-1.5, x in [-1.7, 5.3]); either face (+-0.1) counts
g = wall_stats('glass wall (y=-1.5)', 'y', -1.5, -1.7, 5.3, 1)
walls.append(g)
out['walls'] = walls
cov = [w['along_coverage'] for w in walls[:4]]
out['outer_wall_mean_coverage'] = round(float(np.mean(cov)), 3)
valid = [w for w in walls[:4] if w.get('occupied_cells_in_band')]
out['outer_wall_mean_abs_offset_m'] = round(float(np.mean([abs(w['offset_mean_m']) for w in valid])), 3) if valid else None
out['outer_wall_mean_std_m'] = round(float(np.mean([w['offset_std_m'] for w in valid])), 3) if valid else None

# occupied cells far from every reference wall line (furniture or errors)
near = np.zeros_like(occ)
for ax, pos, lo, hi in (('y', Y1, X0, X1), ('y', Y0, X0, X1), ('x', X0, Y0, Y1), ('x', X1, Y0, Y1), ('y', -1.5, -6.0, 5.3)):
    perp = (wy - pos) if ax == 'y' else (wx - pos); along = wx if ax == 'y' else wy
    near |= (np.abs(perp) < 0.35) & (along > lo - 0.3) & (along < hi + 0.3)
out['occupied_away_from_walls_m2'] = round(float((occ & inside & ~near).sum() * cell), 2)

# unknown regions inside the room (candidates for missed coverage)
unk = (~known) & inside
lab = np.zeros(unk.shape, np.int32); n = 0
ys, xs = np.nonzero(unk)
for y0, x0 in zip(ys, xs):
    if lab[y0, x0]:
        continue
    n += 1; stack = [(y0, x0)]; lab[y0, x0] = n
    while stack:
        y, x = stack.pop()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            yy, xx = y + dy, x + dx
            if 0 <= yy < H and 0 <= xx < W and unk[yy, xx] and not lab[yy, xx]:
                lab[yy, xx] = n; stack.append((yy, xx))
clusters = []
for i in range(1, n + 1):
    m = lab == i; area = float(m.sum() * cell)
    if area >= 0.25:
        clusters.append(dict(area_m2=round(area, 2), centroid_world=[round(float(wx[m].mean()), 2), round(float(wy[m].mean()), 2)]))
clusters.sort(key=lambda d: -d['area_m2'])
out['unknown_inside_clusters_ge_0p25m2'] = clusters[:8]
out['unknown_inside_total_m2'] = round(float(unk.sum() * cell), 2)

# thin-wall duplication: occupied thickness across the wall (should be ~0.2 m wall -> 0.2-0.3 m)
json.dump(out, open(os.path.join(run, 'map_quality.json'), 'w'), indent=1)
print(json.dumps(out, indent=1))

try:
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 7))
    img = np.full(a.shape, 0.8); img[free] = 1.0; img[occ] = 0.0
    ext = [ox, ox + W * res, oy, oy + H * res]
    # draw in world coords by transforming outline of the reference into map frame
    ax.imshow(img, origin='lower', extent=ext, cmap='gray', vmin=0, vmax=1)
    def to_map(xw, yw):
        dx, dy = xw - al['tx'], yw - al['ty']
        return c * dx + s * dy, -s * dx + c * dy
    for (x0, y0, x1, y1) in ((X0, Y0, X1, Y0), (X1, Y0, X1, Y1), (X1, Y1, X0, Y1), (X0, Y1, X0, Y0), (-6.0, -1.5, -2.5, -1.5), (-1.7, -1.5, 5.3, -1.5)):
        p0 = to_map(x0, y0); p1 = to_map(x1, y1)
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], 'r-', lw=1)
    ax.set_title(f"{out['run']}: known {out['known_in_interior_m2']} m2 of {out['interior_area_m2']} (red = reference walls)")
    fig.savefig(os.path.join(run, 'map_overlay.png'), dpi=110, bbox_inches='tight')
except Exception as e:
    print('overlay skipped:', e)
