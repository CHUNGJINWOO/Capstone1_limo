import sys, json, numpy as np
D = sys.argv[1]; path = np.load(sys.argv[2])
modes = ['raw', 'n244', 'norm', 'norm263']; maps = {}
for m in modes:
    try:
        v = np.load(f'{D}/result_{m}_map.npy'); meta = json.load(open(f'{D}/result_{m}_map_meta.json')); maps[m] = (v, meta)
    except Exception as e: print('missing', m, e)
def occ_pts(v, meta):
    ys, xs = np.nonzero(v >= 65); return np.c_[meta['ox'] + (xs + 0.5) * meta['res'], meta['oy'] + (ys + 0.5) * meta['res']]
def chamfer(A, B):
    if len(A) == 0 or len(B) == 0: return float('nan'), float('nan')
    D = np.hypot(A[:, None, 0] - B[None, :, 0], A[:, None, 1] - B[None, :, 1])      # brute force (a few hundred points); scipy is broken here
    return float(D.min(axis=1).mean()), float(D.min(axis=0).mean())
print('recorded path (odom): x %.2f..%.2f, y %.2f..%.2f' % (path[:, 1].min(), path[:, 1].max(), path[:, 2].min(), path[:, 2].max()))
for m, (v, meta) in maps.items():
    known = v != -1; ys, xs = np.nonzero(known)
    if len(xs) == 0: print(m, 'empty'); continue
    x0, x1 = meta['ox'] + xs.min() * meta['res'], meta['ox'] + (xs.max() + 1) * meta['res']; y0, y1 = meta['oy'] + ys.min() * meta['res'], meta['oy'] + (ys.max() + 1) * meta['res']
    P = occ_pts(v, meta)
    # how much of the recorded trajectory lies in known free space (the robot must have been somewhere the map knows about)
    cx = ((path[:, 1] - meta['ox']) / meta['res']).astype(int); cy = ((path[:, 2] - meta['oy']) / meta['res']).astype(int)
    inb = (cx >= 0) & (cx < v.shape[1]) & (cy >= 0) & (cy < v.shape[0]); on_known = np.zeros(len(path), bool); on_known[inb] = known[cy[inb], cx[inb]]
    print('%-8s known %5d cells (%5.1f m2) | occupied %4d | extent x %.2f..%.2f y %.2f..%.2f | trajectory points inside the known map: %3.0f%%' % (m, known.sum(), known.sum() * meta['res'] ** 2, (v >= 65).sum(), x0, x1, y0, y1, 100 * on_known.mean()))
print('\noccupied-cell agreement (mean nearest distance, cm; a->b means each occupied cell of a to the closest of b):')
for a in modes:
    for b in modes:
        if a < b and a in maps and b in maps:
            ca, cb = chamfer(occ_pts(*maps[a]), occ_pts(*maps[b])); print('  %-8s vs %-8s  a->b %5.1f cm   b->a %5.1f cm' % (a, b, ca * 100, cb * 100))
