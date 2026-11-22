"""SIMULATION ONLY (2026-11-21): one-time-axis table from timeline_recorder.py output for one scenario of a run_gazebo_web.py run.

usage: python3 timeline_analyse.py <run dir> <scenario name> [--bin 0.5] [--from S] [--to S]
Columns per bin (seconds from scenario start): Fleet mode / readiness / Arbiter source+blocker, Watchdog, Gate state, Gate in (cmd_vx_in)
and out (/cmd_vel_locked), bridge input (/sim/drive_cmd), Gazebo truth speed and travelled distance, Gate wheel/lidar speed, front scan
range, /scan rate and largest gap, sim-time rate (d ROS time / d monotonic), load average, recorder timer lateness, Nav2 non-active servers.
Also prints the first bin where a command reaches the bridge but the truth does not move, and the first Gate state change.
"""
import json, math, sys
from pathlib import Path

run = Path(sys.argv[1]); scen = sys.argv[2]
arg = lambda k, d: float(sys.argv[sys.argv.index(k) + 1]) if k in sys.argv else d
BIN = arg('--bin', .5)
res = json.loads((run / 'result.json').read_text())
rows = [json.loads(l) for l in (run / 'timeline.jsonl').read_text().splitlines() if l.strip().startswith('{')]
st = next(r for r in rows if r['k'] == 'start'); off = st['wall'] - st['m']          # wall = mono + off
marks = {(n, e): t - off for n, e, t in res['marks']}
t0 = marks[(scen, 'start')]; t1 = marks.get((scen, 'end'), rows[-1]['m'])
lo, hi = t0 + arg('--from', 0.), min(t1, t0 + arg('--to', 1e9))
sel = [r for r in rows if lo - 1 <= r['m'] <= hi]
last = {}
print(f'# {run.name} {scen}: {round(t1 - t0, 1)} s; bins {BIN} s from scenario start')
hdr = 't_s mode ready arb wd gate gin gout drive gz_v gz_d wheel lidar front scan/s gapmax simrate load late nav2_down'.split()
print(' | '.join(hdr))
dist, prev_p, first_stuck, first_gate = 0., None, None, None
b = lo
while b < hi:
    e = b + BIN; win = [r for r in sel if b <= r['m'] < e]
    for r in win:
        last[r['k']] = r
        if r['k'] == 'truth':
            p = r['p']
            if prev_p: dist += math.hypot(p[0] - prev_p[0], p[1] - prev_p[1])
            prev_p = p
    k = lambda kind: [r for r in win if r['k'] == kind]
    h = last.get('http', {}); g = last.get('gate', {}); tr = last.get('truth', {})
    gout = max((abs(r['vx']) for r in k('locked')), default=None); drv = max((abs(r['vx']) for r in k('drive')), default=None)
    sc = [r['m'] for r in k('scan')]; gap = max((y - x for x, y in zip(sc, sc[1:])), default=None)
    ck = [r for r in sel if r['k'] == 'clock' and b - 1.5 <= r['m'] < e]
    rate = (ck[-1]['r'] - ck[0]['r']) / (ck[-1]['m'] - ck[0]['m']) if len(ck) > 1 and ck[-1]['m'] > ck[0]['m'] and ck[0]['r'] is not None else None
    nav = last.get('nav2', {}).get('s') or {}
    down = ','.join(s[:4] for s, v in nav.items() if v not in ('active', None) and s not in ('map_server', 'amcl')) or '-'
    gzv = tr.get('p', [None] * 5)[3] if tr else None
    arb = h.get('arb') or {}
    row = [f'{b - t0:6.1f}', str(h.get('mode')), str(h.get('ready')), f"{arb.get('source')}/{arb.get('blocker')}", str(h.get('wd')),
           str(g.get('state')), str(g.get('cmd_vx_in')), None if gout is None else round(gout, 3), None if drv is None else round(drv, 3),
           None if gzv is None else round(gzv, 3), round(dist, 3), g.get('wheel_vx'), g.get('lidar_speed'), g.get('min_front_scan_m'),
           len(sc) / BIN, None if gap is None else round(gap, 3), None if rate is None else round(rate, 2),
           (last.get('load') or {}).get('loadavg', [None])[0], (last.get('load') or {}).get('timer_late_max_s'), down]
    print(' | '.join(str(x) for x in row))
    if first_stuck is None and drv and drv > .05 and gzv is not None and abs(gzv) < .01: first_stuck = round(b - t0, 1)
    if first_gate is None and g.get('state') not in (None, 'NORMAL'): first_gate = (round(b - t0, 1), g.get('state'), g.get('fault_reasons'))
    b = e
print('# first bin with bridge command > 0.05 and truth |v| < 0.01:', first_stuck)
print('# first non-NORMAL Gate state:', first_gate)
