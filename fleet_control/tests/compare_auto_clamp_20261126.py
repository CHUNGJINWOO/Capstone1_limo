#!/usr/bin/env python3
"""AUTO clamp comparison, isolated and offline (G2, 2026-11-26). Not a robot result and NOT a change of the product clamp.

Compares the product's per-axis AUTO clamp (WatchdogSpeedClamp.clamp_values) with a proportional alternative that exists ONLY in this script:
    s = min(1, fwd/|v| if v>0 else rev/|v|, ang/|w|);  out = (s*v, s*w)
over command sweeps for the AUTO ceilings (0.15 m/s, 0.50 rad/s) and for operator limits (0.06/0.20, 0.10/0.30).
Metrics: ceiling violations, curvature (w/v) change, in-place rotation, narrow-turn commands, stop/zero handling, non-finite input, sign flips.
Tracking quality (goal error, cross-track error, time) needs the closed loop: those numbers are Antigravity's Kinematic runs (NAV2_STABILIZATION_AND_PATROL_20261124.md, read-only reuse).
usage: python3 compare_auto_clamp_20261126.py   (PYTHONPATH must contain fleet_control/agent/limo_robot_agent)"""
import math, json, sys
from limo_robot_agent.speed_clamp import WatchdogSpeedClamp


def make(limit_lin=None, limit_ang=None):
    c = WatchdogSpeedClamp()
    if limit_lin is not None:
        c.applied['auto'] = {'linear_mps': limit_lin, 'angular_rps': limit_ang, 'generation': 1, 'request_id': 'cmp', 'binding': None, 'seen_t': 0}
    return c


def axis(c, v, w):
    return c.clamp_values('auto', v, w)


def proportional(c, v, w):
    if not (math.isfinite(v) and math.isfinite(w)): return 0.0, 0.0
    fwd, rev, ang = c.limits('auto')
    s = 1.0
    if v > 0: s = min(s, fwd / v)
    if v < 0: s = min(s, rev / -v)
    if w != 0: s = min(s, ang / abs(w))
    return s * v, s * w


def sweep():
    for vi in range(-10, 31):
        for wi in range(-20, 21):
            yield vi * 0.01, wi * 0.05          # v in [-0.10, 0.30], w in [-1.0, 1.0]


def evaluate(name, c):
    fwd, rev, ang = c.limits('auto'); out = {'limits': {'forward': fwd, 'reverse': rev, 'angular': ang}}
    for label, fn in (('axis', axis), ('proportional', proportional)):
        viol = flips = n = 0; worst_curv = 0.0; curv_changed = 0; ratio = []
        for v, w in sweep():
            ov, ow = fn(c, v, w); n += 1
            if ov > fwd + 1e-9 or -ov > rev + 1e-9 or abs(ow) > ang + 1e-9: viol += 1
            if (v * ov < 0) or (w * ow < 0): flips += 1
            if abs(v) > 0.02 and abs(ov) > 1e-6:
                k_in, k_out = w / v, ow / ov
                if abs(k_in) > 1e-6:
                    d = abs(k_out - k_in) / abs(k_in); worst_curv = max(worst_curv, d); curv_changed += d > 0.01; ratio.append(abs(k_out) / abs(k_in))
        out[label] = {'commands': n, 'ceiling_violations': viol, 'sign_flips': flips, 'commands_with_curvature_change_gt_1pct': curv_changed, 'worst_curvature_change_ratio': round(worst_curv, 3),
                      'max_curvature_gain': round(max(ratio), 3) if ratio else None}
        # named cases
        cases = {'in_place_rotation(0,0.8)': (0.0, 0.8), 'in_place_rotation(0,-0.3)': (0.0, -0.3), 'narrow_turn(0.10,0.80)': (0.10, 0.80), 'narrow_turn_slow(0.05,0.60)': (0.05, 0.60),
                 'straight(0.30,0)': (0.30, 0.0), 'curve(0.30,0.30)': (0.30, 0.30), 'reverse(-0.2,0.4)': (-0.2, 0.4), 'stop(0,0)': (0.0, 0.0), 'nan': (float('nan'), 0.2), 'inf': (0.1, float('inf'))}
        out[label]['cases'] = {k: [round(x, 4) for x in fn(c, *vw)] for k, vw in cases.items()}
    return out


def main():
    res = {}
    for name, (lin, ang) in {'AUTO ceilings (0.15, 0.50)': (None, None), 'operator limit 0.10/0.30': (0.10, 0.30), 'operator limit 0.06/0.20': (0.06, 0.20), 'linear-only low limit 0.06/0.50': (0.06, 0.50)}.items():
        res[name] = evaluate(name, make(lin, ang))
    print(json.dumps(res, indent=1))
    ok = all(r[k]['ceiling_violations'] == 0 and r[k]['sign_flips'] == 0 for r in res.values() for k in ('axis', 'proportional'))
    print('CEILINGS_AND_SIGNS_OK' if ok else 'VIOLATION'); sys.exit(0 if ok else 1)


main()
