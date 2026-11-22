"""Analyse Phase 3 runs recorded with timeline_recorder.py (test-only). usage: analyze_timeline.py RUN_DIR [RUN_DIR ...] > summary.json

Per run: verdicts, Nav2 bring-up attempts, latency/age maxima (timer jitter, graph query, Fleet HTTP, scan stamp age, TF age, AMCL covariance).
Per FAILED scenario: the FIRST timeline sample inside the scenario window where a blocking condition holds, in this order of evidence:
  server gate_boot_reconciliation flag, server safety_context first failing check, localization not CONFIRMED, Gate not NORMAL,
  watchdog not authorized, Nav2 goal ABORTED(6)/CANCELED(5). The blocking point is reported as found; nothing is inferred beyond it.
"""
import json, sys
from pathlib import Path


def load(run):
    run = Path(run)
    res = json.loads((run / 'result.json').read_text())
    tl = [json.loads(l) for l in (run / 'timeline.jsonl').read_text().splitlines() if l.strip()] if (run / 'timeline.jsonl').exists() else []
    return res, tl


def blockers(s):
    f = s.get('fleet') or {}; out = []
    if f.get('gate_boot_reconciliation'): out.append('SERVER:GATE_BOOT_RECONCILIATION')
    if f.get('mode') == 'NAVIGATION' and f.get('safety_context_reason'): out.append('SERVER_SAFETY_CONTEXT:' + f['safety_context_reason'])
    loc = (f.get('loc') or {}).get('status')
    if f.get('mode') == 'NAVIGATION' and loc not in (None, 'CONFIRMED', 'IDLE', 'UNAVAILABLE'): out.append('LOCALIZATION:' + str(loc))
    g = (f.get('gate') or {}).get('status')
    if g not in (None, 'NORMAL', 'STALL_PENDING'): out.append('GATE:' + str(g))
    if f.get('watchdog') not in (None, 'MOTION_AUTHORIZED'): out.append('WATCHDOG:' + str(f.get('watchdog')))
    for uuid, st in ((s.get('nav_status') or {}).get('goals') or []):
        if st in (5, 6): out.append(f'NAV2_GOAL_{"CANCELED" if st == 5 else "ABORTED"}:{uuid}')
    return out


def num(v): return v if isinstance(v, (int, float)) else None


def maxof(tl, fn):
    vals = [x for x in (fn(s) for s in tl) if isinstance(x, (int, float))]
    return round(max(vals), 3) if vals else None


def analyse(run):
    res, tl = load(run)
    marks = {}
    for name, kind, t in res.get('scenario_marks') or []:
        marks.setdefault(name, {})[kind] = t
    fails = []
    for r in res['results']:
        if r['result'] != 'FAIL': continue
        m = marks.get(r['scenario'], {})
        win = [s for s in tl if m.get('start', 0) <= s['t'] <= m.get('end', 1e18)]
        first = next(((s['t'], blockers(s)) for s in win if blockers(s)), None)
        before = [s for s in tl if s['t'] < m.get('start', 0)]
        carried = blockers(before[-1]) if before else []
        fails.append({'scenario': r['scenario'], 'window_s': round(m.get('end', 0) - m.get('start', 0), 1),
                      'blocking_at_start': carried, 'first_blocker': None if not first else {'t_rel_s': round(first[0] - m['start'], 2), 'blockers': first[1]},
                      'error': str(r.get('error'))[:160] if r.get('error') else None,
                      'detail': {k: r.get(k) for k in ('final', 'mission', 'localization', 'second_goal', 'mission_after_restart') if k in r}})
    return {'run': str(run), 'counts': res.get('counts'), 'nav2_bringup_attempts': res.get('nav2_bringup_attempts'),
            'gate_unlatch_events': [(e.get('before'), e.get('unlatch_result'), (e.get('last_fault_reasons') or [None])[0]) for e in res.get('gate_unlatch_events') or []],
            'samples': len(tl),
            'max': {'timer_jitter_ms': maxof(tl, lambda s: s.get('timer_jitter_ms')),
                    'graph_query_ms': maxof(tl, lambda s: max([v[1] for v in (s.get('graph_pubs_ms') or {}).values() if isinstance(v[1], (int, float))] or [None]) if s.get('graph_pubs_ms') else None),
                    'fleet_http_ms': maxof(tl, lambda s: (s.get('fleet') or {}).get('http_ms')),
                    'scan_stamp_age_s': maxof(tl, lambda s: (s.get('scan') or {}).get('stamp_age')),
                    'tf_map_base_age_s': maxof(tl, lambda s: num((s.get('tf_age') or {}).get('map>base'))),
                    'amcl_cov_xx': maxof(tl, lambda s: ((s.get('amcl') or {}).get('cov') or [None])[0])},
            'tf_lookup_errors': sum(1 for s in tl if isinstance((s.get('tf_age') or {}).get('map>base'), str)),
            'failures': fails}


if __name__ == '__main__':
    print(json.dumps([analyse(r) for r in sys.argv[1:]], indent=1, default=str))
