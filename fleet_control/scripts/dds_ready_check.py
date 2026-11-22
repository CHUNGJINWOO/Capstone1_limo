#!/usr/bin/env python3
"""Post-start readiness gate for the Robot Agent, judged from what the Fleet server actually shows (not from the process being alive).

READY only if, in EVERY one of --samples consecutive samples (--interval apart): the robot is online with fresh state (state_age_s < --max-state-age),
lidar status OK with age < --max-sensor-age, pose OK, map STATIC or UPDATING, and the Agent's sequence counter advanced since the previous sample.
Anything else -> NOT_READY with the reasons; exit code 5. A NOT_READY Agent must not be treated as ready, must not be handed a Lease for any
operation and nothing may be started "because it is up". (The server and Agent already fail closed on NO_DATA; this script is the explicit
start-up gate and the written verdict.)  Read-only: one GET per sample. Loopback URL only; the token is read from a file and never printed.
usage: dds_ready_check.py --url http://127.0.0.1:18190 --robot pcu --token-file FILE [--samples 3] [--interval 1.0] [--out FILE]
Exit: 0 READY, 5 NOT_READY, 2 usage.
"""
import argparse, json, sys, time, urllib.error, urllib.request


def fetch(base, robot, token):
    req = urllib.request.Request(f'{base}/api/v1/robots/{robot}', headers={'Authorization': 'Bearer ' + token})
    with urllib.request.urlopen(req, timeout=5) as r:
        d = json.loads(r.read())
    return d.get('robot', d)


def judge(r, prev_seq, a):
    why = []
    s = (r or {}).get('state') or {}
    if not r.get('online'):
        why.append('robot not online')
    age = r.get('state_age_s')
    if age is None or age >= a.max_state_age:
        why.append(f'state stale ({age})')
    lid = s.get('lidar') or {}
    if lid.get('status') != 'OK' or lid.get('age_s') is None or lid['age_s'] >= a.max_sensor_age:
        why.append(f"lidar {lid.get('status')} age={lid.get('age_s')}")
    if (s.get('pose') or {}).get('status') != 'OK':
        why.append(f"pose {(s.get('pose') or {}).get('status')}")
    if (s.get('map') or {}).get('status') not in ('STATIC', 'UPDATING'):
        why.append(f"map {(s.get('map') or {}).get('status')}")
    seq = s.get('agent_seq')
    if seq is None:
        why.append('no agent_seq')
    elif prev_seq is not None and seq <= prev_seq:
        why.append(f'agent_seq not advancing ({prev_seq}->{seq})')
    return why, seq


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', required=True)
    ap.add_argument('--robot', required=True)
    ap.add_argument('--token-file', required=True)
    ap.add_argument('--samples', type=int, default=3)
    ap.add_argument('--interval', type=float, default=1.0)
    ap.add_argument('--max-state-age', type=float, default=3.0)
    ap.add_argument('--max-sensor-age', type=float, default=1.5)
    ap.add_argument('--out')
    a = ap.parse_args(argv)
    if not a.url.startswith(('http://127.0.0.1:', 'http://localhost:')):
        print('refusing non-local URL', file=sys.stderr)
        return 2
    token = open(a.token_file).read().strip()
    prev, bad, log = None, [], []
    for i in range(a.samples):
        try:
            why, prev = judge(fetch(a.url.rstrip('/'), a.robot, token), prev, a)
        except (urllib.error.URLError, OSError, ValueError) as exc:
            why, prev = [f'server unreachable ({type(exc).__name__})'], None
        if i == 0:
            why = [w for w in why if not w.startswith('agent_seq')]
        log.append(why)
        bad += why
        if i + 1 < a.samples:
            time.sleep(a.interval)
    res = {'verdict': 'READY' if not bad else 'NOT_READY', 'samples': a.samples, 'reasons_per_sample': log, 'time': time.time()}
    print(json.dumps(res))
    if a.out:
        json.dump(res, open(a.out, 'w'))
    return 0 if not bad else 5


if __name__ == '__main__':
    sys.exit(main())
