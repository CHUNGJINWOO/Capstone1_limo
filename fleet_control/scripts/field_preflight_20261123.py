#!/usr/bin/env python3
"""Read-only field preflight for one robot, from the laptop, HTTP only (G2, 2026-11-23). It changes nothing: no Lease, no command.

    field_preflight_20261123.py --server http://127.0.0.1:PORT --token-file FILE --robot ID [--real] [--json]

Reads GET /api/v1/robots/ID (the state the Agent reports every second) and GET /api/v1/dashboard and prints PASS / FAIL / WARN per item:
  A. link: Agent online, fresh, reconciled, safety contract versions, boot ids
  B. safety chain: Safety Gate NORMAL and fresh, motor path LOCKED (no /cmd_vel publisher), Motion Watchdog authorized and fresh,
     velocity arbiter ready/blocker, movement_permitted (informational), missions_enabled
  C. sensors: lidar, odom/pose, map, battery, depth/camera
  D. operation: Lease holder, operating mode, mapping/bag worker state
  E. --real: the Agent must be the real configuration (mode real, simulated false, missions disabled) - anything that could move the
     robot is a FAIL for a preflight whose goal is to connect safely.
Exit code 0 = no FAIL (WARN allowed), 1 = at least one FAIL, 2 = cannot reach the server. The token is read from a file, never printed.
"""
import argparse
import json
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def get(base, token, path):
    req = Request(base + path, headers={'Authorization': 'Bearer ' + token})
    with urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def evaluate(robot, real=False):
    """Pure function over the robot record (testable without a server). Returns a list of (level, item, detail)."""
    out = []

    def add(level, item, detail=''):
        out.append((level, item, str(detail)))

    st = robot.get('state') or {}
    agent = robot.get('agent') or {}
    gate = st.get('safety_gate') or {}
    wd = st.get('motion_watchdog') or {}
    mp = gate.get('motor_path') or {}
    add('PASS' if robot.get('online') else 'FAIL', 'A1 Agent online', f"last seen {robot.get('last_seen_age_s')} s ago")
    age = robot.get('last_seen_age_s')
    add('PASS' if isinstance(age, (int, float)) and 0 <= age <= 3 else 'FAIL', 'A2 state is fresh (<= 3 s)', age)
    add('PASS' if robot.get('reconciled') else 'FAIL', 'A3 Agent reconciled with the server')
    add('PASS' if robot.get('reconciliation_required') is not True else 'FAIL', 'A4 no reconciliation pending')
    add('PASS' if agent.get('protocol_version') == 2 and agent.get('safety_contract_version') == 1 else 'FAIL',
        'A5 protocol 2 / safety contract 1', f"{agent.get('protocol_version')}/{agent.get('safety_contract_version')}")
    add('PASS' if gate.get('status') == 'NORMAL' else 'FAIL', 'B1 Safety Gate NORMAL', gate.get('status'))
    add('PASS' if mp.get('status') == 'LOCKED' else ('WARN' if mp.get('status') == 'SIM_GATED' else 'FAIL'),
        'B2 motor path LOCKED (no /cmd_vel publisher)', json.dumps({k: mp.get(k) for k in ('status', 'cmd_vel_publishers')}))
    add('PASS' if wd.get('state') == 'MOTION_AUTHORIZED' and isinstance(wd.get('age_s'), (int, float)) and wd['age_s'] < 1.0 else 'FAIL',
        'B3 Motion Watchdog authorized and fresh', json.dumps({k: wd.get(k) for k in ('state', 'age_s', 'last_fault')}))
    arb = st.get('velocity_arbiter')
    if arb is not None:
        add('PASS' if arb.get('motor_output') is False else 'FAIL', 'B4 Arbiter has no motor output', arb.get('blocker'))
    add('PASS' if gate.get('movement_permitted') is not True else 'WARN', 'B5 movement_permitted is false (locked)', gate.get('movement_permitted'))
    for key, label in (('lidar', 'C1 LiDAR'), ('pose', 'C2 pose/odom'), ('map', 'C3 map'), ('battery', 'C4 battery'), ('depth_camera', 'C5 depth')):
        block = st.get(key) or {}
        status = block.get('status') if isinstance(block, dict) else None
        add('PASS' if status in ('OK', 'UPDATING', 'STATIC', 'READY', 'VOLTAGE_ONLY') else 'WARN', label, status)
    add('PASS', 'D1 Lease', json.dumps(robot.get('lease'))[:120] if robot.get('lease') else 'none (acquire it in the console)')
    add('PASS', 'D2 operating mode', json.dumps((robot.get('operating_mode') or {}).get('mode')))
    if real:
        add('PASS' if agent.get('mode') == 'real' and agent.get('simulated') is False else 'FAIL', 'E1 real configuration', f"{agent.get('mode')} simulated={agent.get('simulated')}")
        add('PASS' if agent.get('missions_enabled') is False else 'FAIL', 'E2 missions disabled', agent.get('missions_enabled'))
        add('PASS' if not (st.get('mapping_worker') or {}).get('enabled') else 'FAIL', 'E3 no web Mapping Worker on the real Agent')
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--server', required=True); ap.add_argument('--token-file', required=True); ap.add_argument('--robot', required=True)
    ap.add_argument('--real', action='store_true'); ap.add_argument('--json', action='store_true')
    a = ap.parse_args(argv)
    token = open(a.token_file).read().strip()
    base = a.server.rstrip('/') + '/api/v1'
    try:
        robot = get(base, token, '/robots/' + a.robot)
    except HTTPError as e:
        why = {401: 'token rejected', 403: 'token may not read this robot', 404: 'robot not registered: its Agent has never connected to this server'}.get(e.code, 'unexpected answer')
        print(f'FAIL server answered HTTP {e.code}: {why}'); return 2
    except (URLError, OSError) as e:
        print(f'FAIL cannot reach {a.server}: {e}'); return 2
    results = evaluate(robot, a.real)
    if a.json:
        print(json.dumps([dict(level=l, item=i, detail=d) for l, i, d in results], indent=1))
    else:
        for level, item, detail in results:
            print(f'{level:4} {item}' + (f' | {detail}' if detail else ''))
    fails = sum(1 for l, _, _ in results if l == 'FAIL'); warns = sum(1 for l, _, _ in results if l == 'WARN')
    print(f'preflight: {len(results) - fails - warns} PASS, {warns} WARN, {fails} FAIL at {time.strftime("%H:%M:%S")}')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
