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
    # F. speed limits and the direct-writer latch (read-only; G2 2026-11-23). These do not move anything; they show whether a speed limit WOULD be enforced.
    sl = st.get('speed_limits') if isinstance(st.get('speed_limits'), dict) else {}
    ev = sl.get('watchdog_evidence') if isinstance(sl.get('watchdog_evidence'), dict) else {}
    enforced = sl.get('enforced_by')
    add('PASS' if enforced in ('WATCHDOG', 'ARBITER') else ('FAIL' if real else 'WARN'), 'F1 a speed-limit enforcement point is reported (WATCHDOG on a real robot)', enforced)
    if real:
        add('PASS' if enforced == 'WATCHDOG' else 'FAIL', 'F2 real robot: enforced by the Motion Watchdog clamp (a real Agent has no Arbiter)', enforced)
        age = ev.get('evidence_age_s')
        add('PASS' if ev.get('server_verified') is True and ev.get('agent_verified') is True and isinstance(age, (int, float)) and 0 <= age <= 2.0 else 'FAIL',
            'F3 Watchdog evidence verified by the SERVER and the Agent, fresh (<= 2 s)',
            json.dumps({k: ev.get(k) for k in ('server_verified', 'agent_verified', 'evidence_age_s', 'synced', 'publisher_ok', 'fault')}))
        add('PASS' if all(ev.get(k) for k in ('watchdog_boot_id', 'agent_boot_id', 'output_gid')) else 'FAIL', 'F4 Watchdog boot id, Agent boot id and writer GID are present (GID = graph identifier, not authentication)',
            json.dumps({k: (str(ev.get(k))[:12] if ev.get(k) else None) for k in ('watchdog_boot_id', 'agent_boot_id', 'output_gid')}))
    add('PASS' if sl.get('stale') is False or not sl.get('applied') or all(v is None for v in (sl.get('applied') or {}).values()) else 'WARN', 'F5 no stale speed-limit setting (a stale setting still clamps but needs a new request)', sl.get('stale_reason'))
    add('PASS' if mp.get('foreign_writer_latched') is not True else 'FAIL', 'F6 no non-Gate writer was ever seen on /cmd_vel or the driver input (latched until Agent restart; detection only)', json.dumps(mp.get('foreign_writers_seen')))
    add('PASS' if not (mp.get('cmd_vel_publishers') or []) or mp.get('status') != 'LOCKED' else 'FAIL', 'F7 LOCKED means no writer on /cmd_vel or /limo_driver/cmd_vel', json.dumps(mp.get('cmd_vel_publishers')))
    # G: real-Agent mode selector (no Arbiter): the evidence the server needs to confirm an operating mode (read-only)
    arb = st.get('velocity_arbiter') if isinstance(st.get('velocity_arbiter'), dict) else {}
    aev = arb.get('evidence') if isinstance(arb.get('evidence'), dict) else {}
    if real:
        add('PASS' if agent.get('velocity_selector') == 'WATCHDOG_SELECTOR' and arb.get('kind') == 'WATCHDOG_SELECTOR' and agent.get('locked_velocity_arbiter') is False else 'FAIL',
            'G1 the Agent declares and reports the Watchdog mode selector (no Arbiter on a real robot)', json.dumps({k: agent.get(k) for k in ('velocity_selector', 'locked_velocity_arbiter')}))
        add('PASS' if aev.get('nav_input_ok') is True and aev.get('manual_input_ok') is True else 'FAIL', 'G2 both Watchdog input channels are exclusive (nav and manual)', json.dumps({k: aev.get(k) for k in ('nav_input_ok', 'manual_input_ok', 'watchdog_nav_publishers', 'watchdog_manual_publishers')}))
        conflict = aev.get('source_conflict_age_s')
        add('PASS' if not isinstance(conflict, (int, float)) or conflict >= 10 else 'FAIL', 'G3 no motion-source conflict was seen by the Watchdog recently', conflict)
        add('PASS' if arb.get('motor_output') is False and arb.get('selected_linear') in (0, 0.0) and arb.get('selected_turn') in (0, 0.0) else 'FAIL', 'G4 selector output is zero while the robot is idle (it publishes nothing; this is the Watchdog\'s last output)', json.dumps({k: arb.get(k) for k in ('selected_linear', 'selected_turn', 'motor_output')}))
    vel = st.get('velocity') if isinstance(st.get('velocity'), dict) else {}
    add('PASS' if vel.get('status') in ('OK', 'STALE') else 'WARN', 'F8 /odom velocity telemetry present (estimate from wheel odometry, not independent, not proof of motor stop)', vel.get('status'))
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
