#!/usr/bin/env python3
"""Non-motion web checks against a Fleet Server with a REAL-mode Agent (2026-11-22). No ROS here, no motor command.

usage: real_agent_refusals_probe.py --url http://127.0.0.1:18290 --robot pcu --secrets DIR --out RESULT.json [--phase refusals|state]

Phase refusals (needs the operator and admin tokens in DIR; tokens are read from files, never printed):
  Lease acquire -> mode MANUAL request -> manual-bench start -> mapping START -> navigation goal (mission) -> admin Gate reset
  -> manual-bench stop (ACK) -> Lease release. PASS = every motion-related request refused (4xx) and the lease calls accepted.
Phase state (viewer token): online / reconciled / boot id / agent mode / movement_permitted, for restart and link-loss checks.
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request

RES = []


def tok(d, name):
    return open(f'{d}/{name}.token').read().strip()


def http(base, method, path, token, body=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(base + path, data=data, method=method, headers={'Authorization': f'Bearer {token}', 'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=8) as r:
            return r.status, json.loads(r.read() or b'{}')
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read() or b'{}')
        except Exception:
            return e.code, {}
    except Exception as e:  # connection refused etc.
        return None, {'error': type(e).__name__}


def check(name, ok, detail):
    RES.append({'check': name, 'result': 'PASS' if ok else 'FAIL', 'detail': detail})
    print(f"[{'PASS' if ok else 'FAIL'}] {name}  {json.dumps(detail)[:230]}")


def refused(name, code, body):
    err = (body or {}).get('error') or {}
    check(name, code in (400, 403, 409, 422), {'http': code, 'code': err.get('code'), 'message': str(err.get('message'))[:100]})


def state(base, robot, viewer):
    c, b = http(base, 'GET', f'/robots/{robot}', viewer)
    ag = (b or {}).get('agent') or {}
    st = (b or {}).get('state') or {}
    ex = st.get('execution') or {}
    sg = st.get('safety_gate') or {}
    return {'http': c, 'online': (b or {}).get('online'), 'reconciled': (b or {}).get('reconciled'), 'agent_boot_id': ag.get('boot_id'),
            'mode': ag.get('mode'), 'simulated': ag.get('simulated'), 'missions_enabled': ag.get('missions_enabled'),
            'agent_gate_reset': ag.get('gate_reset'), 'agent_manual_bench': ag.get('manual_web_bench'),
            'movement_permitted': sg.get('movement_permitted')}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', required=True); ap.add_argument('--robot', required=True); ap.add_argument('--secrets', required=True)
    ap.add_argument('--out', required=True); ap.add_argument('--phase', default='refusals', choices=['refusals', 'state'])
    a = ap.parse_args()
    base, robot = a.url.rstrip('/') + '/api/v1', a.robot
    viewer = tok(a.secrets, 'viewer')
    if a.phase == 'state':
        s = state(base, robot, viewer)
        print(json.dumps(s))
        json.dump({'phase': 'state', 'state': s}, open(a.out, 'w'), indent=1)
        return 0
    op, admin = tok(a.secrets, 'operator'), tok(a.secrets, 'admin')
    s0 = state(base, robot, viewer)
    check('Agent online, real mode, reconciled', s0['online'] is True and s0['mode'] == 'real' and s0['simulated'] is False and s0['reconciled'] is True, s0)
    if s0['online'] is not True:
        print('SKIP: robot is not online, no request was sent'); json.dump({'phase': 'refusals', 'result': 'SKIP_NOT_ONLINE', 'state': s0}, open(a.out, 'w'), indent=1); return 2
    check('movement_permitted is false', s0['movement_permitted'] is False, {'movement_permitted': s0['movement_permitted']})
    c, lease = http(base, 'POST', f'/robots/{robot}/lock', op, {'action': 'acquire', 'ttl_s': 30})
    check('Lease acquire ACK', c == 200 and isinstance(lease.get('epoch'), int), {'http': c, 'epoch': lease.get('epoch')})
    epoch = lease.get('epoch')
    # 2026-11-22: the server's request key is `target` (the earlier `mode` key only produced a 400 bad_mode and proved nothing)
    c, b = http(base, 'POST', f'/robots/{robot}/mode', op, {'target': 'MANUAL', 'epoch': epoch, 'idempotency_key': 'probe-mode-manual'})
    mode_http = c
    refused('Operating mode MANUAL refused (Agent has no manual bench)', c, b)
    for tgt in ('MAPPING', 'NAVIGATION'):          # observed only: not part of the required refusals; reverted to IDLE below
        c, b = http(base, 'POST', f'/robots/{robot}/mode', op, {'target': tgt, 'epoch': epoch, 'idempotency_key': f'probe-mode-{tgt.lower()}'})
        RES.append({'check': f'observe: mode {tgt}', 'result': 'INFO', 'detail': {'http': c, 'code': ((b or {}).get('error') or {}).get('code')}})
        print(f"[INFO] observe: mode {tgt}  http={c} code={((b or {}).get('error') or {}).get('code')}")
    c, b = http(base, 'POST', f'/robots/{robot}/mode', op, {'target': 'IDLE', 'epoch': epoch, 'idempotency_key': 'probe-mode-idle'})
    RES.append({'check': 'observe: mode IDLE', 'result': 'INFO', 'detail': {'http': c}})
    time.sleep(2.0)
    c, b = http(base, 'POST', f'/robots/{robot}/manual-bench', op, {'action': 'start', 'epoch': epoch})
    refused('Manual bench start refused', c, b)
    c, b = http(base, 'POST', f'/robots/{robot}/mapping', op, dict(action='START', epoch=epoch, idempotency_key='probe-map-start', mode='NEW_MAP'))
    refused('Mapping START refused', c, b)
    c, b = http(base, 'POST', '/missions', op, dict(robot_id=robot, type='navigate_to_pose', idempotency_key='probe-goal', note='refusal probe',
                                                  goal=dict(frame_id='map', x=0.0, y=0.0, yaw_rad=0.0), lease_epoch=epoch))
    refused('Navigation goal refused', c, b)
    mission_id = (b or {}).get('mission_id')
    if c is not None and c < 300 and mission_id:
        http(base, 'POST', f'/missions/{mission_id}/cancel', op, {})
    c, b = http(base, 'POST', f'/robots/{robot}/gate-reset', admin, {'epoch': epoch})
    refused('Gate reset refused', c, b)
    c, b = http(base, 'POST', f'/robots/{robot}/gate-reset', op, {'epoch': epoch})
    refused('Gate reset by operator refused (admin only)', c, b)
    c, b = http(base, 'POST', f'/robots/{robot}/manual-bench', op, {'action': 'stop', 'epoch': epoch, 'session_id': 'none'})
    check('Manual bench STOP answered', c is not None, {'http': c, 'code': ((b or {}).get('error') or {}).get('code')})
    c, b = http(base, 'POST', f'/robots/{robot}/lock', op, {'action': 'release'})
    check('Lease release ACK', c == 200, {'http': c})
    s1 = state(base, robot, viewer)
    check('still not permitted to move after the sequence', s1['movement_permitted'] is False, s1)
    out = {'phase': 'refusals', 'mode_request_http': mode_http, 'results': RES, 'summary': {k: sum(1 for r in RES if r['result'] == k) for k in ('PASS', 'FAIL')}}
    json.dump(out, open(a.out, 'w'), indent=1)
    print('SUMMARY', out['summary'])
    return 1 if out['summary']['FAIL'] else 0


if __name__ == '__main__':
    sys.exit(main())
