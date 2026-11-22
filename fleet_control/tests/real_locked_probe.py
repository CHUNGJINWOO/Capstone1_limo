#!/usr/bin/env python3
"""Motor-locked real-LIMO web check through the existing Fleet server (no ROS here, no motor command).

usage: real_locked_probe.py --url http://127.0.0.1:18190 --robot ROBOT_ID --token-file VIEWER_TOKEN
                            [--operator-token-file OPERATOR_TOKEN --refusal]  [--out RESULT.json]

Read-only by default: GET /robots/{id}, /robots/{id}/mapping, map-registry tree. It judges what the dashboard shows
(online, sensors, map, localization, Safety Gate, motor path, watchdog).

--refusal (needs an operator token) performs the one state-changing check requested for the locked test:
acquire a short Lease, request mapping START (must be REFUSED: real robots are blocked), then release the Lease.
A START that is ACCEPTED is reported as FAIL and is immediately followed by STOP.
Tokens are read from files and never printed.
"""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--url', required=True)
    ap.add_argument('--robot', required=True)
    ap.add_argument('--token-file', required=True)
    ap.add_argument('--operator-token-file')
    ap.add_argument('--refusal', action='store_true')
    ap.add_argument('--out')
    a = ap.parse_args()
    if not a.url.startswith(('http://127.0.0.1:', 'http://localhost:')):
        sys.exit('refusing non-local URL')
    base = a.url.rstrip('/') + '/api/v1'
    view = open(a.token_file).read().strip()
    res = []

    def http(method, path, tok=view, body=None):
        req = urllib.request.Request(base + path, method=method, data=None if body is None else json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json', 'Authorization': 'Bearer ' + tok})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read() or b'null')
        except urllib.error.HTTPError as e:
            try:
                return e.code, json.loads(e.read() or b'null')
            except ValueError:
                return e.code, None
        except Exception as e:  # noqa: BLE001
            return 0, {'error': type(e).__name__}

    def check(name, verdict, detail=''):
        res.append({'name': name, 'verdict': verdict, 'detail': detail})
        print(f'[{verdict}] {name}  {json.dumps(detail, ensure_ascii=False)[:220]}', flush=True)

    c, r = http('GET', f'/robots/{a.robot}')
    if c != 200:
        check('robot known to the server', 'FAIL', c)
        return finish(a, res)
    st = r.get('state') or {}
    # The server keeps the LAST reported state with the age_s it had at report time. An offline robot's values are history.
    live = bool(r.get('online')) and isinstance(r.get('state_age_s'), (int, float)) and r['state_age_s'] < 3.0
    if not live:
        st = {}
        print(f"[INFO] state is history (online={r.get('online')}, state_age_s={r.get('state_age_s')}): live checks below are judged on no data")
    check('LIMO online and reconciled', 'PASS' if r.get('online') and r.get('reconciled') else 'FAIL',
          {'online': r.get('online'), 'reconciled': r.get('reconciled')})
    ag = r.get('agent') or {}
    check('Agent session (protocol 2, contract 1, real mode)', 'PASS' if live and ag.get('protocol_version') == 2 and ag.get('safety_contract_version') == 1 else 'FAIL',
          {k: ag.get(k) for k in ('protocol_version', 'safety_contract_version', 'mode', 'simulated')})
    lid = st.get('lidar') or {}
    check('LiDAR received', 'PASS' if lid.get('status') == 'OK' else 'FAIL', {k: lid.get(k) for k in ('status', 'rate_hz', 'age_s')})
    dep = st.get('depth_camera') or {}
    check('Depth status reported', 'PASS' if dep.get('status') not in (None, 'NO_DATA') else 'SKIP', {k: dep.get(k) for k in ('status', 'code', 'age_s')})
    pose = st.get('pose') or {}
    check('pose in map frame (TF map->base_footprint)', 'PASS' if pose.get('status') == 'OK' and pose.get('frame_id') == 'map' and isinstance(pose.get('age_s'), (int, float)) and r.get('online') else 'FAIL',
          {k: pose.get(k) for k in ('status', 'frame_id', 'age_s', 'source')})
    m = st.get('map') or {}
    check('live map received', 'PASS' if m.get('status') in ('UPDATING', 'STATIC') else 'FAIL', {k: m.get(k) for k in ('status', 'width', 'height', 'resolution')})
    check('localization status (CONFIRMED needed for goals)', 'PASS' if st.get('localization_status') == 'CONFIRMED' else 'INFO',
          {'localization_status': st.get('localization_status'), 'map_version_id': st.get('map_version_id')})
    g = st.get('safety_gate') or {}
    mp = (g.get('motor_path') or {})
    check('Safety Gate status fresh', 'PASS' if g.get('status') not in (None, 'NO_DATA') and isinstance(g.get('age_s'), (int, float)) and g['age_s'] < 1.0 else 'FAIL',
          {k: g.get(k) for k in ('status', 'age_s', 'cmd_blocked')})
    check('motor path LOCKED (no /cmd_vel publisher; Gate -> /cmd_vel_locked)', 'PASS' if mp.get('status') == 'LOCKED' else 'FAIL',
          {k: mp.get(k) for k in ('status', 'cmd_vel_publishers', 'cmd_vel_locked_publishers')})
    check('movement not permitted on the real robot', 'PASS' if g.get('movement_permitted') is False else 'FAIL',
          {'movement_permitted': g.get('movement_permitted'), 'reasons': g.get('movement_block_reasons')})
    wd = st.get('motion_watchdog')
    check('motion watchdog reported', 'INFO' if wd is None else ('PASS' if isinstance(wd, dict) else 'FAIL'), wd)
    nav = st.get('nav2') or {}
    check('Nav2 action server ready', 'PASS' if nav.get('action_server_ready') else 'FAIL', nav)
    c, mt = http('GET', '/map-registry/facilities')
    areas = [ar for f in (mt or {}).get('facilities', []) for b in f.get('buildings', []) for fl in b.get('floors', []) for ar in fl.get('areas', [])]
    c103 = [ar for ar in areas if ar.get('area_id') == 'paichai_c_1f_c103_lounge']
    check('C103 lounge area registered as PARTIAL_FLOOR', 'PASS' if c103 and c103[0].get('coverage') == 'PARTIAL_FLOOR' else 'SKIP', c103[:1])
    c, ms = http('GET', f'/robots/{a.robot}/mapping')
    check('mapping session readable', 'PASS' if c == 200 else 'FAIL', {'http': c, 'state': (ms or {}).get('state')})

    if a.refusal:
        op = open(a.operator_token_file).read().strip()
        c, lease = http('POST', f'/robots/{a.robot}/lock', op, {'action': 'acquire', 'ttl_s': 30})
        check('operator Lease acquired (30 s)', 'PASS' if c == 200 else 'FAIL', {'http': c})
        if c == 200:
            key = f'locked-probe-{int(time.time())}'
            c, b = http('POST', f'/robots/{a.robot}/mapping', op, dict(action='START', epoch=lease.get('epoch'), idempotency_key=key, mode='NEW_MAP'))
            code = ((b or {}).get('error') or {}).get('code')
            check('web START refused on the real robot', 'PASS' if c in (403, 409) else 'FAIL', {'http': c, 'code': code})
            if c == 200:
                http('POST', f'/robots/{a.robot}/mapping', op, dict(action='STOP', epoch=lease.get('epoch'), idempotency_key=key + '-stop'))
                check('START was accepted -> STOP sent', 'FAIL', b)
            gk = f'locked-probe-goal-{int(time.time())}'
            c, b = http('POST', f'/missions', op, dict(robot_id=a.robot, type='navigate_to_pose', idempotency_key=gk, note='locked probe: must be refused',
                                                     goal=dict(frame_id='map', x=0.0, y=0.0, yaw_rad=0.0), lease_epoch=lease.get('epoch')))
            code = ((b or {}).get('error') or {}).get('code')
            check('map-click Goal (Fleet mission) refused on the real robot', 'PASS' if c in (403, 409) else 'FAIL', {'http': c, 'code': code})
            if c in (200, 201) and isinstance(b, dict) and b.get('mission_id'):
                http('POST', f"/missions/{b['mission_id']}/cancel", op, {})
                check('mission was accepted -> cancel sent', 'FAIL', b.get('mission_id'))
            c, _ = http('POST', f'/robots/{a.robot}/lock', op, {'action': 'release'})
            check('Lease released', 'PASS' if c == 200 else 'FAIL', {'http': c})
    return finish(a, res)


def finish(a, res):
    if a.out:
        json.dump({'t': time.time(), 'robot': a.robot, 'checks': res}, open(a.out, 'w'), indent=1, ensure_ascii=False)
    n = {v: sum(r['verdict'] == v for r in res) for v in ('PASS', 'FAIL', 'SKIP', 'INFO')}
    print('SUMMARY', n)
    return 1 if n['FAIL'] else 0


if __name__ == '__main__':
    sys.exit(main())
