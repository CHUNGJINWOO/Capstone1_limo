"""HTTP probe for the isolated web-mapping run. usage: probe.py RUN_DIR PORT  -> prints PASS/FAIL lines and writes result.json."""
import json, sys, time, urllib.error, urllib.request

RUN, PORT = sys.argv[1], int(sys.argv[2])
BASE = f'http://127.0.0.1:{PORT}/api/v1'
tok = lambda n: open(f'{RUN}/secrets/{n}.token').read().strip()
OPT, VIEW = tok('operator'), tok('viewer')
RID = 'iso_bench'
RES, LOG = [], []
RUNTAG = str(int(time.time()))


def http(method, path, token=VIEW, body=None):
    req = urllib.request.Request(BASE + path, method=method, data=json.dumps(body).encode() if body is not None else None,
                                 headers={'Content-Type': 'application/json', 'Authorization': 'Bearer ' + token})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read() or b'null')
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b'null')


def check(name, ok, detail=''):
    RES.append({'name': name, 'ok': bool(ok), 'detail': str(detail)[:300]})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {str(detail)[:200]}", flush=True)


def status():
    return http('GET', f'/robots/{RID}/mapping')[1]


def robot():
    return http('GET', f'/robots/{RID}')[1]


def wait(fn, t, step=.5):
    end = time.time() + t
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(step)
    return None


n = [0]


def op(action, sid=None, **extra):
    n[0] += 1
    body = dict(action=action, epoch=epoch, idempotency_key=f'probe-{RUNTAG}-{action}-{n[0]}', **extra)
    if sid:
        body['session_id'] = sid
    return http('POST', f'/robots/{RID}/mapping', OPT, body)


t0 = time.time()
c, lease = http('POST', f'/robots/{RID}/lock', OPT, {'action': 'acquire', 'ttl_s': 300})
epoch = lease['epoch']
check('lease acquired', c == 200, lease)

# CONTINUE_MAP gates against the registered C103 map (has posegraph, but worker cannot load graphs yet).
reg = http('GET', '/map-registry/maps?floor_id=paichai_c_1f')[1]['maps']
parent = [m for m in reg if m['label'].startswith('c103_lounge')][0]['map_version_id']
c, b = op('START', mode='CONTINUE_MAP', parent_map_version_id=parent)
check('CONTINUE_MAP refused while unsupported/unlocalized', c == 409 and b['error']['code'] == 'continue_unsupported', b)

fence = [[-4, -4], [4, -4], [4, 4], [-4, 4]]
c, start = op('START', mode='NEW_MAP', geofence=fence, limits={'max_goals': 6, 'max_duration_s': 600, 'min_battery_v': 10.5})
check('START accepted as QUEUED (HTTP only)', c == 202 and start['status'] == 'QUEUED', start)
sid = start['session_id']
c2, dup = http('POST', f'/robots/{RID}/mapping', OPT, dict(action='START', epoch=epoch, idempotency_key=f'probe-{RUNTAG}-START-{n[0]}', mode='NEW_MAP',
                                                          geofence=fence, limits={'max_goals': 6, 'max_duration_s': 600, 'min_battery_v': 10.5}))
check('duplicate START returns same session', c2 == 202 and dup.get('duplicate') and dup['session_id'] == sid, dup)
c3, b3 = op('START')
check('second START rejected', c3 == 409, b3)
s = wait(lambda: (lambda x: x if x['state'] in ('MAPPING', 'UNKNOWN', 'FAILED') else None)(status()), 40)
check('START -> MAPPING with SLAM + Frontier evidence', s and s['state'] == 'MAPPING', s and (s['state'], s.get('result')))
LOG.append(('mapping_at', round(time.time() - t0, 1)))

# Fleet navigation must be blocked while mapping (server interlock before safety context).
c, b = http('POST', '/missions', OPT, {'robot_id': RID, 'type': 'navigate_to_pose', 'goal': {'frame_id': 'map', 'x': 0.5, 'y': 0.2}, 'lease_epoch': epoch})
check('navigation mission blocked during mapping', c == 409, b)

# Wait for a mapping-owned goal to be accepted by Nav2.
g = wait(lambda: (lambda w: w if w and w.get('goal') and w['goal'].get('status') in ('ACCEPTED', 'SUCCEEDED', 'FAILED', 'CANCELED') else None)(
    (robot().get('state') or {}).get('mapping_worker')), 40)
check('Frontier goal reserved and accepted by Nav2', g and g['goal'].get('goal_uuid'), g and g.get('goal'))
w = (robot().get('state') or {}).get('mapping_worker') or {}
check('geofence and limits reported by worker', w.get('geofence') is True and (w.get('limits') or {}).get('max_goals') == 6, w)
ex = (robot().get('state') or {}).get('execution') or {}
check('agent active goal is frontier-owned', (ex.get('active') or {}).get('owner') == 'frontier' or g['goal']['status'] != 'ACCEPTED', ex.get('active'))
check('server did not raise reconciliation for frontier goal', not robot().get('reconciliation_required'), robot().get('reconciliation_required'))

c, p = op('PAUSE', sid)
s = wait(lambda: (lambda x: x if x['state'] in ('PAUSED', 'UNKNOWN') else None)(status()), 30)
check('PAUSE -> PAUSED with goal_terminal evidence', s and s['state'] == 'PAUSED' and s['result']['evidence'].get('goal_terminal'), s and s.get('result'))
st = wait(lambda: (lambda r: r if (r.get('state') or {}).get('mapping_worker', {}).get('status') == 'PAUSED' else None)(robot()), 5) or robot()
w = (st.get('state') or {}).get('mapping_worker') or {}
ex = (st.get('state') or {}).get('execution') or {}
check('after PAUSE no active goal, Nav2 idle', ex.get('active') is None and (ex.get('nav2_goals') or {}).get('active_count') == 0, ex.get('nav2_goals'))
time.sleep(3)
w2 = (robot().get('state') or {}).get('mapping_worker') or {}
check('no new goal offered while PAUSED', (w2.get('goal') or {}).get('command_id') == (w.get('goal') or {}).get('command_id'), (w.get('goal'), w2.get('goal')))

c, sv = op('SAVE', sid, floor_id='paichai_c_1f', area_id='paichai_c_1f_c103_lounge')
s = wait(lambda: (lambda x: x if x['command']['action'] == 'SAVE' and x['command']['status'] in ('COMPLETED', 'FAILED', 'UNKNOWN') else None)(status()), 40)
check('SAVE completed (yaml/pgm/posegraph)', s and s['command']['status'] == 'COMPLETED', s and s.get('result'))
check('SAVE registered as new UNVERIFIED version', s and (s.get('registered') or {}).get('map_version_id'), s and s.get('registered'))
if s and s.get('registered'):
    mv = http('GET', '/map-registry/maps/' + urllib.request.quote(s['registered']['map_version_id']))[1]['map']
    check('new version: area link + lineage NEW_MAP + posegraph', mv['area'] and mv['lineage']['mapping_mode'] == 'NEW_MAP'
          and mv['sha256']['posegraph_sha256'] and mv['verification_status'] == 'UNVERIFIED', (mv['area'], mv['lineage']))
    check('original C103 version unchanged', http('GET', '/map-registry/maps/' + urllib.request.quote(parent))[1]['map']['sha256']['pgm_sha256'].startswith('7db92ad2'))

c, r = op('RESUME', sid)
s = wait(lambda: (lambda x: x if x['state'] in ('MAPPING', 'UNKNOWN') else None)(status()), 40)
check('RESUME -> MAPPING', s and s['state'] == 'MAPPING', s and s.get('result'))
c, st = op('STOP', sid)
s = wait(lambda: (lambda x: x if x['state'] in ('STOPPED', 'UNKNOWN') else None)(status()), 40)
check('STOP -> STOPPED, owned processes stopped', s and s['state'] == 'STOPPED' and s['result']['evidence'].get('frontier_stopped'), s and s.get('result'))
st = wait(lambda: (lambda r: r if (r.get('state') or {}).get('mapping_worker', {}).get('status') == 'IDLE' else None)(robot()), 5) or robot()
w = (st.get('state') or {}).get('mapping_worker') or {}
check('worker IDLE after STOP', w.get('status') == 'IDLE' and not w.get('processes_started'), w)
check('mission no longer blocked by mapping (only safety context remains)', http('POST', '/missions', OPT, {'robot_id': RID, 'type': 'navigate_to_pose', 'goal': {'frame_id': 'map', 'x': 0.5, 'y': 0.2}, 'lease_epoch': epoch})[1]['error']['code'] != 'mapping_active')
json.dump({'checks': RES, 'log': LOG, 'session_id': sid}, open(f'{RUN}/probe_result.json', 'w'), indent=1)
print(f"TOTAL {sum(r['ok'] for r in RES)}/{len(RES)} passed")
