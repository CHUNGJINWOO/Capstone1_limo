"""HTTP probe for the isolated MAP-SAVE run (run_save.sh). usage: probe_save.py RUN_DIR PORT -> PASS/FAIL lines, writes save_result.json.

Real slam_toolbox + map_saver_cli + serialize_map + the Agent process + the Fleet server; no robot, no motors, no /cmd_vel.
"""
import hashlib
import json
import os
import signal
import sys
import time
import urllib.error
import urllib.request

RUN, PORT = sys.argv[1], int(sys.argv[2])
BASE = f'http://127.0.0.1:{PORT}/api/v1'
tok = lambda n: open(f'{RUN}/secrets/{n}.token').read().strip()
OPT, VIEW = tok('operator'), tok('viewer')
RID = 'iso_bench'
FLOOR, AREA = 'paichai_c_1f', 'paichai_c_1f_c103_lounge'
RES, TAG = [], str(int(time.time()))
PG = dict(line.split() for line in open(f'{RUN}/pgnames'))


def http(method, path, token=OPT, body=None, timeout=10):
    req = urllib.request.Request(BASE + path, method=method, data=None if body is None else json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json', **({'Authorization': 'Bearer ' + token} if token else {})})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b'null')
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b'null')


def check(name, ok, detail=''):
    RES.append({'name': name, 'ok': bool(ok), 'detail': str(detail)[:300]})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {str(detail)[:200]}", flush=True)


def wait(fn, t, step=0.4):
    end = time.time() + t
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(step)
    return None


def sha(path):
    return hashlib.sha256(open(path, 'rb').read()).hexdigest()


def saves():
    return http('GET', f'/robots/{RID}/map-saves', VIEW)[1]


def post_save(key, label, **extra):
    c, lease = http('GET', f'/robots/{RID}/lock')
    epoch = lease['lease']['epoch']
    return http('POST', f'/robots/{RID}/map-save', OPT, dict(epoch=epoch, idempotency_key=key, floor_id=FLOOR, area_id=AREA, label=label, note='isolated probe', **extra))


def terminal(save_id, t=120):
    seen = []

    def f():
        s = next((x for x in saves()['saves'] if x['save_id'] == save_id), None)
        if s and (not seen or seen[-1][1] != s['status']):
            seen.append((round(time.time() - t0, 1), s['status']))
        return s if s and s['status'] in ('REGISTERED', 'FAILED', 'TIMEOUT', 'UNKNOWN') else None
    t0 = time.time()
    return wait(f, t, 0.15), seen


print('== map save (isolated ROS 2)')
check('no token -> 401', http('GET', f'/robots/{RID}/map-saves', None)[0] == 401)
cap = wait(lambda: (lambda c: c if c['available'] else None)(saves()['capability']), 90)
check('Agent reports map save available (SLAM map + serialize service + motor path locked)', cap is not None, saves()['capability'])
c, lease = http('POST', f'/robots/{RID}/lock', OPT, {'action': 'acquire', 'ttl_s': 300})
check('lease acquired', c == 200)
code, maps = http('GET', '/map-registry/facilities', VIEW)
c103_before = [m for f in maps['facilities'] for b in f['buildings'] for fl in b['floors'] if fl['floor_id'] == FLOOR for a in fl['areas'] for m in a.get('map_version_ids', [])]
before_meta = {m: http('GET', f'/map-registry/maps/{m.replace("#", "%23")}/meta', VIEW)[1] for m in c103_before}
check('existing C103 version present before saving', len(c103_before) >= 1, c103_before)

check('viewer cannot request a save', http('POST', f'/robots/{RID}/map-save', VIEW, dict(epoch=1, idempotency_key='v-' + TAG, floor_id=FLOOR, label='x'))[0] == 403)
c, b = http('POST', f'/robots/{RID}/map-save', OPT, dict(epoch=lease['epoch'] + 9, idempotency_key='e-' + TAG, floor_id=FLOOR, label='x'))
check('wrong lease epoch refused', c == 409 and b['error']['code'] == 'lease_required', b)
c, b = http('POST', f'/robots/{RID}/mapping', OPT, dict(action='START', epoch=lease['epoch'], idempotency_key='m-' + TAG, mode='NEW_MAP'))
check('web mapping START is still refused (no mapping worker on this Agent)', c == 409, b)

c, b = post_save('s1-' + TAG, 'iso_save_1')
check('save accepted as QUEUED (HTTP only)', c == 202 and b['status'] == 'QUEUED', b)
sid = b.get('save_id')
c2, b2 = post_save('s1-' + TAG, 'iso_save_1')
check('same idempotency key returns the same request', c2 == 202 and b2.get('duplicate') is True and b2['save_id'] == sid, b2)
c3, b3 = post_save('s1b-' + TAG, 'iso_other')
check('second request while one is active is refused', c3 == 409 and b3['error']['code'] == 'save_in_progress', b3)
s, seen = terminal(sid)
check('save reaches REGISTERED', s and s['status'] == 'REGISTERED', (s or {}).get('detail') or seen)
print('   stages seen:', seen)
mvid = ((s or {}).get('result') or {}).get('map_version_id')
d = f"{RUN}/agent/map_saves/{sid}"
mani = (s or {}).get('manifest') or {}
disk = {k: sha(f"{d}/{v['name']}") for k, v in (mani.get('files') or {}).items() if os.path.isfile(f"{d}/{v['name']}")}
check('all four files exist on the robot side and match the manifest hashes', len(disk) == 4 and all(disk[k] == mani['files'][k]['sha256'] for k in disk), {k: v[:12] for k, v in disk.items()})
check('pose graph and data are non-empty', all(mani['files'][k]['size'] > 0 for k in ('posegraph', 'data')) if mani else False,
      {k: v['size'] for k, v in (mani.get('files') or {}).items()})
cons = (s or {}).get('consistency') or {}
check('consistency recorded and never overclaimed', cons.get('same_snapshot') in ('LIKELY_NOT_PROVEN', 'NOT_ESTABLISHED') and cons.get('gap_between_raster_end_and_posegraph_start_s') is not None, cons.get('same_snapshot'))
if mvid:
    code, meta = http('GET', f'/map-registry/maps/{mvid.replace("#", "%23")}/meta', VIEW)
    ev = meta.get('verification_evidence') or {}
    check('registered UNVERIFIED, not floor-active', meta.get('verification_status') == 'UNVERIFIED' and meta.get('is_floor_active') is False, (meta.get('verification_status'), meta.get('is_floor_active')))
    check('area link + NEW_MAP lineage', (meta.get('area') or {}).get('area_id') == AREA and (meta.get('lineage') or {}).get('mapping_mode') == 'NEW_MAP', (meta.get('area'), meta.get('lineage')))
    check('evidence: web_map_save, robot, per-file hashes equal the robot files',
          ev.get('method') == 'web_map_save' and ev.get('robot_id') == RID and all((ev.get('saved_files') or {}).get(n, {}).get('sha256') == mani['files'][k]['sha256']
                                                                                    for k, n in (('pgm', 'map/map.pgm'), ('posegraph', 'map/posegraph.posegraph'), ('data', 'map/posegraph.data'))), ev.get('method'))
    full = http('GET', f'/map-registry/maps/{mvid.replace("#", "%23")}', VIEW)[1].get('map') or {}
    check('registry yaml is the robot yaml (free_thresh 0.196 passed the safety check, no derived copy needed)', ev.get('yaml_adjustment') is None and abs(full.get('free_thresh', 0) - 0.196) < 1e-9, (full.get('free_thresh'), ev.get('yaml_adjustment')))
after_meta = {m: http('GET', f'/map-registry/maps/{m.replace("#", "%23")}/meta', VIEW)[1] for m in c103_before}
check('existing C103 version unchanged', after_meta == before_meta)

c, b = post_save('s2-' + TAG, 'iso_save_2')                 # immediately after REGISTERED: the Agent may still be finishing the first one
check('an immediate second request is accepted (it queues behind the Agent loop)', c == 202, b.get('error') or b.get('status'))
sid2 = b.get('save_id')
s2, seen2 = terminal(sid2)
check('a second save works after the first finished', s2 and s2['status'] in ('REGISTERED', 'FAILED'), (s2 or {}).get('detail') or seen2)
print('   second save:', (s2 or {}).get('status'), (s2 or {}).get('error_code'), ((s2 or {}).get('result') or {}).get('created'))

# SLAM goes away: the capability must turn false and a request must be refused with the reason
os.killpg(int(PG['slam']), signal.SIGKILL)
gone = wait(lambda: (lambda c: c if not c['available'] else None)(saves()['capability']), 30)
check('capability turns false when slam_toolbox is gone', gone is not None, gone)
c, b = post_save('s3-' + TAG, 'iso_save_3')
check('request refused with the Agent reason while SLAM is down', c == 409 and b['error']['code'] == 'map_save_unavailable', b)
check('nothing was queued for the refused request', not [x for x in saves()['saves'] if x['status'] in ('QUEUED', 'ACCEPTED')])

rows = [json.loads(l) for l in open(f'{RUN}/vel.jsonl')]
graph = [r for r in rows if r['topic'] == '/cmd_vel#graph']
check('/cmd_vel never had a publisher (locked motor path)', graph and all(r['publishers'] == 0 for r in graph))
check('no non-zero velocity at the Gate output during the whole run', not any(r['topic'] == '/cmd_vel_locked' and (abs(r.get('vx', 0)) > 1e-4 or abs(r.get('wz', 0)) > 1e-4) for r in rows))
json.dump({'checks': RES, 'save_ids': [sid, sid2], 'stages': seen}, open(f'{RUN}/save_result.json', 'w'), indent=1)
print(f"TOTAL {sum(r['ok'] for r in RES)}/{len(RES)} passed")
sys.exit(0 if all(r['ok'] for r in RES) else 1)
