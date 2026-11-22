#!/usr/bin/env python3
"""Motor-locked REAL-LIMO check of the web 'save current SLAM map and register' path (through the Fleet server on 127.0.0.1).

usage: probe_real_save.py --url http://127.0.0.1:18190 --robot pcu --operator-token-file F --viewer-token-file F --ssh pcu@IP --out DIR [--area-contains h107]
Sends exactly one save request (plus an idempotent replay of the same key). No mission, no goal, no mapping start (a START is sent
once and must be REFUSED), no velocity. Tokens come from files and are never printed. Robot-side files are re-hashed over ssh and compared
with the Agent manifest and with what the registry stored.
"""
import argparse, hashlib, json, subprocess, sys, time, urllib.error, urllib.request

ap = argparse.ArgumentParser()
for k in ('--url', '--robot', '--operator-token-file', '--viewer-token-file', '--ssh', '--out'):
    ap.add_argument(k, required=True)
ap.add_argument('--area-contains', default='h107')
ap.add_argument('--label', default='web_save_real_locked')
a = ap.parse_args()
if not a.url.startswith('http://127.0.0.1:'):
    sys.exit('refusing non-local URL')
BASE = a.url.rstrip('/') + '/api/v1'
OPT, VIEW = (open(f).read().strip() for f in (a.operator_token_file, a.viewer_token_file))
RID, TAG, RES = a.robot, str(int(time.time())), []


def http(method, path, tok=OPT, body=None, timeout=15):
    req = urllib.request.Request(BASE + path, method=method, data=None if body is None else json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json', 'Authorization': 'Bearer ' + tok})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b'null')
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b'null')


def check(name, ok, detail=''):
    RES.append({'name': name, 'ok': bool(ok), 'detail': str(detail)[:400]})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {str(detail)[:220]}", flush=True)


def ssh(cmd, t=40):
    p = subprocess.run(['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', a.ssh, cmd], capture_output=True, text=True, timeout=t)
    return p.returncode, p.stdout.strip()


def cmd_vel_publishers():
    rc, out = ssh('source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1; ros2 topic info /cmd_vel 2>&1 | grep -i "publisher count"')
    return out


def tree():
    c, t = http('GET', '/map-registry/facilities', VIEW)
    return [(fl['floor_id'], ar) for f in t['facilities'] for b in f['buildings'] for fl in b['floors'] for ar in fl['areas']]


def meta(m):
    return http('GET', f'/map-registry/maps/{m.replace("#", "%23")}/meta', VIEW)[1]


def versions():
    return {m: meta(m) for _, ar in tree() for m in ar.get('map_version_ids', [])}


print('== real LIMO motor-locked map save + register')
sv = lambda: http('GET', f'/robots/{RID}/map-saves', VIEW)[1]
cap = None
end = time.time() + 40
while time.time() < end:
    cap = sv()['capability']
    if cap['available']:
        break
    time.sleep(1.5)
check('Agent reports map save available on the real robot', cap and cap['available'], cap)
rb = http('GET', f'/robots/{RID}', VIEW)[1]
r = rb.get('robot', rb)
check('robot online and state fresh', r.get('online') and (r.get('state_age_s') or 99) < 3, (r.get('online'), r.get('state_age_s')))
pub0 = cmd_vel_publishers()
check('/cmd_vel has 0 publishers before', 'Publisher count: 0' in pub0, pub0)
before = versions()
areas = [(fid, ar) for fid, ar in tree() if a.area_contains.lower() in ar['area_id'].lower()]
check('target area exists in registry', len(areas) == 1, [x[1]['area_id'] for x in areas])
if len(areas) != 1:
    sys.exit(2)
FLOOR, AREA = areas[0][0], areas[0][1]['area_id']
print('   target', FLOOR, AREA, '| versions before:', len(before))

c, lease = http('POST', f'/robots/{RID}/lock', OPT, {'action': 'acquire', 'ttl_s': 300})
check('lease acquired', c == 200, c)
epoch = lease['epoch']
c, b = http('POST', f'/robots/{RID}/mapping', OPT, dict(action='START', epoch=epoch, idempotency_key='m-' + TAG, mode='NEW_MAP'))
check('web mapping START refused on the real robot', c in (403, 409, 422), (c, (b or {}).get('error')))

body = dict(epoch=epoch, idempotency_key='real1-' + TAG, floor_id=FLOOR, area_id=AREA, label=a.label, note='motor-locked real robot save test')
c, b = http('POST', f'/robots/{RID}/map-save', OPT, body)
check('save accepted as QUEUED', c == 202 and b['status'] == 'QUEUED', b)
sid = b.get('save_id')
c2, b2 = http('POST', f'/robots/{RID}/map-save', OPT, body)
check('replay with same key returns the same save', c2 == 202 and b2.get('save_id') == sid and b2.get('duplicate') is True, b2)
seen, t0, s = [], time.time(), None
while time.time() - t0 < 240:
    s = next((x for x in sv()['saves'] if x['save_id'] == sid), None)
    if s and (not seen or seen[-1][1] != s['status']):
        seen.append((round(time.time() - t0, 1), s['status']))
    if s and s['status'] in ('REGISTERED', 'FAILED', 'TIMEOUT', 'UNKNOWN'):
        break
    time.sleep(0.3)
print('   stages:', seen)
check('save reaches REGISTERED', s and s['status'] == 'REGISTERED', (s or {}).get('detail') or (s or {}).get('error_code'))
mani = (s or {}).get('manifest') or {}
files = mani.get('files') or {}
check('manifest lists yaml/pgm/posegraph/data with sizes', set(files) == {'yaml', 'pgm', 'posegraph', 'data'} and all(v['size'] > 0 for v in files.values()), {k: v['size'] for k, v in files.items()})
if files:
    rc, out = ssh('cd ~/.local/state/limo_robot_agent_field_20261120/map_saves/%s && sha256sum map.yaml map.pgm graph.posegraph graph.data && stat -c "%%n %%s" map.yaml map.pgm graph.posegraph graph.data' % sid)
    rows = out.splitlines()
    robot_sha = {l.split()[1]: l.split()[0] for l in rows[:4] if len(l.split()) == 2}
    robot_size = {l.split()[0]: int(l.split()[1]) for l in rows[4:] if len(l.split()) == 2}
    check('robot-side files re-hashed over ssh equal the manifest (sha256 and size)',
          all(robot_sha.get(v['name']) == v['sha256'] and robot_size.get(v['name']) == v['size'] for v in files.values()), {k: robot_sha.get(v['name'], '?')[:12] for k, v in files.items()})
cons = (s or {}).get('consistency') or {}
check('consistency recorded, never overclaimed', cons.get('same_snapshot') in ('LIKELY_NOT_PROVEN', 'NOT_ESTABLISHED'), cons)
print('   raster window', (mani.get('results') or {}).get('raster'), '| graph window', (mani.get('results') or {}).get('posegraph'))
mvid = ((s or {}).get('result') or {}).get('map_version_id')
if mvid:
    m = meta(mvid)
    ev = m.get('verification_evidence') or {}
    check('registered UNVERIFIED and not floor-active', m.get('verification_status') == 'UNVERIFIED' and m.get('is_floor_active') is False, (m.get('verification_status'), m.get('is_floor_active')))
    check('area link + NEW_MAP lineage', (m.get('area') or {}).get('area_id') == AREA and (m.get('lineage') or {}).get('mapping_mode') == 'NEW_MAP')
    sf = ev.get('saved_files') or {}
    check('registry evidence hashes equal the robot-side files',
          ev.get('method') == 'web_map_save' and ev.get('robot_id') == RID and all(sf.get(n, {}).get('sha256') == files[k]['sha256'] for k, n in (('pgm', 'map/map.pgm'), ('posegraph', 'map/posegraph.posegraph'), ('data', 'map/posegraph.data'))), ev.get('method'))
    full = http('GET', f'/map-registry/maps/{mvid.replace("#", "%23")}', VIEW)[1].get('map') or {}
    print('   registered', mvid, '| free_thresh', full.get('free_thresh'), '| yaml_adjustment', ev.get('yaml_adjustment'))
after = versions()
check('every pre-existing version (C103, H107, ...) is byte-for-byte unchanged in metadata', all(after.get(k) == v for k, v in before.items()), [k for k, v in before.items() if after.get(k) != v])
check('exactly one new version was added', len(after) == len(before) + 1, (len(before), len(after)))
http('POST', f'/robots/{RID}/lock', OPT, {'action': 'release', 'epoch': epoch})
pub1 = cmd_vel_publishers()
check('/cmd_vel has 0 publishers after', 'Publisher count: 0' in pub1, pub1)
json.dump({'checks': RES, 'save_id': sid, 'stages': seen, 'map_version_id': mvid, 'manifest': mani, 'consistency': cons}, open(a.out + '/real_save_result.json', 'w'), indent=1)
print(f"TOTAL {sum(x['ok'] for x in RES)}/{len(RES)} passed")
sys.exit(0 if all(x['ok'] for x in RES) else 1)
