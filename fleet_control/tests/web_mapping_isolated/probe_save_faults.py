"""Isolated fault checks for the web map save: usage probe_save_faults.py RUN_DIR PORT DOMAIN  (stack from run_save.sh; OVERLAY exported).
Case 1: Agent SIGKILLed during SAVING_RASTER, then restarted. Case 2: slam_toolbox SIGKILLed during SAVING_RASTER. Records what the web API shows."""
import json, os, signal, subprocess, sys, time, urllib.request, urllib.error
RUN, PORT, DOM = sys.argv[1], int(sys.argv[2]), sys.argv[3]
BASE = f'http://127.0.0.1:{PORT}/api/v1'
tok = lambda n: open(f'{RUN}/secrets/{n}.token').read().strip()
OPT, VIEW, RID = tok('operator'), tok('viewer'), 'iso_bench'
FLOOR, AREA = 'paichai_c_1f', 'paichai_c_1f_c103_lounge'
PG = dict(l.split() for l in open(f'{RUN}/pgnames'))
RES = []


def http(m, p, t=OPT, b=None):
    r = urllib.request.Request(BASE + p, method=m, data=None if b is None else json.dumps(b).encode(), headers={'Content-Type': 'application/json', 'Authorization': 'Bearer ' + t})
    try:
        with urllib.request.urlopen(r, timeout=10) as x:
            return x.status, json.loads(x.read() or b'null')
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b'null')


def check(n, ok, d=''):
    RES.append({'name': n, 'ok': bool(ok), 'detail': str(d)[:300]}); print(f"  [{'PASS' if ok else 'FAIL'}] {n} {str(d)[:240]}", flush=True)


def saves(): return http('GET', f'/robots/{RID}/map-saves', VIEW)[1]
def versions():
    return sum(len(a.get('map_version_ids', [])) for f in http('GET', '/map-registry/facilities', VIEW)[1]['facilities'] for b in f['buildings'] for fl in b['floors'] for a in fl['areas'])
def wait(fn, t, s=0.3):
    e = time.time() + t
    while time.time() < e:
        v = fn()
        if v: return v
        time.sleep(s)


def request(key):
    epoch = http('GET', f'/robots/{RID}/lock')[1]['lease']['epoch']
    return http('POST', f'/robots/{RID}/map-save', OPT, dict(epoch=epoch, idempotency_key=key, floor_id=FLOOR, area_id=AREA, label=key))


def get(sid): return next((x for x in saves()['saves'] if x['save_id'] == sid), None)


tag = str(int(time.time()))
check('capability available at start', wait(lambda: saves()['capability']['available'], 90))
http('POST', f'/robots/{RID}/lock', OPT, {'action': 'acquire', 'ttl_s': 300})
v0 = versions()
print('== case 1: Agent SIGKILLed while SAVING_RASTER, then restarted')
c, b = request('f1-' + tag); sid = b['save_id']
seen = wait(lambda: (lambda s: s and s['status'] == 'SAVING_RASTER')(get(sid)), 15, 0.05)
check('save reached SAVING_RASTER before the kill', seen)
os.killpg(int(PG['agent']), signal.SIGKILL); tk = time.time()
s = wait(lambda: (lambda s: s if s and s['status'] in ('UNKNOWN', 'TIMEOUT', 'FAILED', 'REGISTERED') else None)(get(sid)), 60)
check('server shows a terminal state (not stuck as running)', s and s['status'] == 'UNKNOWN' and s['error_code'] == 'robot_offline', (s or {}).get('status', None) and (s['status'], s['error_code'], s['detail'], round(time.time() - tk, 1)))
check('nothing registered', versions() == v0, (v0, versions()))
subprocess.run(['bash', os.path.join(os.path.dirname(os.path.abspath(__file__)), 'restart_agent_iso.sh'), RUN, str(PORT), DOM], check=True)
check('capability returns after the Agent restarts', wait(lambda: saves()['capability']['available'], 90))
s = get(sid)
check('the interrupted save stays UNKNOWN (sticky) after the restart; late files are not registered', s['status'] == 'UNKNOWN' and versions() == v0, (s['status'], s['error_code']))
local = json.load(open(f'{RUN}/agent/map_saves/{sid}/status.json'))
check('Agent journal marks the interrupted save INTERRUPTED and keeps its files', local['status'] == 'INTERRUPTED' and local['error_code'] == 'agent_restarted', (local['status'], os.listdir(f'{RUN}/agent/map_saves/{sid}')))
c, b = request('f1b-' + tag); s2 = wait(lambda: (lambda s: s if s and s['status'] in ('REGISTERED', 'FAILED', 'UNKNOWN', 'TIMEOUT') else None)(get(b['save_id'])), 90)
check('a new save works after the restart (new save id, never a replay)', c == 202 and b['save_id'] != sid and s2 and s2['status'] == 'REGISTERED', (s2 or {}).get('status'))
v1 = versions()
print('== case 2: slam_toolbox SIGKILLed while SAVING_RASTER')
c, b = request('f2-' + tag); sid2 = b['save_id']
seen = wait(lambda: (lambda s: s and s['status'] == 'SAVING_RASTER')(get(sid2)), 15, 0.05)
check('save reached SAVING_RASTER before the kill', seen)
os.killpg(int(PG['slam']), signal.SIGKILL)
s = wait(lambda: (lambda s: s if s and s['status'] in ('UNKNOWN', 'TIMEOUT', 'FAILED', 'REGISTERED') else None)(get(sid2)), 90)
check('server shows FAILED with a cause', s and s['status'] == 'FAILED', s and (s['status'], s['error_code'], s['detail']))
check('nothing registered for the failed save', versions() == v1, (v1, versions()))
check('capability is false with the Agent reason afterwards', wait(lambda: not saves()['capability']['available'], 30), saves()['capability'])
rows = [json.loads(l) for l in open(f'{RUN}/vel.jsonl')]
check('/cmd_vel never had a publisher; no non-zero velocity at the Gate', all(r['publishers'] == 0 for r in rows if r['topic'] == '/cmd_vel#graph') and not any(r['topic'] == '/cmd_vel_locked' and (abs(r.get('vx', 0)) > 1e-4 or abs(r.get('wz', 0)) > 1e-4) for r in rows))
json.dump({'checks': RES}, open(f'{RUN}/fault_result.json', 'w'), indent=1)
print(f"TOTAL {sum(r['ok'] for r in RES)}/{len(RES)} passed"); sys.exit(0 if all(r['ok'] for r in RES) else 1)
