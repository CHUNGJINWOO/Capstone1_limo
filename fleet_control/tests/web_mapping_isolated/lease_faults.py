"""Web-control loss scenarios on the isolated motor-locked stack, judged on Nav2 goals and velocity topics.

usage: lease_faults.py RUN_DIR PORT SCENARIO   (fresh run per scenario; SCENARIO = lease | partition | agent_kill | server_restart)
pgids order written by run.sh: server, fixture, stack, vel, agent.
"""
import json, os, signal, subprocess, sys, time, urllib.error, urllib.request

RUN, PORT, SCEN = sys.argv[1], int(sys.argv[2]), sys.argv[3]
BASE = f'http://127.0.0.1:{PORT}/api/v1'
tok = lambda n: open(f'{RUN}/secrets/{n}.token').read().strip()
OPT, VIEW, RID = tok('operator'), tok('viewer'), 'iso_bench'
PG = [int(x) for x in open(f'{RUN}/pgids').read().split()]
RES = []
TAG = str(int(time.time()))


def http(method, path, token=VIEW, body=None, timeout=5):
    req = urllib.request.Request(BASE + path, method=method, data=json.dumps(body).encode() if body is not None else None,
                                 headers={'Content-Type': 'application/json', 'Authorization': 'Bearer ' + token})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b'null')
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b'null')
    except Exception as e:
        return 0, {'error': str(e)}


def check(name, ok, detail=''):
    RES.append({'name': name, 'ok': bool(ok), 'detail': str(detail)[:300]})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {str(detail)[:220]}", flush=True)


def wait(fn, t, step=.5):
    end = time.time() + t
    while time.time() < end:
        v = fn()
        if v:
            return v
        time.sleep(step)
    return None


def vel(since):
    out = []
    for line in open(f'{RUN}/vel.jsonl'):
        r = json.loads(line)
        if r['t'] >= since:
            out.append(r)
    return out


def last_nonzero(rows, topic):
    ts = [r['t'] for r in rows if r['topic'] == topic and (abs(r.get('vx', 0)) > 1e-4 or abs(r.get('wz', 0)) > 1e-4)]
    return max(ts) if ts else None


def nav_active_at_end(rows, topic='/navigate_to_pose/_action/status'):
    st = [r for r in rows if r['topic'] == topic]
    return st[-1]['active'] if st else None


def slam_procs():
    out = subprocess.run(['pgrep', '-f', 'async_slam_toolbox_node|fleet_frontier_candidates'], capture_output=True, text=True).stdout.split()
    return [p for p in out if p != str(os.getpid())]


ttl = 10 if SCEN == 'lease' else 120
c, lease = http('POST', f'/robots/{RID}/lock', OPT, {'action': 'acquire', 'ttl_s': ttl})
epoch = lease['epoch']
c, b = http('POST', f'/robots/{RID}/mapping', OPT, dict(action='START', epoch=epoch, idempotency_key=f'f-{TAG}', mode='NEW_MAP'))
sid = b.get('session_id')
t0 = time.time()
renew = lambda: http('POST', f'/robots/{RID}/lock', OPT, {'action': 'renew', 'ttl_s': ttl})


def moving():
    if SCEN == 'lease' and time.time() - t0 > 4:
        renew()
    w = (http('GET', f'/robots/{RID}')[1].get('state') or {}).get('mapping_worker') or {}
    return w if (w.get('goal') or {}).get('status') == 'ACCEPTED' and last_nonzero(vel(t0), '/cmd_vel_raw') else None


w = wait(moving, 45)
check(f'[{SCEN}] precondition: frontier goal ACCEPTED and /cmd_vel_raw non-zero', w, w and w.get('goal'))
t_fault = time.time()

if SCEN == 'lease':
    # Browser closed: nothing renews the 10 s lease any more.
    exp = http('GET', f'/robots/{RID}/lock')[1]['lease']['expires_at']
    s = wait(lambda: (lambda x: x if x.get('state') == 'UNKNOWN' else None)(http('GET', f'/robots/{RID}/mapping')[1]), ttl + 10)
    t_ref = exp
    check('session UNKNOWN after lease expiry', s, s and round(time.time() - exp, 1))
    time.sleep(15)   # observe the agent-side reaction (renew fence 3 s + goal cancel)
elif SCEN == 'partition':
    # Agent loses the server (network partition emulated by freezing the server process).
    os.killpg(PG[0], signal.SIGSTOP)
    time.sleep(20)
    t_ref = t_fault
elif SCEN == 'agent_kill':
    os.killpg(PG[4], signal.SIGKILL)
    t_ref = t_fault
    time.sleep(20)
elif SCEN == 'server_restart':
    os.killpg(PG[0], signal.SIGKILL)
    time.sleep(2)
    fc = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    p = subprocess.Popen(['bash', '-c', f'cd {fc}/server && PYTHONPATH={fc}/server exec python3 -m fleet_server serve --data-dir {RUN}/fleet --bind 127.0.0.1 --port {PORT}'],
                         stdout=open(f'{RUN}/server_restart.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    open(f'{RUN}/pgids', 'a').write(f'{p.pid}\n')
    t_ref = t_fault
    time.sleep(20)

rows = vel(t_fault - 1)
ln = last_nonzero(rows, '/cmd_vel_raw')
lock_out = last_nonzero(rows, '/cmd_vel_locked')
nav_end = nav_active_at_end(rows)
graph = [r for r in vel(0) if r['topic'] == '/cmd_vel#graph']
check('/cmd_vel had no publisher during the whole run (motor path locked)', graph and all(r['publishers'] == 0 for r in graph), len(graph))
check('Nav2 has no active goal at the end of observation', nav_end == 0, nav_end)
check('/cmd_vel_raw stopped within 10 s after the fault reference', ln is None or ln - t_ref < 10, None if ln is None else round(ln - t_ref, 1))
nav_zero = [r['t'] for r in rows if r['topic'] == '/navigate_to_pose/_action/status' and r['active'] == 0]
print('  timing: last non-zero /cmd_vel_raw %s s, first Nav2 idle %s s after fault reference' % (
    None if ln is None else round(ln - t_ref, 1), round(nav_zero[0] - t_ref, 1) if nav_zero else None))
check('Gate locked output (/cmd_vel_locked) non-zero stopped within 10 s', lock_out is None or lock_out - t_ref < 10, None if lock_out is None else round(lock_out - t_ref, 1))
if SCEN == 'partition':
    check('owned SLAM/Frontier stopped by the agent while the server was unreachable', not slam_procs(), slam_procs())
    os.killpg(PG[0], signal.SIGCONT)
if SCEN != 'agent_kill':
    check('owned SLAM/Frontier processes stopped', wait(lambda: not slam_procs(), 10), slam_procs())
else:
    check('owned SLAM/Frontier killed with the agent (PDEATHSIG)', wait(lambda: not slam_procs(), 10), slam_procs())
s = wait(lambda: (lambda x: x if x.get('state') else None)(http('GET', f'/robots/{RID}/mapping')[1]), 20)
check('server session is UNKNOWN (never silently STOPPED/MAPPING)', s and s['state'] == 'UNKNOWN', s and s.get('state'))
time.sleep(10)
rows2 = vel(time.time() - 10)
check('no automatic restart: no new non-zero /cmd_vel_raw in the last 10 s', last_nonzero(rows2, '/cmd_vel_raw') is None)
c, lease = http('POST', f'/robots/{RID}/lock', OPT, {'action': 'acquire', 'ttl_s': 60})
c, b = http('POST', f'/robots/{RID}/mapping', OPT, dict(action='START', epoch=lease.get('epoch'), idempotency_key=f'g-{TAG}', mode='NEW_MAP'))
check('new START refused afterwards', c == 409, b)
json.dump(RES, open(f'{RUN}/lease_faults_{SCEN}.json', 'w'), indent=1)
print(f"TOTAL {sum(r['ok'] for r in RES)}/{len(RES)} passed")
