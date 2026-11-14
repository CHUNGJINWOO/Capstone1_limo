#!/usr/bin/env python3
"""Server-side checks of the control-ownership lease, its race behaviour and the operator recovery (no ROS, no robot: a scripted fake agent speaks the
agent HTTP protocol). Isolated server on port 18083 with a temp data dir. usage: python3 tests/test_ownership.py"""
import hashlib, json, os, shutil, signal, subprocess, sys, tempfile, threading, time, urllib.error, urllib.request, uuid
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
PORT = int(os.environ.get('OWN_TEST_PORT', '18083')); BASE = f'http://127.0.0.1:{PORT}'
T = tempfile.mkdtemp(prefix='own_test_'); DATA, SEC = os.path.join(T, 'data'), os.path.join(T, 'sec')
R = []


def check(n, ok, d=''): R.append(bool(ok)); print(f"  [{'PASS' if ok else 'FAIL'}] {n} {d}", flush=True)


def tok(n): return open(os.path.join(SEC, n + '.token')).read().strip()


def http(method, path, token, body=None):
    req = urllib.request.Request(BASE + '/api/v1' + path, method=method, data=json.dumps(body).encode() if body is not None else None, headers={'Content-Type': 'application/json'})
    if token: req.add_header('Authorization', 'Bearer ' + token)
    try:
        with urllib.request.urlopen(req, timeout=8) as r: d = r.read(); return r.status, (json.loads(d) if d else None)
    except urllib.error.HTTPError as e:
        d = e.read(); return e.code, (json.loads(d) if d else None)


def add_token(name, role):
    ap = os.path.join(DATA, 'auth.json'); a = json.load(open(ap)); t = uuid.uuid4().hex + uuid.uuid4().hex
    a['tokens'].append({'name': name, 'role': role, 'robot_id': None, 'sha256': hashlib.sha256(t.encode()).hexdigest()}); json.dump(a, open(ap + '.n', 'w')); os.replace(ap + '.n', ap)
    open(os.path.join(SEC, name + '.token'), 'w').write(t)


class FakeAgent:
    def __init__(self, rid='pcu'):
        self.rid, self.boot, self.seq, self.t = rid, uuid.uuid4().hex[:10], 0, tok('agent-' + rid)
        self.active = None; self.nav2_active = 0; self.nav2_known = True; self.received = []

    def hello(self):
        http('POST', '/agent/hello', self.t, {'robot_id': self.rid, 'agent': {'protocol_version': 2, 'boot_id': self.boot, 'mode': 'mock', 'simulated': True, 'missions_enabled': True}})
        return http('POST', '/agent/reconcile', self.t, {'robot_id': self.rid, 'agent_boot_id': self.boot, 'journal_ok': True, 'journal': {}, 'execution': {'active': self.active, 'pending_cancel_ids': []}})

    def post_state(self):
        self.seq += 1
        st = {'schema_version': 1, 'execution': {'active': self.active, 'pending_cancel_ids': [], 'nav2_goals': {'known': self.nav2_known, 'active_count': self.nav2_active if self.nav2_known else None, 'publishers': 1}}}
        c, d = http('POST', '/agent/state', self.t, {'robot_id': self.rid, 'agent_boot_id': self.boot, 'seq': self.seq, 'state': st}); return d

    def ack(self, cid, status, detail=''):
        return http('POST', f'/agent/commands/{cid}/ack', self.t, {'robot_id': self.rid, 'agent_boot_id': self.boot, 'status': status, 'detail': detail})

    def take_goal(self):
        for _ in range(20):
            d = self.post_state() or {}
            for c in d.get('commands', []):
                if c['type'] == 'navigate_to_pose' and c['command_id'] not in self.received:
                    self.received.append(c['command_id']); self.active = {'mission_id': c['mission_id'], 'command_id': c['command_id'], 'goal_handle_available': True}; self.nav2_active = 1
                    self.ack(c['command_id'], 'EXECUTING', 'navigating'); return c
            time.sleep(0.2)


def goal(x=1.0, y=1.0): return {'frame_id': 'map', 'x': x, 'y': y, 'yaw_rad': 0}
def create(token, key=None, x=1.0, robot='pcu'): return http('POST', '/missions', token, {'robot_id': robot, 'type': 'navigate_to_pose', 'goal': goal(x), 'idempotency_key': key})
def finish_goal(ag, status='SUCCEEDED'):
    ag.ack(ag.active['command_id'], status); ag.active = None; ag.nav2_active = 0; ag.post_state()


procs = []
try:
    env = dict(os.environ, PYTHONPATH=os.path.join(ROOT, 'server'))
    subprocess.run([sys.executable, '-m', 'fleet_server', 'init', '--data-dir', DATA, '--secrets-dir', SEC, '--robots', 'pcu', 'pcu2'], cwd=os.path.join(ROOT, 'server'), env=env, check=True, capture_output=True)
    add_token('rmf-adapter', 'operator'); add_token('rmf-other', 'operator')
    procs.append(subprocess.Popen([sys.executable, '-m', 'fleet_server', 'serve', '--data-dir', DATA, '--port', str(PORT), '--offline-after', '30'], cwd=os.path.join(ROOT, 'server'), env=env, stdout=open(os.path.join(T, 'srv.log'), 'w'), stderr=subprocess.STDOUT))
    for _ in range(50):
        try: urllib.request.urlopen(BASE + '/api/v1/health', timeout=1); break
        except Exception: time.sleep(0.2)
    A, OP, RMF, VW, OTHER = tok('admin'), tok('operator'), tok('rmf-adapter'), tok('viewer'), tok('rmf-other')
    ag = FakeAgent(); ag.hello(); ag.post_state()

    print('== lease: acquire / renew / release / conflicts')
    c, d = create(OP, 'w0'); check('without a lease the old behaviour is unchanged (web creates a mission)', c == 201, str(c))
    c, d = http('POST', '/robots/pcu/lock', RMF, {'action': 'acquire'})
    check('lease cannot be taken while another principal has an unfinished mission (409 robot_busy)', c == 409 and d['error']['code'] == 'robot_busy', str(d)[:120])
    ag.take_goal(); finish_goal(ag)
    c, d = http('POST', '/robots/pcu/lock', RMF, {'action': 'acquire', 'ttl_s': 6, 'note': 'rmf'})
    check('lease acquired after the web mission ended', c == 200 and d['owner'] == 'rmf-adapter' and d['epoch'] >= 1, str(d)[:120]); ep = d['epoch']
    c, d = http('POST', '/robots/pcu/lock', RMF, {'action': 'acquire', 'ttl_s': 6}); check('re-acquire by the owner renews and keeps the epoch', c == 200 and d['epoch'] == ep)
    c, d = http('POST', '/robots/pcu/lock', OTHER, {'action': 'acquire'}); check('second principal cannot acquire (409 lock_held)', c == 409 and d['error']['code'] == 'lock_held')
    c, d = create(OP, 'w1'); check('web mission is REJECTED with 409 ownership_conflict (not queued)', c == 409 and d['error']['code'] == 'ownership_conflict', str(d)[:140])
    check('no mission was created by the rejected request', all(m['idempotency_key'] != 'w1' for m in http('GET', '/missions?robot_id=pcu', VW)[1]['missions']))
    c, d = create(RMF, 'r1'); check('the owner can create a mission', c == 201, str(c)); rm = d
    c, d = create(RMF, 'r1'); check('owner replay with the same key returns the same mission (idempotent)', c == 200 and d['mission_id'] == rm['mission_id'])
    c, d = create(OP, 'r1'); check('idempotent replay of an existing key is still answered for others (compat)', c == 200 and d['mission_id'] == rm['mission_id'])
    c, d = http('POST', f"/missions/{rm['mission_id']}/cancel", OP, {}); check('any control role may still cancel (stop is the safe direction)', c == 202, str(c))
    c, d = http('POST', '/robots/pcu/lock', VW, {'action': 'acquire'}); check('viewer cannot touch leases (403)', c == 403)
    c, d = http('POST', '/robots/pcu/lock', agent_t := ag.t, {'action': 'acquire'}); check('agent token cannot touch leases (403)', c == 403)
    c, d = http('POST', '/robots/pcu/lock', OP, {'action': 'release'}); check('non-owner operator cannot release (409)', c == 409)
    c, d = http('POST', '/robots/pcu/lock', OP, {'action': 'force_release'}); check('operator cannot force-release (403)', c == 403)
    c, d = http('POST', '/robots/pcu/lock', RMF, {'action': 'acquire', 'ttl_s': 1}); check('bad ttl refused (400)', c == 400)
    c, d = http('GET', '/robots/pcu', VW); check('robot view shows the lease', d.get('lease', {}).get('owner') == 'rmf-adapter')
    print('== lease expiry (owner stopped renewing)')
    http('POST', '/robots/pcu/lock', RMF, {'action': 'acquire', 'ttl_s': 5}); time.sleep(5.6)
    c, d = http('GET', '/robots/pcu/lock', VW); check('lease expired without renewal', c == 200 and d['lease'] is None)
    c, d = http('POST', '/robots/pcu/lock', RMF, {'action': 'renew'}); check('renew after expiry is refused (409 lock_not_held): must re-acquire', c == 409 and d['error']['code'] == 'lock_not_held')
    http('POST', '/robots/pcu/lock', RMF, {'action': 'acquire'})
    c, d = http('POST', '/robots/pcu/lock', A, {'action': 'force_release', 'note': 'x'}); check('admin can force-release', c == 200 and d['released'])
    c, d = create(OP, 'w2'); check('after the release the web can create again', c == 201)
    ag.take_goal(); finish_goal(ag)

    print('== race: web create vs RMF acquire, 120 rounds (never both succeed)')
    both = neither = 0; web_won = rmf_won = 0
    for i in range(120):
        http('POST', '/robots/pcu/lock', A, {'action': 'force_release'})
        for m in http('GET', '/missions?robot_id=pcu&limit=500', VW)[1]['missions']:
            if not m['terminal']: http('POST', f"/missions/{m['mission_id']}/cancel", A, {})      # nothing was delivered (agent silent): QUEUED -> CANCELED at once
        res = {}; ev = threading.Event()
        def w(): ev.wait(); res['web'] = create(OP, f'race-w-{i}')[0]
        def r():
            ev.wait(); c1 = http('POST', '/robots/pcu/lock', RMF, {'action': 'acquire', 'ttl_s': 30})[0]; res['rmf'] = c1
            if c1 == 200: res['rmf_create'] = create(RMF, f'race-r-{i}')[0]
        ts = [threading.Thread(target=w), threading.Thread(target=r)]
        if i % 2: ts.reverse()
        [t.start() for t in ts]; ev.set(); [t.join() for t in ts]
        web_ok = res['web'] == 201; rmf_ok = res['rmf'] == 200
        if web_ok and rmf_ok: both += 1; print('   BOTH OK round', i, res)
        elif not web_ok and not rmf_ok: neither += 1
        web_won += web_ok; rmf_won += rmf_ok
        live = {m['created_by'] for m in http('GET', '/missions?robot_id=pcu&limit=500', VW)[1]['missions'] if not m['terminal']}
        if len(live) > 1: both += 1; print('   MULTI-OWNER live round', i, live, res)
        if rmf_ok and res.get('rmf_create') != 201: both += 1; print('   owner could not create', res)
    check('120 rounds: web and RMF never both obtained control', both == 0, f'both={both} neither={neither} web_won={web_won} rmf_won={rmf_won}')
    check('both orders happened or one side always lost cleanly (every round had exactly one winner)', neither == 0 and web_won + rmf_won == 120, f'web={web_won} rmf={rmf_won}')
    http('POST', '/robots/pcu/lock', A, {'action': 'force_release'})
    # settle: cancel everything, end goals
    for m in http('GET', '/missions?robot_id=pcu&limit=500', VW)[1]['missions']:
        if not m['terminal']: http('POST', f"/missions/{m['mission_id']}/cancel", A, {})
    for _ in range(10):
        d = ag.post_state() or {}
        for c_ in d.get('commands', []):
            if c_['command_id'] not in ag.received:
                ag.received.append(c_['command_id']); ag.ack(c_['command_id'], 'CANCELED' if c_['type'] == 'navigate_to_pose' else 'SUCCEEDED')
        time.sleep(0.1)

    print('== operator recovery of RECONCILIATION_REQUIRED')
    ag = FakeAgent('pcu2'); ag.hello(); ag.post_state()
    c, d = create(OP, 'rec1', robot='pcu2'); mid = d['mission_id']; g = ag.take_goal(); check('goal delivered to the fake agent', g is not None)
    ag = FakeAgent('pcu2'); ag.nav2_active = 1; ag.hello(); ag.post_state()           # agent restarted: new boot, lost the handle, Nav2 still has the goal
    m = http('GET', f'/missions/{mid}', VW)[1]; check('agent restart -> mission RECONCILIATION_REQUIRED (no resend)', m['status'] == 'RECONCILIATION_REQUIRED', m['status'])
    c, d = create(OP, 'rec2', robot='pcu2'); check('a new movement mission may be queued but...', c == 201); q = d['mission_id']
    time.sleep(1.5); ag.post_state(); check('...it is not dispatched while reconciliation is required', http('GET', f'/missions/{q}', VW)[1]['status'] == 'QUEUED')
    c, d = http('POST', f'/missions/{mid}/recover', OP, {'note': 'checked on the robot ok'}); check('operator role cannot recover (403)', c == 403)
    c, d = http('POST', f'/missions/{mid}/recover', A, {'note': 'short'}); check('recovery needs a meaningful note (400)', c == 400)
    c, d = http('POST', f'/missions/{mid}/recover', A, {'note': 'looked at the robot, goal may still run'}); check('refused while Nav2 still reports an ACTIVE goal (409 goal_still_active)', c == 409 and d['error']['code'] == 'goal_still_active', str(d)[:120])
    ag.nav2_known = False; ag.post_state()
    c, d = http('POST', f'/missions/{mid}/recover', A, {'note': 'robot has stopped, I checked it'}); check('refused when the agent cannot report goal status (409 no_evidence)', c == 409 and d['error']['code'] == 'no_evidence', str(d)[:120])
    ag.nav2_known = True; ag.nav2_active = 0; ag.post_state(); time.sleep(0.2)
    # stale evidence: do not post for > EVIDENCE_MAX_AGE_S
    time.sleep(5.5)
    c, d = http('POST', f'/missions/{mid}/recover', A, {'note': 'robot has stopped, I checked it'}); check('refused when the evidence is stale (409 stale_state)', c == 409 and d['error']['code'] in ('stale_state', 'robot_not_ready'), str(d)[:120])
    ag.post_state()
    c, d = http('POST', f'/missions/{mid}/recover', A, {'note': 'robot has stopped, I checked it'})
    check('admin recovery succeeds with fresh proof that Nav2 has no active goal', c == 200, str(d)[:160])
    m = http('GET', f'/missions/{mid}', VW)[1]
    check('the unknown mission is NOT reported as SUCCEEDED/CANCELED: FAILED + RECOVERED_UNVERIFIED_OUTCOME', m['status'] == 'FAILED' and 'RECOVERED_UNVERIFIED_OUTCOME' in (m['detail'] or ''), f"{m['status']} {m['detail'][:60]}")
    c, d = http('POST', f'/missions/{mid}/recover', A, {'note': 'robot has stopped, I checked it'}); check('recovering twice is refused (409 not_recoverable)', c == 409 and d['error']['code'] == 'not_recoverable')
    d = ag.post_state() or {}
    got = [c_ for c_ in d.get('commands', []) if c_['mission_id'] == q]
    check('only now the queued mission is dispatched (and nothing was replayed for the recovered one)', len(got) == 1 and not [c_ for c_ in d['commands'] if c_['mission_id'] == mid], str(d.get('commands'))[:100])
    ev = http('GET', '/events?limit=100', VW)[1]['events']; check('recovery is recorded in the event log', any(e['kind'] == 'mission_recovered' for e in ev))
finally:
    for p in procs:
        try: p.send_signal(signal.SIGINT); p.wait(8)
        except Exception: p.kill()
    n = sum(R); print(f'\nTOTAL {n}/{len(R)} passed (logs {T})'); sys.exit(0 if n == len(R) else 1)
