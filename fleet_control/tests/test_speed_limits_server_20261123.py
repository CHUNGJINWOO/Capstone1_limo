#!/usr/bin/env python3
"""Server speed-limit API against the REAL Agent-side limiter (G2, 2026-11-23). No ROS: a scripted fake Agent speaks the Agent HTTP protocol, reports
the real SpeedLimiter snapshot and feeds the delivered request to the real SpeedLimiter.process(). usage: python3 tests/test_speed_limits_server_20261123.py"""

import hashlib, json, os, signal, subprocess, sys, tempfile, threading, time, urllib.error, urllib.request, uuid
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
PORT = int(os.environ.get('SL_TEST_PORT', '18091')); BASE = f'http://127.0.0.1:{PORT}'
T = tempfile.mkdtemp(prefix='sl_test_'); DATA, SEC = os.path.join(T, 'data'), os.path.join(T, 'sec')
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


sys.path[:0] = [os.path.join(ROOT, 'agent', 'limo_robot_agent'), os.path.join(ROOT, 'tests')]
import test_agent_safety_v1 as _stub          # stub rclpy for the Agent package import
from limo_robot_agent.speed_limits import SpeedLimiter


class FakeAgent:
    def __init__(self, rid='pcu', enforce=True):
        self.rid, self.boot, self.seq, self.t = rid, uuid.uuid4().hex[:10], 0, tok('agent-' + rid)
        self.limiter = SpeedLimiter(); self.limiter.enforced_by = enforce if isinstance(enforce, str) else ('ARBITER' if enforce else None)
        self.server_boot = None; self.lease = None; self.last_reply = None

    def hello(self):
        c, d = http('POST', '/agent/hello', self.t, {'robot_id': self.rid, 'agent': {'protocol_version': 2, 'safety_contract_version': 1, 'boot_id': self.boot,
                                                                                    'mode': 'mock', 'simulated': True, 'missions_enabled': False}})
        self.server_boot = (d or {}).get('server_boot_id')
        return http('POST', '/agent/reconcile', self.t, {'robot_id': self.rid, 'agent_boot_id': self.boot, 'journal_ok': True, 'journal': {}, 'execution': {'active': None}})

    def post_state(self, apply=True):
        self.seq += 1
        st = {'schema_version': 1, 'speed_limits': self.limiter.snapshot()}
        c, d = http('POST', '/agent/state', self.t, {'robot_id': self.rid, 'agent_boot_id': self.boot, 'seq': self.seq, 'state': st}); d = d or {}
        self.last_reply = d; self.lease = d.get('lease')
        sess = {'connected': True, 'reconciliation_required': False, 'server_boot_id': self.server_boot, 'agent_boot_id': self.boot, 'lease': self.lease,
                'protocol_version': 2, 'safety_contract_version': 1, 'contract_generation': 1, 'speed_limit_request': d.get('speed_limit_request')}
        if apply: self.limiter.process(sess)
        return d


def body(ag, epoch, **kw):
    b = {'lease_epoch': epoch, 'agent_boot_id': ag.boot, 'protocol_version': 2, 'safety_contract_version': 1, 'max_linear_mps': 0.12, 'max_angular_rps': 0.35}
    b.update(kw); return {k: v for k, v in b.items() if v is not None}


def epoch_of(): return ((http('GET', '/robots/pcu/lock', tok('viewer'))[1] or {}).get('lease') or {}).get('epoch')


procs = []
try:
    env = dict(os.environ, PYTHONPATH=os.path.join(ROOT, 'server'))
    subprocess.run([sys.executable, '-m', 'fleet_server', 'init', '--data-dir', DATA, '--secrets-dir', SEC, '--robots', 'pcu', 'pcu2'], cwd=os.path.join(ROOT, 'server'), env=env, check=True, capture_output=True)
    add_token('rmf-adapter', 'operator')
    procs.append(subprocess.Popen([sys.executable, '-m', 'fleet_server', 'serve', '--data-dir', DATA, '--port', str(PORT), '--offline-after', '30'], cwd=os.path.join(ROOT, 'server'), env=env, stdout=open(os.path.join(T, 'srv.log'), 'w'), stderr=subprocess.STDOUT))
    for _ in range(50):
        try: urllib.request.urlopen(BASE + '/api/v1/health', timeout=1); break
        except Exception: time.sleep(0.2)
    A, OP, RMF, VW = tok('admin'), tok('operator'), tok('rmf-adapter'), tok('viewer')
    ag = FakeAgent(); ag.hello(); ag.post_state()
    P = '/robots/pcu/speed-limits'

    print('== authorization and binding')
    c, d = http('POST', P, OP, body(ag, 1)); check('no lease: refused 403 not_lease_owner', c == 403 and d['error']['code'] == 'not_lease_owner', str(d)[:100])
    c, d = http('POST', '/robots/pcu/lock', OP, {'action': 'acquire', 'ttl_s': 120}); ep = d['epoch']; ag.post_state()
    c, d = http('POST', P, RMF, body(ag, ep)); check('another operator (not the owner): 403', c == 403 and d['error']['code'] == 'not_lease_owner')
    c, d = http('POST', P, VW, body(ag, ep)); check('viewer: refused', c in (401, 403), str(c))
    for name, b, code, status in (('wrong lease_epoch', body(ag, ep + 5), 'lease_epoch_mismatch', 409), ('missing lease_epoch', body(ag, None), 'lease_epoch_mismatch', 409),
                                  ('wrong agent_boot_id', body(ag, ep, agent_boot_id='nope'), 'agent_boot_mismatch', 409), ('protocol_version 1', body(ag, ep, protocol_version=1), 'agent_protocol', 409),
                                  ('safety_contract_version 2', body(ag, ep, safety_contract_version=2), 'agent_protocol', 409), ('bad scope', body(ag, ep, scope='fast'), 'bad_scope', 400),
                                  ('negative', body(ag, ep, max_linear_mps=-0.05), 'bad_value', 400), ('zero', body(ag, ep, max_angular_rps=0), 'bad_value', 400),
                                  ('string', body(ag, ep, max_linear_mps='0.1'), 'bad_value', 400), ('bool', body(ag, ep, max_linear_mps=True), 'bad_value', 400),
                                  ('missing value', body(ag, ep, max_angular_rps=None), 'bad_value', 400),
                                  ('above the Agent ceiling (0.20 m/s)', body(ag, ep, max_linear_mps=0.20), 'speed_exceeds_ceiling', 422),
                                  ('above the Agent ceiling (0.6 rad/s)', body(ag, ep, max_angular_rps=0.6), 'speed_exceeds_ceiling', 422),
                                  ('manual above 0.10', body(ag, ep, scope='manual', max_linear_mps=0.12), 'speed_exceeds_ceiling', 422),
                                  ('below the minimum', body(ag, ep, max_linear_mps=0.005), 'speed_below_minimum', 422)):
        c, d = http('POST', P, OP, b); check(f'{name}: {status} {code}', c == status and d['error']['code'] == code, f'{c} {str(d)[:100]}')
    c, d = http('POST', P, OP, dict(body(ag, ep), max_linear_mps=float('nan'))); check('NaN: refused (not accepted as a number)', c in (400, 422), str(c))
    check('nothing was requested by any refusal', http('GET', P, VW)[1]['generation'] == 0)
    ev = http('GET', '/events?limit=200', VW)[1]['events']; check('refusals are audited', sum(e['kind'] == 'speed_limit_refused' for e in ev) >= 10)

    print('== request, pending, Agent confirmation')
    c, d = http('POST', P, OP, body(ag, ep, client_request_id='c1')); g1 = d.get('generation')
    check('valid request: 202, PENDING, approved=false, generation 1', c == 202 and d['status'] == 'PENDING' and d['approved'] is False and g1 == 1, str(d)[:160])
    c, d = http('GET', P, VW); check('GET before the Agent applied: approved=false, no applied value', d['approved'] is False and d['applied'] is None and d['status'] == 'PENDING')
    c, d = http('POST', P, OP, body(ag, ep, client_request_id='c1')); check('same client_request_id + values: duplicate, same generation', c in (200, 202) and d.get('duplicate') is True and d['generation'] == 1)
    c, d = http('POST', P, OP, body(ag, ep, client_request_id='c1', max_linear_mps=0.1)); check('same client_request_id + other values: 409 idempotency_conflict', c == 409 and d['error']['code'] == 'idempotency_conflict')
    ag.post_state(apply=False)
    check('the pending request is delivered with the Agent boot, lease epoch and generation', (ag.last_reply.get('speed_limit_request') or {}).get('generation') == 1 and ag.last_reply['speed_limit_request']['agent_boot_id'] == ag.boot)
    ag.post_state(); c, d = http('GET', P, VW)
    check('after the Agent applied it but before it reported: still approved=false', d['approved'] is False, str(d['status']))
    ag.post_state(); c, d = http('GET', P, VW)
    check('after the Agent REPORTED the same generation: APPLIED, approved=true', d['approved'] is True and d['status'] == 'APPLIED' and d['applied']['linear_mps'] == 0.12 and d['effective']['auto']['linear_mps'] == 0.12, str(d)[:200])
    check('the response carries requested, applied, ceiling, generation, reason, source and report age', all(k in d for k in ('requested', 'applied', 'ceiling', 'generation', 'reason', 'source', 'agent_report_age_s')))
    check('the Agent limiter really clamps now', ag.limiter.clamp('auto', 0.15, 0.5) == (0.12, 0.35))

    print('== generations, supersede, restore')
    c, d = http('POST', P, OP, body(ag, ep, expected_generation=7)); check('stale expected_generation: 409', c == 409 and d['error']['code'] == 'stale_generation')
    c, d = http('POST', P, OP, body(ag, ep, max_linear_mps=0.09, max_angular_rps=0.3, expected_generation=1)); g2 = d['generation']
    c, d = http('POST', P, OP, body(ag, ep, max_linear_mps=0.07, max_angular_rps=0.25)); g3 = d['generation']
    check('a newer request supersedes the pending one (generations increase)', g2 == 2 and g3 == 3)
    ag.post_state(); ag.post_state(); d = http('GET', P, VW)[1]
    check('only the newest generation is applied by the Agent', d['approved'] is True and d['generation'] == 3 and ag.limiter.limits('auto')[0] == 0.07, str(d)[:140])
    c, d = http('POST', P, OP, body(ag, ep, max_linear_mps=0.15, max_angular_rps=0.5)); ag.post_state(); ag.post_state(); d = http('GET', P, VW)[1]
    check('raising back up to the ceiling is an explicit new request and works while the lease is held', d['approved'] is True and ag.limiter.limits('auto')[0] == 0.15)
    c, d = http('POST', P, OP, dict(body(ag, ep, scope='manual', max_linear_mps=0.06, max_angular_rps=0.2))); ag.post_state(); ag.post_state()
    check('manual scope is separate (auto stays 0.15)', ag.limiter.limits('manual')[0] == 0.06 and ag.limiter.limits('auto')[0] == 0.15)

    print('== lease loss, Agent restart, enforcement point')
    c, d = http('POST', P, OP, body(ag, ep, max_linear_mps=0.05, max_angular_rps=0.2)); g_pending = d['generation']
    http('POST', '/robots/pcu/lock', OP, {'action': 'release'}); ag.post_state(apply=False)
    d = http('GET', P, VW)[1]; check('lease released while PENDING: EXPIRED BINDING_CHANGED, approved=false', d['status'] == 'EXPIRED' and d['reason'] == 'BINDING_CHANGED' and d['approved'] is False, str(d)[:140])
    ag.post_state(apply=True); check('the Agent keeps the previous (lower) limit, marked stale; nothing raised it', ag.limiter.stale and ag.limiter.limits('auto')[0] == 0.15 and ag.limiter.limits('manual')[0] == 0.06)
    http('POST', '/robots/pcu/lock', OP, {'action': 'acquire', 'ttl_s': 120}); ep2 = epoch_of(); ag.post_state()
    c, d = http('POST', P, OP, body(ag, ep)); check('the old lease epoch cannot be reused: 409', c == 409 and d['error']['code'] == 'lease_epoch_mismatch')
    d = http('GET', P, VW)[1]; check('after lease loss approved=false until a new request is applied', d['approved'] is False)
    c, d = http('POST', P, OP, body(ag, ep2, max_linear_mps=0.1, max_angular_rps=0.3)); ag.post_state(); ag.post_state(); d = http('GET', P, VW)[1]
    check('new lease + new request: applied, generation continues (not reset)', d['approved'] is True and d['generation'] > g_pending, str(d['generation']))
    old_boot = ag.boot; ag2 = FakeAgent(); ag2.rid = 'pcu'; ag2.t = ag.t; ag2.hello(); ag2.post_state()
    c, d = http('POST', P, OP, body(ag, ep2)); check('after an Agent restart (new boot) the old agent_boot_id is refused: 409', c == 409 and d['error']['code'] == 'agent_boot_mismatch')
    d = http('GET', P, VW)[1]; check('after the restart the new Agent has no limit applied: approved=false, ceilings reported', d['approved'] is False and d['ceiling'] is not None)
    ag3 = FakeAgent('pcu2', enforce=False); ag3.hello(); ag3.post_state(); http('POST', '/robots/pcu2/lock', OP, {'action': 'acquire', 'ttl_s': 60}); ag3.post_state()
    e3 = ((http('GET', '/robots/pcu2/lock', VW)[1] or {}).get('lease') or {}).get('epoch')
    c, d = http('POST', '/robots/pcu2/speed-limits', OP, body(ag3, e3)); check('an Agent without an enforcement point: 409 agent_cannot_enforce (nothing queued)', c == 409 and d['error']['code'] == 'agent_cannot_enforce', str(d)[:100])
    print('== Watchdog enforcement: approved only after the Watchdog confirmed AND the server verified its evidence')
    ag4 = FakeAgent('pcu2', enforce='WATCHDOG'); ag4.hello(); ag4.post_state()
    gate = {'confirmed': False, 'wd_applied': {}, 'age': 0.2, 'boot': None}
    ag4.limiter.confirm_fn = lambda cand: gate['confirmed']
    def evidence():
        return {'watchdog_boot_id': 'wdboot1', 'agent_boot_id': gate['boot'] or ag4.boot, 'output_gid': '010f7f01aa', 'synced': True, 'publisher_ok': True, 'fault': None,
                'enabled': True, 'evidence_age_s': gate['age'], 'applied': gate['wd_applied'], 'agent_verified': True}
    def post4():
        ag4.limiter.watchdog_evidence = evidence(); return ag4.post_state()
    e4 = ((http('GET', '/robots/pcu2/lock', VW)[1] or {}).get('lease') or {}).get('epoch'); post4()
    c, d = http('POST', '/robots/pcu2/speed-limits', OP, body(ag4, e4)); check('enforced_by WATCHDOG is accepted as an enforcement point (202, PENDING)', c == 202 and d['approved'] is False, str(d)[:120])
    post4(); post4(); d = http('GET', '/robots/pcu2/speed-limits', VW)[1]
    check('Agent validated it but the Watchdog has not confirmed: PENDING_ENFORCEMENT is NOT approved and not rejected', d['status'] == 'PENDING' and d['approved'] is False and ag4.limiter.limits('auto')[0] == 0.15, str(d)[:140])
    gate['confirmed'] = True; post4(); post4(); d = http('GET', '/robots/pcu2/speed-limits', VW)[1]
    check('the Agent says APPLIED but the Watchdog evidence does not list that generation: the server does NOT approve', d['approved'] is False and d['status'] == 'PENDING' and d['watchdog_evidence']['applied_match'] is not True and d['watchdog_evidence']['applied'] == {}, str(d['watchdog_evidence'])[:200])
    gen = ag4.limiter.generation; rid = ag4.limiter.last_result['request_id']
    gate['wd_applied'] = {'auto': {'generation': gen, 'request_id': rid, 'linear_mps': 0.12, 'angular_rps': 0.35, 'stale': False}}; post4(); post4(); d = http('GET', '/robots/pcu2/speed-limits', VW)[1]
    check('with matching, fresh Watchdog evidence: APPLIED and approved=true, server_verified computed by the server', d['approved'] is True and d['status'] == 'APPLIED' and d['enforced_by'] == 'WATCHDOG' and d['watchdog_evidence']['server_verified'] is True and d['watchdog_evidence']['applied_match'] is True, str(d)[:200])
    check('the response carries robot_id, agent_boot_id, watchdog_boot_id, request_id, generation, scope', d['robot_id'] == 'pcu2' and d['agent_boot_id'] == ag4.boot and d['watchdog_boot_id'] == 'wdboot1' and d['request_id'] and d['generation'] >= 1 and d['scope'] == 'auto')
    st4 = http('GET', '/robots/pcu2', VW)[1]['state']['speed_limits']['watchdog_evidence']
    check('the console state carries the server verdict next to the Agent evidence', st4['server_verified'] is True and st4['agent_verified'] is True and st4['watchdog_boot_id'] == 'wdboot1')
    for name, mut in (('stale evidence age', lambda: gate.update(age=5.0)), ('Agent boot in the evidence differs', lambda: gate.update(boot='other', age=0.2)),
                      ('Watchdog lists an older generation', lambda: gate.update(boot=None, wd_applied={'auto': {'generation': gen - 1 if gen > 1 else 0, 'request_id': rid, 'linear_mps': 0.12, 'angular_rps': 0.35, 'stale': False}})),
                      ('Watchdog value differs', lambda: gate.update(wd_applied={'auto': {'generation': gen, 'request_id': rid, 'linear_mps': 0.15, 'angular_rps': 0.35, 'stale': False}})),
                      ('Watchdog entry stale (restart restore)', lambda: gate.update(wd_applied={'auto': {'generation': gen, 'request_id': rid, 'linear_mps': 0.12, 'angular_rps': 0.35, 'stale': True}}))):
        mut(); post4(); d = http('GET', '/robots/pcu2/speed-limits', VW)[1]
        check(f'{name}: approved=false', d['approved'] is False, str(d['watchdog_evidence']['server_checks'])[:140])
        if name.startswith('Watchdog'):
            check(f'{name}: the console can still see a verified, working enforcement point (server_verified stays true so a NEW request can be made)', d['watchdog_evidence']['server_verified'] is True and d['watchdog_evidence']['applied_match'] is False)
        gate.update(age=0.2, boot=None, wd_applied={'auto': {'generation': gen, 'request_id': rid, 'linear_mps': 0.12, 'angular_rps': 0.35, 'stale': False}}); post4()
    d = http('GET', '/robots/pcu2/speed-limits', VW)[1]; check('evidence restored: approved again (no sticky state in the view)', d['approved'] is True)
    # regression (found in the full-stack E2E): evidence present while enforced_by is not WATCHDOG (e.g. right after an Agent restart) must not break state ingestion
    ag4.limiter.enforced_by = None; ag4.limiter.watchdog_evidence = evidence(); c = ag4.seq + 1
    code = http('POST', '/agent/state', ag4.t, {'robot_id': 'pcu2', 'agent_boot_id': ag4.boot, 'seq': c, 'state': {'schema_version': 1, 'speed_limits': ag4.limiter.snapshot()}})[0]; ag4.seq = c
    check('state ingestion with stale Watchdog evidence and enforced_by null answers 200 (was HTTP 500)', code == 200, str(code))
    d = http('GET', '/robots/pcu2/speed-limits', VW)[1]; check('... and nothing is approved without an enforcement point', d['approved'] is False)
    ag4.limiter.enforced_by = 'WATCHDOG'; post4()
    time.sleep(0.2)
    print('== server restart keeps the generation counter')
    srv = procs[0]; srv.send_signal(signal.SIGINT); srv.wait(8)
    procs[0] = subprocess.Popen([sys.executable, '-m', 'fleet_server', 'serve', '--data-dir', DATA, '--port', str(PORT), '--offline-after', '30'], cwd=os.path.join(ROOT, 'server'), env=env, stdout=open(os.path.join(T, 'srv2.log'), 'w'), stderr=subprocess.STDOUT)
    for _ in range(50):
        try: urllib.request.urlopen(BASE + '/api/v1/health', timeout=1); break
        except Exception: time.sleep(0.2)
    d = http('GET', P, VW)[1]; check('after a server restart the generation is kept and approved is false until the Agent reconnects and reports', d['generation'] >= 5 and d['approved'] is False, str(d['generation']))
except BaseException:
    import traceback; traceback.print_exc()
    check('test script completed without an exception (aborted run is a FAIL)', False)
finally:
    for p in procs:
        try: p.send_signal(signal.SIGINT); p.wait(8)
        except Exception: p.kill()
    n = sum(R); print(f'\nTOTAL {n}/{len(R)} passed (logs {T})'); sys.exit(0 if n == len(R) else 1)
