#!/usr/bin/env python3
"""Server disconnection / reconnection against the live Watchdog-enforced isolated stack (G2/integration, 2026-11-23). Not a real-robot result.
usage: e2e_server_restart_speed_limit_20261123.py RUN_DIR PORT   (stack started like e2e_web_watchdog_speed_limit_20261123.py)

The Fleet Server is frozen (SIGSTOP) for 25 s and then resumed: the Agent, Watchdog and Gate keep running and see an unreachable server, then a reachable
one with the same boot id. (A real stop/restart of the server cannot be done inside scripts/run_isolated_fleet.sh: the harness tears the whole stack down when
its server child exits, which is how this test first failed; server restarts are covered by test_phase20_http_recovery and the server API test.)
Checks that the applied 0.06 limit survives the outage, nothing raises it, the old approval turns STALE (a connection loss is a binding loss) and only an explicit new request re-establishes it."""
import json, os, signal, subprocess, sys, time, uuid
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def main():
    run, port = Path(sys.argv[1]), int(sys.argv[2])
    token = (run / 'secrets/operator.token').read_text().strip()
    base = f'http://127.0.0.1:{port}/api/v1/robots/locked_bench'
    res = []

    def check(name, ok, detail=''):
        res.append(bool(ok)); print(('PASS ' if ok else 'FAIL ') + name + (f' | {detail}' if detail else ''), flush=True)

    def call(path, data=None, timeout=4):
        req = Request(base + path, None if data is None else json.dumps(data).encode(), headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
        try:
            with urlopen(req, timeout=timeout) as r:
                return r.status, json.loads(r.read())
        except HTTPError as e:
            return e.code, json.loads(e.read() or b'{}')
        except Exception as e:
            return 0, {'error': str(e)}

    def wait(pred, secs=30, step=.5):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            v = pred()
            if v: return v
            time.sleep(step)
        return None

    def renew(): call('/lock', dict(action='renew', ttl_s=300))
    def approved(gen):
        def f():
            renew(); w = call('/speed-limits')[1]; return w if w.get('approved') is True and w.get('generation') == gen else None
        return wait(f, 40)

    wait(lambda: call('')[1].get('online') and call('')[1].get('reconciled'), 60)
    lease = call('/lock', dict(action='acquire', ttl_s=300))[1]; ep = lease['epoch']; rb = call('')[1]
    c, v = call('/speed-limits', dict(lease_epoch=ep, agent_boot_id=rb['agent']['boot_id'], protocol_version=2, safety_contract_version=1, scope='auto', max_linear_mps=0.06, max_angular_rps=0.2, client_request_id=uuid.uuid4().hex[:12]))
    gen = v.get('generation')
    check('S1 0.06 / 0.20 limit approved before the outage', approved(gen) is not None)
    pid = [int(l.split(None, 1)[0]) for l in subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True).stdout.splitlines() if 'fleet_server serve' in l and str(port) in l][0]
    os.kill(pid, signal.SIGSTOP); t0 = time.monotonic()
    try:
        time.sleep(25.0)
        check('S2 while the server is frozen the console API is unreachable (nothing can be raised)', call('/speed-limits', timeout=3)[0] == 0)
    finally:
        os.kill(pid, signal.SIGCONT)
    back = wait(lambda: (lambda r: r if r.get('online') and r.get('reconciled') else None)(call('')[1]), 90)
    check('S3 after the outage the Agent is online and reconciled again', back is not None, f'{time.monotonic() - t0:.1f} s after the freeze')
    sl = (call('')[1].get('state') or {}).get('speed_limits') or {}
    check('S4 the Agent still enforces 0.06 (nothing raised it during or after the outage)', ((sl.get('effective') or {}).get('auto') or {}).get('linear_mps') == 0.06, json.dumps(sl.get('effective')))
    w = call('/speed-limits')[1]
    check('S5 a connection loss makes the setting STALE: approved=false, stale=true (the old approval is not shown as current), effective still 0.06',
          w.get('approved') is False and w.get('stale') is True and ((w.get('effective') or {}).get('auto') or {}).get('linear_mps') == 0.06, json.dumps({k: w.get(k) for k in ('approved', 'status', 'stale', 'stale_reason')}))
    renew(); rb = call('')[1]
    c, v2 = call('/speed-limits', dict(lease_epoch=ep, agent_boot_id=rb['agent']['boot_id'], protocol_version=2, safety_contract_version=1, scope='auto', max_linear_mps=0.06, max_angular_rps=0.2, client_request_id=uuid.uuid4().hex[:12]))
    w2 = approved(v2.get('generation'))
    check('S6 an explicit new request re-establishes the approval with fresh evidence (generation continues)', w2 is not None and v2.get('generation', 0) > gen and w2['watchdog_evidence']['applied_match'] is True, json.dumps({k: (w2 or {}).get(k) for k in ('approved', 'status', 'generation')}))
    n = sum(res); print(f'{n}/{len(res)} PASS', flush=True); os._exit(0 if n == len(res) else 1)


if __name__ == '__main__':
    main()
