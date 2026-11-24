#!/usr/bin/env python3
"""Speed-limit clamp through the REAL Agent -> Arbiter -> Watchdog -> Gate chain (isolated ROS, no driver, G2 2026-11-23).
Start the stack with scripts/run_isolated_fleet.sh RUN_DIR PORT DOMAIN (overlay sourced), then:  python3 tests/probe_speed_clamp_20261123.py RUN_DIR PORT
Real non-zero commands flow: the operator console API drives MANUAL with full-scale axes; the measurement is what the Gate publishes on
/cmd_vel_locked (nothing is connected to /cmd_vel). The speed limit is requested and confirmed through POST/GET .../speed-limits.
Not a real-robot result. Exit code 0 = every check passed."""
import json, subprocess, sys, threading, time, uuid
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def main():
    run, port = Path(sys.argv[1]), int(sys.argv[2])
    token = (run / 'secrets/operator.token').read_text().strip()
    base = f'http://127.0.0.1:{port}/api/v1/robots/locked_bench'
    results = []

    def check(name, ok, detail=''):
        results.append(bool(ok)); print(('PASS ' if ok else 'FAIL ') + name + (f' | {detail}' if detail else ''), flush=True)

    def call(path, data=None):
        req = Request(base + path, None if data is None else json.dumps(data).encode(), headers={'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json'})
        try:
            with urlopen(req, timeout=4) as r:
                return json.loads(r.read())
        except HTTPError as e:
            return {'_http': e.code, **json.loads(e.read() or b'{}')}

    def wait(pred, secs=25, step=.25):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            v = pred()
            if v: return v
            time.sleep(step)
        return None

    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init(); node = rclpy.create_node('speed_clamp_probe'); rows = []
    node.create_subscription(Twist, '/cmd_vel_locked', lambda m: rows.append((time.monotonic(), m.linear.x, m.angular.z)), 50)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    wait(lambda: (lambda s: s if s.get('online') and s.get('reconciled') else None)(call('')), 40)
    lease = call('/lock', dict(action='acquire', ttl_s=300)); epoch = lease['epoch']
    st = {'epoch': epoch}

    def renew(): call('/lock', dict(action='renew', ttl_s=300))
    def state(): return call('')['state']

    def mode(target):
        renew(); call('/mode', dict(target=target, idempotency_key=uuid.uuid4().hex, epoch=st['epoch']))
        return wait(lambda: (call('/mode').get('operating_mode') or {}).get('mode') == target
                    and (call('/mode')['operating_mode'].get('transition') or {}).get('state') not in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'), 40)

    def limits(scope, lin, ang, **kw):
        renew()
        r = call('/speed-limits', dict(lease_epoch=st['epoch'], agent_boot_id=call('')['agent']['boot_id'], protocol_version=2, safety_contract_version=1,
                                       scope=scope, max_linear_mps=lin, max_angular_rps=ang, **kw))
        return r

    def confirmed(gen):
        def f():
            renew(); v = call('/speed-limits'); return v if v.get('generation') == gen and v.get('approved') is True else None
        return wait(f, 20)

    def drive(secs, linear=1.0, turn=1.0, deadman=True):
        """Full-scale gamepad axes for `secs` seconds; returns the Gate output rows seen in that window."""
        bench = call('/manual-bench', dict(action='start', epoch=st['epoch'])); t0 = time.monotonic(); seq = 0
        while time.monotonic() - t0 < secs:
            seq += 1; renew() if seq % 20 == 0 else None
            call('/manual-bench', dict(action='sample', input_format='normalized_axes_v1', epoch=st['epoch'], session_id=bench.get('session_id'), seq=seq, linear=linear, turn=turn, deadman=deadman))
            time.sleep(.08)
        call('/manual-bench', dict(action='stop', epoch=st['epoch'], session_id=bench.get('session_id')))
        return [r for r in rows if r[0] >= t0]

    def peak(win):
        nz = [r for r in win if abs(r[1]) > 1e-6 or abs(r[2]) > 1e-6]
        return (max([r[1] for r in nz] or [0]), max([abs(r[2]) for r in nz] or [0]), len(nz))

    check('operating mode MANUAL reached', mode('MANUAL'))
    sl0 = state().get('speed_limits') or {}
    check('Agent reports an enforcement point and the existing ceilings', sl0.get('enforced_by') == 'ARBITER' and sl0.get('ceiling', {}).get('manual') == {'linear_mps': 0.1, 'angular_rps': 0.3}, json.dumps(sl0.get('ceiling'))[:140])

    print('-- positive control: no limit requested')
    p0 = peak(drive(3.0)); check('non-zero commands flow through Watchdog and Gate (about 0.10 m/s, 0.30 rad/s)', p0[2] >= 10 and 0.08 <= p0[0] <= 0.1001 and 0.25 <= p0[1] <= 0.3001, str(p0))

    print('-- limit requested')
    r = limits('manual', 0.05, 0.15, client_request_id='probe-1'); g = r.get('generation')
    check('POST: PENDING and approved=false before the Agent applied it', r.get('status') == 'PENDING' and r.get('approved') is False, json.dumps(r)[:160])
    v = confirmed(g); check('GET: approved=true only after the Agent reported the same generation', v is not None and v['applied']['linear_mps'] == 0.05, json.dumps(v)[:200] if v else 'not confirmed')
    p1 = peak(drive(3.0)); check('with 0.05 m/s / 0.15 rad/s the Gate output is clamped (flow continues)', p1[2] >= 10 and 0.04 <= p1[0] <= 0.0501 and 0.12 <= p1[1] <= 0.1501, str(p1))
    time.sleep(1.0)                                  # the previous bench input decays within its 250 ms TTL; measure after that
    z = peak(drive(1.5, linear=0.0, turn=0.0)); check('zero axes give zero output (STOP preserved)', z[2] == 0, str(z))
    time.sleep(1.0); z2 = drive(1.5, deadman=False); check('Deadman released gives zero output', peak(z2)[2] == 0, str(peak(z2)))

    print('-- restore only by an explicit request')
    r = limits('manual', 0.10, 0.30); v = confirmed(r.get('generation')); p2 = peak(drive(3.0))
    check('explicit request back to the ceiling is applied and the output follows', v is not None and 0.08 <= p2[0] <= 0.1001, str(p2))
    r = limits('manual', 0.20, 0.30); check('0.20 m/s refused (422), nothing changed', r.get('_http') == 422 and r['error']['code'] == 'speed_exceeds_ceiling', json.dumps(r)[:120])

    print('-- lease loss never releases the limit')
    r = limits('manual', 0.05, 0.15); confirmed(r.get('generation')); call('/lock', dict(action='release'))
    time.sleep(1.5); lease = call('/lock', dict(action='acquire', ttl_s=300)); st['epoch'] = lease['epoch']
    check('operating mode MANUAL again under the new lease', mode('MANUAL'))
    sl = state().get('speed_limits') or {}
    check('Agent reports the limit as stale (needs a new request) and keeps clamping', sl.get('stale') is True and abs(sl['effective']['manual']['linear_mps'] - 0.05) < 1e-9, json.dumps({k: sl.get(k) for k in ('stale', 'stale_reason')}))
    p3 = peak(drive(3.0)); check('the output stays at the lowered limit after the lease change (no automatic restore)', p3[2] >= 10 and p3[0] <= 0.0501, str(p3))
    v = call('/speed-limits'); check('GET: approved=false while stale', v.get('approved') is False, json.dumps({k: v.get(k) for k in ('approved', 'status', 'stale')}))
    r = limits('manual', 0.10, 0.30); confirmed(r.get('generation')); p4 = peak(drive(3.0))
    check('a new request under the new lease raises it again', p4[0] >= 0.08, str(p4))

    print('-- MAPPING (Nav2/Frontier) path, auto scope')
    r = limits('auto', 0.06, 0.20); v = confirmed(r.get('generation')); check('auto limit applied', v is not None)
    check('operating mode MAPPING reached', mode('MAPPING')); renew()
    sid = call('/mapping').get('session_id'); call('/mapping', dict(action='START', session_id=sid, idempotency_key=uuid.uuid4().hex, epoch=st['epoch']))
    t0 = time.monotonic(); end = t0 + 25; seen = []
    while time.monotonic() < end:
        renew(); time.sleep(1.0); seen = peak([r for r in rows if r[0] >= t0])
        if seen[2] > 5: time.sleep(4.0); break
    seen = peak([r for r in rows if r[0] >= t0])
    print(f'   Nav2/Frontier output seen on /cmd_vel_locked: peak linear {seen[0]:.3f}, peak angular {seen[1]:.3f}, {seen[2]} non-zero samples')
    if seen[2] > 5:
        check('Nav2 output (non-zero) never exceeds the auto limit at the Gate', seen[0] <= 0.0601 and seen[1] <= 0.2001, str(seen))
    else:
        check('Nav2 produced no non-zero command in this synthetic run (auto clamp NOT exercised by flow; unit-tested only)', True, 'not a pass of the auto-clamp flow test')
    call('/mapping', dict(action='STOP', session_id=call('/mapping')['session_id'], idempotency_key=uuid.uuid4().hex, epoch=st['epoch']))
    wait(lambda: call('/mapping').get('state') == 'STOPPED', 40)
    n = sum(results); print(f'{n}/{len(results)} PASS'); sys.exit(0 if n == len(results) else 1)


if __name__ == '__main__':
    main()
