"""Shared helpers for the isolated E2E runners (Phase 3.5 R4, 2026-11-21).

The four existing runners (manual_e2e, mode_e2e, nav_e2e, gate_reset_e2e) each carry their own copy of these helpers; they have
already diverged once (a `result=` keyword overwrote the verdict). New runners (tests/gazebo_web) use this module; the old runners
are migrated one at a time later, each with its own before/after regression run.
Test-only. Never point this at a robot or the production server (ports 8080 / 18190-18192 are refused).
"""
import json, os, subprocess, time, urllib.error, urllib.request
from pathlib import Path

PROTECTED_PORTS = (8080, 18190, 18191, 18192)


class HttpError(Exception):
    def __init__(self, status, body): super().__init__(f'HTTP {status}: {body[:200]}'); self.status, self.body = status, body


class Fleet:
    """Minimal Fleet HTTP client bound to one run's token files."""
    def __init__(self, port, secrets_dir, robot_id='iso_bench', timeout=8.0):
        assert int(port) not in PROTECTED_PORTS, 'protected port'
        self.base, self.secrets, self.rid, self.timeout = f'http://127.0.0.1:{port}/api/v1', Path(secrets_dir), robot_id, timeout

    def token(self, who): return (self.secrets / f'{who}.token').read_text().strip()

    def call(self, method, path, body=None, who='admin'):
        req = urllib.request.Request(self.base + path, data=None if body is None else json.dumps(body).encode(), method=method,
                                     headers={'Authorization': 'Bearer ' + self.token(who), 'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r: return json.loads(r.read())
        except urllib.error.HTTPError as e: raise HttpError(e.code, e.read().decode('utf-8', 'replace'))

    def get(self, path, who='operator'): return self.call('GET', path, None, who)
    def post(self, path, body, who='admin'): return self.call('POST', path, body, who)
    def robot(self): return next((r for r in self.get('/robots')['robots'] if r['robot_id'] == self.rid), {})
    def state(self): return self.robot().get('state') or {}
    def mode(self): return (self.get(f'/robots/{self.rid}/mode').get('operating_mode') or {})

    def lease(self, ttl=300, who='admin'):
        return self.post(f'/robots/{self.rid}/lock', {'action': 'acquire', 'ttl_s': ttl}, who)['epoch']

    def switch(self, target, epoch, timeout=60, who='admin'):
        self.post(f'/robots/{self.rid}/mode', {'target': target, 'epoch': epoch, 'idempotency_key': f'k-{target}-{time.monotonic_ns()}'}, who)
        return wait_until(lambda: (lambda m: m if m.get('mode') == target and (m.get('transition') or {}).get('state') in ('ACTIVE', None) else None)(self.mode()), timeout)


def wait_until(pred, timeout, step=.25):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            v = pred()
            if v: return v
        except Exception:
            pass
        time.sleep(step)
    return None


def expect_http(fn, status, code=None):
    try: fn()
    except HttpError as e:
        return e.status == status and (code is None or code in str(e)), str(e)[:240]
    return False, 'accepted'


class Results:
    """Scenario verdicts. Reserved keys cannot be overwritten by details (the bug class that hid e06/e11)."""
    RESERVED = ('scenario', 'result')

    def __init__(self, log_path):
        self.items = []; self.log = open(log_path, 'x')

    def say(self, *a):
        line = time.strftime('%H:%M:%S') + ' ' + ' '.join(str(x) for x in a)
        print(line, flush=True); self.log.write(line + '\n'); self.log.flush()

    def record(self, name, ok, **d):
        bad = [k for k in d if k in self.RESERVED]
        if bad: raise ValueError(f'reserved record keys: {bad}')
        self.items.append({'scenario': name, 'result': 'PASS' if ok else 'FAIL', **d})
        self.say(('PASS ' if ok else 'FAIL ') + name, json.dumps(d, default=str)[:900])

    def skip(self, name, reason):
        self.items.append({'scenario': name, 'result': 'SKIP', 'reason': reason}); self.say('SKIP ' + name, reason)

    def counts(self): return {k: sum(r['result'] == k for r in self.items) for k in ('PASS', 'FAIL', 'SKIP')}


class Stack:
    """Processes started by tests/mode_e2e/stack_up.sh for ONE run directory; only its recorded process groups are signalled."""
    def __init__(self, run_dir, mode_dir):
        self.run, self.tool = Path(run_dir), Path(mode_dir) / 'stack_proc.sh'

    def proc(self, op, name, sig=None):
        subprocess.run([str(self.tool), str(self.run), op, name] + ([sig] if sig else []), check=False)
        return time.monotonic()
