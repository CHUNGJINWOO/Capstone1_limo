#!/usr/bin/env python3
"""Policy tests of MissionRunner with a fake Mission Manager (no ROS, no RMF). python3 tests/test_runner_policy.py"""
import os, sys, tempfile, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from limo_rmf_adapter.runner import MissionRunner
from limo_rmf_adapter.mission_client import MissionApiUnavailable, TERMINAL
R = []


def check(n, ok, d=''): R.append(ok); print(f"  [{'PASS' if ok else 'FAIL'}] {n} {d}")


class Fake:
    def __init__(self):
        self.me = 'rmf-adapter'; self.missions = {}; self.n = 0; self.down = False; self.fail_next_create = 0; self.create_calls = []; self.cancel_calls = []
        self.robot = {'online': True, 'reconciled': True, 'reconciliation_required': False, 'state': {'pose': {'status': 'OK', 'frame_id': 'map', 'x': 0, 'y': 0, 'yaw_rad': 0}}}
        self.auto_cancel = True; self.lease_owner = None; self.lease_calls = []; self.lease_deny = None

    def _chk(self):
        if self.down: raise MissionApiUnavailable('down')

    def lease(self, robot_id, action, ttl_s=None, note=''):
        from limo_rmf_adapter.mission_client import MissionApiError
        self._chk(); self.lease_calls.append(action)
        if self.lease_deny: raise MissionApiError(409, self.lease_deny, 'denied')
        if action == 'release': self.lease_owner = None; return {'released': True}
        if action == 'renew' and self.lease_owner != self.me: raise MissionApiError(409, 'lock_not_held', 'x')
        self.lease_owner = self.me; return {'owner': self.me, 'epoch': 1}

    def get_me(self): self._chk(); return {'name': self.me}
    def get_robot(self, r): self._chk(); return self.robot
    def list_missions(self, r, limit=100): self._chk(); return [dict(m) for m in self.missions.values()]
    def get_mission(self, i): self._chk(); return dict(self.missions[i])

    def create_mission(self, r, goal, key, note=''):
        self.create_calls.append(key)
        for m in self.missions.values():
            if m['idempotency_key'] == key: self._after_create(); return dict(m), False
        self.n += 1; m = {'mission_id': f'm_{self.n}', 'status': 'EXECUTING', 'detail': '', 'idempotency_key': key, 'created_by': self.me, 'goal': goal, 'progress': {'distance_remaining_m': 1.0}}
        self.missions[m['mission_id']] = m
        if self.fail_next_create: self.fail_next_create -= 1; raise MissionApiUnavailable('timeout after the server processed it')
        return dict(m), True

    def _after_create(self): self._chk()

    def cancel_mission(self, i):
        self._chk(); self.cancel_calls.append(i); m = self.missions[i]
        if m['status'] not in TERMINAL: m['status'] = 'CANCELING'
        return dict(m)

    def web_mission(self, status='EXECUTING'):
        self.n += 1; m = {'mission_id': f'm_{self.n}', 'status': status, 'detail': '', 'idempotency_key': 'web-uuid', 'created_by': 'operator', 'goal': {}, 'progress': {}}
        self.missions[m['mission_id']] = m; return m


class Cb:
    def __init__(self): self.done_calls = []; self.prog = []
    def progress(self, g, i, d): self.prog.append((g, i, d))
    def done(self, g, ok, why): self.done_calls.append((g, ok, why))


def make(f, **kw):
    d = tempfile.mkdtemp(); r = MissionRunner(f, 'pcu', os.path.join(d, 's.json'), log=lambda *_: None, poll_s=0.02, **kw); r.start(); return r, d


def until(fn, t=3.0):
    e = time.time() + t
    while time.time() < e:
        if fn(): return True
        time.sleep(0.02)
    return False


print('== single active mission / sequential goals')
f = Fake(); r, _ = make(f); cb = Cb(); r.follow_path([(1, 1, 0), (2, 2, 0)], cb)
until(lambda: len(f.missions) == 1); time.sleep(0.3)
check('only one mission in flight while the first is executing', len(f.missions) == 1)
f.missions['m_1']['status'] = 'SUCCEEDED'; until(lambda: len(f.missions) == 2)
f.missions['m_2']['status'] = 'SUCCEEDED'; until(lambda: cb.done_calls)
check('path done after the last goal succeeded', cb.done_calls == [(1, True, 'arrived')], str(cb.done_calls))
check('idempotency keys are stable and unique', len(set(f.create_calls)) == 2 and all(k.startswith('rmf:pcu:') for k in f.create_calls), str(f.create_calls))
r.close()

print('== ambiguous create (timeout) is retried with the SAME key, no duplicate mission')
f = Fake(); f.fail_next_create = 1; r, _ = make(f); cb = Cb(); r.follow_path([(1, 1, 0)], cb)
until(lambda: len(f.create_calls) >= 2)
check('create retried with the same idempotency key', len(f.create_calls) >= 2 and len(set(f.create_calls)) == 1, str(f.create_calls))
check('server holds exactly one mission', len(f.missions) == 1)
r.close()

print('== ownership: a mission of another owner (web UI) blocks RMF submissions')
f = Fake(); web = f.web_mission(); r, _ = make(f); cb = Cb(); r.follow_path([(1, 1, 0)], cb)
time.sleep(0.6)
check('no RMF mission was created behind the web mission', len(f.create_calls) == 0, str(f.create_calls))
check('RMF tasks are refused while another owner is active', r.busy_foreign() is True and r.snapshot()['link'] == 'FOREIGN_MISSION_ACTIVE', r.snapshot()['link'])
web['status'] = 'SUCCEEDED'; check('after the web mission ends the RMF goal is submitted', until(lambda: len(f.create_calls) == 1))
check('availability recovers', until(lambda: not r.busy_foreign()))
r.close()

print('== a forged rmf: key from another principal does not count as ours')
f = Fake(); m = f.web_mission(); m['idempotency_key'] = 'rmf:pcu:forged:1:0'; r, _ = make(f); r.follow_path([(1, 1, 0)], Cb()); time.sleep(0.5)
check('still blocked', len(f.create_calls) == 0); r.close()

print('== RECONCILIATION_REQUIRED blocks new RMF missions and fails the running path')
f = Fake(); r, _ = make(f); cb = Cb(); r.follow_path([(1, 1, 0)], cb); until(lambda: f.missions)
f.missions['m_1']['status'] = 'RECONCILIATION_REQUIRED'; until(lambda: cb.done_calls)
check('path reported failed (not completed)', cb.done_calls and cb.done_calls[0][1] is False and 'RECONCILIATION' in cb.done_calls[0][2], str(cb.done_calls))
n = len(f.create_calls); r.follow_path([(3, 3, 0)], Cb()); time.sleep(0.6)
check('no new mission while reconciliation is required', len(f.create_calls) == n and r.busy_foreign(), r.snapshot()['link'])
r.close()
f = Fake(); f.robot['reconciliation_required'] = True; f.robot['reconciled'] = False; r, _ = make(f); r.follow_path([(1, 1, 0)], Cb()); time.sleep(0.5)
check('robot-level reconciliation_required blocks submission', not f.create_calls and r.snapshot()['link'] == 'RECONCILIATION_REQUIRED'); r.close()

print('== cancel completes only when the Mission Manager reports a terminal result')
f = Fake(); r, _ = make(f); cb = Cb(); r.follow_path([(1, 1, 0), (2, 2, 0)], cb); until(lambda: f.missions)
r.stop(); until(lambda: f.cancel_calls); time.sleep(0.3)
check('cancel requested exactly for the active mission', f.cancel_calls[:1] == ['m_1'], str(f.cancel_calls))
check('still busy while the robot has not confirmed (CANCELING)', r.busy() and f.missions['m_1']['status'] == 'CANCELING')
r.follow_path([(9, 9, 0)], Cb()); time.sleep(0.4)
check('a new path is NOT submitted before the cancel is confirmed', len(f.create_calls) == 1, str(f.create_calls))
f.missions['m_1']['status'] = 'CANCELED'; check('after CANCELED the next path starts', until(lambda: len(f.create_calls) == 2))
check('no path completion was reported for the canceled generation', cb.done_calls == [])
r.close()

print('== stale generation: results of an older path are dropped')
f = Fake(); r, _ = make(f); cb1 = Cb(); r.follow_path([(1, 1, 0)], cb1); until(lambda: f.missions)
cb2 = Cb(); r.follow_path([(5, 5, 0)], cb2); until(lambda: f.cancel_calls)
f.missions['m_1']['status'] = 'SUCCEEDED'; until(lambda: len(f.create_calls) == 2); time.sleep(0.2)
check('old path did not report done', cb1.done_calls == [], str(cb1.done_calls))
check('new path goal submitted with its own generation key', f.create_calls[-1].split(':')[3] == '2', f.create_calls[-1])
r.close()

print('== connection loss and re-sync')
f = Fake(); r, _ = make(f); cb = Cb(); r.follow_path([(1, 1, 0)], cb); until(lambda: f.missions)
f.down = True; check('link reported as SERVER_UNREACHABLE', until(lambda: r.snapshot()['link'] == 'SERVER_UNREACHABLE'))
f.missions['m_1']['status'] = 'SUCCEEDED'; time.sleep(0.3)
check('no result is invented while disconnected', cb.done_calls == [])
f.down = False; check('after reconnect the real result is adopted exactly once', until(lambda: cb.done_calls == [(1, True, 'arrived')]) and len(f.create_calls) == 1, str(cb.done_calls))
r.close()

print('== adapter restart: a mission left by the previous run is canceled before anything new')
f = Fake(); d = tempfile.mkdtemp(); sp = os.path.join(d, 's.json')
r1 = MissionRunner(f, 'pcu', sp, log=lambda *_: None, poll_s=0.02); r1.start(); r1.follow_path([(1, 1, 0)], Cb()); until(lambda: f.missions); r1.stop_evt.set(); time.sleep(0.1)
r2 = MissionRunner(f, 'pcu', sp, log=lambda *_: None, poll_s=0.02); r2.start(); cb = Cb(); r2.follow_path([(2, 2, 0)], cb)
until(lambda: f.cancel_calls); time.sleep(0.3)
check('orphan canceled', f.cancel_calls[:1] == ['m_1'], str(f.cancel_calls))
check('new path waits for orphan to be terminal', len(f.create_calls) == 1)
f.missions['m_1']['status'] = 'CANCELED'; check('then continues', until(lambda: len(f.create_calls) == 2)); r2.close()
print('== control lease (ownership fencing)')
f = Fake(); r, _ = make(f, lease_ttl_s=1.0); cb = Cb(); r.follow_path([(1, 1, 0)], cb)
check('lease acquired before the first mission is submitted', until(lambda: f.create_calls) and f.lease_calls[:1] == ['acquire'], str(f.lease_calls[:3]))
f.lease_deny = 'ownership_conflict'; f.lease_owner = None
until(lambda: not r.lease_ok(), 3.0)
check('lease lost -> adapter reports NO_CONTROL_LEASE and refuses new RMF tasks', until(lambda: r.snapshot()['link'] == 'NO_CONTROL_LEASE') and r.check_available_now()[0] is False, r.snapshot()['link'])
n = len(f.create_calls); r.follow_path([(7, 7, 0)], Cb()); f.missions['m_1']['status'] = 'SUCCEEDED'; time.sleep(0.6)
check('no mission is submitted without the lease', len(f.create_calls) == n, str(f.create_calls))
f.lease_deny = None; check('lease re-acquired automatically when it is free again', until(lambda: r.lease_ok() and len(f.create_calls) == n + 1))
f.down = True; check('server unreachable: after the local deadline the adapter stops acting (fenced)', until(lambda: not r.lease_ok(), 3.0))
f.down = False; r.close(); check('lease released on shutdown', f.lease_calls[-1] == 'release', str(f.lease_calls[-2:]))
f = Fake(); f.lease_deny = 'robot_busy'; r, _ = make(f); r.follow_path([(1, 1, 0)], Cb()); time.sleep(0.6)
check('lease refused (robot busy with another principal): nothing submitted, reason kept', not f.create_calls and 'robot_busy' in r.lease_reason, r.lease_reason); r.close()
print(f'\n{sum(R)}/{len(R)} checks passed'); sys.exit(0 if all(R) else 1)
