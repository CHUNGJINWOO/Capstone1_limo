#!/usr/bin/env python3
"""Duplicate-command checks without ROS: (1) the agent never executes a re-delivered command_id twice, (2) the server ignores duplicate/stale acks,
(3) an idempotency_key never creates a second mission. usage: python3 tests/test_dedupe.py"""
import os, sys, tempfile, time
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'agent', 'limo_robot_agent'), os.path.join(ROOT, 'server')]
from limo_robot_agent import core
from fleet_server.db import DB
from fleet_server.service import Fleet, ApiError
RES = []
def check(n, ok, d=''): RES.append(ok); print(f"  [{'PASS' if ok else 'FAIL'}] {n} {d}")

# ---- (1) agent: the same command delivered three times (lost ack / server restart) is executed once
class Handler:
    def __init__(self): self.runs = 0
    def start_navigate(self, cmd, ack): self.runs += 1; ack('ACCEPTED', 'ok'); ack('SUCCEEDED', 'done')
    def cancel_mission(self, cmd, ack): ack('SUCCEEDED', 'x')
sent = []
class FakeHttp:
    n = 0
    def __init__(self, *a, **k): pass
    def post(self, path, body, timeout=None):
        if path == '/agent/hello': return {'server_boot_id': 'b1'}
        if path == '/agent/state':
            FakeHttp.n += 1
            cmds = [{'command_id': 'c_dup', 'robot_id': 't', 'owner_boot_id': 'test-boot', 'seq': 1, 'issued_at': time.time(), 'expires_at': time.time()+120, 'type': 'navigate_to_pose', 'mission_id': 'm_x', 'payload': {'goal': {}}}] if FakeHttp.n <= 3 else []
            return {'server_boot_id': 'b1', 'commands': cmds}
        sent.append((path, body.get('status'))); return {'ok': True}
core.Http = FakeHttp
h = Handler(); d = tempfile.mkdtemp()
a = core.AgentCore('t', 'http://x', 'tok', lambda: {}, h, {'boot_id': 'test-boot'}, d, period_s=0.05, log=lambda m: None)
import threading; th = threading.Thread(target=a.run, daemon=True); th.start(); time.sleep(1.0); a.stop()
check('agent executed the re-delivered command exactly once', h.runs == 1, f'runs={h.runs}')
check('agent counted the duplicate deliveries', a.stats['duplicates_ignored'] == 2, str(a.stats))
check('agent re-acknowledged the stored final status for the duplicates', sum(1 for p, s in sent if s == 'SUCCEEDED') >= 3, str(sent)[:120])
a2 = core.AgentCore('t', 'http://x', 'tok', lambda: {}, Handler(), {}, d, log=lambda m: None)
check('de-duplication survives an agent restart (state file)', 'c_dup' in a2.seen and a2.seen['c_dup']['status'] == 'SUCCEEDED')

# ---- (2)(3) server
class Cfg: offline_after_s = 6.0; command_ttl_s = 120.0; resend_after_s = 5.0
db = DB(os.path.join(tempfile.mkdtemp(), 'f.db')); f = Fleet(db, Cfg()); f.hello('r1', {'boot_id':'unit','protocol_version':2}); f.reconcile('r1','unit',{}, {},True)
g = {'frame_id': 'map', 'x': 1, 'y': 2}
m1, c1 = f.create_mission('r1', 'navigate_to_pose', g, 'key-A', '', 't'); m2, c2 = f.create_mission('r1', 'navigate_to_pose', g, 'key-A', '', 't')
check('same idempotency_key -> same mission, created only once', m1['mission_id'] == m2['mission_id'] and c1 and not c2)
m3, _ = f.create_mission('r1', 'navigate_to_pose', g, 'key-B', '', 't'); check('different key -> new mission', m3['mission_id'] != m1['mission_id'])
cmds = f.ingest_state('r1', 1, {}, 'unit')['commands']; cid = cmds[0]['command_id']
check('exactly one command delivered at a time per robot', len(cmds) == 1 and cmds[0]['mission_id'] == m1['mission_id'])
check('un-acked command is not re-sent immediately', f.ingest_state('r1', 2, {}, 'unit')['commands'] == [])
f.cfg.resend_after_s = 0.0; again = f.ingest_state('r1', 3, {}, 'unit')['commands']
check('un-acked command is re-sent with the SAME command_id (attempt 2)', len(again) == 1 and again[0]['command_id'] == cid and again[0]['attempt'] == 2)
r = f.ack_command('r1', cid, 'ACCEPTED'); check('first ack applied', not r['duplicate'])
r = f.ack_command('r1', cid, 'ACCEPTED'); check('duplicate ack ignored', r['duplicate'])
f.ack_command('r1', cid, 'SUCCEEDED', 'done'); r = f.ack_command('r1', cid, 'EXECUTING'); check('stale (regressing) ack ignored', r['duplicate'] and f.mission(m1['mission_id'])['status'] == 'SUCCEEDED')
r = f.ack_command('r1', cid, 'FAILED'); check('terminal state is final (conflicting later ack ignored)', f.mission(m1['mission_id'])['status'] == 'SUCCEEDED')
try: f.ack_command('r2', cid, 'ACCEPTED'); check('ack from another robot refused', False)
except ApiError as e: check('ack from another robot refused', e.status == 404)
print(f'\nTOTAL {sum(RES)}/{len(RES)} passed'); sys.exit(0 if all(RES) else 1)
