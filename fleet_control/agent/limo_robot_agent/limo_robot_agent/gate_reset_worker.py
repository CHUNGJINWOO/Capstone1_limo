"""Admin web Gate reset, Agent side (2026-11-21). The Fleet Server delivers at most one `gate_reset_request` (admin-only, checked
there); this worker re-checks everything with its OWN evidence and only then calls the existing Gate service
/stall_detector/reset_latch (std_srvs/Trigger). The Gate (candidate r12) decides again inside that service and may refuse.

Checks before the call (all must pass, every result is reported):
  request     : not executed before (durable journal, written BEFORE the call = at most once, also across Agent restarts),
                first seen <= REQUEST_MAX_AGE_S ago, issued for THIS Agent boot and the CURRENT server boot
  session     : connected, no reconciliation required, state accepted recently, lease owner/epoch = the request's, mode IDLE (ACTIVE)
  gate        : status fresh, same Gate boot and fault_epoch the admin saw, Gate's own reset_readiness.ready is True
  watchdog    : status fresh, MOTION_AUTHORIZED for this Agent boot, reports its own /cmd_vel_raw writer GID
  publisher   : /cmd_vel_raw has exactly one endpoint, node fleet_motion_watchdog, readable GID = the watchdog's reported GID,
                and that writer belongs to the same DDS participant as the watchdog status writer (GUID prefix). A lingering
                endpoint of a killed writer (two endpoints) or any unclear origin is refused.
  graph       : the locked velocity graph (Arbiter's check: /cmd_vel unused, single Gate output/status, single watchdog)
  zero        : Arbiter output 0 with no source, no manual bench sample, no Agent-owned goal, Nav2 idle evidence
This is graph/consistency evidence, not authentication (see GATE_INPUT_EXCLUSIVITY_20261120.md). After a successful reset nothing is
resumed: the bench sample is dropped, the mode stays IDLE (server), and MANUAL/NAVIGATION need their normal admission again.
"""
import json
import math
import os
import threading
import time
from pathlib import Path

from . import topics
from .ros_graph import gid_hex, participant_prefix

REQUEST_MAX_AGE_S = 5.0
GATE_FRESH_S = .5
WATCHDOG_FRESH_S = .5          # status is published at 5 Hz
OBSERVE_S = 1.5               # every check must hold continuously for this long before the Gate is called
CALL_TIMEOUT_S = 3.0
WATCHDOG_NODE = topics.WATCHDOG_NODE


def _gid(info):
    return gid_hex(info, require_nonzero=True)          # Phase 3.5: shared helper, same semantics


__all__ = ['GateResetWorker', 'participant_prefix']


class GateResetWorker:
    def __init__(self, robot, agent, state_dir, arbiter=None, bench=None, trigger_client=None):
        self.robot, self.agent, self.arbiter, self.bench = robot, agent, arbiter, bench
        self.root = Path(state_dir).resolve() / 'gate_reset'
        self.root.mkdir(parents=True, exist_ok=True)
        self.journal_path = self.root / 'journal.json'
        self.lock = threading.Lock()
        self.journal = self._load()
        self.first_seen = {}
        self.last = None
        self.history = []
        if trigger_client is None:
            from std_srvs.srv import Trigger
            trigger_client = robot.create_client(Trigger, topics.GATE_RESET_SERVICE)
        self.client = trigger_client
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------ durable journal (at most once)
    def _load(self):
        try:
            d = json.loads(self.journal_path.read_text())
            return d if isinstance(d, dict) and isinstance(d.get('executed'), list) else {'executed': []}
        except (OSError, ValueError):
            return {'executed': []}

    def _remember(self, request_id, outcome):
        self.journal['executed'] = (self.journal['executed'] + [{'request_id': request_id, 'outcome': outcome, 't': time.time()}])[-200:]
        tmp = self.journal_path.with_suffix('.tmp')
        with open(tmp, 'w') as f:
            json.dump(self.journal, f); f.flush(); os.fsync(f.fileno())
        os.replace(tmp, self.journal_path)

    def seen(self, request_id):
        return any(e.get('request_id') == request_id for e in self.journal['executed'])

    # ------------------------------------------------------------ evidence
    def evaluate(self, req, now_mono=None):
        """Return (ok, checks) where checks maps name -> {'ok': bool, 'detail': ...}. Pure reads; no side effects."""
        now = time.monotonic() if now_mono is None else now_mono
        c = {}
        def put(name, ok, detail=None): c[name] = {'ok': bool(ok), 'detail': detail}
        r, a = self.robot, self.agent
        rid = req.get('request_id') if isinstance(req, dict) else None
        put('request_new', isinstance(rid, str) and 8 <= len(rid) <= 64 and not self.seen(rid), rid)
        first = self.first_seen.setdefault(rid, now) if isinstance(rid, str) else now
        put('request_age', now - first <= REQUEST_MAX_AGE_S, round(now - first, 3))
        sess = a.session_snapshot()
        put('agent_boot', req.get('agent_boot_id') == a.boot_id, req.get('agent_boot_id'))
        put('server_boot', isinstance(sess.get('server_boot_id'), str) and req.get('server_boot_id') == sess.get('server_boot_id'), sess.get('server_boot_id'))
        age = sess.get('last_state_age_s')
        put('session', sess.get('connected') is True and sess.get('reconciliation_required') is False and type(age) in (int, float) and age < 2.0,
            {'connected': sess.get('connected'), 'reconciliation_required': sess.get('reconciliation_required'), 'last_state_age_s': age})
        lease = sess.get('lease') or {}
        put('lease', lease.get('owner') == req.get('requested_by') and lease.get('epoch') == req.get('lease_epoch')
            and type(lease.get('expires_at')) in (int, float) and lease['expires_at'] > time.time(),
            {'owner': lease.get('owner'), 'epoch': lease.get('epoch')})
        om = sess.get('operating_mode') or {}
        tr = om.get('transition')
        put('mode_idle', om.get('mode') == 'IDLE' and (not tr or tr.get('state') in ('ACTIVE', None)), {'mode': om.get('mode'), 'transition': (tr or {}).get('state')})
        with r.lock:
            gate = dict(r.gate) if isinstance(r.gate, dict) else {}
            gage = r._age('gate')
        put('gate_fresh', gage is not None and 0 <= gage < GATE_FRESH_S, None if gage is None else round(gage, 3))
        put('gate_identity', gate.get('boot_id') == req.get('gate_boot_id') and gate.get('fault_epoch') == req.get('fault_epoch'),
            {'boot_id': gate.get('boot_id'), 'fault_epoch': gate.get('fault_epoch')})
        rr = gate.get('reset_readiness') if isinstance(gate.get('reset_readiness'), dict) else None
        put('gate_ready', rr is not None and rr.get('enforced') is True and rr.get('ready') is True,
            'GATE_RESET_READINESS_UNSUPPORTED' if rr is None else rr.get('blockers'))
        w = r.motion_watchdog
        wd = w[1] if w and isinstance(w[1], dict) else {}
        wage = None if not w else now - w[0]
        put('watchdog', wage is not None and wage < WATCHDOG_FRESH_S and wd.get('state') == 'MOTION_AUTHORIZED'
            and wd.get('agent_boot_id') == r.agent_boot_id, {'state': wd.get('state'), 'age_s': None if wage is None else round(wage, 3)})
        try:
            raw = r.get_publishers_info_by_topic(topics.CMD_VEL_RAW)
            status = r.get_publishers_info_by_topic(topics.WATCHDOG_STATUS)
            raw_g = [_gid(x) for x in raw]; st_g = [_gid(x) for x in status]
            reported = wd.get('output_gid')
            ok = (len(raw) == 1 and raw[0].node_name == WATCHDOG_NODE and raw_g[0] is not None and reported == raw_g[0]
                  and len(status) == 1 and status[0].node_name == WATCHDOG_NODE and st_g[0] is not None
                  and participant_prefix(raw_g[0]) == participant_prefix(st_g[0]))
            put('publisher_identity', ok, {'cmd_vel_raw_writers': [f'{x.node_name}:{(g or "?")[:12]}' for x, g in zip(raw, raw_g)],
                                           'watchdog_reported_gid': (reported or '')[:12] or None,
                                           'status_writers': len(status), 'same_participant': ok})
        except Exception as exc:
            put('publisher_identity', False, f'GRAPH_ERROR:{type(exc).__name__}')
        try:
            locked = self.arbiter.graph_locked() if self.arbiter is not None else self._graph_locked()
            put('graph', locked, None)
        except Exception as exc:
            put('graph', False, f'GRAPH_ERROR:{type(exc).__name__}')
        arb = self.arbiter.snapshot() if self.arbiter is not None else {}
        sample = getattr(self.bench, 'sample', None) if self.bench is not None else None
        owned = getattr(r, 'active', None)
        idle = r.manual_idle_evidence() if hasattr(r, 'manual_idle_evidence') else {}
        put('zero_velocity', (self.arbiter is None or (arb.get('selected_linear') == 0. and arb.get('selected_turn') == 0. and arb.get('source') == 'NONE'))
            and sample is None and not (isinstance(owned, dict) and owned.get('terminal_status') is None) and idle.get('ok') is True,
            {'arbiter_source': arb.get('source'), 'manual_sample': sample is not None, 'owned_goal': isinstance(owned, dict), 'nav_idle': idle.get('reason') if not idle.get('ok') else 'OK'})
        return all(v['ok'] for v in c.values()), c

    def _graph_locked(self):
        """Without a VelocityArbiter there is no locked chain whose graph could be verified: refuse (Phase 3.5 R3, 2026-11-21).
        The earlier fallback only checked /cmd_vel and the Gate output/status writers and accepted a rogue /cmd_vel_raw writer,
        a missing watchdog writer and foreign /fleet/nav_cmd_vel or smoother publishers (pinned in the characterization test)."""
        return False

    # ------------------------------------------------------------ execution
    def call_gate(self):
        from std_srvs.srv import Trigger
        if not self.client.wait_for_service(timeout_sec=1.0):
            return None, 'GATE_RESET_SERVICE_UNAVAILABLE'
        fut = self.client.call_async(Trigger.Request())
        end = time.monotonic() + CALL_TIMEOUT_S
        while not fut.done() and time.monotonic() < end:
            time.sleep(.02)
        if not fut.done():
            fut.cancel(); return None, 'GATE_RESET_SERVICE_TIMEOUT'
        res = fut.result()
        return bool(res.success), str(res.message)[:300]

    def observe(self, req):
        """Evaluate repeatedly for OBSERVE_S: every sample must pass, the watchdog must keep publishing (its status time advances) and the
        /cmd_vel_raw writer GID must not change. A writer killed just before the request leaves a fresh-looking last status and a lingering
        endpoint for a moment; it cannot keep publishing (2026-11-21, isolated E2E e04)."""
        end = time.monotonic() + OBSERVE_S
        wd_times, gids, samples = set(), set(), 0
        while True:
            ok, checks = self.evaluate(req)
            samples += 1
            w = self.robot.motion_watchdog
            if w and isinstance(w[1], dict): wd_times.add(w[1].get('t'))
            gids.add(str((checks.get('publisher_identity') or {}).get('detail')))
            if not ok or time.monotonic() >= end:
                break
            time.sleep(.1)
        if ok and OBSERVE_S > 0:
            alive = len(wd_times) >= 2
            stable = len(gids) == 1
            checks['observation'] = {'ok': alive and stable, 'detail': {'samples': samples, 'watchdog_status_updates': len(wd_times), 'writer_stable': stable}}
            ok = alive and stable
        return ok, checks

    def handle(self, req):
        rid = req.get('request_id')
        ok, checks = self.observe(req)
        rec = {'request_id': rid, 'checks': checks, 'at': time.time()}
        if not ok:
            failed = [k for k, v in checks.items() if not v['ok']]
            rec.update(status='REFUSED', reason='AGENT_PRECHECK:' + ','.join(failed), gate_called=False)
            if checks['request_new']['ok']:
                self._remember(rid, rec['status'])          # a refused request is spent as well: the admin must issue a new one
        else:
            self._remember(rid, 'CALLING')                  # durable BEFORE the side effect
            success, msg = self.call_gate()
            rec.update(gate_called=True, gate_message=msg)
            if success:
                if self.bench is not None:                  # nothing resumes: a stale bench sample is dropped
                    with self.bench.lock: self.bench.sample = None
                rec.update(status='SUCCEEDED', reason=None)
            else:
                rec.update(status='REFUSED' if success is False else 'FAILED', reason=('GATE:' + msg) if success is False else msg)
            self._remember(rid, rec['status'])
        with self.lock:
            self.last = rec; self.history = (self.history + [{k: rec.get(k) for k in ('request_id', 'status', 'reason', 'at')}])[-5:]
        return rec

    def run(self):
        while not self.stop.is_set():
            try:
                req = self.agent.session_snapshot().get('gate_reset_request')
                if isinstance(req, dict) and isinstance(req.get('request_id'), str):
                    with self.lock:
                        done = self.last is not None and self.last.get('request_id') == req['request_id']
                    if not done:
                        self.handle(req)
            except Exception as exc:
                with self.lock:
                    self.last = {'request_id': None, 'status': 'FAILED', 'reason': f'WORKER_ERROR:{type(exc).__name__}', 'at': time.time()}
            self.stop.wait(.1)

    def snapshot(self):
        with self.lock:
            return {'enabled': True, 'last': json.loads(json.dumps(self.last, default=str)) if self.last else None, 'history': list(self.history)}

    def close(self):
        self.stop.set(); self.thread.join(timeout=2)
