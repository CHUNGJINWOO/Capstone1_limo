"""Speed-limit requests (G2, 2026-11-23): POST /api/v1/robots/{id}/speed-limits, GET the same path.

A request asks the Agent to LOWER the final velocity clamp. It is not a motor command and never raises anything above the Agent's own
ceilings. The server records a request as PENDING and returns approved=false; only the Agent's own report of the same generation
turns it into APPLIED (approved=true). The Agent enforces the limit in its Arbiter (agent/.../speed_limits.py).

Rules (all fail closed, every refusal is audited):
  caller is the current Lease owner and quotes the current lease_epoch; agent_boot_id, protocol_version 2 and safety_contract_version 1
  match the connected Agent; the Agent reports an enforcement point (enforced_by 'ARBITER') and its ceilings; 0 < value <= the Agent's
  ceiling (and the server's own cap); finite numbers only. A new request supersedes a pending one; `expected_generation`, if sent,
  must equal the current generation (409 stale_generation); `client_request_id` makes a retry idempotent (same values -> same record).
  A pending request expires when the lease, server boot or Agent boot changes, or after PENDING_TTL_S without the Agent's confirmation.
  Nothing here restores a higher limit automatically: after Lease loss the Agent keeps clamping (stale) until a new request.
"""
import json
import math
import time
import uuid

SL_SERVER_CAP = {'manual': {'linear_mps': 0.10, 'angular_rps': 0.30}, 'auto': {'linear_mps': 0.15, 'angular_rps': 0.50}}
SL_MIN = {'linear_mps': 0.01, 'angular_rps': 0.02}
PENDING_TTL_S = 20.0
SL_SCOPES = ('manual', 'auto')


def _num(x):
    return type(x) in (int, float) and math.isfinite(x)


class SpeedLimitMixin:
    def _sl_key(self, rid):
        return 'speed_limits:' + rid

    def _sl_load(self, rid):
        row = self.db.one('SELECT v FROM meta WHERE k=?', (self._sl_key(rid),))
        try:
            rec = json.loads(row['v']) if row else None
        except (ValueError, TypeError, KeyError):
            rec = None
        return rec if isinstance(rec, dict) else {'generation': 0, 'current': None}

    def _sl_save(self, rid, rec):
        self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', (self._sl_key(rid), json.dumps(rec)))

    def _sl_refuse(self, rid, principal, status, code, msg, **data):
        from .service import ApiError
        self.db.event('speed_limit_refused', f'{getattr(principal, "name", None)}: {code}', rid, level='warning',
                      data=dict(code=code, actor=getattr(principal, 'name', None), **data))
        raise ApiError(status, code, msg)

    EVIDENCE_MAX_AGE_S = 2.0

    def _sl_evidence(self, rid, cur=None):
        """Server-side check of the Watchdog evidence in the Agent's report. `server_verified` is computed here from the facts; it is not
        taken from the Agent. A DDS writer GID / node name is a graph identifier, not an authentication of the Watchdog."""
        r = self.robots.get(rid) or {}
        rep = self._sl_agent_report(rid)
        ev = rep.get('watchdog_evidence')
        checks = {}
        if rep.get('enforced_by') != 'WATCHDOG' or not isinstance(ev, dict):
            return {'server_verified': False, 'applied_match': None, 'checks': {'evidence_present': False}}
        age = ev.get('evidence_age_s')
        state_age = time.time() - (r.get('state_ts') or 0)
        checks['agent_boot_matches'] = ev.get('agent_boot_id') == (r.get('agent') or {}).get('boot_id') and bool(ev.get('agent_boot_id'))
        checks['watchdog_boot_id'] = isinstance(ev.get('watchdog_boot_id'), str) and bool(ev['watchdog_boot_id'])
        checks['output_gid'] = isinstance(ev.get('output_gid'), str) and bool(ev['output_gid'])
        checks['enabled_synced_no_fault'] = ev.get('enabled') is True and ev.get('synced') is True and ev.get('publisher_ok') is True and ev.get('fault') is None
        checks['fresh'] = _num(age) and 0 <= age <= self.EVIDENCE_MAX_AGE_S and state_age <= 3.0
        checks['agent_verified'] = ev.get('agent_verified') is True
        # `server_verified` = the Watchdog is a working, fresh, synchronised enforcement point for THIS Agent boot (capability). It must stay true after a
        # Lease change so the operator can issue a NEW request; whether a particular request is applied is the separate `applied_match`.
        applied_match = None
        if cur and cur.get('status') == 'APPLIED':
            a = (ev.get('applied') or {}).get(cur['scope']) or {}
            ag = (rep.get('applied') or {}).get(cur['scope']) or {}
            rq = cur['requested']
            applied_match = bool(a.get('generation') == cur['generation'] and a.get('request_id') == cur['request_id'] and a.get('stale') is False
                                 and a.get('linear_mps') == rq['max_linear_mps'] and a.get('angular_rps') == rq['max_angular_rps']
                                 and ag.get('generation') == cur['generation'] and ag.get('linear_mps') == rq['max_linear_mps'])
        return {'server_verified': all(checks.values()), 'applied_match': applied_match, 'checks': checks}

    def _sl_annotate(self, rid):
        """Put the server's own verdict next to the Agent's evidence in the stored state the console reads (state.speed_limits.watchdog_evidence)."""
        rep = self._sl_agent_report(rid)
        ev = rep.get('watchdog_evidence')
        if isinstance(ev, dict):
            cur = self._sl_load(rid).get('current')
            v = self._sl_evidence(rid, cur)
            ev['server_verified'] = v['server_verified']; ev['server_checks'] = v['checks']; ev['applied_match'] = v['applied_match']

    def _sl_agent_report(self, rid):
        r = self.robots.get(rid) or {}
        rep = (r.get('state') or {}).get('speed_limits')
        return rep if isinstance(rep, dict) else {}

    # ------------------------------------------------------------------ operator API
    def request_speed_limits(self, robot_id, principal, body):
        from .service import ApiError
        body = body if isinstance(body, dict) else {}
        with self.lock:                      # no db.transaction(): a refusal raises, and the audit row must survive it
            r = self.robots.get(robot_id)
            if r is None:
                raise ApiError(404, 'unknown_robot', 'unknown robot')
            fail = lambda st, code, msg, **d: self._sl_refuse(robot_id, principal, st, code, msg, **d)
            lease = self.lease_view(robot_id)
            if not lease or lease.get('owner') != getattr(principal, 'name', None):
                fail(403, 'not_lease_owner', 'only the current Lease owner may change the speed limits')
            if type(body.get('lease_epoch')) is not int or body['lease_epoch'] != lease['epoch']:
                fail(409, 'lease_epoch_mismatch', 'lease_epoch is missing or not the current epoch')
            agent = r.get('agent') or {}
            if not r.get('online') or not self.agent_contract_ok(robot_id):
                fail(409, 'agent_protocol', 'the Agent is offline or its protocol / safety contract is not valid')
            if body.get('agent_boot_id') != agent.get('boot_id'):
                fail(409, 'agent_boot_mismatch', 'agent_boot_id is missing or not the connected Agent boot')
            if body.get('protocol_version') != 2 or body.get('safety_contract_version') != 1:
                fail(409, 'agent_protocol', 'protocol_version must be 2 and safety_contract_version 1')
            rep = self._sl_agent_report(robot_id)
            if rep.get('enforced_by') not in ('ARBITER', 'WATCHDOG'):
                fail(409, 'agent_cannot_enforce', 'this Agent reports no working enforcement point for speed limits (Arbiter or a synchronised Watchdog clamp)')
            scope = body.get('scope', 'auto')
            if scope not in SL_SCOPES:
                fail(400, 'bad_scope', 'scope must be "auto" or "manual"')
            lin, ang = body.get('max_linear_mps'), body.get('max_angular_rps')
            if not (_num(lin) and _num(ang)) or lin <= 0 or ang <= 0:
                fail(400, 'bad_value', 'max_linear_mps and max_angular_rps must be finite positive numbers')
            ceil = (rep.get('ceiling') or {}).get(scope)
            if not isinstance(ceil, dict) or not (_num(ceil.get('linear_mps')) and _num(ceil.get('angular_rps'))):
                fail(409, 'agent_ceiling_unknown', 'the Agent has not reported its ceiling')
            cap = SL_SERVER_CAP[scope]
            max_lin, max_ang = min(ceil['linear_mps'], cap['linear_mps']), min(ceil['angular_rps'], cap['angular_rps'])
            if lin > max_lin or ang > max_ang:
                fail(422, 'speed_exceeds_ceiling', f'limits above {max_lin} m/s / {max_ang} rad/s are refused', ceiling=dict(linear_mps=max_lin, angular_rps=max_ang))
            if lin < SL_MIN['linear_mps'] or ang < SL_MIN['angular_rps']:
                fail(422, 'speed_below_minimum', f'limits below {SL_MIN["linear_mps"]} m/s / {SL_MIN["angular_rps"]} rad/s are refused (use STOP to stop)')
            rec = self._sl_load(robot_id); cur = rec.get('current')
            cid = body.get('client_request_id')
            if cid is not None and (not isinstance(cid, str) or not 1 <= len(cid) <= 64):
                fail(400, 'bad_client_request_id', 'client_request_id must be a string of 1..64 characters')
            if cid is not None and cur and cur.get('client_request_id') == cid:
                same = (cur['scope'], cur['requested']['max_linear_mps'], cur['requested']['max_angular_rps']) == (scope, lin, ang)
                if not same:
                    fail(409, 'idempotency_conflict', 'client_request_id was already used with different values')
                return dict(self._sl_public(robot_id, cur), duplicate=True)
            exp = body.get('expected_generation')
            if exp is not None and (type(exp) is not int or exp != rec['generation']):
                fail(409, 'stale_generation', 'expected_generation is not the current generation', current_generation=rec['generation'])
            now = time.time()
            if cur and cur.get('status') == 'PENDING':
                cur.update(status='SUPERSEDED', reason='NEWER_REQUEST', resolved_at=now)
            rec['generation'] += 1
            cur = {'generation': rec['generation'], 'request_id': uuid.uuid4().hex, 'client_request_id': cid, 'scope': scope,
                   'requested': {'max_linear_mps': float(lin), 'max_angular_rps': float(ang)}, 'lease_epoch': lease['epoch'],
                   'server_boot_id': self.boot_id, 'agent_boot_id': agent.get('boot_id'), 'requested_by': principal.name,
                   'requested_at': now, 'status': 'PENDING', 'reason': 'PENDING_AGENT_APPLY', 'applied': None, 'resolved_at': None}
            rec['current'] = cur
            self._sl_save(robot_id, rec)
            self.db.event('speed_limit_requested', f'{principal.name}: {scope} {lin} m/s {ang} rad/s (generation {cur["generation"]})', robot_id,
                          data=dict(generation=cur['generation'], scope=scope, linear=lin, angular=ang, lease_epoch=lease['epoch']))
            return self._sl_public(robot_id, cur)

    def speed_limits_view(self, robot_id):
        from .service import ApiError
        with self.lock:
            if robot_id not in self.robots:
                raise ApiError(404, 'unknown_robot', 'unknown robot')
            self._sl_advance(robot_id)
            rec = self._sl_load(robot_id)
            return self._sl_public(robot_id, rec.get('current'))

    def _sl_public(self, robot_id, cur):
        r = self.robots.get(robot_id) or {}
        rep = self._sl_agent_report(robot_id)
        age = None if not r.get('state_ts') else round(time.time() - r['state_ts'], 2)
        confirmed = bool(cur and cur.get('status') == 'APPLIED' and rep.get('generation') == cur['generation'] and rep.get('stale') is False
                         and rep.get('enforced_by') in ('ARBITER', 'WATCHDOG'))          # no enforcement point reported now: not approved
        evidence = None
        if rep.get('enforced_by') == 'WATCHDOG':
            v = self._sl_evidence(robot_id, cur)
            evidence = dict(rep.get('watchdog_evidence') or {}, server_verified=v['server_verified'], server_checks=v['checks'], applied_match=v['applied_match'])
            confirmed = confirmed and v['server_verified'] and v['applied_match'] is True          # a Watchdog-enforced limit is approved only with matching, fresh Watchdog evidence
        return {'robot_id': robot_id, 'agent_boot_id': (cur or {}).get('agent_boot_id') or (r.get('agent') or {}).get('boot_id'),
                'watchdog_boot_id': (evidence or {}).get('watchdog_boot_id'), 'watchdog_evidence': evidence,
                'generation': (cur or {}).get('generation', 0), 'request_id': (cur or {}).get('request_id'),
                'status': (cur or {}).get('status', 'NONE'), 'reason': (cur or {}).get('reason', 'NO_REQUEST'),
                'approved': confirmed,                                   # true only after the Agent reported the same generation as applied and not stale
                'scope': (cur or {}).get('scope'), 'requested': (cur or {}).get('requested'), 'applied': (cur or {}).get('applied'),
                'ceiling': rep.get('ceiling'), 'effective': rep.get('effective'), 'stale': rep.get('stale'), 'stale_reason': rep.get('stale_reason'),
                'enforced_by': rep.get('enforced_by'),
                'source': ('Watchdog clamp (Agent report plus Watchdog evidence checked by the server)' if rep.get('enforced_by') == 'WATCHDOG' else 'Agent arbiter clamp report')
                          + '; configured limits, not verified on a real robot; observed velocity telemetry is odom-based, not independent.',
                'requested_at': (cur or {}).get('requested_at'), 'resolved_at': (cur or {}).get('resolved_at'), 'agent_report_age_s': age}

    # ------------------------------------------------------------------ Agent side (called under self.lock from ingest_state)
    def _sl_advance(self, robot_id):
        rec = self._sl_load(robot_id); cur = rec.get('current')
        if not cur or cur.get('status') != 'PENDING':
            return cur
        r = self.robots.get(robot_id) or {}; rep = self._sl_agent_report(robot_id); now = time.time()
        lease = self.lease_view(robot_id); res = rep.get('last_result') if isinstance(rep.get('last_result'), dict) else {}
        new = None
        mine = res.get('request_id') == cur['request_id'] and res.get('generation') == cur['generation']
        if mine and res.get('status') == 'APPLIED':
            a = (rep.get('applied') or {}).get(cur['scope'])
            new = ('APPLIED', 'APPLIED', a if isinstance(a, dict) else None)
            if rep.get('enforced_by') == 'WATCHDOG':
                probe = dict(cur, status='APPLIED')
                pv = self._sl_evidence(robot_id, probe)
                if not (pv['server_verified'] and pv['applied_match'] is True):
                    new = None                             # the Agent says applied but the Watchdog evidence does not match: stay PENDING (expires)
                    if now - cur['requested_at'] > PENDING_TTL_S:
                        new = ('EXPIRED', 'WATCHDOG_EVIDENCE_MISSING_OR_MISMATCH', None)
        elif mine and res.get('status') == 'REJECTED':
            new = ('REJECTED', str(res.get('reason') or 'AGENT_REJECTED')[:80], None)
        elif mine and res.get('status') == 'PENDING_ENFORCEMENT':
            new = None                                    # the Agent validated it and waits for the Watchdog's confirmation (expires below)
            if (not lease or lease.get('epoch') != cur['lease_epoch'] or (r.get('agent') or {}).get('boot_id') != cur['agent_boot_id']):
                new = ('EXPIRED', 'BINDING_CHANGED', None)
            elif now - cur['requested_at'] > PENDING_TTL_S:
                new = ('EXPIRED', 'AGENT_NO_CONFIRMATION', None)
        elif rep.get('enforced_by') not in ('ARBITER', 'WATCHDOG'):
            new = ('REJECTED', 'NO_ENFORCEMENT_POINT', None)
        elif (not lease or lease.get('epoch') != cur['lease_epoch'] or cur['server_boot_id'] != self.boot_id
              or (r.get('agent') or {}).get('boot_id') != cur['agent_boot_id']):
            new = ('EXPIRED', 'BINDING_CHANGED', None)
        elif now - cur['requested_at'] > PENDING_TTL_S:
            new = ('EXPIRED', 'AGENT_NO_CONFIRMATION', None)
        if new:
            cur.update(status=new[0], reason=new[1], applied=new[2], resolved_at=now)
            self._sl_save(robot_id, rec)
            self.db.event('speed_limit_' + new[0].lower(), f'generation {cur["generation"]}: {new[1]}', robot_id,
                          level='info' if new[0] == 'APPLIED' else 'warning', data=dict(generation=cur['generation'], reason=new[1]))
        return cur

    def speed_limit_ingest(self, robot_id, state):
        # Speed-limit bookkeeping must never be able to break the Agent's state ingestion (a 500 here stops the Agent from reconnecting): a failure is
        # logged as an event and the request simply stays PENDING (it expires), nothing is approved.
        try:
            self._sl_advance(robot_id)
            self._sl_annotate(robot_id)
        except Exception as exc:
            try:
                self.db.event('speed_limit_ingest_error', f'{type(exc).__name__}: {exc}'[:200], robot_id, level='error', data={})
            except Exception:
                pass

    def speed_limit_for_agent(self, robot_id):
        """The pending request, re-sent in every state reply until the Agent reports its result (delivery is idempotent per generation)."""
        cur = self._sl_advance(robot_id)
        if not cur or cur.get('status') != 'PENDING' or not self.agent_contract_ok(robot_id):
            return None
        return {k: cur[k] for k in ('generation', 'request_id', 'scope', 'lease_epoch', 'server_boot_id', 'agent_boot_id')} | {
            'max_linear_mps': cur['requested']['max_linear_mps'], 'max_angular_rps': cur['requested']['max_angular_rps'], 'issued_at': cur['requested_at']}
