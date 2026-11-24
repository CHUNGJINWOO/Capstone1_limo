"""Agent-side speed limits (G2, 2026-11-23): a web-requested LOWER bound for the final velocity, enforced by the Arbiter.

This is not a motor command and never raises anything above the fixed ceilings below (the existing safety profiles).

Ceilings (reused from the existing profiles, not new numbers):
  manual  0.10 m/s forward only (select_locked), +-0.30 rad/s
  auto    +0.15 m/s forward, -0.10 m/s reverse (velocity_arbiter.NAV_LINEAR), +-0.50 rad/s
These are CONFIGURED values of the slow profile; none of them has been verified on the real robot.

Rules
  * no explicit setting            -> ceilings (the behaviour before this feature)
  * a request is accepted only if it is bound to the CURRENT lease epoch, server boot, Agent boot, contract (protocol 2, safety contract 1),
    is newer than the last accepted generation of that binding, and 0 < value <= ceiling (finite numbers, no bool)
  * the applied value is a lower bound: clamp = min(ceiling, applied). It is NEVER raised by the Agent on its own:
      lease lost / epoch changed / server boot or contract generation changed -> the setting is marked stale and keeps clamping
      (the operator must send a new request under the new lease; it is not reused as a valid setting but also never released)
      Agent restart -> the persisted values are loaded as stale and keep clamping; an unreadable file -> SAFE_FALLBACK for both scopes
  * zero and STOP pass through unchanged (a clamp only shortens a non-zero magnitude)
The clamp saturates each axis independently (linear and angular are not scaled together), so a saturated linear speed with a large turn
request changes the path curvature. Nav2 `SpeedLimit` is advisory and is not relied on (see docs).
"""
import json
import math
import os
import threading
import time

MANUAL_CEIL = {'linear_mps': 0.10, 'angular_rps': 0.30}
AUTO_CEIL = {'linear_mps': 0.15, 'angular_rps': 0.50, 'reverse_mps': 0.10}
SAFE_FALLBACK = {'linear_mps': 0.05, 'angular_rps': 0.15}      # used only if the persisted setting is unreadable
MIN_LINEAR, MIN_ANGULAR = 0.01, 0.02
SCOPES = ('manual', 'auto')


def _num(x):
    return type(x) in (int, float) and math.isfinite(x)


def ceilings(scope):
    return MANUAL_CEIL if scope == 'manual' else AUTO_CEIL


class SpeedLimiter:
    def __init__(self, state_path=None, clock=time.time, mono=time.monotonic):
        self.lock = threading.RLock()
        self.clock, self.mono, self.path = clock, mono, state_path
        self.applied = {s: None for s in SCOPES}      # {'linear_mps','angular_rps','generation','lease_epoch','server_boot_id','at'}
        self.binding = None                           # (server_boot_id, lease_epoch, contract_generation) of the last accepted request
        self.generation = 0                           # highest accepted generation within `binding`
        self.stale, self.stale_reason, self.fault = False, None, None
        self.last_result = None
        self.enforced_by = None
        self.request_id_done = None
        self.confirm_fn = None        # set by the Watchdog link: an enforcement point OUTSIDE this process must confirm before a request counts as applied
        self.candidate = None
        self.confirm_timeout_s = 10.0
        self.watchdog_evidence = None   # set by the Watchdog link: what the Watchdog itself reported (boot, writer GID, applied generation and values)
        self._load()

    # ------------------------------------------------------------ persistence (best effort; a failure never raises the limit)
    def _load(self):
        if not self.path or not os.path.exists(self.path):
            return
        try:
            d = json.load(open(self.path))
            for s in SCOPES:
                a = d['applied'].get(s)
                if a is None:
                    continue
                c = ceilings(s)
                lin, ang = float(a['linear_mps']), float(a['angular_rps'])
                if not (_num(lin) and _num(ang) and MIN_LINEAR <= lin <= c['linear_mps'] and MIN_ANGULAR <= ang <= c['angular_rps']):
                    raise ValueError('out of range')
                self.applied[s] = {'linear_mps': lin, 'angular_rps': ang, 'generation': int(a.get('generation', 0)), 'lease_epoch': None,
                                   'server_boot_id': None, 'at': a.get('at')}
            self.stale, self.stale_reason = True, 'AGENT_RESTARTED_NEEDS_NEW_REQUEST'
        except Exception:
            self.applied = {s: dict(SAFE_FALLBACK, generation=0, lease_epoch=None, server_boot_id=None, at=None) for s in SCOPES}
            self.stale, self.stale_reason, self.fault = True, 'PERSISTED_STATE_UNREADABLE', 'PERSISTED_STATE_UNREADABLE'

    def _save(self):
        if not self.path:
            return
        try:
            tmp = self.path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'applied': {s: (None if a is None else {k: a[k] for k in ('linear_mps', 'angular_rps', 'generation', 'at')})
                                       for s, a in self.applied.items()}}, f)
                f.flush(); os.fsync(f.fileno())
            os.replace(tmp, self.path)
        except OSError:
            self.fault = 'PERSIST_FAILED'           # the in-memory (lower) setting still clamps; a restart would fall back to the ceilings

    # ------------------------------------------------------------ clamp
    def limits(self, scope):
        """(max_forward, max_reverse, max_angular) magnitudes that apply now."""
        c = ceilings(scope)
        with self.lock:
            a = self.applied.get(scope)
        lin = c['linear_mps'] if a is None else min(c['linear_mps'], a['linear_mps'])
        ang = c['angular_rps'] if a is None else min(c['angular_rps'], a['angular_rps'])
        rev = 0.0 if scope == 'manual' else min(c['reverse_mps'], lin)
        return lin, rev, ang

    def clamp(self, scope, v, w):
        """Final saturation. Non-finite -> 0. Zero stays zero (STOP is never changed)."""
        if scope not in SCOPES or not (_num(v) and _num(w)):
            return 0.0, 0.0
        lin, rev, ang = self.limits(scope)
        return min(max(v, -rev), lin), min(max(w, -ang), ang)

    # ------------------------------------------------------------ requests from the server (carried in the state reply)
    def process(self, session):
        """Called every Arbiter tick. `session` is AgentCore.session_snapshot(). Idempotent per generation."""
        with self.lock:
            self._check_binding(session)
            self._advance_candidate(session)
            req = (session or {}).get('speed_limit_request')
            if not isinstance(req, dict):
                return
            gen = req.get('generation')
            if type(gen) is int and self.last_result and self.last_result.get('generation') == gen and self.last_result.get('request_id') == req.get('request_id'):
                return                                              # already handled; the result keeps being reported
            self.last_result = self._decide(req, session)

    def _binding(self, session):
        lease = (session or {}).get('lease') or {}
        return ((session or {}).get('server_boot_id'), lease.get('epoch'), (session or {}).get('contract_generation'))

    def _check_binding(self, session):
        if self.binding is None or self.stale:
            return
        lease = (session or {}).get('lease') or {}
        ok = (isinstance(session, dict) and session.get('connected') is True and type(lease.get('epoch')) is int
              and type(lease.get('expires_at')) in (int, float) and lease['expires_at'] > self.clock()
              and self._binding(session) == self.binding)
        if not ok:
            self.stale, self.stale_reason = True, 'BINDING_LOST_LEASE_SERVER_OR_CONTRACT_CHANGED'

    def _reject(self, req, reason):
        return {'generation': req.get('generation') if type(req.get('generation')) is int else None, 'request_id': req.get('request_id'),
                'status': 'REJECTED', 'reason': reason, 'at': self.clock()}

    def _decide(self, req, session):
        if self.enforced_by is None:
            return self._reject(req, 'NO_ENFORCEMENT_POINT')
        lease = (session or {}).get('lease') or {}
        scope, gen = req.get('scope'), req.get('generation')
        if scope not in SCOPES:
            return self._reject(req, 'BAD_SCOPE')
        if type(gen) is not int or gen < 1:
            return self._reject(req, 'BAD_GENERATION')
        if session.get('connected') is not True or session.get('reconciliation_required') is not False:
            return self._reject(req, 'NOT_CONNECTED_OR_RECONCILING')
        if session.get('protocol_version') != 2 or session.get('safety_contract_version') != 1:
            return self._reject(req, 'CONTRACT_INVALID')
        if not (type(lease.get('epoch')) is int and type(lease.get('expires_at')) in (int, float) and lease['expires_at'] > self.clock()):
            return self._reject(req, 'LEASE_MISSING_OR_EXPIRED')
        if req.get('lease_epoch') != lease['epoch'] or req.get('server_boot_id') != session.get('server_boot_id'):
            return self._reject(req, 'LEASE_OR_SERVER_BOOT_MISMATCH')
        if req.get('agent_boot_id') != session.get('agent_boot_id'):
            return self._reject(req, 'AGENT_BOOT_MISMATCH')
        lin, ang = req.get('max_linear_mps'), req.get('max_angular_rps')
        if not (_num(lin) and _num(ang)) or lin <= 0 or ang <= 0:
            return self._reject(req, 'BAD_VALUE')
        c = ceilings(scope)
        if lin < MIN_LINEAR or ang < MIN_ANGULAR:
            return self._reject(req, 'BELOW_MINIMUM')
        if lin > c['linear_mps'] or ang > c['angular_rps']:
            return self._reject(req, 'EXCEEDS_CEILING')
        binding = self._binding(session)
        if binding == self.binding and gen <= self.generation:
            return self._reject(req, 'STALE_GENERATION')
        cand = {'scope': scope, 'linear_mps': float(lin), 'angular_rps': float(ang), 'generation': gen, 'request_id': req.get('request_id'),
                'binding': binding, 'lease_epoch': lease['epoch'], 'server_boot_id': session.get('server_boot_id'), 'deadline': self.mono() + self.confirm_timeout_s}
        if self.confirm_fn is not None:
            # The enforcement point is another process (the Watchdog): nothing is applied until it reports the same generation and values.
            self.candidate = cand
            return {'generation': gen, 'request_id': req.get('request_id'), 'status': 'PENDING_ENFORCEMENT', 'reason': 'AWAITING_WATCHDOG_CONFIRMATION', 'scope': scope, 'at': self.clock()}
        return self._commit(cand)

    def _commit(self, c):
        scope = c['scope']
        self.binding, self.generation, self.stale, self.stale_reason = c['binding'], c['generation'], False, None
        self.applied[scope] = {'linear_mps': c['linear_mps'], 'angular_rps': c['angular_rps'], 'generation': c['generation'], 'lease_epoch': c['lease_epoch'],
                               'server_boot_id': c['server_boot_id'], 'request_id': c['request_id'], 'at': self.clock()}
        self._save()
        return {'generation': c['generation'], 'request_id': c['request_id'], 'status': 'APPLIED', 'reason': 'APPLIED', 'scope': scope, 'at': self.clock()}

    def _advance_candidate(self, session):
        c = self.candidate
        if c is None:
            return
        rej = lambda why: {'generation': c['generation'], 'request_id': c['request_id'], 'status': 'REJECTED', 'reason': why, 'at': self.clock()}
        if self._binding(session) != c['binding']:
            self.candidate, self.last_result = None, rej('BINDING_CHANGED_BEFORE_ENFORCEMENT')
        elif self.confirm_fn is not None and self.confirm_fn(c):
            self.candidate, self.last_result = None, self._commit(c)
        elif self.mono() > c['deadline']:
            self.candidate, self.last_result = None, rej('WATCHDOG_NO_CONFIRMATION')

    def watchdog_scopes(self, session):
        """What the Watchdog should hold: the committed non-stale settings of the CURRENT binding plus the candidate being confirmed."""
        with self.lock:
            out = {}
            if self.binding is not None and not self.stale and self._binding(session) == self.binding:
                for s, a in self.applied.items():
                    if a is not None and a.get('server_boot_id') is not None:
                        out[s] = {'max_linear_mps': a['linear_mps'], 'max_angular_rps': a['angular_rps'], 'generation': a['generation'],
                                  'request_id': a.get('request_id') or 'applied-g%d' % a['generation']}
            c = self.candidate
            if c is not None and self._binding(session) == c['binding']:
                out[c['scope']] = {'max_linear_mps': c['linear_mps'], 'max_angular_rps': c['angular_rps'], 'generation': c['generation'], 'request_id': c['request_id']}
            return out

    def snapshot(self):
        with self.lock:
            out = {'enforced_by': self.enforced_by, 'pending_enforcement': None if self.candidate is None else {k: self.candidate[k] for k in ('scope', 'generation', 'request_id')},
                   'watchdog_evidence': None if self.watchdog_evidence is None else dict(self.watchdog_evidence),
                   'generation': self.generation, 'stale': self.stale, 'stale_reason': self.stale_reason,
                   'fault': self.fault, 'last_result': dict(self.last_result) if self.last_result else None, 'ceiling': {}, 'applied': {}, 'effective': {}}
            for s in SCOPES:
                lin, rev, ang = self.limits(s)
                c = dict(ceilings(s)); a = self.applied[s]
                out['ceiling'][s] = c
                out['applied'][s] = None if a is None else {'scope': s, 'request_id': a.get('request_id'), 'linear_mps': a['linear_mps'], 'angular_rps': a['angular_rps'], 'generation': a['generation'],
                                                            'lease_epoch': a['lease_epoch'], 'age_s': None if a.get('at') is None else round(max(0.0, self.clock() - a['at']), 1)}
                out['effective'][s] = {'linear_mps': lin, 'reverse_mps': rev, 'angular_rps': ang}
            out['source'] = 'agent arbiter clamp (configured limits, not verified on a real robot)'
            return out
