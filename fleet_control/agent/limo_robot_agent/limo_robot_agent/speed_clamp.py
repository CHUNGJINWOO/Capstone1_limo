"""Speed clamp inside the Motion Watchdog (G2, 2026-11-23): the enforcement point on a REAL robot, which has no Arbiter.

Scope of the change to the R2 Watchdog (kept to the minimum): (1) the clamp is applied to a non-zero command only inside the existing
single publish point `_publish_authorized` (state AUTHORIZED, not shutting down, input exclusive); zero publishes, LOCKED, FAILSAFE and
STOP never go through it; (2) one extra FAILSAFE/LOCKED condition SPEED_LIMIT_NOT_SYNCED when the clamp is enabled; (3) a status block.
Disabled by default (`speed_clamp:=false`): every R2 behaviour is then unchanged.

Static ceilings (parameters, defaults = the existing profiles) are separate from the dynamically approved limits. An approved limit is
always <= the static ceiling and is only ever LOWER: nothing here raises a limit on its own.
  * accepted only from the single publisher named in `speed_limit_publisher_nodes` on /fleet/speed_limit, whose message carries the
    agent_boot_id the Watchdog currently sees in the heartbeat (DDS is not authenticated: graph identity only)
  * lowering is accepted at any time; raising needs a higher generation within the same binding (server boot, lease epoch, contract
    generation) or a new binding (a new request the Agent validated under the new lease)
  * a missing, late or stale message keeps the last accepted value (it is NOT released); `stale` is reported
  * a persisted file is loaded as stale and keeps clamping after a restart; an unreadable file means SAFE_FALLBACK; no file means ceilings
  * scope by the single input publisher: velocity_smoother -> auto, manual_mapping_teleop -> manual, the Agent (its Arbiter clamps by
    scope itself) -> static ceilings only
The clamp saturates each axis independently, so a saturated linear speed with a large turn request changes the path curvature.
"""
import json
import math
import os
import threading
import time

STATIC_DEFAULT = {'manual': {'forward': 0.10, 'reverse': 0.10, 'angular': 0.30}, 'auto': {'forward': 0.15, 'reverse': 0.10, 'angular': 0.50}}
SAFE_FALLBACK = {'linear_mps': 0.05, 'angular_rps': 0.15}
MIN_LINEAR, MIN_ANGULAR = 0.01, 0.02
SCOPE_BY_NODE = {'velocity_smoother': 'auto', 'manual_mapping_teleop': 'manual'}
SYNC_MAX_AGE_S = 1.5
STALE_AFTER_S = 3.0
RAISE_CONFIRM_S = 0.15


def _num(x):
    return type(x) in (int, float) and math.isfinite(x)


class WatchdogSpeedClamp:
    def __init__(self, static=None, state_path='', mono=time.monotonic):
        self.lock = threading.RLock()
        self.mono = mono
        self.static = {s: dict(v) for s, v in (static or STATIC_DEFAULT).items()}
        self.path = os.path.expanduser(state_path) if state_path else ''
        self.applied = {'manual': None, 'auto': None}      # {linear_mps, angular_rps, generation, request_id, binding, seen_t}
        self.sync_t = None                                 # last message from the matching Agent boot (a message with no scope also counts)
        self.sync_boot = None
        self.last_seq = None
        self.pending_raise = {}                            # scope -> (key, first_seen): a RAISE needs the same message twice, >= RAISE_CONFIRM_S apart
        self.counters = {'accepted': 0, 'confirmed': 0, 'rejected': 0, 'clamped': 0}
        self.last_reject = None
        self.fault, self.loaded_stale = None, False
        self._load()

    # ---- persistence: best effort; a failure to write never raises a limit
    def _load(self):
        if not self.path or not os.path.exists(self.path):
            return
        try:
            d = json.load(open(self.path))
            for s in self.applied:
                a = d['applied'].get(s)
                if a is None:
                    continue
                lin, ang = float(a['linear_mps']), float(a['angular_rps'])
                c = self.static[s]
                if not (_num(lin) and _num(ang) and MIN_LINEAR <= lin <= c['forward'] and MIN_ANGULAR <= ang <= c['angular']):
                    raise ValueError('out of range')
                self.applied[s] = {'linear_mps': lin, 'angular_rps': ang, 'generation': int(a.get('generation', 0)), 'request_id': a.get('request_id'),
                                   'binding': None, 'seen_t': None}
            self.loaded_stale = True
        except Exception:
            self.applied = {s: dict(SAFE_FALLBACK, generation=0, request_id=None, binding=None, seen_t=None) for s in self.applied}
            self.fault, self.loaded_stale = 'PERSISTED_STATE_UNREADABLE', True

    def _save(self):
        if not self.path:
            return
        try:
            os.makedirs(os.path.dirname(self.path) or '.', exist_ok=True)
            tmp = self.path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'applied': {s: (None if a is None else {k: a[k] for k in ('linear_mps', 'angular_rps', 'generation', 'request_id')})
                                       for s, a in self.applied.items()}}, f)
                f.flush(); os.fsync(f.fileno())
            os.replace(tmp, self.path)
        except OSError:
            self.fault = 'PERSIST_FAILED'

    # ---- limits
    def limits(self, scope):
        """(forward, reverse, angular) magnitudes. scope None = the Agent's own Arbiter output: static ceilings only."""
        if scope not in self.applied:
            c = self.static['auto']
            return c['forward'], c['reverse'], c['angular']
        c, a = self.static[scope], self.applied[scope]
        fwd = c['forward'] if a is None else min(c['forward'], a['linear_mps'])
        ang = c['angular'] if a is None else min(c['angular'], a['angular_rps'])
        return fwd, min(c['reverse'], fwd), ang

    def clamp_values(self, scope, v, w):
        if not (_num(v) and _num(w)):
            return 0.0, 0.0
        fwd, rev, ang = self.limits(scope)
        return min(max(v, -rev), fwd), min(max(w, -ang), ang)

    def apply(self, scope, msg):
        """Clamp a non-zero command in place of the original (called only from the authorized publish point, under its lock)."""
        with self.lock:
            v, w = self.clamp_values(scope, msg.linear.x, msg.angular.z)
            if (v, w) != (msg.linear.x, msg.angular.z):
                self.counters['clamped'] += 1
        out = type(msg)()
        out.linear.x, out.linear.y, out.linear.z = v, msg.linear.y, msg.linear.z
        out.angular.x, out.angular.y, out.angular.z = msg.angular.x, msg.angular.y, w
        return out

    # ---- input from the Agent
    def synced(self, hb_boot):
        with self.lock:
            return (self.sync_t is not None and self.sync_boot is not None and self.sync_boot == hb_boot
                    and self.mono() - self.sync_t <= SYNC_MAX_AGE_S)

    def accept(self, data, hb_boot, publisher_ok):
        """data: the decoded /fleet/speed_limit JSON. Returns the number of accepted scope changes."""
        with self.lock:
            now = self.mono()
            if not publisher_ok or not isinstance(data, dict) or hb_boot is None or data.get('agent_boot_id') != hb_boot:
                self.counters['rejected'] += 1; self.last_reject = 'PUBLISHER_OR_BOOT_MISMATCH'
                return 0
            binding = data.get('binding')
            scopes = data.get('scopes')
            if not isinstance(binding, list) or len(binding) != 3 or not isinstance(scopes, dict):
                self.counters['rejected'] += 1; self.last_reject = 'MALFORMED'
                return 0
            binding = tuple(binding)
            self.sync_t, self.sync_boot = now, hb_boot
            n = 0
            for s, a in scopes.items():
                if s not in self.applied or not isinstance(a, dict):
                    self.counters['rejected'] += 1; self.last_reject = 'BAD_SCOPE'; continue
                lin, ang, gen, rid = a.get('max_linear_mps'), a.get('max_angular_rps'), a.get('generation'), a.get('request_id')
                c = self.static[s]
                if (not (_num(lin) and _num(ang)) or type(gen) is not int or gen < 1 or not isinstance(rid, str)
                        or lin < MIN_LINEAR or ang < MIN_ANGULAR or lin > c['forward'] or ang > c['angular']):
                    self.counters['rejected'] += 1; self.last_reject = 'BAD_VALUE_OR_ABOVE_STATIC_CEILING'; continue
                cur = self.applied[s]
                same = cur is not None and cur['linear_mps'] == lin and cur['angular_rps'] == ang and cur['generation'] == gen and cur['request_id'] == rid
                lowering = cur is not None and lin <= cur['linear_mps'] and ang <= cur['angular_rps']
                is_raise = cur is not None and not lowering
                key = (binding, gen, rid, lin, ang)
                if is_raise and not same:
                    # A raise must be seen twice with the graph still showing exactly one allowed publisher: a second writer that appears
                    # between graph updates cannot get a raise through on one message.
                    first = self.pending_raise.get(s)
                    if first is None or first[0] != key:
                        self.pending_raise[s] = (key, now); self.counters['rejected'] += 1; self.last_reject = 'RAISE_AWAITING_SECOND_MESSAGE'; continue
                    if now - first[1] < RAISE_CONFIRM_S:
                        continue
                self.pending_raise.pop(s, None)
                if cur is None or cur['binding'] is None or cur['binding'] != binding or gen > cur['generation'] or (lowering and not same):
                    if cur is not None and cur['binding'] is None and not lowering and cur['generation'] and gen <= cur['generation']:
                        # a restored (stale, binding-less) value is only replaced by a strictly newer generation or a lower value
                        self.counters['rejected'] += 1; self.last_reject = 'WOULD_RAISE_RESTORED_LIMIT'; continue
                    self.applied[s] = {'linear_mps': float(lin), 'angular_rps': float(ang), 'generation': gen, 'request_id': rid, 'binding': binding, 'seen_t': now}
                    self.counters['accepted'] += 1; n += 1; self.loaded_stale = False
                elif same:
                    cur['seen_t'] = now; cur['binding'] = binding; self.counters['confirmed'] += 1
                else:
                    self.counters['rejected'] += 1; self.last_reject = 'STALE_GENERATION_OR_RAISE'
            if n:
                self._save()
            return n

    def status(self, hb_boot=None):
        with self.lock:
            now = self.mono()
            out = {'enabled': True, 'synced': self.synced(hb_boot), 'sync_age_s': None if self.sync_t is None else round(now - self.sync_t, 3),
                   'static': {s: dict(v) for s, v in self.static.items()}, 'applied': {}, 'effective': {}, 'fault': self.fault,
                   'restored_from_file_unconfirmed': self.loaded_stale, 'last_reject': self.last_reject, **self.counters}
            for s, a in self.applied.items():
                fwd, rev, ang = self.limits(s)
                out['effective'][s] = {'forward_mps': fwd, 'reverse_mps': rev, 'angular_rps': ang}
                out['applied'][s] = None if a is None else {
                    'linear_mps': a['linear_mps'], 'angular_rps': a['angular_rps'], 'generation': a['generation'], 'request_id': a['request_id'],
                    'binding': list(a['binding']) if a['binding'] else None,
                    'stale': a['binding'] is None or a['seen_t'] is None or now - a['seen_t'] > STALE_AFTER_S}
            return out
