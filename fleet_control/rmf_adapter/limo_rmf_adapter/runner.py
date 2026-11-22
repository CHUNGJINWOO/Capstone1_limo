"""MissionRunner: executes a sequence of Nav2 goals for ONE robot exclusively through the Mission Manager API.

Policies (all enforced here, independent of Open-RMF so they can be tested without it):
 * single active mission per robot: at most one mission of ours is in flight; the next goal is submitted only after the previous one is terminal;
 * ownership: a non-terminal mission of another principal (e.g. the web UI) or a RECONCILIATION_REQUIRED mission blocks submissions (no queueing behind it);
 * stale commands: every path carries a generation; results/progress of older generations are dropped, and an older generation's goals are never submitted;
 * duplicate goals: stable idempotency key rmf:<robot>:<boot>:<gen>:<idx>; an ambiguous (timed out) create is retried with the SAME key;
 * cancel: finished only when the Mission Manager reports a terminal status (CANCELED/SUCCEEDED/FAILED...), never on the request being accepted;
 * connection loss: link state OFFLINE, nothing is assumed; after reconnect the active mission is re-read by id and the result is adopted.
"""
import json, os, threading, time, uuid
from .mission_client import MissionApiError, MissionApiUnavailable, TERMINAL

LINK_OK, LINK_OFFLINE, LINK_ROBOT_OFFLINE, LINK_RECONCILE, LINK_FOREIGN, LINK_ORPHAN, LINK_NO_LEASE = 'OK', 'SERVER_UNREACHABLE', 'ROBOT_OFFLINE', 'RECONCILIATION_REQUIRED', 'FOREIGN_MISSION_ACTIVE', 'ORPHAN_MISSION_CANCELING', 'NO_CONTROL_LEASE'


class MissionRunner:
    def __init__(self, client, robot_id, state_path, log=print, poll_s=0.5, comm_fail_threshold=2, submit_timeout_s=60.0, queue_timeout_s=45.0, boot_id=None, use_lease=True, lease_ttl_s=20.0):
        self.c, self.robot_id, self.state_path, self.log = client, robot_id, state_path, log
        self.poll_s, self.comm_fail_threshold, self.submit_timeout_s, self.queue_timeout_s = poll_s, comm_fail_threshold, submit_timeout_s, queue_timeout_s
        self.boot_id = boot_id or uuid.uuid4().hex[:8]
        self.use_lease, self.lease_ttl = use_lease, lease_ttl_s; self.lease_deadline = 0.0; self.lease_last_ok = 0.0; self.lease_try_t = 0.0; self.lease_reason = 'not acquired yet'; self.lease_epoch = None
        self.lock = threading.RLock(); self.stop_evt = threading.Event(); self.thread = None
        self.me = None                      # our principal name, learned from created missions' created_by
        self.link = LINK_OFFLINE; self.comm_fails = 0; self.robot = None; self.last_error = ''
        self.gen = 0                        # current path generation
        self.goals = []; self.idx = 0; self.cb = None   # pending/active path: goals[(x,y,yaw)], next index, callbacks object
        self.active = None                  # {'mission_id','key','gen','idx','created','cancel_sent','status'}
        self.cancel_wanted = False; self.foreign_reason = None; self._foreign_t = 0.0; self.orphans = set(); self.events = []
        self._load_state()

    # ---------- persistence (only ids; no secrets)
    def _load_state(self):
        try:
            with open(self.state_path) as f: s = json.load(f)
            if s.get('mission_id'): self.orphans.add(s['mission_id']); self._ev('adopt', f"previous run left mission {s['mission_id']}: it will be canceled before any new path")
        except FileNotFoundError: pass
        except Exception as e: self.last_error = f'state file unreadable: {type(e).__name__}'; self._ev('state_error', self.last_error)

    def _save_state(self):
        tmp = self.state_path + '.tmp'
        with open(tmp, 'w') as f: json.dump({'mission_id': self.active['mission_id'] if self.active else None, 'robot_id': self.robot_id, 'boot_id': self.boot_id, 't': time.time()}, f)
        os.replace(tmp, self.state_path)

    def _ev(self, kind, msg):
        self.events.append((time.time(), kind, msg)); del self.events[:-200]; self.log(f'[runner {self.robot_id}] {kind}: {msg}')

    # ---------- public API (called from RMF threads)
    def start(self):
        self.thread = threading.Thread(target=self._loop, daemon=True, name='mission-runner'); self.thread.start()

    def close(self):
        self.stop_evt.set(); self.thread and self.thread.join(3)
        if self.use_lease and self.lease_epoch is not None:
            try: self.c.lease(self.robot_id, 'release'); self._ev('lease', 'control lease released')
            except Exception: pass

    def follow_path(self, goals, cb):
        """goals: list of (x, y, yaw) in the Nav2 map frame; cb has progress(gen, idx, remaining_s), done(gen, ok, reason). Replaces the previous path."""
        with self.lock:
            self.gen += 1; g = self.gen
            self.goals, self.idx, self.cb = list(goals), 0, cb
            if self._cancellable(): self.cancel_wanted = True
            self._ev('path', f'generation {g}: {len(goals)} goals' + ('; previous mission is being canceled first' if self.cancel_wanted else ''))
            return g

    def stop(self):
        with self.lock:
            self.gen += 1; self.goals, self.idx = [], 0
            if self._cancellable(): self.cancel_wanted = True; self._ev('stop', 'RMF asked to stop: canceling the active mission')

    def snapshot(self):
        with self.lock:
            return {'link': self.link, 'robot': self.robot, 'gen': self.gen, 'active': dict(self.active) if self.active else None, 'pending_goals': max(0, len(self.goals) - self.idx),
                    'cancel_pending': self.cancel_wanted, 'orphans': sorted(self.orphans), 'last_error': self.last_error}

    def busy_foreign(self):
        """True when another owner's mission (or a reconciliation-required one) is active: RMF tasks must not be accepted then."""
        with self.lock: return bool(self.foreign_reason)

    def lease_ok(self):
        """Fenced by a LOCAL deadline (monotonic): if the lease cannot be renewed in time we stop acting even though the server is unreachable."""
        return (not self.use_lease) or time.monotonic() < self.lease_deadline

    def _manage_lease(self):
        if not self.use_lease: return
        now = time.monotonic(); held = now < self.lease_deadline
        if held and now - self.lease_last_ok < self.lease_ttl / 4: return
        if not held and now - self.lease_try_t < 2.0: return
        self.lease_try_t = now
        try:
            d = self.c.lease(self.robot_id, 'renew' if held else 'acquire', self.lease_ttl, 'RMF fleet adapter')
            self.lease_last_ok = now; self.lease_deadline = now + self.lease_ttl * 0.8
            if not held: self._ev('lease', f"control lease acquired (epoch {d.get('epoch')})")
            self.lease_epoch = d.get('epoch'); self.lease_reason = 'held'
        except MissionApiUnavailable:
            if held: self.lease_reason = 'renew failed: server unreachable (lease valid until local deadline)'
        except MissionApiError as e:
            if held or self.lease_epoch is not None: self._ev('lease', f'control lease LOST/refused: {e.code}')
            self.lease_deadline = 0.0; self.lease_epoch = None; self.lease_reason = f'{e.code}: {e}'
            if e.code in ('lock_not_held',): self.lease_try_t = 0.0

    def check_available_now(self):
        """Fresh (synchronous) ownership check used when RMF asks for a bid: never rely on a 1 s old cache. Returns (ok, reason)."""
        if not self.lease_ok(): return False, f'no control lease ({self.lease_reason})'
        r = self._foreign_block()
        if r == 'unknown': return False, 'Mission Manager not reachable: cannot verify that the robot is free'
        self.foreign_reason = r
        return (r is None), r

    def busy(self):
        with self.lock: return bool(self.goals) or (self.active is not None and not self._terminal(self.active)) or bool(self.orphans)

    def _cancellable(self): return bool(self.active) and not self._terminal(self.active) and self.active.get('status') != 'RECONCILIATION_REQUIRED'

    @staticmethod
    def _terminal(a): return a.get('status') in TERMINAL

    # ---------- loop
    def _loop(self):
        while not self.stop_evt.is_set():
            try: self._tick()
            except Exception as e:
                self.last_error = f'internal: {type(e).__name__}: {e}'; self._ev('internal_error', self.last_error)
            self.stop_evt.wait(self.poll_s)

    def _comm(self, fn, *a):
        """Run an API call; returns (ok, value). A connection problem flips the link state; protocol errors are re-raised."""
        try:
            v = fn(*a); self.comm_fails = 0; return True, v
        except MissionApiUnavailable:
            self.comm_fails += 1
            if self.comm_fails >= self.comm_fail_threshold and self.link != LINK_OFFLINE:
                self.link = LINK_OFFLINE; self._ev('link', 'Mission Manager unreachable: robot reported OFFLINE to RMF; active mission result is UNKNOWN until reconnect')
            return False, None

    def _tick(self):
        if not self.me:
            ok, who = self._comm(self.c.get_me)
            if not ok: return
            self.me = who.get('name')
        ok, robot = self._comm(self.c.get_robot, self.robot_id)
        if not ok: return
        with self.lock:
            prev = self.link; self.robot = robot
            if not robot.get('online'): self.link = LINK_ROBOT_OFFLINE
            elif robot.get('reconciliation_required') or not robot.get('reconciled', True): self.link = LINK_RECONCILE
            else: self.link = LINK_OK
            if prev != self.link: self._ev('link', f'{prev} -> {self.link}')
        self._manage_lease()
        if self.use_lease and not self.lease_ok() and self.link == LINK_OK: self.link = LINK_NO_LEASE
        self._track_active()
        if time.time() - self._foreign_t > 1.0:
            self._foreign_t = time.time(); r = self._foreign_block()
            if r != 'unknown': self.foreign_reason = r
        self._cancel_orphans()
        self._maybe_cancel()
        self._maybe_submit()

    def _track_active(self):
        with self.lock: a = dict(self.active) if self.active else None
        if not a or self._terminal(a): return
        ok, m = self._comm(self.c.get_mission, a['mission_id'])
        if not ok: return
        st = m['status']
        with self.lock:
            if not self.active or self.active['mission_id'] != a['mission_id']: return
            self.active['status'] = st; self.active['detail'] = m.get('detail')
            if a['gen'] == self.gen and not self.cancel_wanted and st in ('EXECUTING', 'ACCEPTED') and self.cb:
                d = ((m.get('progress') or {}).get('distance_remaining_m'))
                if isinstance(d, (int, float)): self.cb.progress(a['gen'], a['idx'], d)
            if st in TERMINAL: self._finish(m)
            elif st == 'RECONCILIATION_REQUIRED':
                self.last_error = f"mission {a['mission_id']} needs reconciliation by the Mission Manager"; self.foreign_reason = self.last_error
                if a['gen'] == self.gen and self.cb: self.cb.done(a['gen'], False, 'RECONCILIATION_REQUIRED'); self.goals, self.idx, self.cb = [], 0, None   # report once
            elif a['gen'] != self.gen and not self.cancel_wanted: self.cancel_wanted = True

    def _finish(self, m):
        a = self.active; st = m['status']; self._ev('mission', f"{a['mission_id']} {st} ({m.get('detail')})")
        stale = a['gen'] != self.gen
        if not stale and self.cb:
            if st == 'SUCCEEDED':
                self.idx += 1
                if self.idx >= len(self.goals): self.cb.done(a['gen'], True, 'arrived'); self.goals, self.idx = [], 0
            else:
                self.cb.done(a['gen'], False, f'{st}: {m.get("detail")}'); self.goals, self.idx = [], 0
        self.cancel_wanted = False
        self.active = None; self._save_state()

    def _cancel_orphans(self):
        for mid in sorted(self.orphans):
            ok, m = self._comm(self.c.get_mission, mid)
            if not ok: return
            if m['status'] in TERMINAL: self.orphans.discard(mid); self._ev('orphan', f'{mid} is {m["status"]}'); self._save_state(); continue
            self.link = LINK_ORPHAN if self.link == LINK_OK else self.link
            if m['status'] != 'CANCELING': self._comm(self.c.cancel_mission, mid); self._ev('orphan', f'cancel requested for {mid} left by a previous adapter run')
        if not self.orphans:
            with self.lock:
                try: os.remove(self.state_path)
                except FileNotFoundError: pass

    def _maybe_cancel(self):
        with self.lock:
            a = self.active
            if not (self.cancel_wanted and a and not self._terminal(a)): return
            if a.get('cancel_sent') and a.get('status') == 'CANCELING': return
            mid = a['mission_id']
        ok, m = self._comm(self.c.cancel_mission, mid)
        if ok:
            with self.lock:
                if self.active and self.active['mission_id'] == mid: self.active['cancel_sent'] = True; self.active['status'] = m['status']
                self._ev('cancel', f'cancel requested for {mid}; waiting for the robot-side result (now {m["status"]})')
                if m['status'] in TERMINAL: self._finish(m)

    def _foreign_block(self):
        ok, ms = self._comm(self.c.list_missions, self.robot_id)
        if not ok: return 'unknown'
        mine = {self.active['mission_id']} if self.active else set()
        for m in ms:
            if m['status'] in TERMINAL: continue
            if m['status'] == 'RECONCILIATION_REQUIRED': return f"mission {m['mission_id']} requires reconciliation"
            if m['mission_id'] in self.orphans or m['mission_id'] in mine: continue
            if not (m.get('idempotency_key') or '').startswith('rmf:') or m.get('created_by') != self.me: return f"mission {m['mission_id']} ({m['status']}) owned by {m.get('created_by')!r} is active"
        return None

    def _maybe_submit(self):
        with self.lock:
            if self.active or self.orphans or self.cancel_wanted or not self.goals or self.link != LINK_OK or not self.lease_ok(): return
            gen, idx = self.gen, self.idx; x, y, yaw = self.goals[idx]
        why = self._foreign_block()
        if why == 'unknown': return
        with self.lock:
            if why:
                if self.link == LINK_OK: self.link = LINK_FOREIGN
                if self.last_error != why: self._ev('blocked', why + ': RMF path is not submitted (no queueing behind another owner)')
                self.last_error = why; return
            if gen != self.gen: return                           # superseded while we were checking
        key = f'rmf:{self.robot_id}:{self.boot_id}:{gen}:{idx}'
        ok, res = self._comm(self.c.create_mission, self.robot_id, {'frame_id': 'map', 'x': x, 'y': y, 'yaw_rad': yaw}, key, f'source=rmf gen={gen} wp={idx}')
        if not ok: return                                       # ambiguous: retried next tick with the SAME key (server returns the same mission)
        m, created = res
        with self.lock:
            self.active = {'mission_id': m['mission_id'], 'key': key, 'gen': gen, 'idx': idx, 'status': m['status'], 'created': created, 'cancel_sent': False, 'detail': m.get('detail'), 't': time.time()}
            self._save_state(); self._ev('submit', f"{m['mission_id']} -> ({x:.2f},{y:.2f}) key={key} created={created}")
            if gen != self.gen: self.cancel_wanted = True
