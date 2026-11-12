"""Agent core (no ROS imports): HTTP client with retry/backoff, state loop, command de-duplication, ack delivery.
Protocol (server side in fleet_server): hello -> state (1 Hz; response carries commands) -> command acks; map upload on change.
Duplicate protection: every command has a command_id; the agent stores the last acknowledged status per command_id in a small JSON file, so a
re-delivered command (lost ack, server restart, agent restart) is NOT executed again: the stored status is re-sent instead."""
import json, math, os, threading, time, urllib.error, urllib.request, uuid

TERMINAL = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED'}


class HttpError(Exception):
    def __init__(self, status, body): super().__init__(f'HTTP {status}: {body[:200]}'); self.status, self.body = status, body


class Http:
    def __init__(self, base_url, token, timeout=4.0): self.base, self.token, self.timeout = base_url.rstrip('/'), token, timeout

    def post(self, path, body, timeout=None):
        req = urllib.request.Request(self.base + '/api/v1' + path, data=json.dumps(body).encode(), method='POST',
                                     headers={'Authorization': 'Bearer ' + self.token, 'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r: return json.loads(r.read())
        except urllib.error.HTTPError as e: raise HttpError(e.code, e.read().decode('utf-8', 'replace'))


class AgentCore:
    def __init__(self, robot_id, base_url, token, snapshot_fn, handler, agent_info, state_dir, map_fn=None, period_s=1.0, log=print):
        self.robot_id, self.http, self.snapshot_fn, self.handler, self.agent_info, self.map_fn, self.period, self.log = robot_id, Http(base_url, token), snapshot_fn, handler, agent_info, map_fn, period_s, log
        self.agent_info = dict(agent_info, protocol_version=2, boot_id=agent_info.get('boot_id') or uuid.uuid4().hex)
        self.boot_id = self.agent_info['boot_id']
        os.makedirs(state_dir, exist_ok=True); self.seen_path = os.path.join(state_dir, f'{robot_id}_commands.json')
        self.journal_ok = True; self.highest_nav_seq = 0
        try:
            with open(self.seen_path) as f: data = json.load(f)
            self.seen = data['commands'] if data.get('version') == 2 else data
            self.highest_nav_seq = data.get('highest_nav_seq', 0) if data.get('version') == 2 else 0
            if not isinstance(self.seen, dict): raise ValueError('invalid journal')
            for record in self.seen.values():
                if record['status'] not in TERMINAL:
                    record.update(status='RECONCILIATION_REQUIRED', detail='agent restarted; Nav2 goal handle not recoverable')
        except FileNotFoundError: self.seen = {}
        except Exception:
            self.seen = {}; self.journal_ok = False  # Fail closed; never overwrite the corrupt evidence.
        self.lock = threading.RLock(); self.pending_acks = []; self.seq = 0; self.running = False; self.connected = False; self.server_boot = None
        self.stop_evt = threading.Event(); self.retry_delay = 0.; self.last_success = None
        self.stats = {'state_posts': 0, 'failures': 0, 'reconnects': 0, 'duplicates_ignored': 0}; self.last_map_version = None

    # ---------------- thread-safe API used by the command handler
    def ack(self, command_id, status, detail='', progress=None):
        with self.lock:
            record = self.seen.setdefault(command_id, {})
            if record.get('status') in TERMINAL and record['status'] != status: return
            record.update(status=status, detail=detail, progress=progress, t=time.time())
            self._save()
            # Only the latest durable status per ID is needed; outages cannot grow an unbounded feedback queue.
            self.pending_acks = [a for a in self.pending_acks if a['command_id'] != command_id]
            self.pending_acks.append({'command_id': command_id, 'status': status, 'detail': detail, 'progress': progress})

    def _save(self):
        if not self.journal_ok: return
        tmp = self.seen_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump({'version': 2, 'highest_nav_seq': self.highest_nav_seq, 'commands': self.seen}, f)
            f.flush(); os.fsync(f.fileno())
        os.replace(tmp, self.seen_path)

    def _flush_acks(self):
        while True:
            with self.lock:
                if not self.pending_acks: return
                a = self.pending_acks[0]
            self.http.post(f"/agent/commands/{a['command_id']}/ack", {'robot_id': self.robot_id, 'agent_boot_id': self.boot_id, 'status': a['status'], 'detail': a['detail'], 'progress': a['progress']})
            with self.lock:
                if self.pending_acks and self.pending_acks[0] is a: self.pending_acks.pop(0)

    def _on_command(self, cmd):
        cid = cmd['command_id']
        with self.lock: prev = self.seen.get(cid)
        if prev is not None:                                   # duplicate delivery: never execute twice
            self.stats['duplicates_ignored'] += 1; self.log(f'duplicate command {cid} ignored (stored status {prev["status"]})')
            if prev['status'] == 'RECEIVED': return
            with self.lock: self.pending_acks.append({'command_id': cid, 'status': prev['status'], 'detail': prev['detail'] + ' (duplicate delivery)', 'progress': None})
            return
        if not self.journal_ok:
            self.ack(cid, 'RECONCILIATION_REQUIRED', 'local journal unreadable; manual investigation required'); return
        if cmd.get('robot_id') != self.robot_id or cmd.get('owner_boot_id') != self.boot_id:
            self.log('command rejected locally: robot/session ownership mismatch'); return
        now = time.time()
        try:
            issued, expires, seq = float(cmd['issued_at']), float(cmd['expires_at']), int(cmd['seq'])
            valid = all(math.isfinite(v) for v in (issued, expires)) and issued <= now + 5 and issued < expires and seq > 0
        except (KeyError, TypeError, ValueError): valid = False; seq = 0; expires = 0
        with self.lock:
            self.seen[cid] = {'status': 'RECEIVED', 'detail': 'received by agent', 't': now,
                              'mission_id': cmd.get('mission_id'), 'type': cmd.get('type'), 'seq': seq}
            blocked = any(r.get('status') == 'RECONCILIATION_REQUIRED' for k, r in self.seen.items() if k != cid)
            if valid and expires >= now and cmd['type'] == 'navigate_to_pose':
                stale = seq <= self.highest_nav_seq
                self.highest_nav_seq = max(self.highest_nav_seq, seq)
            else: stale = False
            self._save()   # Receipt is durable BEFORE the handler can send a goal.
        if not valid or expires < now or now - issued > 120 or stale:
            self.ack(cid, 'EXPIRED' if valid else 'REJECTED', 'expired, stale sequence, or invalid command timestamp'); return
        if blocked and cmd['type'] == 'navigate_to_pose':
            self.ack(cid, 'REJECTED', 'BLOCKED_BY_UNRESOLVED_COMMAND: an earlier execution is still uncertain; no goal was sent'); return      # provably never sent: terminal, does not cascade
        self.log(f"command {cid} {cmd['type']} mission={cmd.get('mission_id')}")
        if cmd['type'] == 'navigate_to_pose': self.handler.start_navigate(cmd, lambda st, d='', pr=None: self.ack(cid, st, d, pr))
        elif cmd['type'] == 'cancel_mission': self.handler.cancel_mission(cmd, lambda st, d='', pr=None: self.ack(cid, st, d, pr))
        else: self.ack(cid, 'REJECTED', f"unknown command type {cmd['type']}")

    def _apply_recovery(self, ids):
        """An admin recovered these commands on the server (after the SERVER saw fresh proof that Nav2 has no active goal). The agent keeps failing closed unless
        ITS OWN current measurement agrees: no goal handle and no active Nav2 goal. The command then becomes terminal FAILED (outcome unverified) in the journal."""
        todo = [c for c in ids if self.seen.get(c, {}).get('status') == 'RECONCILIATION_REQUIRED' or self.seen.get(c, {}).get('status') in ('RECEIVED', 'ACCEPTED', 'EXECUTING', 'SENT')]
        if not todo or not self.journal_ok: return
        ex = self.snapshot_fn().get('execution', {}); g = ex.get('nav2_goals') or {}
        if ex.get('active') or g.get('known') is not True or g.get('active_count') != 0:
            self.log('operator recovery received but local Nav2 evidence does not confirm an idle Nav2; journal kept blocked'); return
        with self.lock:
            for c in todo:
                self.seen[c].update({'status': 'FAILED', 'detail': 'operator recovery acknowledged; outcome unverified'})
            self._save()
        self.log(f'journal: cleared {len(todo)} operator-recovered command(s) after local Nav2 check')

    def _hello(self):
        r = self.http.post('/agent/hello', {'robot_id': self.robot_id, 'agent': self.agent_info})
        if self.server_boot and r.get('server_boot_id') != self.server_boot: self.log('server restarted; reconnected')
        changed = self.server_boot != r.get('server_boot_id')
        self.server_boot = r.get('server_boot_id')
        if changed: self.last_map_version = None
        with self.lock:
            journal = {k: dict(v) for k, v in self.seen.items()}
        result = self.http.post('/agent/reconcile', {'robot_id': self.robot_id, 'agent_boot_id': self.boot_id,
                              'journal': journal, 'journal_ok': self.journal_ok, 'execution': self.snapshot_fn().get('execution', {})})
        for command in result.get('commands', []):
            if command['status'] == 'RECONCILIATION_REQUIRED':
                cid = command['command_id']
                with self.lock:
                    record = self.seen.setdefault(cid, {'t': time.time()})
                    if record.get('status') not in TERMINAL:
                        record.update(status='RECONCILIATION_REQUIRED', detail='server requires reconciliation')
                    self._save()
        self.connected = True; self.log(f'connected and reconciled (boot {self.server_boot})')

    def _maybe_upload_map(self):
        if not self.map_fn: return
        if time.time() - getattr(self, '_last_map_t', 0) < 3.0: return                  # at most one map upload per 3 s
        m = self.map_fn(self.last_map_version)
        if m is None: return
        version, meta, png = m
        import base64
        self.http.post('/agent/map', {'robot_id': self.robot_id, 'agent_boot_id': self.boot_id, 'version': version, 'meta': meta, 'png_base64': base64.b64encode(png).decode()}, timeout=15)
        self.last_map_version = version; self._last_map_t = time.time(); self.log(f'map v{version} uploaded ({len(png)} bytes)')

    def run(self):
        self.running = True; backoff = 1.0
        while self.running:
            t0 = time.time()
            try:
                if not self.connected: self._hello(); self.stats['reconnects'] += 1
                self._flush_acks()
                self.seq += 1
                st = self.snapshot_fn(); st['agent_seq'] = self.seq
                st['communication'] = {'connected': self.connected, 'last_success': self.last_success, 'failures': self.stats['failures'], 'reconnects': self.stats['reconnects'], 'retry_delay_s': self.retry_delay}
                resp = self.http.post('/agent/state', {'robot_id': self.robot_id, 'agent_boot_id': self.boot_id, 'seq': self.seq, 'state': st}); self.stats['state_posts'] += 1
                self.last_success = time.time()
                if resp.get('server_boot_id') != self.server_boot or resp.get('needs_reconcile'):
                    self.connected = False; continue  # Discard deliveries until a fresh ownership reconciliation.
                for c in resp.get('commands', []): self._on_command(c)
                self._apply_recovery(resp.get('recovered_commands') or [])
                self._flush_acks(); self._maybe_upload_map(); backoff = 1.0; self.retry_delay = 0.
            except HttpError as e:
                self.stats['failures'] += 1
                self.connected = False
                self.retry_delay = min(backoff, 10); backoff = min(backoff * 2, 10)
                self.log(f'server error HTTP {e.status}; reconnect in {self.retry_delay:.0f}s'); self.stop_evt.wait(self.retry_delay)
            except Exception as e:
                self.stats['failures'] += 1; self.connected = False
                self.retry_delay = min(backoff, 10); backoff = min(backoff * 2, 10)
                self.log(f'server unreachable ({type(e).__name__}); retry in {self.retry_delay:.0f}s'); self.stop_evt.wait(self.retry_delay)
            self.stop_evt.wait(max(0.0, self.period - (time.time() - t0)))

    def stop(self): self.running = False; self.stop_evt.set()
