"""Agent core (no ROS imports): HTTP client with retry/backoff, state loop, command de-duplication, ack delivery.
Protocol (server side in fleet_server): hello -> state (1 Hz; response carries commands) -> command acks; map upload on change.
Duplicate protection: every command has a command_id; the agent stores the last acknowledged status per command_id in a small JSON file, so a
re-delivered command (lost ack, server restart, agent restart) is NOT executed again: the stored status is re-sent instead."""
import hashlib, json, math, os, threading, time, urllib.error, urllib.request, uuid

from .safety_protocol import (
    SAFETY_CONTRACT_VERSION,
    is_valid_epoch,
    is_valid_id,
    is_valid_timestamp,
    parse_safety_context,
)

TERMINAL = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED'}
STATUS_RANK = {
    'RECEIVED': 0,
    'DISPATCHED': 1,
    'SENT': 1,
    'ACCEPTED': 2,
    'EXECUTING': 3,
    'CANCELING': 3,
    'RECONCILIATION_REQUIRED': 3,
    **{s: 4 for s in TERMINAL},
}


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
    def __init__(self, robot_id, base_url, token, snapshot_fn, handler, agent_info, state_dir, map_fn=None, period_s=1.0, log=print, require_safety_context=False):
        self.robot_id, self.http, self.snapshot_fn, self.handler, self.agent_info, self.map_fn, self.period, self.log = robot_id, Http(base_url, token), snapshot_fn, handler, agent_info, map_fn, period_s, log
        self.require_safety_context = bool(require_safety_context)
        self.agent_info = dict(agent_info, protocol_version=2, safety_contract_version=SAFETY_CONTRACT_VERSION,
                               boot_id=agent_info.get('boot_id') or uuid.uuid4().hex)
        self.boot_id = self.agent_info['boot_id']
        os.makedirs(state_dir, exist_ok=True); self.seen_path = os.path.join(state_dir, f'{robot_id}_commands.json')
        self.journal_ok = True; self.highest_nav_seq = 0
        converted = False
        try:
            with open(self.seen_path) as f: data = json.load(f)
            self.seen = data['commands'] if data.get('version') == 2 else data
            self.highest_nav_seq = data.get('highest_nav_seq', 0) if data.get('version') == 2 else 0
            if not isinstance(self.seen, dict): raise ValueError('invalid journal')
            for record in self.seen.values():
                if record.get('identity_conflict') is True:
                    self.journal_ok = False
                if record['status'] not in TERMINAL:
                    record.update(status='RECONCILIATION_REQUIRED', detail='agent restarted; Nav2 goal handle not recoverable')
                    converted = True
        except FileNotFoundError: self.seen = {}
        except Exception:
            self.seen = {}; self.journal_ok = False  # Fail closed; never overwrite the corrupt evidence.
        self.lock = threading.RLock(); self.pending_acks = []; self.seq = 0; self.running = False; self.connected = False; self.server_boot = None
        self.lease = None; self.operating_mode = None; self.navigation_request = None; self.gate_reset_request = None; self.server_safety_contract_version = None; self.last_state_ok_mono = None
        self.state_post_rtt_ms = None
        self.server_reconciliation_required = True
        self.stop_evt = threading.Event(); self.retry_delay = 0.; self.last_success = None
        self.stats = {'state_posts': 0, 'failures': 0, 'reconnects': 0, 'duplicates_ignored': 0}; self.last_map_version = None
        if converted and self.journal_ok:
            self._save()

    # ---------------- thread-safe API used by the command handler
    def ack(self, command_id, status, detail='', progress=None):
        with self.lock:
            # Never report a result that cannot be durably journaled. The
            # server will reconcile an unacknowledged command after timeout.
            if not self.journal_ok:
                self.log('ack withheld: local journal is unavailable')
                return False
            record = self.seen.setdefault(command_id, {})
            cur = record.get('status')
            if cur in TERMINAL:
                return cur == status
            if cur == 'RECONCILIATION_REQUIRED' and status not in TERMINAL:
                # Late goal acceptances, feedback, or cancel acks must never clear unknown outcome without a terminal result.
                return False
            if cur in STATUS_RANK and status in STATUS_RANK and STATUS_RANK[status] < STATUS_RANK[cur]:
                return False
            record.update(status=status, detail=detail, progress=progress, t=time.time())
            try:
                self._save()
            except (OSError, ValueError, TypeError):
                self.pending_acks.clear()
                self.log('ack withheld: journal persistence failed')
                return False
            # Only the latest durable status per ID is needed; outages cannot grow an unbounded feedback queue.
            self.pending_acks = [a for a in self.pending_acks if a['command_id'] != command_id]
            self.pending_acks.append({'command_id': command_id, 'status': status, 'detail': detail, 'progress': progress})
            return True

    def record_command_meta(self, command_id, **fields):
        """Persist non-status metadata (e.g., goal_uuid, safety_context) in the atomic journal."""
        with self.lock:
            if not self.journal_ok or command_id not in self.seen:
                raise RuntimeError('command journal unavailable or command missing')
            if fields.get('submission_intent') is True:
                if self.seen[command_id].get('submission_intent') is True:
                    raise RuntimeError('submission intent already reserved')
                if any(cid != command_id and rec.get('submission_intent') is True and rec.get('status') not in TERMINAL
                       for cid, rec in self.seen.items()):
                    raise RuntimeError('another command has an unresolved submission reservation')
            self.seen[command_id].update(fields)
            try:
                self._save()
            except Exception:
                self.journal_ok = False
                raise

    def session_snapshot(self):
        with self.lock:
            age = None if self.last_state_ok_mono is None else max(0.0, time.monotonic() - self.last_state_ok_mono)
            return {
                'connected': bool(self.connected),
                'reconciliation_required': self.server_reconciliation_required,
                'server_boot_id': self.server_boot,
                'agent_boot_id': self.boot_id,
                'last_state_age_s': None if age is None else round(age, 3),
                'lease': dict(self.lease) if isinstance(self.lease, dict) else None,
                'operating_mode': dict(self.operating_mode) if isinstance(getattr(self, 'operating_mode', None), dict) else None,
                'navigation_request': dict(self.navigation_request) if isinstance(getattr(self, 'navigation_request', None), dict) else None,
                'gate_reset_request': dict(self.gate_reset_request) if isinstance(getattr(self, 'gate_reset_request', None), dict) else None,
                'safety_contract_version': self.server_safety_contract_version,
            }

    def is_execution_allowed(self, max_age_s=2.0):
        with self.lock:
            if not self.connected or self.server_reconciliation_required or self.last_state_ok_mono is None:
                return False
            return (time.monotonic() - self.last_state_ok_mono) <= max_age_s

    def _save(self):
        if not self.journal_ok: return
        try:
            tmp = self.seen_path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump({'version': 2, 'highest_nav_seq': self.highest_nav_seq, 'commands': self.seen}, f)
                f.flush(); os.fsync(f.fileno())
            os.replace(tmp, self.seen_path)
            directory_fd = os.open(os.path.dirname(self.seen_path), os.O_RDONLY | os.O_DIRECTORY)
            try: os.fsync(directory_fd)
            finally: os.close(directory_fd)
        except Exception:
            self.journal_ok = False
            raise

    def _flush_acks(self):
        while True:
            with self.lock:
                if not self.journal_ok or not self.pending_acks: return
                a = self.pending_acks[0]
            self.http.post(f"/agent/commands/{a['command_id']}/ack", {'robot_id': self.robot_id, 'agent_boot_id': self.boot_id, 'status': a['status'], 'detail': a['detail'], 'progress': a['progress']})
            with self.lock:
                if self.pending_acks and self.pending_acks[0] is a: self.pending_acks.pop(0)

    def _on_command(self, cmd):
        if not isinstance(cmd, dict) or not is_valid_id(cmd.get('command_id')):
            self.log('command rejected locally: invalid command envelope'); return
        cid = cmd['command_id']
        identity_fields = ('robot_id', 'mission_id', 'type', 'seq', 'payload', 'issued_at', 'expires_at', 'owner_boot_id',
                           'owner_type', 'owner_id', 'operation_id')
        try:
            identity = hashlib.sha256(json.dumps({k: cmd.get(k) for k in identity_fields}, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
        except (TypeError, ValueError):
            self.log('command rejected locally: invalid immutable command identity'); return
        with self.lock:
            prev = self.seen.get(cid)
            if prev is not None:                                   # duplicate delivery: never execute twice
                previous_identity = prev.get('identity_sha256')
                if previous_identity is not None and previous_identity != identity:
                    prev['identity_conflict'] = True
                    try: self._save()
                    finally: self.journal_ok = False
                    self.log(f'command {cid} identity conflict; journal blocked')
                    return
                if previous_identity is None and cmd.get('type') == 'navigate_to_pose':
                    self.log(f'command {cid} legacy identity unknown; duplicate not replayed')
                    return
                self.stats['duplicates_ignored'] += 1; self.log(f'duplicate command {cid} ignored (stored status {prev["status"]})')
                if prev['status'] == 'RECEIVED': return
                self.pending_acks.append({'command_id': cid, 'status': prev['status'], 'detail': prev['detail'] + ' (duplicate delivery)', 'progress': None})
                return
        if not self.journal_ok:
            self.ack(cid, 'RECONCILIATION_REQUIRED', 'local journal unreadable; manual investigation required'); return
        if cmd.get('robot_id') != self.robot_id or cmd.get('owner_boot_id') != self.boot_id:
            self.log('command rejected locally: robot/session ownership mismatch'); return
        now = time.time()
        issued_raw, expires_raw, seq_raw = cmd.get('issued_at'), cmd.get('expires_at'), cmd.get('seq')
        if is_valid_timestamp(issued_raw) and is_valid_timestamp(expires_raw) and is_valid_epoch(seq_raw) and seq_raw > 0:
            issued, expires, seq = float(issued_raw), float(expires_raw), seq_raw
            valid = issued <= now + 5 and issued < expires
        else:
            valid, issued, expires, seq = False, 0.0, 0.0, 0
        payload = cmd.get('payload') if isinstance(cmd.get('payload'), dict) else {}
        sc_raw = payload.get('safety_context')
        sc = None
        sc_status = 'OK'
        sc_detail = ''
        if cmd.get('type') == 'navigate_to_pose' and (self.require_safety_context or sc_raw is not None):
            sc, sc_err = parse_safety_context(sc_raw)
            if sc is None:
                sc_status, sc_detail = 'REJECTED', sc_err
            elif sc['agent_boot_id'] != self.boot_id or sc['server_boot_id'] != self.server_boot:
                sc_status, sc_detail = 'REJECTED', 'SESSION_MISMATCH: safety_context boot_id mismatch'
            elif float(sc['issued_at']) > now + 5:
                sc_status, sc_detail = 'REJECTED', 'CLOCK_SKEW: safety_context issued_at is in the future'
            elif float(sc['expires_at']) <= now:
                sc_status, sc_detail = 'EXPIRED', 'EXPIRED: safety_context expired on receipt'
        with self.lock:
            if cid in self.seen:
                # Another delivery may have reserved the same ID since the
                # initial duplicate check; re-check its immutable identity.
                return self._on_command(cmd)
            entry = {'status': 'RECEIVED', 'detail': 'received by agent', 't': now,
                     'mission_id': cmd.get('mission_id'), 'type': cmd.get('type'), 'seq': seq, 'identity_sha256': identity}
            if sc is not None:
                entry['safety_context'] = sc
            self.seen[cid] = entry
            blocked = any(r.get('status') == 'RECONCILIATION_REQUIRED' for k, r in self.seen.items() if k != cid)
            if valid and sc_status == 'OK' and expires > now and cmd['type'] == 'navigate_to_pose':
                stale = seq <= self.highest_nav_seq
                self.highest_nav_seq = max(self.highest_nav_seq, seq)
            else: stale = False
            self._save()   # Receipt is durable BEFORE the handler can send a goal.
        if not valid or expires <= now or now - issued > 120 or stale:
            self.ack(cid, 'EXPIRED' if valid else 'REJECTED', 'expired, stale sequence, or invalid command timestamp'); return
        if sc_status != 'OK':
            self.ack(cid, sc_status, sc_detail); return
        if blocked and cmd['type'] == 'navigate_to_pose':
            self.ack(cid, 'REJECTED', 'BLOCKED_BY_UNRESOLVED_COMMAND: an earlier execution is still uncertain; no goal was sent'); return      # provably never sent: terminal, does not cascade
        self.log(f"command {cid} {cmd['type']} mission={cmd.get('mission_id')}")
        if cmd['type'] == 'navigate_to_pose': self.handler.start_navigate(cmd, lambda st, d='', pr=None: self.ack(cid, st, d, pr))
        elif cmd['type'] == 'cancel_mission': self.handler.cancel_mission(cmd, lambda st, d='', pr=None: self.ack(cid, st, d, pr))
        elif cmd['type'] in ('record_start', 'record_stop') and hasattr(self.handler, 'recording_command'):
            self.handler.recording_command(cmd, lambda st, d='', pr=None: self.ack(cid, st, d, pr))
        elif cmd['type'] in ('slam_start', 'slam_save', 'slam_stop') and hasattr(self.handler, 'slam_command'):
            self.handler.slam_command(cmd, lambda st, d='', pr=None: self.ack(cid, st, d, pr))
        else: self.ack(cid, 'REJECTED', f"unknown command type {cmd['type']}")

    def _apply_recovery(self, ids):
        """Clear a locally unresolved command ONLY with this Agent's own Nav2 evidence (2026-11-20).

        The server's recovery hint alone never ends a local goal (an empty status array proves nothing). An entry is
        resolved only if (1) the server lists it as operator-recovered, (2) it is RECONCILIATION_REQUIRED here with a
        recorded goal UUID, and (3) the NavigateToPose status array observed by THIS Agent reports that exact UUID in a
        terminal state. Anything else stays blocked."""
        with self.lock:
            pending = [c for c in ids if (self.seen.get(c) or {}).get('status') == 'RECONCILIATION_REQUIRED']
        if not pending:
            return
        try:
            observed = (self.snapshot_fn().get('execution') or {}).get('nav_terminal_observed') or {}
        except Exception:
            observed = {}
        changed = []
        with self.lock:
            for cid in pending:
                rec = self.seen.get(cid) or {}
                uid = rec.get('goal_uuid')
                st = observed.get(uid) if isinstance(uid, str) else None
                if st in ('SUCCEEDED', 'CANCELED', 'FAILED'):
                    rec.update(status=st, terminal_status=st, terminal_source='nav2_status_array', terminal_agent_boot_id=self.boot_id,
                               detail='operator recovery confirmed by the Nav2 status array observed by this agent')
                    changed.append((cid, st))
            if changed:
                self._save()
        for cid, st in changed:
            self.log(f'operator recovery of {cid} confirmed locally: Nav2 reported {st}')
        if len(changed) < len(pending):
            self.log('operator recovery hint ignored for commands without Nav2 terminal evidence')

    def _hello(self):
        with self.lock:
            self.connected = False
            self.lease = None; self.operating_mode = None; self.navigation_request = None; self.gate_reset_request = None
            self.server_safety_contract_version = None
            self.last_state_ok_mono = None
        r = self.http.post('/agent/hello', {'robot_id': self.robot_id, 'agent': self.agent_info})
        if self.server_boot and r.get('server_boot_id') != self.server_boot: self.log('server restarted; reconnected')
        changed = self.server_boot != r.get('server_boot_id')
        self.server_boot = r.get('server_boot_id')
        self.server_safety_contract_version = r.get('safety_contract_version')
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
        with self.lock:
            self.server_reconciliation_required = result.get('reconciliation_required') is not False
            self.connected = True
            self.last_state_ok_mono = time.monotonic()
        self.log(f'connected and reconciled (boot {self.server_boot})')

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
                # Actual durable result records, not cancel-service ACKs. Bounded
                # telemetry for explicit operator recovery in SIM_ONLY tests.
                with self.lock:
                    if self.journal_ok and isinstance(st.get('execution'), dict):
                        st['execution']['journal_terminals'] = [
                            dict(command_id=cid, **{k: rec.get(k) for k in
                                 ('mission_id', 'goal_uuid', 'terminal_status', 'terminal_source',
                                  'terminal_agent_boot_id', 'safety_context', 'status')})
                            for cid, rec in list(self.seen.items())[-20:]
                            if rec.get('terminal_source') == 'nav2_action_result'
                            and rec.get('status') in ('SUCCEEDED', 'CANCELED', 'FAILED')]

                st['communication'] = {'connected': self.connected, 'last_success': self.last_success, 'failures': self.stats['failures'], 'reconnects': self.stats['reconnects'], 'retry_delay_s': self.retry_delay}
                st['communication']['previous_state_post_rtt_ms'] = self.state_post_rtt_ms
                post_started = time.monotonic()
                resp = self.http.post('/agent/state', {'robot_id': self.robot_id, 'agent_boot_id': self.boot_id, 'seq': self.seq, 'state': st}); self.stats['state_posts'] += 1
                self.state_post_rtt_ms = round((time.monotonic() - post_started) * 1000, 1)
                self.last_success = time.time()
                if resp.get('server_boot_id') != self.server_boot or resp.get('needs_reconcile'):
                    with self.lock:
                        self.connected = False
                        self.lease = None; self.operating_mode = None; self.navigation_request = None; self.gate_reset_request = None
                        self.server_safety_contract_version = None
                        self.last_state_ok_mono = None
                    continue  # Discard deliveries until a fresh ownership reconciliation.
                with self.lock:
                    self.server_reconciliation_required = resp.get('reconciliation_required') is True
                    self.last_state_ok_mono = time.monotonic()
                    if 'lease' in resp:
                        self.lease = resp.get('lease')
                    self.operating_mode = resp.get('operating_mode') if isinstance(resp.get('operating_mode'), dict) else None
                    self.navigation_request = resp.get('navigation_request') if isinstance(resp.get('navigation_request'), dict) else None
                    self.gate_reset_request = resp.get('gate_reset_request') if isinstance(resp.get('gate_reset_request'), dict) else None
                    if 'safety_contract_version' in resp:
                        self.server_safety_contract_version = resp.get('safety_contract_version')
                for c in resp.get('commands', []): self._on_command(c)
                self._apply_recovery(resp.get('recovered_commands') or [])
                self._flush_acks(); self._maybe_upload_map(); backoff = 1.0; self.retry_delay = 0.
            except HttpError as e:
                self.stats['failures'] += 1
                with self.lock:
                    self.connected = False
                    self.lease = None; self.operating_mode = None; self.navigation_request = None; self.gate_reset_request = None
                    self.last_state_ok_mono = None
                self.retry_delay = min(backoff, 10); backoff = min(backoff * 2, 10)
                self.log(f'server error HTTP {e.status}; reconnect in {self.retry_delay:.0f}s'); self.stop_evt.wait(self.retry_delay)
            except Exception as e:
                self.stats['failures'] += 1
                with self.lock:
                    self.connected = False
                    self.lease = None; self.operating_mode = None; self.navigation_request = None; self.gate_reset_request = None
                    self.last_state_ok_mono = None
                self.retry_delay = min(backoff, 10); backoff = min(backoff * 2, 10)
                self.log(f'server unreachable ({type(e).__name__}); retry in {self.retry_delay:.0f}s'); self.stop_evt.wait(self.retry_delay)
            self.stop_evt.wait(max(0.0, self.period - (time.time() - t0)))

    def stop(self): self.running = False; self.stop_evt.set()
