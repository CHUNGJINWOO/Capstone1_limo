"""Fleet logic (no HTTP here, so a future Open-RMF Fleet Adapter or another front-end can use the same operations).

Identifiers: robot_id (agent identity), mission_id (operator/RMF task), command_id (one delivery unit to the agent), idempotency_key
(caller-chosen, unique per robot: re-submitting the same key returns the existing mission, never a second one).
Mission states : QUEUED -> DISPATCHED -> ACCEPTED -> EXECUTING -> SUCCEEDED | FAILED | CANCELED | REJECTED | EXPIRED ; CANCELING in between on cancel.
Command states : PENDING -> SENT -> ACCEPTED -> EXECUTING -> terminal (SUCCEEDED | FAILED | CANCELED | REJECTED | EXPIRED).
The agent pulls commands in the response of every state post; an unacknowledged command is re-sent with the same command_id (the agent de-duplicates)."""
import base64, copy, json, math, threading, time, uuid
from contextlib import contextmanager
from .recovery import RecoveryMixin
from .ownership import OwnershipMixin
from .safety import SafetyMixin
from .mapping import MappingMixin
from .map_save import MapSaveMixin
from .operating_mode import OperatingModeMixin
from .navigation import NavigationMixin
from .gate_reset import GateResetMixin
from .readiness import ReadinessMixin
from .guide import GuideMixin

MISSION_TERMINAL = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED'}
MISSION_INFLIGHT = {'DISPATCHED', 'ACCEPTED', 'EXECUTING', 'CANCELING', 'RECONCILIATION_REQUIRED'}
CMD_RANK = {'PENDING': 0, 'SENT': 1, 'ACCEPTED': 2, 'EXECUTING': 3, 'SUCCEEDED': 4, 'FAILED': 4, 'CANCELED': 4, 'REJECTED': 4, 'EXPIRED': 4}
CMD_TERMINAL = {k for k, v in CMD_RANK.items() if v == 4}
MANUAL_BENCH_TTL = 0.25
CMD_RANK['RECONCILIATION_REQUIRED'] = 3
ACK_STATUSES = {'ACCEPTED', 'EXECUTING', 'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED', 'RECONCILIATION_REQUIRED'}


class ApiError(Exception):
    def __init__(self, status, code, message):
        super().__init__(message); self.status, self.code, self.message = status, code, message


def _j(s):
    return json.loads(s) if s else None


class Fleet(GuideMixin, ReadinessMixin, GateResetMixin, NavigationMixin, OperatingModeMixin, MapSaveMixin, MappingMixin, SafetyMixin, RecoveryMixin, OwnershipMixin):
    def __init__(self, db, cfg):
        self.db, self.cfg, self.lock = db, cfg, threading.RLock()
        self.boot_id = uuid.uuid4().hex[:12]
        self.last_persist = {}
        self.manual_benches = {}
        self.map_registry = None          # set by the HTTP layer when the Map Registry is enabled
        self._ensure_mapping_schema()
        self._ensure_map_save_schema()
        # restore persisted robots (they start as offline until their agent reports again)
        self.robots = {}
        for r in db.q('SELECT * FROM robots'):
            self.robots[r['robot_id']] = {'robot_id': r['robot_id'], 'first_seen': r['first_seen'], 'last_seen': r['last_seen'], 'agent': _j(r['agent_json']) or {},
                                          'state': _j(r['state_json']), 'state_ts': r['state_ts'], 'map_version': r['map_version'], 'map_meta': _j(r['map_meta_json']), 'online': False}
        with self.lock, self.db.transaction():
            for robot_id in self.robots:
                self.invalidate_movement(robot_id, 'server restarted; prior movement authorization invalid')
        self.db.event('server_started', f'server started (boot {self.boot_id}); {len(self.robots)} robot(s) restored as offline')

    @contextmanager
    def state_transaction(self):
        """Fleet lock must be held before DB lock; fail closed after rollback."""
        robots, persisted = copy.deepcopy(self.robots), dict(self.last_persist)
        try:
            with self.db.transaction():
                yield
        except BaseException:
            self.robots, self.last_persist = robots, persisted
            for r in self.robots.values():
                r['reconciled'] = False
            raise

    # ---------------------------------------------------------------- agent side
    def _manual_bench_robot(self, robot_id):
        robot = self.robots.get(robot_id) or {}
        agent = robot.get('agent') or {}
        age = time.time() - (robot.get('state_ts') or 0)
        real_bench = agent.get('mode') == 'real' and agent.get('simulated') is False
        locked_mock = (agent.get('mode') == 'mock' and agent.get('simulated') is True
                       and agent.get('locked_velocity_arbiter') is True)
        if (agent.get('manual_web_bench') is not True or not (real_bench or locked_mock)
                or (agent.get('missions_enabled') is not False and not locked_mock)
                or not robot.get('online') or not robot.get('reconciled')
                or robot.get('reconciliation_required') or not 0 <= age <= 2):
            self.manual_benches.pop(robot_id, None)
            raise ApiError(409, 'manual_bench_unavailable', 'fresh reconciled real bench Agent required')
        if any(r['status'] not in MISSION_TERMINAL for r in self.db.q(
                'SELECT status FROM missions WHERE robot_id=?', (robot_id,))):
            self.manual_benches.pop(robot_id, None)
            raise ApiError(409, 'robot_busy', 'finish existing missions before the bench test')
        # This endpoint is only a zero-output bench; it does not authorize motion.
        # Deny even bench input when mapping or navigation evidence is uncertain.
        try:
            mapping_busy = self.mapping_blocks_motion(robot_id)
        except Exception:
            self.manual_benches.pop(robot_id, None)
            raise ApiError(409, 'mapping_unknown', 'mapping status unavailable')
        if mapping_busy:
            self.manual_benches.pop(robot_id, None)
            raise ApiError(409, 'mapping_active_or_unknown', 'mapping must be stopped')
        execution = (robot.get('state') or {}).get('execution') or {}
        goals = execution.get('nav2_goals') if isinstance(execution, dict) else None
        goal_age = goals.get('age_s') if isinstance(goals, dict) else None
        # Action servers publish status only on change, so an idle Nav2 is usually silent. The Agent reports
        # continuity evidence (unique status publisher with a stable GID, newest status without active goals);
        # a recently received empty array is the older equivalent.
        idle = execution.get('nav_idle') if isinstance(execution, dict) else None
        continuity_idle = isinstance(idle, dict) and idle.get('ok') is True
        recent_status = (isinstance(goals, dict) and goals.get('received_status_array') is True
                         and type(goal_age) in (int, float) and math.isfinite(goal_age) and 0 <= goal_age + age <= 2)
        if (not isinstance(goals, dict) or goals.get('known') is not True
                or not (continuity_idle or recent_status)
                or type(goals.get('publishers')) is not int or goals['publishers'] != 1
                or type(goals.get('active_count')) is not int or goals['active_count'] != 0
                or execution.get('active') is not None):
            self.manual_benches.pop(robot_id, None)
            reason = (idle or {}).get('reason') if isinstance(idle, dict) else None
            raise ApiError(409, 'nav_status_unknown_or_busy',
                           'idle Nav2 evidence is required for manual input' + (f' ({reason})' if reason else ''))
        if locked_mock:
            arbiter = (robot.get('state') or {}).get('velocity_arbiter') or {}
            if not isinstance(arbiter, dict) or arbiter.get('mode') != 'MOCK_LOCKED' or arbiter.get('ready') is not True:
                self.manual_benches.pop(robot_id, None)
                blocker = arbiter.get('blocker') if isinstance(arbiter, dict) else None
                raise ApiError(409, 'mock_arbiter_not_ready',
                               'locked mock selector is not ready' + (f' ({blocker})' if blocker else ''))
        return agent

    def _nav_summary(self, robot_id):
        rec = self._nav_load(robot_id)
        if not rec:
            return None
        return {k: rec.get(k) for k in ('request_id', 'map_version_id', 'verification_status', 'map_verified', 'initial_pose', 'map_meta')} | {
            'localized': self._nav_localized(robot_id, rec)}

    def control_mode(self, robot_id):
        """Single exclusive operating mode as the server sees it: MAPPING > AUTO > MANUAL > IDLE (UNKNOWN fails closed).

        Admission paths enforce exclusivity (manual start, mission create and mapping START each reject the others);
        this view is what the operator UI shows."""
        r = self.robots.get(robot_id) or {}
        rec = self.mode_view(robot_id, advance=False)
        if rec is not None and rec['mode'] in ('TRANSITIONING', 'FAULT'):
            return rec['mode']
        try:
            if self.mapping_blocks_motion(robot_id):
                return 'MAPPING'
        except Exception:
            return 'UNKNOWN'
        execution = (r.get('state') or {}).get('execution') or {}
        if (isinstance(execution, dict) and execution.get('active') is not None) or any(
                m['status'] not in MISSION_TERMINAL for m in self.db.q('SELECT status FROM missions WHERE robot_id=?', (robot_id,))):
            return 'AUTO'
        manual = self.manual_benches.get(robot_id)
        if manual and manual['until'] > time.monotonic():
            return 'MANUAL'
        return 'IDLE'

    def manual_bench(self, robot_id, actor, body):
        with self.lock:
            lease = self.lease_view(robot_id)
            existing = self.manual_benches.get(robot_id)
            if not lease or (existing and existing['epoch'] != lease['epoch']):
                self.manual_benches.pop(robot_id, None)
            if (not lease or lease['owner'] != actor or type(body.get('epoch')) is not int
                    or lease['epoch'] != body['epoch']):
                raise ApiError(409, 'lease_required', 'current owner and epoch required')
            current = self.manual_benches.get(robot_id)
            action = body.get('action')
            if action != 'stop':
                try:
                    self.require_mode(robot_id, 'MANUAL', 'mode_not_manual')
                except ApiError:
                    self.manual_benches.pop(robot_id, None)
                    raise
            if action == 'stop' and current and body.get('session_id') == current['id'] and current['epoch'] == lease['epoch']:
                self.manual_benches.pop(robot_id)
                self.db.event('manual_bench_stopped', actor, robot_id, data={'session_id': current['id']})
                return {'stopped': True}
            agent = self._manual_bench_robot(robot_id)
            now = time.monotonic()
            if current and (current['boot'] != agent.get('boot_id') or now >= current['until']):
                self.manual_benches.pop(robot_id, None)
                current = None
            if action == 'start':
                if (current and current['epoch'] == lease['epoch'] and current['boot'] == agent['boot_id']
                        and current['until'] > now):
                    raise ApiError(409, 'manual_bench_busy', 'another browser already owns the bench sample stream')
                session = uuid.uuid4().hex
                self.manual_benches[robot_id] = {'id': session, 'epoch': lease['epoch'], 'boot': agent['boot_id'],
                                                  'seq': -1, 'until': now + MANUAL_BENCH_TTL, 'sample': None}
                self.db.event('manual_bench_started', actor, robot_id, data={'session_id': session})
                return {'session_id': session}
            if (action != 'sample' or not current or current['id'] != body.get('session_id')
                    or current['epoch'] != lease['epoch'] or current['boot'] != agent['boot_id']
                    or now >= current['until']):
                raise ApiError(409, 'manual_bench_expired', 'start a fresh bench session')
            seq, linear, turn, deadman = (body.get(k) for k in ('seq', 'linear', 'turn', 'deadman'))
            if (type(seq) is not int or not current['seq'] < seq <= 2**53 - 1
                    or type(deadman) is not bool or any(type(axis) not in (int, float)
                                                        or not -1 <= axis <= 1 for axis in (linear, turn))):
                raise ApiError(400, 'bad_joy_sample', 'expected increasing sequence and valid axes/deadman')
            current.update(seq=seq, until=now + MANUAL_BENCH_TTL,
                           sample={'linear': linear, 'turn': turn, 'deadman': deadman})
            return {'received': seq}

    def pull_manual_bench(self, robot_id, boot_id):
        with self.lock:
            self.check_session(robot_id, boot_id)
            self._manual_bench_robot(robot_id)
            rec = self.mode_view(robot_id)
            if rec is not None and rec['mode'] != 'MANUAL':
                self.manual_benches.pop(robot_id, None)
                return {'sample': None}
            current = self.manual_benches.get(robot_id)
            lease = self.lease_view(robot_id)
            if not lease or not current or current['epoch'] != lease['epoch'] or current['boot'] != boot_id:
                self.manual_benches.pop(robot_id, None)
                return {'sample': None}
            remaining = min(current['until'] - time.monotonic(), lease['expires_at'] - time.time())
            if remaining <= 0:
                self.manual_benches.pop(robot_id, None)
                return {'sample': None}
            if current['sample'] is None:
                return {'sample': None}
            return {'sample': dict(current['sample'], session_id=current['id'], seq=current['seq'],
                                   ttl_s=remaining)}

    def hello(self, robot_id, agent):
        with self.lock, self.state_transaction():
            now = time.time(); r = self.robots.get(robot_id)
            boot = agent.get('boot_id')
            if boot and self.db.one('SELECT 1 FROM meta WHERE k=?', (f'retired:{robot_id}:{boot}',)):
                raise ApiError(409, 'agent_session_retired', 'a newer agent session superseded this boot')
            old_boot = (r or {}).get('agent', {}).get('boot_id')
            # Every accepted hello retires the previous manual session.
            self.manual_benches.pop(robot_id, None)
            if old_boot:
                self.invalidate_movement(robot_id, 'agent reconnected; prior movement authorization invalid')
            if old_boot and boot != old_boot:
                self.db.x('DELETE FROM meta WHERE k=?', ('safety_epoch_invalid:'+robot_id,))
                self.db.x('DELETE FROM meta WHERE k=?', ('safety_epoch_highwater:'+robot_id,))
                self.db.x('INSERT OR IGNORE INTO meta(k,v) VALUES(?,?)', (f'retired:{robot_id}:{old_boot}', str(now)))
                for c in self.db.q("SELECT * FROM commands WHERE robot_id=? AND status NOT IN ('PENDING','SUCCEEDED','FAILED','CANCELED','REJECTED','EXPIRED')", (robot_id,)):
                    self.require_reconciliation(c, 'agent session changed; old Nav2 handle cannot be assumed recoverable')
            if r is None:
                r = self.robots[robot_id] = {'robot_id': robot_id, 'first_seen': now, 'last_seen': now, 'agent': agent, 'state': None, 'state_ts': None, 'map_version': None, 'map_meta': None, 'online': False}
                self.db.event('robot_registered', f'robot {robot_id} registered', robot_id)
            r['agent'] = agent; r['last_seen'] = now
            # A new hello requires fresh state, even if the process retained its boot ID.
            r['state'], r['state_ts'] = None, None
            if boot != old_boot: r.pop('seq', None)
            r['reconciled'] = False
            r['reconciliation_required'] = False
            self._set_online(r, True, 'agent hello')
            self._persist(r, force=True)
            return {'server_time': now, 'server_boot_id': self.boot_id, 'heartbeat_s': 1.0, 'offline_after_s': self.cfg.offline_after_s, 'protocol_version': 2}

    def ingest_state(self, robot_id, seq, state, agent_boot_id=None):
        with self.lock, self.state_transaction():
            r = self.robots.get(robot_id)
            if r is None: raise ApiError(409, 'unknown_robot', 'send hello first')
            if not isinstance(state, dict): raise ApiError(400, 'bad_state', 'state must be an object')
            if r['agent'].get('protocol_version') == 2:
                self.check_session(robot_id, agent_boot_id)
                if seq <= r.get('seq', -1): raise ApiError(409, 'stale_state', 'state sequence must increase')
            now = time.time(); prev = r.get('state') or {}
            r['state'], r['state_ts'], r['last_seen'] = state, now, now
            r['seq'] = seq
            self._set_online(r, True, 'state received')
            self._diff_events(robot_id, prev, state)
            self._persist(r)
            if r.get('reconciled') and r['agent'].get('protocol_version') == 2:
                self.check_execution(robot_id, state.get('execution'))
            # Observe epochs even in fault reports: reset must not replay an older epoch.
            gate = state.get('safety_gate')
            gate = gate if isinstance(gate, dict) else {}
            # A different observed Gate boot is a new, unreconciled generation.
            # Persist the first observed identity so later replay cannot clear
            # the block across Agent or Server restarts. This is not an
            # authenticated anti-rollback source.
            gate_boot = gate.get('boot_id')
            if isinstance(gate_boot, str) and gate_boot.strip() and len(gate_boot) <= 256:
                key = 'observed_gate_boot:' + robot_id
                observed = self.db.one('SELECT v FROM meta WHERE k=?', (key,))
                if observed is None:
                    self.db.x('INSERT INTO meta(k,v) VALUES(?,?)', (key, gate_boot))
                elif observed['v'] != gate_boot:
                    self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', ('gate_boot_reconciliation:'+robot_id, 'gate boot changed; reconciliation required'))
            candidate = {'boot': r['agent'].get('boot_id'), 'fault_epoch': gate.get('fault_epoch'),
                         'map_epoch': state.get('map_epoch'), 'localization_epoch': state.get('localization_epoch'), 'operation_revision': state.get('operation_revision')}
            names = ('fault_epoch', 'map_epoch', 'localization_epoch', 'operation_revision')
            if all(type(candidate[k]) is int and candidate[k] >= 0 for k in names):
                row = self.db.one('SELECT v FROM meta WHERE k=?', ('safety_epoch_highwater:'+robot_id,))
                high = json.loads(row['v']) if row else None
                if high and high['boot'] == candidate['boot'] and any(candidate[k] < high.get(k, 0) for k in names):
                    self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', ('safety_epoch_invalid:'+robot_id, 'epoch regressed within agent session'))
                else:
                    self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', ('safety_epoch_highwater:'+robot_id, json.dumps(candidate)))
            current = self.safety_context(robot_id)
            if current is None:
                current = self.safety_context(robot_id, active_continuation=True)
            previous = self.db.one('SELECT v FROM meta WHERE k=?', ('safety_context:'+robot_id,))
            if current and previous:
                old = json.loads(previous['v'])
                if old['boot'] == current['boot'] and any(current[k] < old[k] for k in ('fault_epoch', 'map_epoch', 'localization_epoch')):
                    self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', ('safety_epoch_invalid:'+robot_id, 'epoch regressed within agent session'))
                    current = None
            if not current or (previous and json.loads(previous['v']) != current):
                self.invalidate_movement(robot_id, 'safety context missing, stale or changed')
            if current:
                self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', ('safety_context:'+robot_id, json.dumps(current)))
            self.mode_view(robot_id)                 # advance a pending mode transition on fresh evidence
            self.dispatch_next(robot_id)
            res = self._deliver(robot_id)
            res['safety_contract_version'] = 1
            res['lease'] = self.lease_view(robot_id)
            res['operating_mode'] = self.agent_mode_view(robot_id)
            res['navigation_request'] = self.navigation_for_agent(robot_id)
            self.gate_reset_ingest(robot_id, state)                      # result for the DELIVERED request only
            res['gate_reset_request'] = self.gate_reset_for_agent(robot_id)   # delivered once, to the Agent boot it was issued for
            res['recovered_commands'] = self.recovered_command_ids(robot_id)         # operator-recovered commands: the agent may clear them from its own journal after re-checking Nav2
            return res

    def ingest_map(self, robot_id, version, meta, png_b64):
        with self.lock:
            r = self.robots.get(robot_id)
            if r is None: raise ApiError(409, 'unknown_robot', 'send hello first')
            png = base64.b64decode(png_b64)
            old = r.get('map_version')
            r['map_version'], r['map_meta'] = version, meta
            self.db.x('UPDATE robots SET map_version=?, map_meta_json=?, map_png=? WHERE robot_id=?', (version, json.dumps(meta), png, robot_id))
            if old != version: self.db.event('map_updated', f'map v{version} {meta.get("width")}x{meta.get("height")} known={meta.get("known_cells")}', robot_id)
            return {'ok': True}

    def _diff_events(self, robot_id, prev, cur):
        def g(d, *path):
            for p in path:
                d = d.get(p) if isinstance(d, dict) else None
            return d
        for label, path in (('safety_gate', ('safety_gate', 'status')), ('nav2', ('nav2', 'status')), ('lidar', ('lidar', 'status')), ('depth_camera', ('depth_camera', 'status')), ('map', ('map', 'status'))):
            a, b = g(prev, *path), g(cur, *path)
            if prev and a != b and b is not None:
                lvl = 'warning' if label == 'safety_gate' and b not in ('NORMAL', 'NO_DATA') else 'info'
                self.db.event(f'{label}_changed', f'{label}: {a} -> {b}', robot_id, level=lvl)
        for e in g(cur, 'errors') or []:
            if e not in (g(prev, 'errors') or []): self.db.event('robot_error', str(e), robot_id, level='warning')

    def _deliver(self, robot_id):
        with self.lock, self.state_transaction():
            return self._deliver_atomic(robot_id)

    def _deliver_atomic(self, robot_id):
        """Commands for this agent: new ones and re-sends of unacknowledged ones. Marks them SENT."""
        now = time.time(); out = []
        r = self.robots[robot_id]
        if not r.get('reconciled') or r.get('reconciliation_required'):
            return {'ok': True, 'server_time': now, 'server_boot_id': self.boot_id, 'commands': [],
                    'needs_reconcile': not r.get('reconciled'), 'reconciliation_required': bool(r.get('reconciliation_required'))}
        for c in self.db.q("SELECT * FROM commands WHERE robot_id=? AND status IN ('PENDING','SENT') ORDER BY seq", (robot_id,)):
            c = self.db.one('SELECT * FROM commands WHERE command_id=?', (c['command_id'],))
            if c['status'] not in ('PENDING', 'SENT'): continue
            if type(c['expires_at']) not in (int, float) or not math.isfinite(c['expires_at']) or c['expires_at'] <= 0:
                if c['type'] == 'navigate_to_pose': self.invalidate_movement(robot_id, 'invalid command expiry')
                continue
            if now >= c['expires_at']: continue
            if c['status'] == 'SENT' and now - (c['sent_at'] or 0) < self.cfg.resend_after_s: continue
            if c['type'] == 'navigate_to_pose':
                m = self.db.one('SELECT * FROM missions WHERE mission_id=?', (c['mission_id'],))
                context = self.safety_context(robot_id)
                if not m or not self.mission_authorized(m, context) or not self.command_context_valid(c, m, context):
                    self.invalidate_movement(robot_id, 'delivery admission context missing, stale or changed')
                    continue
                if m['status'] not in ('QUEUED', 'DISPATCHED'): continue
                execution = (r['state'] or {}).get('execution') or {}
                if c['status'] == 'PENDING' and (execution.get('active') is not None or execution['nav2_goals']['active_count'] != 0):
                    self.invalidate_movement(robot_id, 'Nav2 not idle at delivery')
                    continue
            owner = r['agent'].get('boot_id')
            self.db.x("UPDATE commands SET status='SENT', sent_at=?, attempts=attempts+1, owner_boot_id=? WHERE command_id=?", (now, owner, c['command_id']))
            if c['status'] == 'PENDING' and c['mission_id']: self._set_mission(c['mission_id'], 'DISPATCHED', 'command delivered to agent', only_if={'QUEUED', 'DISPATCHED'})
            out.append({'command_id': c['command_id'], 'robot_id': robot_id, 'mission_id': c['mission_id'], 'type': c['type'], 'seq': c['seq'], 'attempt': c['attempts'] + 1,
                        'payload': _j(c['payload_json']), 'issued_at': c['created_at'], 'expires_at': c['expires_at'], 'owner_boot_id': owner})
        return {'ok': True, 'server_time': now, 'server_boot_id': self.boot_id, 'commands': out}

    def ack_command(self, robot_id, command_id, status, detail='', progress=None, agent_boot_id=None):
        if status not in ACK_STATUSES: raise ApiError(400, 'bad_status', f'status must be one of {sorted(ACK_STATUSES)}')
        with self.lock, self.state_transaction():
            c = self.db.one('SELECT * FROM commands WHERE command_id=? AND robot_id=?', (command_id, robot_id))
            if c is None: raise ApiError(404, 'unknown_command', 'no such command for this robot')
            if agent_boot_id is not None:
                self.check_session(robot_id, agent_boot_id)
                if c['owner_boot_id'] != agent_boot_id: raise ApiError(409, 'command_owner_mismatch', 'command is owned by another agent session')
            cur = c['status']
            if CMD_RANK[status] < CMD_RANK[cur] or (cur in CMD_TERMINAL and status != cur):
                return {'ok': True, 'duplicate': True, 'command_status': cur}                         # stale or duplicate ack: ignore, stay idempotent
            if status == cur and status != 'EXECUTING':                                               # repeated ack of the same state (EXECUTING may repeat: it carries progress)
                return {'ok': True, 'duplicate': True, 'command_status': cur}
            if cur == 'RECONCILIATION_REQUIRED' and status in ('ACCEPTED', 'EXECUTING'):
                return {'ok': True, 'duplicate': True, 'command_status': cur}  # only explicit handle reconciliation may resume
            self.db.x('UPDATE commands SET status=?, detail=?, acked_at=? WHERE command_id=?', (status, detail, time.time(), command_id))
            mid = c['mission_id']
            if c['type'] == 'navigate_to_pose' and mid:
                self._set_mission(mid, status if status != 'CANCELED' else 'CANCELED', detail, progress=progress)
            elif c['type'] == 'cancel_mission' and mid:
                if status in ('SUCCEEDED', 'CANCELED'): self._set_mission(mid, 'CANCELED', detail or 'canceled by agent', only_not_terminal=True)
                elif status == 'REJECTED' and (detail or '').startswith('NOTHING_TO_CANCEL'):
                    self.db.event('cancel_moot', f'cancel of {mid}: {detail}', robot_id, mid)           # the agent itself saw the goal end; the mission keeps its real result
                elif status in ('FAILED', 'REJECTED', 'EXPIRED', 'RECONCILIATION_REQUIRED'):
                    self._set_mission(mid, 'RECONCILIATION_REQUIRED', 'cancel not confirmed: ' + detail, only_not_terminal=True)
                    self.db.event('cancel_failed', f'cancel of {mid} not confirmed: {detail}', robot_id, mid, level='warning')
            if status in CMD_TERMINAL: self.dispatch_next(robot_id)
            return {'ok': True, 'duplicate': False, 'command_status': status}

    # ---------------------------------------------------------------- operator side
    def slam_command(self, robot_id, action, session_id, actor, floor_id=None):
        import re
        if action not in ('start','save','stop') or not isinstance(session_id,str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}',session_id):
            raise ApiError(400,'invalid_slam_request','invalid action or session ID')
        typ='slam_'+action
        if action=='save' and (not isinstance(floor_id,str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}',floor_id)):
            raise ApiError(400,'invalid_floor','valid floor_id is required')
        with self.lock,self.state_transaction():
            r=self.robots.get(robot_id)
            if not r:raise ApiError(404,'unknown_robot','no such robot')
            if action=='start':
                if self.operation_status(robot_id)['state'] not in ('IDLE','STOPPED'):
                    raise ApiError(409,'mapping_active','web mapping session is active or UNKNOWN')
                peer=(r.get('state') or {}).get('recording') or {}
                if peer.get('status') in ('STARTING','RECORDING','STOP_REQUESTED','VERIFYING','UNKNOWN') and peer.get('session_id')!=session_id:
                    raise ApiError(409,'session_mismatch','active recording has a different or unknown session')
                pending=self.db.q("SELECT payload_json FROM commands WHERE robot_id=? AND type='record_start' AND status NOT IN ('SUCCEEDED','FAILED','CANCELED','REJECTED','EXPIRED')",(robot_id,))
                if any((_j(c['payload_json']) or {}).get('session_id')!=session_id for c in pending):raise ApiError(409,'session_mismatch','recording start is pending for another session')
            manual_stop=action=='stop' and bool(((r.get('state') or {}).get('joystick') or {}).get('enabled'))
            if manual_stop and (not isinstance(floor_id,str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}',floor_id)):
                floor_id=None
            commands=self.db.q("SELECT * FROM commands WHERE robot_id=? AND type IN ('slam_start','slam_save','slam_stop') ORDER BY seq DESC",(robot_id,))
            for c in commands:
                if c['type']==typ and (_j(c['payload_json']) or {}).get('session_id')==session_id:
                    if (action=='save' or (manual_stop and c['status']!='FAILED')) and (_j(c['payload_json']) or {}).get('floor_id')!=floor_id:
                        raise ApiError(409,'slam_identity_conflict','same session save has a different floor')
                    if not (manual_stop and c['status']=='FAILED'):
                        return {'command_id':c['command_id'],'status':c['status'],'duplicate':True}
            if any(c['status'] not in CMD_TERMINAL for c in commands):raise ApiError(409,'slam_pending','unresolved SLAM command')
            fresh=r.get('online') and r.get('state_ts') and time.time()-r['state_ts']<=self.cfg.offline_after_s
            state=(r.get('state') or {}).get('slam') if fresh else None
            if not isinstance(state,dict) or not state.get('enabled'):raise ApiError(409,'slam_unavailable','SLAM is unavailable')
            if action=='start' and state.get('status') not in ('IDLE','STOPPED','COMPLETED','FAILED'):raise ApiError(409,'slam_busy','active or unknown session')
            if action in ('save','stop') and (state.get('session_id')!=session_id or state.get('status') not in (('RUNNING',) if action=='save' else (('RUNNING','DEGRADED','STARTING','ERROR') if manual_stop else ('RUNNING','DEGRADED','STARTING')))):
                raise ApiError(409,'slam_session_mismatch','no matching active session')
            now=time.time();cid='c_'+uuid.uuid4().hex[:20]
            payload={'session_id':session_id}
            if action=='save' or manual_stop:payload['floor_id']=floor_id
            if manual_stop:payload['manual_stop']=True
            self.db.x('INSERT INTO commands(command_id,robot_id,mission_id,type,payload_json,status,seq,created_at,expires_at) VALUES(?,?,?,?,?,?,?,?,?)',(cid,robot_id,None,typ,json.dumps(payload),'PENDING',self.db.next_seq('cmd_seq'),now,now+self.cfg.command_ttl_s))
            self.db.event('slam_command',f'{actor}: {typ} {session_id}',robot_id)
            return {'command_id':cid,'status':'PENDING','duplicate':False}

    def recording_status(self, robot_id):
        with self.lock:
            r = self.robots.get(robot_id)
            if not r: raise ApiError(404, 'unknown_robot', 'no such robot')
            fresh = bool(r.get('online') and r.get('state_ts') and time.time() - r['state_ts'] <= self.cfg.offline_after_s)
            state = (r.get('state') or {}).get('recording') if fresh else None
            commands = self.db.q("SELECT command_id,type,status,detail,payload_json FROM commands WHERE robot_id=? AND type IN ('record_start','record_stop') ORDER BY seq DESC LIMIT 10", (robot_id,))
            return {'recording': state if isinstance(state, dict) else {'status': 'UNKNOWN', 'enabled': False, 'detail': 'agent state unavailable'},
                    'commands': [{**{k:c[k] for k in ('command_id','type','status','detail')}, 'session_id': (_j(c['payload_json']) or {}).get('session_id')} for c in commands]}

    def recording_command(self, robot_id, action, session_id, actor):
        import re
        if action not in ('start', 'stop') or not isinstance(session_id, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', session_id):
            raise ApiError(400, 'invalid_recording_request', 'invalid action or session ID')
        typ = 'record_' + action
        with self.lock, self.state_transaction():
            r = self.robots.get(robot_id)
            if not r: raise ApiError(404, 'unknown_robot', 'no such robot')
            if action == 'start':
                peer=(r.get('state') or {}).get('slam') or {}
                if peer.get('status') in ('STARTING','RUNNING','SAVING','DEGRADED','STOP_REQUESTED','UNKNOWN') and peer.get('session_id')!=session_id:
                    raise ApiError(409,'session_mismatch','active SLAM has a different or unknown session')
                pending=self.db.q("SELECT payload_json FROM commands WHERE robot_id=? AND type='slam_start' AND status NOT IN ('SUCCEEDED','FAILED','CANCELED','REJECTED','EXPIRED')",(robot_id,))
                if any((_j(c['payload_json']) or {}).get('session_id')!=session_id for c in pending):raise ApiError(409,'session_mismatch','SLAM start is pending for another session')
            old = self.db.q('SELECT command_id,status,payload_json FROM commands WHERE robot_id=? AND type=? ORDER BY seq DESC', (robot_id, typ))
            for c in old:
                if (_j(c['payload_json']) or {}).get('session_id') == session_id:
                    return {'command_id': c['command_id'], 'status': c['status'], 'duplicate': True}
            view = self.recording_status(robot_id)
            state = view['recording']; status = state.get('status')
            if not state.get('enabled'): raise ApiError(409, 'recording_unavailable', 'recording agent unavailable')
            if any(c['status'] not in CMD_TERMINAL for c in view['commands']):
                raise ApiError(409, 'recording_command_pending', 'previous recording command is unresolved')
            if action == 'start' and status not in ('IDLE', 'VERIFIED', 'FAILED'):
                raise ApiError(409, 'recording_busy', 'recorder active or state unknown')
            if action == 'stop' and (state.get('session_id') != session_id or status not in ('STARTING', 'RECORDING')):
                raise ApiError(409, 'recording_session_mismatch', 'no matching owned active recording')
            now = time.time(); cid = 'c_' + uuid.uuid4().hex[:20]
            self.db.x('INSERT INTO commands(command_id,robot_id,mission_id,type,payload_json,status,seq,created_at,expires_at) VALUES(?,?,?,?,?,?,?,?,?)',
                      (cid, robot_id, None, typ, json.dumps({'session_id': session_id}), 'PENDING', self.db.next_seq('cmd_seq'), now, now + self.cfg.command_ttl_s))
            self.db.event('recording_command', f'{actor}: {typ} {session_id}', robot_id)
            return {'command_id': cid, 'status': 'PENDING', 'duplicate': False}

    def create_mission(self, robot_id, mtype, goal, idempotency_key=None, note='', created_by='', lease_epoch=None):
        if mtype != 'navigate_to_pose': raise ApiError(400, 'bad_type', "only 'navigate_to_pose' is supported in stage 1")
        if not isinstance(goal, dict) or goal.get('frame_id') != 'map': raise ApiError(400, 'bad_goal', "goal.frame_id must be 'map'")
        try: x, y, yaw = float(goal['x']), float(goal['y']), float(goal.get('yaw_rad', 0.0))
        except (KeyError, TypeError, ValueError): raise ApiError(400, 'bad_goal', 'goal needs numeric x, y (and optional yaw_rad)')
        if not all(math.isfinite(v) for v in (x, y, yaw)) or abs(x) > 1000 or abs(y) > 1000: raise ApiError(400, 'bad_goal', 'goal values out of range')
        if idempotency_key is not None and not (isinstance(idempotency_key, str) and 1 <= len(idempotency_key) <= 128): raise ApiError(400, 'bad_key', 'idempotency_key must be a string of 1..128 chars')
        with self.lock, self.db.transaction():
            if robot_id not in self.robots: raise ApiError(404, 'unknown_robot', f'robot {robot_id} has never registered')
            if idempotency_key:
                ex = self.db.one('SELECT * FROM missions WHERE robot_id=? AND idempotency_key=?', (robot_id, idempotency_key))
                if ex:
                    if ex['created_by'] != created_by or _j(ex['goal_json']) != {'frame_id':'map','x':x,'y':y,'yaw_rad':yaw}:
                        raise ApiError(409, 'idempotency_conflict', 'key already belongs to a different request or principal')
                    return self._mission_view(ex), False
            manual = self.manual_benches.get(robot_id)
            if manual and manual['until'] > time.monotonic():
                raise ApiError(409, 'manual_active', 'stop manual input before starting an AUTO mission')
            self.require_mode(robot_id, 'NAVIGATION', 'mode_not_navigation')
            self.require_ready(robot_id, 'mission')          # Phase 4: only with --require-supervisor-ready
            self.require_valid_goal(robot_id, x, y)
            context = self.safety_context(robot_id)
            lease = self.lease_view(robot_id)
            execution = (self.robots[robot_id].get('state') or {}).get('execution') or {}
            if not context or execution.get('active') is not None or execution['nav2_goals']['active_count'] != 0:
                raise ApiError(409, 'safety_not_ready', 'fresh verified safety context and idle Nav2 required')
            if not lease or lease['owner'] != created_by or type(lease_epoch) is not int or lease['epoch'] != lease_epoch:
                raise ApiError(409, 'lease_required', 'current owner and lease epoch required')
            if self.mapping_blocks_motion(robot_id):
                raise ApiError(409, 'mapping_active', 'mapping session must reach STOPPED (UNKNOWN needs reconciliation)')
            mid = 'm_' + uuid.uuid4().hex[:12]; now = time.time()
            self.db.x('INSERT INTO missions(mission_id,robot_id,type,goal_json,idempotency_key,status,detail,created_by,note,created_at,updated_at) VALUES(?,?,?,?,?,?,?,?,?,?,?)',
                      (mid, robot_id, mtype, json.dumps({'frame_id': 'map', 'x': x, 'y': y, 'yaw_rad': yaw}), idempotency_key, 'QUEUED', 'created', created_by, note, now, now))
            self.db.x('INSERT INTO meta(k,v) VALUES(?,?)', ('admission:'+mid, json.dumps({'context':context,'lease_epoch':lease_epoch})))
            self.db.event('mission_created', f'mission {mid} -> ({x:.2f}, {y:.2f}, yaw {yaw:.2f}) by {created_by}', robot_id, mid)
            self.dispatch_next(robot_id)
            return self._mission_view(self.db.one('SELECT * FROM missions WHERE mission_id=?', (mid,))), True

    def cancel_mission(self, mission_id, requested_by=''):
        with self.lock, self.state_transaction():
            m = self.db.one('SELECT * FROM missions WHERE mission_id=?', (mission_id,))
            if m is None: raise ApiError(404, 'unknown_mission', 'no such mission')
            st = m['status']
            if st in MISSION_TERMINAL or st == 'CANCELING': return self._mission_view(m)           # idempotent
            self.db.x('UPDATE missions SET cancel_requested=1 WHERE mission_id=?', (mission_id,))
            self.db.event('mission_cancel_requested', f'cancel requested by {requested_by}', m['robot_id'], mission_id)
            if st == 'QUEUED':
                self._set_mission(mission_id, 'CANCELED', 'canceled before dispatch'); self.db.x("UPDATE commands SET status='CANCELED', detail='mission canceled before delivery' WHERE mission_id=? AND status='PENDING'", (mission_id,))
            else:
                self._set_mission(mission_id, 'CANCELING', 'cancel command queued for the robot')
                cid = 'c_' + uuid.uuid4().hex[:12]; now = time.time()
                self.db.x('INSERT INTO commands(command_id,robot_id,mission_id,type,payload_json,status,seq,created_at,expires_at) VALUES(?,?,?,?,?,?,?,?,?)',
                          (cid, m['robot_id'], mission_id, 'cancel_mission', json.dumps({'mission_id': mission_id}), 'PENDING', self.db.next_seq('cmd_seq'), now, now + self.cfg.command_ttl_s))
            return self._mission_view(self.db.one('SELECT * FROM missions WHERE mission_id=?', (mission_id,)))

    def dispatch_next(self, robot_id):
        with self.lock, self.state_transaction():
            if not self.robots.get(robot_id, {}).get('reconciled') or self.robots[robot_id].get('reconciliation_required'): return
            if self.db.one(f"SELECT 1 AS x FROM missions WHERE robot_id=? AND status IN ({','.join('?' * len(MISSION_INFLIGHT))})", (robot_id, *sorted(MISSION_INFLIGHT))): return
            m = self.db.one("SELECT * FROM missions WHERE robot_id=? AND status='QUEUED' ORDER BY created_at LIMIT 1", (robot_id,))
            if not m or m['cancel_requested']: return
            if self.mapping_blocks_motion(robot_id): return          # mapping owns the goal slot; keep the mission queued
            if not self.mission_authorized(m, self.safety_context(robot_id)):
                self.invalidate_movement(robot_id, 'admission context missing, stale or changed')
                return
            execution = self.robots[robot_id]['state']['execution']
            if execution['active'] is not None or execution['nav2_goals']['active_count'] != 0:
                self.invalidate_movement(robot_id, 'Nav2 not idle at dispatch')
                return
            if self.db.one("SELECT 1 AS x FROM commands WHERE mission_id=? AND type='navigate_to_pose'", (m['mission_id'],)): return
            now = time.time(); cid = 'c_' + uuid.uuid4().hex[:12]
            admission = json.loads(self.db.one('SELECT v FROM meta WHERE k=?', ('admission:'+m['mission_id'],))['v'])
            lease = self.lease_view(robot_id)
            expires = min(now + self.cfg.command_ttl_s, lease['expires_at'])
            safety = {'version': 1, 'server_boot_id': self.boot_id, 'agent_boot_id': admission['context']['boot'],
                      'gate_boot_id': admission['context']['gate_boot_id'], 'fault_epoch': admission['context']['fault_epoch'],
                      'operation_mode': 'NAVIGATION', 'operation_revision': admission['context']['operation_revision'],
                      'map_version_id': admission['context']['map_version_id'], 'map_epoch': admission['context']['map_epoch'],
                      'localization_epoch': admission['context']['localization_epoch'], 'localization_status': 'CONFIRMED',
                      'lease_epoch': admission['lease_epoch'], 'lease_owner': lease['owner'], 'issued_at': now, 'expires_at': expires}
            self.db.x('INSERT INTO commands(command_id,robot_id,mission_id,type,payload_json,status,seq,created_at,expires_at) VALUES(?,?,?,?,?,?,?,?,?)',
                      (cid, robot_id, m['mission_id'], 'navigate_to_pose', json.dumps({'goal': _j(m['goal_json']), 'safety_context': safety}), 'PENDING', self.db.next_seq('cmd_seq'), now, expires))
            self.db.event('command_created', f'command {cid} created for mission {m["mission_id"]}', robot_id, m['mission_id'])

    def _set_mission(self, mid, status, detail='', progress=None, only_if=None, only_not_terminal=False):
        m = self.db.one('SELECT status FROM missions WHERE mission_id=?', (mid,))
        if not m: return
        cur = m['status']
        if cur in MISSION_TERMINAL and cur != status: return                                         # terminal states are final
        if only_if is not None and cur not in only_if: return
        if only_not_terminal and cur in MISSION_TERMINAL: return
        if cur == 'CANCELING' and status in ('ACCEPTED', 'EXECUTING', 'DISPATCHED'): return          # do not go back after a cancel request
        self.db.x('UPDATE missions SET status=?, detail=?, progress_json=COALESCE(?, progress_json), updated_at=? WHERE mission_id=?',
                  (status, detail, json.dumps(progress) if progress is not None else None, time.time(), mid))
        if cur != status:
            row = self.db.one('SELECT robot_id FROM missions WHERE mission_id=?', (mid,))
            self.db.event('mission_status', f'mission {mid}: {cur} -> {status}' + (f' ({detail})' if detail else ''), row['robot_id'], mid, level='warning' if status in ('FAILED', 'REJECTED', 'EXPIRED') else 'info')

    # ---------------------------------------------------------------- housekeeping / views
    def _set_online(self, r, online, why):
        if r['online'] != online:
            r['online'] = online
            self.db.event('robot_online' if online else 'robot_offline', f"robot {r['robot_id']} is {'ONLINE' if online else 'OFFLINE'} ({why})", r['robot_id'], level='info' if online else 'warning')
            self.db.x('UPDATE robots SET online=? WHERE robot_id=?', (1 if online else 0, r['robot_id']))

    def _persist(self, r, force=False):
        now = time.time()
        if not force and now - self.last_persist.get(r['robot_id'], 0) < 2.0: return
        self.last_persist[r['robot_id']] = now
        self.db.x('INSERT INTO robots(robot_id,first_seen,last_seen,agent_json,state_json,state_ts,map_version,map_meta_json,online) VALUES(?,?,?,?,?,?,?,?,?) '
                  'ON CONFLICT(robot_id) DO UPDATE SET last_seen=excluded.last_seen, agent_json=excluded.agent_json, state_json=excluded.state_json, state_ts=excluded.state_ts, online=excluded.online',
                  (r['robot_id'], r['first_seen'], r['last_seen'], json.dumps(r['agent']), json.dumps(r['state']) if r['state'] else None, r['state_ts'], r['map_version'], json.dumps(r['map_meta']) if r['map_meta'] else None, 1 if r['online'] else 0))

    def housekeeping(self):
        with self.lock, self.state_transaction():
            now = time.time()
            for rid in list(self.robots):
                self.mode_view(rid)
            for r in self.robots.values():
                if r['online'] and now - (r['last_seen'] or 0) > self.cfg.offline_after_s:
                    self.manual_benches.pop(r['robot_id'], None)
                    self._set_online(r, False, f'no report for {self.cfg.offline_after_s:.0f} s')
                    r['reconciled'] = False
                    self.invalidate_movement(r['robot_id'], 'communication lost; prior movement authorization invalid')
                    for c in self.db.q("SELECT * FROM commands WHERE robot_id=? AND status IN ('SENT','ACCEPTED','EXECUTING')", (r['robot_id'],)):
                        self.require_reconciliation(c, 'communication lost; execution outcome unknown')
            for c in self.db.q("SELECT * FROM commands WHERE status IN ('PENDING','SENT') AND expires_at IS NOT NULL AND expires_at < ?", (now,)):
                if c['attempts'] or c['type'] == 'cancel_mission':
                    self.require_reconciliation(c, 'command expired after delivery or cancellation outcome unconfirmed')
                    continue
                self.db.x("UPDATE commands SET status='EXPIRED', detail='not acknowledged in time' WHERE command_id=?", (c['command_id'],))
                if c['mission_id']: self._set_mission(c['mission_id'], 'EXPIRED', 'command not acknowledged by the robot in time', only_if={'QUEUED', 'DISPATCHED', 'CANCELING'})
                self.db.event('command_expired', f"command {c['command_id']} expired", c['robot_id'], c['mission_id'], level='warning')
                self.dispatch_next(c['robot_id'])
        try: self.guide_tick()                   # Phase 5: visitor requests follow their missions / queued dispatch
        except Exception as e:
            try: self.db.event('guide_tick_error', f'{type(e).__name__}: {e}'[:200], level='error')
            except Exception: pass

    def _mission_view(self, m):
        cmds = self.db.q('SELECT command_id,type,status,attempts,seq,created_at,sent_at,acked_at,owner_boot_id FROM commands WHERE mission_id=? ORDER BY seq', (m['mission_id'],))
        return {'mission_id': m['mission_id'], 'robot_id': m['robot_id'], 'type': m['type'], 'goal': _j(m['goal_json']), 'status': m['status'], 'detail': m['detail'], 'progress': _j(m.get('progress_json')),
                'idempotency_key': m['idempotency_key'], 'created_by': m['created_by'], 'note': m['note'], 'created_at': m['created_at'], 'updated_at': m['updated_at'],
                'cancel_requested': bool(m['cancel_requested']), 'terminal': m['status'] in MISSION_TERMINAL, 'commands': cmds}

    def missions(self, robot_id=None, status=None, limit=50):
        sql, a = 'SELECT * FROM missions', []
        w = []
        if robot_id: w.append('robot_id=?'); a.append(robot_id)
        if status: w.append('status=?'); a.append(status)
        if w: sql += ' WHERE ' + ' AND '.join(w)
        sql += ' ORDER BY created_at DESC LIMIT ?'; a.append(int(limit))
        return [self._mission_view(m) for m in self.db.q(sql, a)]

    def mission(self, mid):
        m = self.db.one('SELECT * FROM missions WHERE mission_id=?', (mid,))
        if not m: raise ApiError(404, 'unknown_mission', 'no such mission')
        return self._mission_view(m)

    def robot_view(self, r):
        now = time.time()
        return {'robot_id': r['robot_id'], 'online': bool(r['online']), 'last_seen': r['last_seen'], 'last_seen_age_s': None if not r['last_seen'] else round(now - r['last_seen'], 1),
                'first_seen': r['first_seen'], 'agent': r['agent'], 'state': r['state'], 'state_age_s': None if not r['state_ts'] else round(now - r['state_ts'], 1),
                'map_version': r['map_version'], 'map_meta': r['map_meta'], 'reconciled': bool(r.get('reconciled')),
                'lease': self.lease_view(r['robot_id']), 'control_mode': self.control_mode(r['robot_id']),
                'operating_mode': self.mode_view(r['robot_id'], advance=False),
                'safety_context_reason': (getattr(self, 'safety_context_reasons', {}) or {}).get(r['robot_id']),   # diagnostics: first failing admission check
                'gate_boot_reconciliation': bool(self.db.one('SELECT 1 FROM meta WHERE k=?', ('gate_boot_reconciliation:' + r['robot_id'],))),
                'readiness': (lambda rd: {'level': rd['level'], 'reasons': rd['reasons'][:8]})(self.readiness(r['robot_id'])),
                'navigation': self._nav_summary(r['robot_id']), 'reconciliation_required': bool(r.get('reconciliation_required')) or bool(self.db.one("SELECT 1 FROM missions WHERE robot_id=? AND status='RECONCILIATION_REQUIRED'", (r['robot_id'],)))}

    def robots_view(self):
        with self.lock: return [self.robot_view(r) for r in sorted(self.robots.values(), key=lambda r: r['robot_id'])]

    def events(self, since_id=0, limit=100):
        rows = self.db.q('SELECT * FROM events WHERE id>? ORDER BY id DESC LIMIT ?', (int(since_id), int(limit)))
        return [{'id': e['id'], 'ts': e['ts'], 'robot_id': e['robot_id'], 'mission_id': e['mission_id'], 'level': e['level'], 'kind': e['kind'], 'message': e['message']} for e in rows]

    def map_png(self, robot_id):
        r = self.db.one('SELECT map_png, map_version, map_meta_json FROM robots WHERE robot_id=?', (robot_id,))
        if not r or not r['map_png']: raise ApiError(404, 'no_map', 'no map received from this robot')
        return r['map_png'], r['map_version'], _j(r['map_meta_json'])
