"""Fleet logic (no HTTP here, so a future Open-RMF Fleet Adapter or another front-end can use the same operations).

Identifiers: robot_id (agent identity), mission_id (operator/RMF task), command_id (one delivery unit to the agent), idempotency_key
(caller-chosen, unique per robot: re-submitting the same key returns the existing mission, never a second one).
Mission states : QUEUED -> DISPATCHED -> ACCEPTED -> EXECUTING -> SUCCEEDED | FAILED | CANCELED | REJECTED | EXPIRED ; CANCELING in between on cancel.
Command states : PENDING -> SENT -> ACCEPTED -> EXECUTING -> terminal (SUCCEEDED | FAILED | CANCELED | REJECTED | EXPIRED).
The agent pulls commands in the response of every state post; an unacknowledged command is re-sent with the same command_id (the agent de-duplicates)."""
import base64, json, math, threading, time, uuid
from .recovery import RecoveryMixin
from .ownership import OwnershipMixin

MISSION_TERMINAL = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED'}
MISSION_INFLIGHT = {'DISPATCHED', 'ACCEPTED', 'EXECUTING', 'CANCELING', 'RECONCILIATION_REQUIRED'}
CMD_RANK = {'PENDING': 0, 'SENT': 1, 'ACCEPTED': 2, 'EXECUTING': 3, 'SUCCEEDED': 4, 'FAILED': 4, 'CANCELED': 4, 'REJECTED': 4, 'EXPIRED': 4}
CMD_TERMINAL = {k for k, v in CMD_RANK.items() if v == 4}
CMD_RANK['RECONCILIATION_REQUIRED'] = 3
ACK_STATUSES = {'ACCEPTED', 'EXECUTING', 'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED', 'RECONCILIATION_REQUIRED'}


class ApiError(Exception):
    def __init__(self, status, code, message):
        super().__init__(message); self.status, self.code, self.message = status, code, message


def _j(s):
    return json.loads(s) if s else None


class Fleet(RecoveryMixin, OwnershipMixin):
    def __init__(self, db, cfg):
        self.db, self.cfg, self.lock = db, cfg, threading.RLock()
        self.boot_id = uuid.uuid4().hex[:12]
        self.last_persist = {}
        # restore persisted robots (they start as offline until their agent reports again)
        self.robots = {}
        for r in db.q('SELECT * FROM robots'):
            self.robots[r['robot_id']] = {'robot_id': r['robot_id'], 'first_seen': r['first_seen'], 'last_seen': r['last_seen'], 'agent': _j(r['agent_json']) or {},
                                          'state': _j(r['state_json']), 'state_ts': r['state_ts'], 'map_version': r['map_version'], 'map_meta': _j(r['map_meta_json']), 'online': False}
        self.db.event('server_started', f'server started (boot {self.boot_id}); {len(self.robots)} robot(s) restored as offline')

    # ---------------------------------------------------------------- agent side
    def hello(self, robot_id, agent):
        with self.lock:
            now = time.time(); r = self.robots.get(robot_id)
            boot = agent.get('boot_id')
            if boot and self.db.one('SELECT 1 FROM meta WHERE k=?', (f'retired:{robot_id}:{boot}',)):
                raise ApiError(409, 'agent_session_retired', 'a newer agent session superseded this boot')
            old_boot = (r or {}).get('agent', {}).get('boot_id')
            if old_boot and boot != old_boot:
                self.db.x('INSERT OR IGNORE INTO meta(k,v) VALUES(?,?)', (f'retired:{robot_id}:{old_boot}', str(now)))
                for c in self.db.q("SELECT * FROM commands WHERE robot_id=? AND status NOT IN ('PENDING','SUCCEEDED','FAILED','CANCELED','REJECTED','EXPIRED')", (robot_id,)):
                    self.require_reconciliation(c, 'agent session changed; old Nav2 handle cannot be assumed recoverable')
            if r is None:
                r = self.robots[robot_id] = {'robot_id': robot_id, 'first_seen': now, 'last_seen': now, 'agent': agent, 'state': None, 'state_ts': None, 'map_version': None, 'map_meta': None, 'online': False}
                self.db.event('robot_registered', f'robot {robot_id} registered', robot_id)
            r['agent'] = agent; r['last_seen'] = now
            if boot != old_boot: r.pop('seq', None)
            r['reconciled'] = False
            r['reconciliation_required'] = False
            self._set_online(r, True, 'agent hello')
            self._persist(r, force=True)
            return {'server_time': now, 'server_boot_id': self.boot_id, 'heartbeat_s': 1.0, 'offline_after_s': self.cfg.offline_after_s, 'protocol_version': 2}

    def ingest_state(self, robot_id, seq, state, agent_boot_id=None):
        with self.lock:
            r = self.robots.get(robot_id)
            if r is None: raise ApiError(409, 'unknown_robot', 'send hello first')
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
            self.dispatch_next(robot_id)
            res = self._deliver(robot_id)
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
        """Commands for this agent: new ones and re-sends of unacknowledged ones. Marks them SENT."""
        now = time.time(); out = []
        r = self.robots[robot_id]
        if not r.get('reconciled') or r.get('reconciliation_required'):
            return {'ok': True, 'server_time': now, 'server_boot_id': self.boot_id, 'commands': [],
                    'needs_reconcile': not r.get('reconciled'), 'reconciliation_required': bool(r.get('reconciliation_required'))}
        for c in self.db.q("SELECT * FROM commands WHERE robot_id=? AND status IN ('PENDING','SENT') ORDER BY seq", (robot_id,)):
            if c['expires_at'] and now > c['expires_at']: continue
            if c['status'] == 'SENT' and now - (c['sent_at'] or 0) < self.cfg.resend_after_s: continue
            if c['type'] == 'navigate_to_pose' and self.mission(c['mission_id'])['status'] in ('CANCELING', 'RECONCILIATION_REQUIRED'): continue
            owner = r['agent'].get('boot_id')
            self.db.x("UPDATE commands SET status='SENT', sent_at=?, attempts=attempts+1, owner_boot_id=? WHERE command_id=?", (now, owner, c['command_id']))
            if c['status'] == 'PENDING' and c['mission_id']: self._set_mission(c['mission_id'], 'DISPATCHED', 'command delivered to agent', only_if={'QUEUED', 'DISPATCHED'})
            out.append({'command_id': c['command_id'], 'robot_id': robot_id, 'mission_id': c['mission_id'], 'type': c['type'], 'seq': c['seq'], 'attempt': c['attempts'] + 1,
                        'payload': _j(c['payload_json']), 'issued_at': c['created_at'], 'expires_at': c['expires_at'], 'owner_boot_id': owner})
        return {'ok': True, 'server_time': now, 'server_boot_id': self.boot_id, 'commands': out}

    def ack_command(self, robot_id, command_id, status, detail='', progress=None, agent_boot_id=None):
        if status not in ACK_STATUSES: raise ApiError(400, 'bad_status', f'status must be one of {sorted(ACK_STATUSES)}')
        with self.lock:
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
    def create_mission(self, robot_id, mtype, goal, idempotency_key=None, note='', created_by=''):
        if mtype != 'navigate_to_pose': raise ApiError(400, 'bad_type', "only 'navigate_to_pose' is supported in stage 1")
        if not isinstance(goal, dict) or goal.get('frame_id') != 'map': raise ApiError(400, 'bad_goal', "goal.frame_id must be 'map'")
        try: x, y, yaw = float(goal['x']), float(goal['y']), float(goal.get('yaw_rad', 0.0))
        except (KeyError, TypeError, ValueError): raise ApiError(400, 'bad_goal', 'goal needs numeric x, y (and optional yaw_rad)')
        if not all(math.isfinite(v) for v in (x, y, yaw)) or abs(x) > 1000 or abs(y) > 1000: raise ApiError(400, 'bad_goal', 'goal values out of range')
        if idempotency_key is not None and not (isinstance(idempotency_key, str) and 1 <= len(idempotency_key) <= 128): raise ApiError(400, 'bad_key', 'idempotency_key must be a string of 1..128 chars')
        with self.lock:
            if robot_id not in self.robots: raise ApiError(404, 'unknown_robot', f'robot {robot_id} has never registered')
            if idempotency_key:
                ex = self.db.one('SELECT * FROM missions WHERE robot_id=? AND idempotency_key=?', (robot_id, idempotency_key))
                if ex: return self._mission_view(ex), False
            self.check_creator_allowed(robot_id, created_by)          # atomic with the insert below (same lock)
            mid = 'm_' + uuid.uuid4().hex[:12]; now = time.time()
            self.db.x('INSERT INTO missions(mission_id,robot_id,type,goal_json,idempotency_key,status,detail,created_by,note,created_at,updated_at) VALUES(?,?,?,?,?,?,?,?,?,?,?)',
                      (mid, robot_id, mtype, json.dumps({'frame_id': 'map', 'x': x, 'y': y, 'yaw_rad': yaw}), idempotency_key, 'QUEUED', 'created', created_by, note, now, now))
            self.db.event('mission_created', f'mission {mid} -> ({x:.2f}, {y:.2f}, yaw {yaw:.2f}) by {created_by}', robot_id, mid)
            self.dispatch_next(robot_id)
            return self._mission_view(self.db.one('SELECT * FROM missions WHERE mission_id=?', (mid,))), True

    def cancel_mission(self, mission_id, requested_by=''):
        with self.lock:
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
        with self.lock:
            if not self.robots.get(robot_id, {}).get('reconciled') or self.robots[robot_id].get('reconciliation_required'): return
            if self.db.one(f"SELECT 1 AS x FROM missions WHERE robot_id=? AND status IN ({','.join('?' * len(MISSION_INFLIGHT))})", (robot_id, *sorted(MISSION_INFLIGHT))): return
            m = self.db.one("SELECT * FROM missions WHERE robot_id=? AND status='QUEUED' ORDER BY created_at LIMIT 1", (robot_id,))
            if not m or m['cancel_requested']: return
            if self.db.one("SELECT 1 AS x FROM commands WHERE mission_id=? AND type='navigate_to_pose'", (m['mission_id'],)): return
            now = time.time(); cid = 'c_' + uuid.uuid4().hex[:12]
            self.db.x('INSERT INTO commands(command_id,robot_id,mission_id,type,payload_json,status,seq,created_at,expires_at) VALUES(?,?,?,?,?,?,?,?,?)',
                      (cid, robot_id, m['mission_id'], 'navigate_to_pose', json.dumps({'goal': _j(m['goal_json'])}), 'PENDING', self.db.next_seq('cmd_seq'), now, now + self.cfg.command_ttl_s))
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
        with self.lock:
            now = time.time()
            for r in self.robots.values():
                if r['online'] and now - (r['last_seen'] or 0) > self.cfg.offline_after_s:
                    self._set_online(r, False, f'no report for {self.cfg.offline_after_s:.0f} s')
                    r['reconciled'] = False
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
                'lease': self.lease_view(r['robot_id']), 'reconciliation_required': bool(r.get('reconciliation_required')) or bool(self.db.one("SELECT 1 FROM missions WHERE robot_id=? AND status='RECONCILIATION_REQUIRED'", (r['robot_id'],)))}

    def robots_view(self):
        with self.lock: return [self.robot_view(r) for r in sorted(self.robots.values(), key=lambda r: r['robot_id'])]

    def events(self, since_id=0, limit=100):
        rows = self.db.q('SELECT * FROM events WHERE id>? ORDER BY id DESC LIMIT ?', (int(since_id), int(limit)))
        return [{'id': e['id'], 'ts': e['ts'], 'robot_id': e['robot_id'], 'mission_id': e['mission_id'], 'level': e['level'], 'kind': e['kind'], 'message': e['message']} for e in rows]

    def map_png(self, robot_id):
        r = self.db.one('SELECT map_png, map_version, map_meta_json FROM robots WHERE robot_id=?', (robot_id,))
        if not r or not r['map_png']: raise ApiError(404, 'no_map', 'no map received from this robot')
        return r['map_png'], r['map_version'], _j(r['map_meta_json'])
