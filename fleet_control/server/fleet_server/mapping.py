"""Web-controlled mapping sessions (START / PAUSE / RESUME / SAVE / STOP).

Ported selectively from origin/main `fleet_server/mapping.py` (ea2d963) onto the local Fleet:
- one durable session row per robot (`operation_sessions`) and an idempotency journal (`operation_requests`);
- HTTP acceptance (QUEUED) is separate from the worker result (COMPLETED / FAILED / UNKNOWN);
- every request needs the current Lease owner + epoch; non-START requests need the current session id;
- worker evidence decides the outcome; missing evidence for a goal-stopping action yields UNKNOWN;
- UNKNOWN is sticky: no API clears it and START is refused until an operator reconciles out of band.

Local adaptations:
- only the `mapping` kind (local `recording_session` already owns rosbag control);
- the worker is accepted only from SIM/mock or isolated agents that advertise `mapping_worker: true`
  (real-robot mapping stays blocked until field safety conditions are cleared);
- Fleet navigation missions and the sim `slam_*` commands are mutually exclusive with a live session;
- SAVE may name a registered floor (and area); the worker then uploads the saved bundle and it is
  registered as an UNVERIFIED MapVersion through the existing Map Registry.
"""
import json
import re
import time
import uuid

STABLE = {'START': {'IDLE', 'STOPPED'}, 'PAUSE': {'MAPPING'}, 'RESUME': {'PAUSED'},
          'SAVE': {'PAUSED'}, 'STOP': {'MAPPING', 'PAUSED'}, 'RECONCILE': {'UNKNOWN'}}
PENDING = dict(START='STARTING', PAUSE='PAUSING', RESUME='STARTING', SAVE='SAVING', STOP='STOPPING', RECONCILE='RECONCILING')
DONE = dict(START='MAPPING', PAUSE='PAUSED', RESUME='MAPPING', SAVE='PAUSED', STOP='STOPPED', RECONCILE='STOPPED')
REQUIRED = {
    'START': ('slam_running', 'frontier_running'),
    'RESUME': ('slam_running', 'frontier_running'),
    'PAUSE': ('frontier_paused', 'goal_terminal'),
    'SAVE': ('yaml_ok', 'pgm_ok', 'posegraph_ok'),
    'STOP': ('frontier_stopped', 'goal_terminal'),
    # Operator reconciliation of an UNKNOWN session (e.g. after an Agent restart or a lost Lease): the worker of the
    # CURRENT Agent boot proves its owned processes are gone, no SLAM/Frontier node remains in the graph and no
    # NavigateToPose/FollowPath goal is active. Never automatic; failure keeps the session UNKNOWN.
    'RECONCILE': ('processes_stopped', 'graph_clear', 'goal_terminal'),
}
MISSION_DONE = ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED')
GOAL_DONE = ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED')
COMMAND_TTL_S = 20.0
MODES = ('NEW_MAP', 'CONTINUE_MAP')
LIMIT_DEFAULTS = {'max_duration_s': 900.0, 'max_goals': 40, 'min_battery_v': None}
LIMIT_RANGES = {'max_duration_s': (30.0, 3600.0), 'max_goals': (1, 200), 'min_battery_v': (9.0, 13.0)}
WORKER_STALE_S = 3.0
RENEW_FENCE_S = 3.0          # worker fence after a renew; capped by the Lease's remaining time
ID_RE = re.compile(r'[A-Za-z0-9][A-Za-z0-9._-]{0,63}')

SCHEMA = """
CREATE TABLE IF NOT EXISTS operation_sessions(robot_id TEXT, kind TEXT, body TEXT NOT NULL, PRIMARY KEY(robot_id,kind));
CREATE TABLE IF NOT EXISTS operation_requests(robot_id TEXT, kind TEXT, actor TEXT, idem TEXT, body TEXT NOT NULL,
  PRIMARY KEY(robot_id,kind,actor,idem));
"""


class MappingMixin:
    KIND = 'mapping'

    def _ensure_mapping_schema(self):
        with self.db.lock:
            self.db.c.executescript(SCHEMA)

    # ------------------------------------------------------------ status
    def operation_status(self, robot_id, kind='mapping'):
        from .service import ApiError

        with self.lock:
            if robot_id not in self.robots:
                raise ApiError(404, 'unknown_robot', 'robot not registered')
            row = self.db.one('SELECT body FROM operation_sessions WHERE robot_id=? AND kind=?', (robot_id, kind))
            if not row:
                return {'robot_id': robot_id, 'kind': kind, 'state': 'IDLE'}
            session = json.loads(row['body'])
            robot = self.robots[robot_id]
            lease = self.lease_view(robot_id)
            command = session['command']
            worker = ((robot.get('state') or {}).get('mapping_worker') or {}).get('status')
            if session['state'] not in ('STOPPED', 'UNKNOWN') and (
                    (worker in ('LOST', 'UNKNOWN', 'FAILED') and session['state'] != 'RECONCILING')   # UNKNOWN is what RECONCILE resolves
                    or session.get('server_boot') != self.boot_id
                    or time.time() - (robot.get('state_ts') or 0) > WORKER_STALE_S
                    or not robot.get('reconciled') or robot.get('reconciliation_required')
                    or (robot.get('agent') or {}).get('boot_id') != session['boot']
                    or not lease or lease['epoch'] != session['epoch']
                    or (command['status'] in ('QUEUED', 'ACCEPTED') and time.time() >= command['expires_at'])):
                session['state'] = 'UNKNOWN'
                if command['status'] in ('QUEUED', 'ACCEPTED'):
                    command['status'] = 'UNKNOWN'
                session['detail'] = 'Worker, lease, or command evidence expired. Reconciliation required.'
                self._write_operation(session)
                self.db.event('mapping_unknown', session['detail'], robot_id, data={'session_id': session['session_id']},
                              level='warning')
            return session

    def owns_mapping_goal(self, robot_id, boot, active):
        """A Frontier goal reported as `active` is owned only by the live session's reserved goal on the same boot."""
        if not isinstance(active, dict) or active.get('owner') != 'frontier':
            return False
        robot = self.robots.get(robot_id) or {}
        if (robot.get('agent') or {}).get('mapping_worker') is not True:
            return False
        s = self.operation_status(robot_id)
        # UNKNOWN keeps ownership of the reserved goal: the session blocks motion and START, but the goal is ours
        # (same boot, session and reserved id) and the worker is halting it, so it is not an unowned-goal fault.
        return (s['state'] in ('MAPPING', 'PAUSING', 'STOPPING', 'UNKNOWN', 'RECONCILING') and s.get('boot') == boot
                and s.get('session_id') == active.get('mission_id') and s.get('owned_goal') == active.get('command_id'))

    def mapping_blocks_motion(self, robot_id):
        """Any non-final session (including UNKNOWN) owns the goal slot; Fleet navigation must wait."""
        if robot_id not in self.robots:
            return False
        return self.operation_status(robot_id)['state'] not in ('IDLE', 'STOPPED')

    def _write_operation(self, session, new_command=False):
        with self.db.transaction():  # nests as a savepoint inside mission/state transactions
            self.db.x('INSERT INTO operation_sessions(robot_id,kind,body) VALUES(?,?,?) '
                      'ON CONFLICT(robot_id,kind) DO UPDATE SET body=excluded.body',
                      (session['robot_id'], session['kind'], json.dumps(session)))
            if new_command:
                c = session['command']
                self.db.x('INSERT INTO operation_requests(robot_id,kind,actor,idem,body) VALUES(?,?,?,?,?)',
                          (session['robot_id'], session['kind'], session['owner'], c['idempotency_key'], json.dumps(c)))

    def _mapping_real_block(self, robot_id):
        """Raise mapping_real_blocked for an Agent that reports a real robot, BEFORE the readiness check (2026-11-21: readiness is
        now enforced for real Agents, and must not hide the explicit real-robot block reason). The full check stays in
        _mapping_worker_robot."""
        from .service import ApiError
        agent = (self.robots.get(robot_id) or {}).get('agent') or {}
        if agent.get('mode') == 'real' or agent.get('simulated') is False:
            raise ApiError(409, 'mapping_real_blocked', 'real-robot mapping is blocked until field safety conditions clear')

    def _mapping_worker_robot(self, robot_id):
        from .service import ApiError

        r = self.robots.get(robot_id) or {}
        agent = r.get('agent') or {}
        if agent.get('mapping_worker') is not True:
            raise ApiError(409, 'worker_unavailable', 'agent does not run a mapping worker')
        # Real-robot mapping stays blocked: power cut-outs, USB-UART loss, Depth jitter and the physical
        # E-Stop are unresolved. Only simulated/mock or isolated-ROS agents may host the worker.
        if agent.get('simulated') is not True or agent.get('mode') not in ('mock', 'sim'):
            raise ApiError(409, 'mapping_real_blocked', 'real-robot mapping is blocked until field safety conditions clear')
        if (not r.get('reconciled') or r.get('reconciliation_required')
                or not 0 <= time.time() - (r.get('state_ts') or 0) < WORKER_STALE_S):
            raise ApiError(409, 'worker_unavailable', 'fresh reconciled mapping worker required')
        return r

    # ------------------------------------------------------------ operator requests
    def request_operation(self, robot_id, actor, body):
        from .service import ApiError

        kind = self.KIND
        if not isinstance(body, dict):
            raise ApiError(400, 'bad_operation', 'JSON object required')
        with self.lock:
            state = self.operation_status(robot_id, kind)
            lease = self.lease_view(robot_id)
            if (not lease or lease['owner'] != actor or type(body.get('epoch')) is not int
                    or lease['epoch'] != body['epoch']):
                raise ApiError(409, 'lease_required', 'current owner and epoch required')
            action, key = body.get('action'), body.get('idempotency_key')
            if action == 'START':
                self._mapping_real_block(robot_id)                # real-robot block first: its reason must stay visible
                self.require_ready(robot_id, 'mapping START')     # Phase 4: profile flag, or always for a real Agent
            if (not isinstance(action, str) or action not in STABLE or not isinstance(key, str)
                    or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,100}', key)):
                raise ApiError(400, 'bad_operation', 'valid action and idempotency_key required')
            target = None
            if action == 'SAVE' and body.get('floor_id') is not None:
                target = self._save_target(body)
            plan = self._start_plan(robot_id, body) if action == 'START' else None
            old = self.db.one('SELECT body FROM operation_requests WHERE robot_id=? AND kind=? AND actor=? AND idem=?',
                              (robot_id, kind, actor, key))
            if old:
                command = json.loads(old['body'])
                if (command['action'] != action or command['epoch'] != lease['epoch']
                        or (action != 'START' and command['session_id'] != body.get('session_id'))
                        or command.get('registry_target') != target or command.get('plan') != plan):
                    raise ApiError(409, 'idempotency_conflict', 'key belongs to a different request')
                return {'duplicate': True, 'command_id': command['command_id'], 'session_id': command['session_id'],
                        'status': command['status']}
            if state['state'] not in STABLE[action]:
                raise ApiError(409, 'operation_state_conflict', state['state'])
            r = self._mapping_worker_robot(robot_id)
            if action in ('START', 'RESUME'):
                self.require_mode(robot_id, 'MAPPING', 'mode_not_mapping')
                self._require_mapping_slot_free(robot_id, r, action)
            if action == 'RECONCILE':
                if body.get('session_id') != state.get('session_id'):
                    raise ApiError(409, 'session_conflict', 'current session id required')
            elif action != 'START' and (body.get('session_id') != state.get('session_id') or state['epoch'] != lease['epoch']):
                raise ApiError(409, 'session_conflict', 'current session and lease required')
            sid = uuid.uuid4().hex if action == 'START' else state['session_id']
            command = dict(command_id=uuid.uuid4().hex, session_id=sid, robot_id=robot_id, kind=kind, action=action,
                           idempotency_key=key, status='QUEUED', owner=actor, epoch=lease['epoch'],
                           expires_at=time.time() + COMMAND_TTL_S, registry_target=target,
                           plan=plan if action == 'START' else state.get('plan'))
            session = dict(robot_id=robot_id, kind=kind, session_id=sid, state=PENDING[action],
                           boot=r['agent']['boot_id'], server_boot=self.boot_id, owner=actor, epoch=lease['epoch'],
                           command=command, result=None, detail='',
                           last_save=state.get('last_save') if action != 'START' else None,
                           registered=state.get('registered') if action != 'START' else None,
                           owned_goal=state.get('owned_goal') if action != 'START' else None,
                           plan=plan if action == 'START' else state.get('plan'))
            self._write_operation(session, new_command=True)
            self.db.event('mapping_command', f'{actor}: {action} {sid}', robot_id, data={'command_id': command['command_id']})
            return {'session_id': sid, 'command_id': command['command_id'], 'status': 'QUEUED', 'duplicate': False}

    def _start_plan(self, robot_id, body):
        """Validate mode / parent version / geofence / termination limits for START."""
        from .service import ApiError

        mode = body.get('mode', 'NEW_MAP')
        if mode not in MODES:
            raise ApiError(400, 'bad_mode', 'mode must be NEW_MAP or CONTINUE_MAP')
        plan = {'mode': mode, 'parent_map_version_id': None, 'geofence': None, 'limits': dict(LIMIT_DEFAULTS)}
        limits = body.get('limits') or {}
        if not isinstance(limits, dict) or set(limits) - set(LIMIT_DEFAULTS):
            raise ApiError(400, 'bad_limits', 'limits may contain max_duration_s, max_goals, min_battery_v')
        for k, v in limits.items():
            lo, hi = LIMIT_RANGES[k]
            if v is None and k == 'min_battery_v':
                continue
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not lo <= v <= hi:
                raise ApiError(400, 'bad_limits', f'{k} must be within {lo}..{hi}')
            plan['limits'][k] = int(v) if k == 'max_goals' else float(v)
        fence = body.get('geofence')
        if fence is not None:
            if (not isinstance(fence, list) or not 3 <= len(fence) <= 64
                    or not all(isinstance(p, list) and len(p) == 2
                               and all(isinstance(c, (int, float)) and not isinstance(c, bool) and abs(c) < 1000 for c in p)
                               for p in fence)):
                raise ApiError(400, 'bad_geofence', 'geofence must be 3..64 [x, y] map-frame points')
            plan['geofence'] = [[float(x), float(y)] for x, y in fence]
        if mode == 'CONTINUE_MAP':
            parent = body.get('parent_map_version_id')
            reg = getattr(self, 'map_registry', None)
            if reg is None or not isinstance(parent, str):
                raise ApiError(400, 'parent_required', 'CONTINUE_MAP needs a registered parent_map_version_id')
            try:
                mv = reg.get_map_version(parent)
            except Exception:
                raise ApiError(404, 'parent_not_found', 'parent map version is not registered')
            sha = mv.get('sha256') or {}
            if not sha.get('posegraph_sha256') or not sha.get('posegraph_data_sha256'):
                raise ApiError(409, 'parent_without_posegraph', 'a PGM/YAML-only map cannot be continued')
            if (mv.get('quarantine') or {}).get('status') == 'ACTIVE':
                raise ApiError(409, 'parent_quarantined', 'parent map version is quarantined')
            agent = (self.robots.get(robot_id) or {}).get('agent') or {}
            if agent.get('mapping_continue') is not True:
                raise ApiError(409, 'continue_unsupported', 'this mapping worker cannot load a pose graph yet')
            loc = ((self.robots.get(robot_id) or {}).get('state') or {}).get('mapping_localization') or {}
            if loc.get('status') != 'CONFIRMED' or loc.get('map_version_id') != parent:
                raise ApiError(409, 'localization_unconfirmed', 'robot localization on the parent map is not confirmed')
            plan['parent_map_version_id'] = parent
        return plan

    def _save_target(self, body):
        from .service import ApiError

        floor, area = body.get('floor_id'), body.get('area_id')
        if not isinstance(floor, str) or not ID_RE.fullmatch(floor) or (area is not None and (not isinstance(area, str) or not ID_RE.fullmatch(area))):
            raise ApiError(400, 'invalid_floor', 'floor_id (and optional area_id) must be registry identifiers')
        reg = getattr(self, 'map_registry', None)
        if reg is None or not reg.one('SELECT 1 FROM floors WHERE floor_id=?', (floor,)):
            raise ApiError(400, 'invalid_floor', 'registered floor is required')
        if area is not None and not reg.one('SELECT 1 FROM map_areas WHERE area_id=? AND floor_id=?', (area, floor)):
            raise ApiError(400, 'invalid_area', 'area must be registered on the same floor')
        return {'floor_id': floor, 'area_id': area}

    def _require_mapping_slot_free(self, robot_id, r, action):
        from .service import ApiError

        execution = (r.get('state') or {}).get('execution') or {}
        nav = execution.get('nav2_goals') or {}
        # START may begin before Nav2 publishes its action status (isolated graph still coming up).
        starting_empty_graph = action == 'START' and nav.get('publishers') == 0 and nav.get('active_count') is None
        idle = (nav.get('known') is True and nav.get('publishers') == 1
                and type(nav.get('active_count')) is int and nav['active_count'] == 0)
        if execution.get('active') is not None or not (idle or starting_empty_graph):
            raise ApiError(409, 'nav_terminal_unproven', 'Nav2 idle evidence required')
        if nav.get('foreign_active_goal_uuids'):
            raise ApiError(409, 'foreign_goal_present', 'a NavigateToPose goal not owned by Fleet is active')
        if any(m['status'] not in MISSION_DONE for m in self.db.q('SELECT status FROM missions WHERE robot_id=?', (robot_id,))):
            raise ApiError(409, 'destination_pending', 'finish or cancel navigation missions first')
        manual = self.manual_benches.get(robot_id)
        if manual and manual['until'] > time.monotonic():
            raise ApiError(409, 'manual_active', 'stop manual input first')
        slam = (r.get('state') or {}).get('slam') or {}
        if slam.get('enabled') and slam.get('status') not in (None, 'IDLE', 'STOPPED', 'COMPLETED', 'FAILED'):
            raise ApiError(409, 'slam_session_active', 'stop the separate SLAM session first')
        pending = self.db.q("SELECT 1 FROM commands WHERE robot_id=? AND type IN ('slam_start','slam_save','slam_stop') "
                            "AND status NOT IN ('SUCCEEDED','FAILED','CANCELED','REJECTED','EXPIRED')", (robot_id,))
        if pending:
            raise ApiError(409, 'slam_pending', 'unresolved SLAM command')

    # ------------------------------------------------------------ worker side
    def worker_operation(self, robot_id, boot, body):
        from .service import ApiError

        with self.lock:
            self.check_session(robot_id, boot)
            if body.get('kind') != self.KIND:
                raise ApiError(400, 'bad_kind', 'mapping required')
            self._mapping_worker_robot(robot_id)
            s = self.operation_status(robot_id)
            c = s.get('command')
            if body.get('action') == 'poll':
                renew = (s['state'] in ('MAPPING', 'PAUSED') and s.get('session_id') == body.get('session_id')
                         and s.get('boot') == boot)
                response = {'command': c if c and c['status'] == 'QUEUED' else None}
                if 'session_id' in body:
                    response['renew'] = renew
                    if renew:
                        # How long the worker may keep going without another renew: never past the Lease expiry.
                        lease = self.lease_view(robot_id)
                        response['renew_valid_s'] = round(max(0.0, min(RENEW_FENCE_S, lease['expires_at'] - time.time())), 3) if lease else 0.0
                return response
            if body.get('action') == 'reserve_goal':
                return self._reserve_goal(robot_id, boot, s, body)
            if (not c or s['boot'] != boot or body.get('command_id') != c['command_id']
                    or body.get('session_id') != s['session_id']):
                raise ApiError(409, 'operation_mismatch', 'session, command and boot must match')
            if body.get('action') == 'accept' and c['status'] in ('QUEUED', 'ACCEPTED'):
                first = c['status'] == 'QUEUED'
                c['status'] = 'ACCEPTED'
                if first:
                    self._write_operation(s)
                return {'accepted': first, 'status': c['status']}
            if body.get('action') == 'result' and c['status'] in ('COMPLETED', 'FAILED', 'UNKNOWN'):
                return s  # the response may have been lost; never rewrite the first result
            if body.get('action') != 'result' or c['status'] != 'ACCEPTED':
                raise ApiError(409, 'operation_not_active', c['status'])
            evidence = body.get('evidence')
            if not isinstance(evidence, dict) or body.get('outcome') not in ('OK', 'FAILED', 'UNKNOWN'):
                raise ApiError(400, 'bad_result', 'outcome and evidence required')
            action = c['action']
            required = REQUIRED[action]
            values = {k: evidence.get(k) is True for k in required}
            outcome = body['outcome']
            if outcome == 'OK' and not all(values.values()):
                outcome = 'UNKNOWN' if 'goal_terminal' in required else 'FAILED'
            c['status'] = {'OK': 'COMPLETED', 'FAILED': 'FAILED', 'UNKNOWN': 'UNKNOWN'}[outcome]
            s['state'] = DONE[action] if outcome == 'OK' else (
                'PAUSED' if action == 'SAVE' and outcome == 'FAILED' else 'UNKNOWN' if action == 'RECONCILE' else outcome)
            s['result'] = dict(action=action, status=c['status'], evidence=values, detail=str(body.get('detail', ''))[:300])
            s['detail'] = s['result']['detail']
            if action == 'SAVE':
                s['last_save'] = dict(s['result'], files=body.get('files') if isinstance(body.get('files'), dict) else None)
            if action in ('STOP', 'RECONCILE') and outcome == 'OK':
                s['owned_goal'] = None
            self._write_operation(s)
            self.db.event('mapping_result', f"{action} -> {c['status']}", robot_id,
                          data={'session_id': s['session_id'], 'evidence': values},
                          level='info' if outcome == 'OK' else 'warning')
            return s

    def _reserve_goal(self, robot_id, boot, s, body):
        from .service import ApiError

        cid, prev, owned = body.get('goal_id'), body.get('previous'), s.get('owned_goal')
        if (s['state'] != 'MAPPING' or s['boot'] != boot or s['session_id'] != body.get('session_id')
                or not isinstance(cid, str) or not re.fullmatch('[0-9a-f]{32}', cid)
                or (owned and (not isinstance(prev, dict) or owned != prev.get('command_id') or cid == owned
                               or prev.get('status') not in GOAL_DONE
                               or (prev.get('status') != 'REJECTED'
                                   and not re.fullmatch('[0-9a-f]{32}', str(prev.get('goal_uuid'))))))):
            raise ApiError(409, 'goal_reservation_required', 'live mapping session and resolved prior goal required')
        s['owned_goal'] = cid
        self._write_operation(s)
        return {'reserved': True}

    def mapping_save_command(self, robot_id, boot, session_id, command_id):
        """The SAVE command that may upload a bundle: accepted, owned by this boot, with a registry target."""
        with self.lock:
            s = self.operation_status(robot_id)
            c = s.get('command') or {}
            if (s.get('state') != 'SAVING' or s.get('boot') != boot or s.get('session_id') != session_id
                    or c.get('command_id') != command_id or c.get('action') != 'SAVE' or c.get('status') != 'ACCEPTED'
                    or not c.get('registry_target')):
                return None
            return dict(c)

    def mapping_registered(self, robot_id, command_id, map_version_id, area_id):
        with self.lock:
            s = self.operation_status(robot_id)
            if (s.get('command') or {}).get('command_id') != command_id:
                return
            s['registered'] = {'map_version_id': map_version_id, 'area_id': area_id, 'command_id': command_id}
            self._write_operation(s)
            self.db.event('mapping_registered', f'map version {map_version_id}', robot_id,
                          data={'session_id': s['session_id'], 'area_id': area_id})
