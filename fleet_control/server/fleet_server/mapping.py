"""Separate Mapping and Bag command journal for an operator operation worker."""
import json
import re
import time
import uuid


STABLE = {
    'mapping': {'START': {'IDLE', 'STOPPED'}, 'PAUSE': {'MAPPING'},
                'RESUME': {'PAUSED'}, 'SAVE': {'PAUSED'}, 'STOP': {'MAPPING', 'PAUSED'}},
    'bag': {'START': {'IDLE', 'STOPPED'}, 'STOP': {'RECORDING'}},
}
PENDING = dict(START='STARTING', PAUSE='PAUSING', RESUME='STARTING', SAVE='SAVING', STOP='STOPPING')
DONE = dict(START='MAPPING', PAUSE='PAUSED', RESUME='MAPPING', SAVE='PAUSED', STOP='STOPPED')


class MappingMixin:
    def operation_status(self, robot_id, kind):
        from .service import ApiError

        with self.lock:
            if robot_id not in self.robots:
                raise ApiError(404, 'unknown_robot', 'robot not registered')
            row = self.db.one('SELECT body FROM operation_sessions WHERE robot_id=? AND kind=?',
                              (robot_id, kind))
            if not row:
                return {'robot_id': robot_id, 'kind': kind, 'state': 'IDLE'}
            session = json.loads(row['body'])
            robot = self.robots[robot_id]
            lease = self.lease_view(robot_id)
            command = session['command']
            worker_status = ((robot.get('state') or {}).get(kind + '_worker') or {}).get('status')
            if session['state'] not in ('STOPPED', 'UNKNOWN') and (
                    worker_status in ('LOST', 'UNKNOWN', 'FAILED')
                    or
                    session.get('server_boot') != self.boot_id
                    or time.time() - (robot.get('state_ts') or 0) > 3
                    or not robot.get('reconciled') or robot.get('reconciliation_required')
                    or robot.get('agent', {}).get('boot_id') != session['boot']
                    or not lease or lease['epoch'] != session['epoch']
                    or (command['status'] in ('QUEUED', 'ACCEPTED') and time.time() >= command['expires_at'])):
                session['state'] = 'UNKNOWN'
                if command['status'] in ('QUEUED', 'ACCEPTED'):
                    command['status'] = 'UNKNOWN'
                session['detail'] = 'Worker, lease, or command evidence expired. Reconciliation required.'
                self._write_operation(session)
            return session

    def _write_operation(self, session, new_command=False):
        with self.db.lock, self.db.c:
            self.db.c.execute('BEGIN IMMEDIATE')
            self.db.x(
                'INSERT INTO operation_sessions(robot_id,kind,body) VALUES(?,?,?) '
                'ON CONFLICT(robot_id,kind) DO UPDATE SET body=excluded.body',
                (session['robot_id'], session['kind'], json.dumps(session)))
            if new_command:
                c = session['command']
                self.db.x(
                    'INSERT INTO operation_requests(robot_id,kind,actor,idem,body) VALUES(?,?,?,?,?)',
                    (session['robot_id'], session['kind'], session['owner'], c['idempotency_key'], json.dumps(c)))

    def _operation_worker(self, robot_id, kind):
        from .service import ApiError

        r = self.robots.get(robot_id) or {}
        agent = r.get('agent') or {}
        real_mapping = kind == 'mapping' and agent.get('isolated_ros_mapping') is True
        if (agent.get('mode') != 'mock' or agent.get('simulated') is not True
                or not (agent.get('mapping_fixture') is True or
                        agent.get('isolated_ros_' + kind) is True)
                or (agent.get('missions_enabled') is not False and not real_mapping)
                or not r.get('reconciled') or r.get('reconciliation_required')
                or not 0 <= time.time() - (r.get('state_ts') or 0) < 3):
            raise ApiError(409, 'worker_unavailable', 'fresh reconciled operation worker required')
        return r

    def request_operation(self, robot_id, kind, actor, body):
        from .service import ApiError

        with self.lock:
            state = self.operation_status(robot_id, kind)
            lease = self.lease_view(robot_id)
            if (not lease or lease['owner'] != actor or type(body.get('epoch')) is not int
                    or lease['epoch'] != body['epoch']):
                raise ApiError(409, 'lease_required', 'current owner and epoch required')
            action, key = body.get('action'), body.get('idempotency_key')
            if not isinstance(action, str) or action not in STABLE[kind] or not isinstance(key, str) or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,100}', key):
                raise ApiError(400, 'bad_operation', 'valid action and idempotency_key required')
            old = self.db.one(
                'SELECT body FROM operation_requests WHERE robot_id=? AND kind=? AND actor=? AND idem=?',
                (robot_id, kind, actor, key))
            if old:
                command = json.loads(old['body'])
                if (command['action'] != action or command['epoch'] != lease['epoch']
                        or (action != 'START' and command['session_id'] != body.get('session_id'))):
                    raise ApiError(409, 'idempotency_conflict', 'key belongs to a different request')
                return {'duplicate': True, 'command_id': command['command_id'], 'session_id': command['session_id']}
            if state['state'] not in STABLE[kind][action]:
                raise ApiError(409, 'operation_state_conflict', state['state'])
            r = self._operation_worker(robot_id, kind)
            if kind == 'mapping' and action in ('START', 'RESUME'):
                nav = ((r.get('state') or {}).get('execution') or {}).get('nav2_goals') or {}
                active = ((r.get('state') or {}).get('execution') or {}).get('active')
                starting_empty_graph = (action == 'START' and r['agent'].get('isolated_ros_mapping') is True
                                        and nav.get('publishers') == 0 and nav.get('active_count') is None)
                idle = (nav.get('known') is True and nav.get('publishers') == 1
                        and type(nav.get('active_count')) is int and nav['active_count'] == 0)
                if active is not None or not (idle or starting_empty_graph):
                    raise ApiError(409, 'nav_terminal_unproven', 'Nav2 idle evidence required')
                if any(m['status'] not in ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED')
                       for m in self.db.q('SELECT status FROM missions WHERE robot_id=?', (robot_id,))):
                    raise ApiError(409, 'destination_pending', 'finish destination missions first')
                manual = self.manual_benches.get(robot_id)
                if manual and manual['until'] > time.monotonic():
                    raise ApiError(409, 'manual_active', 'stop manual input first')
            if action != 'START' and (body.get('session_id') != state['session_id'] or state['epoch'] != lease['epoch']):
                raise ApiError(409, 'session_conflict', 'current session and lease required')
            sid = uuid.uuid4().hex if action == 'START' else state['session_id']
            command = dict(command_id=uuid.uuid4().hex, session_id=sid, robot_id=robot_id,
                           kind=kind, action=action, idempotency_key=key, status='QUEUED',
                           owner=actor, epoch=lease['epoch'], expires_at=time.time()+20)
            session = dict(robot_id=robot_id, kind=kind, session_id=sid, state=PENDING[action],
                           boot=r['agent']['boot_id'], server_boot=self.boot_id, owner=actor, epoch=lease['epoch'],
                           command=command, result=None,
                           last_save=state.get('last_save') if action != 'START' else None,
                           owned_goal=state.get('owned_goal') if action != 'START' else None)
            self._write_operation(session, new_command=True)
            return {'session_id': sid, 'command_id': command['command_id'], 'status': 'QUEUED'}

    def worker_operation(self, robot_id, boot, body):
        from .service import ApiError

        with self.lock:
            self.check_session(robot_id, boot)
            kind = body.get('kind')
            if kind not in ('mapping', 'bag'):
                raise ApiError(400, 'bad_kind', 'mapping or bag required')
            self._operation_worker(robot_id, kind)
            s = self.operation_status(robot_id, kind)
            c = s.get('command')
            if body.get('action') == 'poll':
                renew = (s['state'] in ('RECORDING', 'MAPPING', 'PAUSED')
                         and s.get('session_id') == body.get('session_id')
                         and s.get('boot') == boot)
                response = {'command': c if c and c['status'] == 'QUEUED' else None}
                if 'session_id' in body:
                    response['renew'] = renew
                return response
            if kind == 'mapping' and body.get('action') == 'reserve_goal':
                cid = body.get('goal_id')
                prev = body.get('previous')
                owned = s.get('owned_goal')
                if (s['state'] != 'MAPPING' or s['boot'] != boot
                        or s['session_id'] != body.get('session_id')
                        or self.robots[robot_id]['agent'].get('isolated_ros_mapping') is not True
                        or not isinstance(cid, str) or not re.fullmatch('[0-9a-f]{32}', cid)
                        or (owned and (not isinstance(prev, dict) or owned != prev.get('command_id')
                                      or cid == owned
                                      or prev.get('status') not in ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED')
                                      or (prev.get('status') != 'REJECTED' and
                                          not re.fullmatch('[0-9a-f]{32}', str(prev.get('goal_uuid'))))))):
                    raise ApiError(409, 'goal_reservation_required', 'live Mapping session and resolved prior Goal required')
                s['owned_goal'] = cid
                self._write_operation(s)
                return {'reserved': True}
            if (not c or s['boot'] != boot or body.get('command_id') != c['command_id']
                    or body.get('session_id') != s['session_id']):
                raise ApiError(409, 'operation_mismatch', 'session, command and boot must match')
            if body.get('action') == 'accept' and c['status'] in ('QUEUED', 'ACCEPTED'):
                first = c['status'] == 'QUEUED'
                c['status'] = 'ACCEPTED'
                if first:
                    self._write_operation(s)
                return {'accepted': first, 'status': c['status']}
            if body.get('action') == 'result' and c['status'] in ('COMPLETED', 'FAILED'):
                return s  # response may have been lost; never rewrite the first result
            if body.get('action') != 'result' or c['status'] != 'ACCEPTED':
                raise ApiError(409, 'operation_not_active', c['status'])
            evidence = body.get('evidence')
            if not isinstance(evidence, dict) or body.get('outcome') not in ('OK', 'FAILED', 'UNKNOWN'):
                raise ApiError(400, 'bad_result', 'outcome and evidence required')
            action = c['action']
            required = {
                ('mapping', 'START'): ('slam_running', 'frontier_running'),
                ('mapping', 'RESUME'): ('slam_running', 'frontier_running'),
                ('mapping', 'PAUSE'): ('frontier_paused', 'goal_terminal'),
                ('mapping', 'SAVE'): ('yaml_ok', 'pgm_ok', 'posegraph_ok'),
                ('mapping', 'STOP'): ('frontier_stopped', 'goal_terminal'),
                ('bag', 'START'): ('recorder_running',),
                ('bag', 'STOP'): ('owned_recorder_stopped', 'bag_verified'),
            }[(kind, action)]
            values = {key: evidence.get(key) is True for key in required}
            outcome = body['outcome']
            if outcome == 'OK' and not all(values.values()):
                outcome = 'UNKNOWN' if 'goal_terminal' in required else 'FAILED'
            c['status'] = {'OK': 'COMPLETED', 'FAILED': 'FAILED', 'UNKNOWN': 'UNKNOWN'}[outcome]
            target = 'RECORDING' if kind == 'bag' and action == 'START' else DONE[action]
            s['state'] = target if outcome == 'OK' else (
                'PAUSED' if action == 'SAVE' and outcome == 'FAILED' else outcome)
            s['result'] = dict(action=action, status=c['status'], evidence=values,
                               detail=str(body.get('detail', ''))[:300])
            if action == 'SAVE':
                s['last_save'] = s['result']
            self._write_operation(s)
            return s

    def owns_mapping_goal(self, robot_id, boot, active):
        if active.get('owner') != 'frontier':
            return False
        robot = self.robots.get(robot_id) or {}
        if (robot.get('agent') or {}).get('isolated_ros_mapping') is not True:
            return False
        s = self.operation_status(robot_id, 'mapping')
        return (s['state'] in ('MAPPING', 'PAUSING', 'STOPPING')
                and s.get('boot') == boot and s.get('session_id') == active.get('mission_id')
                and s.get('owned_goal') == active.get('command_id'))

    def mapping_blocks_motion(self, robot_id):
        return self.operation_status(robot_id, 'mapping')['state'] not in ('IDLE', 'STOPPED')
