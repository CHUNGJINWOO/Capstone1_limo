"""Operator ROS bag sessions for the isolated BagWorker (G2 2026-11-23: ported from origin/main `mapping.py`, kind 'bag').

origin/main served Mapping and Bag through one generic operation journal. The v5b Mapping journal (`mapping.py`) is
Mapping-only and keeps the R1 Mapping STOP / Lease rules, so the Bag half lives here, unchanged in behaviour: the same
`operation_sessions` / `operation_requests` tables (kind 'bag'), the same START / STOP state machine, idempotency keys,
Lease owner/epoch checks and the evidence the Agent must report (`recorder_running`; `owned_recorder_stopped`, `bag_verified`).

Additions for the integrated tree (fail-closed, never wider than main):
- the worker must be an isolated mock Agent that announced `isolated_ros_bag: true` and does NOT run the local
  `recording_session` (`recording_enabled` true): exactly one rosbag owner per Agent;
- START needs a compatible Agent contract (`agent_contract_ok`); poll / accept / result of an existing session and STOP stay
  possible so a running recorder can always be stopped.
A bag recorder never moves the robot; nothing here grants motion.
"""
import json
import re
import time
import uuid

KIND = 'bag'
STABLE = {'START': {'IDLE', 'STOPPED'}, 'STOP': {'RECORDING'}}
PENDING = {'START': 'STARTING', 'STOP': 'STOPPING'}
DONE = {'START': 'RECORDING', 'STOP': 'STOPPED'}
REQUIRED = {'START': ('recorder_running',), 'STOP': ('owned_recorder_stopped', 'bag_verified')}


class BagOperationMixin:
    def bag_status(self, robot_id):
        from .service import ApiError

        with self.lock:
            if robot_id not in self.robots:
                raise ApiError(404, 'unknown_robot', 'robot not registered')
            row = self.db.one('SELECT body FROM operation_sessions WHERE robot_id=? AND kind=?', (robot_id, KIND))
            if not row:
                return {'robot_id': robot_id, 'kind': KIND, 'state': 'IDLE'}
            session = json.loads(row['body'])
            robot = self.robots[robot_id]
            lease = self.lease_view(robot_id)
            command = session['command']
            worker_status = ((robot.get('state') or {}).get('bag_worker') or {}).get('status')
            if session['state'] not in ('STOPPED', 'UNKNOWN') and (
                    worker_status in ('LOST', 'UNKNOWN', 'FAILED')
                    or session.get('server_boot') != self.boot_id
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

    def _bag_worker(self, robot_id, *, starting=False):
        from .service import ApiError

        r = self.robots.get(robot_id) or {}
        agent = r.get('agent') or {}
        if (agent.get('mode') != 'mock' or agent.get('simulated') is not True
                or agent.get('isolated_ros_bag') is not True or agent.get('recording_enabled') is True
                or agent.get('missions_enabled') is not False
                or not r.get('reconciled') or r.get('reconciliation_required')
                or not 0 <= time.time() - (r.get('state_ts') or 0) < 3):
            raise ApiError(409, 'worker_unavailable', 'fresh reconciled isolated bag worker required')
        if starting and not self.agent_contract_ok(robot_id):
            raise ApiError(409, 'agent_protocol', 'incompatible agent protocol or safety contract for a bag session')
        return r

    def request_bag(self, robot_id, actor, body):
        from .service import ApiError

        with self.lock:
            state = self.bag_status(robot_id)
            lease = self.lease_view(robot_id)
            if (not lease or lease['owner'] != actor or type(body.get('epoch')) is not int
                    or lease['epoch'] != body['epoch']):
                raise ApiError(409, 'lease_required', 'current owner and epoch required')
            action, key = body.get('action'), body.get('idempotency_key')
            if (not isinstance(action, str) or action not in STABLE or not isinstance(key, str)
                    or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,100}', key)):
                raise ApiError(400, 'bad_operation', 'valid action and idempotency_key required')
            old = self.db.one('SELECT body FROM operation_requests WHERE robot_id=? AND kind=? AND actor=? AND idem=?',
                              (robot_id, KIND, actor, key))
            if old:
                command = json.loads(old['body'])
                if (command['action'] != action or command['epoch'] != lease['epoch']
                        or (action != 'START' and command['session_id'] != body.get('session_id'))):
                    raise ApiError(409, 'idempotency_conflict', 'key belongs to a different request')
                return {'duplicate': True, 'command_id': command['command_id'], 'session_id': command['session_id']}
            if state['state'] not in STABLE[action]:
                raise ApiError(409, 'operation_state_conflict', state['state'])
            r = self._bag_worker(robot_id, starting=action == 'START')
            if action != 'START' and (body.get('session_id') != state['session_id'] or state['epoch'] != lease['epoch']):
                raise ApiError(409, 'session_conflict', 'current session and lease required')
            sid = uuid.uuid4().hex if action == 'START' else state['session_id']
            command = dict(command_id=uuid.uuid4().hex, session_id=sid, robot_id=robot_id, kind=KIND, action=action,
                           idempotency_key=key, status='QUEUED', owner=actor, epoch=lease['epoch'], expires_at=time.time() + 20)
            session = dict(robot_id=robot_id, kind=KIND, session_id=sid, state=PENDING[action], boot=r['agent']['boot_id'],
                           server_boot=self.boot_id, owner=actor, epoch=lease['epoch'], command=command, result=None)
            self._write_operation(session, new_command=True)
            return {'session_id': sid, 'command_id': command['command_id'], 'status': 'QUEUED'}

    def bag_worker_operation(self, robot_id, boot, body):
        from .service import ApiError

        with self.lock:
            self.check_session(robot_id, boot)
            self._bag_worker(robot_id)
            s = self.bag_status(robot_id)
            c = s.get('command')
            if body.get('action') == 'poll':
                renew = s['state'] == 'RECORDING' and s.get('session_id') == body.get('session_id') and s.get('boot') == boot
                response = {'command': c if c and c['status'] == 'QUEUED' else None}
                if 'session_id' in body:
                    response['renew'] = renew
                return response
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
            values = {k: evidence.get(k) is True for k in REQUIRED[action]}
            outcome = body['outcome']
            if outcome == 'OK' and not all(values.values()):
                outcome = 'FAILED'
            c['status'] = {'OK': 'COMPLETED', 'FAILED': 'FAILED', 'UNKNOWN': 'UNKNOWN'}[outcome]
            s['state'] = DONE[action] if outcome == 'OK' else outcome
            s['result'] = dict(action=action, status=c['status'], evidence=values, detail=str(body.get('detail', ''))[:300])
            self._write_operation(s)
            return s
