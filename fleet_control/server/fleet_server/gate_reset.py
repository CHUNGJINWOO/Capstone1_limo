"""Admin web Gate reset, server side (2026-11-21).

POST /robots/{id}/gate-reset  (admin role enforced HERE, not by hiding a button)
  body: {epoch, expected_gate_boot_id, expected_fault_epoch, note}
  The server checks what it can know (role, lease owner/epoch, Agent online + reconciled, mode IDLE without transition, no active
  mission / mapping session / manual bench, Gate report fresh and not NORMAL, the Gate generation the admin saw, no other request in
  flight) and queues ONE request bound to the current Agent boot and server boot. It is delivered to the Agent once (state-post
  reply); the Agent re-checks with its own evidence and calls the Gate, and the Gate decides again (candidate r12 reset_readiness).
GET  /robots/{id}/gate-reset  -> Gate status, Gate's own readiness, server blockers for the caller, current/last request.

Every request, server refusal, delivery and result is an audit event (events table) with actor, request_id and reason.
A reset never changes the operating mode: the robot stays IDLE and MANUAL/NAVIGATION need their normal admission afterwards.
Request states: QUEUED -> DELIVERED -> SUCCEEDED | REFUSED | FAILED; or EXPIRED / EXPIRED_NO_RESULT / ABANDONED_SERVER_RESTART /
ABANDONED_AGENT_RESTART. Nothing is retried automatically.
"""
import json
import time
import uuid

QUEUE_TTL_S = 10.0          # QUEUED and not delivered within this -> EXPIRED
RESULT_TTL_S = 15.0         # DELIVERED without an Agent result within this -> EXPIRED_NO_RESULT
GATE_REPORT_MAX_AGE_S = 2.0
PENDING = ('QUEUED', 'DELIVERED')


class GateResetMixin:
    # ------------------------------------------------------------ storage
    def _gr_key(self, robot_id): return 'gate_reset_request:' + robot_id

    def _gr_load(self, robot_id):
        row = self.db.one('SELECT v FROM meta WHERE k=?', (self._gr_key(robot_id),))
        return json.loads(row['v']) if row else None

    def _gr_save(self, rec):
        self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', (self._gr_key(rec['robot_id']), json.dumps(rec)))

    def _gr_event(self, kind, msg, robot_id, level='info', **data):
        self.db.event(kind, msg, robot_id, level=level, data=data)

    def _gr_advance(self, robot_id, now=None):
        """Expire / abandon a pending request when its delivery window, the server boot or the Agent boot no longer matches."""
        rec = self._gr_load(robot_id)
        if not rec or rec['status'] not in PENDING:
            return rec
        now = time.time() if now is None else now
        agent_boot = ((self.robots.get(robot_id) or {}).get('agent') or {}).get('boot_id')
        why = None
        if rec['server_boot_id'] != self.boot_id: why = 'ABANDONED_SERVER_RESTART'
        elif rec['agent_boot_id'] != agent_boot: why = 'ABANDONED_AGENT_RESTART'
        elif rec['status'] == 'QUEUED' and now >= rec['created_at'] + QUEUE_TTL_S: why = 'EXPIRED'
        elif rec['status'] == 'DELIVERED' and now >= rec['delivered_at'] + RESULT_TTL_S: why = 'EXPIRED_NO_RESULT'
        if why:
            rec.update(status=why, finished_at=now, reason=why)
            self._gr_save(rec)
            self._gr_event('gate_reset_' + why.lower(), f"request {rec['request_id']} {why}", robot_id, level='warning', request_id=rec['request_id'])
        return rec

    # ------------------------------------------------------------ eligibility
    def gate_reset_blockers(self, robot_id, principal, body=None):
        """Reasons the server refuses a request now (also shown in the UI). Empty list = the server would queue it."""
        from .service import MISSION_TERMINAL
        b = []
        if principal is None or principal.role != 'admin': b.append('ROLE_NOT_ADMIN')
        r = self.robots.get(robot_id)
        if r is None: return b + ['UNKNOWN_ROBOT']
        if not r.get('online'): b.append('AGENT_OFFLINE')
        if not r.get('reconciled') or r.get('reconciliation_required'): b.append('AGENT_NOT_RECONCILED')
        if not self.agent_contract_ok(robot_id): b.append('AGENT_PROTOCOL')
        if ((r.get('agent') or {}).get('capabilities') or {}).get('gate_reset') is not True and (r.get('agent') or {}).get('gate_reset') is not True:
            b.append('AGENT_GATE_RESET_UNSUPPORTED')
        lease = self.lease_view(robot_id)
        if not lease or principal is None or lease['owner'] != principal.name: b.append('LEASE_NOT_HELD_BY_CALLER')
        elif body is not None and body.get('epoch') != lease['epoch']: b.append('LEASE_EPOCH_MISMATCH')
        mode = self.mode_view(robot_id, advance=False)
        if mode is None: b.append('OPERATING_MODE_UNAVAILABLE')
        else:
            if mode.get('mode') != 'IDLE': b.append(f"MODE_NOT_IDLE:{mode.get('mode')}")
            t = mode.get('transition') or {}
            if t.get('state') not in (None, 'ACTIVE'): b.append(f"MODE_TRANSITION:{t.get('state')}")
        active = [m['mission_id'] for m in self.db.q('SELECT mission_id,status FROM missions WHERE robot_id=?', (robot_id,))
                  if m['status'] not in MISSION_TERMINAL]
        if active: b.append('MISSION_ACTIVE')
        try:
            ms = self.operation_status(robot_id).get('state')
        except Exception:
            ms = 'UNKNOWN'
        if ms not in ('IDLE', 'STOPPED', 'COMPLETED', None): b.append(f'MAPPING_SESSION:{ms}')
        if self.manual_benches.get(robot_id): b.append('MANUAL_SESSION_ACTIVE')
        wd = ((r.get('state') or {}).get('motion_watchdog') or {})
        if wd.get('state') != 'MOTION_AUTHORIZED' or wd.get('agent_boot_matches') is not True: b.append(f"WATCHDOG_NOT_AUTHORIZED:{wd.get('state')}")
        gate = ((r.get('state') or {}).get('safety_gate') or {})
        view = gate.get('reset_view') or {}
        if time.time() - (r.get('state_ts') or 0) > GATE_REPORT_MAX_AGE_S: b.append('GATE_REPORT_STALE')
        if not view.get('boot_id'): b.append('GATE_STATUS_MISSING')
        if view.get('state') in ('NORMAL', 'STALL_PENDING') and not view.get('fault_latched') and not (view.get('cmd_input') or {}).get('sticky'):
            b.append('GATE_NOT_FAULTED')
        if body is not None and (body.get('expected_gate_boot_id') != view.get('boot_id') or body.get('expected_fault_epoch') != view.get('fault_epoch')):
            b.append('GATE_CHANGED_SINCE_VIEWED')
        pend = self._gr_advance(robot_id)
        if pend and pend['status'] in PENDING: b.append('RESET_IN_PROGRESS')
        # a reset does not change fault_epoch: a (Gate boot, fault epoch) that was already reset cannot be reset again (replay /
        # stale view while the Agent report still shows the pre-reset state); a new fault has a new epoch.
        done = self.db.one('SELECT v FROM meta WHERE k=?', ('gate_reset_done:' + robot_id,))
        if done and json.loads(done['v']) == [view.get('boot_id'), view.get('fault_epoch')]: b.append('FAULT_EPOCH_ALREADY_RESET')
        return b

    # ------------------------------------------------------------ API
    def request_gate_reset(self, robot_id, principal, body):
        from .service import ApiError
        body = body if isinstance(body, dict) else {}
        with self.lock:
            refusal = None
            with self.db.transaction():
                if principal is None or principal.role != 'admin':
                    refusal = (403, 'forbidden', 'only an admin may request a Gate reset', ['ROLE_NOT_ADMIN'])
                elif robot_id not in self.robots:
                    raise ApiError(404, 'unknown_robot', 'unknown robot')
                else:
                    blockers = self.gate_reset_blockers(robot_id, principal, body)
                    if blockers:
                        code = 'gate_reset_in_progress' if blockers == ['RESET_IN_PROGRESS'] else 'gate_reset_refused'
                        refusal = (409, code, ','.join(blockers), blockers)
                    else:
                        r = self.robots[robot_id]; view = r['state']['safety_gate']['reset_view']; lease = self.lease_view(robot_id)
                        now = time.time()
                        rec = {'request_id': uuid.uuid4().hex, 'robot_id': robot_id, 'requested_by': principal.name, 'lease_epoch': lease['epoch'],
                               'server_boot_id': self.boot_id, 'agent_boot_id': r['agent']['boot_id'],
                               'gate_boot_id': view.get('boot_id'), 'fault_epoch': view.get('fault_epoch'),
                               'gate_state_at_request': view.get('state'), 'fault_reasons_at_request': view.get('last_fault_reasons') or view.get('fault_reasons'),
                               'note': str(body.get('note') or '')[:200], 'created_at': now, 'status': 'QUEUED'}
                        self._gr_save(rec)
                        self._gr_event('gate_reset_requested', f"{principal.name}: request {rec['request_id']} (Gate {view.get('state')})", robot_id,
                                       actor=principal.name, request_id=rec['request_id'], gate_state=view.get('state'), fault_reasons=rec['fault_reasons_at_request'])
                        return self._gr_public(rec)
            # refusals are audited OUTSIDE the transaction above (raising inside it would roll the audit row back)
            status, code, msg, reasons = refusal
            actor, role = getattr(principal, 'name', None), getattr(principal, 'role', None)
            self._gr_event('gate_reset_refused', f'{actor} ({role}): {msg}', robot_id, level='warning', actor=actor, role=role,
                           reason=reasons[0] if reasons == ['ROLE_NOT_ADMIN'] else reasons, note=str(body.get('note') or '')[:200])
            raise ApiError(status, code, msg)

    def gate_reset_view(self, robot_id, principal):
        from .service import ApiError
        with self.lock:
            r = self.robots.get(robot_id)
            if r is None: raise ApiError(404, 'unknown_robot', 'unknown robot')
            rec = self._gr_advance(robot_id)
            gate = (r.get('state') or {}).get('safety_gate') or {}
            agent_rep = (r.get('state') or {}).get('gate_reset') or {}
            return {'robot_id': robot_id,
                    'gate': {'status': gate.get('status'), 'view': gate.get('reset_view'), 'report_age_s': round(time.time() - (r.get('state_ts') or 0), 2)},
                    'server_blockers': self.gate_reset_blockers(robot_id, principal),
                    'request': self._gr_public(rec) if rec else None,
                    'agent_last': agent_rep.get('last'), 'agent_enabled': agent_rep.get('enabled') is True,
                    'note': 'the Gate decides last (reset_readiness); a reset keeps the robot IDLE'}

    def _gr_public(self, rec):
        return {k: rec.get(k) for k in ('request_id', 'status', 'reason', 'requested_by', 'created_at', 'delivered_at', 'finished_at',
                                         'gate_state_at_request', 'fault_reasons_at_request', 'gate_message', 'failed_checks')}

    # ------------------------------------------------------------ Agent side (called from ingest_state under self.lock)
    def gate_reset_for_agent(self, robot_id):
        """Deliver a QUEUED request exactly once to the Agent boot it was issued for."""
        rec = self._gr_advance(robot_id)
        if not rec or rec['status'] != 'QUEUED' or not self.agent_contract_ok(robot_id):
            return None
        rec.update(status='DELIVERED', delivered_at=time.time())
        self._gr_save(rec)
        self._gr_event('gate_reset_delivered', f"request {rec['request_id']} delivered", robot_id, request_id=rec['request_id'])
        return {k: rec[k] for k in ('request_id', 'requested_by', 'lease_epoch', 'server_boot_id', 'agent_boot_id', 'gate_boot_id', 'fault_epoch')}

    def gate_reset_ingest(self, robot_id, state):
        """Take the Agent's result for the DELIVERED request only (a result for any other / older id is ignored = no replay)."""
        last = ((state or {}).get('gate_reset') or {}).get('last')
        if not isinstance(last, dict):
            return
        rec = self._gr_load(robot_id)
        if not rec or rec['status'] != 'DELIVERED' or last.get('request_id') != rec['request_id']:
            return
        status = last.get('status')
        if status not in ('SUCCEEDED', 'REFUSED', 'FAILED'):
            return
        failed = [k for k, v in (last.get('checks') or {}).items() if isinstance(v, dict) and not v.get('ok')]
        rec.update(status=status, finished_at=time.time(), reason=last.get('reason'), gate_message=last.get('gate_message'),
                   failed_checks=failed, gate_called=last.get('gate_called'))
        self._gr_save(rec)
        if status == 'SUCCEEDED':
            self.manual_benches.pop(robot_id, None)        # nothing resumes after a reset
            self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', ('gate_reset_done:' + robot_id, json.dumps([rec['gate_boot_id'], rec['fault_epoch']])))
        self._gr_event('gate_reset_result', f"request {rec['request_id']} {status}: {last.get('reason') or last.get('gate_message')}", robot_id,
                       level='info' if status == 'SUCCEEDED' else 'warning', request_id=rec['request_id'], actor=rec['requested_by'],
                       status=status, reason=last.get('reason'), gate_message=last.get('gate_message'), failed_checks=failed,
                       checks=last.get('checks'))
