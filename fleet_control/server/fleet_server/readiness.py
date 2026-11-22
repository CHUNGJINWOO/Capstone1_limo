"""Phase 4 (2026-11-21): robot readiness without SSH - supervisor reports, READY / DEGRADED / FAULT, predefined service actions.

The robot-side supervisor (limo_robot_agent.supervisor) owns the start order of the ROS services and the Agent. It posts its
view every second (POST /api/v1/agent/supervisor, the robot's agent token) and receives the service actions an ADMIN queued.

Readiness is a VIEW for operators and a precondition for MAPPING / NAVIGATION when the server runs with
--require-supervisor-ready. It never grants motion: the operating-mode transition, the Agent's own selector, the Motion
Watchdog and the Safety Gate keep every existing check. Service actions:
  - only the actions in ACTIONS on service names the supervisor itself reported from its LOCAL config file (the web never
    supplies a command line, an argument or a path);
  - admin only; stop / restart are refused while the robot is in MANUAL / MAPPING / NAVIGATION, while a transition is running,
    or while a mission is in flight (stop the work first); start / retry of a STOPPED / FAILED service are always allowed;
  - each request and each result is an event.
"""
import math, time, uuid

ACTIONS = ('start', 'stop', 'restart', 'retry')
SUPERVISOR_STALE_S = 5.0
SENSOR_STALE_S = 1.0
GATE_FAULT_STATES = ('FAILSAFE_SENSOR_TIMEOUT', 'FAILSAFE_ICP_DEGRADED', 'STALL_LATCHED', 'FAILSAFE_STARTUP_TIMEOUT')
SERVICE_STATES = ('STOPPED', 'WAITING', 'STARTING', 'RUNNING', 'READY', 'BACKOFF', 'FAILED', 'STOPPING')
REQUIRED_SERVICES = ('stack', 'watchdog', 'agent')         # server-side default (trusted); --required-services overrides
REQUIRED_TOPICS = ('/scan', '/odom')                        # --required-topics overrides
AGENT_STATE_STALE_S = 2.0
GATE_STATUS_STALE_S = 1.0          # Gate status is 10 Hz; the Agent reports its receive age
WATCHDOG_STATUS_STALE_S = 1.5
MAX_TOPIC_AGE_LIMIT_S = 5.0        # a reported max_age_s above this is not trusted (falls back to SENSOR_STALE_S)


def _num(v):
    """A finite real number (bool excluded) or None."""
    if isinstance(v, bool) or not isinstance(v, (int, float)): return None
    return float(v) if math.isfinite(v) else None


def _age(now, t):
    t = _num(t)
    if t is None: return None
    a = now - t
    return a if a >= -1.0 else None          # small clock skew tolerated; a timestamp from the future is invalid


def _round(v): return None if v is None else round(v, 1)


class ReadinessMixin:
    def _sup(self):
        if not hasattr(self, 'supervisors'):
            self.supervisors, self.service_actions = {}, {}
        return self.supervisors

    # ------------------------------------------------------------------ supervisor side
    def supervisor_report(self, robot_id, body):
        from .service import ApiError
        if not isinstance(body, dict) or not isinstance(body.get('boot_id'), str) or not isinstance(body.get('services'), list):
            raise ApiError(400, 'bad_supervisor_report', 'boot_id and services required')
        sups = self._sup()
        with self.lock:
            prev = sups.get(robot_id)
            now = time.time()
            services = []
            for s in body['services'][:16]:
                if not isinstance(s, dict) or not isinstance(s.get('name'), str): continue
                services.append({k: s.get(k) for k in ('name', 'state', 'pid', 'restarts', 'last_exit', 'since', 'detail', 'required', 'actions')})
            if prev is None or prev['boot_id'] != body['boot_id']:
                self.db.event('supervisor_boot', f"supervisor {body['boot_id']} started on {robot_id}"
                              + (f" (previous {prev['boot_id']})" if prev else ''), robot_id, level='warning' if prev else 'info')
                # a new supervisor boot never inherits queued actions of the previous one
                for a in self.service_actions.get(robot_id, []):
                    if a['status'] in ('QUEUED', 'SENT'):
                        a.update(status='EXPIRED', detail='supervisor restarted before executing the action', updated_at=now)
            old = {s['name']: s.get('state') for s in (prev or {}).get('services', [])}
            for s in services:
                if old.get(s['name']) != s.get('state'):
                    lvl = 'error' if s.get('state') == 'FAILED' else 'warning' if s.get('state') in ('BACKOFF', 'STOPPED') else 'info'
                    self.db.event('service_state', f"{robot_id}/{s['name']}: {old.get(s['name']) or 'NEW'} -> {s.get('state')}"
                                  + (f" ({s.get('detail')})" if s.get('detail') else ''), robot_id, level=lvl,
                                  data={'service': s['name'], 'state': s.get('state'), 'last_exit': s.get('last_exit'), 'restarts': s.get('restarts')})
            for res in (body.get('action_results') or [])[:16]:
                a = next((x for x in self.service_actions.get(robot_id, []) if x['action_id'] == (res or {}).get('action_id')), None)
                if a and a['status'] in ('QUEUED', 'SENT'):
                    a.update(status='DONE' if res.get('ok') else 'FAILED', detail=str(res.get('detail', ''))[:200], updated_at=now)
                    self.db.event('service_action_result', f"{robot_id}/{a['service']} {a['action']}: {a['status']} {a['detail']}", robot_id,
                                  level='info' if res.get('ok') else 'warning')
            sups[robot_id] = {'boot_id': body['boot_id'], 'received_at': now, 'services': services,
                              'ros': body.get('ros') if isinstance(body.get('ros'), dict) else {}, 'host': body.get('host') if isinstance(body.get('host'), dict) else {}}
            pending = [a for a in self.service_actions.get(robot_id, []) if a['status'] == 'QUEUED' and a['supervisor_boot'] == body['boot_id']]
            for a in pending: a['status'] = 'SENT'
            return {'server_boot_id': self.boot_id, 'actions': [{k: a[k] for k in ('action_id', 'service', 'action')} for a in pending]}

    # ------------------------------------------------------------------ operator side
    # ------------------------------------------------------------------ requirements (trusted server configuration, not the robot report)
    def _supervisor_required(self, r):
        """Supervisor evidence is required when the server profile says so (--require-supervisor-ready, set in run_field_server.sh) OR the
        Agent itself reports a real robot (mode 'real' or simulated False; the shipped Agent always sends both). A forgotten server flag can
        therefore never skip it for a real robot (Codex 2026-11-21). Only an Agent that is explicitly mock/sim, or that predates these fields,
        follows the server flag (legacy isolated E2E)."""
        agent = (r or {}).get('agent') or {}
        real = agent.get('mode') == 'real' or agent.get('simulated') is False
        return bool(getattr(self.cfg, 'require_supervisor_ready', False)) or real

    def _required_services(self):
        return tuple(getattr(self.cfg, 'required_services', None) or REQUIRED_SERVICES)

    def _required_topics(self):
        return tuple(getattr(self.cfg, 'required_topics', None) or REQUIRED_TOPICS)

    def readiness(self, robot_id):
        """System readiness, fail closed.

        level READY      : every REQUIRED piece of evidence is present, well-formed, fresh and healthy (supervisor + required services,
                           required sensor topics, Agent state, Safety Gate and Motion Watchdog reports with finite in-range ages).
        motion_permitted : READY AND the Agent's own Gate verdict currently permits movement (Gate not blocking commands, Watchdog
                           authorized). Neither grants motion: the operating mode, Lease, Arbiter, Motion Watchdog and Gate decide.
        Missing, duplicated, malformed (NaN / Inf / negative / wrong type) or stale evidence is never READY."""
        sups = self._sup()
        with self.lock:
            r = self.robots.get(robot_id)
            if r is None:
                return {'robot_id': robot_id, 'level': 'FAULT', 'reasons': ['UNKNOWN_ROBOT'], 'services': [], 'motion_permitted': False}
            now = time.time(); fault, degraded = [], []
            sup = sups.get(robot_id)
            required = self._supervisor_required(r)
            if sup is None:
                (fault if required else degraded).append('SUPERVISOR_NOT_REPORTING')
            else:
                age = _age(now, sup.get('received_at'))
                if age is None or age > SUPERVISOR_STALE_S: fault.append('SUPERVISOR_STALE')
                names = [s.get('name') for s in sup.get('services', [])]
                for n in sorted({n for n in names if names.count(n) > 1}): fault.append(f'SERVICE_DUPLICATE:{n}')
                for n in self._required_services():
                    if n not in names: fault.append(f'SERVICE_MISSING:{n}')
                for s in sup.get('services', []):
                    st = s.get('state')
                    if st not in SERVICE_STATES: fault.append(f"SERVICE_STATE_INVALID:{s.get('name')}")
                    elif st == 'FAILED': fault.append(f"SERVICE_FAILED:{s['name']}")
                    elif st in ('STOPPED', 'STOPPING') and (s.get('required', True) is not False or s['name'] in self._required_services()):
                        fault.append(f"SERVICE_STOPPED:{s['name']}")
                    elif st in ('WAITING', 'STARTING', 'RUNNING', 'BACKOFF'): degraded.append(f"SERVICE_{st}:{s['name']}")
                topics = ((sup.get('ros') or {}).get('topics')) or {}
                if not isinstance(topics, dict): topics = {}; fault.append('TOPIC_REPORT_INVALID')
                for t in self._required_topics():
                    if t not in topics: degraded.append(f'NO_DATA:{t}')
                for topic, info in topics.items():
                    if not isinstance(info, dict): degraded.append(f'TOPIC_INVALID:{topic}'); continue
                    lim = _num(info.get('max_age_s'))
                    lim = SENSOR_STALE_S if lim is None or not 0 < lim <= MAX_TOPIC_AGE_LIMIT_S else lim
                    if info.get('age_s') is None: degraded.append(f'NO_DATA:{topic}')
                    else:
                        a = _num(info.get('age_s'))
                        if a is None or a < 0: degraded.append(f'INVALID_AGE:{topic}')
                        elif a > lim: degraded.append(f'STALE:{topic}')
            if not r.get('online'): fault.append('AGENT_OFFLINE')
            elif not r.get('reconciled'): degraded.append('AGENT_NOT_RECONCILED')
            if r.get('reconciliation_required') or self.db.one("SELECT 1 FROM missions WHERE robot_id=? AND status='RECONCILIATION_REQUIRED'", (robot_id,)):
                fault.append('RECONCILIATION_REQUIRED')
            if self.db.one('SELECT 1 FROM meta WHERE k=?', ('gate_boot_reconciliation:' + robot_id,)):
                fault.append('GATE_BOOT_RECONCILIATION')
            st = r.get('state') or {}
            gate = st.get('safety_gate') if isinstance(st.get('safety_gate'), dict) else None
            wdr = st.get('motion_watchdog') if isinstance(st.get('motion_watchdog'), dict) else None
            if r.get('online'):
                sa = _age(now, r.get('state_ts'))
                if sa is None or sa > AGENT_STATE_STALE_S: degraded.append('AGENT_STATE_STALE')
                if gate is None: degraded.append('GATE:NO_DATA')
                else:
                    gs = gate.get('status')
                    if gs in GATE_FAULT_STATES or gate.get('fault_latched') is True: fault.append(f'GATE_FAULT:{gs}')
                    elif gs not in ('NORMAL', 'STALL_PENDING'): degraded.append(f'GATE_NOT_NORMAL:{gs}')
                    ga = _num(gate.get('age_s'))
                    if ga is None or ga < 0 or ga > GATE_STATUS_STALE_S or gate.get('rx_fresh') is False: degraded.append('GATE:STALE')
                    if gate.get('cmd_blocked') is not False: degraded.append('GATE:CMD_BLOCKED' if gate.get('cmd_blocked') is True else 'GATE:CMD_BLOCKED_UNKNOWN')
                ni = ((st.get('execution') or {}).get('nav_idle')) or {}
                for k in ('navigate_to_pose', 'follow_path'):
                    why = (ni.get(k) or {}).get('reason') or ''
                    if 'MISSING' in why or 'NOT_UNIQUE' in why: degraded.append(f'NAV2:{why}')
                if wdr is None or wdr.get('state') is None: degraded.append('WATCHDOG:NO_DATA')
                else:
                    if wdr.get('state') != 'MOTION_AUTHORIZED': degraded.append(f"WATCHDOG:{wdr.get('state')}")
                    wa = _num(wdr.get('age_s'))
                    if wa is None or wa < 0 or wa > WATCHDOG_STATUS_STALE_S: degraded.append('WATCHDOG:STALE')
            level = 'FAULT' if fault else 'DEGRADED' if degraded else 'READY'
            mp_block = [] if level == 'READY' else ['NOT_READY']
            if level == 'READY' and (gate or {}).get('movement_permitted') is not True:
                mp_block += [str(x) for x in ((gate or {}).get('movement_block_reasons') or ['GATE_MOVEMENT_NOT_PERMITTED'])][:4]
            return {'robot_id': robot_id, 'level': level, 'reasons': fault + degraded, 'checked_at': now,
                    'motion_permitted': not mp_block, 'motion_block_reasons': mp_block, 'supervisor_required': required,
                    'required_services': list(self._required_services()), 'required_topics': list(self._required_topics()),
                    'supervisor': None if sup is None else {'boot_id': sup['boot_id'], 'age_s': _round(_age(now, sup.get('received_at'))), 'host': sup.get('host')},
                    'services': (sup or {}).get('services', []), 'ros': (sup or {}).get('ros') or {},
                    'actions': [dict(a) for a in self.service_actions.get(robot_id, [])[-10:]] if hasattr(self, 'service_actions') else []}

    def require_ready(self, robot_id, what):
        from .service import ApiError
        if not self._supervisor_required(self.robots.get(robot_id)):
            return                     # explicitly simulated mock robot on a server without --require-supervisor-ready (legacy E2E)
        rd = self.readiness(robot_id)
        if rd['level'] != 'READY':
            raise ApiError(409, 'robot_not_ready', f"{what} refused: robot is {rd['level']} ({', '.join(rd['reasons'][:4])})")

    def _service_idle_block(self, robot_id):
        """None when fresh Agent evidence proves the robot idle, else a (code, message) refusal. Used for stop / restart, which interrupt a
        running service: FAULT, TRANSITIONING, RECONCILIATION_REQUIRED and an unknown / stale state are NOT idle (Codex 2026-11-21)."""
        r = self.robots.get(robot_id)
        if r is None or not r.get('online'):
            return 'agent_offline', 'the Agent is offline: no evidence that the robot is idle (use start / retry for a STOPPED / FAILED service)'
        a = _age(time.time(), r.get('state_ts'))
        if a is None or a > AGENT_STATE_STALE_S:
            return 'agent_state_stale', 'the Agent state is stale: no fresh evidence that the robot is idle'
        if r.get('reconciliation_required') or self.db.one("SELECT 1 FROM missions WHERE robot_id=? AND status='RECONCILIATION_REQUIRED'", (robot_id,)):
            return 'reconciliation_required', 'reconcile the robot (admin recover) before stopping a service'
        if self.db.one("SELECT 1 FROM missions WHERE robot_id=? AND status IN ('QUEUED','DISPATCHED','ACCEPTED','EXECUTING','CANCELING')", (robot_id,)):
            return 'mission_active', 'a mission is in flight'
        rec = self._mode_load(robot_id) if hasattr(self, '_mode_load') else None
        if rec is not None or getattr(self.cfg, 'require_operating_mode', False):
            mode = (self.mode_view(robot_id, advance=False) or {})
            t = mode.get('transition') or {}
            if t.get('state') in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'):
                return 'transition_in_progress', 'a mode transition is running'
            if mode.get('mode') in ('MANUAL', 'MAPPING', 'NAVIGATION'):
                return 'robot_busy', f"switch the robot to IDLE first (mode {mode.get('mode')})"
            if mode.get('mode') != 'IDLE':
                return 'robot_not_idle', f"mode {mode.get('mode')} is not IDLE (recover the robot to IDLE first)"
        st = r.get('state') or {}
        ex = st.get('execution') if isinstance(st.get('execution'), dict) else {}
        if ex.get('active') is not None:
            return 'robot_busy', 'the Agent reports an active goal'
        ng = ex.get('nav2_goals')
        if isinstance(ng, dict) and ng.get('active_count') not in (0, None):
            return 'robot_busy', 'Nav2 reports an active goal'
        if isinstance(ng, dict) and (ng.get('active_goal_uuids') or ng.get('foreign_active_goal_uuids')):
            return 'robot_busy', 'Nav2 reports an active goal'
        mw = st.get('mapping_worker') if isinstance(st.get('mapping_worker'), dict) else {}
        if mw.get('status') not in (None, 'IDLE', 'DISABLED', 'STOPPED') or mw.get('processes_started') is True:
            return 'robot_busy', f"a mapping session is not idle ({mw.get('status')})"
        if robot_id in getattr(self, 'manual_benches', {}):
            return 'robot_busy', 'a manual session is open'
        arb = st.get('velocity_arbiter') if isinstance(st.get('velocity_arbiter'), dict) else {}
        if arb.get('source') not in (None, 'NONE'):
            return 'robot_busy', f"the velocity arbiter has an active source ({arb.get('source')})"
        return None

    def request_service_action(self, robot_id, principal, body):
        """Admin-only, lease-bound request for one named service action of the robot's own supervisor configuration.
        start / retry : the service must be STOPPED / FAILED / BACKOFF (bringing a process up grants no motion).
        stop / restart: fresh Agent evidence must prove the robot idle (_service_idle_block)."""
        from .service import ApiError
        if principal.role != 'admin':
            self.db.event('service_action_refused', f'{principal.name} ({principal.role}) may not control services', robot_id, level='warning')
            raise ApiError(403, 'forbidden', 'only an admin may start, stop or restart robot services')
        if (not isinstance(body, dict) or type(body.get('action')) is not str or body.get('action') not in ACTIONS
                or type(body.get('service')) is not str or not body['service']):
            raise ApiError(400, 'bad_service_action', 'action must be one of ' + ', '.join(ACTIONS) + ' and service a reported name')
        if type(body.get('epoch')) is not int:
            raise ApiError(400, 'bad_epoch', 'epoch (the integer epoch of the lease this admin holds) is required')
        sups = self._sup()
        with self.lock:
            sup = sups.get(robot_id)
            if robot_id not in self.robots and sup is None:
                raise ApiError(404, 'unknown_robot', 'robot not registered')
            lease = self.lease_view(robot_id)
            if not lease or lease['owner'] != principal.name or lease['epoch'] != body['epoch']:
                raise ApiError(409, 'lease_required', 'hold the live lease of this robot (and send its epoch) to control its services')
            sa = None if sup is None else _age(time.time(), sup.get('received_at'))
            if sa is None or sa > SUPERVISOR_STALE_S:
                raise ApiError(409, 'supervisor_unavailable', 'no fresh supervisor report for this robot')
            svc = next((s for s in sup['services'] if s.get('name') == body['service']), None)
            if svc is None:
                raise ApiError(404, 'unknown_service', 'service is not in the supervisor configuration')
            if body['action'] not in (svc.get('actions') or ACTIONS):
                raise ApiError(409, 'action_not_allowed', f"{body['action']} is not allowed for {body['service']}")
            if body['action'] in ('start', 'retry'):
                if svc.get('state') not in ('STOPPED', 'FAILED', 'BACKOFF'):
                    raise ApiError(409, 'service_not_stopped', f"{body['service']} is {svc.get('state')}; start / retry only bring a STOPPED / FAILED service up")
            else:
                blk = self._service_idle_block(robot_id)
                if blk:
                    self.db.event('service_action_refused', f"{body['action']} {robot_id}/{body['service']}: {blk[0]}", robot_id, level='warning')
                    raise ApiError(409, blk[0], blk[1])
            q = self.service_actions.setdefault(robot_id, [])
            if any(a['status'] in ('QUEUED', 'SENT') for a in q):
                raise ApiError(409, 'action_pending', 'a previous service action is still pending')
            now = time.time()
            a = {'action_id': 'sa_' + uuid.uuid4().hex[:12], 'service': body['service'], 'action': body['action'], 'status': 'QUEUED',
                 'requested_by': principal.name, 'supervisor_boot': sup['boot_id'], 'created_at': now, 'updated_at': now, 'detail': ''}
            q.append(a); del q[:-50]
            self.db.event('service_action', f"{principal.name} requested {body['action']} {robot_id}/{body['service']}", robot_id,
                          data={'action_id': a['action_id']})
            return dict(a)
