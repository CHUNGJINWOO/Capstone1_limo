"""Exclusive operating mode per robot (IDLE / MANUAL / MAPPING / NAVIGATION) and the transitions between them.

A transition is a server-driven sequence; the HTTP request only queues it (202):

  STOPPING_PREVIOUS  end whatever is active, through the existing mechanisms only:
                     manual-bench session dropped; mapping session -> the existing STOP command (worker evidence);
                     non-terminal Fleet missions -> the existing cancel_mission (Nav2 result reported by the Agent)
  VERIFYING_IDLE     fresh Agent report shows, continuously for VERIFY_HOLD_S: no Agent-owned goal, the stopped
                     owned goals have terminal results, NavigateToPose/FollowPath idle evidence, Motion Watchdog
                     authorized (its own cancel-all proof), Velocity Arbiter output zero, Mapping Worker idle
  CONFIRMING         the target mode is published to the Agent; waits until the Agent's Velocity Arbiter reports that it
                     operates in that mode (agents without an Arbiter skip this step)
  ACTIVE             target mode entered (MANUAL / MAPPING / NAVIGATION only allow their own commands)
  FAILED             nothing restarts by itself. Mode becomes IDLE when the previous activity was proven stopped,
                     FAULT otherwise (FAULT admits no motion; a new transition to IDLE re-runs the stop/verify steps)

Publisher-GID continuity and an empty cancel-all are inputs, never sufficient on their own: the goals this
transition stopped must also have terminal results (mission status from the Nav2 action result, mapping STOP
evidence `goal_terminal`).

The mode is invalidated (IDLE, or FAULT while a mapping session is unresolved) when the Lease owner/epoch, the Agent
boot or the server boot changes. Robots that never received a mode request keep the previous (legacy) behaviour unless
the server runs with `require_operating_mode`.
"""
import json
import time
import uuid

MODES = ('IDLE', 'MANUAL', 'MAPPING', 'NAVIGATION')
TRANSITION_TIMEOUT_S = 30.0
VERIFY_HOLD_S = 1.0
STATE_FRESH_S = 1.5
ACTIVE_MISMATCH_GRACE_S = 3.0      # a fresh Agent report that contradicts an ACTIVE mode this long makes the mode FAULT (no automatic return to it)
MOTION_MODES = ('MANUAL', 'MAPPING', 'NAVIGATION')
MISSION_TERMINAL = {'SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED'}
MAPPING_BUSY = ('STARTING', 'PAUSING', 'SAVING', 'STOPPING', 'RECONCILING')


class OperatingModeMixin:
    # ------------------------------------------------------------ storage
    def _mode_key(self, robot_id):
        return 'operating_mode:' + robot_id

    def _mode_load(self, robot_id):
        row = self.db.one('SELECT v FROM meta WHERE k=?', (self._mode_key(robot_id),))
        if row:
            return json.loads(row['v'])
        if getattr(getattr(self, 'cfg', None), 'require_operating_mode', False) and robot_id in self.robots:
            return self._mode_new('IDLE', None, 'initial')
        return None

    def _mode_new(self, mode, previous, reason):
        return {'mode': mode, 'revision': (previous or {}).get('revision', 0) + 1, 'epoch': None, 'owner': None,
                'agent_boot': None, 'server_boot': self.boot_id, 'since': time.time(), 'reason': reason,
                'transition': (previous or {}).get('transition')}

    def _mode_save(self, robot_id, rec):
        with self.db.transaction():
            self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', (self._mode_key(robot_id), json.dumps(rec)))

    def _mode_set(self, robot_id, rec, mode, reason, level='info'):
        if rec['mode'] != mode or rec.get('reason') != reason:
            old = rec['mode']
            rec.update(mode=mode, revision=rec.get('revision', 0) + 1, since=time.time(), reason=reason,
                       server_boot=self.boot_id)
            if mode in ('IDLE', 'FAULT'):
                rec.update(epoch=None, owner=None, agent_boot=None)
            self.db.event('operating_mode', f'{old} -> {mode}: {reason}', robot_id, level=level)
        return rec

    # ------------------------------------------------------------ view / invalidation
    def mode_view(self, robot_id, advance=True):
        """Current mode record (None = legacy robot that never selected a mode). Applies invalidation first."""
        with self.lock:
            rec = self._mode_load(robot_id)
            if rec is None:
                return None
            changed = self._mode_invalidate(robot_id, rec)
            if advance and rec.get('transition') and rec['transition']['state'] in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'):
                changed = self._mode_advance(robot_id, rec) or changed
            if changed:
                self._mode_save(robot_id, rec)
            return rec

    def _mapping_unresolved(self, robot_id):
        try:
            return self.operation_status(robot_id)['state'] not in ('IDLE', 'STOPPED')
        except Exception:
            return True

    def _mode_invalidate(self, robot_id, rec):
        if rec['mode'] in ('IDLE', 'FAULT'):
            return False
        r = self.robots.get(robot_id) or {}
        lease = self.lease_view(robot_id)
        why = None
        if rec.get('server_boot') != self.boot_id:
            why = 'server restarted'
        elif not lease or lease['epoch'] != rec.get('epoch') or lease['owner'] != rec.get('owner'):
            why = 'lease expired or changed'
        elif not self.agent_contract_ok(robot_id):
            why = 'agent protocol or safety contract incompatible'
        elif rec['mode'] != 'TRANSITIONING' and (r.get('agent') or {}).get('boot_id') != rec.get('agent_boot'):
            why = 'agent restarted'
        elif (rec.get('transition') or {}).get('state') in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING') and \
                (r.get('agent') or {}).get('boot_id') != rec['transition'].get('agent_boot'):
            why = 'agent restarted during the transition (request voided, no automatic completion)'
        elif rec['mode'] in MOTION_MODES and (rec.get('transition') or {}).get('state') not in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'):
            blocker, hard = self._mode_active_blocker(robot_id, rec)
            seen = getattr(self, '_mode_mismatch', None)
            if seen is None:
                seen = self._mode_mismatch = {}
            if blocker and hard:
                first = seen.setdefault(robot_id, (time.monotonic(), rec['revision']))
                if first[1] != rec['revision']:
                    first = seen[robot_id] = (time.monotonic(), rec['revision'])
                if hard == 'LATCHED' or time.monotonic() - first[0] >= ACTIVE_MISMATCH_GRACE_S:
                    why = f"the Agent's Watchdog no longer confirms {rec['mode']}: {blocker}"
            else:
                seen.pop(robot_id, None)
        if why is None:
            return False
        t = rec.get('transition')
        in_flight = bool(t and t['state'] in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'))
        if in_flight:
            t.update(state='FAILED', reason=why, updated_at=time.time())
        # An interrupted transition never proved the stop: FAULT (a new transition must verify idle first).
        self._mode_set(robot_id, rec, 'FAULT' if (in_flight or self._mapping_unresolved(robot_id)) else 'IDLE', why, 'warning')
        self.manual_benches.pop(robot_id, None)
        self.navigation_clear(robot_id, why)
        return True

    def _mode_active_blocker(self, robot_id, rec):
        """(blocker, hard) for an ACTIVE motion mode on a WATCHDOG_SELECTOR Agent, from the newest Agent report. None = the Watchdog confirms exactly this mode now.
        hard=True: a fresh report contradicts the mode (Watchdog mode/revision/boot/channel/state/exclusivity). hard=False: no usable evidence (offline, stale)."""
        r = self.robots.get(robot_id) or {}
        agent = r.get('agent') or {}
        if self._selector_kind(agent) != 'WATCHDOG_SELECTOR' or rec.get('mode') not in MOTION_MODES:
            return None, False
        if not r.get('online') or not r.get('reconciled') or r.get('reconciliation_required'):
            return 'agent offline or not reconciled', False
        if not 0 <= time.time() - (r.get('state_ts') or 0) <= STATE_FRESH_S:
            return 'agent report stale', False
        state = r.get('state') or {}
        arb = state.get('velocity_arbiter')
        if not isinstance(arb, dict) or arb.get('kind') != 'WATCHDOG_SELECTOR':
            return 'velocity selector report missing', True
        if arb.get('operating_mode') != rec['mode']:
            return f"agent selector reports {arb.get('operating_mode')}, the server mode is {rec['mode']}", True
        why = self._selector_evidence_blocker(state, arb, rec['mode'], rec.get('revision'))
        if not why and rec.get('watchdog_boot') and (arb.get('evidence') or {}).get('watchdog_boot_id') != rec['watchdog_boot']:
            why = 'the Watchdog restarted after the mode was confirmed (the old confirmation does not carry over)'
        trips = (arb.get('evidence') or {}).get('watchdog_trips')
        if not why and rec.get('watchdog_trips') is not None and (not isinstance(trips, int) or trips > rec['watchdog_trips']):
            return 'the Watchdog tripped (FAILSAFE) after the mode was confirmed: a trip ends the mode, the mode is not restored when the Watchdog re-authorizes', 'LATCHED'
        return (why, True) if why else (None, False)

    def mode_motion_status(self, robot_id):
        """Mode change is not motion permission: whether the Watchdog currently confirms the ACTIVE mode. verified None = not applicable (no mode, IDLE/FAULT, transition
        in flight, or an Agent without a Watchdog selector); False = new motion commands and mission admissions are refused."""
        with self.lock:
            rec = self._mode_load(robot_id)
            if rec is None or rec.get('mode') not in MOTION_MODES:
                return {'verified': None, 'blocker': None, 'mode': None if rec is None else rec.get('mode')}
            if (rec.get('transition') or {}).get('state') in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'):
                return {'verified': None, 'blocker': 'transition in progress', 'mode': rec['mode']}
            blocker, hard = self._mode_active_blocker(robot_id, rec)
            applicable = self._selector_kind((self.robots.get(robot_id) or {}).get('agent') or {}) == 'WATCHDOG_SELECTOR'
            return {'verified': (blocker is None) if applicable else None, 'blocker': blocker, 'hard': hard if blocker else False, 'mode': rec['mode'], 'revision': rec.get('revision')}

    def agent_mode_view(self, robot_id):
        """What the Agent's selector needs (None for legacy robots)."""
        rec = self.mode_view(robot_id)
        if rec is None:
            return None
        t = rec.get('transition') or {}
        if not self.agent_contract_ok(robot_id):
            return {'mode': 'IDLE', 'operation_mode': 'IDLE', 'revision': rec['revision'], 'epoch': None,
                    'transition_state': t.get('state'), 'server_boot': rec.get('server_boot')}
        # While a transition is still stopping a NAVIGATION mission, the operation producer keeps NAVIGATION so the
        # mission's Safety Context stays valid and its cancel ends with a Nav2 result instead of a reconciliation.
        # Velocity is already zero: the Arbiter selects nothing while the mode is TRANSITIONING.
        operation = 'NAVIGATION' if (rec['mode'] == 'TRANSITIONING' and t.get('from') == 'NAVIGATION'
                                     and t.get('state') == 'STOPPING_PREVIOUS') else rec['mode']
        revision = t.get('from_revision', rec['revision']) if operation != rec['mode'] else rec['revision']
        return {'mode': rec['mode'], 'operation_mode': operation, 'revision': revision, 'epoch': rec.get('epoch'),
                'transition_state': t.get('state'), 'server_boot': rec.get('server_boot')}

    def require_mode(self, robot_id, needed, code, motion=True):
        """Admission guard for manual / mapping START-RESUME / mission creation. Legacy robots pass. motion=False (map LOAD / INITIAL_POSE / UNLOAD: nothing moves) only checks the mode record."""
        from .service import ApiError
        rec = self.mode_view(robot_id)
        if rec is None:
            return
        if rec['mode'] != needed:
            raise ApiError(409, code, f"operating mode is {rec['mode']}; switch to {needed} first")
        t = rec.get('transition') or {}
        if t.get('state') in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'):
            raise ApiError(409, code, f"mode transition to {needed} is {t['state']}")
        if needed in MOTION_MODES and motion:
            ms = self.mode_motion_status(robot_id)
            if ms['verified'] is False:
                raise ApiError(409, 'mode_unverified', f"{needed} is the server mode but the Watchdog does not confirm it now ({ms['blocker']}); no new motion or mission is admitted")

    # ------------------------------------------------------------ requests
    def request_mode(self, robot_id, actor, body):
        from .service import ApiError
        if not isinstance(body, dict):
            raise ApiError(400, 'bad_mode_request', 'JSON object required')
        target, key = body.get('target'), body.get('idempotency_key')
        if target not in MODES:
            raise ApiError(400, 'bad_mode', 'target must be one of ' + ', '.join(MODES))
        if not isinstance(key, str) or not 1 <= len(key) <= 100:
            raise ApiError(400, 'bad_mode_request', 'idempotency_key required')
        with self.lock:
            if robot_id not in self.robots:
                raise ApiError(404, 'unknown_robot', 'robot not registered')
            lease = self.lease_view(robot_id)
            if not lease or lease['owner'] != actor or type(body.get('epoch')) is not int or lease['epoch'] != body['epoch']:
                raise ApiError(409, 'lease_required', 'current owner and epoch required')
            if target == 'MAPPING':
                self._mapping_real_block(robot_id)                  # real-robot block first: its reason must stay visible
            if target != 'IDLE':
                self.require_agent_contract(robot_id, f'{target} mode')
            if target in ('MAPPING', 'NAVIGATION'):
                self.require_ready(robot_id, f'{target} mode')      # Phase 4: profile flag, or always for a real Agent
            rec = self.mode_view(robot_id) or self._mode_new('IDLE', None, 'first mode request')
            t = rec.get('transition')
            if t and t.get('idempotency_key') == key and t.get('actor') == actor:
                return {'duplicate': True, 'mode': rec}
            if t and t['state'] in ('STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'):
                raise ApiError(409, 'transition_in_progress', f"{t['from']} -> {t['to']} is {t['state']}")
            try:
                mapping_state = self.operation_status(robot_id)['state']
            except Exception:
                mapping_state = 'UNKNOWN'
            if mapping_state in MAPPING_BUSY:
                raise ApiError(409, 'mapping_busy', f'mapping command in progress ({mapping_state}); retry when it finishes')
            agent = (self.robots[robot_id].get('agent') or {})
            if target == 'MANUAL' and agent.get('manual_web_bench') is not True and not self._selector_manual_path(self.robots[robot_id]):
                raise ApiError(409, 'mode_unsupported', 'agent has no manual input path')
            if target == 'MAPPING':
                self._mapping_worker_robot(robot_id)      # keeps mapping_real_blocked for real agents
            if target == 'NAVIGATION' and agent.get('missions_enabled') is not True and agent.get('real_nav_dry_run') is not True:
                raise ApiError(409, 'mode_unsupported', 'agent has missions disabled')
            if rec['mode'] == target and target != 'IDLE' and rec.get('epoch') == lease['epoch']:
                return {'duplicate': False, 'unchanged': True, 'mode': rec}
            now = time.time()
            rec['transition'] = {'id': uuid.uuid4().hex[:12], 'from': rec['mode'], 'to': target, 'state': 'STOPPING_PREVIOUS',
                                 'actor': actor, 'epoch': lease['epoch'], 'idempotency_key': key, 'requested_at': now,
                                 'updated_at': now, 'deadline': now + TRANSITION_TIMEOUT_S, 'reason': None,
                                 'stopped': {'manual': False, 'mapping_session': None, 'missions': []},
                                 'verify_since': None, 'blocker': None, 'from_revision': rec.get('revision'),
                                 # generation pinned at request time: a new Agent boot or server boot voids this request
                                 'agent_boot': (self.robots[robot_id].get('agent') or {}).get('boot_id'), 'server_boot': self.boot_id}
            self._mode_set(robot_id, rec, 'TRANSITIONING', f"{rec['transition']['from']} -> {target} requested by {actor}")
            rec['epoch'], rec['owner'] = lease['epoch'], actor
            self._mode_advance(robot_id, rec)
            self._mode_save(robot_id, rec)
            return {'duplicate': False, 'mode': rec}

    # ------------------------------------------------------------ driver
    def _mode_fail(self, robot_id, rec, reason, stopped_ok):
        t = rec['transition']
        t.update(state='FAILED', reason=reason, updated_at=time.time())
        mode = 'IDLE' if stopped_ok and not self._mapping_unresolved(robot_id) else 'FAULT'
        self._mode_set(robot_id, rec, mode, f"transition to {t['to']} failed: {reason}", 'warning')
        self.manual_benches.pop(robot_id, None)
        self.navigation_clear(robot_id, 'mode transition failed')
        return True

    def _mode_advance(self, robot_id, rec):
        t = rec['transition']
        now = time.time()
        if t['state'] == 'STOPPING_PREVIOUS':
            done, failure = self._mode_stop_step(robot_id, rec)
            if failure:
                return self._mode_fail(robot_id, rec, failure, stopped_ok=False)
            if not done:
                if now > t['deadline']:
                    return self._mode_fail(robot_id, rec, 'previous activity did not stop in time', stopped_ok=False)
                return True
            t.update(state='VERIFYING_IDLE', updated_at=now, verify_since=None)
            if t['from'] == 'NAVIGATION' or t['to'] != 'NAVIGATION':
                self.navigation_clear(robot_id, f"leaving NAVIGATION ({t['from']} -> {t['to']})")
        if t['state'] == 'VERIFYING_IDLE':
            blocker = self._mode_idle_blocker(robot_id, t)
            t['blocker'] = blocker
            if blocker:
                t['verify_since'] = None
                if now > t['deadline']:
                    # Unproven stop is never reported as IDLE: FAULT until a later transition proves idle.
                    return self._mode_fail(robot_id, rec, 'idle not proven: ' + blocker, stopped_ok=False)
                return True
            if t['verify_since'] is None:
                t['verify_since'] = now
                return True
            if now - t['verify_since'] < VERIFY_HOLD_S:
                return True
            agent_boot = ((self.robots.get(robot_id) or {}).get('agent') or {}).get('boot_id')
            t.update(state='CONFIRMING', updated_at=now, confirm_since=now)
            self._mode_set(robot_id, rec, t['to'], f"{t['to']} published to the Agent (stop and idle verified)")
            if t['to'] != 'IDLE':
                rec.update(epoch=t['epoch'], owner=t['actor'], agent_boot=agent_boot)
            return True
        if t['state'] == 'CONFIRMING':
            r = self.robots.get(robot_id) or {}
            fresh = (r.get('state_ts') or 0) > t['confirm_since']
            # Re-check the full stop/safety evidence on a report newer than the publication, then the Agent's own
            # confirmation; a failure in between keeps waiting and ends in FAULT at the deadline (never ACTIVE).
            blocker = self._mode_idle_blocker(robot_id, t) if fresh else 'waiting for an Agent report after publication'
            if blocker is None:
                blocker = self._mode_confirm_blocker(r, t['to'], rec.get('revision'))
            t['blocker'] = blocker
            if blocker is None:
                ev_boot = (((r.get('state') or {}).get('velocity_arbiter') or {}).get('evidence') or {}).get('watchdog_boot_id')
                rec['watchdog_trips'] = (((r.get('state') or {}).get('velocity_arbiter') or {}).get('evidence') or {}).get('watchdog_trips')
                rec['watchdog_boot'] = ev_boot if self._selector_kind(r.get('agent') or {}) == 'WATCHDOG_SELECTOR' and t['to'] != 'IDLE' else None
                t.update(state='ACTIVE', updated_at=now)
                self.db.event('operating_mode', f"{t['to']} confirmed by the Agent selector", robot_id)
                return True
            if now > t['deadline']:
                return self._mode_fail(robot_id, rec, 'agent did not confirm the new mode safely: ' + blocker, stopped_ok=False)
            return True
        return False

    def _mode_stop_step(self, robot_id, rec):
        """(done, failure). Ends manual input, the mapping session and Fleet missions through their own paths."""
        from .service import ApiError
        t = rec['transition']
        stopped = t['stopped']
        self.manual_benches.pop(robot_id, None)        # the Agent's sample expires within its 250 ms TTL
        stopped['manual'] = True
        try:
            s = self.operation_status(robot_id)
        except ApiError:
            s = {'state': 'IDLE'}
        state = s['state']
        if state == 'UNKNOWN':
            return False, 'mapping session UNKNOWN (reconciliation required)'
        if state in ('MAPPING', 'PAUSED'):
            lease = self.lease_view(robot_id)
            try:
                self.request_operation(robot_id, t['actor'], {
                    'action': 'STOP', 'epoch': lease['epoch'] if lease else None, 'session_id': s.get('session_id'),
                    'idempotency_key': 'mode-' + t['id'] + '-stop'})
            except ApiError as exc:
                return False, f'mapping STOP refused: {exc.code}'
            stopped['mapping_session'] = s.get('session_id')
            return False, None
        if state in MAPPING_BUSY:
            return False, None
        if stopped.get('mapping_session') and s.get('session_id') == stopped['mapping_session']:
            result = s.get('result') or {}
            if result.get('action') != 'STOP' or result.get('status') != 'COMPLETED' or (result.get('evidence') or {}).get('goal_terminal') is not True:
                return False, 'mapping STOP finished without goal-terminal evidence'
        pending = [m for m in self.db.q('SELECT mission_id,status FROM missions WHERE robot_id=?', (robot_id,))
                   if m['status'] not in MISSION_TERMINAL]
        for m in pending:
            if m['status'] == 'RECONCILIATION_REQUIRED':
                continue        # may still end with a Nav2 result; otherwise the deadline fails the transition (FAULT)
            if m['mission_id'] not in stopped['missions']:
                stopped['missions'].append(m['mission_id'])
            if m['status'] != 'CANCELING':
                self.cancel_mission(m['mission_id'], requested_by=t['actor'] + ' (mode change)')
        return not pending, None

    # G2 2026-11-23: what the Watchdog selects per mode (the server's own copy: it does not take the Agent's word for the table)
    SELECTOR_CHANNEL = {'MANUAL': 'manual', 'MAPPING': 'nav', 'NAVIGATION': 'nav'}
    SELECTOR_NODES = {'nav': ('velocity_smoother',), 'manual': ('manual_mapping_teleop',)}

    def _selector_manual_path(self, r):
        """A real Agent has no web bench; its manual input is the on-board joystick on the Watchdog's manual channel. It counts as a manual input path only while the
        Agent's own selector (a WATCHDOG_SELECTOR) reports, from the graph, exactly the allowed joystick publisher on that channel and an exclusive channel."""
        agent = r.get('agent') or {}
        arb = (r.get('state') or {}).get('velocity_arbiter')
        if agent.get('velocity_selector') != 'WATCHDOG_SELECTOR' or not isinstance(arb, dict) or arb.get('kind') != 'WATCHDOG_SELECTOR':
            return False
        graph = (arb.get('evidence') or {}).get('agent_graph')
        return isinstance(graph, dict) and graph.get('manual_publishers') == list(self.SELECTOR_NODES['manual']) and (arb.get('evidence') or {}).get('manual_input_ok') is True

    @staticmethod
    def _selector_kind(agent):
        if agent.get('locked_velocity_arbiter') is True:
            return 'ARBITER'
        return 'WATCHDOG_SELECTOR' if agent.get('velocity_selector') == 'WATCHDOG_SELECTOR' else None

    def _mode_confirm_blocker(self, r, target, revision=None):
        """The Agent selector must report the target mode AND be ready in it (capability contract).
        A real Agent has no Arbiter; its selector (WATCHDOG_SELECTOR) is accepted only with Watchdog evidence the server re-checks here: the Watchdog itself
        reports the same mode and revision, selects the channel this server table assigns to the mode, both input channels are exclusive with allowed
        publishers, there is no source conflict, and it is the Watchdog whose evidence the server already verified for the speed clamp."""
        agent = r.get('agent') or {}
        state = r.get('state') or {}
        arb = state.get('velocity_arbiter')
        kind = self._selector_kind(agent)
        if kind is None:
            return None if target == 'IDLE' else 'agent has no velocity selector to confirm ' + target
        if not isinstance(arb, dict):
            return 'velocity arbiter report missing'
        if kind == 'WATCHDOG_SELECTOR' and arb.get('kind') != 'WATCHDOG_SELECTOR':
            return 'velocity selector report is not a Watchdog selector report'
        if arb.get('operating_mode') != target:
            return f"agent selector still in {arb.get('operating_mode')}"
        expected = {'IDLE': 'MODE_IDLE', 'MANUAL': None, 'MAPPING': 'NO_OWNED_GOAL', 'NAVIGATION': 'NO_OWNED_GOAL'}[target]
        if arb.get('blocker') != expected:
            return f"agent selector not ready: {arb.get('blocker')}"
        if kind == 'WATCHDOG_SELECTOR' and target != 'IDLE':
            why = self._selector_evidence_blocker(state, arb, target, revision)
            if why:
                return why
        return None

    def _selector_evidence_blocker(self, state, arb, target, revision=None):
        """The server-side re-check of the Watchdog evidence in a WATCHDOG_SELECTOR report for `target` (used at confirmation and again while the mode is ACTIVE)."""
        ev = arb.get('evidence')
        if not isinstance(ev, dict):
            return 'selector evidence missing'
        chan = self.SELECTOR_CHANNEL[target]
        if ev.get('watchdog_state') != 'MOTION_AUTHORIZED' or ev.get('watchdog_mode') != target:
            return f"the Watchdog does not confirm {target} (it reports {ev.get('watchdog_mode')})"
        if revision is not None and ev.get('watchdog_revision') != revision:
            return f"the Watchdog confirms mode revision {ev.get('watchdog_revision')}, not {revision}"
        if ev.get('selected_channel') != chan:
            return 'the Watchdog selects the wrong input channel for ' + target
        if ev.get('nav_input_ok') is not True or ev.get('manual_input_ok') is not True:
            return 'a motion input channel is not exclusive'
        conflict = ev.get('source_conflict_age_s')
        if isinstance(conflict, (int, float)) and conflict < 3.0:
            return 'a motion source conflict was seen'
        graph = ev.get('agent_graph') if isinstance(ev.get('agent_graph'), dict) else {}
        for name, nodes in (('nav_publishers', self.SELECTOR_NODES['nav']), ('manual_publishers', self.SELECTOR_NODES['manual'])):
            pubs = graph.get(name)
            if not isinstance(pubs, list) or len(pubs) > 1 or any(p not in nodes for p in pubs):
                return 'the Agent sees an unexpected publisher on the motion input (' + name + ')'
        sl = (state.get('speed_limits') or {}).get('watchdog_evidence') or {}
        if sl.get('server_verified') is not True or sl.get('watchdog_boot_id') != ev.get('watchdog_boot_id'):
            return 'the Watchdog evidence is not verified or belongs to another Watchdog boot'
        return None

    def _mode_idle_blocker(self, robot_id, t):
        """First reason the robot is not proven idle (None = idle) from a fresh Agent report."""
        r = self.robots.get(robot_id) or {}
        state = r.get('state') or {}
        if not r.get('online') or not r.get('reconciled') or r.get('reconciliation_required'):
            return 'agent offline or not reconciled'
        if t.get('to') != 'IDLE' and not self.agent_contract_ok(robot_id):
            return 'agent protocol or safety contract incompatible'
        if not 0 <= time.time() - (r.get('state_ts') or 0) <= STATE_FRESH_S:
            return 'agent report stale'
        if (r.get('state_ts') or 0) <= t.get('requested_at', 0):
            return 'waiting for an Agent report newer than the request'
        if t.get('agent_boot') and (r.get('agent') or {}).get('boot_id') != t['agent_boot']:
            return 'agent boot changed'
        for mid in t['stopped']['missions']:
            m = self.db.one('SELECT status FROM missions WHERE mission_id=?', (mid,))
            if not m or m['status'] not in MISSION_TERMINAL:
                return f'mission {mid} has no terminal result'
        execution = state.get('execution') or {}
        if execution.get('active') is not None:
            return 'agent still owns a Nav2 goal'
        goals = execution.get('nav2_goals') or {}
        if goals.get('foreign_active_goal_uuids'):
            return 'foreign NavigateToPose goal active'
        if goals.get('active_count') not in (0, None):
            return 'NavigateToPose goal active'
        idle = execution.get('nav_idle') or {}
        if idle.get('ok') is not True:
            return 'action idle evidence missing: ' + str(idle.get('reason') or 'not reported')
        agent = r.get('agent') or {}
        locked = self._selector_kind(agent) is not None
        wd = state.get('motion_watchdog')
        if not isinstance(wd, dict) or wd.get('required') is not True:
            if locked or t.get('to') != 'IDLE':
                return 'motion watchdog report missing (required for ' + str(t.get('to')) + ')'
        elif wd.get('state') != 'MOTION_AUTHORIZED' or wd.get('agent_boot_matches') is not True:
            return 'motion watchdog not authorized (' + str(wd.get('state')) + ')'
        arb = state.get('velocity_arbiter')
        if locked and not isinstance(arb, dict):
            return 'velocity arbiter report missing'
        if isinstance(arb, dict) and (arb.get('selected_linear') not in (0, 0.0) or arb.get('selected_turn') not in (0, 0.0)):
            return 'velocity arbiter output not zero'
        loc = state.get('localization') or {}
        if loc.get('processes_started'):
            return 'saved-map localization (map_server/AMCL) still running'
        worker = state.get('mapping_worker') or {}
        if worker.get('enabled') and (worker.get('status') != 'IDLE' or worker.get('processes_started')):
            return 'mapping worker not idle (' + str(worker.get('status')) + ')'
        return None
