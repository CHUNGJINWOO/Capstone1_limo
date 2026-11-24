"""Control-ownership lease and operator recovery.

Lease: per robot, at most one live lease. Movement admission also requires a live lease and matching epoch. Only its owner (the authenticated token NAME) may create missions for that robot: any other
creator gets HTTP 409 `ownership_conflict` instead of being queued. Check-and-create and check-and-acquire run under the same server lock, so there is no
window between "ownership checked" and "mission created". A lease expires after `ttl_s` unless renewed (a dead RMF adapter cannot block the robot forever);
expiry never touches a running mission. Cancel stays allowed for every control role (stopping is the safe direction).
Acquire is refused (409) while another principal still has a queued/in-flight mission or while reconciliation is required: ownership can only be taken over
a robot that is not being driven by someone else.

Recovery of RECONCILIATION_REQUIRED requires authenticated terminal and ownership
evidence. No such producer is implemented yet: an empty status array is not
enough, and both operator recovery routes remain blocked with HTTP 409."""
import json, math, time

LEASE_MIN_TTL, LEASE_MAX_TTL, LEASE_DEFAULT_TTL = 5.0, 300.0, 20.0

class OwnershipMixin:
    # ------------------------------------------------------------ lease
    def lease_view(self, robot_id, now=None):
        now = now or time.time(); r = self.db.one('SELECT * FROM locks WHERE robot_id=?', (robot_id,))
        if not r or type(r['expires_at']) not in (int,float) or not math.isfinite(r['expires_at']) or r['expires_at'] <= now or type(r['epoch']) is not int or r['epoch'] < 0: return None
        return {'robot_id': robot_id, 'owner': r['owner'], 'epoch': r['epoch'], 'acquired_at': r['acquired_at'], 'expires_at': r['expires_at'], 'ttl_s': r['ttl_s'], 'note': r['note'],
                'remaining_s': round(r['expires_at'] - now, 1)}

    def lease_action(self, robot_id, principal, action, ttl_s=None, note=''):
        from .service import ApiError, MISSION_TERMINAL
        with self.lock, self.db.transaction():
            if robot_id not in self.robots: raise ApiError(404, 'unknown_robot', f'robot {robot_id} has never registered')
            now = time.time(); cur = self.lease_view(robot_id, now); name = principal.name
            if ttl_s is not None and type(ttl_s) not in (int, float): raise ApiError(400, 'bad_ttl', 'ttl_s must be a number')
            try: ttl = LEASE_DEFAULT_TTL if ttl_s is None else float(ttl_s)
            except (TypeError, ValueError, OverflowError): raise ApiError(400, 'bad_ttl', 'ttl_s must be a number')
            if not (LEASE_MIN_TTL <= ttl <= LEASE_MAX_TTL): raise ApiError(400, 'bad_ttl', f'ttl_s must be {LEASE_MIN_TTL:g}..{LEASE_MAX_TTL:g}')
            note = str(note or '')[:200]
            if action == 'acquire':
                if cur and cur['owner'] != name:
                    raise ApiError(409, 'lock_held', f"robot {robot_id} is controlled by {cur['owner']!r} (lease {cur['remaining_s']} s left)")
                if cur:   # same owner: renew, keep the epoch
                    self.db.x('UPDATE locks SET expires_at=?, ttl_s=? WHERE robot_id=?', (now + ttl, ttl, robot_id)); return self.lease_view(robot_id)
                others = [m for m in self.db.q('SELECT mission_id,created_by,status FROM missions WHERE robot_id=?', (robot_id,))
                          if m['status'] not in MISSION_TERMINAL and (m['created_by'] != name or m['status'] == 'RECONCILIATION_REQUIRED')]
                if others:
                    code = 'reconciliation_required' if any(m['status'] == 'RECONCILIATION_REQUIRED' for m in others) else 'robot_busy'
                    raise ApiError(409, code, f"cannot take control: mission {others[0]['mission_id']} ({others[0]['status']}, by {others[0]['created_by']!r}) is not finished")
                if self.robots[robot_id].get('reconciliation_required'):
                    raise ApiError(409, 'reconciliation_required', 'robot requires reconciliation; cannot take control')
                # A new lease generation can never inherit a prior bench session.
                self.manual_benches.pop(robot_id, None)
                epoch = self.db.next_seq('lock_epoch')
                self.db.x('INSERT INTO locks(robot_id,owner,epoch,acquired_at,expires_at,ttl_s,note) VALUES(?,?,?,?,?,?,?) ON CONFLICT(robot_id) DO UPDATE SET owner=excluded.owner, '
                          'epoch=excluded.epoch, acquired_at=excluded.acquired_at, expires_at=excluded.expires_at, ttl_s=excluded.ttl_s, note=excluded.note', (robot_id, name, epoch, now, now + ttl, ttl, note))
                self.db.event('lease_acquired', f'{name} took control of {robot_id} (epoch {epoch}, ttl {ttl:g}s)', robot_id)
                return self.lease_view(robot_id)
            if action == 'renew':
                if not cur or cur['owner'] != name: raise ApiError(409, 'lock_not_held', 'no live lease owned by this principal; acquire again')
                self.db.x('UPDATE locks SET expires_at=?, ttl_s=? WHERE robot_id=?', (now + ttl, ttl, robot_id)); return self.lease_view(robot_id)
            if action in ('release', 'force_release'):
                if action == 'force_release' and principal.role != 'admin': raise ApiError(403, 'forbidden', 'only an admin may force-release a lease')
                if not cur:
                    self.manual_benches.pop(robot_id, None)
                    return {'released': False, 'lease': None}
                if action == 'release' and cur['owner'] != name and principal.role != 'admin': raise ApiError(409, 'lock_held', f"lease is owned by {cur['owner']!r}")
                self.manual_benches.pop(robot_id, None)
                self.db.x('DELETE FROM locks WHERE robot_id=?', (robot_id,))
                self.db.event('lease_released', f"{name} released control of {robot_id} (was {cur['owner']!r}{', forced' if action == 'force_release' else ''})", robot_id, level='warning' if action == 'force_release' else 'info')
                return {'released': True, 'lease': None}
            raise ApiError(400, 'bad_action', "action must be acquire, renew, release or force_release")

    def check_creator_allowed(self, robot_id, created_by):
        from .service import ApiError
        cur = self.lease_view(robot_id)
        if cur and cur['owner'] != created_by:
            raise ApiError(409, 'ownership_conflict', f"robot {robot_id} is under control of {cur['owner']!r}; {created_by!r} may not create missions now")

    # ------------------------------------------------------------ operator recovery
    def recovery_evidence(self, robot_id):
        """(ok, reason, evidence). Evidence is what the agent MEASURED; nothing is assumed."""
        r = self.robots.get(robot_id)
        if not r: return False, 'unknown_robot', {}
        if not r.get('online') or not r.get('reconciled'): return False, 'robot_not_ready', {'online': bool(r.get('online')), 'reconciled': bool(r.get('reconciled'))}
        age = None if not r.get('state_ts') else time.time() - r['state_ts']
        if age is None or not math.isfinite(age) or not 0 <= age <= min(2.0, self.cfg.offline_after_s): return False, 'stale_state', {'state_age_s': age}
        ex = (r.get('state') or {}).get('execution') or {}
        if not isinstance(ex, dict) or 'active' not in ex: return False, 'no_evidence', {}
        g = ex.get('nav2_goals')
        # Narrow recovery of a stale in-memory latch after same-session,
        # journaled terminal results. Explicit operator request only; no real
        # Gate attestation exists, so this path is SIM_ONLY and defaults OFF.
        if self._sim_terminal_recovery_ready(robot_id, ex):
            return True, 'sim_owned_terminals_confirmed', {
                'SIM_ONLY': True, 'terminals': ex['journal_terminals'], 'observed_at': r['state_ts']}
        locked = self._locked_mock_recovery(robot_id, r, ex, age)
        if locked is not None:
            return locked
        ev = {'state_age_s': round(age, 1), 'agent_active': ex.get('active'), 'nav2_goals': g, 'observed_at': r['state_ts']}
        if not isinstance(g, dict) or g.get('known') is not True or g.get('received_status_array') is not True or type(g.get('active_count')) is not int or g['active_count'] < 0: return False, 'no_evidence', ev
        goal_age = g.get('age_s')
        if isinstance(goal_age, bool) or not isinstance(goal_age, (int, float)) or not math.isfinite(goal_age) or goal_age < 0 or goal_age + age > 2.0: return False, 'no_evidence', ev
        if ex.get('active'): return False, 'goal_still_active', ev
        if g['active_count'] > 0: return False, 'goal_still_active', ev
        # An empty status sample does not prove the fate or ownership of a
        # previously delivered goal. There is currently no authenticated
        # terminal-evidence producer, so operator recovery must remain blocked.
        return False, 'terminal_evidence_unavailable', ev

    def _locked_mock_recovery(self, robot_id, r, ex, age):
        """Recovery evidence for the isolated mock graph with no motor (locked Arbiter, Gate motor path LOCKED).

        None = not such an agent (real / SIM paths keep their own rules; real stays refused). Otherwise (ok, why, ev).
        Every delivered, unresolved navigate command needs a terminal from Nav2 itself: the action result journaled by
        the Agent, or the goal's terminal state in the NavigateToPose status array. In addition, NOW: no Agent-owned
        goal, NavigateToPose/FollowPath idle by publisher continuity, Watchdog authorized, Arbiter output zero, Gate
        NORMAL, Mapping Worker without processes. Missing evidence refuses."""
        agent = r.get('agent') or {}
        state = r.get('state') or {}
        gate = state.get('safety_gate') or {}
        # 2026-11-27: a real_nav_dry_run Agent (Watchdog selector, no Arbiter) uses the same rule through `locked_motor_proof` = software evidence of a locked motion path, NOT a physical
        # motor lock; every other check below (terminal evidence per delivered command, idle, Gate NORMAL, Watchdog authorized, zero output) is unchanged.
        dry = agent.get('real_nav_dry_run') is True and agent.get('velocity_selector') == 'WATCHDOG_SELECTOR'
        if not (dry or (agent.get('mode') == 'mock' and agent.get('simulated') is True and agent.get('locked_velocity_arbiter') is True)):
            return None
        ev = {'LOCKED_MOCK': not dry, 'LOCKED_MOTOR_DRY_RUN': dry, 'state_age_s': round(age, 2), 'observed_at': r['state_ts'], 'agent_boot': agent.get('boot_id')}
        checks = {
            'motor_path_locked': isinstance(gate.get('motor_path'), dict) and gate['motor_path'].get('status') == 'LOCKED',
            'gate_normal': gate.get('status') == 'NORMAL' and gate.get('cmd_blocked') is False,
            'no_agent_goal': ex.get('active') is None,
            'nav_idle': (ex.get('nav_idle') or {}).get('ok') is True and (ex.get('nav2_goals') or {}).get('active_count') == 0
                        and not (ex.get('nav2_goals') or {}).get('foreign_active_goal_uuids'),
            'watchdog_authorized': (state.get('motion_watchdog') or {}).get('state') == 'MOTION_AUTHORIZED'
                                   and (state.get('motion_watchdog') or {}).get('agent_boot_matches') is True,
            'arbiter_zero': (state.get('velocity_arbiter') or {}).get('selected_linear') in (0, 0.0)
                            and (state.get('velocity_arbiter') or {}).get('selected_turn') in (0, 0.0),
            'mapping_worker_stopped': not (state.get('mapping_worker') or {}).get('processes_started'),
        }
        ev['checks'] = checks
        if not all(checks.values()):
            return False, 'idle_not_proven', ev
        journal = {j.get('command_id'): j for j in (ex.get('journal_terminals') or []) if isinstance(j, dict)}
        observed = ex.get('nav_terminal_observed') if isinstance(ex.get('nav_terminal_observed'), dict) else {}
        terminals = {}
        for c in self.db.q("SELECT * FROM commands WHERE robot_id=? AND type='navigate_to_pose' AND status NOT IN "
                           "('SUCCEEDED','FAILED','CANCELED','REJECTED','EXPIRED')", (robot_id,)):
            if not c['attempts'] and c['status'] == 'PENDING':
                terminals[c['command_id']] = {'status': 'REJECTED', 'source': 'never_delivered'}
                continue
            uid = self.db.one('SELECT v FROM meta WHERE k=?', ('observed_goal_uuid:' + c['command_id'],))
            j = journal.get(c['command_id'])
            if j and j.get('terminal_source') == 'nav2_action_result' and j.get('terminal_status') in ('SUCCEEDED', 'CANCELED', 'FAILED') \
                    and (not uid or j.get('goal_uuid') == uid['v']):
                terminals[c['command_id']] = {'status': j['terminal_status'], 'source': 'nav2_action_result'}
            elif uid and observed.get(uid['v']) in ('SUCCEEDED', 'CANCELED', 'FAILED'):
                terminals[c['command_id']] = {'status': observed[uid['v']], 'source': 'nav2_status_array'}
            else:
                ev['missing_terminal'] = c['command_id']
                return False, 'terminal_evidence_missing', ev
        ev['terminals'] = terminals
        return True, 'locked_mock_terminals_confirmed', ev

    def _apply_recovered_terminals(self, robot_id, ev, principal):
        """Write the evidenced terminal states (never a guess) to the commands and their missions."""
        for cid, t in (ev.get('terminals') or {}).items():
            c = self.db.one('SELECT * FROM commands WHERE command_id=?', (cid,))
            if not c:
                continue
            self.db.x('UPDATE commands SET status=?, detail=? WHERE command_id=?',
                      (t['status'], f"operator recovery: {t['source']}", cid))
            if c['mission_id']:
                m = self.db.one('SELECT status FROM missions WHERE mission_id=?', (c['mission_id'],))
                if m and m['status'] not in ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED', 'EXPIRED'):
                    self._set_mission(c['mission_id'], t['status'],
                                      f"RECOVERED by {principal.name}: Nav2 reported {t['status']} ({t['source']})")
        for m in self.db.q("SELECT mission_id FROM missions WHERE robot_id=? AND status='RECONCILIATION_REQUIRED'", (robot_id,)):
            open_cmds = self.db.one("SELECT 1 FROM commands WHERE mission_id=? AND type='navigate_to_pose' AND status NOT IN "
                                    "('SUCCEEDED','FAILED','CANCELED','REJECTED','EXPIRED')", (m['mission_id'],))
            if not open_cmds:
                last = self.db.one("SELECT status FROM commands WHERE mission_id=? AND type='navigate_to_pose' ORDER BY seq DESC LIMIT 1", (m['mission_id'],))
                self._set_mission(m['mission_id'], last['status'] if last else 'FAILED',
                                  f"RECOVERED by {principal.name}: all goals have Nav2 terminal evidence")

    def recover_mission(self, mission_id, principal, note):
        from .service import ApiError, MISSION_TERMINAL
        note = str(note or '').strip()
        if len(note) < 10: raise ApiError(400, 'note_required', 'note (>= 10 characters) must say what the operator verified')
        with self.lock:
            m = self.db.one('SELECT * FROM missions WHERE mission_id=?', (mission_id,))
            if not m: raise ApiError(404, 'unknown_mission', 'no such mission')
            if m['status'] != 'RECONCILIATION_REQUIRED': raise ApiError(409, 'not_recoverable', f"mission is {m['status']}, not RECONCILIATION_REQUIRED")
            ok, why, ev = self.recovery_evidence(m['robot_id'])
            if not ok:
                with self.db.transaction():
                    self.db.event('recovery_refused', f"recovery of {mission_id} refused: {why}", m['robot_id'], mission_id, level='warning', data=ev)
                raise ApiError(409, why, f'no proof that the Nav2 goal has ended ({why}); no new movement mission is allowed until there is')
            with self.state_transaction():
                self.invalidate_movement(m['robot_id'], 'recovery invalidates earlier movement')
                rec = {'by': principal.name, 'at': time.time(), 'note': note[:300], 'evidence': ev}
                self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', (f'recovery:{mission_id}', json.dumps(rec)))
                if ev.get('LOCKED_MOCK'):
                    self._apply_recovered_terminals(m['robot_id'], ev, principal)
                    self._refresh_block(m['robot_id'])
                    self.db.event('mission_recovered', f"{principal.name} recovered {mission_id} with Nav2 terminal evidence: {note[:100]}",
                                  m['robot_id'], mission_id, level='warning', data=rec)
                    return dict(self.mission(mission_id), recovery=rec)
                for c in self.db.q("SELECT * FROM commands WHERE mission_id=? AND status NOT IN ('SUCCEEDED','FAILED','CANCELED','REJECTED','EXPIRED')", (mission_id,)):
                    self.db.x("UPDATE commands SET status='FAILED', detail='operator recovery: outcome unverified' WHERE command_id=?", (c['command_id'],))
                self._set_mission(mission_id, 'FAILED', f"RECOVERED_UNVERIFIED_OUTCOME: result of the original goal is unknown; {principal.name} verified Nav2 has no active goal")
                self._refresh_block(m['robot_id'])
                self.db.event('mission_recovered', f"{principal.name} recovered {mission_id} (outcome unknown, FAILED): {note[:100]}", m['robot_id'], mission_id, level='warning', data=rec)
                return dict(self.mission(mission_id), recovery=rec)

    def recovered_command_ids(self, robot_id):
        return [c['command_id'] for c in self.db.q("SELECT command_id FROM commands WHERE robot_id=? AND detail LIKE 'operator recovery:%' ORDER BY seq DESC LIMIT 20", (robot_id,))]

    def recover_robot_block(self, robot_id, principal, note):
        """Clears the robot-level 'unknown active mission ownership' block under the same evidence rule."""
        from .service import ApiError
        note = str(note or '').strip()
        if len(note) < 10: raise ApiError(400, 'note_required', 'note (>= 10 characters) must say what the operator verified')
        with self.lock:
            if robot_id not in self.robots: raise ApiError(404, 'unknown_robot', 'no such robot')
            gate_flag = self.db.one('SELECT 1 FROM meta WHERE k=?', ('gate_boot_reconciliation:' + robot_id,)) is not None
            has_block = bool(self.robots[robot_id].get('reconciliation_required') or self.db.one('SELECT 1 FROM meta WHERE k=?', (f'reconciliation_block:{robot_id}',)))
            if not has_block and not gate_flag: raise ApiError(409, 'not_recoverable', 'robot has no reconciliation block')
            if not has_block:
                return self._reconcile_gate_boot(robot_id, principal, note)
            ok, why, ev = self.recovery_evidence(robot_id)
            if not ok:
                with self.db.transaction():
                    self.db.event('recovery_refused', f'recovery of robot block refused: {why}', robot_id, level='warning', data=ev)
                raise ApiError(409, why, f'no proof that the Nav2 goal has ended ({why})')
            with self.state_transaction():
                self.invalidate_movement(robot_id, 'recovery invalidates earlier movement')
                if ev.get('LOCKED_MOCK'):
                    self._apply_recovered_terminals(robot_id, ev, principal)
                self.db.x('DELETE FROM meta WHERE k=?', (f'reconciliation_block:{robot_id}',)); self._refresh_block(robot_id)
                # Adopt only the explicitly reviewed current generation after
                # invalidating old queued work. Otherwise the next state packet
                # compares against the pre-fault context and kills a NEW mission.
                if ev.get('SIM_ONLY') is True:
                    context = self.safety_context(robot_id)
                    if context is None:
                        raise ApiError(409, 'safety_changed_during_recovery', 'current safety context is not ready')
                    self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)',
                              ('safety_context:'+robot_id, json.dumps(context)))
                self.db.event('robot_block_recovered', f'{principal.name} cleared the reconciliation block: {note[:100]}', robot_id, level='warning', data={'evidence': ev})
            if gate_flag:                                  # a Gate boot change is reconciled separately, with its own evidence
                return self._reconcile_gate_boot(robot_id, principal, note)
            return self.robot_view(self.robots[robot_id])

    def gate_boot_evidence(self, robot_id):
        """(ok, reason, evidence) for adopting the CURRENT Gate boot after a boot change (2026-11-21). Measured values only."""
        r = self.robots.get(robot_id) or {}
        st = r.get('state') or {}; gate = st.get('safety_gate') or {}; ex = st.get('execution') or {}
        goals = ex.get('nav2_goals') if isinstance(ex.get('nav2_goals'), dict) else {}
        wd = st.get('motion_watchdog') or {}
        age = None if not r.get('state_ts') else time.time() - r['state_ts']
        row = self.db.one('SELECT v FROM meta WHERE k=?', ('observed_gate_boot:' + robot_id,))
        ev = {'previous_gate_boot': row['v'] if row else None, 'current_gate_boot': gate.get('boot_id'), 'gate_status': gate.get('status'),
              'fault_latched': gate.get('fault_latched'), 'cmd_blocked': gate.get('cmd_blocked'), 'state_age_s': None if age is None else round(age, 2),
              'agent_active': ex.get('active'), 'nav2_active_count': goals.get('active_count'), 'watchdog': wd.get('state')}
        if not r.get('online') or not r.get('reconciled'): return False, 'robot_not_ready', ev
        if age is None or not 0 <= age <= 2.0: return False, 'stale_state', ev
        bid = gate.get('boot_id')
        if not isinstance(bid, str) or not bid.strip() or len(bid) > 256: return False, 'gate_boot_unknown', ev
        if gate.get('status') != 'NORMAL' or gate.get('fault_latched') is not False or gate.get('cmd_blocked') is not False: return False, 'gate_not_normal', ev
        if ex.get('active') is not None or goals.get('active_count') != 0: return False, 'goal_still_active', ev
        if wd.get('required') is True and (wd.get('state') != 'MOTION_AUTHORIZED' or wd.get('agent_boot_matches') is not True): return False, 'watchdog_not_authorized', ev
        return True, None, ev

    def _reconcile_gate_boot(self, robot_id, principal, note):
        from .service import ApiError
        ok, why, ev = self.gate_boot_evidence(robot_id)
        if not ok:
            with self.db.transaction():
                self.db.event('gate_boot_reconcile_refused', f'{principal.name}: {why}', robot_id, level='warning', data=ev)
            raise ApiError(409, why, f'Gate boot change cannot be reconciled yet ({why})')
        with self.state_transaction():
            self.invalidate_movement(robot_id, 'Gate boot reconciled; earlier admissions are not reused')
            self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', ('observed_gate_boot:' + robot_id, ev['current_gate_boot']))
            self.db.x('DELETE FROM meta WHERE k=?', ('gate_boot_reconciliation:' + robot_id,))
            self.db.x('DELETE FROM meta WHERE k=?', ('safety_context:' + robot_id,))
            self.db.event('gate_boot_reconciled', f'{principal.name} adopted Gate boot {str(ev["current_gate_boot"])[:12]}: {note[:100]}', robot_id, level='warning', data=ev)
        return self.robot_view(self.robots[robot_id])

    def _sim_terminal_recovery_ready(self, robot_id, execution):
        from .safety import sim_r7_report_valid
        r = self.robots[robot_id]; state = r.get('state') or {}
        required = getattr(self.cfg, 'sim_nav_evidence_actions', ())
        gate = state.get('safety_gate') or {}; goals = execution.get('nav2_goals') or {}
        if (execution.get('active') is not None or not isinstance(goals, dict)
                or goals.get('active_count') != 0
                or not sim_r7_report_valid(r.get('agent') or {}, gate, goals, required)
                or self.safety_context(robot_id, operator_terminal_review=True) is None):
            return False
        records = execution.get('journal_terminals')
        if not isinstance(records, list) or not records or len(records) > 20: return False
        pending = self.db.one("SELECT 1 FROM commands WHERE robot_id=? AND status NOT IN ('SUCCEEDED','FAILED','CANCELED','REJECTED','EXPIRED')", (robot_id,))
        if pending: return False
        reviewed = set()
        for record in records:
            if not isinstance(record, dict): return False
            cid = record.get('command_id'); uid = record.get('goal_uuid'); context = record.get('safety_context')
            if (not isinstance(cid, str) or cid in reviewed or not isinstance(uid, str) or len(uid) != 32
                    or any(c not in '0123456789abcdef' for c in uid)
                    or not isinstance(context, dict) or context.get('gate_boot_id') != gate.get('boot_id')
                    or context.get('agent_boot_id') != r['agent']['boot_id']
                    or record.get('terminal_agent_boot_id') != r['agent']['boot_id']
                    or record.get('terminal_source') != 'nav2_action_result'
                    or record.get('terminal_status') not in ('SUCCEEDED','CANCELED','FAILED')
                    or record.get('terminal_status') != record.get('status')): return False
            c = self.db.one('SELECT * FROM commands WHERE command_id=? AND robot_id=?', (cid, robot_id))
            if (not c or c['type'] != 'navigate_to_pose' or c['owner_boot_id'] != r['agent']['boot_id']
                    or c['mission_id'] != record.get('mission_id') or c['status'] != record['terminal_status']
                    or json.loads(c['payload_json']).get('safety_context') != context): return False
            observed = self.db.one('SELECT v FROM meta WHERE k=?', ('observed_goal_uuid:'+cid,))
            if not observed or observed['v'] != uid: return False
            reviewed.add(cid)
        # Every delivered goal of the current session must have its result;
        # missing/older journals never grant recovery from an unknown goal.
        delivered = self.db.q("SELECT command_id FROM commands WHERE robot_id=? AND type='navigate_to_pose' AND attempts>0 AND owner_boot_id=?",(robot_id,r['agent']['boot_id']))
        return bool(delivered) and all(c['command_id'] in reviewed for c in delivered)

    def _refresh_block(self, robot_id):
        blocked = bool(self.db.one('SELECT 1 FROM meta WHERE k=?', (f'reconciliation_block:{robot_id}',))) or bool(
            self.db.one("SELECT 1 FROM missions WHERE robot_id=? AND status='RECONCILIATION_REQUIRED'", (robot_id,)))
        self.robots[robot_id]['reconciliation_required'] = blocked
