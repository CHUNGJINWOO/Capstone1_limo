"""Control-ownership lease and operator recovery (additive; with no lease held the old behaviour is unchanged).

Lease: per robot, at most one live lease. While a lease is live only its owner (the authenticated token NAME) may create missions for that robot: any other
creator gets HTTP 409 `ownership_conflict` instead of being queued. Check-and-create and check-and-acquire run under the same server lock, so there is no
window between "ownership checked" and "mission created". A lease expires after `ttl_s` unless renewed (a dead RMF adapter cannot block the robot forever);
expiry never touches a running mission. Cancel stays allowed for every control role (stopping is the safe direction).
Acquire is refused (409) while another principal still has a queued/in-flight mission or while reconciliation is required: ownership can only be taken over
a robot that is not being driven by someone else.

Recovery of RECONCILIATION_REQUIRED: only an admin, only with fresh robot-side evidence that Nav2 has NO active goal (state.execution.nav2_goals measured by
the agent from the navigate_to_pose action status topic) and no goal handle owned by the agent. The unknown mission is NEVER marked SUCCEEDED or CANCELED:
it becomes FAILED with detail RECOVERED_UNVERIFIED_OUTCOME and a recovery record; nothing is sent to the robot."""
import json, time

LEASE_MIN_TTL, LEASE_MAX_TTL, LEASE_DEFAULT_TTL = 5.0, 300.0, 20.0
EVIDENCE_MAX_AGE_S = 5.0


class OwnershipMixin:
    # ------------------------------------------------------------ lease
    def lease_view(self, robot_id, now=None):
        now = now or time.time(); r = self.db.one('SELECT * FROM locks WHERE robot_id=?', (robot_id,))
        if not r or r['expires_at'] < now: return None
        return {'robot_id': robot_id, 'owner': r['owner'], 'epoch': r['epoch'], 'acquired_at': r['acquired_at'], 'expires_at': r['expires_at'], 'ttl_s': r['ttl_s'], 'note': r['note'],
                'remaining_s': round(r['expires_at'] - now, 1)}

    def lease_action(self, robot_id, principal, action, ttl_s=None, note=''):
        from .service import ApiError, MISSION_TERMINAL
        with self.lock:
            if robot_id not in self.robots: raise ApiError(404, 'unknown_robot', f'robot {robot_id} has never registered')
            now = time.time(); cur = self.lease_view(robot_id, now); name = principal.name
            try: ttl = LEASE_DEFAULT_TTL if ttl_s is None else float(ttl_s)
            except (TypeError, ValueError): raise ApiError(400, 'bad_ttl', 'ttl_s must be a number')
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
                if not cur: return {'released': False, 'lease': None}
                if action == 'release' and cur['owner'] != name and principal.role != 'admin': raise ApiError(409, 'lock_held', f"lease is owned by {cur['owner']!r}")
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
        if age is None or age > EVIDENCE_MAX_AGE_S: return False, 'stale_state', {'state_age_s': age}
        ex = (r.get('state') or {}).get('execution') or {}
        g = ex.get('nav2_goals')
        ev = {'state_age_s': round(age, 1), 'agent_active': ex.get('active'), 'nav2_goals': g, 'observed_at': r['state_ts']}
        if not isinstance(g, dict) or g.get('known') is not True or not isinstance(g.get('active_count'), int): return False, 'no_evidence', ev
        if ex.get('active'): return False, 'goal_still_active', ev
        if g['active_count'] > 0: return False, 'goal_still_active', ev
        return True, 'ok', ev

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
                self.db.event('recovery_refused', f"recovery of {mission_id} refused: {why}", m['robot_id'], mission_id, level='warning', data=ev)
                raise ApiError(409, why, f'no proof that the Nav2 goal has ended ({why}); no new movement mission is allowed until there is')
            rec = {'by': principal.name, 'at': time.time(), 'note': note[:300], 'evidence': ev}
            self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', (f'recovery:{mission_id}', json.dumps(rec)))
            for c in self.db.q("SELECT * FROM commands WHERE mission_id=? AND status NOT IN ('SUCCEEDED','FAILED','CANCELED','REJECTED','EXPIRED')", (mission_id,)):
                self.db.x("UPDATE commands SET status='FAILED', detail='operator recovery: outcome unverified' WHERE command_id=?", (c['command_id'],))
            self._set_mission(mission_id, 'FAILED', f"RECOVERED_UNVERIFIED_OUTCOME: result of the original goal is unknown; {principal.name} verified Nav2 has no active goal")
            self._refresh_block(m['robot_id'])
            self.db.event('mission_recovered', f"{principal.name} recovered {mission_id} (outcome unknown, FAILED): {note[:100]}", m['robot_id'], mission_id, level='warning', data=rec)
            self.dispatch_next(m['robot_id'])
            return dict(self.mission(mission_id), recovery=rec)

    def recovered_command_ids(self, robot_id):
        return [c['command_id'] for c in self.db.q("SELECT command_id FROM commands WHERE robot_id=? AND detail='operator recovery: outcome unverified' ORDER BY seq DESC LIMIT 20", (robot_id,))]

    def recover_robot_block(self, robot_id, principal, note):
        """Clears the robot-level 'unknown active mission ownership' block under the same evidence rule."""
        from .service import ApiError
        note = str(note or '').strip()
        if len(note) < 10: raise ApiError(400, 'note_required', 'note (>= 10 characters) must say what the operator verified')
        with self.lock:
            if robot_id not in self.robots: raise ApiError(404, 'unknown_robot', 'no such robot')
            if not self.db.one('SELECT 1 FROM meta WHERE k=?', (f'reconciliation_block:{robot_id}',)): raise ApiError(409, 'not_recoverable', 'robot has no reconciliation block')
            ok, why, ev = self.recovery_evidence(robot_id)
            if not ok: raise ApiError(409, why, f'no proof that the Nav2 goal has ended ({why})')
            self.db.x('DELETE FROM meta WHERE k=?', (f'reconciliation_block:{robot_id}',)); self._refresh_block(robot_id)
            self.db.event('robot_block_recovered', f'{principal.name} cleared the reconciliation block: {note[:100]}', robot_id, level='warning', data={'evidence': ev})
            self.dispatch_next(robot_id)
            return self.robot_view(self.robots[robot_id])

    def _refresh_block(self, robot_id):
        blocked = bool(self.db.one('SELECT 1 FROM meta WHERE k=?', (f'reconciliation_block:{robot_id}',))) or bool(
            self.db.one("SELECT 1 FROM missions WHERE robot_id=? AND status='RECONCILIATION_REQUIRED'", (robot_id,)))
        self.robots[robot_id]['reconciliation_required'] = blocked
