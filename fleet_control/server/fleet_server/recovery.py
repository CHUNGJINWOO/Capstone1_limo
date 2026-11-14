"""Session fencing and conservative reconciliation. Never starts a robot goal."""
import time


class RecoveryMixin:
    def check_session(self, robot_id, boot_id):
        from .service import ApiError
        r = self.robots.get(robot_id)
        expected = (r or {}).get('agent', {}).get('boot_id')
        if not r or not boot_id or boot_id != expected:
            raise ApiError(409, 'agent_session_mismatch', 'hello/reconcile required for this agent session')
        return r

    def require_reconciliation(self, command, reason):
        from .service import CMD_TERMINAL
        if command['status'] in CMD_TERMINAL:
            return
        self.db.x("UPDATE commands SET status='RECONCILIATION_REQUIRED', detail=? WHERE command_id=?", (reason, command['command_id']))
        if command['mission_id']:
            self._set_mission(command['mission_id'], 'RECONCILIATION_REQUIRED', reason)

    def reconcile(self, robot_id, boot_id, journal, execution, journal_ok=True):
        """Only a retained same-session handle or a persisted terminal result resolves uncertainty."""
        from .service import ApiError, CMD_TERMINAL
        with self.lock:
            r = self.check_session(robot_id, boot_id)
            if not isinstance(journal, dict) or not isinstance(execution, dict):
                raise ApiError(400, 'bad_reconciliation', 'journal and execution must be objects')
            active = execution.get('active') or {}
            reports = []
            r['reconciliation_required'] = not journal_ok or bool(self.db.one('SELECT 1 FROM meta WHERE k=?', (f'reconciliation_block:{robot_id}',)))
            for c in self.db.q('SELECT * FROM commands WHERE robot_id=? ORDER BY seq', (robot_id,)):
                if c['status'] in CMD_TERMINAL:
                    # Recover a crash between persisting a terminal receipt and updating the mission.
                    if c['mission_id']:
                        mission = self.mission(c['mission_id'])
                        if not mission['terminal']:
                            if c['type'] == 'navigate_to_pose':
                                self._set_mission(c['mission_id'], c['status'], 'recovered persisted terminal command result')
                            elif c['status'] in ('SUCCEEDED', 'CANCELED'):
                                self._set_mission(c['mission_id'], 'CANCELED', 'recovered confirmed cancellation result')
                    continue
                record = journal.get(c['command_id']) or {}
                known = record.get('status')
                # Terminal receipts persisted before an ACK was lost can be recovered across boots.
                if known in CMD_TERMINAL and record.get('mission_id') == c['mission_id']:
                    self.db.x('UPDATE commands SET owner_boot_id=? WHERE command_id=?', (boot_id, c['command_id']))
                    self.ack_command(robot_id, c['command_id'], known, record.get('detail', 'recovered durable result'), agent_boot_id=boot_id)
                elif c['attempts'] == 0 and c['status'] == 'PENDING' and journal_ok:
                    continue  # Never delivered; normal TTL still applies before its FIRST delivery.
                elif (c['type'] == 'navigate_to_pose' and c['owner_boot_id'] == boot_id
                      and active.get('command_id') == c['command_id'] and active.get('mission_id') == c['mission_id']
                      and active.get('goal_handle_available') is True and known in ('ACCEPTED', 'EXECUTING')):
                    self.db.x("UPDATE commands SET status=?, detail='retained same-session goal handle' WHERE command_id=?", (known, c['command_id']))
                    m = self.mission(c['mission_id'])
                    self._set_mission(c['mission_id'], 'CANCELING' if m['cancel_requested'] else known, 'retained same-session goal handle')
                elif (c['type'] == 'cancel_mission' and c['owner_boot_id'] == boot_id
                      and c['command_id'] in execution.get('pending_cancel_ids', []) and active.get('goal_handle_available') is True
                      and active.get('mission_id') == c['mission_id']):
                    pass  # Wait for the original target's terminal result; do not invent cancellation success.
                else:
                    self.require_reconciliation(c, 'delivery/result/goal-handle ownership uncertain; no automatic goal replay')
                reports.append({'command_id': c['command_id'], 'status': self.db.one('SELECT status FROM commands WHERE command_id=?', (c['command_id'],))['status']})
            if active:
                owned = self.db.one('SELECT * FROM commands WHERE command_id=? AND robot_id=? AND mission_id=?',
                                    (active.get('command_id'), robot_id, active.get('mission_id')))
                if not owned or owned['owner_boot_id'] != boot_id or owned['status'] in CMD_TERMINAL:
                    r['reconciliation_required'] = True
                    if not owned:
                        self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', (f'reconciliation_block:{robot_id}', 'unknown active mission ownership'))
                    if owned and owned['status'] not in CMD_TERMINAL:
                        self.require_reconciliation(owned, 'agent/server active mission ownership mismatch')
                    self.db.event('ownership_mismatch', 'agent/server active mission ownership mismatch', robot_id, level='warning')
            r['reconciled'] = True
            self._persist(r, force=True)
            return {'ok': True, 'server_boot_id': self.boot_id, 'commands': reports,
                    'reconciliation_required': r['reconciliation_required'] or any(c['status'] == 'RECONCILIATION_REQUIRED' for c in reports)}

    def check_execution(self, robot_id, execution):
        active = (execution or {}).get('active') or {}
        if active:
            owned = self.db.one('SELECT * FROM commands WHERE command_id=? AND robot_id=? AND mission_id=?',
                                (active.get('command_id'), robot_id, active.get('mission_id')))
            if not owned:
                self.robots[robot_id]['reconciliation_required'] = True
                self.db.x('INSERT OR REPLACE INTO meta(k,v) VALUES(?,?)', (f'reconciliation_block:{robot_id}', 'unknown active mission ownership'))
                self.db.event('ownership_mismatch', 'state reports an unowned active mission; manual reconciliation required', robot_id, level='warning')
        for c in self.db.q("SELECT * FROM commands WHERE robot_id=? AND type='navigate_to_pose' AND status IN ('ACCEPTED','EXECUTING')", (robot_id,)):
            if (active.get('command_id') != c['command_id'] or active.get('mission_id') != c['mission_id']
                    or active.get('goal_handle_available') is not True):
                self.require_reconciliation(c, 'running mission is missing its owned Nav2 goal handle')
