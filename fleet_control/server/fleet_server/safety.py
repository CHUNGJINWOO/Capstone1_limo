"""Server-side fail-closed admission for reported navigation safety context.

Agent execution-time validation is still required: HTTP delivery is not atomic with ROS execution.
"""
import json
import math
import time

class SafetyMixin:
    def agent_contract_ok(self, robot_id):
        r = self.robots.get(robot_id) or {}
        return agent_contract_valid(r.get('agent'))

    def require_agent_contract(self, robot_id, context='control'):
        from .service import ApiError
        if not self.agent_contract_ok(robot_id):
            raise ApiError(409, 'agent_protocol', f'incompatible agent protocol or safety contract for {context}')

    def _sc_fail(self, robot_id, reason):
        """Record the FIRST failing check of safety_context (diagnostics only; the admission decision is unchanged)."""
        if not hasattr(self, 'safety_context_reasons'): self.safety_context_reasons = {}
        self.safety_context_reasons[robot_id] = {'reason': reason, 't': time.time()}
        return None

    def safety_context(self, robot_id, *, operator_terminal_review=False, active_continuation=False):
        r = self.robots.get(robot_id) or {}
        state = r.get('state') or {}
        gate = state.get('safety_gate')
        gate = gate if isinstance(gate, dict) else {}
        if not r.get('online') or not r.get('reconciled') or (r.get('reconciliation_required') and not operator_terminal_review):
            return self._sc_fail(robot_id, 'AGENT_OFFLINE_OR_UNRECONCILED')
        if self.db.one("SELECT 1 FROM missions WHERE robot_id=? AND status='RECONCILIATION_REQUIRED'", (robot_id,)) or self.db.one('SELECT 1 FROM meta WHERE k=?', ('reconciliation_block:'+robot_id,)):
            return self._sc_fail(robot_id, 'RECONCILIATION_REQUIRED_MISSION_OR_BLOCK')
        if self.db.one('SELECT 1 FROM meta WHERE k=?', ('safety_epoch_invalid:'+robot_id,)):
            return self._sc_fail(robot_id, 'SAFETY_EPOCH_INVALID')
        if self.db.one('SELECT 1 FROM meta WHERE k=?', ('gate_boot_reconciliation:'+robot_id,)):
            return self._sc_fail(robot_id, 'GATE_BOOT_RECONCILIATION')
        agent = r.get('agent') or {}
        if not agent_contract_valid(agent):
            return self._sc_fail(robot_id, 'AGENT_PROTOCOL')
        ttl = self.cfg.command_ttl_s
        if not finite_number(ttl) or ttl <= 0: return self._sc_fail(robot_id, 'COMMAND_TTL')
        ts = r.get('state_ts')
        if not finite_number(ts) or not 0 <= time.time()-ts <= min(2.0, self.cfg.offline_after_s):
            return self._sc_fail(robot_id, 'STATE_STALE')
        age = gate.get('age_s')
        if not finite_number(age) or age < 0 or age + (time.time()-ts) > 2.0:
            return self._sc_fail(robot_id, 'GATE_REPORT_STALE')
        pending_continuation = (active_continuation and gate.get('status') == 'STALL_PENDING'
                                and gate.get('safety_state') == 'NORMAL'
                                and gate.get('movement_block_reasons') == ['GATE_NOT_NORMAL:STALL_PENDING'])
        if ((gate.get('status') != 'NORMAL' or gate.get('movement_permitted') is not True)
                and not pending_continuation): return self._sc_fail(robot_id, 'GATE_NOT_NORMAL_OR_MOVEMENT_NOT_PERMITTED')
        if gate.get('fault_latched') is not False or gate.get('cmd_blocked') is not False: return self._sc_fail(robot_id, 'GATE_LATCHED_OR_BLOCKED')
        if 'fault_epoch_pending' in gate and (type(gate['fault_epoch_pending']) is not int or gate['fault_epoch_pending'] != 0):
            return self._sc_fail(robot_id, 'GATE_FAULT_EPOCH_PENDING')
        if 'safety_state' in gate and gate['safety_state'] != 'NORMAL': return self._sc_fail(robot_id, 'GATE_SAFETY_STATE')
        if gate.get('r7_invalid') is True: return self._sc_fail(robot_id, 'GATE_R7_INVALID')
        epoch = gate.get('fault_epoch')
        gate_boot = gate.get('boot_id')
        revision = state.get('operation_revision')
        if not valid_id(gate_boot) or not valid_epoch(revision):
            return self._sc_fail(robot_id, 'GATE_BOOT_OR_REVISION_INVALID')
        if not valid_epoch(epoch) or state.get('operation_mode') != 'NAVIGATION':
            return self._sc_fail(robot_id, 'FAULT_EPOCH_OR_MODE_NOT_NAVIGATION')
        me, le = state.get('map_epoch'), state.get('localization_epoch')
        mv = state.get('map_version_id')
        if not valid_epoch(me) or not valid_epoch(le) or not valid_id(mv) or state.get('localization_map_version_id') != mv or state.get('map_verified') is not True or state.get('localization_status') != 'CONFIRMED':
            return self._sc_fail(robot_id, 'MAP_OR_LOCALIZATION_NOT_CONFIRMED')
        execution = state.get('execution')
        if not isinstance(execution,dict) or 'active' not in execution or not isinstance(execution.get('nav2_goals'),dict):
            return self._sc_fail(robot_id, 'EXECUTION_REPORT_INVALID')
        goals=execution['nav2_goals']
        goal_age = goals.get('age_s')
        _act = execution.get('active')
        sim_proof = sim_r7_report_valid(agent, gate, goals, getattr(self.cfg, 'sim_nav_evidence_actions', ()),
                                        _act.get('goal_uuid') if isinstance(_act, dict) else None)
        if goals.get('source') == 'SIM_ONLY_GATE_R7' and not sim_proof: return self._sc_fail(robot_id, 'SIM_R7_PROOF')
        # Isolated mock graph with no motor at all (Agent locked arbiter, Gate output /cmd_vel_locked, motor path
        # LOCKED): the Gate's STALL_PENDING confirmation window may be continued with the same goal-ownership checks
        # below; the SIM-only Gate r7 attestation does not exist there. Real / unlocked motor paths are unaffected.
        locked_mock_proof = (agent.get('mode') == 'mock' and agent.get('simulated') is True
                             and agent.get('locked_velocity_arbiter') is True
                             and isinstance(gate.get('motor_path'), dict) and gate['motor_path'].get('status') == 'LOCKED')
        if pending_continuation:
            active = execution.get('active')
            if not (sim_proof or locked_mock_proof) or not isinstance(active, dict) or active.get('goal_handle_available') is not True: return self._sc_fail(robot_id, 'CONTINUATION_PROOF')
            cmd = self.db.one('SELECT * FROM commands WHERE robot_id=? AND command_id=? AND mission_id=?',
                              (robot_id, active.get('command_id'), active.get('mission_id')))
            observed = self.db.one('SELECT v FROM meta WHERE k=?', ('observed_goal_uuid:' + str(active.get('command_id')),))
            if (not cmd or cmd['type'] != 'navigate_to_pose' or cmd['status'] not in ('ACCEPTED', 'EXECUTING')
                    or cmd['owner_boot_id'] != agent['boot_id'] or not observed
                    or observed['v'] != active.get('goal_uuid')
                    or not (goals.get('active_goal_uuids') == [observed['v']]
                            or (goals.get('subordinate_active_goal_uuids')       # validated against Gate evidence by sim_proof
                                and set(goals.get('active_goal_uuids') or []) == {observed['v'], *goals['subordinate_active_goal_uuids']}))): return self._sc_fail(robot_id, 'CONTINUATION_GOAL_OWNERSHIP')
        # Action status is published on change only: an unchanged status is proven by publisher-GID continuity
        # reported by the Agent (`status_continuous`), otherwise a recent message is required.
        continuous = goals.get('status_continuous') is True
        if goals.get('known') is not True or (goals.get('received_status_array') is not True and not sim_proof and not continuous) or type(goals.get('active_count')) is not int or goals['active_count'] < 0:
            return self._sc_fail(robot_id, 'NAV2_STATUS_UNKNOWN')
        if not continuous and (not finite_number(goal_age) or goal_age < 0 or goal_age + (time.time()-ts) > 2.0):
            return self._sc_fail(robot_id, 'NAV2_STATUS_STALE')
        context = {'boot':r['agent']['boot_id'],'gate_boot_id':gate_boot,'fault_epoch':epoch,'operation_revision':revision,'map_epoch':me,'localization_epoch':le,'map_version_id':mv}
        # Independent motion watchdog (reported only when the Agent runs with require_motion_watchdog): it must be
        # authorized and fresh, and its trip epoch is part of the admission context, so any watchdog trip or restart
        # invalidates every mission admitted before it (no automatic re-departure after a motion fault).
        wd = state.get('motion_watchdog')
        if isinstance(wd, dict) and wd.get('required') is True:
            age = wd.get('age_s')
            if (wd.get('state') != 'MOTION_AUTHORIZED' or wd.get('agent_boot_matches') is not True or not valid_id(wd.get('epoch'))
                    or not finite_number(age) or age < 0 or age + (time.time()-ts) > 2.0):
                return self._sc_fail(robot_id, 'WATCHDOG_NOT_AUTHORIZED')
            context['motion_watchdog_epoch'] = wd['epoch']
        if not hasattr(self, 'safety_context_reasons'): self.safety_context_reasons = {}
        self.safety_context_reasons[robot_id] = {'reason': None, 't': time.time()}
        return context

    def invalidate_movement(self, robot_id, reason):
        """Caller holds Fleet.lock then DB.transaction. Never infer delivered outcome."""
        rows=self.db.q("SELECT * FROM commands WHERE robot_id=? AND type='navigate_to_pose' AND status NOT IN ('SUCCEEDED','FAILED','CANCELED','REJECTED','EXPIRED')",(robot_id,))
        for c in rows:
            if c['status']=='PENDING' and c['attempts']==0:
                self.db.x("UPDATE commands SET status='REJECTED', detail=? WHERE command_id=?",(reason,c['command_id']))
                self._set_mission(c['mission_id'],'FAILED',reason)
            else:
                self.require_reconciliation(c, reason)
                self.robots[robot_id]['reconciliation_required']=True
        for m in self.db.q("SELECT mission_id FROM missions WHERE robot_id=? AND status='QUEUED'",(robot_id,)):
            self._set_mission(m['mission_id'],'FAILED',reason)

    def mission_authorized(self, mission, context):
        if not context:
            return False
        raw=self.db.one('SELECT v FROM meta WHERE k=?',('admission:'+mission['mission_id'],))
        if not raw:
            return False
        try: admission=json.loads(raw['v'])
        except (TypeError,ValueError): return False
        if not isinstance(admission, dict): return False
        lease=self.lease_view(mission['robot_id'])
        return bool(lease and lease['owner']==mission['created_by'] and valid_epoch(admission.get('lease_epoch')) and
                    lease['epoch']==admission.get('lease_epoch') and admission.get('context')==context)

    def command_context_valid(self, command, mission, context):
        try:
            payload = json.loads(command['payload_json'])
            c = payload['safety_context']
            lease = self.lease_view(mission['robot_id'])
            if type(c.get('version')) is not int or c['version'] != 1 or not lease: return False
            if any(not valid_epoch(c.get(k)) for k in ('fault_epoch','operation_revision','map_epoch','localization_epoch','lease_epoch')): return False
            if any(not valid_id(c.get(k)) for k in ('server_boot_id','agent_boot_id','gate_boot_id','map_version_id','lease_owner')): return False
            for key in ('issued_at','expires_at'):
                if not finite_number(c.get(key)) or c[key] <= 0: return False
            return (c['server_boot_id'] == self.boot_id and c['agent_boot_id'] == context['boot'] and
                    all(c[k] == context[k] for k in ('gate_boot_id','fault_epoch','operation_revision','map_epoch','localization_epoch','map_version_id')) and
                    c['lease_epoch'] == lease['epoch'] and c['lease_owner'] == lease['owner'] and
                    c['operation_mode'] == 'NAVIGATION' and c['localization_status'] == 'CONFIRMED' and
                    c['issued_at'] == command['created_at'] and c['expires_at'] == command['expires_at'] and c['issued_at'] < c['expires_at'])
        except (ValueError, TypeError, KeyError, AttributeError):
            return False

def valid_id(value):
    return isinstance(value, str) and bool(value.strip()) and len(value) <= 256


def agent_contract_valid(agent):
    """Strict Agent-Server safety contract check (protocol_version==2, safety_contract_version==1, valid boot_id)."""
    if not isinstance(agent, dict):
        return False
    if type(agent.get('protocol_version')) is not int or agent['protocol_version'] != 2 or type(agent.get('safety_contract_version')) is not int or agent['safety_contract_version'] != 1 or not valid_id(agent.get('boot_id')):
        return False
    return True

def valid_epoch(value):
    return type(value) is int and 0 <= value < 2**63

def finite_number(value):
    try: return type(value) in (int, float) and math.isfinite(value)
    except OverflowError: return False


def sim_subordinate_valid(goals, owned_uuid):
    """Server-side re-derivation of the SIM_ONLY FollowPath-child attribution from the Gate's own evidence (never trusts the Agent's list alone).
    No subordinate reported -> True. Otherwise the owned NavigateToPose goal must be the sole active goal of navigate_to_pose, one (or, for the replacement hand-over, two)
    follow_path goals (different UUIDs) must be active, every other observed action idle, and the reported subordinate list must equal it."""
    sub = goals.get('subordinate_active_goal_uuids') or []
    if not sub: return True
    ev = goals.get('gate_nav_evidence')
    if not isinstance(ev, dict) or not isinstance(owned_uuid, str) or len(owned_uuid) != 32 or goals.get('foreign_active_goal_uuids'): return False
    servers, ids = ev.get('servers'), ev.get('active_uuids')
    if not isinstance(servers, dict) or not isinstance(ids, list): return False
    nav, fp = servers.get('navigate_to_pose') or {}, servers.get('follow_path') or {}
    if nav.get('active_uuids') != [owned_uuid] or fp.get('verdict') != 'ACTIVE_GOALS': return False
    if sorted(sub) != sorted(fp.get('active_uuids') or []) or not 1 <= len(sub) <= 2 or owned_uuid in sub or set(ids) != {owned_uuid, *sub}: return False   # 2 = hand-over of the replacement FollowPath goal
    return all(name in ('navigate_to_pose', 'follow_path') or (r.get('verdict') != 'ACTIVE_GOALS' and not r.get('active_uuids')) for name, r in servers.items())


def sim_r7_report_valid(agent, gate, goals, required_actions, owned_uuid=None):
    """Explicit server-local SIM_ONLY opt-in; never usable by real Agent reports."""
    if (not required_actions or agent.get('mode') != 'sim' or agent.get('simulated') is not True
            or goals.get('source') != 'SIM_ONLY_GATE_R7'
            or gate.get('contract_version') != 'gate-status/1.2-sim'
            or gate.get('trust') != 'UNAUTHENTICATED_SIM_ONLY'
            or gate.get('single_publisher_graph') is not True
            or gate.get('contract_met') is not True or gate.get('rx_fresh') is not True
            or gate.get('source_fresh') is not True or gate.get('boot_changed') is not False
            or gate.get('r7_invalid') is not False or gate.get('safety_state') != 'NORMAL'
            or type(gate.get('status_seq')) is not int or not 0 < gate['status_seq'] < 2**64):
        return False
    ev = gate.get('nav_evidence')
    if not isinstance(ev, dict) or ev != goals.get('gate_nav_evidence'):
        return False
    servers, ids = ev.get('servers'), ev.get('active_uuids')
    if (type(ev.get('scope_version')) is not int or ev['scope_version'] != 1 or ev.get('observed') is not True
            or not isinstance(servers, dict) or not set(required_actions).issubset(servers)
            or not isinstance(ids, list) or len(ids) > 64
            or any(not isinstance(u, str) or len(u) != 32 or any(c not in '0123456789abcdef' for c in u) for u in ids)
            or len(ids) != len(set(ids))):
        return False
    verdicts = []
    for name, rec in servers.items():
        if (not valid_id(name) or not isinstance(rec, dict) or not valid_id(rec.get('writer_gid'))
                or len(rec['writer_gid']) != 16 or any(c not in '0123456789abcdef' for c in rec['writer_gid'])
                or rec.get('verdict') not in ('IDLE_PROVEN_EL', 'IDLE_PROVEN_ES', 'ACTIVE_GOALS')):
            return False
        verdicts.append(rec['verdict'])
    idle = all(v != 'ACTIVE_GOALS' for v in verdicts)
    return (type(ev.get('all_idle_proven')) is bool and ev['all_idle_proven'] == idle
            and bool(ids) != idle and type(goals.get('active_count')) is int
            and goals['active_count'] == len(ids) and goals.get('active_goal_uuids') == ids
            and type(goals.get('publishers')) is int and goals['publishers'] == 1 and not goals.get('foreign_active_goal_uuids')
            and sim_subordinate_valid(goals, owned_uuid))
