"""Pure-Python Safety Context v1 parser, gate observation tracker, and execution fencing.

No ROS imports: shared by AgentCore, RosRobot, and unit tests. All checks fail closed.
Does NOT fabricate unimplemented Gate/Operation/Map/Localization producers.
"""
import math
import copy
from pathlib import Path
import time

SAFETY_CONTRACT_VERSION = 1
SERVER_PROTOCOL_VERSION = 2      # the Fleet Server hello protocol this Agent is built for (exact int, no bool/float/str)
MAX_EPOCH = (1 << 63) - 1
MAX_ID_LEN = 256
MAX_TIMESTAMP_S = 1e12

REQUIRED_SAFETY_CONTEXT_FIELDS = (
    'version',
    'server_boot_id',
    'agent_boot_id',
    'gate_boot_id',
    'fault_epoch',
    'operation_mode',
    'operation_revision',
    'map_version_id',
    'map_epoch',
    'localization_status',
    'localization_epoch',
    'lease_owner',
    'lease_epoch',
    'issued_at',
    'expires_at',
)

GATE_FAULT_STATES = frozenset({'STALL_LATCHED', 'FAILSAFE_SENSOR_TIMEOUT', 'FAILSAFE_ICP_DEGRADED'})


def _is_nonneg_int(val):
    return type(val) is int and 0 <= val <= MAX_EPOCH


def _is_finite_num(val):
    if type(val) not in (int, float):
        return False
    try:
        return math.isfinite(float(val))
    except (OverflowError, ValueError):
        return False


def _is_valid_ts(val):
    if not _is_finite_num(val):
        return False
    f = float(val)
    return 0.0 < f <= MAX_TIMESTAMP_S


def _is_nonempty_str(val):
    return isinstance(val, str) and len(val.strip()) > 0 and len(val) <= MAX_ID_LEN


is_valid_epoch = _is_nonneg_int
is_valid_id = _is_nonempty_str
is_valid_timestamp = _is_valid_ts


def parse_safety_context(raw):
    """Parse and strictly validate a v1 safety_context dictionary.

    Returns (validated_dict, None) on success, or (None, reason_str) on failure.
    """
    if not isinstance(raw, dict):
        return None, 'SAFETY_CONTEXT_MISSING: safety_context must be a JSON object'
    missing = [k for k in REQUIRED_SAFETY_CONTEXT_FIELDS if k not in raw]
    if missing:
        return None, f"SAFETY_CONTEXT_INVALID: missing field(s): {','.join(missing)}"
    if type(raw['version']) is not int or raw['version'] != SAFETY_CONTRACT_VERSION:
        return None, f"SAFETY_CONTEXT_INVALID: unsupported version {raw['version']!r}"
    for key in ('server_boot_id', 'agent_boot_id', 'gate_boot_id', 'operation_mode', 'map_version_id', 'localization_status', 'lease_owner'):
        if not _is_nonempty_str(raw[key]):
            return None, f'SAFETY_CONTEXT_INVALID: {key} must be a non-empty string (<= {MAX_ID_LEN} chars)'
    for key in ('fault_epoch', 'operation_revision', 'map_epoch', 'localization_epoch', 'lease_epoch'):
        if not _is_nonneg_int(raw[key]):
            return None, f'SAFETY_CONTEXT_INVALID: {key} must be an integer in [0, 2^63 - 1]'
    for key in ('issued_at', 'expires_at'):
        if not _is_valid_ts(raw[key]):
            return None, f'SAFETY_CONTEXT_INVALID: {key} must be a finite valid timestamp'
    if float(raw['expires_at']) <= float(raw['issued_at']):
        return None, 'SAFETY_CONTEXT_INVALID: expires_at must be greater than issued_at'
    return dict(raw), None


class GateObservationTracker:
    """Tracks Safety Gate topic reception freshness separately from source clock freshness."""

    def __init__(self):
        self.rx_mono = None
        self.last_msg = None
        self.parse_ok = False
        self.last_sim_time = None
        self.sim_time_changed_mono = None
        self.source_clock_regressed = False
        self.max_fault_epoch = None
        self.epoch_regressed = False
        self.last_boot_id = None
        self.boot_changed = False
        self.max_status_seq = None
        self.r7_invalid = False
        try:
            self.host_boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip().replace('-', '')
        except OSError:
            self.host_boot_id = None

    def on_message(self, msg_dict, now_mono=None):
        """Record a received /stall_detector/status_json message without forging missing fields."""
        now_m = time.monotonic() if now_mono is None else float(now_mono)
        self.rx_mono = now_m
        if not isinstance(msg_dict, dict):
            self.last_msg = None
            self.parse_ok = False
            return
        # Once an r7 publisher has been observed, downgrade/replay cannot
        # restore permission. This latch deliberately requires reconciliation.
        if msg_dict.get('contract_version') == 'gate-status/1.2-sim' or self.max_status_seq is not None:
            seq = msg_dict.get('status_seq')
            mono = msg_dict.get('published_mono_ns')
            if (msg_dict.get('contract_version') != 'gate-status/1.2-sim'
                    or type(seq) is not int or not 0 < seq < 2**64
                    or (self.max_status_seq is not None and seq <= self.max_status_seq)
                    or type(mono) is not int or not 0 < mono < 2**64
                    or msg_dict.get('boot_id') != msg_dict.get('gate_boot_id')
                    or not self.host_boot_id or msg_dict.get('host_boot_id') != self.host_boot_id):
                self.r7_invalid = True
            elif self.last_boot_id in (None, msg_dict.get('boot_id')):
                self.max_status_seq = seq
        self.last_msg = copy.deepcopy(msg_dict)
        self.parse_ok = True
        sim_t = msg_dict.get('sim_time')
        if _is_finite_num(sim_t):
            sim_f = float(sim_t)
            if self.last_sim_time is None:
                self.last_sim_time = sim_f
                self.sim_time_changed_mono = now_m
            elif sim_f < self.last_sim_time - 0.1:
                self.source_clock_regressed = True
                self.last_sim_time = sim_f
                self.sim_time_changed_mono = now_m
            elif sim_f > self.last_sim_time + 1e-6:
                self.last_sim_time = sim_f
                self.sim_time_changed_mono = now_m
        boot = msg_dict.get('boot_id')
        if _is_nonempty_str(boot):
            if self.last_boot_id is not None and boot != self.last_boot_id:
                self.boot_changed = True
            self.last_boot_id = boot
        fe = msg_dict.get('fault_epoch')
        if _is_nonneg_int(fe):
            if self.max_fault_epoch is not None and fe < self.max_fault_epoch:
                self.epoch_regressed = True
            else:
                self.max_fault_epoch = fe

    def evaluate(self, now_mono=None, max_rx_age_s=2.0, max_source_stall_s=1.0, expect_source_clock=False):
        """Evaluate Gate RX freshness, source observation freshness, and contract readiness."""
        now_m = time.monotonic() if now_mono is None else float(now_mono)
        rx_age = None if self.rx_mono is None else max(0.0, now_m - self.rx_mono)
        rx_fresh = rx_age is not None and rx_age <= max_rx_age_s and self.parse_ok and isinstance(self.last_msg, dict)

        src_age = None
        if self.sim_time_changed_mono is not None:
            src_age = max(0.0, now_m - self.sim_time_changed_mono)
        if not rx_fresh or self.source_clock_regressed:
            source_fresh = False
        elif src_age is not None:
            source_fresh = src_age <= max_source_stall_s
        elif expect_source_clock:
            source_fresh = False
        else:
            source_fresh = True

        g = self.last_msg if (self.parse_ok and isinstance(self.last_msg, dict)) else {}
        if g.get('contract_version') == 'gate-status/1.2-sim':
            mono = g.get('published_mono_ns')
            source_fresh = (source_fresh and not self.r7_invalid and type(mono) is int
                            and 0 <= now_m - mono / 1e9 <= max_rx_age_s)
        raw_state = g.get('state')
        if self.rx_mono is None or not self.parse_ok:
            status = 'NO_DATA'
        elif not rx_fresh or not source_fresh:
            status = 'STALE'
        elif _is_nonempty_str(raw_state):
            status = str(raw_state)
        else:
            status = 'UNKNOWN'

        boot_id = g.get('boot_id') if _is_nonempty_str(g.get('boot_id')) else None
        raw_fe = g.get('fault_epoch')
        fault_epoch = raw_fe if _is_nonneg_int(raw_fe) else None
        raw_fl = g.get('fault_latched')
        if isinstance(raw_fl, bool):
            fault_latched = raw_fl or (status in GATE_FAULT_STATES)
        else:
            fault_latched = True if status in GATE_FAULT_STATES else None
        raw_cb = g.get('cmd_blocked')
        cmd_blocked = raw_cb if isinstance(raw_cb, bool) else None

        missing = []
        if boot_id is None:
            missing.append('boot_id')
        if fault_epoch is None:
            missing.append('fault_epoch')
        if raw_fl is None or not isinstance(raw_fl, bool):
            missing.append('fault_latched')
        if cmd_blocked is None:
            missing.append('cmd_blocked')
        # A supplied pending counter is evidence that the published epoch is
        # not yet durable. Absence is retained for legacy peers, not treated
        # as proof of persistence or as a real Gate attestation.
        pending_ok = ('fault_epoch_pending' not in g or
                      (type(g['fault_epoch_pending']) is int and g['fault_epoch_pending'] == 0))

        result = {
            'boot_id': boot_id,
            'status': status,
            'state': raw_state,
            'age_s': None if rx_age is None else round(rx_age, 3),
            'rx_age_s': None if rx_age is None else round(rx_age, 3),
            'rx_fresh': rx_fresh,
            'source_age_s': None if src_age is None else round(src_age, 3),
            'source_fresh': source_fresh,
            'source_sim_time': self.last_sim_time,
            'source_clock_regressed': self.source_clock_regressed,
            'fault_epoch': fault_epoch,
            'epoch_regressed': self.epoch_regressed,
            'boot_changed': self.boot_changed,
            'fault_latched': fault_latched,
            'cmd_blocked': cmd_blocked,
            'contract_met': len(missing) == 0 and not self.epoch_regressed and not self.boot_changed and pending_ok,
            'missing_fields': missing,
            'wheel_odom_stale': g.get('wheel_odom_stale'),
            'est_odom_stale': g.get('est_odom_stale'),
            'depth_stale': g.get('depth_stale'),
            'fault_reasons': g.get('fault_reasons'),
            'last_fault_reasons': g.get('last_fault_reasons'),
            'fault_goals_blocking': g.get('fault_goals_blocking'),
            'not_ready': g.get('not_ready'),
            'counters': {
                k: g.get(k)
                for k in ('stall_events', 'failsafe_events', 'icp_degraded_events', 'operator_unlatch_count')
                if k in g
            },
        }
        if g.get('contract_version') == 'gate-status/1.2-sim' or self.max_status_seq is not None:
            result.update({k: copy.deepcopy(g.get(k)) for k in
                           ('contract_version', 'status_seq', 'safety_state', 'trust', 'nav_evidence')})
            result['r7_invalid'] = self.r7_invalid
            result['contract_met'] = result['contract_met'] and not self.r7_invalid and source_fresh
        if 'fault_epoch_pending' in g:
            result['fault_epoch_pending'] = g['fault_epoch_pending']
        return result


def lease_wellformed(lease):
    """Structure of the Fleet lease view (ownership.lease_view): non-empty owner, exact non-negative int epoch, valid expiry timestamp.
    Expiry against the clock and owner/epoch against a command stay where they were (movement/active-goal checks)."""
    return (isinstance(lease, dict) and _is_nonempty_str(lease.get('owner')) and _is_nonneg_int(lease.get('epoch'))
            and _is_valid_ts(lease.get('expires_at')))


def server_contract_valid(ss):
    """Fail-closed check of the server contract carried by a session snapshot: BOTH versions must be present exact ints.
    A missing, null, bool, float, string or different value is invalid (R1.4.1: a missing protocol_version used to pass)."""
    if not isinstance(ss, dict):
        return False
    pv, sv = ss.get('protocol_version'), ss.get('safety_contract_version')
    return (type(pv) is int and pv == SERVER_PROTOCOL_VERSION
            and type(sv) is int and sv == SAFETY_CONTRACT_VERSION)


def compute_movement_permitted(
    *,
    allow_missions,
    gate_eval,
    motor_path_ok,
    op_ctx,
    map_ctx,
    loc_ctx,
    nav2_goals,
    session_ctx,
    now_wall=None,
    max_state_age_s=2.0,
    max_nav2_age_s=2.0,
):
    """Fail-closed software movement permission; never depends on Gate NORMAL alone."""
    now_w = time.time() if now_wall is None else float(now_wall)
    reasons = []
    if allow_missions is not True:
        reasons.append('MISSIONS_DISABLED')
    if motor_path_ok is not True:
        reasons.append('MOTOR_PATH_UNSAFE')

    g = gate_eval if isinstance(gate_eval, dict) else {}
    if g.get('rx_fresh') is not True:
        reasons.append('GATE_RX_STALE_OR_MISSING')
    if g.get('source_fresh') is not True:
        reasons.append('GATE_SOURCE_STALE')
    if g.get('r7_invalid') is True or ('safety_state' in g and g['safety_state'] != 'NORMAL'):
        reasons.append('GATE_R7_STATE_INVALID')
    if g.get('status') != 'NORMAL':
        reasons.append(f"GATE_NOT_NORMAL:{g.get('status', 'UNKNOWN')}")
    if g.get('fault_latched') is not False:
        reasons.append('GATE_FAULT_LATCHED')
    if g.get('cmd_blocked') is not False:
        reasons.append('GATE_CMD_BLOCKED')
    if not _is_nonempty_str(g.get('boot_id')):
        reasons.append('GATE_BOOT_ID_MISSING')
    if not _is_nonneg_int(g.get('fault_epoch')) or g.get('epoch_regressed') is True:
        reasons.append('GATE_FAULT_EPOCH_INVALID')
    if g.get('boot_changed') is True:
        reasons.append('GATE_BOOT_RECONCILIATION_REQUIRED')
    if 'fault_epoch_pending' in g and (type(g['fault_epoch_pending']) is not int or g['fault_epoch_pending'] != 0):
        reasons.append('GATE_EPOCH_PENDING_OR_INVALID')

    op = op_ctx if isinstance(op_ctx, dict) else {}
    if op.get('operation_mode') != 'NAVIGATION' or not _is_nonneg_int(op.get('operation_revision')):
        reasons.append('OPERATION_MODE_UNCONFIRMED')

    mp = map_ctx if isinstance(map_ctx, dict) else {}
    if mp.get('map_verified') is not True or not _is_nonempty_str(mp.get('map_version_id')) or not _is_nonneg_int(mp.get('map_epoch')):
        reasons.append('MAP_UNVERIFIED')

    lc = loc_ctx if isinstance(loc_ctx, dict) else {}
    if (
        lc.get('localization_status') != 'CONFIRMED'
        or not _is_nonempty_str(mp.get('map_version_id'))
        or lc.get('localization_map_version_id') != mp.get('map_version_id')
        or not _is_nonneg_int(lc.get('localization_epoch'))
    ):
        reasons.append('LOCALIZATION_UNCONFIRMED')

    ng = nav2_goals if isinstance(nav2_goals, dict) else {}
    ng_age = ng.get('age_s')
    # Action status is published on change only; publisher-GID continuity (`status_continuous`) replaces message age.
    continuous = ng.get('status_continuous') is True
    if (
        ng.get('known') is not True
        or (ng.get('received_status_array') is not True and ng.get('source') != 'SIM_ONLY_GATE_R7' and not continuous)
        or not _is_nonneg_int(ng.get('active_count'))
        or (not continuous and (not _is_finite_num(ng_age) or ng_age < 0 or ng_age > max_nav2_age_s))
    ):
        reasons.append('NAV2_STATUS_STALE_OR_UNKNOWN')
    if ng.get('foreign_active_goal_uuids'):
        reasons.append('NAV2_FOREIGN_GOAL_ACTIVE')

    ss = session_ctx if isinstance(session_ctx, dict) else {}
    ss_age = ss.get('last_state_age_s')
    if (
        ss.get('connected') is not True
        or not _is_finite_num(ss_age)
        or ss_age < 0
        or ss_age > max_state_age_s
        or not server_contract_valid(ss)
        or not _is_nonempty_str(ss.get('server_boot_id'))
    ):
        reasons.append('SERVER_DISCONNECTED_OR_STALE')

    lease = ss.get('lease')
    if (
        not isinstance(lease, dict)
        or not _is_nonempty_str(lease.get('owner'))
        or not _is_nonneg_int(lease.get('epoch'))
        or not _is_valid_ts(lease.get('expires_at'))
        or float(lease['expires_at']) <= now_w
    ):
        reasons.append('LEASE_MISSING_OR_EXPIRED')

    return len(reasons) == 0, reasons


def verify_navigation_execution(
    cmd,
    *,
    robot_id,
    agent_boot_id,
    allow_missions,
    gate_eval,
    motor_path_ok,
    op_ctx,
    map_ctx,
    loc_ctx,
    nav2_goals,
    session_ctx,
    active_rec,
    now_wall=None,
    max_state_age_s=2.0,
    max_nav2_age_s=2.0,
    max_clock_skew_s=5.0,
):
    """Validate all local safety preconditions before queuing or submitting a Nav2 goal.

    Returns (ok, ack_status, detail, parsed_safety_context).
    """
    now_w = time.time() if now_wall is None else float(now_wall)
    if not isinstance(cmd, dict):
        return False, 'REJECTED', 'BAD_COMMAND: command must be a dict', None
    if cmd.get('robot_id') != robot_id or cmd.get('owner_boot_id') != agent_boot_id:
        return False, 'REJECTED', 'OWNERSHIP_MISMATCH: robot_id or owner_boot_id mismatch', None

    payload = cmd.get('payload')
    if not isinstance(payload, dict):
        return False, 'REJECTED', 'BAD_PAYLOAD: payload missing', None
    sc, err = parse_safety_context(payload.get('safety_context'))
    if sc is None:
        return False, 'REJECTED', err, None
    if sc['agent_boot_id'] != agent_boot_id:
        return False, 'REJECTED', 'SESSION_MISMATCH: agent_boot_id changed since admission', None

    ss = session_ctx if isinstance(session_ctx, dict) else {}
    if ss.get('connected') is not True or sc['server_boot_id'] != ss.get('server_boot_id'):
        return False, 'REJECTED', 'SESSION_MISMATCH: disconnected or server_boot_id changed', None
    stamp = cmd.get('contract_generation')
    if stamp is not None and ss.get('contract_generation') != stamp:
        # R1.4.3: the contract was invalidated (and possibly recovered) after this command was received; the old approval is dead.
        return False, 'REJECTED', 'CONTRACT_GENERATION_CHANGED: server contract was invalidated after this command was received', None

    cmd_issued = cmd.get('issued_at')
    cmd_expires = cmd.get('expires_at')
    if not _is_valid_ts(cmd_issued) or not _is_valid_ts(cmd_expires):
        return False, 'REJECTED', 'BAD_TIMESTAMPS: command issued_at/expires_at invalid', None
    if float(cmd_issued) > now_w + max_clock_skew_s or float(sc['issued_at']) > now_w + max_clock_skew_s:
        return False, 'REJECTED', 'CLOCK_SKEW: command or safety_context issued_at is in the future', None
    if now_w >= float(cmd_expires) or now_w >= float(sc['expires_at']):
        return False, 'EXPIRED', 'EXPIRED: command or safety_context expired before Nav2 submission', None

    goal = payload.get('goal')
    if not isinstance(goal, dict):
        return False, 'REJECTED', 'BAD_GOAL: goal missing', None
    yaw_raw = goal.get('yaw_rad', 0.0)
    if not all(_is_finite_num(v) for v in (goal.get('x'), goal.get('y'), yaw_raw)):
        return False, 'REJECTED', 'BAD_GOAL: goal needs finite numeric x, y, yaw_rad', None
    gx, gy, gyaw = float(goal['x']), float(goal['y']), float(yaw_raw)
    if goal.get('frame_id') != 'map' or not all(math.isfinite(v) for v in (gx, gy, gyaw)):
        return False, 'REJECTED', "BAD_GOAL: goal frame must be 'map' with finite values", None

    if active_rec is not None:
        return False, 'REJECTED', f"BUSY: mission {active_rec.get('mission_id')} is still active", None

    ng = nav2_goals if isinstance(nav2_goals, dict) else {}
    if ng.get('active_count') != 0 or bool(ng.get('active_goal_uuids')):
        return False, 'REJECTED', 'NAV2_NOT_IDLE: active Nav2 goal already present', None

    permitted, block_reasons = compute_movement_permitted(
        allow_missions=allow_missions,
        gate_eval=gate_eval,
        motor_path_ok=motor_path_ok,
        op_ctx=op_ctx,
        map_ctx=map_ctx,
        loc_ctx=loc_ctx,
        nav2_goals=nav2_goals,
        session_ctx=session_ctx,
        now_wall=now_w,
        max_state_age_s=max_state_age_s,
        max_nav2_age_s=max_nav2_age_s,
    )
    if not permitted:
        code = block_reasons[0]
        return False, 'REJECTED', f'{code}: movement not permitted ({", ".join(block_reasons)})', None

    if gate_eval['boot_id'] != sc['gate_boot_id'] or gate_eval['fault_epoch'] != sc['fault_epoch']:
        return False, 'REJECTED', 'SAFETY_CONTEXT_CHANGED: gate boot_id or fault_epoch changed', None
    if op_ctx.get('operation_mode') != sc['operation_mode'] or op_ctx.get('operation_revision') != sc['operation_revision']:
        return False, 'REJECTED', 'SAFETY_CONTEXT_CHANGED: operation_mode or operation_revision changed', None
    if map_ctx.get('map_version_id') != sc['map_version_id'] or map_ctx.get('map_epoch') != sc['map_epoch']:
        return False, 'REJECTED', 'SAFETY_CONTEXT_CHANGED: map_version_id or map_epoch changed', None
    if loc_ctx.get('localization_status') != sc['localization_status'] or loc_ctx.get('localization_epoch') != sc['localization_epoch']:
        return False, 'REJECTED', 'SAFETY_CONTEXT_CHANGED: localization_status or localization_epoch changed', None

    lease = ss.get('lease') or {}
    if lease.get('owner') != sc['lease_owner'] or lease.get('epoch') != sc['lease_epoch']:
        return False, 'REJECTED', 'LEASE_MISMATCH: lease owner or epoch changed', None

    return True, 'OK', 'verified', sc


def check_active_goal_safety(
    active_rec,
    *,
    allow_missions,
    gate_eval,
    motor_path_ok,
    op_ctx,
    map_ctx,
    loc_ctx,
    session_ctx,
    now_wall=None,
    disconnect_cancel_after_s=3.0,
):
    """Return a non-empty fault reason string if an active Agent-owned goal must be canceled."""
    if not isinstance(active_rec, dict):
        return None
    sc = active_rec.get('safety_context')
    if not isinstance(sc, dict):
        return None
    now_w = time.time() if now_wall is None else float(now_wall)

    if allow_missions is not True:
        return 'MISSIONS_DISABLED'
    if motor_path_ok is not True:
        return 'MOTOR_PATH_UNSAFE'

    g = gate_eval if isinstance(gate_eval, dict) else {}
    if g.get('rx_fresh') is not True or g.get('source_fresh') is not True:
        return 'GATE_STALE_OR_MISSING'
    if g.get('r7_invalid') is True or ('safety_state' in g and g['safety_state'] != 'NORMAL'):
        return 'GATE_R7_STATE_INVALID'
    if g.get('status') not in ('NORMAL', 'STALL_PENDING'):
        return f"GATE_FAULT:{g.get('status')}"
    if g.get('fault_latched') is not False or g.get('cmd_blocked') is not False:
        return 'GATE_BLOCKED_OR_LATCHED'
    if 'fault_epoch_pending' in g and (type(g['fault_epoch_pending']) is not int or g['fault_epoch_pending'] != 0):
        return 'GATE_EPOCH_PENDING_OR_INVALID'
    if g.get('boot_id') != sc.get('gate_boot_id') or g.get('fault_epoch') != sc.get('fault_epoch') or g.get('epoch_regressed') is True or g.get('boot_changed') is True:
        return 'GATE_EPOCH_OR_BOOT_CHANGED'

    ss = session_ctx if isinstance(session_ctx, dict) else {}
    if ss.get('reconciliation_required') is True:
        return 'SERVER_RECONCILIATION_REQUIRED'
    if not server_contract_valid(ss):
        return 'SERVER_CONTRACT_INVALID'          # an active goal is cancelled when the session's contract is lost or invalid
    stamp = active_rec.get('contract_generation')
    if stamp is not None and ss.get('contract_generation') != stamp:
        return 'SERVER_CONTRACT_INVALID'          # R1.4.3: an invalidation after this goal was admitted is a latched obligation: a later recovery keeps the generation changed
    ss_age = ss.get('last_state_age_s')
    if (
        ss.get('connected') is not True
        or not _is_finite_num(ss_age)
        or ss_age > disconnect_cancel_after_s
        or ss.get('server_boot_id') != sc.get('server_boot_id')
    ):
        return 'SERVER_DISCONNECTED_OR_RESTARTED'

    lease = ss.get('lease')
    if (
        not isinstance(lease, dict)
        or lease.get('owner') != sc.get('lease_owner')
        or lease.get('epoch') != sc.get('lease_epoch')
        or not _is_valid_ts(lease.get('expires_at'))
        or float(lease['expires_at']) <= now_w
    ):
        return 'LEASE_REVOKED_OR_EXPIRED'

    op = op_ctx if isinstance(op_ctx, dict) else {}
    if op.get('operation_mode') != sc.get('operation_mode') or op.get('operation_revision') != sc.get('operation_revision'):
        return 'OPERATION_CONTEXT_CHANGED'

    mp = map_ctx if isinstance(map_ctx, dict) else {}
    if mp.get('map_verified') is not True or mp.get('map_version_id') != sc.get('map_version_id') or mp.get('map_epoch') != sc.get('map_epoch'):
        return 'MAP_CONTEXT_CHANGED'

    lc = loc_ctx if isinstance(loc_ctx, dict) else {}
    if (
        lc.get('localization_status') != sc.get('localization_status')
        or lc.get('localization_map_version_id') != sc.get('map_version_id')
        or lc.get('localization_epoch') != sc.get('localization_epoch')
    ):
        return 'LOCALIZATION_CONTEXT_CHANGED'

    if now_w > float(sc['expires_at']):
        return 'SAFETY_CONTEXT_EXPIRED'

    return None


SIM_SUBORDINATE_ACTION = 'follow_path'          # the only Nav2-internal child action of NavigateToPose that SIM_ONLY evidence may attribute to the owned goal
SIM_MAX_FOLLOW_PATH_GOALS = 2
SIM_PENDING_EVIDENCE_GRACE_S = 1.0               # Gate evidence is sampled ~10 Hz; the Agent's own status sample can lead it right after acceptance


def sim_classify_nav2_goals(ev, owned_uuid, subordinate_ok=None, overlap_ok=None):
    """Split the Gate-observed active Nav2 goal UUIDs into (subordinate, foreign, basis).

    SIM_ONLY limited trust model (NOT proof of external-goal blocking, not DDS authentication). ROS action UUIDs of a parent
    NavigateToPose and its FollowPath child legitimately differ, and Nav2 publishes no parent link, so follow_path goals are attributed
    to the owned goal only when ALL of these hold; anything else stays foreign:
      * the owned UUID is the sole active goal of the navigate_to_pose server (an extra NavigateToPose goal is foreign);
      * one active follow_path goal, or exactly TWO for the short hand-over in which Nav2's action server accepts the replacement
        FollowPath goal before the previous one reaches its terminal state (observed in Gazebo, <~0.1 s); more than two is foreign;
      * the owned UUID never appears on follow_path;
      * every other observed action server (spin, backup, navigate_through_poses, ...) is idle-proven with no active UUID;
      * subordinate_ok(uuid) accepts EVERY follow_path goal (Agent: first observed after the owned goal was submitted, and the
        follow_path action clients in the ROS graph are only bt_navigator nodes);
      * for two goals, overlap_ok(frozenset(goals)) accepts the pair (Agent: the same pair has been visible for <= 0.5 s).
    Returns (subordinate_uuids, foreign_uuids, basis)."""
    servers = ev.get('servers') or {}
    ids = list(ev.get('active_uuids') or [])
    foreign_all = [u for u in ids if u != owned_uuid]
    nav = servers.get('navigate_to_pose') or {}
    fp = servers.get(SIM_SUBORDINATE_ACTION) or {}
    if not owned_uuid or owned_uuid not in ids or nav.get('active_uuids') != [owned_uuid]:
        return [], foreign_all, 'OWNED_NOT_SOLE_NAVIGATE_TO_POSE'
    cands = sorted(fp.get('active_uuids') or [])
    if not cands:
        return [], foreign_all, 'NO_CHILD'
    if len(cands) > SIM_MAX_FOLLOW_PATH_GOALS or owned_uuid in cands or fp.get('verdict') != 'ACTIVE_GOALS':
        return [], foreign_all, 'CHILD_COUNT_OR_IDENTITY'
    for name, rec in servers.items():
        if name not in ('navigate_to_pose', SIM_SUBORDINATE_ACTION) and (rec.get('verdict') == 'ACTIVE_GOALS' or rec.get('active_uuids')):
            return [], foreign_all, 'OTHER_ACTION_ACTIVE'
    if set(ids) != {owned_uuid, *cands}:
        return [], foreign_all, 'ID_SET_MISMATCH'
    if subordinate_ok is None or any(subordinate_ok(c) is not True for c in cands):
        return [], foreign_all, 'CHILD_PROVENANCE_UNCONFIRMED'
    if len(cands) == 2 and (overlap_ok is None or overlap_ok(frozenset(cands)) is not True):
        return [], foreign_all, 'HANDOVER_OVERLAP_TOO_LONG_OR_UNCONFIRMED'
    return cands, [], 'FOLLOW_PATH_CHILD_OF_OWNED_NAVIGATE_TO_POSE' + ('_HANDOVER' if len(cands) == 2 else '')


def sim_r7_nav_evidence(gate, required_actions, direct, owned_uuid=None, owned_age_s=None, subordinate_ok=None, overlap_ok=None):
    """SIM_ONLY observation alternative; never fabricates a GoalStatusArray.

    Requires a configured observation scope. Active UUIDs without established
    ownership remain foreign (see sim_classify_nav2_goals for the one narrow subordinate case).
    owned_age_s: seconds since the Agent submitted the owned goal (None = unknown). Only while it is within
    SIM_PENDING_EVIDENCE_GRACE_S may the Agent's own status sample lead idle Gate evidence by exactly the owned UUID."""
    if (not required_actions or not isinstance(gate, dict)
            or gate.get('contract_version') != 'gate-status/1.2-sim'
            or gate.get('trust') != 'UNAUTHENTICATED_SIM_ONLY'
            or gate.get('single_publisher_graph') is not True
            or gate.get('contract_met') is not True
            or gate.get('rx_fresh') is not True or gate.get('source_fresh') is not True
            or gate.get('safety_state') != 'NORMAL'):
        return None
    ev = gate.get('nav_evidence')
    if not isinstance(ev, dict) or type(ev.get('scope_version')) is not int or ev['scope_version'] != 1 or ev.get('observed') is not True:
        return None
    servers, ids = ev.get('servers'), ev.get('active_uuids')
    if (not isinstance(servers, dict) or not set(required_actions).issubset(servers)
            or not isinstance(ids, list) or len(ids) > 64
            or any(not isinstance(u, str) or len(u) != 32 or any(c not in '0123456789abcdef' for c in u) for u in ids)
            or len(set(ids)) != len(ids)):
        return None
    verdicts = []
    for name, record in servers.items():
        if (not _is_nonempty_str(name) or not isinstance(record, dict)
                or not _is_nonempty_str(record.get('writer_gid'))
                or len(record['writer_gid']) != 16 or any(c not in '0123456789abcdef' for c in record['writer_gid'])
                or record.get('verdict') not in ('IDLE_PROVEN_EL', 'IDLE_PROVEN_ES', 'ACTIVE_GOALS')):
            return None
        verdicts.append(record['verdict'])
    idle = all(v != 'ACTIVE_GOALS' for v in verdicts)
    if type(ev.get('all_idle_proven')) is not bool or ev['all_idle_proven'] != idle or bool(ids) == idle:
        return None
    # A sample that contradicts Gate evidence must not be silently discarded.
    direct_ids = direct.get('active_goal_uuids', [])
    pending = None
    if direct.get('received_status_array'):
        if not isinstance(direct_ids, list):
            return None
        missing = set(direct_ids) - set(ids)
        if missing:
            # Not a contradiction only in one case: the Gate evidence is still fully idle and lags the Agent's own accepted goal.
            if (missing == {owned_uuid} and not ids and idle and isinstance(owned_age_s, (int, float))
                    and 0.0 <= owned_age_s <= SIM_PENDING_EVIDENCE_GRACE_S):
                pending = owned_uuid
            else:
                return None
    if type(direct.get('publishers')) is not int or direct['publishers'] != 1:
        return None
    sub, foreign, basis = sim_classify_nav2_goals(ev, owned_uuid, subordinate_ok, overlap_ok)
    return dict(direct, source='SIM_ONLY_GATE_R7', known=True, age_s=gate['age_s'],
                active_count=len(ids), active_goal_uuids=list(ids),
                foreign_active_goal_uuids=foreign, subordinate_active_goal_uuids=sub, subordinate_basis=basis,
                owned_pending_evidence=pending,
                required_actions=list(required_actions), gate_nav_evidence=copy.deepcopy(ev))
