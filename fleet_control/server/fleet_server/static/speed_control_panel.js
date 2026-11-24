'use strict';
/* Web Speed Control UI (Phase C — 2026-11-23).
 * Integrates with Fleet Server POST/GET /api/v1/robots/{id}/speed-limits (SPEED_LIMIT_API_20261123.md).
 *
 * Safety invariants:
 *  - Never publishes /cmd_vel directly; never bypasses R1 Lease, Safety Contract v1, Arbiter, Watchdog, or Gate.
 *  - Uses dynamic ceiling reported by the Agent (no hardcoded upper-bound constants); disables input if ceiling missing.
 *  - POST returns 202 PENDING (approved:false); UI polls GET every 1s (up to 20s) and marks APPLIED only on approved:true && !stale.
 *  - Enables UI only when enforced_by === 'ARBITER' (mock/sim) or future 'WATCHDOG' WITH explicit server+Agent verification evidence.
 *  - Never enables on real robot without positive enforcement proof (enforced_by null -> agent_cannot_enforce).
 *  - Displays /odom velocity strictly as estimated telemetry (dimmed on STALE, no numbers on UNAVAILABLE).
 */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.SpeedControlPanel = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const MIN_LINEAR_MPS = 0.01;
  const MIN_ANGULAR_RPS = 0.02;
  const STATE_MAX_AGE_S = 2.0;
  const DEFAULT_POLL_INTERVAL_MS = 1000;
  const DEFAULT_POLL_TIMEOUT_MS = 20000;

  const ERROR_MESSAGES = Object.freeze({
    not_lease_owner: 'Lease를 먼저 획득하세요 (현재 Lease 소유자가 아님).',
    lease_epoch_mismatch: 'Lease 세대(epoch)가 변경되었습니다 — 상태를 새로고침 후 다시 시도하세요.',
    agent_protocol: 'Agent 오프라인 또는 안전 계약(Protocol v2 / Contract v1) 불일치.',
    agent_boot_mismatch: 'Agent가 재시작되었습니다(boot_id 변경) — 상태를 새로고침하세요.',
    agent_cannot_enforce: '이 로봇은 서버 속도 제한을 강제할 수 없습니다 (실차 Agent 포함).',
    agent_ceiling_unknown: 'Agent가 속도 상한(ceiling)을 보고하지 않아 입력이 차단되었습니다.',
    bad_scope: '잘못된 scope입니다 (manual 또는 auto만 허용).',
    bad_value: '속도 값이 유한한 양수가 아닙니다.',
    bad_client_request_id: '요청 식별자(client_request_id) 형식이 잘못되었습니다.',
    speed_exceeds_ceiling: '요청 속도가 Agent 보고 상한(ceiling)을 초과했습니다.',
    speed_below_minimum: `최소 속도(${MIN_LINEAR_MPS} m/s, ${MIN_ANGULAR_RPS} rad/s) 미만입니다 (정지는 STOP 사용).`,
    stale_generation: '세대(generation)가 오래되었습니다 — 최신 상태를 확인 후 다시 요청하세요.',
    idempotency_conflict: '동일 요청 ID에 서로 다른 값이 전달되었습니다.',
  });

  function formatErrorCode(code, fallback) {
    if (typeof code === 'string' && ERROR_MESSAGES[code]) return `${ERROR_MESSAGES[code]} (${code})`;
    if (typeof fallback === 'string' && fallback) return fallback;
    return code ? String(code) : '속도 제한 요청이 거부되었습니다.';
  }

  function extractScopeCeiling(speedLimits, scope) {
    if (!speedLimits || typeof speedLimits !== 'object') return null;
    const cRoot = speedLimits.ceiling;
    if (!cRoot || typeof cRoot !== 'object') return null;
    const c = cRoot[scope];
    if (!c || typeof c !== 'object') return null;
    const lin = c.linear_mps !== undefined ? Number(c.linear_mps) : Number(c.max_linear_mps);
    const ang = c.angular_rps !== undefined ? Number(c.angular_rps) : Number(c.max_angular_rps);
    const rev = c.reverse_mps !== undefined && Number.isFinite(Number(c.reverse_mps)) ? Number(c.reverse_mps) : null;
    if (!Number.isFinite(lin) || !Number.isFinite(ang) || lin <= 0 || ang <= 0) return null;
    return { linear_mps: lin, angular_rps: ang, reverse_mps: rev };
  }

  function validateSpeedInput(linear, angular, ceiling) {
    if (!ceiling || !Number.isFinite(ceiling.linear_mps) || !Number.isFinite(ceiling.angular_rps)) {
      return { ok: false, code: 'agent_ceiling_unknown', error: ERROR_MESSAGES.agent_ceiling_unknown };
    }
    const v = Number(linear);
    const w = Number(angular);
    if (!Number.isFinite(v) || !Number.isFinite(w)) {
      return { ok: false, code: 'bad_value', error: ERROR_MESSAGES.bad_value };
    }
    if (v < MIN_LINEAR_MPS - 1e-9 || w < MIN_ANGULAR_RPS - 1e-9) {
      return { ok: false, code: 'speed_below_minimum', error: ERROR_MESSAGES.speed_below_minimum };
    }
    if (v > ceiling.linear_mps + 1e-9) {
      return {
        ok: false,
        code: 'speed_exceeds_ceiling',
        error: `선속도(${v.toFixed(2)} m/s)가 Agent 보고 상한(${ceiling.linear_mps.toFixed(2)} m/s)을 초과합니다.`,
      };
    }
    if (w > ceiling.angular_rps + 1e-9) {
      return {
        ok: false,
        code: 'speed_exceeds_ceiling',
        error: `각속도(${w.toFixed(2)} rad/s)가 Agent 보고 상한(${ceiling.angular_rps.toFixed(2)} rad/s)을 초과합니다.`,
      };
    }
    return { ok: true, code: '', error: '', linear: Math.round(v * 1000) / 1000, angular: Math.round(w * 1000) / 1000 };
  }

  function evaluateEnforcement(speedLimits, ctx) {
    const sl = speedLimits || {};
    const enforcedBy = sl.enforced_by || (ctx && ctx.enforcedBy) || null;
    if (enforcedBy === 'ARBITER') {
      return {
        supported: true,
        enforcedBy: 'ARBITER',
        label: 'ARBITER Clamp (Mock/시뮬 강제)',
        reason: '',
      };
    }
    if (enforcedBy === 'WATCHDOG') {
      // Future Watchdog clamp: bare boolean flags alone MUST NOT enable real-robot speed control.
      // Requires fresh, corroborated Server + Agent enforcement evidence bound to current boot IDs and writer GID.
      const ev = sl.watchdog_evidence || sl.enforcement_evidence || null;
      const age = ev && ev.evidence_age_s !== undefined && ev.evidence_age_s !== null && ev.evidence_age_s !== ''
        ? Number(ev.evidence_age_s) : NaN;
      const hasCorroboratedEvidence = Boolean(
        ev && typeof ev === 'object' &&
        ev.server_verified === true &&
        ev.agent_verified === true &&
        typeof ev.watchdog_boot_id === 'string' && ev.watchdog_boot_id.length > 0 &&
        typeof ev.agent_boot_id === 'string' && ev.agent_boot_id.length > 0 &&
        (!ctx || !ctx.agentBootId || ev.agent_boot_id === ctx.agentBootId) &&
        typeof ev.output_gid === 'string' && ev.output_gid.length > 0 &&
        Number.isFinite(age) && age >= 0 && age <= STATE_MAX_AGE_S &&
        ctx && ctx.watchdogAuthorized === true
      );
      if (hasCorroboratedEvidence) {
        return {
          supported: true,
          enforcedBy: 'WATCHDOG',
          label: 'WATCHDOG Clamp (Server·Agent 검증 증거 확인됨)',
          reason: '',
        };
      }
      return {
        supported: false,
        enforcedBy: 'WATCHDOG',
        label: 'WATCHDOG (검증된 적용 증거 부재 · 차단)',
        reason: 'Watchdog Clamp는 단순 플래그로 허용되지 않으며 Server·Agent 검증 적용 증거(boot_id·output_gid·신선도·Watchdog 승인)가 필요합니다.',
      };
    }
    return {
      supported: false,
      enforcedBy: null,
      label: '강제 지점 없음 (실차 Agent 포함 · 차단)',
      reason: ERROR_MESSAGES.agent_cannot_enforce,
    };
  }

  function verifyAppliedMatch(req, view, currentCtx) {
    if (!req || !view || typeof view !== 'object') {
      return { ok: false, reason: '응답 데이터 없음' };
    }
    if (view.stale === true) {
      return { ok: false, reason: 'STALE 응답은 적용 완료로 인정하지 않습니다.' };
    }
    if (String(view.status || '').toUpperCase() !== 'APPLIED' || view.approved !== true) {
      return { ok: false, reason: `상태 불일치 (status=${view.status}, approved=${view.approved})` };
    }
    if (currentCtx && currentCtx.robotId && currentCtx.robotId !== req.robotId) {
      return { ok: false, reason: `robot_id 변경됨 (${req.robotId} -> ${currentCtx.robotId})` };
    }
    if (view.robot_id && view.robot_id !== req.robotId) {
      return { ok: false, reason: `응답 robot_id 불일치 (${view.robot_id} != ${req.robotId})` };
    }
    if (currentCtx && currentCtx.agentBootId && req.agentBootId && currentCtx.agentBootId !== req.agentBootId) {
      return { ok: false, reason: `Agent 재시작 감지 (boot ${req.agentBootId} -> ${currentCtx.agentBootId})` };
    }
    const respBoot = view.agent_boot_id || (view.applied && view.applied.agent_boot_id);
    if (respBoot && req.agentBootId && respBoot !== req.agentBootId) {
      return { ok: false, reason: `응답 agent_boot_id 불일치 (${respBoot} != ${req.agentBootId})` };
    }
    if (view.scope !== req.scope) {
      return { ok: false, reason: `scope 불일치 (응답 ${view.scope} != 요청 ${req.scope})` };
    }
    if (view.applied && view.applied.scope && view.applied.scope !== req.scope) {
      return { ok: false, reason: `applied.scope 불일치 (${view.applied.scope} != ${req.scope})` };
    }
    if (!Number.isInteger(view.generation) || view.generation !== req.generation) {
      return { ok: false, reason: `generation 불일치 (응답 ${view.generation} != 요청 ${req.generation})` };
    }
    if (!view.applied || typeof view.applied !== 'object') {
      return { ok: false, reason: 'applied 상세 값이 누락되었습니다.' };
    }
    if (!Number.isInteger(view.applied.generation) || view.applied.generation !== req.generation) {
      return { ok: false, reason: `applied.generation 불일치 (${view.applied.generation} != ${req.generation})` };
    }
    if (req.requestId && view.request_id && view.request_id !== req.requestId) {
      return { ok: false, reason: `request_id 불일치 (${view.request_id} != ${req.requestId})` };
    }
    const appLin = Number(view.applied.linear_mps);
    const appAng = Number(view.applied.angular_rps);
    if (!Number.isFinite(appLin) || !Number.isFinite(appAng)) {
      return { ok: false, reason: 'applied 속도 값이 유효한 숫자가 아닙니다.' };
    }
    if (Math.abs(appLin - req.requestedLinear) > 1e-6 || Math.abs(appAng - req.requestedAngular) > 1e-6) {
      return {
        ok: false,
        reason: `applied 값 불일치 (응답 ${appLin.toFixed(2)}/${appAng.toFixed(2)} != 요청 ${req.requestedLinear.toFixed(2)}/${req.requestedAngular.toFixed(2)})`,
      };
    }
    return { ok: true, reason: '' };
  }

  function evaluateSpeedGate(ctx, scope, speedLimits) {
    const enf = evaluateEnforcement(speedLimits, ctx);
    const ceiling = extractScopeCeiling(speedLimits, scope);
    const base = { allowed: false, code: '', reason: '', enforcement: enf, ceiling };
    if (!ctx || !ctx.robotId) {
      return { ...base, reason: '로봇을 선택하세요.' };
    }
    if (ctx.apiUnsupported === true) {
      return { ...base, reason: '서버가 /speed-limits API를 지원하지 않습니다 (404).' };
    }
    if (ctx.online !== true || !Number.isFinite(ctx.stateAgeS) || ctx.stateAgeS < 0 || ctx.stateAgeS > STATE_MAX_AGE_S) {
      return { ...base, code: 'agent_protocol', reason: '로봇 상태가 OFFLINE 또는 STALE입니다.' };
    }
    if (ctx.canControl !== true) {
      return { ...base, code: 'not_lease_owner', reason: '운영자 제어 권한이 필요합니다.' };
    }
    if (ctx.reconciliationRequired === true) {
      return { ...base, reason: 'RECONCILIATION_REQUIRED — 복구 완료 전 속도 변경 차단.' };
    }
    const contractOk = ctx.protocolVersion === 2 && ctx.safetyContractVersion === 1 && typeof ctx.agentBootId === 'string' && ctx.agentBootId.length > 0;
    if (!contractOk) {
      return { ...base, code: 'agent_protocol', reason: ERROR_MESSAGES.agent_protocol };
    }
    const leaseOk = ctx.leaseOwned === true && Number.isInteger(ctx.leaseEpoch) && ctx.leaseEpoch >= 1;
    if (!leaseOk) {
      return { ...base, code: 'not_lease_owner', reason: ERROR_MESSAGES.not_lease_owner };
    }
    if (!enf.supported) {
      return { ...base, code: 'agent_cannot_enforce', reason: enf.reason };
    }
    if (!ceiling) {
      return { ...base, code: 'agent_ceiling_unknown', reason: ERROR_MESSAGES.agent_ceiling_unknown };
    }
    return { allowed: true, code: '', reason: '', enforcement: enf, ceiling };
  }

  function classifySpeedStatus(view, ctx) {
    if (ctx && (ctx.online !== true || !Number.isFinite(ctx.stateAgeS) || ctx.stateAgeS > STATE_MAX_AGE_S)) {
      return {
        status: 'STALE',
        approved: false,
        tone: 'warn',
        text: '적용 상태 미확인 — 통신 단절 또는 오래된 상태 보고로 실제 적용 상한을 확인할 수 없습니다.',
      };
    }
    if (!view || typeof view !== 'object') {
      return { status: 'NONE', approved: false, tone: 'muted', text: '요청 이력 없음' };
    }
    if (view.stale === true) {
      const reportAge = view.agent_report_age_s !== undefined && view.agent_report_age_s !== null ? Number(view.agent_report_age_s) : NaN;
      const effConfirmed = Number.isFinite(reportAge) && reportAge >= 0 && reportAge <= STATE_MAX_AGE_S &&
        view.effective && typeof view.effective === 'object';
      if (!effConfirmed) {
        return {
          status: 'STALE',
          approved: false,
          tone: 'warn',
          text: `STALE — 적용 상태 미확인 (실제 적용 상한 확인 불가 · ${view.stale_reason || view.reason || 'STALE'} — 다시 확인·설정하세요)`,
        };
      }
      return {
        status: 'STALE',
        approved: false,
        tone: 'warn',
        text: `STALE — 이전 설정 만료 (실제 적용 상한 재확인 필요 · 승인 상태 아님 · ${view.stale_reason || view.reason || 'STALE'})`,
      };
    }
    const st = String(view.status || 'NONE').toUpperCase();
    if (st === 'APPLIED' && view.approved === true) {
      return {
        status: 'APPLIED',
        approved: true,
        tone: 'ok',
        text: `APPLIED (gen ${view.generation ?? '-'}) — Agent 적용 확인됨`,
      };
    }
    if (st === 'PENDING') {
      return {
        status: 'PENDING',
        approved: false,
        tone: 'warn',
        text: `PENDING (gen ${view.generation ?? '-'}) — Agent 적용 확인 대기 중 (${view.reason || 'PENDING_AGENT_APPLY'})`,
      };
    }
    if (st === 'REJECTED') {
      return {
        status: 'REJECTED',
        approved: false,
        tone: 'bad',
        text: `REJECTED — ${formatErrorCode(view.reason, view.reason)}`,
      };
    }
    if (st === 'EXPIRED') {
      return {
        status: 'EXPIRED',
        approved: false,
        tone: 'bad',
        text: `EXPIRED — 적용 확인 시간 초과 또는 세션 변경 (${view.reason || 'AGENT_NO_CONFIRMATION'})`,
      };
    }
    if (st === 'SUPERSEDED') {
      return {
        status: 'SUPERSEDED',
        approved: false,
        tone: 'muted',
        text: `SUPERSEDED — 새 요청으로 대체됨 (${view.reason || 'NEWER_REQUEST'})`,
      };
    }
    return { status: 'NONE', approved: false, tone: 'muted', text: '요청 이력 없음 (NONE)' };
  }

  function classifySafetyLimiting(ctx) {
    if (!ctx || ctx.online !== true || !Number.isFinite(ctx.stateAgeS) || ctx.stateAgeS > STATE_MAX_AGE_S) {
      return { status: 'STALE_TELEMETRY', label: 'STALE / 미수신 (안전 제한 상태 미확인)', tone: 'warn' };
    }
    if (ctx.motorLocked === true) {
      return { status: 'MOTOR_LOCKED', label: '모터 잠금 (실차 구동 차단 · 영속도 강제)', tone: 'warn' };
    }
    if (ctx.gateBlocked === true || ctx.gateFault === true) {
      return { status: 'LIMITED_BY_SAFETY_GATE', label: `Safety Gate 감속·차단 (${ctx.gateReason || 'GATE_BLOCK'})`, tone: 'bad' };
    }
    if (ctx.watchdogAuthorized === false) {
      return { status: 'LIMITED_BY_WATCHDOG', label: `Motion Watchdog 차단 (${ctx.watchdogState || 'LOCKED'})`, tone: 'warn' };
    }
    if (ctx.safetySlowdown === true) {
      return { status: 'LIMITED_BY_SAFETY_GATE', label: `안전 제한 감속 활성 (${ctx.slowdownReason || 'DEPTH/COSTMAP'})`, tone: 'warn' };
    }
    return { status: 'NORMAL', label: '정상 (강제 상한 내 동작)', tone: 'ok' };
  }

  function fmtPair(obj, linKey, angKey) {
    if (!obj || typeof obj !== 'object') return '-';
    const lin = obj[linKey] !== undefined ? Number(obj[linKey]) : Number(obj.linear_mps ?? obj.max_linear_mps);
    const ang = obj[angKey] !== undefined ? Number(obj[angKey]) : Number(obj.angular_rps ?? obj.max_angular_rps);
    if (!Number.isFinite(lin) || !Number.isFinite(ang)) return '-';
    return `v ${lin.toFixed(2)} m/s · w ${ang.toFixed(2)} rad/s`;
  }

  function mount(host, opts) {
    if (!host || !opts || typeof opts.getContext !== 'function') {
      throw new Error('SpeedControlPanel.mount requires host and opts.getContext');
    }
    const pollIntervalMs = Math.max(20, Number(opts.pollIntervalMs) || DEFAULT_POLL_INTERVAL_MS);
    const pollTimeoutMs = Math.max(100, Number(opts.pollTimeoutMs) || DEFAULT_POLL_TIMEOUT_MS);

    const state = {
      serverView: null,        // Latest GET/POST /robots/{id}/speed-limits response
      apiUnsupported: false,   // True if server responded 404 to /speed-limits
      polling: null,           // { timer, startedAt, requestId, generation, robotId, scope }
      reqSeq: 0,
      lastIdentity: null,
      lastResult: { status: 'NONE', message: '변경 이력 없음' },
      destroyed: false,
    };

    const ceilingTip = '이 상한은 Agent가 보고한 설정값이며 실차 검증값이 아닙니다.';

    host.innerHTML = `<section class="card speed-control" aria-label="속도 제한 및 /odom 속도 관측">
      <div class="sec-head">
        <h2>속도 제한 · 관측 (Phase C 계약)</h2>
        <span class="badge neutral" data-spd="statusBadge">NONE</span>
      </div>
      <p class="muted small">웹 UI는 <code>/cmd_vel</code>을 직접 발행하지 않으며, POST(202 PENDING) 후 GET 폴링으로 Agent 적용(<code>approved:true</code>)을 확인합니다.</p>
      <div class="speed-telemetry-grid">
        <div class="speed-box">
          <small>강제 지점 · 모드</small>
          <strong data-spd="enforcedBy">확인 중</strong>
          <span class="muted small" data-spd="modeBadge">MODE UNKNOWN</span>
        </div>
        <div class="speed-box">
          <small>/odom 추정 속도 (독립 측정 아님 · 정지 보장 아님)</small>
          <strong data-spd="obsVel">UNAVAILABLE</strong>
          <span class="muted small" data-spd="obsVelMeta">/odom 텔레메트리 미수신</span>
        </div>
        <div class="speed-box">
          <small>안전 제한 감속·차단</small>
          <strong data-spd="limitState">확인 중</strong>
          <span class="muted small" data-spd="contractMeta">Contract / Lease 확인 중</span>
        </div>
      </div>
      <dl class="speed-contract-meta small">
        <div><dt>요청(requested)</dt><dd data-spd="metaRequested">-</dd></div>
        <div><dt>적용(applied)</dt><dd data-spd="metaApplied">-</dd></div>
        <div><dt>실효(effective)</dt><dd data-spd="metaEffective">-</dd></div>
        <div><dt>세대 · 출처 · 경과</dt><dd data-spd="metaGenSource">-</dd></div>
      </dl>
      <div class="speed-section" data-spd="manualSection">
        <div class="sec-head">
          <h3>1. 수동 속도 제한 (scope: manual · 전진만)</h3>
          <span class="badge neutral" data-spd="manualCeilingBadge" title="${ceilingTip}">ceiling 미수신</span>
        </div>
        <div class="mform">
          <label title="${ceilingTip}">선속도 (m/s)<input data-spd="manualLin" type="number" min="${MIN_LINEAR_MPS}" step="0.01" value="" disabled/></label>
          <label title="${ceilingTip}">각속도 (rad/s)<input data-spd="manualAng" type="number" min="${MIN_ANGULAR_RPS}" step="0.01" value="" disabled/></label>
          <button type="button" data-spd="manualApply" disabled>수동 제한 요청</button>
        </div>
        <p class="small muted" data-spd="manualGateNote">Agent ceiling 및 강제 지점 확인 중</p>
      </div>
      <div class="speed-section" data-spd="autoSection">
        <div class="sec-head">
          <h3>2. 자율 속도 제한 (scope: auto · Nav2/Mapping)</h3>
          <span class="badge neutral" data-spd="autoCeilingBadge" title="${ceilingTip}">ceiling 미수신</span>
        </div>
        <div class="mform">
          <label title="${ceilingTip}">자율 선속도 (m/s)<input data-spd="autoLin" type="number" min="${MIN_LINEAR_MPS}" step="0.01" value="" disabled/></label>
          <label title="${ceilingTip}">자율 각속도 (rad/s)<input data-spd="autoAng" type="number" min="${MIN_ANGULAR_RPS}" step="0.01" value="" disabled/></label>
          <button type="button" data-spd="autoApply" disabled>자율 제한 요청</button>
        </div>
        <p class="small muted" data-spd="autoGateNote">Agent ceiling 및 강제 지점 확인 중</p>
      </div>
      <div class="speed-result small" role="status" aria-live="polite" data-spd="result">변경 이력 없음</div>
    </section>`;

    const q = (name) => host.querySelector(`[data-spd="${name}"]`);

    function stopPolling() {
      if (state.polling && state.polling.timer) {
        clearTimeout(state.polling.timer);
      }
      state.polling = null;
    }

    function setResult(status, message) {
      state.lastResult = { status, message };
      const el = q('result');
      el.textContent = `[${status}] ${message}`;
      el.className = 'speed-result small ' + (status === 'APPLIED' ? 'ok' : (status === 'REJECTED' || status === 'EXPIRED') ? 'bad' : (status === 'PENDING' || status === 'STALE') ? 'warn' : 'muted');
    }

    function identityOf(c) {
      if (!c) return '';
      return `${c.robotId || ''}\0${c.serverBootId || ''}\0${c.leaseEpoch ?? ''}\0${c.agentBootId || ''}\0${c.protocolVersion ?? ''}\0${c.safetyContractVersion ?? ''}`;
    }

    function mergedSpeedLimits(ctx) {
      const fromState = (ctx && ctx.speedLimitsState && typeof ctx.speedLimitsState === 'object') ? ctx.speedLimitsState : null;
      const fromApi = state.serverView;
      if (!fromApi && !fromState) return null;
      const merged = Object.assign({}, fromState || {}, fromApi || {});
      // Preserve ceiling, enforced_by, and corroborated watchdog evidence from state if serverView omits them
      if (!merged.ceiling && fromState && fromState.ceiling) merged.ceiling = fromState.ceiling;
      if (!merged.enforced_by && fromState && fromState.enforced_by) merged.enforced_by = fromState.enforced_by;
      if (!merged.watchdog_evidence && fromState && fromState.watchdog_evidence) merged.watchdog_evidence = fromState.watchdog_evidence;
      if (!merged.enforcement_evidence && fromState && fromState.enforcement_evidence) merged.enforcement_evidence = fromState.enforcement_evidence;
      return merged;
    }

    function syncManualCapIfApplied(sl) {
      if (!sl || sl.stale === true || typeof opts.onManualCapChange !== 'function') return;
      const eff = (sl.effective && sl.effective.manual) || (sl.scope === 'manual' && sl.approved && !sl.stale ? sl.applied : null);
      if (eff && Number.isFinite(Number(eff.linear_mps)) && Number.isFinite(Number(eff.angular_rps))) {
        opts.onManualCapChange({ maxLinear: Number(eff.linear_mps), maxAngular: Number(eff.angular_rps) });
      }
    }

    async function fetchServerView(robotId, skipAutoSync) {
      if (!robotId || typeof opts.request !== 'function' || state.destroyed) return null;
      try {
        const res = await opts.request(`/robots/${encodeURIComponent(robotId)}/speed-limits`, { method: 'GET' });
        if (res.status === 404) {
          state.apiUnsupported = true;
          return null;
        }
        state.apiUnsupported = false;
        const body = await res.json();
        if (res.ok && body && typeof body === 'object') {
          state.serverView = body;
          if (!skipAutoSync) syncManualCapIfApplied(body);
          return body;
        }
      } catch (_) {
        // Network error handled by reconnect/offline telemetry
      }
      return null;
    }

    function updateScopeControls(scope, gate, prefix) {
      const linEl = q(prefix + 'Lin');
      const angEl = q(prefix + 'Ang');
      const btnEl = q(prefix + 'Apply');
      const badgeEl = q(prefix + 'CeilingBadge');
      const noteEl = q(prefix + 'GateNote');
      const c = gate.ceiling;
      if (c) {
        badgeEl.textContent = `상한(설정값) v≤${c.linear_mps.toFixed(2)} · w≤${c.angular_rps.toFixed(2)}${c.reverse_mps != null ? ' (후진 ' + c.reverse_mps.toFixed(2) + ')' : ''}`;
        badgeEl.className = 'badge ' + (gate.allowed ? 'ok' : 'neutral');
        linEl.max = String(c.linear_mps);
        angEl.max = String(c.angular_rps);
        if (linEl.value === '') linEl.value = c.linear_mps.toFixed(2);
        if (angEl.value === '') angEl.value = c.angular_rps.toFixed(2);
        linEl.disabled = !gate.allowed;
        angEl.disabled = !gate.allowed;
      } else {
        badgeEl.textContent = 'ceiling 미수신 (입력 차단)';
        badgeEl.className = 'badge warn';
        linEl.disabled = true;
        angEl.disabled = true;
      }
      btnEl.disabled = !gate.allowed || Boolean(state.polling);
      noteEl.textContent = state.polling
        ? `폴링 중 (${state.polling.scope} · gen ${state.polling.generation ?? '-'})...`
        : gate.allowed
          ? `요청 가능 — POST(202 PENDING) 후 GET 폴링으로 Agent 적용을 확인합니다.`
          : `차단됨: ${gate.reason}`;
    }

    function refresh() {
      if (state.destroyed) return;
      const rawCtx = opts.getContext() || {};
      const ctx = { ...rawCtx, apiUnsupported: state.apiUnsupported };
      const id = identityOf(ctx);

      // Detect Lease loss, epoch change, server reboot, or Agent restart
      if (state.lastIdentity !== null && state.lastIdentity !== id) {
        const prevRobot = state.lastIdentity.split('\0')[0];
        if (prevRobot !== (ctx.robotId || '')) {
          stopPolling();
          state.serverView = null;
          state.apiUnsupported = false;
        } else if (state.polling) {
          stopPolling();
          setResult('STALE', 'Lease·세대·서버 또는 Agent 세션이 변경되어 진행 중인 폴링을 중단했습니다 — 적용 상태 미확인.');
        } else if (state.serverView && state.serverView.approved && !state.serverView.stale) {
          state.serverView = { ...state.serverView, stale: true, approved: false, stale_reason: 'SESSION_OR_LEASE_CHANGED', agent_report_age_s: null };
          setResult('STALE', 'Lease·세션 변경 또는 재시작 감지 — 실제 적용 상한 미확인 (최신 상태를 확인 후 다시 설정하세요).');
        }
      }
      state.lastIdentity = id;

      const sl = mergedSpeedLimits(ctx);
      const cls = classifySpeedStatus(sl, ctx);
      const statusBadge = q('statusBadge');
      statusBadge.textContent = cls.status;
      statusBadge.className = 'badge ' + (cls.tone === 'ok' ? 'ok' : cls.tone === 'bad' ? 'bad' : cls.tone === 'warn' ? 'warn' : 'neutral');

      const enf = evaluateEnforcement(sl, ctx);
      q('enforcedBy').textContent = enf.label;
      const mode = String(ctx.operationMode || 'UNKNOWN').toUpperCase();
      q('modeBadge').textContent = `MODE ${mode}`;
      q('manualSection').classList.toggle('speed-active', mode === 'MANUAL');
      q('autoSection').classList.toggle('speed-active', mode === 'NAVIGATION' || mode === 'AUTO' || mode === 'MAPPING');

      // /odom estimated velocity display (dimmed on STALE, no numbers on UNAVAILABLE)
      const obs = ctx.observedVelocity || { status: 'UNAVAILABLE', linear_x: null, angular_z: null };
      const obsEl = q('obsVel');
      if (obs.status === 'OK' && Number.isFinite(obs.linear_x) && Number.isFinite(obs.angular_z)) {
        obsEl.textContent = `v ${obs.linear_x.toFixed(2)} m/s · w ${obs.angular_z.toFixed(2)} rad/s`;
        obsEl.className = '';
        q('obsVelMeta').textContent = `출처: ${obs.source || '/odom'} (추정값 · LIVE)`;
      } else if (obs.status === 'STALE') {
        const staleNums = Number.isFinite(obs.linear_x) && Number.isFinite(obs.angular_z)
          ? `STALE (v ${obs.linear_x.toFixed(2)} · w ${obs.angular_z.toFixed(2)})`
          : 'STALE';
        obsEl.textContent = staleNums;
        obsEl.className = 'vel-stale';
        q('obsVelMeta').textContent = '/odom 속도 지연 — 흐리게 표시됨';
      } else {
        obsEl.textContent = 'UNAVAILABLE';
        obsEl.className = 'muted';
        q('obsVelMeta').textContent = '/odom 속도 미수신 (숫자 미표시)';
      }

      // Safety limiting & contract
      const lim = classifySafetyLimiting(ctx);
      q('limitState').textContent = lim.label;
      q('limitState').className = lim.tone;
      q('contractMeta').textContent = `Contract v${ctx.safetyContractVersion ?? '?'} (proto ${ctx.protocolVersion ?? '?'}) · Lease ${ctx.leaseOwned ? '보유(epoch ' + ctx.leaseEpoch + ')' : '미보유'}`;

      // Contract detail fields: requested, applied, effective, generation, source, age
      q('metaRequested').textContent = sl && sl.requested ? `${sl.scope || ''}: ${fmtPair(sl.requested, 'max_linear_mps', 'max_angular_rps')}` : '-';
      q('metaApplied').textContent = sl && sl.applied
        ? `${fmtPair(sl.applied, 'linear_mps', 'angular_rps')} (gen ${sl.applied.generation ?? '-'}, age ${sl.applied.age_s != null ? Number(sl.applied.age_s).toFixed(1) + 's' : '-'})`
        : '-';
      const eff = sl && sl.effective;
      if (ctx.online !== true || !Number.isFinite(ctx.stateAgeS) || ctx.stateAgeS > STATE_MAX_AGE_S) {
        q('metaEffective').textContent = '미확인 (통신 단절/STALE)';
      } else if (eff && (eff.manual || eff.auto)) {
        const parts = [];
        if (eff.manual) parts.push(`manual ${fmtPair(eff.manual, 'linear_mps', 'angular_rps')}`);
        if (eff.auto) parts.push(`auto ${fmtPair(eff.auto, 'linear_mps', 'angular_rps')}`);
        q('metaEffective').textContent = parts.join(' | ');
      } else {
        q('metaEffective').textContent = fmtPair(eff, 'linear_mps', 'angular_rps');
      }
      const ageTxt = sl && sl.agent_report_age_s != null ? `${Number(sl.agent_report_age_s).toFixed(1)}s` : '-';
      q('metaGenSource').textContent = sl
        ? `gen ${sl.generation ?? '-'} · ${sl.source || '-'} · 보고경과 ${ageTxt}`
        : '-';

      if (ctx.online !== true || !Number.isFinite(ctx.stateAgeS) || ctx.stateAgeS > STATE_MAX_AGE_S) {
        setResult('STALE', cls.text);
      } else if (sl && (sl.stale === true || (!state.polling && sl.status && sl.status !== 'NONE'))) {
        if (state.lastResult.status === 'NONE' || sl.stale === true) {
          setResult(cls.status, cls.text);
        }
      }

      updateScopeControls('manual', evaluateSpeedGate(ctx, 'manual', sl), 'manual');
      updateScopeControls('auto', evaluateSpeedGate(ctx, 'auto', sl), 'auto');
    }

    async function submitScope(scope) {
      const prefix = scope === 'auto' ? 'auto' : 'manual';
      const rawCtx = opts.getContext() || {};
      const ctx = { ...rawCtx, apiUnsupported: state.apiUnsupported };
      const sl = mergedSpeedLimits(ctx);
      const gate = evaluateSpeedGate(ctx, scope, sl);
      if (!gate.allowed) {
        setResult('REJECTED', gate.reason);
        refresh();
        return { ok: false, status: 'REJECTED', reason: gate.reason };
      }
      const check = validateSpeedInput(q(prefix + 'Lin').value, q(prefix + 'Ang').value, gate.ceiling);
      if (!check.ok) {
        setResult('REJECTED', check.error);
        return { ok: false, status: 'REJECTED', reason: check.error };
      }
      if (typeof opts.request !== 'function') {
        const msg = 'Fleet API 요청 핸들러가 연결되어 있지 않습니다.';
        setResult('REJECTED', msg);
        return { ok: false, status: 'REJECTED', reason: msg };
      }

      stopPolling();
      const clientRequestId = `ui-${scope}-${Date.now()}-${++state.reqSeq}`;
      const payload = {
        lease_epoch: ctx.leaseEpoch,
        agent_boot_id: ctx.agentBootId,
        protocol_version: 2,
        safety_contract_version: 1,
        scope,
        max_linear_mps: check.linear,
        max_angular_rps: check.angular,
        client_request_id: clientRequestId,
      };
      if (sl && Number.isInteger(sl.generation)) {
        payload.expected_generation = sl.generation;
      }

      let res, body;
      try {
        res = await opts.request(`/robots/${encodeURIComponent(ctx.robotId)}/speed-limits`, {
          method: 'POST',
          body: JSON.stringify(payload),
        });
        body = await res.json().catch(() => ({}));
      } catch (err) {
        const msg = err && err.message ? err.message : '네트워크 오류';
        setResult('REJECTED', `요청 전송 실패: ${msg}`);
        refresh();
        return { ok: false, status: 'REJECTED', reason: msg };
      }

      if (res.status === 404) {
        state.apiUnsupported = true;
        setResult('REJECTED', '서버가 /speed-limits API를 지원하지 않습니다 (404).');
        refresh();
        return { ok: false, status: 'REJECTED', reason: 'HTTP 404' };
      }
      if (!res.ok) {
        const errCode = (body && body.error && body.error.code) || body.code || body.reason || '';
        const errMsg = formatErrorCode(errCode, (body && body.error && body.error.message) || `HTTP ${res.status}`);
        setResult('REJECTED', errMsg);
        refresh();
        return { ok: false, status: 'REJECTED', code: errCode, reason: errMsg };
      }

      const pollCtx = {
        timer: null,
        startedAt: Date.now(),
        requestId: body.request_id || clientRequestId,
        generation: body.generation,
        robotId: ctx.robotId,
        agentBootId: ctx.agentBootId,
        scope,
        requestedLinear: check.linear,
        requestedAngular: check.angular,
      };

      // POST 202 always returns PENDING (approved:false) — never trust APPLIED unless strictly matched
      state.serverView = body;
      if (body.status === 'APPLIED' && body.approved === true) {
        const m = verifyAppliedMatch(pollCtx, body, opts.getContext());
        if (m.ok) {
          syncManualCapIfApplied(body);
          setResult('APPLIED', classifySpeedStatus(body, opts.getContext()).text);
          refresh();
          return { ok: true, status: 'APPLIED', view: body };
        }
        setResult('REJECTED', `적용 응답 불일치로 APPLIED 미인정: ${m.reason}`);
        refresh();
        return { ok: false, status: 'REJECTED', reason: m.reason };
      }

      const cls = classifySpeedStatus(body, ctx);
      setResult(cls.status, cls.text);
      state.polling = pollCtx;
      refresh();

      return await new Promise((resolve) => {
        async function step() {
          if (state.destroyed || state.polling !== pollCtx) {
            resolve({ ok: false, status: state.lastResult.status, reason: state.lastResult.message });
            return;
          }
          const latest = await fetchServerView(pollCtx.robotId, true);
          if (state.destroyed || state.polling !== pollCtx) {
            resolve({ ok: false, status: state.lastResult.status, reason: state.lastResult.message });
            return;
          }
          const curCtx = opts.getContext() || {};
          if (latest) {
            const c = classifySpeedStatus(latest, curCtx);
            if (latest.stale === true) {
              stopPolling();
              setResult('STALE', c.text);
              refresh();
              resolve({ ok: false, status: 'STALE', view: latest });
              return;
            }
            if (String(latest.status || '').toUpperCase() === 'APPLIED' || latest.approved === true) {
              const m = verifyAppliedMatch(pollCtx, latest, curCtx);
              stopPolling();
              if (m.ok) {
                syncManualCapIfApplied(latest);
                setResult('APPLIED', c.text);
                refresh();
                resolve({ ok: true, status: 'APPLIED', view: latest });
              } else {
                setResult('REJECTED', `적용 응답 불일치로 APPLIED 미인정: ${m.reason}`);
                refresh();
                resolve({ ok: false, status: 'REJECTED', reason: m.reason, view: latest });
              }
              return;
            }
            if (c.status === 'REJECTED' || c.status === 'EXPIRED' || c.status === 'SUPERSEDED') {
              stopPolling();
              setResult(c.status, c.text);
              refresh();
              resolve({ ok: false, status: c.status, view: latest });
              return;
            }
          }
          if (Date.now() - pollCtx.startedAt >= pollTimeoutMs) {
            stopPolling();
            setResult('EXPIRED', `Agent 적용 확인 시간 초과 (${Math.round(pollTimeoutMs / 1000)}s) — 적용 상태 미확인.`);
            refresh();
            resolve({ ok: false, status: 'EXPIRED', reason: 'POLL_TIMEOUT' });
            return;
          }
          refresh();
          pollCtx.timer = setTimeout(step, pollIntervalMs);
        }
        pollCtx.timer = setTimeout(step, pollIntervalMs);
      });
    }

    q('manualApply').addEventListener('click', () => { submitScope('manual'); });
    q('autoApply').addEventListener('click', () => { submitScope('auto'); });
    refresh();

    return {
      refresh,
      syncFromServer: () => {
        const ctx = opts.getContext() || {};
        return fetchServerView(ctx.robotId, false).then((v) => { refresh(); return v; });
      },
      submitScope,
      getServerView: () => state.serverView,
      getLastResult: () => ({ ...state.lastResult }),
      destroy: () => {
        state.destroyed = true;
        stopPolling();
      },
    };
  }

  return {
    MIN_LINEAR_MPS,
    MIN_ANGULAR_RPS,
    ERROR_MESSAGES,
    formatErrorCode,
    extractScopeCeiling,
    validateSpeedInput,
    evaluateEnforcement,
    verifyAppliedMatch,
    evaluateSpeedGate,
    classifySpeedStatus,
    classifySafetyLimiting,
    mount,
  };
});
