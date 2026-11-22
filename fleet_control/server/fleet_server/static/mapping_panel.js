'use strict';
/* Web Mapping control (START / PAUSE / RESUME / SAVE / STOP) over POST /robots/{id}/mapping.
   HTTP 202 only means "queued"; the session state shown comes from GET /robots/{id}/mapping (worker evidence).
   Pure helpers are exported for Node tests. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.MappingPanel = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const STABLE = { START: ['IDLE', 'STOPPED'], PAUSE: ['MAPPING'], RESUME: ['PAUSED'], SAVE: ['PAUSED'], STOP: ['MAPPING', 'PAUSED'], RECONCILE: ['UNKNOWN'] };
  const LABEL = {
    IDLE: '대기', STARTING: '시작 중', MAPPING: '맵핑 중', PAUSING: '일시정지 중', PAUSED: '일시정지',
    SAVING: '저장 중', STOPPING: '종료 중', STOPPED: '종료됨', UNKNOWN: 'UNKNOWN · 상태 조정(RECONCILE) 필요',
    RECONCILING: '상태 조정 중',
  };

  function esc(s) {
    return String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  }

  /** Which buttons may be pressed. Server stays the authority; this only prevents obviously invalid requests. */
  function allowedActions(session, ctx) {
    const out = {};
    const st = (session && session.state) || 'UNKNOWN';
    const pending = session && session.command && ['QUEUED', 'ACCEPTED'].includes(session.command.status);
    for (const a of Object.keys(STABLE)) {
      let why = '';
      if (!ctx.canControl) why = '운영자 권한 필요';
      else if (!ctx.online) why = '로봇 오프라인';
      else if (!ctx.workerEnabled) why = ctx.realAgent
        ? 'Mapping Worker 없음 — 실차 Agent에서는 안전 조건 충족 전까지 차단됩니다(아래 차단 3곳)'
        : 'Mapping Worker 없음 — 이 Agent가 웹 맵핑(web_mapping_enabled)으로 실행되지 않았습니다';
      else if (!ctx.leaseOwned) why = 'Lease 미보유';
      else if (ctx.busy || pending) why = '이전 명령 처리 중';
      else if (!STABLE[a].includes(st)) why = `현재 상태 ${st}에서 불가`;
      else if (a === 'START' && ctx.missionActive) why = '이동 임무 종료 필요';
      else if ((a === 'START' || a === 'RESUME') && ctx.operatingMode && ctx.operatingMode !== 'MAPPING') why = `운용 모드가 ${ctx.operatingMode} — 먼저 MAPPING으로 전환`;
      out[a] = { enabled: !why, why };
    }
    return out;
  }

  /** Group the disabled buttons by reason so the operator reads each reason once (visible text, not only a tooltip). */
  function groupReasons(allowed) {
    const groups = [];
    for (const [a, v] of Object.entries(allowed)) {
      if (v.enabled) continue;
      const g = groups.find((x) => x.why === v.why);
      if (g) g.actions.push(a); else groups.push({ why: v.why, actions: [a] });
    }
    return groups;
  }

  /** The three places that keep real-robot mapping blocked, as visible, factual statements (null for SIM/mock agents).
      They are NOT bypassed by the UI: enabling a button here would still be refused by the Agent and the server. */
  function realRobotBlocks(robot) {
    const a = robot && robot.agent;
    if (!a || !(a.mode === 'real' || a.simulated === false)) return null;
    return [
      { id: 'ui', text: `웹: 이 Agent는 Mapping Worker를 실행하지 않아 버튼이 비활성입니다 (agent.mapping_worker=${a.mapping_worker === true})`, active: a.mapping_worker !== true },
      { id: 'agent', text: 'Agent: 웹 맵핑 Worker는 SIM/mock · 격리 ROS(localhost) · 모터 잠금에서만 시작되며 실차 Agent는 시작 단계에서 거부합니다', active: true },
      { id: 'server', text: `서버: 실차 Agent(mode=${a.mode ?? '?'}, simulated=${a.simulated ?? '?'})의 맵핑 명령은 mapping_real_blocked로 거부합니다`, active: true },
    ];
  }

  /** Command lifecycle as the operator must read it: HTTP acceptance is not execution, execution is not success.
      `local` = {sending, httpError} for the request this browser just sent; `nowS` = browser epoch seconds (approximate). */
  function commandPhase(session, nowS, local) {
    if (local && local.sending) return { phase: 'sending', tone: 'muted', label: '전송 중', detail: '요청을 서버로 보내는 중' };
    if (local && local.httpError) return { phase: 'rejected', tone: 'bad', label: '서버가 거부함', detail: local.httpError };
    const c = session && session.command;
    if (!c) return { phase: 'none', tone: 'muted', label: '명령 없음', detail: '' };
    const remain = (typeof c.expires_at === 'number' && typeof nowS === 'number') ? Math.max(0, Math.round(c.expires_at - nowS)) : null;
    const wait = remain != null ? ` · 약 ${remain}초 안에 진행되지 않으면 타임아웃` : '';
    switch (c.status) {
      case 'QUEUED': return { phase: 'accepted', tone: 'warn', label: '접수됨 · Worker 수락 대기', detail: `${c.action}${wait}` };
      case 'ACCEPTED': return { phase: 'running', tone: 'warn', label: '실행 중', detail: `${c.action}${wait}` };
      case 'COMPLETED': return { phase: 'succeeded', tone: 'ok', label: '성공', detail: `${c.action} · Worker 증거로 확정` };
      case 'FAILED': return { phase: 'failed', tone: 'bad', label: '실패', detail: `${c.action}${session.detail ? ' · ' + session.detail : ''}` };
      case 'UNKNOWN': return { phase: 'timeout', tone: 'bad', label: '타임아웃 · 결과 미확인', detail: `${c.action} · Worker 증거가 없어 UNKNOWN. 자동 재개 없음, 현장 확인 필요` };
      default: return { phase: 'unknown', tone: 'bad', label: String(c.status || 'UNKNOWN'), detail: c.action || '' };
    }
  }

  const PHASE_ORDER = { sending: 0, accepted: 1, running: 2, succeeded: 3, failed: 3, timeout: 3, rejected: 3 };
  function phaseHtml(p) {
    if (!p || p.phase === 'none') return '<span class="muted">명령 없음</span>';
    const idx = PHASE_ORDER[p.phase];
    const end = ['succeeded', 'failed', 'timeout', 'rejected'].includes(p.phase) ? p.label : '결과';
    const steps = [['전송', 0], ['접수', 1], ['실행 중', 2], [end, 3]];
    return steps.map(([t, i]) => `<span class="step${i < idx ? ' past' : ''}${i === idx ? ' cur ' + p.tone : ''}">${esc(t)}</span>`).join('<span class="arrow">→</span>')
      + `<span class="small ${p.tone === 'ok' ? '' : p.tone}" style="margin-left:6px">${esc(p.label)}${p.detail ? ' — ' + esc(p.detail) : ''}</span>`;
  }

  function summary(session) {
    if (!session) return '상태 미수신';
    const st = session.state || 'UNKNOWN';
    const c = session.command;
    const parts = [`${LABEL[st] || st}`];
    if (session.session_id) parts.push(`세션 ${String(session.session_id).slice(0, 8)}`);
    if (c) parts.push(`마지막 명령 ${c.action} → ${c.status}`);
    if (session.detail) parts.push(session.detail);
    return parts.join(' · ');
  }

  function evidenceText(session) {
    const r = session && session.result;
    if (!r) return '';
    const ev = Object.entries(r.evidence || {}).map(([k, v]) => `${k}=${v ? 'O' : 'X'}`).join(', ');
    return `${r.action} 결과 ${r.status}${ev ? ` (${ev})` : ''}`;
  }

  /** START body options from the form: mode, parent (CONTINUE), termination limits, optional geofence. */
  function startOptions(sel, read) {
    const v = read || ((id) => { const e = document.getElementById(id); return e ? e.value : ''; });
    const opts = { mode: v('webMapMode') || 'NEW_MAP', limits: {} };
    if (opts.mode === 'CONTINUE_MAP') {
      if (!sel.mapVersionId || sel.mapVersionId === '__live__') throw Error('CONTINUE MAP: 지도 버전 선택에서 부모 저장 지도를 먼저 고르세요');
      opts.parent_map_version_id = sel.mapVersionId;
    }
    if (v('webMapMaxGoals') !== '') opts.limits.max_goals = Number(v('webMapMaxGoals'));
    if (v('webMapMaxDur') !== '') opts.limits.max_duration_s = Number(v('webMapMaxDur'));
    if (v('webMapMinV') !== '') opts.limits.min_battery_v = Number(v('webMapMinV'));
    const fence = (v('webMapFence') || '').trim();
    if (fence) {
      let poly;
      try { poly = JSON.parse(fence); } catch (_) { throw Error('Geofence JSON 형식 오류'); }
      if (!Array.isArray(poly) || poly.length < 3) throw Error('Geofence는 3개 이상의 [x,y] 점');
      opts.geofence = poly;
    }
    return opts;
  }

  let deps = null;
  let session = null;
  let busy = false;
  let lastErr = '';
  let lastRejected = '';

  function el(id) { return document.getElementById(id); }

  function init(d) {
    deps = d;
    for (const a of Object.keys(STABLE)) {
      const b = el('webMap' + a);
      if (b) b.addEventListener('click', () => send(a));
    }
  }

  async function refresh() {
    if (!deps) return;
    const robot = deps.getRobot();
    if (!robot) { session = null; return render(); }
    try {
      const r = await deps.api(`/robots/${encodeURIComponent(robot.robot_id)}/mapping`);
      session = r.ok ? await r.json() : null;
    } catch (_) {
      session = null;
    }
    await keepLease(robot);
    render();
  }

  /** While this tab watches a live session, renew the operator's Lease before it expires (browser heartbeat).
      Closing the tab or losing the network lets the Lease expire -> the server marks the session UNKNOWN. */
  function needsLeaseRenewal(sess, lease, me) {
    return !!(sess && ['STARTING', 'MAPPING', 'PAUSING', 'PAUSED', 'SAVING', 'STOPPING'].includes(sess.state)
      && lease && me && lease.owner === me.name && lease.epoch === sess.epoch && typeof lease.remaining_s === 'number' && lease.remaining_s < 15);
  }
  async function keepLease(robot) {
    if (!needsLeaseRenewal(session, robot.lease, deps.getMe())) return;
    try {
      await deps.api(`/robots/${encodeURIComponent(robot.robot_id)}/lock`, { method: 'POST', body: JSON.stringify({ action: 'renew', ttl_s: robot.lease.ttl_s || 60 }) });
    } catch (_) { /* expiry is the fail-closed outcome */ }
  }

  async function send(action) {
    const robot = deps && deps.getRobot();
    if (!robot || busy) return;
    const body = { action, epoch: robot.lease ? robot.lease.epoch : null, idempotency_key: 'web-' + action + '-' + Date.now() + '-' + Math.random().toString(16).slice(2, 8) };
    if (action !== 'START') body.session_id = session && session.session_id;
    if (action === 'START') {
      try { Object.assign(body, startOptions(deps.getSelection())); } catch (e) { lastErr = e.message; render(); return; }
    }
    if (action === 'SAVE') {
      const sel = deps.getSelection();
      if (sel.floorId) { body.floor_id = sel.floorId; if (sel.areaId) body.area_id = sel.areaId; }
    }
    if (action === 'RECONCILE' && !window.confirm('UNKNOWN 맵핑 세션을 상태 조정합니다. Worker가 자신이 띄운 SLAM·Frontier 종료, 그래프 정리, 활성 Goal 없음을 증명할 때만 STOPPED가 되며 자동 재개는 없습니다. 계속할까요?')) return;
    if (action === 'STOP' && !window.confirm('자동 맵핑을 종료합니다. 저장하지 않은 지도는 Worker 세션 폴더에만 남습니다. 계속할까요?')) return;
    busy = true; lastErr = ''; lastRejected = ''; render();
    try {
      const r = await deps.api(`/robots/${encodeURIComponent(robot.robot_id)}/mapping`, { method: 'POST', body: JSON.stringify(body) });
      const j = await r.json().catch(() => ({}));
      if (!r.ok) throw Error((j.error && `${j.error.code}: ${j.error.message}`) || `HTTP ${r.status}`);
      lastErr = '';
    } catch (e) {
      lastErr = e.message;
      lastRejected = e.message;
    } finally {
      busy = false;
      await refresh();
    }
  }

  function render() {
    if (!deps || !el('webMapStatus')) return;
    const robot = deps.getRobot();
    const me = deps.getMe();
    const ctx = {
      canControl: !!me && (me.role === 'operator' || me.role === 'admin'),
      online: !!(robot && robot.online),
      workerEnabled: !!(robot && robot.agent && robot.agent.mapping_worker === true),
      realAgent: !!(robot && robot.agent && (robot.agent.mode === 'real' || robot.agent.simulated === false)),
      leaseOwned: !!(robot && robot.lease && me && robot.lease.owner === me.name),
      busy,
      missionActive: !!(robot && robot.state && robot.state.execution && robot.state.execution.active),
      operatingMode: robot && robot.operating_mode ? robot.operating_mode.mode : null,
    };
    const allowed = allowedActions(session, ctx);
    for (const [a, v] of Object.entries(allowed)) {
      const b = el('webMap' + a);
      if (b) { b.disabled = !v.enabled; b.title = v.why || ''; }
    }
    el('webMapStatus').textContent = robot ? `${robot.robot_id} · ${summary(session)}` : '로봇 미선택';
    el('webMapStatus').className = session && session.state === 'UNKNOWN' ? 'bad' : '';
    el('webMapEvidence').textContent = evidenceText(session);
    const sel = deps.getSelection();
    const reg = session && session.registered;
    el('webMapSave').textContent = reg
      ? `등록됨: ${reg.map_version_id} (UNVERIFIED)${reg.area_id ? ' · 영역 ' + reg.area_id : ''}`
      : (sel.floorId ? `SAVE 등록 대상: ${sel.floorId}${sel.areaId ? ' / ' + sel.areaId : ''}` : 'SAVE 등록 대상 없음(로봇 세션 폴더에만 저장)');
    const w = robot && robot.state && robot.state.mapping_worker;
    el('webMapWorker').textContent = w ? `Worker ${w.status}${w.mode ? ' · ' + w.mode : ''}${w.frontier_candidates != null ? ` · 후보 ${w.frontier_candidates}` : ''}${w.goal ? ` · 목표 ${w.goal.status}` : ''}${w.geofence ? ` · Geofence 제외 ${w.geofence_rejected}` : ''}${w.stop_reason ? ` · 목표 제공 중단: ${w.stop_reason} (완료 아님)` : ''}${w.tools_error ? ' · ' + w.tools_error : ''}` : 'Worker 상태 미수신';
    el('webMapError').textContent = lastErr;
    const blocks = el('webMapBlocks');
    if (blocks) {
      const items = groupReasons(allowed).map((g) => `<li><b>${esc(g.actions.join(' · '))}</b> 비활성 — ${esc(g.why)}</li>`);
      const enabled = Object.entries(allowed).filter(([, v]) => v.enabled).map(([a]) => a);
      if (enabled.length) items.push(`<li class="ok">지금 누를 수 있는 명령: <b>${esc(enabled.join(' · '))}</b></li>`);
      const real = realRobotBlocks(robot);
      if (real) {
        items.push('<li class="bad"><b>실차 맵핑 차단 3곳</b> (UI에서 해제하지 않으며, Gate·Watchdog 적용과 실차 안전 측정이 끝나기 전에는 열지 않습니다)<ul>'
          + real.map((r) => `<li>${esc(r.text)}</li>`).join('') + '</ul></li>');
      }
      blocks.innerHTML = items.join('');
    }
    const ph = el('webMapPhase');
    if (ph) ph.innerHTML = phaseHtml(commandPhase(session, Date.now() / 1000, { sending: busy, httpError: lastRejected }));
  }

  function currentState() { return session ? session.state : null; }

  return { allowedActions, groupReasons, realRobotBlocks, commandPhase, phaseHtml, needsLeaseRenewal, summary, evidenceText, startOptions, init, refresh, render, esc, currentState };
});
