'use strict';
/* Admin Gate reset card (2026-11-21). Shows the Safety Gate state, fault reasons, the Gate's OWN reset readiness, the server's
   blockers for the current user and the last request result. The button only sends a request: the server enforces the admin role,
   the Agent re-checks its evidence and the Gate decides last. Disabling the button is a convenience, not the security boundary.
   Pure helpers exported for Node tests. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.GateResetPanel = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  const SERVER = {
    ROLE_NOT_ADMIN: '관리자 권한 필요', LEASE_NOT_HELD_BY_CALLER: '내 Lease 필요', LEASE_EPOCH_MISMATCH: 'Lease epoch 불일치',
    AGENT_OFFLINE: 'Agent 오프라인', AGENT_NOT_RECONCILED: 'Agent 재동기화 필요', AGENT_GATE_RESET_UNSUPPORTED: 'Agent에 Gate Reset 기능 없음',
    OPERATING_MODE_UNAVAILABLE: '운용 모드 미확인', MISSION_ACTIVE: '진행 중 임무 있음', MANUAL_SESSION_ACTIVE: '수동 세션 진행 중',
    GATE_REPORT_STALE: 'Gate 보고 오래됨', GATE_STATUS_MISSING: 'Gate 상태 없음', GATE_NOT_FAULTED: 'Gate 정상(Reset 불필요)',
    GATE_CHANGED_SINCE_VIEWED: '화면 이후 Gate 상태 변경', FAULT_EPOCH_ALREADY_RESET: '이 장애는 이미 Reset됨 (상태 갱신 대기)', RESET_IN_PROGRESS: '다른 Reset 요청 처리 중',
  };
  const GATE = {
    GATE_INITIALIZING: 'Gate 초기화 중', NOTHING_TO_RESET: 'Reset할 장애 없음', INPUT_NO_WRITER: '입력 발행자 없음',
    INPUT_GID_UNKNOWN: '입력 발행자 GID 불명', INPUT_WRITER_NOT_ALLOWED: '허용되지 않은 입력 발행자', MOTION_COMMANDED: '속도 명령 수신 중',
    REJECTED_MOTION_RECENT: '거부된 속도 명령 최근 수신', INPUT_SILENT: '입력 발행자가 발행하지 않음', OUTPUT_NOT_ZERO: 'Gate 출력이 0이 아님', HEALTH_NOT_EVALUATED: 'Gate 상태 평가 전',
  };
  const STATUS = { QUEUED: '대기', DELIVERED: 'Agent 전달됨', SUCCEEDED: '성공', REFUSED: '거부', FAILED: '실패', EXPIRED: '만료',
    EXPIRED_NO_RESULT: '결과 없음(만료)', ABANDONED_SERVER_RESTART: '서버 재시작으로 폐기', ABANDONED_AGENT_RESTART: 'Agent 재시작으로 폐기' };
  // Retryability of each blocker (display only; the server and the Gate still decide). 'wait' = clears by itself, retry later;
  // 'action' = an operator action is needed first; 'none' = there is nothing to reset.
  const WAIT = ['MOTION_COMMANDED', 'REJECTED_MOTION_RECENT', 'OUTPUT_NOT_ZERO', 'INPUT_SILENT', 'HEALTH_NOT_EVALUATED', 'GATE_INITIALIZING',
    'WATCHDOG_NOT_AUTHORIZED', 'GATE_REPORT_STALE', 'RESET_IN_PROGRESS', 'INPUT_WRITERS', 'FAULT_EPOCH_ALREADY_RESET', 'INPUT_PEER_WRITERS'];
  const NONE = ['NOTHING_TO_RESET', 'GATE_NOT_FAULTED'];
  const kind = code => { const k = String(code || '').split(':')[0]; return NONE.includes(k) ? 'none' : (WAIT.includes(k) ? 'wait' : 'action'); };
  function retry(blockers) {
    const kinds = blockers.map(kind);
    if (!blockers.length) return { code: 'READY', text: '지금 요청 가능' };
    if (kinds.includes('none')) return { code: 'NONE', text: 'Reset 불필요 (Gate에 해제할 장애 없음)' };
    if (kinds.includes('action')) return { code: 'ACTION', text: '재시도 전 조치 필요 (아래 항목)' };
    return { code: 'WAIT', text: '조건이 스스로 해소되면 재시도 가능 (자동 Reset 없음)' };
  }
  const label = (map, code) => {
    const s = String(code || ''); const k = s.split(':')[0];
    if (k === 'INPUT_WRITERS') return `입력 발행자 ${s.split(':')[1]}개 (유령 endpoint 포함 가능)`;
    if (k === 'MODE_NOT_IDLE') return `운용 모드 ${s.split(':')[1]} (IDLE 필요)`;
    if (k === 'MAPPING_SESSION') return `맵핑 세션 ${s.split(':')[1]}`;
    if (k === 'WATCHDOG_NOT_AUTHORIZED') return `Watchdog 미승인 (${s.split(':')[1]})`;
    if (k === 'SENSOR_FAULT' || k === 'FAULT_PERSISTS') return `장애 지속: ${s.slice(k.length + 1)}`;
    return map[k] ? map[k] + (s.includes(':') && !map[s] ? ` (${s.slice(k.length + 1)})` : '') : s;
  };

  /** Everything the card shows, from the GET .../gate-reset response. No values are invented: missing data stays '미보고'. */
  function view(res, me) {
    const v = (res && res.gate && res.gate.view) || {};
    const rr = v.reset_readiness;
    const isAdmin = !!(me && me.role === 'admin');
    const serverBlockers = (res && res.server_blockers) || [];
    const gateBlockers = rr && Array.isArray(rr.blockers) ? rr.blockers : [];
    const gateReady = !!(rr && rr.enforced === true && rr.ready === true);
    const can = isAdmin && serverBlockers.length === 0 && gateReady;
    const req = res && res.request;
    const lr = v.last_reset;
    const all = serverBlockers.concat(gateBlockers);
    return {
      rows: [
        ['Gate 상태', v.state || '미보고'],
        ['현재 장애', (v.fault_reasons && v.fault_reasons.length) ? v.fault_reasons.join(', ') : (v.state ? '없음' : '미보고')],
        ['마지막 장애', (v.last_fault_reasons && v.last_fault_reasons.length) ? v.last_fault_reasons.join(', ') : '없음'],
        ['입력 발행자', v.cmd_input ? `${v.cmd_input.state || '?'}${v.cmd_input.sticky ? ' (Reset 필요)' : ''} · ${(v.cmd_input.publishers || []).join(', ') || '없음'}` : '미보고'],
        ['Gate 판단', rr ? (rr.enforced ? (gateReady ? 'Reset 허용' : 'Reset 거부') : 'Gate가 Reset 조건을 검사하지 않음 (지원 안 함)') : '미보고'],
        ['보고 경과', res && res.gate && typeof res.gate.report_age_s === 'number' ? `${res.gate.report_age_s}s` : '미보고'],
        ['Gate 최근 Reset 판단', lr ? `${lr.result === 'ACCEPTED' ? '수락' : '거부'}${(lr.blockers || []).length ? ' · ' + lr.blockers.map(c => label(GATE, c)).join(', ') : ''}` : '없음'],
        ['Gate Reset 거부 횟수', typeof v.reset_refusals === 'number' ? String(v.reset_refusals) : '미보고'],
      ],
      retry: retry(all), blockerKinds: all.map(c => ({ code: c, kind: kind(c) })),
      serverBlockers: serverBlockers.map(c => label(SERVER, c)),
      gateBlockers: gateBlockers.map(c => label(GATE, c)),
      supported: !!(rr && rr.enforced === true),
      can, isAdmin,
      request: req ? `${STATUS[req.status] || req.status}${req.reason ? ' · ' + req.reason : ''}${req.gate_message ? ' · Gate: ' + req.gate_message : ''}${(req.failed_checks || []).length ? ' · 실패 검사: ' + req.failed_checks.join(', ') : ''}` : '요청 없음',
      pending: !!(req && (req.status === 'QUEUED' || req.status === 'DELIVERED')),
    };
  }

  function mount(host, opts) {
    host.innerHTML = `<div class="sec-head"><h2>Safety Gate 복구 (관리자)</h2><span class="badge" data-gr="badge">확인 중</span></div>
      <dl class="mission-kv" data-gr="rows"></dl>
      <p class="small"><b>재시도</b> <span data-gr="retry"></span></p>
      <p class="small muted">Gate 조건: Reset 직전 1초 이상 속도 명령 0·출력 0, 입력 발행자 1개(Watchdog)가 발행 중, 다른 장애 없음. 조건이 맞지 않으면 Gate가 거부하며, 자동으로 다시 시도하지 않습니다.</p>
      <p class="small"><b>서버 판단</b></p><ul class="block-list small" data-gr="server"></ul>
      <p class="small"><b>Gate 판단</b></p><ul class="block-list small" data-gr="gate"></ul>
      <div class="toolbar-row small"><label>메모 <input data-gr="note" maxlength="200" placeholder="복구 사유"/></label>
        <button type="button" data-gr="reset">Gate Reset 요청</button></div>
      <p class="small" data-gr="result" role="status"></p><p class="small bad" data-gr="error" role="alert"></p>
      <p class="small muted">요청은 서버(관리자·Lease·IDLE·임무 없음)와 Agent(Watchdog·발행자 GID·DDS 그래프·속도 0)가 다시 검사하고 Gate가 최종 판단합니다. 성공 후에도 IDLE이 유지되며 기존 수동 세션·Goal은 재개되지 않습니다.</p>`;
    const q = n => host.querySelector(`[data-gr="${n}"]`);
    let last = null, error = '';
    const path = () => { const r = opts.getRobot(); return r ? `/robots/${encodeURIComponent(r.robot_id)}/gate-reset` : null; };
    async function load() {
      const p = path(); if (!p) { last = null; return; }
      try { const r = await opts.request(p, {}); last = r.ok ? await r.json() : null; } catch (e) { last = null; }
    }
    q('reset').addEventListener('click', async () => {
      error = '';
      const r = opts.getRobot(); const v = (last && last.gate && last.gate.view) || {};
      if (!window.confirm(`[관리자 확인] ${r && r.robot_id}의 Safety Gate 래치를 해제 요청합니다. 로봇은 IDLE을 유지합니다. 계속할까요?`)) return;
      try {
        const res = await opts.request(path(), { method: 'POST', body: JSON.stringify({ epoch: r && r.lease ? r.lease.epoch : null,
          expected_gate_boot_id: v.boot_id, expected_fault_epoch: v.fault_epoch, note: q('note').value }) });
        const j = await res.json().catch(() => ({}));
        if (!res.ok) error = (j.error && `${j.error.code}: ${j.error.message}`) || `HTTP ${res.status}`;
      } catch (e) { error = e.message; }
      await load(); render();
    });
    function render() {
      const v = view(last, opts.getMe());
      q('rows').innerHTML = v.rows.map(([k, val]) => `<div><dt>${esc(k)}</dt><dd>${esc(val)}</dd></div>`).join('');
      q('server').innerHTML = v.serverBlockers.length ? v.serverBlockers.map(b => `<li>${esc(b)}</li>`).join('') : '<li>서버 조건 충족</li>';
      q('gate').innerHTML = !v.supported ? '<li>Gate가 Reset 조건을 보고하지 않음 — 요청 불가</li>'
        : (v.gateBlockers.length ? v.gateBlockers.map(b => `<li>${esc(b)}</li>`).join('') : '<li>Gate 조건 충족</li>');
      q('retry').textContent = v.retry.text;
      q('reset').disabled = !v.can || v.pending;
      q('badge').textContent = v.pending ? '처리 중' : (v.can ? 'Reset 가능' : 'Reset 불가');
      q('badge').className = 'badge ' + (v.can ? 'ok' : 'warn');
      q('result').textContent = '최근 요청: ' + v.request; q('error').textContent = error;
    }
    const timer = setInterval(async () => { await load(); render(); }, 1000);
    load().then(render);
    return { render, destroy() { clearInterval(timer); } };
  }
  return { view, label, kind, retry, mount };
});
