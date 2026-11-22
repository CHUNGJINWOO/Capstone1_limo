'use strict';
/* Phase 4 (2026-11-21): robot readiness and services card for the operator dashboard.
   READY / DEGRADED / FAULT with every reason, the supervisor's services (state, restarts, last exit) and recent service actions.
   Admins get Start / Stop / Restart / Retry for the actions the ROBOT's own config allows; the server re-checks the role and the
   robot state, and the supervisor re-checks its local allowlist. Nothing here moves the robot or starts mapping / navigation.
   Pure helpers exported for Node tests. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.SystemPanel = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  const LEVEL = { READY: ['준비 완료', 'ok'], DEGRADED: ['일부 준비 안 됨', 'warn'], FAULT: ['장애', 'bad'] };
  const REASON = {
    SUPERVISOR_NOT_REPORTING: '로봇 감독 프로세스 보고 없음', SUPERVISOR_STALE: '로봇 감독 프로세스 보고 끊김', AGENT_OFFLINE: 'Agent 오프라인',
    AGENT_NOT_RECONCILED: 'Agent 재동기화 중', AGENT_STATE_STALE: 'Agent 상태 오래됨', RECONCILIATION_REQUIRED: '임무 상태 조정 필요(관리자 복구)',
    GATE_BOOT_RECONCILIATION: 'Gate 재시작 확인 필요(관리자 복구)', SERVICE_FAILED: '서비스 실패(재시도 필요)', SERVICE_STOPPED: '서비스 정지됨',
    SERVICE_WAITING: '서비스 대기', SERVICE_STARTING: '서비스 시작 중', SERVICE_RUNNING: '서비스 준비 확인 중', SERVICE_BACKOFF: '서비스 재시작 대기',
    GATE_FAULT: 'Safety Gate 장애 래치', GATE_NOT_NORMAL: 'Safety Gate 준비 안 됨', WATCHDOG: 'Motion Watchdog', GATE: 'Safety Gate 보고', SERVICE_MISSING: '필수 서비스 보고 없음', SERVICE_DUPLICATE: '서비스 이름 중복', SERVICE_STATE_INVALID: '서비스 상태 형식 오류', INVALID_AGE: '데이터 시각 형식 오류', TOPIC_INVALID: '토픽 보고 형식 오류', NO_DATA: '데이터 없음', STALE: '데이터 끊김', NAV2: 'Nav2 준비 안 됨(관리자: stack 재시작)',
  };
  const SVC = { WAITING: '대기', STARTING: '시작 중', RUNNING: '확인 중', READY: '준비', BACKOFF: '재시작 대기', FAILED: '실패', STOPPED: '정지', STOPPING: '정지 중' };
  function reasonText(code) {
    const s = String(code || ''); const i = s.indexOf(':'); const k = i < 0 ? s : s.slice(0, i); const arg = i < 0 ? '' : s.slice(i + 1);
    return (REASON[k] || k) + (arg ? ` · ${arg}` : '');
  }
  // The server requires the admin to hold this robot's live lease and checks the epoch; null -> refused with lease_required.
  function leaseEpoch(robot, me) { const l = robot && robot.lease; return l && me && l.owner === me.name && Number.isInteger(l.epoch) ? l.epoch : null; }
  function levelView(rd) { const l = rd && LEVEL[rd.level]; return l ? { text: l[0], tone: l[1], code: rd.level } : { text: '확인 불가', tone: 'bad', code: 'UNKNOWN' }; }
  function mount(host, opts) {
    host.innerHTML = `<div class="sec-head"><h2>로봇 준비 상태 · 서비스</h2><span class="badge" data-sp="level">확인 중</span></div>
      <p class="small muted">서비스 제어는 이 로봇의 Lease를 보유한 관리자만 가능합니다. 자동 기동은 모터 구동 허가가 아닙니다. MAPPING/NAVIGATION은 READY일 때만 서버가 허용합니다.</p>
      <ul class="block-list small" data-sp="reasons"></ul>
      <table class="small"><thead><tr><th>서비스</th><th>상태</th><th>재시작</th><th>마지막 종료</th><th></th></tr></thead><tbody data-sp="svc"></tbody></table>
      <p class="small" data-sp="result" role="status"></p><p class="small muted" data-sp="meta"></p>`;
    const q = n => host.querySelector(`[data-sp="${n}"]`);
    let busy = false, last = null;
    async function load() {
      const robot = opts.getRobot(); if (!robot) return;
      try {
        const r = await opts.request(`/robots/${encodeURIComponent(robot.robot_id)}/readiness`);
        last = await r.json(); render();
      } catch (_) { q('level').textContent = '조회 실패'; q('level').className = 'badge bad'; }
    }
    function render() {
      const rd = last || {}; const lv = levelView(rd); const me = opts.getMe(); const admin = me && me.role === 'admin';
      const mp = rd.level === 'READY' ? (rd.motion_permitted ? ' · 주행 허가 상태' : ` · 주행 차단: ${(rd.motion_block_reasons || []).join(', ')}`) : '';
      q('level').textContent = `${lv.code} · ${lv.text}${mp}`; q('level').className = `badge ${lv.tone}`; q('level').dataset.level = lv.code;
      q('reasons').innerHTML = (rd.reasons || []).map(c => `<li data-code="${esc(c)}">${esc(reasonText(c))}</li>`).join('') || '<li class="muted">차단 원인 없음</li>';
      q('svc').innerHTML = (rd.services || []).map(s => {
        const ex = s.last_exit ? `코드 ${esc(s.last_exit.code)}` : '-';
        const btns = admin ? (s.actions || []).map(a => `<button type="button" data-act="${esc(a)}" data-svc="${esc(s.name)}"${busy ? ' disabled' : ''}>${esc({ start: '시작', stop: '정지', restart: '재시작', retry: '재시도' }[a] || a)}</button>`).join(' ') : '';
        return `<tr data-service="${esc(s.name)}" data-state="${esc(s.state)}"><td>${esc(s.name)}</td><td title="${esc(s.detail || '')}">${esc(SVC[s.state] || s.state)}</td><td>${esc(s.restarts ?? 0)}</td><td>${ex}</td><td>${btns}</td></tr>`;
      }).join('') || '<tr><td colspan="5" class="muted">감독 프로세스 보고 없음</td></tr>';
      const a = (rd.actions || []).slice(-1)[0];
      q('meta').textContent = rd.supervisor ? `감독 boot ${rd.supervisor.boot_id} · 보고 ${rd.supervisor.age_s} s 전${a ? ` · 마지막 요청 ${a.service} ${a.action}: ${a.status}` : ''}` : '';
    }
    host.addEventListener('click', async ev => {
      const b = ev.target.closest('button[data-act]'); if (!b || busy) return;
      const robot = opts.getRobot(); if (!robot) return;
      if (['stop', 'restart'].includes(b.dataset.act) && !window.confirm(`${robot.robot_id}/${b.dataset.svc} ${b.dataset.act}? 실행 중 작업은 먼저 IDLE로 전환해야 합니다.`)) return;
      busy = true; render(); q('result').textContent = '요청 중';
      try {
        const r = await opts.request(`/robots/${encodeURIComponent(robot.robot_id)}/services`, { method: 'POST', body: JSON.stringify({ service: b.dataset.svc, action: b.dataset.act, epoch: leaseEpoch(robot, opts.getMe()) }) });
        const j = await r.json(); q('result').textContent = r.ok ? `접수: ${j.service} ${j.action} (${j.action_id})` : `거부: ${j.error?.message || r.status}`;
      } catch (e) { q('result').textContent = '요청 실패'; }
      finally { busy = false; await load(); }
    });
    const timer = setInterval(load, 1500); load();
    return { refresh: load, destroy() { clearInterval(timer); } };
  }
  return { mount, reasonText, levelView, leaseEpoch };
});
