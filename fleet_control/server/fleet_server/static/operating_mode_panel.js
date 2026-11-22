'use strict';
/* Operating mode card: IDLE / MANUAL / MAPPING / NAVIGATION over POST /robots/{id}/mode.
   HTTP 202 only queues a transition; the steps shown come from the server's mode record (stop previous ->
   verify idle -> Agent confirmation -> ACTIVE | FAILED). Status rows are read from the robot's last report and
   are shown as missing when not reported (never filled with assumed values). Pure helpers exported for Node tests. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.OperatingModePanel = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const MODES = ['IDLE', 'MANUAL', 'MAPPING', 'NAVIGATION'];
  const LABEL = { IDLE: '대기 IDLE', MANUAL: '수동 MANUAL', MAPPING: '맵핑 MAPPING', NAVIGATION: '자율주행 NAVIGATION',
    TRANSITIONING: '전환 중', FAULT: 'FAULT · 이동 차단', UNKNOWN: 'UNKNOWN' };
  const STEPS = [['STOPPING_PREVIOUS', '이전 작업 종료'], ['VERIFYING_IDLE', '정지·Goal 종료 검증'], ['CONFIRMING', 'Agent 확인'], ['ACTIVE', '완료']];
  const PENDING = ['STOPPING_PREVIOUS', 'VERIFYING_IDLE', 'CONFIRMING'];
  const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  const short = u => typeof u === 'string' ? u.slice(0, 8) : '';

  function buttons(robot, me) {
    const rec = robot && robot.operating_mode;
    const t = rec && rec.transition;
    const agent = (robot && robot.agent) || {};
    const leaseOwned = !!(robot && robot.lease && me && robot.lease.owner === me.name);
    const out = {};
    for (const m of MODES) {
      let why = '';
      if (!me || !['operator', 'admin'].includes(me.role)) why = '운영자 권한 필요';
      else if (!robot || !robot.online) why = '로봇 오프라인';
      else if (!leaseOwned) why = 'Lease 미보유 — 먼저 제어권 획득';
      else if (t && PENDING.includes(t.state)) why = `전환 진행 중 (${t.from} → ${t.to})`;
      else if (rec && rec.mode === m && !(t && t.state === 'FAILED' && t.to === m) && m !== 'IDLE') why = '현재 모드';
      else if (m === 'MANUAL' && agent.manual_web_bench !== true) why = 'Agent에 수동 입력 경로 없음';
      else if (m === 'MAPPING' && agent.mapping_worker !== true) why = 'Agent에 Mapping Worker 없음';
      else if (m === 'MAPPING' && (agent.mode === 'real' || agent.simulated === false)) why = '실차 맵핑 차단 유지';
      else if (m === 'NAVIGATION' && agent.missions_enabled !== true) why = 'Agent 임무 비활성';
      out[m] = { enabled: !why, why };
    }
    return out;
  }

  function stepsHtml(t) {
    if (!t) return '<span class="muted">전환 기록 없음</span>';
    const idx = STEPS.findIndex(([k]) => k === t.state);
    const failed = t.state === 'FAILED';
    const parts = STEPS.map(([k, label], i) => {
      const cls = failed ? (i === 0 ? 'step cur bad' : 'step') : i < idx ? 'step past' : i === idx ? 'step cur ' + (k === 'ACTIVE' ? 'ok' : 'warn') : 'step';
      return `<span class="${cls}">${esc(label)}</span>`;
    }).join('<span class="arrow">→</span>');
    const why = failed ? `실패: ${t.reason || '사유 없음'} — 자동 재시도 없음` : (t.blocker ? `대기 사유: ${t.blocker}` : '');
    return `<div class="small"><b>${esc(t.from)} → ${esc(t.to)}</b> · ${esc(failed ? 'FAILED' : t.state)}</div><div class="phase">${parts}</div>`
      + (why ? `<div class="small ${failed ? 'bad' : 'warn'}">${esc(why)}</div>` : '');
  }

  function rows(robot) {
    const s = (robot && robot.state) || {};
    const ex = s.execution || {};
    const active = ex.active;
    const goals = ex.nav2_goals || {};
    const idle = ex.nav_idle || {};
    const arb = s.velocity_arbiter;
    const wd = s.motion_watchdog || {};
    const gate = s.safety_gate || {};
    const worker = s.mapping_worker;
    const lease = robot && robot.lease;
    const out = [];
    out.push(['Lease', lease ? `${lease.owner} · epoch ${lease.epoch} · ${lease.remaining_s}s 남음` : '없음']);
    out.push(['활성 Nav2 Goal', active ? `${active.owner || 'fleet'} · ${active.mission_id || ''} · goal ${short(active.goal_uuid)}${active.cancel_requested ? ' · 취소 요청됨' : ''}`
      : (goals.active_count ? `Agent 소유 아님 ${goals.active_count}개 (외부 Goal)` : '없음')]);
    out.push(['Nav2 idle 증거', idle.ok === true ? 'OK (서버 유일·GID 연속·비종료 Goal 없음)' : (idle.reason ? `없음 · ${idle.reason}` : '미보고')]);
    out.push(['Mapping Worker', worker && worker.enabled ? `${worker.status}${worker.goal ? ' · 목표 ' + worker.goal.status : ''}${worker.processes_started ? ' · SLAM/Frontier 실행 중' : ''}` : '없음']);
    out.push(['속도 출처', arb ? `${arb.source || 'NONE'} (${Number(arb.selected_linear || 0).toFixed(2)} m/s, ${Number(arb.selected_turn || 0).toFixed(2)} rad/s) · Agent 인식 모드 ${arb.operating_mode ?? '없음'}` : 'Arbiter 미보고']);
    out.push(['Manual Deadman', arb ? (arb.deadman ? '유지 중' : '해제') : '미보고']);
    out.push(['차단 사유', arb && arb.blocker ? arb.blocker : (arb ? '없음' : '미보고')]);
    out.push(['Watchdog', wd.state ? `${wd.state}${wd.last_fault ? ' · 마지막 고장 ' + wd.last_fault : ''}` : '미보고']);
    out.push(['Safety Gate', gate.status ? `${gate.status}${gate.cmd_blocked ? ' · 명령 차단' : ''}` : '미보고']);
    return out;
  }

  function view(robot, me) {
    const rec = robot && robot.operating_mode;
    const mode = rec ? rec.mode : null;
    const badge = mode ? (LABEL[mode] || mode) : `모드 미선택 (기존 방식) · ${robot && robot.control_mode ? robot.control_mode : 'UNKNOWN'}`;
    const tone = mode === 'FAULT' ? 'bad' : mode === 'TRANSITIONING' ? 'warn' : 'ok';
    return { badge, tone, reason: rec ? rec.reason : '', steps: stepsHtml(rec && rec.transition), rows: rows(robot), buttons: buttons(robot, me) };
  }

  function mount(host, opts) {
    host.innerHTML = `<div class="sec-head"><h2>운용 모드</h2><span class="badge" data-om="badge">확인 중</span></div>
      <p class="small muted" data-om="reason"></p>
      <div class="toolbar-row">${MODES.map(m => `<button type="button" data-om-mode="${m}">${esc(LABEL[m])}</button>`).join(' ')}</div>
      <div data-om="steps" role="status" aria-live="polite"></div><p class="small bad" data-om="error" role="alert"></p>
      <dl class="mission-kv" data-om="rows"></dl><ul class="block-list small" data-om="why"></ul>
      <p class="small muted">HTTP 202는 접수만 의미합니다. 이전 작업 종료, 정지·Goal 종료 검증, Agent 확인을 모두 통과해야 새 모드에서만 명령이 허용됩니다. 실패하면 자동 재출발하지 않습니다.</p>`;
    const q = n => host.querySelector(`[data-om="${n}"]`);
    let busy = false, error = '';
    function render() {
      const robot = opts.getRobot(), me = opts.getMe();
      const v = view(robot, me);
      q('badge').textContent = v.badge; q('badge').className = 'badge ' + v.tone;
      q('reason').textContent = v.reason || '';
      q('steps').innerHTML = v.steps;
      q('rows').innerHTML = v.rows.map(([k, val]) => `<div><dt>${esc(k)}</dt><dd>${esc(val)}</dd></div>`).join('');
      q('error').textContent = error;
      const why = [];
      for (const [m, b] of Object.entries(v.buttons)) {
        const el = host.querySelector(`[data-om-mode="${m}"]`);
        el.disabled = busy || !b.enabled; el.title = b.why;
        if (!b.enabled && b.why !== '현재 모드') why.push(`<li><b>${esc(m)}</b> — ${esc(b.why)}</li>`);
      }
      q('why').innerHTML = why.join('');
    }
    async function request(target) {
      const robot = opts.getRobot();
      if (!robot || busy) return;
      busy = true; error = ''; render();
      try {
        const r = await opts.request(`/robots/${encodeURIComponent(robot.robot_id)}/mode`, { method: 'POST',
          body: JSON.stringify({ target, epoch: robot.lease ? robot.lease.epoch : null, idempotency_key: 'web-mode-' + Date.now() + '-' + Math.random().toString(16).slice(2, 8) }) });
        const j = await r.json().catch(() => ({}));
        if (!r.ok) throw Error((j.error && `${j.error.code}: ${j.error.message}`) || `HTTP ${r.status}`);
      } catch (e) { error = '전환 요청 거부: ' + e.message; }
      finally { busy = false; if (opts.refresh) await opts.refresh(); render(); }
    }
    for (const m of MODES) host.querySelector(`[data-om-mode="${m}"]`).addEventListener('click', () => request(m));
    const timer = setInterval(render, 500); render();
    return { render, destroy() { clearInterval(timer); } };
  }

  return { MODES, view, buttons, rows, stepsHtml, mount };
});
