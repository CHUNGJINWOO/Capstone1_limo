'use strict';
/* Saved-map navigation card (NAVIGATION mode): Registry map version -> LOAD -> initial pose -> localization -> goal check.
   HTTP 202 only queues a request; map loading and localization results come from the Agent report (localization block).
   Missions are still submitted with the existing goal drawer; the server re-checks mode, localization and the goal cell.
   Pure helpers exported for Node tests. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.NavigationPanel = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  const LOC = { IDLE: '대기', LOADING: '지도 다운로드·map_server/AMCL 기동 중', WAIT_INITIAL_POSE: '초기 위치 지정 필요',
    CONVERGING: '위치 추정 수렴 중', CONFIRMED: '위치 확인됨 (AMCL + 스캔/지도 일치)', DEGRADED: '위치 신뢰도 저하 · 이동 차단',
    FAILED: '실패', UNAVAILABLE: 'Agent에 Localization Worker 없음' };
  const GOAL = { OUTSIDE_MAP: '지도 밖', OCCUPIED: '벽·장애물 셀', UNKNOWN_AREA: '미지 영역', FOOTPRINT_OCCUPIED: '로봇 반경 안에 장애물',
    FOOTPRINT_UNKNOWN: '로봇 반경 안에 미지 영역', FOOTPRINT_OUTSIDE_MAP: '로봇 반경이 지도 밖', NO_MAP_LOADED: '지도 미로딩', NOT_FINITE: '좌표 오류' };

  /** Which step the operator is on and why the next one is blocked (server stays the authority). */
  function steps(robot, me) {
    const om = robot && robot.operating_mode, nav = robot && robot.navigation;
    const loc = (robot && robot.state && robot.state.localization) || {};
    const leaseOwned = !!(robot && robot.lease && me && robot.lease.owner === me.name);
    const out = { load: '', pose: '', goal: '' };
    if (!me || !['operator', 'admin'].includes(me.role)) out.load = '운영자 권한 필요';
    else if (!leaseOwned) out.load = 'Lease 미보유';
    else if (!om || om.mode !== 'NAVIGATION' || (om.transition && om.transition.state !== 'ACTIVE' && om.transition.state !== 'FAILED'))
      out.load = '운용 모드를 NAVIGATION으로 전환 필요';
    else if (!robot.agent || robot.agent.navigation_localization !== true) out.load = 'Agent에 저장 지도 Localization 없음';
    out.pose = out.load || (!nav ? '먼저 지도를 로딩' : (loc.status === 'LOADING' ? '지도 로딩 중' : (loc.status === 'FAILED' ? '로딩 실패 · 다시 로딩' : '')));
    out.goal = out.pose || (!(nav && nav.localized) ? '위치 확인(CONFIRMED) 후 목표 가능' : (nav.map_verified ? '' : 'UNVERIFIED 지도 · 임무 불가'));
    return out;
  }

  function view(robot, me) {
    const nav = robot && robot.navigation;
    const loc = (robot && robot.state && robot.state.localization) || {};
    const a = loc.amcl_pose;
    const rows = [
      ['지도', nav ? `${nav.map_version_id} · ${nav.verification_status}${nav.map_verified ? '' : ' (주행 불가)'}` : '미로딩'],
      ['위치 추정', `${LOC[loc.status] || loc.status || '미보고'}${loc.detail ? ' · ' + loc.detail : ''}`],
      ['초기 위치', nav && nav.initial_pose ? `(${nav.initial_pose.x.toFixed(2)}, ${nav.initial_pose.y.toFixed(2)}, ${nav.initial_pose.yaw.toFixed(2)}) #${nav.initial_pose.seq}` : '미지정'],
      ['스캔/지도 일치', loc.match_ratio == null ? '미보고' : `${Math.round(loc.match_ratio * 100)}% (확인 기준 60%)`],
      ['AMCL', a ? `(${a.x}, ${a.y}, ${a.yaw}) 분산 xx ${a.cov_xx} · yy ${a.cov_yy} · yaw ${a.cov_yaw}` : '미보고'],
      ['계획기 미지 영역', loc.planner_allow_unknown === false ? '불허 (저장 지도 순찰)' : loc.planner_allow_unknown === true ? '허용' : '미보고'],
      ['Nav2 경로', loc.nav_path ? `${loc.nav_path.points.length}점 · ${loc.nav_path.age_s}s 전` : '없음'],
    ];
    return { rows, blocked: steps(robot, me) };
  }

  function goalText(r) {
    if (!r) return '';
    return r.ok ? `유효한 목표 (반경 ${r.clearance_m} m 여유)${r.localized ? '' : ' · 위치 미확인이라 제출 불가'}` : `거부: ${GOAL[r.reason] || r.reason}`;
  }

  function mount(host, opts) {
    host.innerHTML = `<div class="sec-head"><h2>저장 지도 자율주행</h2><span class="badge" data-nv="badge">확인 중</span></div>
      <div class="toolbar-row small"><label>지도 버전 <select data-nv="map"><option value="">목록 불러오는 중</option></select></label>
        <button type="button" data-nv="load">지도 로딩</button></div>
      <div class="toolbar-row small"><label>x <input data-nv="x" type="number" step="0.05" class="in-s"/></label><label>y <input data-nv="y" type="number" step="0.05" class="in-s"/></label>
        <label>yaw <input data-nv="yaw" type="number" step="0.05" class="in-s" value="0"/></label>
        <button type="button" data-nv="fromgoal" title="목표 입력칸(지도 클릭 좌표)을 가져옵니다">지도 클릭 좌표 사용</button>
        <button type="button" data-nv="pose">초기 위치 지정</button><button type="button" data-nv="check">목표 검사</button></div>
      <p class="small" data-nv="result" role="status"></p><p class="small bad" data-nv="error" role="alert"></p>
      <dl class="mission-kv" data-nv="rows"></dl><ul class="block-list small" data-nv="why"></ul>
      <p class="small muted">HTTP 접수와 실제 결과는 다릅니다. 위치가 CONFIRMED이고 VERIFIED 지도일 때만 임무가 허용되며, 목표는 서버가 지도 범위·벽·미지 영역·로봇 반경으로 다시 검사합니다.</p>`;
    const q = n => host.querySelector(`[data-nv="${n}"]`);
    let error = '', result = '';
    async function call(path, body, method = 'POST') {
      const r = await opts.request(path, method === 'GET' ? {} : { method, body: JSON.stringify(body) });
      const j = await r.json().catch(() => ({}));
      if (!r.ok) throw Error((j.error && `${j.error.code}: ${j.error.message}`) || `HTTP ${r.status}`);
      return j;
    }
    async function loadMaps() {
      try {
        const j = await call('/map-registry/maps', null, 'GET');
        const list = j.map_versions || j.maps || [];
        q('map').innerHTML = list.map(m => `<option value="${esc(m.map_version_id)}">${esc(m.map_version_id)} · ${esc(m.verification_status)}</option>`).join('') || '<option value="">등록된 지도 없음</option>';
      } catch (e) { q('map').innerHTML = '<option value="">지도 목록 실패</option>'; }
    }
    const robotPath = suffix => { const r = opts.getRobot(); return r ? `/robots/${encodeURIComponent(r.robot_id)}${suffix}` : null; };
    async function act(fn) { error = ''; try { await fn(); } catch (e) { error = e.message; } if (opts.refresh) await opts.refresh(); render(); }
    q('load').addEventListener('click', () => act(async () => {
      const r = opts.getRobot(); await call(robotPath('/navigation'), { action: 'LOAD', epoch: r.lease ? r.lease.epoch : null, map_version_id: q('map').value });
      result = '로딩 요청 접수 — Agent의 위치 추정 상태를 확인하세요';
    }));
    q('pose').addEventListener('click', () => act(async () => {
      const r = opts.getRobot();
      await call(robotPath('/navigation'), { action: 'INITIAL_POSE', epoch: r.lease ? r.lease.epoch : null, x: +q('x').value, y: +q('y').value, yaw: +q('yaw').value });
      result = '초기 위치 전달 — CONFIRMED가 될 때까지 기다리세요';
    }));
    q('check').addEventListener('click', () => act(async () => {
      const j = await call(robotPath(`/goal-check?x=${encodeURIComponent(q('x').value)}&y=${encodeURIComponent(q('y').value)}`), null, 'GET');
      result = goalText(j);
    }));
    q('fromgoal').addEventListener('click', () => { const g = opts.getGoal ? opts.getGoal() : null; if (g) { q('x').value = g.x; q('y').value = g.y; q('yaw').value = g.yaw; } });
    function render() {
      const robot = opts.getRobot(), me = opts.getMe(); const v = view(robot, me);
      const loc = (robot && robot.state && robot.state.localization) || {};
      q('badge').textContent = LOC[loc.status] || loc.status || '미보고';
      q('badge').className = 'badge ' + (loc.status === 'CONFIRMED' ? 'ok' : ['FAILED', 'DEGRADED'].includes(loc.status) ? 'bad' : 'warn');
      q('rows').innerHTML = v.rows.map(([k, val]) => `<div><dt>${esc(k)}</dt><dd>${esc(val)}</dd></div>`).join('');
      q('load').disabled = !!v.blocked.load; q('pose').disabled = !!v.blocked.pose;
      q('why').innerHTML = Object.entries(v.blocked).filter(([, w]) => w).map(([k, w]) => `<li><b>${{ load: '지도 로딩', pose: '초기 위치', goal: '목표 제출' }[k]}</b> — ${esc(w)}</li>`).join('');
      q('error').textContent = error; q('result').textContent = result;
    }
    loadMaps(); const timer = setInterval(render, 500); render();
    return { render, reloadMaps: loadMaps, destroy() { clearInterval(timer); } };
  }
  return { view, steps, goalText, mount };
});
