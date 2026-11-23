'use strict';
/* Independent locked-bench panel. Host supplies authenticated Fleet API and fresh,
 * authoritative evidence; this module never publishes ROS messages. */
(function (root, factory) {
  const exported = factory();
  if (typeof module === 'object' && module.exports) module.exports = exported;
  root.ManualControlPanel = exported;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const PERIOD_MS = 80;
  const REQUEST_TIMEOUT_MS = 180;
  const EVIDENCE_MAX_AGE_MS = 1000;
  const MOVEMENT = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']);
  const DEADMAN = 'Space';
  const finiteAxis = v => typeof v === 'number' && Number.isFinite(v) && v >= -1 && v <= 1;
  const zero = () => ({ linear: 0, turn: 0, deadman: false });
  const STALE = '최신 안전 증거 없음';
  // Agent selector / server blocker codes shown to the operator (unknown codes are shown verbatim).
  const BLOCKER_TEXT = {
    STARTUP: 'Agent 선택기 기동 중', AGENT_EXECUTION_NOT_ALLOWED: 'Agent 실행 권한 없음 (서버 세션 미확정)',
    FLEET_DISCONNECTED: 'Fleet 서버 연결 끊김', RECONCILIATION_REQUIRED: '이전 임무 상태 조정 필요',
    LEASE_MISSING_OR_EXPIRED: 'Agent가 유효한 Lease를 아직 받지 못함', FLEET_MISSION_ACTIVE: 'AUTO 임무 진행 중',
    NAV2_SERVER_MISSING: 'Nav2 NavigateToPose 서버 없음', NAV2_SERVER_NOT_UNIQUE: 'Nav2 서버가 둘 이상 (Goal 은닉 가능)',
    NAV2_STATUS_SETTLING: 'Nav2 상태 확인 중', NAV2_GOAL_ACTIVE: '다른 Nav2 Goal 실행 중',
    NAV2_SERVER_CHANGED_WITH_ACTIVE_GOAL: 'Nav2 서버 교체 후 종료 증거 대기',
    FOLLOW_PATH_SERVER_MISSING: 'FollowPath 서버(controller_server) 없음', FOLLOW_PATH_SERVER_NOT_UNIQUE: 'FollowPath 서버가 둘 이상',
    FOLLOW_PATH_UNEXPECTED_SERVER_NODE: 'FollowPath 서버 노드 불일치', FOLLOW_PATH_STATUS_SETTLING: 'FollowPath 상태 확인 중',
    FOLLOW_PATH_GOAL_ACTIVE: 'FollowPath 실행 중', FOLLOW_PATH_SERVER_CHANGED_WITH_ACTIVE_GOAL: 'FollowPath 서버 교체 후 종료 증거 없음',
    MODE_IDLE: '운용 모드 IDLE', MODE_TRANSITIONING: '운용 모드 전환 중', MODE_FAULT: '운용 모드 FAULT', NO_OWNED_GOAL: '실행 중인 Goal 없음',
    NAV_INPUT_STALE: 'Nav2 속도 입력 없음', MANUAL_INPUT_PRESENT: '수동 입력과 충돌', GOAL_OWNER_MISMATCH: 'Goal 소유자와 모드 불일치',
    WATCHDOG_NOT_AUTHORIZED: 'Motion Watchdog 미승인 (Goal 종료 증명 대기)', GATE_STATUS_STALE: 'Safety Gate 상태 수신 끊김',
    GATE_NOT_NORMAL: 'Safety Gate 차단 상태', ODOM_STALE: 'Odometry 수신 끊김', VELOCITY_GRAPH_NOT_LOCKED: '속도 경로 그래프 불일치 (/cmd_vel 우회 의심)',
  };
  const blockerText = code => typeof code === 'string' && code ? (BLOCKER_TEXT[code] || code) : '';
  function admissibility(c) {
    if (!c || !Number.isFinite(c.evidenceAgeMs) || c.evidenceAgeMs < 0 || c.evidenceAgeMs > EVIDENCE_MAX_AGE_MS) return STALE;
    if (c.canControl !== true) return '운영자 권한 필요';
    if (c.online !== true || c.agentReady !== true) return 'Agent 연결 또는 준비 상태 불명';
    if (c.motorLocked !== true) return '모터 잠금 증거 없음';
    if (c.leaseOwned !== true || !Number.isSafeInteger(c.leaseEpoch) || c.leaseEpoch < 1) return '내 Lease 없음';
    if (c.operatingMode && c.operatingMode !== 'MANUAL') return `운용 모드 ${c.operatingMode} — 운용 모드 카드에서 MANUAL로 전환 필요`;
    if (c.mode !== 'IDLE' && c.mode !== 'MANUAL') return c.mode === 'MAPPING' ? '맵핑 중 입력 차단' : c.mode === 'AUTO' ? 'AUTO 임무 진행 중 · 취소 후 사용' : '운용 모드 불명';
    if (c.blocker) return blockerText(c.blocker);
    if (c.navIdleProven !== true) return 'Nav2 Goal 종료 증거 없음';
    if (c.gateNormal !== true) return 'Safety Gate 준비 안 됨';
    if (typeof c.robotId !== 'string' || !/^[A-Za-z0-9_.-]{1,64}$/.test(c.robotId)) return '로봇 선택 필요';
    if (typeof c.agentBootId !== 'string' || !c.agentBootId || typeof c.serverBootId !== 'string' || !c.serverBootId) return '서버 / Agent 세션 불명';
    return '';
  }
  function keyboardAxes(keys) {
    return { linear: Number(keys.has('KeyW') || keys.has('ArrowUp')) - Number(keys.has('KeyS') || keys.has('ArrowDown')),
      turn: Number(keys.has('KeyA') || keys.has('ArrowLeft')) - Number(keys.has('KeyD') || keys.has('ArrowRight')),
      deadman: keys.has(DEADMAN) };
  }
  function pointerAxes(x, y, rect) {
    if (![x, y, rect?.left, rect?.top, rect?.width, rect?.height].every(Number.isFinite) || rect.width <= 0 || rect.height <= 0) return null;
    let turn = -((x - (rect.left + rect.width / 2)) / (rect.width / 2));
    let linear = -((y - (rect.top + rect.height / 2)) / (rect.height / 2));
    const magnitude = Math.hypot(turn, linear);
    if (magnitude > 1) { turn /= magnitude; linear /= magnitude; }
    return { linear, turn, deadman: true };
  }
  function gamepadAxes(pad) {
    if (!pad?.connected || !finiteAxis(pad.axes?.[0]) || !finiteAxis(pad.axes?.[1]) || !pad.buttons?.[4]) return null;
    return { linear: -pad.axes[1], turn: -pad.axes[0], deadman: pad.buttons[4].pressed === true };
  }
  function editable(el) {
    return !!el && (el.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(el.tagName) || !!el.closest?.('[contenteditable]:not([contenteditable="false"])'));
  }
  function mount(host, opts) {
    if (!host || !opts || typeof opts.request !== 'function' || typeof opts.getContext !== 'function') throw new Error('host, request, getContext required');
    host.innerHTML = `<section class="manual-control" aria-label="수동 조작 입력 시험">
      <h2>수동 조작 · 모터 잠금 입력 시험</h2><p>이 패널은 잠금 벤치 전용입니다. 소프트웨어 정지는 물리 비상정지를 대신하지 않습니다.</p>
      <dl class="mc-state"><div><dt>모드</dt><dd data-mc="mode">확인 중</dd></div><div><dt>Lease</dt><dd data-mc="lease">확인 중</dd></div><div><dt>Agent</dt><dd data-mc="agent">확인 중</dd></div><div><dt>Gate</dt><dd data-mc="gate">확인 중</dd></div></dl>
      <p role="status" aria-live="polite" data-mc="status">입력 대기</p><p data-mc="block">안전 상태 확인 중</p><label>입력 장치 <select data-mc="source"><option value="keyboard">키보드</option><option value="pointer">마우스 / 터치</option><option value="gamepad">게임패드</option></select></label>
      <div class="mc-actions"><button type="button" data-mc="start">입력 시험 시작</button><button type="button" data-mc="stop">정지 요청</button></div>
      <p>키보드: WASD / 방향키 + Space 유지. 마우스·터치: 원을 누른 채 이동. 게임패드: 왼쪽 스틱 + L1(버튼 4) 유지. 후진은 잠금 Agent에서 차단됩니다.</p>
      <div class="mc-stick" data-mc="stick" role="application" aria-label="누르고 있는 동안만 동작하는 가상 조이스틱"><span data-mc="knob"></span></div>
      <output data-mc="values">선속도 요청 0.00 m/s · 각속도 요청 0.00 rad/s · Deadman 해제</output></section>`;
    const q = name => host.querySelector(`[data-mc="${name}"]`);
    const status = q('status'), sourceEl = q('source'), stick = q('stick'), knob = q('knob');
    const context = () => { try { return opts.getContext(); } catch (_) { return null; } };
    const keys = new Set(); let pointer = null, session = null, starting = false, generation = 0, destroyed = false, padIdentity = null, padDown = false, lastValues = zero();
    const handlers = [];
    const on = (target, type, fn, options) => { target.addEventListener(type, fn, options); handlers.push(() => target.removeEventListener(type, fn, options)); };
    const say = value => { status.textContent = value; };
    function view(values) {
      lastValues = values;
      q('values').textContent = `선속도 요청 ${(Math.max(0, values.linear) * .10).toFixed(2)} m/s · 각속도 요청 ${(values.turn * .30).toFixed(2)} rad/s · Deadman ${values.deadman ? '유지' : '해제'} (모터 잠금)`;
      knob.style.transform = `translate(${(-values.turn * 38).toFixed(1)}px, ${(-values.linear * 38).toFixed(1)}px)`;
    }
    function renderContext(c, reason) {
      q('mode').textContent = session ? 'MANUAL · 잠금 경로' : ({ IDLE:'대기', AUTO:'AUTO', MAPPING:'MAPPING', MANUAL:'MANUAL (다른 세션)' }[c?.mode] || 'UNKNOWN');
      q('lease').textContent = c?.leaseOwned === true ? '내 Lease' : '없음 / 다른 사용자';
      q('agent').textContent = c?.online === true && c?.agentReady === true ? '연결' : '미확인';
      q('gate').textContent = c?.gateNormal === true ? '정상' : '차단 / 미확인';
      q('block').textContent = reason ? `입력 차단: ${reason}` : '잠금 입력 시험 가능';
      q('start').disabled = !!session || starting || !!reason;
      q('stop').disabled = !session && !starting;
    }
    async function post(path, body, timeout = REQUEST_TIMEOUT_MS) {
      const controller = new AbortController();
      const timer = setTimeout(() => controller.abort(), timeout);
      try {
        const res = await opts.request(path, { method:'POST', body:JSON.stringify(body), signal:controller.signal });
        const result = await res.json();
        if (!res.ok) throw new Error(result?.error?.message || `HTTP ${res.status}`);
        return result;
      } finally { clearTimeout(timer); }
    }
    function sendStop(s) {
      if (!s) return;
      post(s.path, { action:'stop', epoch:s.epoch, session_id:s.id }).catch(() => {});
    }
    function stop(reason = '정지 요청 · 다시 시작해야 합니다') {
      const old = session; session = null; generation++; keys.clear(); pointer = null; padIdentity = null; padDown = false;
      view(zero()); say(reason); renderContext(context(), admissibility(context())); sendStop(old);
    }
    function activePanel() {
      try { return !destroyed && !document.hidden && document.hasFocus() && (typeof opts.isActive !== 'function' || opts.isActive() === true); }
      catch (_) { return false; }
    }
    function identity(c) { return `${c.robotId}\0${c.leaseEpoch}\0${c.agentBootId}\0${c.serverBootId}`; }
    function onlyPad() {
      if (!window.isSecureContext || typeof navigator.getGamepads !== 'function') return null;
      try { const pads = Array.from(navigator.getGamepads()).filter(p => p?.connected); return pads.length === 1 ? pads[0] : null; } catch { return null; }
    }
    async function begin() {
      if (session || starting || destroyed) return;
      // Start with fresh evidence: fetch once when it is older than half the bound, then judge with the SAME bound. Otherwise a
      // Start just before the host's 1 s poll was refused (STALE) or the new session was stopped ~40 ms later by tick()
      // (2026-11-21 Gazebo pointer diagnosis); while the session runs the host refreshes every 300 ms (see active()).
      const c0 = context();
      if (typeof opts.refreshEvidence === 'function' && !(c0 && Number.isFinite(c0.evidenceAgeMs) && c0.evidenceAgeMs >= 0 && c0.evidenceAgeMs <= EVIDENCE_MAX_AGE_MS / 2)) {
        starting = true;
        try { await opts.refreshEvidence(); } catch (_) { /* judged below on whatever evidence exists */ } finally { starting = false; }
        if (session || destroyed) return;
      }
      const ctx = context(), why = admissibility(ctx), input = sourceEl.value;
      if (why || !activePanel()) { say(why || '활성 탭에서 시작하세요'); return; }
      if (keys.size || pointer) { say('입력을 모두 놓고 시작하세요'); return; }
      const pad = input === 'gamepad' ? onlyPad() : null;
      if (input === 'gamepad' && (!gamepadAxes(pad) || pad.buttons[4].pressed)) { say('게임패드 하나를 연결하고 L1을 놓으세요'); return; }
      const attempt = ++generation, path = `/robots/${encodeURIComponent(ctx.robotId)}/manual-bench`, epoch = ctx.leaseEpoch;
      starting = true; renderContext(ctx, '시작 중');
      try {
        const result = await post(path, { action:'start', epoch });
        if (typeof result.session_id !== 'string' || !result.session_id) throw new Error('세션 응답 오류');
        const next = { id:result.session_id, path, epoch, identity:identity(ctx), source:input, seq:0, busy:false };
        const latest = context();
        if (generation !== attempt || !activePanel() || admissibility(latest) || identity(latest) !== next.identity) { sendStop(next); return; }
        session = next;
        if (pad) padIdentity = { index:pad.index, id:pad.id, timestamp:pad.timestamp, changedAt:performance.now() };
        say('잠금 입력 시험 준비 · Deadman 대기'); renderContext(ctx, '');
        await tick();
      } catch (error) { if (generation === attempt) stop(`시작 거부: ${error.message}`); }
      finally { starting = false; renderContext(context(), admissibility(context())); }
    }
    async function tick() {
      const c = context(), why = admissibility(c); renderContext(c, why);
      const s = session;
      if (!s) return;
      if (why || !activePanel() || identity(c) !== s.identity || sourceEl.value !== s.source) { stop(why || '연결·세션·화면 변경'); return; }
      let sample = zero();
      if (s.source === 'keyboard') sample = keyboardAxes(keys);
      else if (s.source === 'pointer' && pointer) sample = pointer.sample;
      else if (s.source === 'gamepad') {
        const pad = onlyPad(); sample = gamepadAxes(pad);
        if (!sample || pad.index !== padIdentity?.index || pad.id !== padIdentity?.id) { stop('게임패드 연결 해제'); return; }
        if (padDown && !sample.deadman) { stop('Deadman 해제'); return; }
        if (pad.timestamp !== padIdentity.timestamp) { padIdentity.timestamp = pad.timestamp; padIdentity.changedAt = performance.now(); }
        if (sample.deadman && (!Number.isFinite(pad.timestamp) || pad.timestamp <= 0 || performance.now() - padIdentity.changedAt >= 250)) { stop('게임패드 갱신 중단'); return; }
        padDown = sample.deadman;
      }
      if (!finiteAxis(sample.linear) || !finiteAxis(sample.turn) || typeof sample.deadman !== 'boolean') { stop('잘못된 입력'); return; }
      if (!sample.deadman) sample = zero();
      view(sample);
      if (s.busy) return;
      s.busy = true;
      try { await post(s.path, { action:'sample', epoch:s.epoch, session_id:s.id, input_format:'normalized_axes_v1', seq:++s.seq, ...sample }); }
      catch (e) { if (session === s) stop(`입력 만료 / 연결 오류: ${e.message}`); }
      finally { s.busy = false; }
    }
    on(q('start'), 'click', begin); on(q('stop'), 'click', () => stop());
    on(sourceEl, 'change', () => stop('입력 장치 변경'));
    on(window, 'keydown', e => {
      if (!session || session.source !== 'keyboard' || editable(e.target) || editable(document.activeElement)) return;
      if (MOVEMENT.has(e.code) || e.code === DEADMAN) {
        e.preventDefault();
        // A key held before this session must be released and pressed again.
        if (!e.repeat || keys.has(e.code)) keys.add(e.code);
      }
    });
    on(window, 'keyup', e => {
      if (!session || session.source !== 'keyboard') return;
      const wasDown = keys.has(DEADMAN); keys.delete(e.code);
      if (e.code === DEADMAN && wasDown) stop('Deadman 해제');
    });
    on(stick, 'pointerdown', e => {
      if (!session || session.source !== 'pointer' || pointer || !e.isPrimary || (e.pointerType === 'mouse' && e.button !== 0)) return;
      const sample = pointerAxes(e.clientX, e.clientY, stick.getBoundingClientRect());
      if (!sample) { stop('잘못된 터치 입력'); return; }
      e.preventDefault(); pointer = { id:e.pointerId, sample };
      try {
        if (typeof stick.setPointerCapture !== 'function') throw new Error('pointer capture unavailable');
        stick.setPointerCapture(e.pointerId);
      } catch (_) { stop('포인터 추적 실패'); return; }
      view(sample);
    });
    on(stick, 'pointermove', e => {
      if (!pointer || e.pointerId !== pointer.id) return;
      const sample = pointerAxes(e.clientX, e.clientY, stick.getBoundingClientRect());
      if (!sample) { stop('잘못된 터치 입력'); return; }
      e.preventDefault(); pointer.sample = sample; view(sample);
    });
    for (const type of ['pointerup','pointercancel','lostpointercapture']) on(stick, type, e => { if (pointer?.id === e.pointerId) stop('터치 / 마우스 해제'); });
    on(window, 'blur', () => stop('브라우저 포커스 해제'));
    on(document, 'focusin', e => { if (session?.source === 'keyboard' && editable(e.target)) stop('입력창 포커스'); });
    on(window, 'pagehide', () => stop('페이지 종료'));
    on(window, 'offline', () => stop('네트워크 연결 끊김'));
    on(window, 'gamepaddisconnected', e => { if (session?.source === 'gamepad' && e.gamepad.index === padIdentity?.index) stop('게임패드 연결 해제'); });
    on(document, 'visibilitychange', () => { if (document.hidden) stop('브라우저 비활성'); });
    const timer = setInterval(tick, PERIOD_MS); renderContext(context(), admissibility(context()));
    // active(): a session is starting or running. The host refreshes the safety evidence faster meanwhile (2026-11-21: with the
    // 1 s dashboard poll the evidence age routinely exceeded EVIDENCE_MAX_AGE_MS just before each poll and tick() stopped the
    // session - the intermittent "pointer does nothing" in Gazebo; the 1000 ms bound itself is unchanged).
    return { stop, refresh:tick, active:() => !!session || starting, destroy() { if (destroyed) return; stop('패널 종료'); destroyed = true; clearInterval(timer); handlers.forEach(off => off()); } };
  }
  return { mount, admissibility, blockerText, keyboardAxes, pointerAxes, gamepadAxes, editable, PERIOD_MS, REQUEST_TIMEOUT_MS };
});
