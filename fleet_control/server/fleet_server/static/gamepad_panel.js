'use strict';
(() => {
  const form = $('gamepadForm');
  const number = id => $(id).valueAsNumber;
  const state = message => { $('gpState').textContent = message; };
  let session = null, starting = false, generation = 0;
  let bindings = [], gestures = new Map(), loadedDevice = null;
  const inputs = ['gpLinear','gpTurn','gpDeadman','gpCalV','gpCalW','gpDeadzone','gpMaxLinear','gpMaxTurn'];
  const inverted = ['gpInvertLinear','gpInvertTurn'];
  const profileKey = pad => 'gamepad-profile:' + pad.id + ':' + pad.axes.length + ':' + pad.buttons.length;
  function calibration(id) {
    const parts = $(id).value.split(',').map(value => value.trim());
    const [min, center, max] = parts.map(Number);
    if (parts.length !== 3 || parts.some(value => !value) ||
        ![min, center, max].every(Number.isFinite) || min < -1 || max > 1 ||
        center-min < .05 || max-center < .05) throw Error('보정값은 최소 < 중심 < 최대여야 합니다.');
    return [min, center, max];
  }
  function renderBindings() {
    $('gpBindings').textContent = bindings.map(b => b.buttons.join('+') + ' ' + b.gesture + ' → ' + b.action).join(' · ');
  }
  function loadProfile() {
    const pad = selectedPad();
    if (!pad) return;
    loadedDevice = profileKey(pad);
    bindings = []; gestures = new Map();
    for (const id of inputs) $(id).value = $(id).defaultValue;
    for (const id of inverted) $(id).checked = $(id).defaultChecked;
    try {
      const profile = JSON.parse(localStorage.getItem(profileKey(pad)));
      for (const id of inputs) if (typeof profile?.values?.[id] === 'string') $(id).value = profile.values[id];
      for (const id of inverted) $(id).checked = profile?.inverted?.[id] === true;
      for (const binding of profile?.bindings || []) {
        if (validBinding(binding, pad)) bindings.push(binding);
      }
    } catch { state('저장된 장치 설정을 읽지 못했습니다.'); }
    renderBindings();
  }
  function validBinding(binding, pad) {
    const actions = [...$('gpAction').options].map(o => o.value);
    return actions.includes(binding.action) && ['short','long'].includes(binding.gesture) &&
      Array.isArray(binding.buttons) && binding.buttons.length > 0 && binding.buttons.length <= 3 &&
      new Set(binding.buttons).size === binding.buttons.length &&
      binding.buttons.every(i => Number.isInteger(i) && i >= 0 && i < pad.buttons.length &&
        i !== number('gpDeadman') && !bindings.some(b => b.buttons.includes(i))) &&
      !bindings.some(b => b.action === binding.action);
  }
  function processBindings(pad) {
    if (!pad || document.hidden || !document.hasFocus() || pad.buttons[number('gpDeadman')]?.pressed) {
      gestures.clear(); return;
    }
    for (const binding of bindings) {
      if (binding.buttons.includes(number('gpDeadman'))) continue;
      const down = binding.buttons.every(i => pad.buttons[i]?.pressed);
      const previous = gestures.get(binding);
      const now = performance.now();
      if (down && !previous) gestures.set(binding, now);
      if (!down && previous !== undefined &&
          binding.buttons.every(i => !pad.buttons[i]?.pressed)) {
        gestures.delete(binding);
        const elapsed = now - previous;
        if ((binding.gesture === 'short' && elapsed >= 40 && elapsed < 700) ||
            (binding.gesture === 'long' && elapsed >= 700 && elapsed <= 3000)) {
          window.sendOperation?.(binding.action);
        }
      }
    }
  }
  const pads = () => {
    if (!window.isSecureContext || typeof navigator.getGamepads !== 'function') {
      throw Error('HTTPS 또는 localhost에서 Gamepad 기능을 사용하세요.');
    }
    return Array.from(navigator.getGamepads()).filter(pad => pad?.connected);
  };
  function scan() {
    try {
      const selected = $('gpDevice').value;
      const devices = pads();
      $('gpDevice').replaceChildren(...devices.map(pad => new Option(pad.id, pad.index)));
      if (devices.some(pad => String(pad.index) === selected)) $('gpDevice').value = selected;
      const pad = selectedPad();
      if (pad && loadedDevice !== profileKey(pad)) loadProfile();
      if (!devices.length) loadedDevice = null;
      if (!devices.length) state('이 탭을 활성화하고 게임패드 버튼을 눌러 주세요.');
    } catch (error) { state(error.message); }
  }
  function selectedPad() {
    if ($('gpDevice').value === '') return null;
    return pads().find(pad => String(pad.index) === $('gpDevice').value);
  }
  function post(path, message, timeout = 150) {
    return api(path, {method: 'POST', body: JSON.stringify(message), signal: AbortSignal.timeout(timeout)});
  }
  function stop(reason) {
    generation++;
    const old = session;
    session = null;
    if (old) {
      state(reason);
      return post(old.path, {action: 'stop', epoch: old.epoch, session_id: old.id}).catch(() => {});
    }
  }
  form.onsubmit = async event => {
    event.preventDefault();
    if (session || starting) return;
    const attempt = generation;
    starting = true;
    try {
      if (document.hidden || !document.hasFocus()) throw Error('활성 탭에서 시작하세요.');
      const pad = selectedPad();
      const linear = number('gpLinear'), turn = number('gpTurn'), button = number('gpDeadman');
      if (!pad || linear === turn || !Number.isFinite(pad.axes[linear]) ||
          !Number.isFinite(pad.axes[turn]) || !pad.buttons[button] || pad.buttons[button].pressed) {
        throw Error('서로 다른 실제 축과 Deadman을 지정한 뒤 버튼을 놓으세요.');
      }
      const rid = $('gpRobot').value;
      if (window.selectOperationRobot && !await window.selectOperationRobot(rid)) return;
      if (attempt !== generation) return;
      const calV = calibration('gpCalV'), calW = calibration('gpCalW');
      const root = '/robots/' + encodeURIComponent(rid);
      const response = await api(root + '/lock', {signal: AbortSignal.timeout(1000)});
      if (!response.ok) throw Error('Fleet Lease를 확인할 수 없습니다.');
      const {lease} = await response.json();
      if (!lease) throw Error('Fleet 제어권을 먼저 획득하세요.');
      if (attempt !== generation) return;
      const path = root + '/manual-bench';
      const result = await post(path, {action: 'start', epoch: lease.epoch});
      if (!result.ok) throw Error((await result.json()).error?.message || '모의 입력 시작 거부');
      const {session_id: id, ticket} = await result.json();
      if (attempt !== generation) return;
      session = {path, id, epoch: lease.epoch, ticket, index: pad.index, device: pad.id,
        linear, turn, button, calV, calW, seq: 0, down: false, busy: false, stamp: pad.timestamp, changedAt: performance.now(),
        deadzone: number('gpDeadzone'), maxV: number('gpMaxLinear'), maxW: number('gpMaxTurn'),
        signV: $('gpInvertLinear').checked ? -1 : 1, signW: $('gpInvertTurn').checked ? -1 : 1};
      state('모의 입력 준비 · Deadman 대기');
    } catch (error) { state(error.message); }
    finally { starting = false; }
  };
  async function tick() {
    let pad;
    try { pad = selectedPad(); } catch (error) { stop(error.message); return; }
    if (pad) $('gpInputs').textContent = '축 ' + Array.from(pad.axes, (v, i) => i + '=' + v.toFixed(2)).join('  ') +
      ' · 누른 버튼 ' + Array.from(pad.buttons, (b, i) => b.pressed ? i : null).filter(i => i !== null).join(', ');
    const s = session;
    if (!s) { processBindings(pad); return; }
    if (document.hidden || !document.hasFocus() || !pad || pad.index !== s.index || pad.id !== s.device) {
      stop('입력 연결 또는 탭 포커스가 끊겼습니다.'); return;
    }
    const down = pad.buttons[s.button]?.pressed === true;
    if (s.down && !down) { stop('Deadman 해제 · 영속도 요청'); return; }
    if (pad.timestamp !== s.stamp) {
      s.stamp = pad.timestamp;
      s.changedAt = performance.now();
    }
    if (down && (!Number.isFinite(s.stamp) || s.stamp <= 0 || performance.now() - s.changedAt >= 250)) {
      stop('게임패드 입력 갱신 중단 · 영속도 요청'); return;
    }
    s.down = down;
    if (s.busy) return;
    const axis = (value, sign, [min, center, max]) => {
      if (!Number.isFinite(value)) return 0;
      const normalized = (value-center) / (value >= center ? max-center : center-min);
      return Math.abs(normalized) > s.deadzone ? Math.max(-1, Math.min(1, normalized * sign)) : 0;
    };
    const linear = down ? Math.max(0, axis(pad.axes[s.linear], s.signV, s.calV)) * s.maxV : 0;
    const turn = down ? axis(pad.axes[s.turn], s.signW, s.calW) * s.maxW : 0;
    s.busy = true;
    try {
      const result = await post(s.path, {action: 'sample', epoch: s.epoch, session_id: s.id,
        ticket: s.ticket, seq: ++s.seq, deadman: down, linear, turn});
      if (!result.ok) throw Error('Fleet 입력이 거부됐습니다.');
      if (session !== s) return;
      s.ticket = (await result.json()).ticket;
      state('요청 ' + linear.toFixed(2) + ' m/s · ' + turn.toFixed(2) + ' rad/s');
    } catch (error) { if (session === s) stop(error.message); }
    finally { s.busy = false; }
  }
  window.stopManualPreview = stop;
  $('gpDevice').onchange = () => { stop('장치 변경'); loadProfile(); };
  $('gpClear').onclick = () => { stop('버튼 설정 변경'); bindings = []; gestures.clear(); renderBindings(); };
  $('gpBind').onclick = () => {
    try {
      const pad = selectedPad();
      const buttons = $('gpChord').value.trim().split('+').map(v => /^\d+$/.test(v.trim()) ? Number(v) : -1);
      const binding = {buttons, action: $('gpAction').value, gesture: $('gpGesture').value};
      if (!pad || !validBinding(binding, pad)) throw Error('중복 버튼, Deadman 및 잘못된 조합은 사용할 수 없습니다.');
      bindings.push(binding); renderBindings();
    } catch (error) { state(error.message); }
  };
  $('gpSaveProfile').onclick = () => {
    try {
      const pad = selectedPad();
      if (!pad) throw Error('장치를 선택하세요.');
      calibration('gpCalV'); calibration('gpCalW');
      if (bindings.some(b => b.buttons.includes(number('gpDeadman')))) throw Error('Deadman 중복을 제거하세요.');
      localStorage.setItem(profileKey(pad), JSON.stringify({
        values: Object.fromEntries(inputs.map(id => [id, $(id).value])),
        inverted: Object.fromEntries(inverted.map(id => [id, $(id).checked])), bindings
      }));
      state('장치 설정 저장됨');
    } catch (error) { state(error.message); }
  };
  $('gpLease').onclick = async () => {
    if (!$('gpRobot').reportValidity()) return;
    try {
      const rid = $('gpRobot').value;
      if (window.selectOperationRobot && !await window.selectOperationRobot(rid)) return;
      const root = '/robots/' + encodeURIComponent(rid);
      const response = await post(root + '/lock', {action: 'acquire', ttl_s: 15}, 1000);
      if (!response.ok) throw Error('Fleet Lease 획득 또는 갱신이 거부됐습니다.');
      state('Fleet Lease 15초 · 모의 입력을 시작하세요.');
    } catch (error) { state(error.message); }
  };
  $('gpScan').onclick = scan;
  $('gpStop').onclick = () => stop('중지됨');
  $('logout').addEventListener('click', () => stop('로그아웃'), {capture: true});
  window.addEventListener('blur', () => stop('탭 비활성 · 영속도 요청'));
  window.addEventListener('pagehide', () => stop('페이지 종료 · 영속도 요청'));
  window.addEventListener('gamepaddisconnected', event => {
    if (event.gamepad.index === session?.index) stop('게임패드 연결 해제');
    scan();
  });
  window.addEventListener('gamepadconnected', scan);
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) stop('탭 비활성 · 영속도 요청');
  });
  setInterval(tick, 50);
})();
