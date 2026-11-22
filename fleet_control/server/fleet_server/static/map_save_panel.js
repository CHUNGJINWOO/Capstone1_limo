'use strict';
/* Web "save the current SLAM map and register it" (save only: no mapping start, no motion).
   POST /robots/{id}/map-save, GET /robots/{id}/map-saves. HTTP acceptance (QUEUED) is not a result: the stages and the final REGISTERED /
   FAILED / TIMEOUT / UNKNOWN come from the Agent and the server. Pure helpers are exported for Node tests. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.MapSavePanel = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const ID_RE = /^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$/;
  const STEPS = [['QUEUED', '접수'], ['ACCEPTED', 'Agent 수락'], ['SAVING_RASTER', '지도 이미지 저장'], ['SAVING_POSEGRAPH', 'Pose Graph 저장'], ['UPLOADING', '전송'], ['REGISTERED', '등록']];
  const ORDER = Object.fromEntries(STEPS.map(([k], i) => [k, i]));
  const ACTIVE = ['QUEUED', 'ACCEPTED', 'SAVING_RASTER', 'SAVING_POSEGRAPH', 'UPLOADING'];

  const REASONS = {
    agent_not_ready: '로봇이 저장할 준비가 되지 않았습니다(SLAM 지도 또는 저장 서비스 없음).',
    motor_path_not_locked: '모터 경로가 잠금 상태가 아니라 저장하지 않았습니다.',
    raster_failed: '지도 이미지(PGM/YAML) 저장에 실패했습니다.',
    posegraph_failed: 'Pose Graph 저장에 실패했습니다.',
    saved_file_invalid: '저장된 파일이 비어 있거나 없습니다.',
    transfer_corrupt: '전송 중 파일이 손상되었습니다(크기·SHA-256 불일치).',
    upload_failed: '서버로 전송하지 못했습니다. 파일은 로봇에 남아 있습니다.',
    unknown_pixels_misclassified: '임계값 안전 검사에서 거부되었습니다(미확인 칸을 빈 공간으로 읽게 됨).',
    policy_violation: '지도 등록 정책에서 거부되었습니다.',
    conflict: '같은 이름·내용의 지도가 다른 속성으로 이미 등록되어 있습니다. 지도 이름을 바꿔 다시 저장하세요.',
    checksum_mismatch: '체크섬이 일치하지 않습니다.',
    validation_error: '지도 파일 검증에 실패했습니다.',
    timeout: '제한 시간 안에 끝나지 않았습니다. 로봇에 파일이 남아 있을 수 있으니 새로 저장하세요.',
    agent_restarted: '저장 중 Agent가 재시작되었습니다. 결과를 알 수 없으니 새로 저장하세요(자동 재개 없음).',
    robot_offline: '저장 중 로봇 연결이 끊겨 결과를 알 수 없습니다.',
    duplicate_attempt: '같은 저장 요청이 이미 로봇에서 시작된 적이 있어 실행하지 않았습니다.',
    save_not_active: '이미 끝났거나 만료된 저장 요청입니다.',
  };

  function esc(s) {
    return String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  }

  function errorText(code, message) {
    const base = REASONS[code];
    const tail = message ? ` (${message})` : '';
    return base ? `${base}${tail}` : `${code || '실패'}${tail}`;
  }

  /** Why the save button is disabled, in order. Empty list = the request may be sent (the server still decides). */
  function blockReasons(c) {
    const out = [];
    if (!c.canControl) out.push('운영자 권한이 필요합니다.');
    if (!c.robot) { out.push('로봇을 선택하세요.'); return out; }
    if (!c.online) out.push('로봇이 오프라인입니다.');
    else if (!c.agentCapable) out.push('이 Agent는 지도 저장 기능이 꺼져 있습니다(map_save_enabled로 실행되지 않음).');
    else if (!c.capAvailable) out.push(`로봇이 지금은 저장할 수 없습니다: ${c.capReason || '이유 미보고'}`);
    if (c.online && !c.leaseOwned) out.push('제어권(Lease)이 필요합니다.');
    if (!c.floorId) out.push('저장할 건물·층을 위에서 선택하세요(등록 대상).');
    if (!c.labelValid) out.push('지도 이름은 영문/숫자로 시작하고 영문·숫자·. _ - 만 64자 이내로 입력하세요.');
    if (c.activeSave) out.push('이 로봇의 저장이 아직 진행 중입니다.');
    if (c.busy) out.push('요청을 보내는 중입니다.');
    return out;
  }

  /** The state of a save as the operator must read it. */
  function savePhase(save, nowS, local) {
    if (local && local.sending) return { phase: 'sending', tone: 'muted', label: '전송 중', detail: '요청을 서버로 보내는 중', index: -1 };
    if (local && local.httpError) return { phase: 'rejected', tone: 'bad', label: '서버가 거부함', detail: local.httpError, index: -1 };
    if (!save) return { phase: 'none', tone: 'muted', label: '저장 기록 없음', detail: '', index: -1 };
    const s = save.status;
    const remain = typeof save.expires_at === 'number' && typeof nowS === 'number' ? Math.max(0, Math.round(save.expires_at - nowS)) : null;
    const wait = remain != null ? ` · 약 ${remain}초 안에 끝나지 않으면 타임아웃` : '';
    switch (s) {
      case 'QUEUED': return { phase: 'queued', tone: 'warn', label: '접수됨 · Agent 수락 대기', detail: wait.replace(/^ · /, ''), index: 0 };
      case 'ACCEPTED': return { phase: 'running', tone: 'warn', label: 'Agent가 수락함', detail: wait.replace(/^ · /, ''), index: 1 };
      case 'SAVING_RASTER': return { phase: 'running', tone: 'warn', label: '지도 이미지(PGM/YAML) 저장 중', detail: wait.replace(/^ · /, ''), index: 2 };
      case 'SAVING_POSEGRAPH': return { phase: 'running', tone: 'warn', label: 'Pose Graph 저장 중', detail: wait.replace(/^ · /, ''), index: 3 };
      case 'UPLOADING': return { phase: 'running', tone: 'warn', label: '서버로 전송·등록 중', detail: wait.replace(/^ · /, ''), index: 4 };
      case 'REGISTERED': return { phase: 'succeeded', tone: 'ok', label: '등록 완료 (UNVERIFIED)', detail: (save.result && save.result.map_version_id) || '', index: 5 };
      case 'FAILED': return { phase: 'failed', tone: 'bad', label: '실패', detail: errorText(save.error_code, save.detail), index: ORDER.UPLOADING };
      case 'TIMEOUT': return { phase: 'timeout', tone: 'bad', label: '타임아웃 · 결과 미확인', detail: errorText('timeout', ''), index: ORDER.UPLOADING };
      case 'UNKNOWN': return { phase: 'unknown', tone: 'bad', label: '결과 미확인', detail: errorText(save.error_code, ''), index: ORDER.UPLOADING };
      default: return { phase: 'unknown', tone: 'bad', label: String(s || 'UNKNOWN'), detail: '', index: -1 };
    }
  }

  function stepperHtml(p) {
    if (!p || p.phase === 'none') return '<span class="muted">저장 기록 없음</span>';
    const terminalBad = ['failed', 'timeout', 'unknown', 'rejected'].includes(p.phase);
    const steps = STEPS.map(([k, t], i) => {
      if (terminalBad && i === STEPS.length - 1) return ['x', p.phase === 'failed' ? '실패' : (p.phase === 'rejected' ? '거부' : '미확인'), i];
      return [k, t, i];
    });
    const html = steps.map(([k, t, i]) => {
      const cls = i < p.index ? ' past' : (i === p.index && !terminalBad ? ' cur ' + p.tone : (terminalBad && i === steps.length - 1 ? ' cur bad' : ''));
      return `<span class="step${cls}">${esc(t)}</span>`;
    }).join('<span class="arrow">→</span>');
    return `${html}<span class="small ${p.tone === 'ok' ? '' : p.tone}" style="margin-left:6px">${esc(p.label)}${p.detail ? ' — ' + esc(p.detail) : ''}</span>`;
  }

  function fmtTs(t) {
    if (!Number.isFinite(Number(t))) return '';
    const d = new Date(Number(t) * 1000);
    return d.toLocaleTimeString('ko-KR', { hour12: false }) + '.' + String(d.getMilliseconds()).padStart(3, '0');
  }

  /** Per-file result of one save (size, hash, save window) from the Agent manifest, verified by the server before registration. */
  function filesRows(save) {
    const f = save && save.manifest && save.manifest.files;
    if (!f) return [];
    return [['yaml', 'map.yaml'], ['pgm', 'map.pgm'], ['posegraph', 'graph.posegraph'], ['data', 'graph.data']].filter(([k]) => f[k]).map(([k, n]) => {
      const e = f[k];
      return [n, `${e.size} B · ${esc(e.sha256 ? e.sha256.slice(0, 16) + '…' : '해시 없음')} · 저장 ${fmtTs(e.saved_start)}–${fmtTs(e.saved_end)}${e.problem ? ' · ⚠ ' + esc(e.problem) : ''}`];
    });
  }

  /** What may and may not be claimed about the raster and the pose graph belonging together. */
  function consistencyRows(save) {
    const c = save && save.consistency;
    if (!c) return [];
    const yn = (v) => (v === true ? '예' : (v === false ? '아니오' : '알 수 없음'));
    const same = c.same_snapshot === 'LIKELY_NOT_PROVEN' ? '같은 시점일 가능성이 높음(증명되지 않음)' : '같은 시점이라고 확정할 수 없음';
    return [
      ['두 저장물의 일치', same],
      ['저장 사이 간격', c.gap_between_raster_end_and_posegraph_start_s != null ? `${c.gap_between_raster_end_and_posegraph_start_s} s (이미지 끝 → Pose Graph 시작)` : '알 수 없음'],
      ['저장 중 라이브 지도 변화 없음', yn(c.live_map_unchanged_during_save)],
      ['저장 중 로봇 정지', `${yn(c.robot_stationary_during_save)}${c.robot_moved_m != null ? ` (이동 ${c.robot_moved_m} m, 회전 ${c.robot_rotated_rad} rad)` : ''}`],
    ];
  }

  // ---------------------------------------------------------------- DOM (browser only)
  let deps = null;
  let saves = [];
  let capability = null;
  let busy = false;
  let local = null;
  let shownRegistered = '';

  const el = (id) => document.getElementById(id);

  function init(d) {
    if (typeof document === 'undefined' || !el('mapSaveCard')) return;
    deps = d;
    el('mapSaveBtn').addEventListener('click', send);
    ['mapSaveLabel'].forEach((id) => el(id).addEventListener('input', render));
  }

  async function refresh() {
    if (!deps || !el('mapSaveCard')) return;
    const robot = deps.getRobot();
    if (!robot) { saves = []; capability = null; return render(); }
    try {
      const r = await deps.api(`/robots/${encodeURIComponent(robot.robot_id)}/map-saves?limit=5`);
      if (r.ok) { const j = await r.json(); saves = j.saves || []; capability = j.capability || null; }
    } catch (_) { /* keep the last view; the page shows the connection state elsewhere */ }
    render();
  }

  async function send() {
    const robot = deps && deps.getRobot();
    if (!robot || busy) return;
    const sel = deps.getSelection();
    const body = {
      epoch: robot.lease ? robot.lease.epoch : null,
      idempotency_key: 'save-' + Date.now() + '-' + Math.random().toString(16).slice(2, 10),
      floor_id: sel.floorId, label: el('mapSaveLabel').value.trim(), note: el('mapSaveNote').value.trim(), threshold_policy: el('mapSavePolicy').value,
    };
    if (sel.areaId) body.area_id = sel.areaId;
    busy = true; local = { sending: true }; render();
    try {
      const r = await deps.api(`/robots/${encodeURIComponent(robot.robot_id)}/map-save`, { method: 'POST', body: JSON.stringify(body) });
      const j = await r.json().catch(() => ({}));
      if (!r.ok) throw Error((j.error && `${j.error.code}: ${j.error.message}`) || `HTTP ${r.status}`);
      local = null;
    } catch (e) {
      local = { httpError: e.message };
    } finally {
      busy = false;
      await refresh();
    }
  }

  function render() {
    if (!deps || !el('mapSaveCard')) return;
    const robot = deps.getRobot();
    const me = deps.getMe();
    const sel = deps.getSelection();
    const cap = (robot && robot.state && robot.state.map_save) || {};
    const ctx = {
      canControl: !!me && (me.role === 'operator' || me.role === 'admin'),
      robot: !!robot, online: !!(robot && robot.online),
      agentCapable: !!(robot && robot.agent && robot.agent.map_save === true),
      capAvailable: !!(capability ? capability.available : cap.available), capReason: (capability && capability.reason) || cap.reason || '',
      leaseOwned: !!(robot && robot.lease && me && robot.lease.owner === me.name),
      floorId: sel.floorId, labelValid: ID_RE.test(el('mapSaveLabel').value.trim()),
      activeSave: saves.some((s) => ACTIVE.includes(s.status)), busy,
    };
    const reasons = blockReasons(ctx);
    el('mapSaveBtn').disabled = reasons.length > 0;
    el('mapSaveBtn').title = reasons.join(' / ');
    el('mapSaveBlocks').innerHTML = reasons.length ? reasons.map((r) => `<li>${esc(r)}</li>`).join('') : '<li class="ok">지금 저장할 수 있습니다.</li>';
    el('mapSaveTarget').innerHTML = deps.getTargetText ? deps.getTargetText() : '';
    const latest = saves[0] || null;
    el('mapSavePhase').innerHTML = stepperHtml(savePhase(latest, Date.now() / 1000, local));
    const rows = latest ? filesRows(latest).concat(consistencyRows(latest)) : [];
    let html = rows.length ? '<table class="reg-kv">' + rows.map((r) => `<tr><th>${esc(r[0])}</th><td class="mono">${r[1]}</td></tr>`).join('') + '</table>' : '';
    if (latest && latest.status === 'REGISTERED' && latest.result) {
      html = `<div class="small ok">등록됨: <span class="mono">${esc(latest.result.map_version_id)}</span> · UNVERIFIED · 로봇에 적용되지 않음` + (latest.result.yaml_adjustment ? ' · YAML 보정본(원본 보존)' : '') + '</div>' + html;
      if (shownRegistered !== latest.save_id) {
        shownRegistered = latest.save_id;
        if (deps.onRegistered) deps.onRegistered(latest);
      }
    }
    el('mapSaveResult').innerHTML = html;
  }

  return { blockReasons, savePhase, stepperHtml, filesRows, consistencyRows, errorText, init, refresh, render, ID_RE, ACTIVE, STEPS };
});
