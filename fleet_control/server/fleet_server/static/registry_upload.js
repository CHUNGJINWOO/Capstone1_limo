'use strict';
/* Map Registry upload UI (Phase 2-C). Registration ONLY: uploads are stored as UNVERIFIED and are never loaded onto a robot.
   Pure helpers are exported (module.exports / window.RegistryUpload) so they can be unit-tested in Node. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.RegistryUpload = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const MAX_YAML = 256 * 1024;
  const MAX_PGM = 32 * 1024 * 1024;
  const MAX_POSEGRAPH = 128 * 1024 * 1024;
  const MAX_TOTAL_B64 = 44 * 1024 * 1024; // keep below the server write limit (48 MiB) with JSON overhead
  const ID_RE = /^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$/;
  const FRAME_RE = /^[A-Za-z0-9][A-Za-z0-9._/-]{0,63}$/;

  function esc(s) {
    return String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  }

  /** Light, display-only summary of a ROS map.yaml. The server remains the authority (strict parser). */
  function summarizeYaml(text) {
    const out = { image: null, resolution: null, origin: null, frame_id: null, negate: null, warnings: [] };
    const lines = String(text || '').split(/\r?\n/);
    for (const raw of lines) {
      const line = raw.replace(/#.*$/, '').trim();
      const m = line.match(/^([A-Za-z0-9_]+)\s*:\s*(.*)$/);
      if (!m) continue;
      const k = m[1];
      const v = m[2].trim();
      if (k === 'image') out.image = v.replace(/^["']|["']$/g, '');
      else if (k === 'resolution') out.resolution = Number(v);
      else if (k === 'negate') out.negate = Number(v);
      else if (k === 'frame_id') out.frame_id = v.replace(/^["']|["']$/g, '');
      else if (k === 'origin') {
        const a = v.replace(/^\[|\]$/g, '').split(',').map((x) => Number(x.trim()));
        if (a.length === 3 && a.every(Number.isFinite)) out.origin = a;
      }
    }
    if (out.image && out.image !== 'map.pgm') out.warnings.push(`YAML의 image가 "${out.image}" 입니다. 서버는 image: map.pgm 만 허용합니다.`);
    if (!Number.isFinite(out.resolution) || out.resolution <= 0) out.warnings.push('resolution을 읽을 수 없습니다.');
    if (!out.origin) out.warnings.push('origin([x, y, yaw])을 읽을 수 없습니다.');
    if (out.origin && Math.abs(out.origin[2]) > 1e-6) out.warnings.push('origin yaw ≠ 0 : 회전된 지도 좌표계로 등록됩니다.');
    if (out.frame_id && !FRAME_RE.test(out.frame_id)) out.warnings.push('frame_id 형식이 올바르지 않습니다.');
    return out;
  }

  /** Parse only the P5 header (width/height/maxval) for a pre-upload sanity display. */
  function pgmHeader(u8) {
    try {
      if (!u8 || u8.length < 8 || u8[0] !== 0x50 || u8[1] !== 0x35) return null;
      let i = 2;
      const toks = [];
      while (toks.length < 3 && i < Math.min(u8.length, 256)) {
        while (i < u8.length && (u8[i] === 0x20 || u8[i] === 0x0a || u8[i] === 0x0d || u8[i] === 0x09)) i++;
        if (u8[i] === 0x23) { while (i < u8.length && u8[i] !== 0x0a) i++; continue; }
        let s = '';
        while (i < u8.length && u8[i] > 0x20) s += String.fromCharCode(u8[i++]);
        if (s) toks.push(Number(s));
      }
      if (toks.length < 3 || !toks.every(Number.isFinite)) return null;
      return { width: toks[0], height: toks[1], maxval: toks[2] };
    } catch (e) { return null; }
  }

  /** Offset of the first pixel byte of a binary PGM (P5), or null. */
  function pgmDataOffset(u8) {
    try {
      let i = 2;
      let n = 0;
      while (n < 3 && i < Math.min(u8.length, 256)) {
        while (i < u8.length && (u8[i] === 0x20 || u8[i] === 0x0a || u8[i] === 0x0d || u8[i] === 0x09)) i++;
        if (u8[i] === 0x23) { while (i < u8.length && u8[i] !== 0x0a) i++; continue; }
        while (i < u8.length && u8[i] > 0x20) i++;
        n++;
      }
      return n === 3 ? i + 1 : null;
    } catch (e) { return null; }
  }

  /** Same rule as the server and scripts/register_saved_map.py: would ROS map_server read the UNKNOWN pixels (205) of a
      trinary map_saver_cli map as free or occupied? (map_saver_cli writes free_thresh 0.25; unknown p = 50/255 = 0.196.) */
  function thresholdCheck(yamlText, u8) {
    const t = { free: null, occ: null, negate: 0 };
    for (const raw of String(yamlText || '').split(/\r?\n/)) {
      const m = raw.replace(/#.*$/, '').trim().match(/^([A-Za-z0-9_]+)\s*:\s*(.*)$/);
      if (!m) continue;
      if (m[1] === 'free_thresh') t.free = Number(m[2]);
      else if (m[1] === 'occupied_thresh') t.occ = Number(m[2]);
      else if (m[1] === 'negate') t.negate = Number(m[2]) ? 1 : 0;
    }
    const hdr = pgmHeader(u8);
    const off = pgmDataOffset(u8);
    if (!Number.isFinite(t.free) || !Number.isFinite(t.occ) || !hdr || off == null || u8.length - off < hdr.width * hdr.height) return { judged: false };
    const n = hdr.width * hdr.height;
    let occ = 0; let unk = 0; let free = 0;
    for (let i = off; i < off + n; i++) { const v = u8[i]; if (v === 0) occ++; else if (v === 205) unk++; else if (v === 254) free++; }
    if (occ + unk + free !== n) return { judged: true, trinary: false, issue: false };
    const p = t.negate ? 205 / 255 : 50 / 255;
    const issue = !(t.free <= p && p <= t.occ);
    const fixable = issue && !t.negate && t.free > p && p <= t.occ;
    return { judged: true, trinary: true, issue, fixable, unknownCells: unk, knownCells: occ + free,
      pUnknown: Math.round(p * 100000) / 100000, free: t.free, occ: t.occ, negate: t.negate, derived: t.negate ? null : Math.floor(p * 1000) / 1000 };
  }

  /** Korean explanation for a thresholdCheck result (empty when nothing to say). */
  function thresholdMessage(c) {
    if (!c || !c.judged || !c.trinary) return { kind: '', text: '' };
    if (!c.issue) return { kind: 'ok', text: `임계값 검사 통과 (free_thresh ${c.free}, 미확인 p=${c.pUnknown}) · 확인된 칸 ${c.knownCells}개` };
    if (c.fixable) {
      return { kind: 'warn', text: `이 YAML의 free_thresh=${c.free}로는 ROS가 미확인 칸(${c.unknownCells}개)을 빈 공간으로 읽습니다(미확인 p=${c.pUnknown}). `
        + `기본 정책은 등록을 거부합니다. "보정본 등록"을 고르면 원본은 그대로 보존하고 free_thresh=${c.derived}인 복사본을 등록하며 원본·보정본 해시와 사유를 기록합니다.` };
    }
    return { kind: 'bad', text: `이 YAML의 임계값(free ${c.free}, occupied ${c.occ}, negate ${c.negate})은 미확인 칸을 빈 공간/점유로 읽게 만들며 자동 보정할 수 없습니다. map.yaml을 고쳐 다시 저장하세요.` };
  }

  /** Rows describing what the server STORED (GET /map-registry/maps/{id}/meta): the "saved result" check. */
  function evidenceRows(meta) {
    if (!meta || typeof meta !== 'object') return [];
    const m = meta;
    const ev = m.verification_evidence || {};
    const g = ev.grid_counts_pgm || {};
    const loc = ev.location || {};
    const adj = ev.yaml_adjustment;
    const rows = [
      ['위치(운영자 선언)', [loc.facility, loc.building, loc.floor, loc.area].filter(Boolean).join(' › ') + (loc.coverage ? ` (${loc.coverage === 'PARTIAL_FLOOR' ? '층 일부' : '층 전체'})` : '')],
      ['영역 연결', m.area ? `${m.area.name} (${m.area.area_id})` : '없음'],
      ['계보', m.lineage ? `${m.lineage.mapping_mode}${m.lineage.parent_map_version_id ? ' ← ' + m.lineage.parent_map_version_id : ''} · 세션 ${m.lineage.session_id}` : '없음'],
      ['확인된 칸 / 면적', g.known_cells != null ? `${g.known_cells}칸 · 약 ${g.known_area_m2} m² (자유 ${g.free}, 점유 ${g.occupied}, 미확인 ${g.unknown})` : ''],
      ['YAML', adj ? `보정본 등록: free_thresh ${adj.free_thresh_from} → ${adj.free_thresh_to}` : '업로드한 원본 그대로'],
      ['원본 YAML sha256', adj ? adj.original_yaml_sha256 : (ev.artifacts_sha256 || {})['map/map.yaml']],
      ['등록된 YAML sha256', adj ? adj.derived_yaml_sha256 : ''],
      ['업로더 · 메모', [ev.uploader, ev.note].filter(Boolean).join(' · ')],
      ['클라이언트 체크섬', ev.client_checksums_provided === true ? '전송 전 해시가 서버 계산값과 일치' : (ev.client_checksums_provided === false ? '제공되지 않음' : '')],
      ['검증 상태', `${m.verification_status || ''}${m.is_floor_active ? ' · 층 활성 지도' : ' · 층 활성 지도 아님'}`],
    ];
    return rows.filter((r) => r[1] !== undefined && r[1] !== null && r[1] !== '');
  }

  function bytesToB64(u8) {
    let s = '';
    const CH = 0x8000;
    for (let i = 0; i < u8.length; i += CH) s += String.fromCharCode.apply(null, u8.subarray(i, i + CH));
    return btoa(s);
  }

  function toHex(buf) {
    return Array.from(new Uint8Array(buf)).map((b) => b.toString(16).padStart(2, '0')).join('');
  }

  async function sha256Hex(u8) {
    const c = typeof globalThis !== 'undefined' ? globalThis.crypto : null;
    if (!c || !c.subtle) return null;
    return toHex(await c.subtle.digest('SHA-256', u8));
  }

  async function sha256OfText(text) {
    return sha256Hex(new TextEncoder().encode(text));
  }

  /** Client-side validation before any bytes are read. Returns a list of Korean error strings. */
  function validateInputs(v) {
    const errs = [];
    if (!v.floorId) errs.push('업로드 대상 층을 선택하세요(건물·층이 없으면 아래에서 먼저 등록).');
    if (!ID_RE.test(v.label || '')) errs.push('지도 이름은 영문/숫자로 시작하고 영문·숫자·. _ - 만 64자 이내로 입력하세요.');
    if (v.frameId && !FRAME_RE.test(v.frameId)) errs.push('좌표계(frame_id) 형식이 올바르지 않습니다.');
    if (!v.yaml) errs.push('map.yaml 파일을 선택하세요.');
    else if (v.yaml.size > MAX_YAML) errs.push(`map.yaml이 ${MAX_YAML} 바이트를 초과합니다.`);
    if (!v.pgm) errs.push('PGM 파일을 선택하세요.');
    else if (v.pgm.size > MAX_PGM) errs.push(`PGM이 ${MAX_PGM} 바이트를 초과합니다.`);
    if (v.posegraph && v.posegraph.size > MAX_POSEGRAPH) errs.push('posegraph 파일이 크기 제한을 초과합니다.');
    if (v.posegraphData && !v.posegraph) errs.push('posegraph.data 는 posegraph 파일과 함께만 업로드할 수 있습니다.');
    if (v.posegraphData && v.posegraphData.size > MAX_POSEGRAPH) errs.push('posegraph.data 파일이 크기 제한을 초과합니다.');
    if (v.mode === 'CONTINUE_MAP' && (!v.parentId || v.parentId === '__live__')) errs.push('CONTINUE_MAP: 지도 버전 목록에서 이어 그릴 부모 저장 지도를 먼저 선택하세요.');
    if (v.thresholdCheck && v.thresholdCheck.issue && (v.policy || 'reject') === 'reject') errs.push('임계값 검사 실패: 위 안내대로 "보정본 등록"을 선택하거나 map.yaml을 고치세요.');
    if (v.thresholdCheck && v.thresholdCheck.issue && !v.thresholdCheck.fixable) errs.push('이 YAML은 자동 보정할 수 없습니다(map.yaml을 고쳐 다시 저장).');
    const total = ((v.yaml && v.yaml.size) || 0) + ((v.pgm && v.pgm.size) || 0) + ((v.posegraph && v.posegraph.size) || 0) + ((v.posegraphData && v.posegraphData.size) || 0);
    if (Math.ceil(total * 4 / 3) > MAX_TOTAL_B64) errs.push('전체 업로드 크기가 서버 요청 한도(약 44 MiB, base64 기준)를 초과합니다.');
    return errs;
  }

  /** Map an HTTP result to a user-facing Korean message. */
  function describeError(status, body) {
    const e = (body && body.error) || {};
    const code = e.code || '';
    const msg = e.message || '';
    const tail = msg ? ` (${msg})` : '';
    if (status === 0) return '서버에 연결할 수 없습니다. 네트워크/서버 상태를 확인하세요. 같은 파일을 다시 올리면 안전하게 재시도됩니다.';
    if (status === 400) return `요청 형식 오류: 파일 내용·이름·좌표계를 확인하세요.${tail}`;
    if (status === 401) return '인증이 필요합니다(토큰 만료).';
    if (status === 403) return `권한이 없거나 허용되지 않은 값입니다. 지도 등록은 operator/admin 권한이 필요합니다.${tail}`;
    if (status === 404) return `대상(층/건물)을 찾을 수 없거나 아직 제공되지 않는 기능입니다.${tail}`;
    if (status === 409 && /area/i.test(msg)) return `영역 충돌: ${msg}`;
    if (status === 409) return `충돌: 같은 이름·픽셀의 지도가 다른 해상도/원점/좌표계/파일로 이미 등록되어 있습니다. 지도 이름(버전)을 바꿔 다시 등록하세요.${tail}`;
    if (status === 413) return `파일이 너무 큽니다. 서버 크기 제한을 초과했습니다.${tail}`;
    if (status === 422) {
      if (code === 'checksum_mismatch') return `체크섬 불일치: 전송 중 파일이 손상되었습니다. 다시 시도하세요.${tail}`;
      if (e.detail && e.detail.code === 'unknown_pixels_misclassified') {
        const d = e.detail;
        return `임계값 안전 검사 거부: free_thresh ${d.free_thresh}로는 미확인 칸 ${d.unknown_cells}개를 ${d.free_thresh > d.p_unknown ? '빈 공간' : '점유'}으로 읽습니다.`
          + (d.fixable_by_free_thresh ? ' "보정본 등록"을 선택하면 free_thresh=' + d.derived_free_thresh + '인 복사본으로 등록합니다(원본 보존).' : ' 자동 보정할 수 없으니 map.yaml을 고쳐 다시 저장하세요.');
      }
      return `정책 위반 또는 검증 실패(예: YAML/PGM 비정상).${tail}`;
    }
    if (status === 429) return '인증 실패가 반복되어 잠시 차단되었습니다. 잠시 후 다시 시도하세요.';
    if (status === 503) return '다른 지도 업로드가 처리 중입니다. 잠시 후 다시 시도하세요.';
    return `업로드 실패 (HTTP ${status})${tail}`;
  }

  /** Describe an upload result for the result panel. Returns {kind, title, rows}. */
  function describeResult(status, body) {
    const mv = (body && body.map_version) || {};
    let kind = 'ok';
    let title = '신규 지도 버전이 UNVERIFIED 상태로 등록되었습니다.';
    if (body && body.idempotent_replay) { title = '같은 요청(멱등 키)이 이미 처리되어 이전 결과를 다시 표시합니다.'; kind = 'warn'; }
    else if (status === 200 && body && body.idempotent) { title = '동일한 지도가 이미 등록되어 있습니다(중복 — 변경 없음).'; kind = 'warn'; }
    const o = Array.isArray(mv.origin) ? mv.origin : [];
    const sha = mv.sha256 || {};
    const rows = [
      ['지도 버전 ID', mv.map_version_id],
      ['지도 이름', mv.label],
      ['검증 상태', mv.verification_status],
      ['좌표계(frame_id)', mv.frame_id],
      ['해상도', mv.resolution != null ? `${mv.resolution} m/px` : ''],
      ['원점(x, y, yaw)', o.length === 3 ? `${o[0]}, ${o[1]}, ${o[2]}` : ''],
      ['크기', mv.width != null ? `${mv.width} × ${mv.height} px` : ''],
      ['픽셀 content_sha256', sha.content_sha256],
      ['번들 bundle_sha256', mv.bundle_sha256 || sha.bundle_sha256],
      ['YAML sha256', sha.yaml_sha256],
      ['PGM sha256', sha.pgm_sha256],
      ['영역', body && body.area_id],
      ['계보', body && body.lineage ? `${body.lineage.mapping_mode}${body.lineage.parent_map_version_id ? ' ← ' + body.lineage.parent_map_version_id : ''}` : ''],
      ['YAML 보정', body && body.yaml_adjustment ? `free_thresh ${body.yaml_adjustment.free_thresh_from} → ${body.yaml_adjustment.free_thresh_to}` : ''],
    ].filter((r) => r[1] !== undefined && r[1] !== null && r[1] !== '');
    return { kind, title, rows };
  }

  function formatTs(ts) {
    if (!Number.isFinite(Number(ts))) return '';
    return new Date(Number(ts) * 1000).toLocaleString('ko-KR', { hour12: false });
  }

  // ---------------------------------------------------------------- DOM wiring (browser only)
  function init(ctx) {
    if (typeof document === 'undefined') return;
    const $ = (id) => document.getElementById(id);
    if (!$('regUploadPanel')) return;
    let busy = false;
    let thresholdState = null;      // thresholdCheck() of the files currently selected

    function canWrite() {
      const me = ctx.getMe();
      return Boolean(me && (me.role === 'operator' || me.role === 'admin'));
    }

    function setStatus(el, text, cls) {
      el.textContent = text || '';
      el.className = 'small ' + (cls || 'muted');
    }

    async function post(path, body, idem) {
      let r;
      try {
        r = await ctx.api(path, {
          method: 'POST',
          body: JSON.stringify(body),
          headers: idem ? { 'Idempotency-Key': idem } : {},
          timeoutMs: 180000,
        });
      } catch (e) {
        if (String(e && e.message) === '401') return { status: 401, body: {} };
        return { status: 0, body: {} };
      }
      let j = {};
      try { j = await r.json(); } catch (e) { j = {}; }
      return { status: r.status, body: j };
    }

    function renderRows(rows) {
      return '<table class="reg-kv">' + rows.map((r) => `<tr><th>${esc(r[0])}</th><td class="mono">${esc(r[1])}</td></tr>`).join('') + '</table>';
    }

    function update() {
      const sel = ctx.getSelection();
      const floor = ctx.getFloorInfo();
      const w = canWrite();
      $('regUpFloor').textContent = floor ? `${floor.building} / ${floor.level} (${floor.floorId})` : '선택된 층 없음';
      $('regUpFloor').className = floor ? 'mono' : 'warn';
      const blocked = !!(thresholdState && thresholdState.issue && ($('regUpPolicy').value === 'reject' || !thresholdState.fixable));
      $('regUpBtn').disabled = busy || !w || !sel.floorId || blocked;
      $('regUpBtn').title = blocked ? '임계값 안전 검사에 걸렸습니다(위 안내 참고)' : '';
      ['regFacBtn', 'regBldBtn', 'regFlrBtn', 'regAreaBtn'].forEach((id) => { $(id).disabled = busy || !w; });
      $('regAreaFloor').textContent = floor ? `${floor.building} / ${floor.level} (${floor.floorId})` : '선택된 층 없음';
      const areas = (ctx.getAreas && ctx.getAreas()) || [];
      const asel = $('regUpArea');
      const aopts = '<option value="">영역 없음</option>' + areas.map((a) => `<option value="${esc(a.area_id)}">${esc(a.name || a.area_id)}${a.coverage === 'PARTIAL_FLOOR' ? ' (층 일부)' : ''}</option>`).join('');
      if (asel.dataset.opts !== aopts) {
        const prevA = asel.value || (sel.areaId || '');
        asel.innerHTML = aopts;
        asel.dataset.opts = aopts;
        if (prevA && areas.some((a) => a.area_id === prevA)) asel.value = prevA;
      }
      $('regUpRole').textContent = w ? '' : '읽기 전용 권한(viewer): 지도 등록은 operator/admin만 가능합니다. 이력 조회는 가능합니다.';
      const facs = ctx.getFacilities();
      const fsel = $('regBldFac');
      const optHtml = facs.length ? facs.map((f) => `<option value="${esc(f.facility_id)}">${esc(f.name || f.facility_id)}</option>`).join('') : '<option value="">시설 없음</option>';
      if (fsel.dataset.opts !== optHtml) {
        const prev = fsel.value;
        fsel.innerHTML = optHtml;
        fsel.dataset.opts = optHtml;
        if (prev && facs.some((f) => f.facility_id === prev)) fsel.value = prev;
      }
      const bsel = ctx.getBuildingInfo();
      $('regFlrBld').textContent = bsel ? `${bsel.name || bsel.building_id} (${bsel.building_id})` : '선택된 건물 없음';
    }

    async function previewFiles() {
      const box = $('regUpPreview');
      const warns = [];
      const parts = [];
      const y = $('regUpYaml').files[0];
      const p = $('regUpPgm').files[0];
      if (y && y.size <= MAX_YAML) {
        const s = summarizeYaml(await y.text());
        parts.push(`해상도 ${s.resolution ?? '?'} m/px · 원점 ${s.origin ? s.origin.join(', ') : '?'} · frame_id ${s.frame_id || 'map(기본)'}`);
        warns.push(...s.warnings);
      } else if (y) warns.push('map.yaml이 크기 제한을 초과합니다.');
      if (p && p.size <= MAX_PGM) {
        const h = pgmHeader(new Uint8Array(await p.slice(0, 256).arrayBuffer()));
        if (h) parts.push(`PGM ${h.width} × ${h.height} px (maxval ${h.maxval})`);
        else warns.push('PGM(P5) 헤더를 읽을 수 없습니다. 올바른 PGM 파일인지 확인하세요.');
      } else if (p) warns.push('PGM이 크기 제한을 초과합니다.');
      thresholdState = null;
      let tmsg = { kind: '', text: '' };
      if (y && p && y.size <= MAX_YAML && p.size <= MAX_PGM) {
        thresholdState = thresholdCheck(await y.text(), new Uint8Array(await p.arrayBuffer()));
        tmsg = thresholdMessage(thresholdState);
      }
      box.innerHTML = (parts.length ? `<div>${esc(parts.join(' | '))}</div>` : '') + warns.map((w) => `<div class="warn">⚠ ${esc(w)}</div>`).join('')
        + (tmsg.text ? `<div class="reg-warn ${tmsg.kind === 'bad' ? 'bad' : ''} ${tmsg.kind === 'ok' ? 'ok' : ''}">${esc(tmsg.text)}</div>` : '');
      update();
    }

    ['regUpYaml', 'regUpPgm'].forEach((id) => { $(id).addEventListener('change', () => { previewFiles().catch(() => {}); }); });
    ['regUpPolicy', 'regUpMode'].forEach((id) => { $(id).addEventListener('change', () => update()); });

    async function readU8(file) { return new Uint8Array(await file.arrayBuffer()); }

    async function doUpload() {
      if (busy) return;
      const res = $('regUpResult');
      const st = $('regUpStatus');
      res.innerHTML = '';
      const sel = ctx.getSelection();
      const v = {
        floorId: sel.floorId,
        label: $('regUpLabel').value.trim(),
        frameId: $('regUpFrame').value.trim(),
        yaml: $('regUpYaml').files[0],
        pgm: $('regUpPgm').files[0],
        posegraph: $('regUpPg').files[0],
        posegraphData: $('regUpPgData').files[0],
        mode: $('regUpMode').value,
        parentId: sel.mapVersionId,
        policy: $('regUpPolicy').value,
        thresholdCheck: thresholdState,
      };
      const errs = validateInputs(v);
      if (errs.length) { setStatus(st, errs.join(' / '), 'bad'); return; }
      busy = true; update();
      try {
        setStatus(st, '파일을 읽고 체크섬을 계산하는 중...', 'muted');
        const yb = await readU8(v.yaml);
        const pb = await readU8(v.pgm);
        const pgb = v.posegraph ? await readU8(v.posegraph) : null;
        const pgdb = v.posegraphData ? await readU8(v.posegraphData) : null;
        const ysha = await sha256Hex(yb);
        const psha = await sha256Hex(pb);
        const pgsha = pgb ? await sha256Hex(pgb) : null;
        const pgdsha = pgdb ? await sha256Hex(pgdb) : null;
        const body = {
          floor_id: v.floorId,
          label: v.label,
          source_mode: $('regUpSource').value,
          yaml_b64: bytesToB64(yb),
          pgm_b64: bytesToB64(pb),
        };
        if (v.frameId) body.frame_id = v.frameId;
        const areaId = $('regUpArea').value;
        if (areaId) body.area_id = areaId;
        body.lineage = { mapping_mode: v.mode };
        if (v.mode === 'CONTINUE_MAP') body.lineage.parent_map_version_id = v.parentId;
        body.threshold_policy = v.policy;
        const note = $('regUpNote').value.trim();
        if (note) body.note = note;
        if (pgb) body.posegraph_b64 = bytesToB64(pgb);
        if (pgdb) body.posegraph_data_b64 = bytesToB64(pgdb);
        let idem = null;
        if (ysha && psha) {
          body.expected_checksums = { yaml_sha256: ysha, pgm_sha256: psha };
          if (pgsha) body.expected_checksums.posegraph_sha256 = pgsha;
          if (pgdsha) body.expected_checksums.posegraph_data_sha256 = pgdsha;
          // Deterministic key: re-sending the same files is a safe replay; different files never collide.
          idem = 'up-' + (await sha256OfText([v.floorId, v.label, body.source_mode, v.frameId, ysha, psha, pgsha || '', pgdsha || '',
            areaId, v.mode, v.parentId || '', v.policy, note].join('|'))).slice(0, 40);
        } else {
          idem = 'up-' + Date.now().toString(16) + Math.random().toString(16).slice(2, 10);
        }
        setStatus(st, '업로드 중... (UNVERIFIED로 등록되며 로봇에는 적용되지 않습니다)', 'muted');
        const r = await post('/map-registry/maps', body, idem);
        if (r.status === 200 || r.status === 201) {
          const d = describeResult(r.status, r.body);
          setStatus(st, d.title, d.kind === 'ok' ? 'ok' : 'warn');
          let html = renderRows(d.rows);
          try {
            const mid = r.body && r.body.map_version && r.body.map_version.map_version_id;
            if (mid) {
              const mr = await ctx.api(`/map-registry/maps/${encodeURIComponent(mid)}/meta`);
              if (mr.ok) html += '<div class="small" style="margin-top:6px"><b>서버에 저장된 결과</b></div>' + renderRows(evidenceRows(await mr.json()));
            }
          } catch (e) { /* the registration itself succeeded; the stored-result view is best effort */ }
          if (ysha) {
            html += r.body && r.body.yaml_adjustment
              ? '<div class="muted small">전송 전 클라이언트 체크섬(원본 yaml/pgm)이 서버 계산값과 일치했습니다. YAML은 보정본으로 등록되었고 원본 해시는 위에 기록되어 있습니다.</div>'
              : '<div class="muted small">전송 전 클라이언트 체크섬(yaml/pgm)이 서버 계산값과 일치하여 등록되었습니다.</div>';
          }
          html += '<div class="alert-banner warn-banner" style="margin-top:6px">등록만 완료되었습니다. 이 지도는 UNVERIFIED이며 로봇에 적재·전환되지 않았고 이동·임무에 사용할 수 없습니다.</div>';
          res.innerHTML = html;
          await ctx.refresh(v.floorId);
        } else {
          setStatus(st, describeError(r.status, r.body), 'bad');
        }
      } catch (e) {
        setStatus(st, '파일을 읽는 중 오류가 발생했습니다: ' + esc(e && e.message), 'bad');
      } finally {
        busy = false; update();
      }
    }

    async function hierarchyPost(path, body, statusId, key) {
      const st = $(statusId);
      if (busy) return;
      busy = true; update();
      try {
        setStatus(st, '등록 중...', 'muted');
        const r = await post(path, body, key);
        if (r.status === 200 || r.status === 201) {
          setStatus(st, r.body && r.body.idempotent_replay ? '이미 처리된 요청입니다.' : '등록되었습니다.', 'ok');
          await ctx.refresh();
        } else setStatus(st, describeError(r.status, r.body), 'bad');
      } finally { busy = false; update(); }
    }

    $('regUpBtn').onclick = doUpload;
    $('regFacBtn').onclick = () => hierarchyPost('/map-registry/facilities', {
      facility_id: $('regFacId').value.trim(), name: $('regFacName').value.trim(),
    }, 'regHierStatus');
    $('regBldBtn').onclick = () => hierarchyPost('/map-registry/buildings', {
      building_id: $('regBldId').value.trim(), facility_id: $('regBldFac').value, name: $('regBldName').value.trim(),
    }, 'regHierStatus');
    $('regFlrBtn').onclick = () => {
      const b = ctx.getBuildingInfo();
      const elev = Number($('regFlrElev').value || 0);
      hierarchyPost('/map-registry/floors', {
        floor_id: $('regFlrId').value.trim(), building_id: b ? b.building_id : '',
        level_name: $('regFlrLevel').value.trim(), elevation_m: Number.isFinite(elev) ? elev : 0,
      }, 'regHierStatus');
    };

    $('regAreaBtn').onclick = () => {
      const sel = ctx.getSelection();
      if (!sel.floorId) { setStatus($('regAreaStatus'), '영역을 만들 층을 먼저 선택하세요.', 'bad'); return; }
      const id = $('regAreaId').value.trim();
      const name = $('regAreaName').value.trim();
      if (!ID_RE.test(id)) { setStatus($('regAreaStatus'), 'area_id는 영문/숫자로 시작하고 영문·숫자·. _ - 만 64자 이내로 입력하세요.', 'bad'); return; }
      if (!name) { setStatus($('regAreaStatus'), '영역 이름을 입력하세요.', 'bad'); return; }
      hierarchyPost('/map-registry/areas', { area_id: id, floor_id: sel.floorId, name, coverage: $('regAreaCoverage').value }, 'regAreaStatus');
    };

    $('regHistBtn').onclick = async () => {
      const sel = ctx.getSelection();
      const out = $('regHistBody');
      if (!sel.mapVersionId || sel.mapVersionId === '__live__') {
        out.innerHTML = '<div class="warn small">지도 버전 목록에서 저장된 지도를 먼저 선택하세요(라이브 지도는 이력이 없습니다).</div>';
        return;
      }
      const enc = encodeURIComponent(sel.mapVersionId);
      out.innerHTML = '<div class="muted small">조회 중...</div>';
      try {
        const [mr, hr] = await Promise.all([ctx.api(`/map-registry/maps/${enc}`), ctx.api(`/map-registry/maps/${enc}/history`)]);
        if (!mr.ok || !hr.ok) {
          const j = await (mr.ok ? hr : mr).json().catch(() => ({}));
          out.innerHTML = `<div class="bad small">${esc(describeError((mr.ok ? hr : mr).status, j))}</div>`;
          return;
        }
        const m = (await mr.json()).map || {};
        const h = (await hr.json()).history || [];
        const q = m.quarantine || {};
        let html = `<div class="small">검증 상태 <span class="badge ${m.verification_status === 'VERIFIED' ? 'ok' : (m.verification_status === 'UNVERIFIED' ? 'warn' : 'bad')}">${esc(m.verification_status)}</span>`
          + ` · 격리 ${q.is_quarantined ? '<span class="badge bad">격리됨</span>' : '<span class="badge">없음</span>'}`
          + ` · 층 활성 포인터 ${m.is_floor_active ? '있음' : '없음'} · 좌표계 ${esc(m.frame_id || 'map')}</div>`;
        html += '<div class="muted small">검증 승인·철회·격리 해제는 웹에서 제공되지 않습니다(정책 확정 전).</div>';
        html += '<table><thead><tr><th>#</th><th>시각</th><th>전이</th><th>수행자</th><th>사유</th></tr></thead><tbody>'
          + (h.length ? h.map((x) => `<tr><td>${esc(x.seq)}</td><td>${esc(formatTs(x.ts))}</td><td>${esc(x.from_status)} → ${esc(x.to_status)}</td><td>${esc(x.actor)} (${esc(x.actor_role)})</td><td>${esc(x.reason)}</td></tr>`).join('') : '<tr><td colspan="5" class="muted">이력 없음</td></tr>')
          + '</tbody></table>';
        out.innerHTML = html;
      } catch (e) {
        out.innerHTML = '<div class="bad small">서버에 연결할 수 없습니다.</div>';
      }
    };

    update();
    return { update };
  }

  let instance = null;
  return {
    init(ctx) { instance = init(ctx); return instance; },
    update() { if (instance) instance.update(); },
    summarizeYaml, pgmHeader, pgmDataOffset, thresholdCheck, thresholdMessage, evidenceRows, bytesToB64, sha256Hex, validateInputs, describeError, describeResult,
    MAX_YAML, MAX_PGM, MAX_POSEGRAPH,
  };
});
