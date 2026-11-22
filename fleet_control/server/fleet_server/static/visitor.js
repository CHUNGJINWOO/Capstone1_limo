'use strict';
/* Visitor guide page (Phase 5, 2026-11-21). Mobile first. Uses ONLY /api/v1/visitor/* with the visitor link token
   (`Authorization: Visitor <token>`) and a random per-browser session id. It cannot reach any operator endpoint: the server
   refuses this token everywhere else. The page never shows coordinates, robots' internals or diagnostics. */
(function () {
  const $ = id => document.getElementById(id);
  const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  const store = (k, v) => { try { if (v === undefined) return localStorage.getItem(k); if (v === null) localStorage.removeItem(k); else localStorage.setItem(k, v); } catch (_) { return null; } };
  // token: from the QR link fragment (#t=...), kept for this tab only and removed from the address bar
  const m = /[#&]t=([A-Za-z0-9_-]{16,64})/.exec(location.hash);
  if (m) { try { sessionStorage.setItem('visitor_token', m[1]); } catch (_) {} history.replaceState(null, '', location.pathname); }
  let token = null; try { token = sessionStorage.getItem('visitor_token'); } catch (_) {}
  // Per-link namespace (Codex 2026-11-21): two visitor links opened in one browser never share a session id or a remembered request.
  // FNV-1a of the token: an identifier for the key only (the token itself stays in sessionStorage, never in localStorage).
  const ns = (t => { let h = 0x811c9dc5; for (const c of String(t || '')) { h ^= c.codePointAt(0); h = Math.imul(h, 0x01000193) >>> 0; } return h.toString(16).padStart(8, '0'); })(token);
  const K_SESSION = `visitor_session:${ns}`, K_REQUEST = `visitor_request:${ns}`;
  try { localStorage.removeItem('visitor_session'); localStorage.removeItem('visitor_request'); } catch (_) {}   // pre-fix global keys
  let session = store(K_SESSION);
  if (!session || !/^[A-Za-z0-9_-]{16,64}$/.test(session)) {
    const a = new Uint8Array(18); crypto.getRandomValues(a); session = Array.from(a, b => (b % 36).toString(36)).join(''); store(K_SESSION, session);
  }
  const state = { catalog: null, floor: '', area: '', q: '', request: store(K_REQUEST), timer: null };

  async function api(path, opt = {}) {
    const r = await fetch('/api/v1/visitor' + path, { method: opt.method || 'GET', body: opt.body ? JSON.stringify(opt.body) : undefined,
      headers: { 'Authorization': 'Visitor ' + token, 'X-Visitor-Session': session, 'Content-Type': 'application/json' } });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) { const e = new Error(j.error?.message || `HTTP ${r.status}`); e.status = r.status; e.code = j.error?.code; throw e; }
    return j;
  }
  const show = id => ['vLink', 'vPick', 'vReq'].forEach(x => $(x).classList.toggle('hidden', x !== id));
  const err = t => { $('vError').textContent = t || ''; };

  function linkProblem(why) { show('vLink'); $('vLinkWhy').textContent = why || ''; $('vService').textContent = '안내 링크 확인 필요'; }

  async function loadCatalog() {
    const c = await api('/catalog'); state.catalog = c;
    $('vService').textContent = c.service_open ? '안내 서비스 운영 중' : '현재 안내 서비스가 운영 중이 아닙니다';
    const sel = $('vFloor');
    sel.innerHTML = c.floors.map(f => `<option value="${esc(f.floor_id)}">${esc(f.facility)} · ${esc(f.building)} · ${esc(f.level_name)}</option>`).join('') || '<option value="">안내 가능한 장소 없음</option>';
    if (!state.floor || !c.floors.some(f => f.floor_id === state.floor)) state.floor = c.floors[0]?.floor_id || '';
    sel.value = state.floor; renderAreas();
  }
  function renderAreas() {
    const f = (state.catalog?.floors || []).find(x => x.floor_id === state.floor);
    const areas = f ? f.areas : [];
    if (!areas.some(a => a.area_id === state.area)) state.area = '';
    $('vAreas').innerHTML = [`<button type="button" data-area="" aria-pressed="${state.area === ''}">전체</button>`]
      .concat(areas.map(a => `<button type="button" data-area="${esc(a.area_id)}" aria-pressed="${state.area === a.area_id}">${esc(a.name)}</button>`)).join('');
  }
  async function loadList() {
    const qs = new URLSearchParams(); if (state.floor) qs.set('floor_id', state.floor); if (state.area) qs.set('area_id', state.area); if (state.q) qs.set('q', state.q);
    const { destinations } = await api('/destinations?' + qs.toString());
    $('vList').innerHTML = destinations.map(d => `<li class="v-item" data-dest="${esc(d.destination_id)}" data-available="${d.available}">
        <b>${esc(d.name)}</b><button type="button" data-go="${esc(d.destination_id)}" ${d.available ? '' : 'disabled'} aria-label="${esc(d.name)}(으)로 안내 요청">안내 요청</button>
        <span class="v-desc">${esc(d.description || '')}</span>${d.available ? '' : `<span class="v-why">이용 불가: ${esc(d.reason_text || d.reason)}</span>`}</li>`).join('')
      || '<li class="v-muted">조건에 맞는 목적지가 없습니다.</li>';
  }
  const ORDER = ['RECEIVED', 'ACCEPTED', 'IN_PROGRESS', 'COMPLETED'];
  function renderRequest(r) {
    show('vReq'); $('vReqDest').textContent = `목적지: ${r.destination_name || ''}`;
    $('vReqStatus').textContent = r.status_text + (r.status === 'QUEUED' && r.queue_position ? ` (대기 ${r.queue_position}번째)` : '');
    $('vReqStatus').dataset.status = r.status;
    $('vReqReason').textContent = r.reason_text || '';
    const idx = ORDER.indexOf(r.status === 'QUEUED' ? 'RECEIVED' : r.status === 'CANCELING' ? 'IN_PROGRESS' : r.status);
    document.querySelectorAll('#vSteps li').forEach((li, i) => { li.classList.toggle('done', idx >= 0 && i < idx || r.status === 'COMPLETED'); li.classList.toggle('now', i === idx && r.status !== 'COMPLETED'); });
    $('vCancel').classList.toggle('hidden', r.terminal); $('vCancel').disabled = r.status === 'CANCELING';
    $('vAgain').classList.toggle('hidden', !r.terminal);
  }
  async function poll() {
    clearTimeout(state.timer);
    if (!state.request) return;
    try {
      const r = await api('/requests/' + state.request); renderRequest(r); err('');
      if (!r.terminal) state.timer = setTimeout(poll, 2000);
    } catch (e) {
      if (e.status === 401) return linkProblem(e.message);
      if (e.status === 404) { store(K_REQUEST, null); state.request = null; return pick(); }
      err('연결이 불안정합니다. 잠시 후 다시 확인합니다.'); state.timer = setTimeout(poll, 4000);
    }
  }
  async function pick() {
    show('vPick');
    try { await loadCatalog(); await loadList(); err(''); }
    catch (e) { if (e.status === 401) return linkProblem(e.message); err('목적지를 불러오지 못했습니다.'); }
  }
  $('vFloor').addEventListener('change', e => { state.floor = e.target.value; state.area = ''; renderAreas(); loadList().catch(() => err('목적지를 불러오지 못했습니다.')); });
  $('vAreas').addEventListener('click', e => { const b = e.target.closest('button[data-area]'); if (!b) return; state.area = b.dataset.area; renderAreas(); loadList().catch(() => {}); });
  let qTimer = null;
  $('vSearch').addEventListener('input', e => { clearTimeout(qTimer); qTimer = setTimeout(() => { state.q = e.target.value.trim().slice(0, 40); loadList().catch(() => {}); }, 250); });
  $('vList').addEventListener('click', async e => {
    const b = e.target.closest('button[data-go]'); if (!b) return;
    b.disabled = true;
    try {
      const r = await api('/requests', { method: 'POST', body: { destination_id: b.dataset.go } });
      state.request = r.request_id; store(K_REQUEST, r.request_id); renderRequest(r); err(''); if (!r.terminal) poll();
    } catch (ex) {
      if (ex.status === 401) return linkProblem(ex.message);
      err(ex.code === 'request_active' ? '이미 진행 중인 안내 요청이 있습니다.' : ex.code === 'rate_limited' ? '요청이 많습니다. 잠시 후 다시 시도해 주세요.' : '요청을 보내지 못했습니다.');
      b.disabled = false;
    }
  });
  $('vCancel').addEventListener('click', async () => {
    if (!state.request || !window.confirm('안내 요청을 취소할까요?')) return;
    try { renderRequest(await api(`/requests/${state.request}/cancel`, { method: 'POST', body: {} })); poll(); } catch (e) { err('취소 요청을 보내지 못했습니다.'); }
  });
  $('vAgain').addEventListener('click', () => { store(K_REQUEST, null); state.request = null; pick(); });
  if (!token) linkProblem('');
  else if (state.request) poll(); else pick();
})();
