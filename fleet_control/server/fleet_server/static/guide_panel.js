'use strict';
/* Visitor guide administration card (Phase 5, 2026-11-21) in the operator dashboard.
   - Guide service open / close for the selected robot (the server requires the lease holder; visitor missions run under it).
   - Destinations: list with availability / reason; add one from the map-click goal fields on the robot's LOADED map version
     (the server validates the pose with the same goal check as missions and refuses otherwise).
   - Admin: issue / revoke visitor links (QR + plain URL; the token is shown once). */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.GuidePanel = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  function mount(host, opts) {
    host.innerHTML = `<div class="sec-head"><h2>방문객 안내 서비스</h2><span class="badge" data-gp="svc">확인 중</span></div>
      <div class="toolbar-row small"><button type="button" data-gp="open">안내 서비스 시작</button><button type="button" data-gp="close">중지</button></div>
      <h3 class="small">목적지</h3><table class="small"><thead><tr><th>이름</th><th>지도</th><th>안내</th></tr></thead><tbody data-gp="dests"></tbody></table>
      <div class="toolbar-row small"><label>이름 <input data-gp="name" class="in-s" maxlength="80"/></label><label>설명 <input data-gp="desc" maxlength="200"/></label>
        <label>구역 ID <input data-gp="area" class="in-s"/></label><button type="button" data-gp="add">지도 클릭 좌표로 목적지 추가</button></div>
      <div data-gp="admin" class="hidden"><h3 class="small">방문객 링크(QR)</h3>
        <div class="toolbar-row small"><label>이름 <input data-gp="label" class="in-s" value="안내데스크"/></label><label>유효(시간) <input data-gp="ttl" type="number" min="1" max="720" value="168" class="in-s"/></label>
          <button type="button" data-gp="issue">링크 발급</button></div>
        <div data-gp="qr"></div><ul class="block-list small" data-gp="links"></ul></div>
      <p class="small" data-gp="result" role="status"></p>`;
    const q = n => host.querySelector(`[data-gp="${n}"]`);
    const say = t => { q('result').textContent = t; };
    async function call(path, body) {
      const r = await opts.request(path, body ? { method: 'POST', body: JSON.stringify(body) } : undefined);
      const j = await r.json().catch(() => ({})); if (!r.ok) throw new Error(j.error?.message || `HTTP ${r.status}`); return j;
    }
    async function load() {
      const robot = opts.getRobot(); const me = opts.getMe();
      q('admin').classList.toggle('hidden', !(me && me.role === 'admin'));
      if (robot) try {
        const s = await call(`/guide/service/${encodeURIComponent(robot.robot_id)}`);
        q('svc').textContent = s.effective ? '운영 중' : (s.enabled ? `중지됨 (${s.reason})` : '중지'); q('svc').className = 'badge ' + (s.effective ? 'ok' : 'off');
        q('svc').dataset.effective = String(s.effective);
      } catch (_) {}
      try {
        const { destinations } = await call('/guide/destinations');
        q('dests').innerHTML = destinations.map(d => `<tr data-dest="${esc(d.destination_id)}"><td>${esc(d.name)}</td><td title="${esc(d.map_version_id)}">${esc(String(d.map_version_id).split('#')[0])}</td>
          <td>${d.available ? '가능' : esc(d.reason_text || d.reason)}</td></tr>`).join('') || '<tr><td colspan="3" class="muted">등록된 목적지 없음</td></tr>';
      } catch (_) {}
      if (me && me.role === 'admin') try {
        const { links } = await call('/visitor-links');
        q('links').innerHTML = links.map(l => `<li>${esc(l.label)} · ${l.active ? '유효' : '만료/폐기'} · ${new Date(l.expires_at * 1000).toLocaleString()} ${l.active ? `<button type="button" data-revoke="${esc(l.link_id)}">폐기</button>` : ''}</li>`).join('');
      } catch (_) {}
    }
    host.addEventListener('click', async ev => {
      const t = ev.target; const robot = opts.getRobot();
      try {
        if (t === q('open') || t === q('close')) { if (!robot) return; await call('/guide/service', { robot_id: robot.robot_id, enabled: t === q('open') }); say('반영됨'); }
        else if (t === q('add')) {
          const nav = robot && robot.navigation; const goal = opts.getGoal();
          const mv = nav && (nav.map_version_id || nav.request?.map_version_id);
          if (!mv) { say('먼저 NAVIGATION에서 지도를 로딩하세요(목적지는 로딩된 지도 버전에 묶입니다).'); return; }
          const d = await call('/guide/destinations', { name: q('name').value, description: q('desc').value, area_id: q('area').value || undefined,
            map_version_id: mv, x: goal.x, y: goal.y, yaw: goal.yaw || 0 });
          say(`목적지 추가: ${d.name} (${d.available ? '안내 가능' : d.reason_text || d.reason})`);
        } else if (t === q('issue')) {
          const l = await call('/visitor-links', { label: q('label').value, ttl_s: Math.round(Number(q('ttl').value || 168) * 3600) });
          const url = location.origin + l.path;
          q('qr').innerHTML = (window.QrEncode ? window.QrEncode.svg(url, 4) : '') + `<p class="small">QR 코드 아래 주소를 함께 안내하세요(한 번만 표시): <code data-gp="url">${esc(url)}</code></p>`;
          say('방문객 링크 발급됨');
        } else if (t.dataset && t.dataset.revoke) { await call(`/visitor-links/${t.dataset.revoke}/revoke`, {}); say('링크 폐기됨'); }
        else return;
      } catch (e) { say('거부: ' + e.message); }
      load();
    });
    const timer = setInterval(load, 3000); load();
    return { refresh: load, destroy() { clearInterval(timer); } };
  }
  return { mount };
});
