'use strict';
const API = '/api/v1'; let token = sessionStorage.getItem('fleet_token') || '', role = '', data = null, mapImg = {}, mapVer = {};
const $ = id => document.getElementById(id);
const uuid = () => ([1e7]+-1e3+-4e3+-8e3+-1e11).replace(/[018]/g, c => (c ^ (Math.random() * 16) >> c / 4).toString(16));   // crypto.randomUUID needs a secure context (plain http on the tailnet is not)
const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
async function api(path, opt = {}) {
  const r = await fetch(API + path, {...opt, headers: {'Authorization': 'Bearer ' + token, 'Content-Type': 'application/json', ...(opt.headers || {})}});
  if (r.status === 401) { showLogin('토큰이 올바르지 않습니다'); throw new Error('401'); }
  return r;
}
function showLogin(msg) { $('login').classList.remove('hidden'); $('main').classList.add('hidden'); $('loginerr').textContent = msg || ''; $('logout').classList.add('hidden'); }
$('go').onclick = async () => { token = $('tok').value.trim(); $('tok').value = ''; sessionStorage.setItem('fleet_token', token); await start(); };
$('logout').onclick = () => { sessionStorage.removeItem('fleet_token'); token = ''; showLogin(''); };
async function start() {
  if (!token) return showLogin('');
  const r = await fetch(API + '/me', {headers: {'Authorization': 'Bearer ' + token}});
  if (!r.ok) { sessionStorage.removeItem('fleet_token'); return showLogin(r.status === 401 ? '토큰이 올바르지 않습니다' : '접속 실패 ' + r.status); }
  const me = await r.json(); role = me.role; $('who').textContent = `${me.name} (${me.role})`; $('logout').classList.remove('hidden');
  $('login').classList.add('hidden'); $('main').classList.remove('hidden');
  const canCtl = role === 'operator' || role === 'admin'; $('mform').querySelectorAll('input,select,button').forEach(e => e.disabled = !canCtl);
  if (!canCtl) $('merr').textContent = 'viewer 권한: 임무를 만들거나 취소할 수 없습니다';
  tick(); if (!window._t) window._t = setInterval(tick, 1000);
}
const cls = s => ({'OK':'ok','NORMAL':'ok','READY':'ok','ONLINE':'ok','STATIC':'ok','UPDATING':'ok','VOLTAGE_ONLY':'warn','ACTIVE_GOAL':'ok'})[s] || (s === 'NO_DATA' || s === 'UNKNOWN' || s === 'UNAVAILABLE' || s === 'NO_MAP' ? 'muted' : (s ? 'bad' : 'muted'));
const fmt = (v, d = 2) => (v === null || v === undefined || Number.isNaN(v)) ? '-' : Number(v).toFixed(d);
function robotCard(r) {
  const s = r.state || {}, a = r.agent || {}, p = s.pose || {}, b = s.battery || {}, l = s.lidar || {}, d = s.depth_camera || {}, g = s.safety_gate || {}, n = s.nav2 || {}, m = s.map || {}, ms = s.mission || {};
  const row = (k, v, c) => `<div>${k}</div><div class="${c || ''}">${v}</div>`;
  return `<div class="card rob"><h3>${esc(r.robot_id)}<span class="badge ${r.online ? 'on' : 'off'}">${r.online ? 'ONLINE' : 'OFFLINE'}</span>${a.simulated ? '<span class="badge sim">SIMULATED</span>' : ''}<span class="badge">${esc(a.mode || '')}</span></h3>
  <div class="grid">${row('마지막 수신', r.last_seen_age_s === null ? '-' : r.last_seen_age_s + ' s 전', r.online ? '' : 'bad')}
  ${row('위치 (' + esc(p.frame_id || '-') + ')', p.status === 'OK' || p.status === 'STALE' ? `x ${fmt(p.x)}  y ${fmt(p.y)}  yaw ${fmt(p.yaw_rad)} rad` : esc(p.status || 'NO_DATA'), cls(p.status))}
  ${row('지도', `${esc(m.status || 'NO_MAP')}${m.version ? ' v' + m.version : ''}${m.known_cells ? ' · ' + m.known_cells + ' cells' : ''}${m.last_update_age_s != null ? ' · 갱신 ' + fmt(m.last_update_age_s, 0) + ' s 전' : ''}`, cls(m.status))}
  ${row('Nav2', esc(n.status || 'UNAVAILABLE') + (n.action_server_ready ? ' (action server ready)' : ''), cls(n.status))}
  ${row('배터리', b.voltage_v != null ? `${fmt(b.voltage_v)} V (백분율 미확인)` : 'UNKNOWN', cls(b.status))}
  ${row('LiDAR', esc(l.status || 'NO_DATA') + (l.rate_hz ? ` · ${fmt(l.rate_hz, 1)} Hz · ${l.beams_min}~${l.beams_max} beams` : ''), cls(l.status))}
  ${row('Depth 카메라', esc(d.status || 'NO_DATA') + (d.code ? ' · ' + esc(d.code) : ''), cls(d.status))}
  ${row('Safety Gate', esc(g.status || 'NO_DATA') + (g.cmd_blocked ? ' · 명령 차단 중' : ''), cls(g.status))}
  ${row('모터 출력 경로', esc((g.motor_path || {}).status || 'UNKNOWN') + ((g.motor_path || {}).cmd_vel_publishers && g.motor_path.cmd_vel_publishers.length ? ' · /cmd_vel 발행: ' + esc(g.motor_path.cmd_vel_publishers.join(',')) : ''), ((g.motor_path || {}).status === 'ARMED') ? 'bad' : 'muted')}
  ${row('활성 임무', ms.active_mission_id ? `${esc(ms.active_mission_id)} · ${esc(ms.state || '')}${ms.detail ? ' · ' + esc(ms.detail) : ''}` : '없음')}
  ${row('연결 복구', r.reconciliation_required ? 'RECONCILIATION_REQUIRED · 목표 자동 재전송 금지' : (r.reconciled === undefined ? '서버가 복구 상태를 제공하지 않음' : (r.reconciled ? '상태·소유권 동기화 완료' : '재동기화 대기')), r.reconciliation_required ? 'bad' : 'muted')}
  ${row('오류', (s.errors && s.errors.length) ? esc(s.errors.join(' | ')) : '없음', (s.errors && s.errors.length) ? 'warn' : '')}</div></div>`;
}
async function loadMap(rid, ver) {
  if (mapVer[rid] === ver && mapImg[rid]) return;
  const r = await api(`/robots/${encodeURIComponent(rid)}/map.png`); if (!r.ok) return;
  const blob = await r.blob(); const img = new Image(); img.onload = () => { mapImg[rid] = img; mapVer[rid] = ver; drawMap(); }; img.src = URL.createObjectURL(blob);
}
function drawMap() {
  const rid = $('mapRobot').value, cv = $('map'), c = cv.getContext('2d'); c.fillStyle = '#000'; c.fillRect(0, 0, cv.width, cv.height);
  const r = data && data.robots.find(x => x.robot_id === rid), img = mapImg[rid]; if (!r || !img || !r.map_meta) { $('mapinfo').textContent = '지도 없음'; return; }
  const meta = r.map_meta, sc = Math.min(cv.width / img.width, cv.height / img.height), ox = (cv.width - img.width * sc) / 2, oy = (cv.height - img.height * sc) / 2;
  c.imageSmoothingEnabled = false; c.drawImage(img, ox, oy, img.width * sc, img.height * sc);
  const toPx = (x, y) => [ox + ((x - meta.origin_x) / meta.resolution) * sc, oy + (img.height - (y - meta.origin_y) / meta.resolution) * sc];
  const p = (r.state || {}).pose;
  if (p && (p.status === 'OK' || p.status === 'STALE') && p.frame_id === 'map') {
    const [px, py] = toPx(p.x, p.y), L = 12; c.save(); c.translate(px, py); c.rotate(-p.yaw_rad); c.fillStyle = p.status === 'OK' ? '#ff3b30' : '#888'; c.beginPath(); c.moveTo(L, 0); c.lineTo(-L / 2, L / 2); c.lineTo(-L / 2, -L / 2); c.closePath(); c.fill(); c.restore();
  } else if (p && p.frame_id && p.frame_id !== 'map') { c.fillStyle = '#ffcc00'; c.fillText('위치 프레임 ' + p.frame_id + ' (map 아님): 지도에 표시하지 않음', 10, 20); }
  for (const m of data.missions) if (m.robot_id === rid && !m.terminal) { const [gx, gy] = toPx(m.goal.x, m.goal.y); c.strokeStyle = '#34c759'; c.lineWidth = 2; c.beginPath(); c.arc(gx, gy, 8, 0, 6.3); c.stroke(); c.fillStyle = '#34c759'; c.fillText(m.mission_id.slice(2, 6), gx + 10, gy); }
  $('mapinfo').textContent = `v${meta.version || ''} ${meta.width}x${meta.height} @${meta.resolution} m`;
}
$('map').onclick = ev => {
  const rid = $('mapRobot').value, r = data && data.robots.find(x => x.robot_id === rid), img = mapImg[rid]; if (!r || !img || !r.map_meta) return;
  const cv = $('map'), rc = cv.getBoundingClientRect(), k = cv.width / rc.width, cx = (ev.clientX - rc.left) * k, cy = (ev.clientY - rc.top) * k, meta = r.map_meta;
  const sc = Math.min(cv.width / img.width, cv.height / img.height), ox = (cv.width - img.width * sc) / 2, oy = (cv.height - img.height * sc) / 2;
  $('mx').value = (meta.origin_x + ((cx - ox) / sc) * meta.resolution).toFixed(2); $('my').value = (meta.origin_y + (img.height - (cy - oy) / sc) * meta.resolution).toFixed(2); $('mrobot').value = rid;
};
function fillSelect(sel, ids) { const cur = sel.value; if (sel.options.length !== ids.length || [...sel.options].some((o, i) => o.value !== ids[i])) { sel.innerHTML = ids.map(i => `<option>${esc(i)}</option>`).join(''); if (ids.includes(cur)) sel.value = cur; } }
async function tick() {
  let r; try { r = await api('/dashboard'); } catch (e) { $('srv').textContent = '서버 연결 끊김'; $('srv').className = 'bad'; return; }
  if (!r.ok) return; data = await r.json(); const sv = data.server;
  $('srv').textContent = `서버 ${new Date(sv.time * 1000).toLocaleTimeString()} · boot ${sv.boot_id} · v${sv.version}`; $('srv').className = 'muted';
  const ids = data.robots.map(x => x.robot_id); fillSelect($('mapRobot'), ids); fillSelect($('mrobot'), ids);
  $('robots').innerHTML = data.robots.map(robotCard).join('') || '<div class="card muted">등록된 로봇이 없습니다</div>';
  for (const x of data.robots) if (x.map_version) loadMap(x.robot_id, x.map_version);
  const canCtl = role === 'operator' || role === 'admin';
  $('missions').querySelector('tbody').innerHTML = data.missions.map(m => `<tr><td>${esc(m.mission_id)}</td><td>${esc(m.robot_id)}</td><td>(${fmt(m.goal.x)}, ${fmt(m.goal.y)}, ${fmt(m.goal.yaw_rad)})</td><td class="st-${m.status}">${m.status}${m.detail ? '<br><span class="muted small">' + esc(m.detail) + '</span>' : ''}</td><td class="small">${m.progress && m.progress.distance_remaining_m != null ? fmt(m.progress.distance_remaining_m) + ' m 남음' : ''}</td><td>${(!m.terminal && canCtl) ? `<button class="small" data-cancel="${m.mission_id}">취소</button>` : ''}</td></tr>`).join('');
  $('events').innerHTML = data.events.map(e => `<div class="${e.level}">${new Date(e.ts * 1000).toLocaleTimeString()} [${esc(e.robot_id || '-')}] ${esc(e.kind)}: ${esc(e.message)}</div>`).join('');
  drawMap();
}
$('missions').onclick = async ev => {
  const id = ev.target.dataset && ev.target.dataset.cancel; if (!id) return;
  $('merr').textContent = '';
  try {
    const r = await api(`/missions/${id}/cancel`, {method: 'POST', body: '{}'});
    if (!r.ok) { const j = await r.json(); $('merr').textContent = (j.error && j.error.message) || r.status; }
  } catch (e) { $('merr').textContent = '취소 요청 결과 불명확: 연결 복구 후 임무 상태를 확인하세요'; }
  tick();
};
$('mapRobot').onchange = drawMap;
$('mform').onsubmit = async ev => {
  ev.preventDefault(); $('merr').textContent = '';
  const body = {robot_id: $('mrobot').value, type: 'navigate_to_pose', goal: {frame_id: 'map', x: +$('mx').value, y: +$('my').value, yaw_rad: +$('myaw').value}, idempotency_key: 'ui-' + uuid()};
  const r = await api('/missions', {method: 'POST', body: JSON.stringify(body)}); if (!r.ok) { const j = await r.json(); $('merr').textContent = (j.error && j.error.message) || r.status; } tick();
};
start();
