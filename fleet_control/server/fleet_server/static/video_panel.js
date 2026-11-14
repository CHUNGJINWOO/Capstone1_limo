'use strict';
// AI video panel. Talks ONLY to the separate video/AI host (never to the Mission Manager), with its own viewer token. Distinguishes:
//   data older than STALE_S / host not RECEIVING  -> "no data" overlay (never shown as current)
//   fresh result with zero detections             -> "nothing found" (a normal result)
//   host unreachable                              -> connection error
(() => {
  const $ = id => document.getElementById(id); const STALE_S = 2.0; let timer = null, busy = false, lastUrl = null;
  const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const fmt = (v, d = 1) => (v === null || v === undefined || Number.isNaN(v)) ? '-' : Number(v).toFixed(d);
  // Robot-side numbers arrive from the sender as telemetry; if it is older than 5 s it is shown as old, never as current.
  function telemetryRows(rt, row) {
    if (!rt) return row('로봇 측 정보', '아직 없음 (송신 노드 텔레메트리 미수신)', 'bad');
    const old = rt.age_s > 5, sy = rt.system || {}, c = rt.counters || {};
    return row('로봇 측 정보 나이', fmt(rt.age_s, 1) + ' s' + (old ? ' (오래됨)' : ''), old ? 'bad' : '')
      + row('로봇 쪽 오래된 결과 차단 / 오래된 프레임 폐기', `${c.stale_results_blocked || 0} / ${c.stale_before_send || 0}`)
      + row('로봇 CPU / RAM', `${fmt(sy.cpu_pct_total)} % / ${sy.mem_used_mb ?? '-'}/${sy.mem_total_mb ?? '-'} MB (송신 노드 ${fmt(sy.proc_cpu_pct_of_1core)} %, ${fmt(sy.proc_rss_mb)} MB)`)
      + row('로봇 측정 왕복(평균)', rt.rtt_ms_avg === null || rt.rtt_ms_avg === undefined ? '-' : fmt(rt.rtt_ms_avg) + ' ms')
      + row('두 장치 시계 오차(추정, 동기화 아님)', rt.clock ? `${fmt(rt.clock.offset_ms_est)} ms ± ${fmt(rt.clock.uncertainty_ms)}` : '-');
  }
  const base = () => $('aiBase').value.trim().replace(/\/+$/, '');
  { const q = new URLSearchParams(location.search).get('video');       // convenience: ?video=http://host:port prefills the address (plain origin only)
    if (q && /^https?:\/\/[A-Za-z0-9.\-]+(:\d{1,5})?$/.test(q)) sessionStorage.setItem('video_base', q); }
  $('aiTok').value = sessionStorage.getItem('video_token') || ''; $('aiBase').value = sessionStorage.getItem('video_base') || $('aiBase').value;
  function setState(txt, ok) { const b = $('aiState'); b.textContent = txt; b.className = 'badge ' + (ok ? 'on' : 'off'); }
  function overlay(msg) { const o = $('aiOverlay'); if (msg) { o.textContent = msg; o.classList.remove('hidden'); } else o.classList.add('hidden'); }
  async function poll() {
    if (busy) return; busy = true; const h = {'Authorization': 'Bearer ' + $('aiTok').value.trim()};
    try {
      const sr = await fetch(base() + '/api/v1/status', {headers: h, cache: 'no-store'});
      if (sr.status === 401) { $('aiErr').textContent = '영상 서버 토큰이 올바르지 않습니다'; setState('인증 실패', false); stop(); return; }
      const s = await sr.json(); $('aiErr').textContent = '';
      const lr = s.last_result, age = s.last_frame_age_s, stale = s.ai_state !== 'RECEIVING' || age === null || age > STALE_S;
      setState({RECEIVING: '수신 중', STALLED: '영상 끊김(정지)', SENDER_DISCONNECTED: '송신 측 연결 끊김', NO_SENDER: '송신 측 없음'}[s.ai_state] || s.ai_state, s.ai_state === 'RECEIVING' && !stale);
      try {
        const fr = await fetch(base() + '/api/v1/frame.jpg', {headers: h, cache: 'no-store'});
        if (fr.ok) { const blob = await fr.blob(); const u = URL.createObjectURL(blob); $('aiImg').src = u; if (lastUrl) URL.revokeObjectURL(lastUrl); lastUrl = u; }
      } catch (e) { /* keep the previous image, the overlay below says it is old */ }
      overlay(stale ? `데이터 없음 / 오래됨 (마지막 프레임 ${age === null ? '없음' : fmt(age, 1) + ' s 전'}, 상태 ${s.ai_state})` : '');
      const row = (k, v, c) => `<div>${k}</div><div class="${c || ''}">${esc(v)}</div>`;
      $('aiStats').innerHTML = row('AI 처리 연결', s.ai_state, s.ai_state === 'RECEIVING' ? 'ok' : 'bad') + row('수신 FPS', fmt(s.rx_fps)) + row('수신 대역폭', fmt(s.rx_kbps, 0) + ' kbit/s')
        + row('영상 지연(마지막 프레임 나이)', age === null ? '-' : fmt(age, 2) + ' s', stale ? 'bad' : '') + row('영상 전송 지연(평균)', fmt(s.transport_ms_avg) + ' ms') + row('추론 지연(평균/p95)', `${fmt(s.infer_ms_avg)} / ${fmt(s.infer_ms_p95)} ms`)
        + row('호스트 처리 합계(평균/p95)', `${fmt(s.server_total_ms_avg)} / ${fmt(s.server_total_ms_p95)} ms`) + row('마지막 프레임 시각', s.last_frame_wall ? new Date(s.last_frame_wall * 1000).toLocaleTimeString() : '-')
        + row('폐기(오래됨/덮어씀)', `${s.dropped_stale} / ${s.dropped_overwritten}`) + row('모델', s.model) + telemetryRows(s.robot_telemetry, row);
      const tb = $('aiDets').querySelector('tbody'); tb.innerHTML = '';
      if (lr && !stale) { for (const d of lr.detections) tb.insertAdjacentHTML('beforeend', `<tr><td>${esc(d.cls)}</td><td>${fmt(d.conf, 2)}</td><td>${[d.x1, d.y1, d.x2, d.y2].map(v => Math.round(v)).join(', ')}</td></tr>`);
        $('aiNote').textContent = lr.n === 0 ? '검출 0개: 이번 프레임에서 대상 객체를 찾지 못했습니다 (수신은 정상).' : `프레임 seq ${lr.seq}, frame_id ${lr.frame_id}, 결과 나이 ${fmt(lr.age_s, 2)} s`; }
      else $('aiNote').textContent = stale ? '결과가 오래되었거나 수신 실패 상태입니다. 위 표는 현재 값이 아니므로 비웠습니다.' : '아직 결과가 없습니다.';
    } catch (e) { setState('영상 서버 연결 실패', false); $('aiErr').textContent = '영상 서버에 연결할 수 없습니다: ' + e.message; overlay('영상 서버 연결 실패'); }
    finally { busy = false; }
  }
  function stop() { if (timer) { clearInterval(timer); timer = null; } }
  $('aiForm').onsubmit = e => { e.preventDefault(); sessionStorage.setItem('video_token', $('aiTok').value.trim()); sessionStorage.setItem('video_base', base()); stop(); poll(); timer = setInterval(poll, 500); };
  $('aiStop').onclick = () => { stop(); setState('중지됨', false); };
})();
