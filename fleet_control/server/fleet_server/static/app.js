'use strict';
(() => {
  const API = '/api/v1';
  const H = window.FleetHelpers;
  const $ = (id) => document.getElementById(id);

  let token = sessionStorage.getItem('fleet_token') || '';
  let me = null;
  let data = null;
  let fullMissions = null;
  let selectedRobotId = '';
  let selectedMissionId = '';
  let drawerReturnFocus = null;
  let activeTab = 'overview';
  let spatialMode = 'floor';
  let buildingRenderSeq = 0;
  let thumbnailUrls = [];
  let showAllPanels = false;
  let mapView = null;
  let mapMode = 'goal';
  let pollTimer = null;
  let lastDashboardAt = 0;
  let tickBusy = false;
  let registryFacilities = [];
  let registryMapsByFloor = {};
  let registryState = {
    initialized: false,
    loading: false,
    error: '',
    offline: false,
    selectedBuildingId: '',
    selectedFloorId: '',
    selectedAreaId: '',
    selectedMapVersionId: '__live__',
    preview: null,
    previewSeq: 0,
  };
  let previewObjectUrl = null;
  let recordingBusy = false;
  let slamBusy = false;

  const uuid = () =>
    ([1e7] + -1e3 + -4e3 + -8e3 + -1e11).replace(/[018]/g, (c) =>
      (c ^ ((Math.random() * 16) >> (c / 4))).toString(16)
    );

  async function api(path, opt) {
    const o = opt || {};
    const timeoutMs = Number(o.timeoutMs || 0);
    let ac = null;
    let tid = null;
    if (timeoutMs > 0 && !o.signal && typeof AbortController !== 'undefined') {
      ac = new AbortController();
      tid = setTimeout(() => ac.abort(), timeoutMs);
    }
    try {
      const r = await fetch(API + path, {
        ...o,
        signal: o.signal || (ac ? ac.signal : undefined),
        cache: 'no-store',
        headers: {
          Authorization: 'Bearer ' + token,
          'Content-Type': 'application/json',
          ...(o.headers || {}),
        },
      });
      if (r.status === 401) {
        showLogin('토큰이 올바르지 않거나 만료되었습니다');
        throw new Error('401');
      }
      return r;
    } finally {
      if (tid) clearTimeout(tid);
    }
  }

  function showLogin(msg) {
    closeGoalDrawer();
    $('login').classList.remove('hidden');
    $('main').classList.add('hidden');
    $('loginerr').textContent = msg || '';
    $('logout').classList.add('hidden');
    $('who').textContent = '';
    if (pollTimer) {
      clearInterval(pollTimer);
      pollTimer = null;
    }
  }

  $('go').onclick = async () => {
    token = $('tok').value.trim();
    $('tok').value = '';
    sessionStorage.setItem('fleet_token', token);
    await start();
  };
  $('tok').addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      $('go').click();
    }
  });
  $('logout').onclick = () => {
    sessionStorage.removeItem('fleet_token');
    token = '';
    me = null;
    showLogin('');
  };

  $('navCollapse').onclick = () => { const compact=document.body.classList.toggle('nav-collapsed'); $('navCollapse').setAttribute('aria-expanded', String(!compact)); $('navCollapse').setAttribute('aria-label', compact ? '내비게이션 펼치기' : '내비게이션 접기'); if(mapView) requestAnimationFrame(()=>mapView.resize()); };
  // Tab & All-View switching (keeps #aiPanel in DOM for video_panel.js)
  function applyTabVisibility() {
    document.body.classList.toggle('all-panels', showAllPanels);
    const mapSlot = $('mapsMapSlot');
    const overviewSlot = $('overviewMapSlot');
    const mapCard = document.querySelector('.map-card');
    (activeTab === 'control' && !showAllPanels ? $('operationMapSlot') : activeTab === 'spatial' && !showAllPanels ? $('spatialMapSlot') : activeTab === 'maps' && !showAllPanels ? mapSlot : overviewSlot).appendChild(mapCard);
    const rgb = ($('aiImg') && $('aiImg').closest('.aiimgbox')) || document.querySelector('#sensorRgb .aiimgbox, #spatialRgb .aiimgbox');
    if (rgb) (activeTab === 'spatial' && !showAllPanels ? $('spatialRgb') : $('sensorRgb')).appendChild(rgb);
    if (mapView) mapView.readOnly = showAllPanels || !['maps', 'control'].includes(activeTab);
    $('mapReadOnlyLabel').textContent = !showAllPanels && ['maps','control'].includes(activeTab) ? '목표 지정 → Drawer에서 최종 확인' : '조회 전용';
    document.querySelectorAll('#navTabs .tab').forEach((btn) => {
      btn.classList.toggle('active', !showAllPanels && btn.dataset.tab === activeTab);
      btn.setAttribute('aria-current', !showAllPanels && btn.dataset.tab === activeTab ? 'page' : 'false');
    });
    document.querySelectorAll('.tab-panel').forEach((panel) => {
      const visible = showAllPanels || panel.dataset.panel === activeTab || (panel.dataset.panel === 'operations' && activeTab === 'settings');
      panel.classList.toggle('hidden', !visible);
    });
    $('toggleAllView').textContent = showAllPanels ? '탭 보기로 전환' : '전체 상세 펼치기';
    if (activeTab !== 'control') closeGoalDrawer();
    $('initPoseBox').classList.toggle('hidden', activeTab !== 'maps' || mapMode !== 'initialpose');
    if (activeTab === 'spatial' && spatialMode === 'building') renderBuildingView();
    if (mapView) requestAnimationFrame(() => mapView.resize());
  }

  $('navTabs').addEventListener('click', (e) => {
    const btn = e.target.closest('.tab');
    if (!btn) return;
    showAllPanels = false;
    navigate(btn.dataset.tab);
    applyTabVisibility();
    if (activeTab === 'rmf' && !fullMissions) fetchFullMissions();
  });

  $('toggleAllView').onclick = () => {
    showAllPanels = !showAllPanels;
    applyTabVisibility();
    if (showAllPanels && !fullMissions) fetchFullMissions();
  };

  $('spatialFloor').onclick = () => mapView && mapView.fit();
  $('spatialZoomIn').onclick = () => mapView && mapView.zoomBy(1.3);
  $('spatialZoomOut').onclick = () => mapView && mapView.zoomBy(1 / 1.3);
  $('spatialRobot').onclick = () => mapView && mapView.fitRobot();
  $('spatialReset').onclick = () => { if (!mapView) return; $('mapRot').value='0'; $('mapRotVal').textContent='0°'; mapView.setRotationDeg(0); mapView.fit(); };
  $('spatialFullscreen').onclick = async () => { try { if (document.fullscreenElement) await document.exitFullscreen(); else await $('spatialWorkspace').requestFullscreen(); } catch (e) { $('spatialSummary').textContent = '전체화면을 사용할 수 없습니다'; } };
  for (const [a,b] of [['spatialLayerMap','layerMap'],['spatialLayerGoals','layerGoals'],['spatialLayerTrail','layerTrail']]) { $(a).onchange = () => { $(b).checked=$(a).checked; $(b).dispatchEvent(new Event('change')); }; $(b).addEventListener('change', () => { $(a).checked=$(b).checked; }); }
  function setSpatialMode(mode) {
    spatialMode = mode;
    const workspace = $('spatialWorkspace');
    workspace.dataset.mode = mode;
    workspace.classList.remove('camera-focus');
    $('buildingView').classList.toggle('hidden', mode !== 'building');
    document.querySelectorAll('[data-spatial-mode]').forEach(b => { const selected = b.dataset.spatialMode === mode; b.classList.toggle('active', selected); b.setAttribute('aria-pressed', String(selected)); });
    $('spatialFocus').textContent = '영상 확대';
    if (mode === 'building') renderBuildingView();
    if (mapView) requestAnimationFrame(() => mapView.resize());
  }
  document.querySelectorAll('[data-spatial-mode]').forEach(b => b.addEventListener('click', () => setSpatialMode(b.dataset.spatialMode)));
  $('spatialFocus').onclick = () => { const focus=$('spatialWorkspace').classList.toggle('camera-focus'); $('spatialFocus').textContent=focus ? '지도 확대' : '영상 확대'; if (mapView) requestAnimationFrame(() => mapView.resize()); };
  $('spatialRotation').oninput = e => { $('mapRot').value=e.target.value; $('mapRot').dispatchEvent(new Event('input')); $('spatialRotationLabel').textContent=e.target.value+'°'; };
  async function renderBuildingView() {
    const seq=++buildingRenderSeq;
    thumbnailUrls.forEach(URL.revokeObjectURL); thumbnailUrls=[];
    const buildings=allBuildingsFromFacilities(), current=findBuilding(registryState.selectedBuildingId);
    $('buildingList').innerHTML=buildings.length ? buildings.map(b => `<button class="building-item ${b.building_id===registryState.selectedBuildingId?'selected':''}" data-building="${H.esc(b.building_id)}"><span class="building-icon">▤</span><span><strong>${H.esc(b.name || b.building_id)}</strong><small>${H.esc(b.facility_name)} · ${(b.floors||[]).length} FLOORS</small></span><b>›</b></button>`).join('') : '<p class="muted small">등록된 건물이 없습니다.</p>';
    $('buildingTitle').textContent=current ? current.name || current.building_id : '등록된 건물 없음';
    const floors=current?.floors || [];
    $('buildingFloors').innerHTML=floors.length ? floors.map(f => `<button class="floor-tile" data-floor="${H.esc(f.floor_id)}"><div class="floor-thumb" data-thumb="${H.esc(f.floor_id)}"><span>지도 불러오는 중…</span></div><div class="floor-meta"><strong>${H.esc(f.level_name || f.floor_id)}</strong><span>지도 ${H.esc(f.map_version_count ?? 0)}개</span>${(f.areas||[]).length?`<small>영역 ${H.esc(f.areas.map(a=>a.name||a.area_id).join(', '))}</small>`:''}<small>로봇 소속 UNKNOWN</small></div></button>`).join('') : '<div class="empty-floor">등록된 층이 없습니다</div>';
    await Promise.all(floors.map(async f => {
      const floorId=f.floor_id;
      try {
        const res=await api(`/map-registry/maps?floor_id=${encodeURIComponent(floorId)}`); if (!res.ok) throw Error('목록 조회 실패');
        const json=await res.json(), maps=json.maps||[]; registryMapsByFloor[floorId]=maps;
        const map=maps.find(m=>m.is_floor_active)||maps[0]; if (!map) throw Error('저장 지도 없음');
        const imgRes=await api(`/map-registry/maps/${encodeURIComponent(map.map_version_id)}/preview.png`); if (!imgRes.ok) throw Error('미리보기 불가');
        const blob=await imgRes.blob(); if(seq!==buildingRenderSeq) return;
        const slot=[...$('buildingFloors').querySelectorAll('[data-thumb]')].find(x=>x.dataset.thumb===floorId); if(!slot)return;
        const url=URL.createObjectURL(blob);thumbnailUrls.push(url); slot.replaceChildren();const img=document.createElement('img');img.src=url;img.alt=`${f.level_name || floorId} 저장 지도 미리보기 (${map.verification_status})`;slot.appendChild(img);
        const label=document.createElement('span');label.className='thumbnail-label';label.textContent=`저장 지도 · ${map.verification_status || 'UNKNOWN'}`;slot.appendChild(label);
      } catch(e) { if(seq!==buildingRenderSeq)return; const slot=[...$('buildingFloors').querySelectorAll('[data-thumb]')].find(x=>x.dataset.thumb===floorId);if(slot)slot.textContent=e.message || '조회 실패'; }
    }));
  }
  $('buildingList').addEventListener('click', async e => { const item=e.target.closest('[data-building]'); if(!item)return; registryState.selectedBuildingId=item.dataset.building; const b=findBuilding(item.dataset.building);registryState.selectedFloorId=b?.floors?.[0]?.floor_id || ''; await selectRegistryMapVersion('__live__');await fetchFloorMaps(registryState.selectedFloorId);renderBuildingView(); });
  $('buildingFloors').addEventListener('click', async e => { const tile=e.target.closest('[data-floor]');if(!tile)return;registryState.selectedFloorId=tile.dataset.floor;await selectRegistryMapVersion('__live__');await fetchFloorMaps(registryState.selectedFloorId);const maps=registryMapsByFloor[registryState.selectedFloorId]||[];const map=maps.find(m=>m.is_floor_active)||maps[0];if(map)await selectRegistryMapVersion(map.map_version_id);setSpatialMode('floor'); });
  setSpatialMode('floor');
  function updateMapSummary() {
    const label = id => { const e=$(id); return e?.selectedOptions?.[0]?.textContent?.trim() || 'UNKNOWN'; };
    $('mapSelectionSummary').textContent = `${label('mapBuilding')} / ${label('mapLevel')} / ${label('mapArea')} / ${label('mapVersionSelect')}`;
  }
  $('mapSettingsToggle').onclick = () => {
    const panel=$('mapSettingsPanel'), opening=panel.classList.contains('hidden');
    panel.classList.toggle('hidden', !opening);
    $('mapSettingsToggle').setAttribute('aria-expanded', String(opening));
    $('mapSettingsToggle').textContent=opening ? '⚙ 설정 접기' : '⚙ 설정';
    if (mapView) requestAnimationFrame(() => mapView.resize());
  };
  $('mapFullscreen').onclick = async () => { try { if (document.fullscreenElement) await document.exitFullscreen(); else await document.querySelector('.map-card').requestFullscreen(); } catch (_) { $('mapSelectionSummary').textContent='전체화면을 사용할 수 없습니다'; } };
  for (const [a,b] of [['quickLayerMap','layerMap'],['quickLayerGoals','layerGoals'],['quickLayerTrail','layerTrail']]) {
    $(a).onchange=()=>{ $(b).checked=$(a).checked; $(b).dispatchEvent(new Event('change')); };
    $(b).addEventListener('change',()=>{ $(a).checked=$(b).checked; });
  }
  function initMapView() {
    if (mapView) return;
    mapView = new window.FleetMapView($('map'), {
      onCursor: (w) => {
        $('mapCursor').textContent = `커서: x ${H.fmt(w.x)} · y ${H.fmt(w.y)} m`;
      },
      onScale: (txt) => {
        $('mapScale').textContent = '축척: ' + txt;
      },
      onPickPoint: (w) => {
        if (registryState.preview) {
          $('merr').textContent = '레지스트리 미리보기 모드에서는 NavigateToPose 목표를 지정할 수 없습니다. 라이브 지도로 전환하세요.';
          return;
        }
        if (mapMode === 'goal' || activeTab === 'control') {
          $('mx').value = w.x.toFixed(2);
          $('my').value = w.y.toFixed(2);
          if (selectedRobotId) $('mrobot').value = selectedRobotId;
          openGoalDrawer();
          $('goalConfirmed').checked = false;
          updateGoalReview();
        } else {
          $('initX').value = w.x.toFixed(2);
          $('initY').value = w.y.toFixed(2);
        }
      },
      onPreviewInspect: (w) => {
        $('mapCursor').textContent = `미리보기 조회 좌표: x ${w.x.toFixed(2)}, y ${w.y.toFixed(2)} m (목표 지정 차단됨)`;
        if (me && (me.role === 'operator' || me.role === 'admin')) {
          $('merr').textContent = '레지스트리 미리보기 모드에서는 NavigateToPose 목표 클릭이 차단됩니다.';
        }
      },
    });
    mapView.readOnly = showAllPanels || !['maps', 'control'].includes(activeTab);
    $('mapFit').onclick = () => mapView.fit();
    $('mapZoomIn').onclick = () => mapView.zoomBy(1.3);
    $('mapZoomOut').onclick = () => mapView.zoomBy(1 / 1.3);
    $('mapRot').oninput = (e) => {
      $('mapRotVal').textContent = e.target.value + '°';
      mapView.setRotationDeg(e.target.value);
    };
    $('mapRotReset').onclick = () => {
      $('mapRot').value = '0';
      $('mapRotVal').textContent = '0°';
      mapView.setRotationDeg(0);
    };
    $('layerMap').onchange = (e) => mapView.setLayer('map', e.target.checked);
    $('layerGoals').onchange = (e) => mapView.setLayer('goals', e.target.checked);
    $('layerTrail').onchange = (e) => mapView.setLayer('trail', e.target.checked);
    if ($('layerCostmap')) $('layerCostmap').onchange = (e) => mapView.setLayer('costmap', e.target.checked);
    if ($('layerPlan')) $('layerPlan').onchange = (e) => mapView.setLayer('plan', e.target.checked);
    document.querySelectorAll('input[name="mapMode"]').forEach((radio) => {
      radio.onchange = () => {
        mapMode = radio.value;
        $('initPoseBox').classList.toggle('hidden', mapMode !== 'initialpose');
      };
    });
    window.addEventListener('resize', () => mapView && mapView.resize());
    window.addEventListener('fleet-theme-change', () => mapView && mapView.request());
  }

  async function start() {
    if (!token) return showLogin('');
    let r;
    try {
      r = await fetch(API + '/me', { headers: { Authorization: 'Bearer ' + token }, cache: 'no-store' });
    } catch (e) {
      return showLogin('서버에 연결할 수 없습니다');
    }
    if (!r.ok) {
      sessionStorage.removeItem('fleet_token');
      return showLogin(r.status === 401 ? '토큰이 올바르지 않습니다' : '접속 실패 ' + r.status);
    }
    me = await r.json();
    $('who').textContent = `${me.name} (${me.role})`;
    $('logout').classList.remove('hidden');
    $('login').classList.add('hidden');
    $('main').classList.remove('hidden');

    const canCtl = me.role === 'operator' || me.role === 'admin';
    const isAdmin = me.role === 'admin';
    $('mform').querySelectorAll('input,select,button').forEach((el) => { el.disabled = !canCtl; });
    $('leaseAcquire').disabled = !canCtl;
    $('leaseRenew').disabled = !canCtl;
    $('leaseRelease').disabled = !canCtl;
    $('leaseTtl').disabled = !canCtl;
    $('leaseNote').disabled = !canCtl;
    $('leaseForceRelease').classList.toggle('hidden', !isAdmin);
    $('leaseForceRelease').disabled = !isAdmin;
    $('recoverRobotBtn').disabled = !isAdmin;
    $('recoverRobotNote').disabled = !isAdmin;
    if (!canCtl) {
      $('merr').textContent = 'viewer 권한: 임무 생성·취소 및 제어권 변경 불가';
    } else {
      $('merr').textContent = '';
    }

    initMapView();
    applyTabVisibility();
    await tick();
    if (!pollTimer) pollTimer = setInterval(tick, 1000);
  }

  function fillSelect(sel, options, keepAll) {
    const cur = sel.value;
    const vals = keepAll ? ['ALL'].concat(options) : options;
    if (sel.options.length !== vals.length || [...sel.options].some((o, i) => o.value !== vals[i])) {
      sel.innerHTML = vals
        .map((v) => `<option value="${H.esc(v)}">${v === 'ALL' ? '전체' : H.esc(v)}</option>`)
        .join('');
      if (vals.includes(cur)) sel.value = cur;
    }
  }

  let liveMapPixels = null;   // decoded live robot map used for the goal pre-check
  // 2026-11-21: the SERVER goal check (GET /robots/{id}/goal-check, the same rule POST /missions enforces: loaded map version,
  // inside, FREE, 0.20 m clearance) decides. The local raster check stayed stricter (0.32 m) and refused points the server
  // accepts; it is now shown only as an advisory note. No new check is added anywhere.
  const serverGoal = { key: '', res: null, busy: false };
  const GOAL_REASON = { OUTSIDE_MAP: '지도 경계 밖', OCCUPIED: '점유 셀', UNKNOWN_AREA: '미관측(UNKNOWN) 셀', FOOTPRINT_OCCUPIED: '차체 반경(0.20 m) 안 점유 셀',
    FOOTPRINT_UNKNOWN: '차체 반경(0.20 m) 안 미관측 셀', FOOTPRINT_OUTSIDE_MAP: '차체 반경이 지도 밖', NO_MAP_LOADED: '로딩된 지도 없음', NOT_FINITE: '좌표 오류' };
  async function refreshServerGoal() {
    const rid = $('mrobot').value, x = +$('mx').value, y = +$('my').value;
    if (!rid || !Number.isFinite(x) || !Number.isFinite(y) || $('mx').value === '' || $('my').value === '') return;
    const key = `${rid}|${x}|${y}`;
    if (serverGoal.busy || serverGoal.key === key) return;
    serverGoal.busy = true;
    try {
      const r = await api(`/robots/${encodeURIComponent(rid)}/goal-check?x=${x}&y=${y}`);
      serverGoal.key = key; serverGoal.res = r.ok ? await r.json() : { ok: false, reason: `HTTP ${r.status}` };
    } catch (_) { serverGoal.key = ''; serverGoal.res = null; }
    finally { serverGoal.busy = false; }
    updateGoalReview();
  }
  function goalPrecheck() {
    const rid = $('mrobot').value;
    const key = `${rid}|${+$('mx').value}|${+$('my').value}`;
    let res = { ok: false, reasons: [] };
    if (window.GoalCheck && liveMapPixels && liveMapPixels.rid === rid) {
      res = window.GoalCheck.evaluate(liveMapPixels.meta, liveMapPixels.pixels, +$('mx').value, +$('my').value);
    }
    const local = res.ok ? null : res.reasons.slice();
    const sv = serverGoal.key === key ? serverGoal.res : null;
    if (!sv) { setTimeout(refreshServerGoal, 0); res.ok = false; res.reasons = ['서버 목표 검사 중']; }
    else if (sv.ok) { res.ok = true; res.reasons = local && local.length ? [`참고(화면 검사): ${local.join(', ')} — 서버 기준 통과`] : []; res.server = sv; }
    else { res.ok = false; res.reasons = [`서버 검사: ${GOAL_REASON[sv.reason] || sv.reason}`]; res.server = sv; }
    const st = window.MappingPanel && window.MappingPanel.currentState ? window.MappingPanel.currentState() : null;
    if (st && !['IDLE', 'STOPPED'].includes(st)) { res.ok = false; res.reasons.push(`자동 맵핑 ${st}: 이동 임무 차단`); }
    return res;
  }
  async function loadMap(rid, ver) {
    if (!mapView || mapView.hasMapVersion(rid, ver)) return;
    try {
      const r = await api(`/robots/${encodeURIComponent(rid)}/map.png`);
      if (!r.ok) return;
      const blob = await r.blob();
      const url = URL.createObjectURL(blob);
      const img = new Image();
      img.onload = () => {
        URL.revokeObjectURL(url);
        mapView.setMapImage(rid, ver, img);
        const robot = (data?.robots || []).find((x) => x.robot_id === rid);
        const meta = robot && robot.map_meta;
        if (window.GoalCheck && meta && meta.width && meta.height) {
          try { liveMapPixels = { rid, ver, meta, pixels: window.GoalCheck.pixelsFromImage(img, meta.width, meta.height) }; } catch (_) { liveMapPixels = null; }
        }
      };
      img.onerror = () => URL.revokeObjectURL(url);
      img.src = url;
    } catch (e) { /* ignore transient map load failure */ }
  }

  function normalizeRegistryMeta(m) {
    const orig = Array.isArray(m && m.origin)
      ? m.origin
      : [(m && m.origin_x) || 0, (m && m.origin_y) || 0, (m && m.origin_yaw) || 0];
    return {
      width: Number(m.width),
      height: Number(m.height),
      resolution: Number(m.resolution),
      origin_x: Number(orig[0] || 0),
      origin_y: Number(orig[1] || 0),
      origin_yaw: Number(orig[2] || 0),
    };
  }

  function clearPreviewObjectUrl() {
    if (previewObjectUrl) {
      URL.revokeObjectURL(previewObjectUrl);
      previewObjectUrl = null;
    }
  }

  function updateMissionFormBlockState() {
    const canCtl = Boolean(me && (me.role === 'operator' || me.role === 'admin'));
    const blocked = Boolean(registryState.preview) || registryState.offline || !lastDashboardAt || performance.now() - lastDashboardAt > 3000;
    const btn = $('mcreate');
    if (btn) {
      const pre = goalPrecheck();
      btn.disabled = !canCtl || blocked || !$('goalConfirmed').checked || !pre.ok;
      btn.title = blocked
        ? '미리보기 또는 서버 상태 미확인: 라이브 지도와 서버 연결을 확인하세요'
        : '';
    }
  }

  function allBuildingsFromFacilities() {
    const list = [];
    for (const fac of registryFacilities || []) {
      for (const b of fac.buildings || []) {
        list.push({ ...b, facility_name: fac.name || fac.facility_id });
      }
    }
    return list;
  }

  function findBuilding(buildingId) {
    return allBuildingsFromFacilities().find((b) => b.building_id === buildingId) || null;
  }

  function findFloor(building, floorId) {
    if (!building) return null;
    return (building.floors || []).find((f) => f.floor_id === floorId) || null;
  }

  // Area + sensor provenance recorded at registration. Shown as stated evidence, never as a live detection.
  function registryProvenanceText(raw) {
    if (!raw) return '';
    const parts = [];
    if (raw.area && raw.area.name) parts.push(`영역 ${raw.area.name}${raw.area.coverage === 'PARTIAL_FLOOR' ? '(부분 공간)' : ''}`);
    const s = raw.verification_evidence && raw.verification_evidence.sensors;
    if (s && s.slam_raster) {
      const yn = (v) => (v === true ? 'O' : v === false ? 'X' : '?');
      const nav = s.navigation_costmaps_during_mapping || {};
      parts.push(`지도 생성 LiDAR ${yn(s.slam_raster.lidar)} · Depth ${yn(s.slam_raster.depth)}`);
      parts.push(`맵핑 중 회피 Depth ${yn(nav.depth)} · 기억 노드 ${yn(nav.obstacle_memory)}`);
    } else {
      parts.push('센서 정보 미기록');
    }
    const ev = raw.verification_evidence || {};
    if (ev.run_id) parts.push(`run ${ev.run_id}`);
    if (ev.location && ev.location.source === 'operator_declared') parts.push('건물·층·영역은 운영자 지정(자동 인식 아님)');
    return parts.length ? ' · ' + parts.join(' · ') : '';
  }

  function renderRegistryUi() {
    const buildings = allBuildingsFromFacilities();
    if (!buildings.some((b) => b.building_id === registryState.selectedBuildingId)) {
      registryState.selectedBuildingId = buildings[0] ? buildings[0].building_id : '';
    }
    const bSel = findBuilding(registryState.selectedBuildingId);
    const floors = bSel ? (bSel.floors || []) : [];
    if (!floors.some((f) => f.floor_id === registryState.selectedFloorId)) {
      registryState.selectedFloorId = floors[0] ? floors[0].floor_id : '';
    }

    const bEl = $('mapBuilding');
    const fEl = $('mapLevel');
    const vEl = $('mapVersionSelect');
    if (bEl) {
      bEl.innerHTML = buildings.length
        ? buildings.map((b) => `<option value="${H.esc(b.building_id)}">${H.esc(b.name || b.building_id)} (${H.esc(b.facility_name)})</option>`).join('')
        : '<option value="">등록된 건물 없음</option>';
      bEl.value = registryState.selectedBuildingId;
      bEl.disabled = !buildings.length;
    }
    if (fEl) {
      fEl.innerHTML = floors.length
        ? floors.map((f) => `<option value="${H.esc(f.floor_id)}">${H.esc(f.level_name || f.floor_id)} (${H.esc(f.map_version_count || 0)}개)</option>`).join('')
        : '<option value="">등록된 층 없음</option>';
      fEl.value = registryState.selectedFloorId;
      fEl.disabled = !floors.length;
    }

    const floorSel = findFloor(bSel, registryState.selectedFloorId);
    const areas = (floorSel && floorSel.areas) || [];
    if (!areas.some((a) => a.area_id === registryState.selectedAreaId)) {
      registryState.selectedAreaId = areas.length === 1 ? areas[0].area_id : '';
    }
    const aEl = $('mapArea');
    if (aEl) {
      aEl.innerHTML = ['<option value="">전체 (영역 미지정 포함)</option>']
        .concat(areas.map((a) => `<option value="${H.esc(a.area_id)}">${H.esc(a.name || a.area_id)}${a.coverage === 'PARTIAL_FLOOR' ? ' · 부분 공간' : ''} (${H.esc((a.map_version_ids || []).length)}개)</option>`))
        .join('');
      aEl.value = registryState.selectedAreaId;
      aEl.disabled = !areas.length;
    }
    const allFloorMaps = registryMapsByFloor[registryState.selectedFloorId] || [];
    const floorMaps = registryState.selectedAreaId
      ? allFloorMaps.filter((mv) => mv.area && mv.area.area_id === registryState.selectedAreaId)
      : allFloorMaps;
    if (vEl) {
      const opts = ['<option value="__live__">라이브 지도 (로봇 탑재 Agent)</option>'];
      for (const mv of floorMaps) {
        const activeTag = mv.is_floor_active ? ' ★활성' : '';
        opts.push(
          `<option value="${H.esc(mv.map_version_id)}">${H.esc(mv.label || mv.map_version_id)} [${H.esc(mv.verification_status)}${activeTag}]</option>`
        );
      }
      vEl.innerHTML = opts.join('');
      const hasSel = registryState.selectedMapVersionId === '__live__' ||
        floorMaps.some((mv) => mv.map_version_id === registryState.selectedMapVersionId);
      if (!hasSel) {
        registryState.selectedMapVersionId = '__live__';
      }
      vEl.value = registryState.selectedMapVersionId;
      vEl.disabled = false;
    }

    const sumEl = $('mapSelSummary');
    if (sumEl) {
      const pathParts = [bSel && bSel.facility_name, bSel && (bSel.name || bSel.building_id), floorSel && (floorSel.level_name || floorSel.floor_id)];
      const areaSel = areas.find((a) => a.area_id === registryState.selectedAreaId);
      if (areaSel) pathParts.push(areaSel.name || areaSel.area_id);
      const mvSel = floorMaps.find((m) => m.map_version_id === registryState.selectedMapVersionId);
      const verTxt = registryState.selectedMapVersionId === '__live__'
        ? '라이브 지도(로봇 탑재)'
        : (mvSel ? `${mvSel.label || mvSel.map_version_id} · ${mvSel.verification_status} · ${mvSel.is_floor_active ? '층 활성 지도' : '층 활성 지도 아님'}` : '');
      const pp = pathParts.filter(Boolean);
      sumEl.innerHTML = pp.length
        ? `<span class="mono">${H.esc(pp.join(' › '))}</span> · ${H.esc(verTxt)} · <span class="muted">선택은 미리보기이며 로봇의 지도는 바뀌지 않습니다</span>`
        : '<span class="muted">등록된 건물·층이 없습니다.</span>';
    }
    const badge = $('mapSourceBadge');
    const banner = $('mapPreviewBanner');
    const prev = registryState.preview;
    if (badge) {
      if (registryState.offline) {
        badge.textContent = '서버 OFFLINE';
        badge.className = 'badge off';
      } else if (prev) {
        badge.textContent = '레지스트리 미리보기';
        badge.className = 'badge ' + (prev.error || prev.verification_status === 'REJECTED' ? 'bad' : 'warn');
      } else {
        badge.textContent = '라이브 지도';
        badge.className = 'badge on';
      }
    }

    if (banner) {
      if (registryState.offline) {
        banner.textContent = '서버 연결 끊김 (OFFLINE) — Map Registry 및 실시간 로봇 상태를 갱신할 수 없습니다.';
        banner.className = 'alert-banner';
      } else if (registryState.error) {
        banner.textContent = `Map Registry 조회 실패: ${registryState.error}`;
        banner.className = 'alert-banner';
      } else if (prev) {
        let extra = '';
        let cls = 'alert-banner warn-banner';
        if (prev.loading) {
          extra = `불러오는 중 (${prev.mapVersionId})...`;
        } else if (prev.error) {
          extra = `오류: ${prev.error}`;
          cls = 'alert-banner';
        } else {
          const stText = prev.verification_status === 'UNVERIFIED'
            ? '미검증(UNVERIFIED · 로봇 활성화 불가)'
            : (prev.verification_status === 'REJECTED' ? '거부됨(REJECTED)' : '검증됨(VERIFIED)');
          if (prev.verification_status === 'REJECTED') cls = 'alert-banner';
          const o = prev.meta ? H.gridOrigin(prev.meta) : null;
          extra = `${prev.mapVersionId} · ${stText}${o ? ` · origin=(${o.x.toFixed(2)}, ${o.y.toFixed(2)}, yaw=${o.yaw.toFixed(3)}) · ${prev.meta.resolution}m/px` : ''}${registryProvenanceText(prev.rawMeta)}`;
        }
        banner.textContent = `레지스트리 미리보기 — 로봇 탑재 지도 아님 (NavigateToPose 클릭·전송 차단) | ${extra}`;
        banner.className = cls;
      } else if (registryState.initialized && buildings.length === 0) {
        banner.textContent = 'Map Registry에 등록된 건물·층·지도가 없습니다. 현재 라이브 로봇 지도를 표시합니다.';
        banner.className = 'alert-banner info-banner';
      } else if (registryState.initialized && registryState.selectedFloorId && floorMaps.length === 0) {
        const fl = findFloor(bSel, registryState.selectedFloorId);
        banner.textContent = `선택한 층(${(fl && fl.level_name) || registryState.selectedFloorId})에 저장된 지도가 없습니다.`;
        banner.className = 'alert-banner info-banner';
      } else {
        banner.className = 'alert-banner warn-banner hidden';
        banner.textContent = '';
      }
    }

    if (prev) {
      $('mapinfo').textContent = prev.meta
        ? `미리보기 ${prev.mapVersionId} · ${prev.meta.width}x${prev.meta.height} · ${prev.meta.resolution}m/px (${prev.verification_status})`
        : `레지스트리 미리보기 (${prev.mapVersionId})`;
      $('mapinfo').className = (prev.error || prev.verification_status === 'REJECTED') ? 'bad small' : 'warn small';
    }

    updateMapSummary();
    if (window.RegistryUpload) window.RegistryUpload.update();
    updateMissionFormBlockState();
  }

  async function fetchFloorMaps(floorId) {
    if (!floorId) {
      renderRegistryUi();
      return;
    }
    try {
      const r = await api(`/map-registry/maps?floor_id=${encodeURIComponent(floorId)}`);
      if (!r.ok) {
        const j = await r.json().catch(() => ({}));
        registryState.error = (j.error && `${j.error.code}: ${j.error.message}`) || `HTTP ${r.status}`;
        renderRegistryUi();
        return;
      }
      const j = await r.json();
      registryMapsByFloor[floorId] = j.maps || [];
      registryState.error = '';
      registryState.offline = false;
    } catch (e) {
      registryState.offline = true;
    }
    renderRegistryUi();
  }

  async function fetchRegistryHierarchy() {
    if (!me || me.role === 'agent') return;
    registryState.loading = true;
    try {
      const r = await api('/map-registry/facilities');
      if (!r.ok) {
        const j = await r.json().catch(() => ({}));
        registryState.error = (j.error && `${j.error.code}: ${j.error.message}`) || `HTTP ${r.status}`;
        registryState.initialized = true;
        registryState.loading = false;
        renderRegistryUi();
        return;
      }
      const j = await r.json();
      registryFacilities = j.facilities || [];
      registryState.error = '';
      registryState.offline = false;
      registryState.initialized = true;
      registryState.loading = false;
      renderRegistryUi();
      if (registryState.selectedFloorId) {
        await fetchFloorMaps(registryState.selectedFloorId);
      }
    } catch (e) {
      registryState.loading = false;
      registryState.offline = true;
      renderRegistryUi();
    }
  }

  async function selectRegistryMapVersion(mvid) {
    registryState.selectedMapVersionId = mvid || '__live__';
    if (!mvid || mvid === '__live__') {
      registryState.previewSeq += 1;
      clearPreviewObjectUrl();
      registryState.preview = null;
      $('mapCursor').textContent = '커서: x -, y - m';
      if (mapView) mapView.setRegistryPreview(null);
      if (me && (me.role === 'operator' || me.role === 'admin') && $('merr').textContent.includes('미리보기')) {
        $('merr').textContent = '';
      }
      renderRegistryUi();
      renderAll();
      return;
    }

    const seq = ++registryState.previewSeq;
    clearPreviewObjectUrl();
    const loadingPreview = {
      mapVersionId: mvid,
      fitKey: mvid,
      loading: true,
      meta: null,
      img: null,
      verification_status: 'UNKNOWN',
      verified: false,
      error: '',
    };
    registryState.preview = loadingPreview;
    if (mapView) mapView.setRegistryPreview(loadingPreview);
    renderRegistryUi();

    try {
      const encId = encodeURIComponent(mvid);
      const [metaRes, pngRes] = await Promise.all([
        api(`/map-registry/maps/${encId}/meta`),
        api(`/map-registry/maps/${encId}/preview.png`),
      ]);
      if (seq !== registryState.previewSeq) return;

      if (!metaRes.ok) {
        const j = await metaRes.json().catch(() => ({}));
        throw new Error((j.error && `${j.error.code}: ${j.error.message}`) || `meta HTTP ${metaRes.status}`);
      }
      if (!pngRes.ok) {
        const j = await pngRes.json().catch(() => ({}));
        throw new Error((j.error && `${j.error.code}: ${j.error.message}`) || `preview HTTP ${pngRes.status}`);
      }

      const rawMeta = await metaRes.json();
      const blob = await pngRes.blob();
      if (seq !== registryState.previewSeq) return;

      const url = URL.createObjectURL(blob);
      previewObjectUrl = url;
      const img = new Image();
      img.onload = () => {
        if (seq !== registryState.previewSeq) return;
        const meta = normalizeRegistryMeta(rawMeta);
        const vStatus = String(rawMeta.verification_status || 'UNVERIFIED');
        registryState.preview = {
          mapVersionId: rawMeta.map_version_id || mvid,
          fitKey: (rawMeta.map_version_id || mvid) + ':' + (rawMeta.sha256 ? rawMeta.sha256.pgm_sha256 || '' : ''),
          loading: false,
          meta,
          rawMeta,
          img,
          verification_status: vStatus,
          verified: vStatus === 'VERIFIED',
          error: '',
        };
        registryState.offline = false;
        if (mapView) mapView.setRegistryPreview(registryState.preview);
        renderRegistryUi();
      };
      img.onerror = () => {
        if (seq !== registryState.previewSeq) return;
        clearPreviewObjectUrl();
        registryState.preview = {
          mapVersionId: mvid,
          fitKey: mvid,
          loading: false,
          meta: null,
          img: null,
          verification_status: 'UNKNOWN',
          verified: false,
          error: '미리보기 PNG 디코딩 실패',
        };
        if (mapView) mapView.setRegistryPreview(registryState.preview);
        renderRegistryUi();
      };
      img.src = url;
    } catch (e) {
      if (seq !== registryState.previewSeq) return;
      const isNet = e instanceof TypeError;
      if (isNet) registryState.offline = true;
      registryState.preview = {
        mapVersionId: mvid,
        fitKey: mvid,
        loading: false,
        meta: null,
        img: null,
        verification_status: 'UNKNOWN',
        verified: false,
        error: isNet ? '서버 연결 끊김 (OFFLINE)' : (e.message || '미리보기 불러오기 실패'),
      };
      if (mapView) mapView.setRegistryPreview(registryState.preview);
      renderRegistryUi();
    }
  }

  function renderFleetStrip(robots, missions) {
    $('fleetCount').textContent = robots.length + ' ROBOTS';
    if (!robots.length) {
      $('robots').innerHTML = '<div class="muted small">등록된 로봇이 없습니다.</div>';
      return;
    }
    $('robots').innerHTML = robots
      .map((r) => {
        const sg = H.classifySafetyGate(r);
        const mp = H.classifyMotorPath(r);
        const am = H.classifyAllowMissions(r);
        const origin = H.classifyFleetOrigin(r, missions);
        const selCls = r.robot_id === selectedRobotId ? ' selected' : '';
        const recCls = r.reconciliation_required ? ' reconcile' : '';
        return `<div class="rob-pill${selCls}${recCls}" data-robot="${H.esc(r.robot_id)}" tabindex="0">
          <div class="rob-pill-head">
            <strong>${H.esc(r.robot_id)}</strong>
            <span class="badge ${r.online ? 'on' : 'off'}">${r.online ? 'ONLINE' : 'OFFLINE'}</span>
            ${r.agent && r.agent.simulated ? '<span class="badge sim">SIMULATED</span>' : '<span class="badge">' + H.esc((r.agent && r.agent.mode) || 'UNKNOWN') + '</span>'}
            ${r.reconciliation_required ? '<span class="badge bad">RECONCILE</span>' : ''}
          </div>
          <div class="rob-pill-meta">
            <div><span class="muted">Safety Gate:</span> <span class="${sg.tone}">${H.esc(sg.label)}</span></div>
            <div><span class="muted">모터:</span> <span class="${mp.tone}">${H.esc(mp.status)}</span></div>
            <div><span class="muted">allow_missions:</span> <span class="${am.tone}">${am.value === null ? 'UNKNOWN' : String(am.value)}</span></div>
            <div><span class="muted">수신:</span> <span class="${r.online ? '' : 'bad'}">${H.fmtAge(r.last_seen_age_s)}</span></div>
            <div><span class="muted">Lease:</span> ${r.lease ? H.esc(r.lease.owner) + ' (' + H.fmt(r.lease.remaining_s, 0) + 's)' : '<span class="muted">없음</span>'}</div>
            <div><span class="muted">구분:</span> <span class="${origin.kind === 'rmf' ? 'warn' : ''}">${H.esc(origin.label)}</span></div>
          </div>
        </div>`;
      })
      .join('');
  }

  function renderSelectedRobotDetail(r) {
    if (!r) {
      $('selRobotTitle').textContent = '선택 없음';
      $('selRobotAge').textContent = '';
      $('reconcileBanner').classList.add('hidden');
      $('robotDetailBody').innerHTML = '<div class="muted small">로봇이 등록되면 상태가 표시됩니다.</div>';
      return;
    }

    const s = r.state || {};
    const a = r.agent || {};
    const p = s.pose || {};
    const b = s.battery || {};
    const l = s.lidar || {};
    const d = s.depth_camera || {};
    const sg = (s.safety_gate && typeof s.safety_gate === 'object') ? s.safety_gate : {};
    const gateClass = H.classifySafetyGate(r);
    const mp = H.classifyMotorPath(r);
    const am = H.classifyAllowMissions(r);
    const n = s.nav2 || {};
    const m = s.map || {};
    const ex = s.execution || {};
    const ng = ex.nav2_goals || {};
    const comm = s.communication || {};
    const ms = s.mission || {};
    const faults = H.collectFaultReasons(r);

    $('selRobotTitle').textContent = r.robot_id;
    $('selRobotTitle').className = 'badge ' + (r.online ? 'on' : 'off');
    $('selRobotAge').textContent = `마지막 보고: ${H.fmtAge(r.last_seen_age_s)} · 상태 나이: ${H.fmtAge(r.state_age_s)}`;

    const recBanner = $('reconcileBanner');
    if (r.reconciliation_required || r.reconciled === false) {
      recBanner.classList.remove('hidden');
      recBanner.textContent = r.reconciliation_required
        ? 'RECONCILIATION_REQUIRED: 미확인 실행/소유권 상태로 신규 이동이 차단됨 · 목표 자동 재전송 금지 (admin 증거 기반 복구 필요)'
        : '재동기화 대기 중 (reconciled=false): Agent /agent/reconcile 완료 전까지 이동 명령 차단';
    } else {
      recBanner.classList.add('hidden');
    }

    const row = (k, v, cls) => `<div>${k}</div><div class="${cls || ''}">${v}</div>`;
    const offlinePrefix = !r.online ? '<span class="bad">[OFFLINE 과거 표본]</span> ' : '';
    const stl = H.staleInfo(r);
    // Sensor rows of a robot that is offline / has an old state are history: say so on EACH row and grey them out.
    const srow = (k, v, cls) => (stl.stale ? row(k, `<span class="stale-tag">[${H.esc(stl.label)}]</span> ${v}`, 'stale-row') : row(k, v, cls));

    $('robotDetailBody').innerHTML = `
      <div class="detail-group">
        <h3>연결 · 세션 · 복구 상태</h3>
        <div class="grid">
          ${row('연결 상태', r.online ? 'ONLINE' : 'OFFLINE (통신 끊김 · 아래는 마지막 수신값)', r.online ? 'ok' : 'bad')}
          ${row('마지막 보고 / 상태 나이', `${H.fmtAge(r.last_seen_age_s)} / ${H.fmtAge(r.state_age_s)}`, r.online ? '' : 'bad')}
          ${row('Agent 세션', `${H.esc(a.mode || 'UNKNOWN')} · boot=${H.esc(a.boot_id || 'UNKNOWN')} · proto=${H.esc(a.protocol_version ?? '?')} / safety_v${H.esc(a.safety_contract_version ?? '?')}`)}
          ${row('Reconciliation', r.reconciliation_required ? 'RECONCILIATION_REQUIRED' : (r.reconciled ? '동기화 완료 (reconciled)' : '대기 (미동기화)'), r.reconciliation_required ? 'bad' : (r.reconciled ? 'ok' : 'warn'))}
          ${row('Agent↔Server 통신', comm.connected !== undefined ? `${comm.connected ? 'connected' : 'disconnected'} · failures ${H.esc(comm.failures ?? 0)} · reconnects ${H.esc(comm.reconnects ?? 0)}` : 'UNKNOWN', comm.connected === false ? 'bad' : 'muted')}
        </div>
      </div>

      <div class="detail-group">
        <h3>Safety Gate · 모터 잠금 · Safety Context v1</h3>
        <div class="grid">
          ${row('Safety Gate 상태', offlinePrefix + H.esc(gateClass.label), gateClass.tone)}
          ${row('Gate 상세', `fault_latched=${H.esc(sg.fault_latched ?? 'UNKNOWN')} · cmd_blocked=${H.esc(sg.cmd_blocked ?? 'UNKNOWN')} · fault_epoch=${H.esc(sg.fault_epoch ?? '-')} · boot=${H.esc(sg.boot_id || 'UNKNOWN')}`, sg.fault_latched ? 'bad' : '')}
          ${row('Gate 신선도', `rx_age=${H.fmt(sg.rx_age_s ?? sg.age_s)}s (fresh=${H.esc(sg.rx_fresh ?? 'UNKNOWN')}) · src_age=${H.fmt(sg.source_age_s)}s (fresh=${H.esc(sg.source_fresh ?? 'UNKNOWN')})`)}
          ${row('모터 출력 경로', `${H.esc(mp.label)}${mp.publishers.length ? ' · /cmd_vel: ' + H.esc(mp.publishers.join(', ')) : ''}`, mp.tone)}
          ${row('allow_missions', H.esc(am.label), am.tone)}
          ${row('movement_permitted', sg.movement_permitted === true ? 'true (Gate 보고값 · 이동 승인 아님)' : (sg.movement_permitted === false ? 'false (이동 차단)' : 'UNKNOWN'), sg.movement_permitted === true ? 'muted' : (sg.movement_permitted === false ? 'warn' : 'muted'))}
          ${row('운용 모드', `${H.esc(s.operation_mode || 'UNKNOWN')} (rev ${H.esc(s.operation_revision ?? '-')})`, s.operation_mode === 'NAVIGATION' ? 'ok' : 'muted')}
          ${row('Fault / 오류 원인', faults.length ? faults.map((f) => `<div>• ${H.esc(f)}</div>`).join('') : '없음', faults.length ? 'bad' : 'ok')}
        </div>
      </div>

      <div class="detail-group">
        <h3>배터리 · 센서 · 위치추정 · 지도 · Nav2</h3>
        <div class="grid">
          ${srow('배터리', b.voltage_v != null ? `${H.fmt(b.voltage_v)} V (백분율 미제공 · ${H.esc(b.status || '')})${b.error_code ? ' · err=' + H.esc(b.error_code) : ''}` : 'UNKNOWN', H.toneForStatus(b.status || 'UNKNOWN'))}
          ${srow('LiDAR', `${H.esc(l.status || 'NO_DATA')}${l.rate_hz != null ? ` · ${H.fmt(l.rate_hz, 1)} Hz · ${H.esc(l.beams_min)}~${H.esc(l.beams_max)} beams` : ''}${l.age_s != null ? ` · age ${H.fmt(H.shownAge(l.age_s, r))}s` : ''}`, H.toneForStatus(l.status || 'NO_DATA'))}
          ${srow('Depth 카메라', `${H.esc(d.status || 'NO_DATA')}${d.code ? ' · ' + H.esc(d.code) : ''}${d.age_s != null ? ' · age ' + H.fmt(H.shownAge(d.age_s, r)) + 's' : ''}`, H.toneForStatus(d.status || 'NO_DATA'))}
          ${srow('현재 위치 (' + H.esc(p.frame_id || 'UNKNOWN') + ')', p.status === 'OK' || p.status === 'STALE' ? `x ${H.fmt(p.x)} m · y ${H.fmt(p.y)} m · yaw ${H.fmt(p.yaw_rad)} rad (${H.fmt((p.yaw_rad || 0) * 180 / Math.PI, 1)}°)` : H.esc(p.status || 'NO_DATA'), H.toneForStatus(p.status || 'NO_DATA'))}
          ${srow('위치추정 (Localization)', `${H.esc(s.localization_status || 'UNKNOWN')} · map=${H.esc(s.localization_map_version_id || 'NONE')} · epoch=${H.esc(s.localization_epoch ?? '-')}`, H.toneForStatus(s.localization_status || 'UNKNOWN'))}
          ${srow('지도 상태', `${H.esc(m.status || 'NO_MAP')}${m.version ? ' · v' + H.esc(m.version) : ''}${m.known_cells != null ? ' · ' + H.esc(m.known_cells) + ' cells' : ''} · id=${H.esc(s.map_version_id || 'NONE')} (epoch=${H.esc(s.map_epoch ?? '-')}, verified=${H.esc(s.map_verified ?? 'UNKNOWN')})`, H.toneForStatus(m.status || 'NO_MAP'))}
          ${srow('Nav2 상태', `${H.esc(n.status || 'UNAVAILABLE')} (ready=${H.esc(n.action_server_ready ?? 'UNKNOWN')}) · goals known=${H.esc(ng.known ?? 'UNKNOWN')}, active=${H.esc(ng.active_count ?? 'UNKNOWN')}, arr_age=${H.fmt(H.shownAge(ng.age_s, r))}s`, H.toneForStatus(n.status || 'UNAVAILABLE'))}
          ${row('현재 실행/임무', ex.active ? `${H.esc(ex.active.mission_id)} (handle=${H.esc(ex.active.goal_handle_available)}, cancel=${H.esc(ex.active.cancel_requested)})` : (ms.active_mission_id ? `${H.esc(ms.active_mission_id)} · ${H.esc(ms.state || '')}` : '없음'))}
        </div>
      </div>
    `;
  }

  function renderLeaseAndReadiness(r) {
    if (!r) {
      $('leaseStatusBadge').textContent = '로봇 없음';
      $('leaseStatusBadge').className = 'badge';
      $('leaseInfo').innerHTML = '';
      $('safetyReadyBadge').textContent = '로봇 없음';
      $('safetyPrecheckSummary').innerHTML = '';
      return;
    }
    const l = r.lease;
    if (l) {
      const mine = me && l.owner === me.name;
      $('leaseStatusBadge').textContent = mine ? `내 제어권 (${H.fmt(l.remaining_s, 1)}s)` : `${l.owner} 점유 중`;
      $('leaseStatusBadge').className = 'badge ' + (mine ? 'on' : 'warn');
      $('leaseInfo').innerHTML = `
        <div>소유자</div><div class="${mine ? 'ok' : 'warn'}">${H.esc(l.owner)}</div>
        <div>Lease Epoch</div><div>${H.esc(l.epoch)}</div>
        <div>남은 TTL</div><div>${H.fmt(l.remaining_s, 1)} s / ${H.fmt(l.ttl_s, 0)} s</div>
        <div>메모</div><div>${H.esc(l.note || '-')}</div>`;
    } else {
      $('leaseStatusBadge').textContent = '미점유';
      $('leaseStatusBadge').className = 'badge';
      $('leaseInfo').innerHTML = '<div>상태</div><div class="muted">현재 활성 제어권 Lease가 없습니다. 임무 생성 전 [획득]을 누르세요.</div>';
    }

    const readiness = H.evaluateSafetyReadiness(r, me ? me.name : '');
    $('safetyReadyBadge').textContent = readiness.readyForMission ? '보고 조건 충족 · 서버 승인 필요' : `차단 (${readiness.blockers.length}건)`;
    $('safetyReadyBadge').className = 'badge ' + (readiness.readyForMission ? 'progress' : 'bad');
    $('safetyPrecheckSummary').innerHTML = readiness.checks
      .map((c) => `<div class="precheck-item ${c.ok ? 'pass' : 'fail'}"><span>${c.ok ? '✓' : '✕'} ${H.esc(c.label)}</span><span class="${c.ok ? 'ok' : 'bad'}">${H.esc(c.detail)}</span></div>`)
      .join('');
  }

  function renderPreflight(r) {
    if (!r) {
      $('preflightChecklist').innerHTML = '<div class="muted small">로봇을 선택하세요.</div>';
      return;
    }
    const s = r.state || {};
    const sg = s.safety_gate || {};
    const mp = H.classifyMotorPath(r);
    const am = H.classifyAllowMissions(r);
    const items = [
      { label: 'Agent 온라인 연결', ok: r.online === true, val: r.online ? `ONLINE (${H.fmtAge(r.last_seen_age_s)})` : 'OFFLINE' },
      { label: 'Safety Gate 상태', ok: r.online && sg.status === 'NORMAL' && sg.fault_latched === false, val: H.classifySafetyGate(r).label },
      { label: '모터 잠금 / 출력 경로', ok: mp.status === 'LOCKED' || mp.status === 'SIM_GATED', val: mp.label },
      { label: 'allow_missions 설정', ok: am.value !== null, val: am.label },
      { label: 'LiDAR 스캔 수신', ok: r.online && (s.lidar || {}).status === 'OK', val: `${(s.lidar || {}).status || 'NO_DATA'}` },
      { label: 'Depth 카메라 상태', ok: r.online && (s.depth_camera || {}).status === 'OK', val: `${(s.depth_camera || {}).status || 'NO_DATA'}` },
      { label: '위치추정 (Localization)', ok: s.localization_status === 'CONFIRMED', val: `${s.localization_status || 'UNKNOWN'}` },
      { label: 'Nav2 Action Server', ok: (s.nav2 || {}).action_server_ready === true, val: `${(s.nav2 || {}).status || 'UNAVAILABLE'}` },
      { label: 'Reconciliation 차단 여부', ok: !r.reconciliation_required && r.reconciled === true, val: r.reconciliation_required ? 'RECONCILIATION_REQUIRED' : (r.reconciled ? '정상' : '미동기화') },
    ];
    $('preflightChecklist').innerHTML = items
      .map((it) => `<div class="check-row"><span>${H.esc(it.label)}</span><span class="badge ${it.ok ? 'on' : 'warn'}">${H.esc(it.val)}</span></div>`)
      .join('');
  }

  function renderRmfAndMatrix(robots, missions) {
    const rmfMissions = missions.filter((m) => H.classifyMissionSource(m) === 'RMF');
    const rmfActive = rmfMissions.filter((m) => !m.terminal).length;
    const rmfFailed = rmfMissions.filter((m) => ['FAILED', 'REJECTED', 'EXPIRED', 'RECONCILIATION_REQUIRED'].includes(m.status)).length;
    const rmfLeases = robots.filter((r) => r.lease && r.lease.owner === 'rmf-adapter').map((r) => r.robot_id);

    $('rmfSummaryGrid').innerHTML = `
      <div>RMF Core WebSocket/Graph</div><div class="warn">UNAVAILABLE (fleet_server 직접 미연동)</div>
      <div>RMF Adapter Lease 보유 로봇</div><div>${rmfLeases.length ? H.esc(rmfLeases.join(', ')) : '<span class="muted">없음</span>'}</div>
      <div>RMF 출처 임무 (전체/진행/실패)</div><div>${rmfMissions.length}건 / 진행 ${rmfActive}건 / 실패·거부 ${rmfFailed}건</div>
      <div>다중 Fleet 구분</div><div class="muted">현재 서버 단일 그룹 (향후 fleet_name 필드 확장 대응)</div>
    `;

    $('fleetMatrixTable').querySelector('tbody').innerHTML = robots.length
      ? robots
          .map((r) => {
            const sg = H.classifySafetyGate(r);
            const mp = H.classifyMotorPath(r);
            const am = H.classifyAllowMissions(r);
            const origin = H.classifyFleetOrigin(r, missions);
            const ms = (r.state || {}).mission || {};
            return `<tr data-pick-robot="${H.esc(r.robot_id)}">
              <td><strong>${H.esc(r.robot_id)}</strong></td>
              <td>${r.agent && r.agent.simulated ? '<span class="badge sim">SIM</span> ' : ''}${H.esc(origin.label)}</td>
              <td><span class="badge ${r.online ? 'on' : 'off'}">${r.online ? 'ONLINE' : 'OFFLINE'}</span></td>
              <td class="${sg.tone}">${H.esc(sg.label)}</td>
              <td><span class="${mp.tone}">${H.esc(mp.status)}</span> / <span class="${am.tone}">${am.value === null ? 'UNKNOWN' : String(am.value)}</span></td>
              <td>${r.lease ? H.esc(r.lease.owner) + ' (e' + r.lease.epoch + ')' : '-'}</td>
              <td>${ms.active_mission_id ? H.esc(ms.active_mission_id) + ' (' + H.esc(ms.state || '') + ')' : '-'}</td>
            </tr>`;
          })
          .join('')
      : '<tr><td colspan="7" class="muted">등록된 로봇이 없습니다.</td></tr>';
  }

  function missionRowHtml(m, includeCmds) {
    const canCtl = me && (me.role === 'operator' || me.role === 'admin');
    const isAdmin = me && me.role === 'admin';
    const src = H.classifyMissionSource(m);
    const srcBadge = src === 'RMF' ? '<span class="badge rmf">RMF</span>' : '<span class="badge">WEB/API</span>';
    const prog = m.progress && m.progress.distance_remaining_m != null ? `남은거리 ${H.fmt(m.progress.distance_remaining_m)} m` : '';
    const cmdsTxt = includeCmds && Array.isArray(m.commands) && m.commands.length
      ? '<div class="muted small">' + m.commands.map((c) => `${H.esc(c.command_id)}(${H.esc(c.type)}:${H.esc(c.status)}, try=${c.attempts})`).join(' · ') + '</div>'
      : '';
    const cancelBtn = !m.terminal && m.status !== 'CANCELING' && canCtl ? `<button type="button" class="small" data-cancel="${H.esc(m.mission_id)}">취소</button>` : '';
    const recoverBtn = m.status === 'RECONCILIATION_REQUIRED' && isAdmin ? `<button type="button" class="small danger" data-recover-mission="${H.esc(m.mission_id)}">복구(admin)</button>` : '';
    const when = m.created_at ? new Date(m.created_at * 1000).toLocaleTimeString() : '-';

    if (includeCmds) {
      return `<tr>
        <td><code>${H.esc(m.mission_id)}</code></td>
        <td class="small">${H.esc(when)}</td>
        <td>${H.esc(m.robot_id)}</td>
        <td>${srcBadge} <span class="small">${H.esc(m.created_by || '-')}</span></td>
        <td class="small">(${H.fmt(m.goal && m.goal.x)}, ${H.fmt(m.goal && m.goal.y)}, ${H.fmt(m.goal && m.goal.yaw_rad)})</td>
        <td class="st-${H.esc(m.status)}">${H.esc(m.status)}</td>
        <td class="small">${m.detail ? H.esc(m.detail) : '-'}${prog ? ' · ' + H.esc(prog) : ''}${cmdsTxt}</td>
        <td><div class="row-gap">${cancelBtn}${recoverBtn}</div></td>
      </tr>`;
    }
    return `<tr>
      <td><code>${H.esc(m.mission_id)}</code></td>
      <td>${H.esc(m.robot_id)}</td>
      <td>${srcBadge} <span class="small">${H.esc(m.created_by || '-')}</span></td>
      <td class="small">(${H.fmt(m.goal && m.goal.x)}, ${H.fmt(m.goal && m.goal.y)}, ${H.fmt(m.goal && m.goal.yaw_rad)})</td>
      <td class="st-${H.esc(m.status)}">${H.esc(m.status)}</td>
      <td class="small">${m.detail ? H.esc(m.detail) : '-'}${prog ? ' · ' + H.esc(prog) : ''}</td>
      <td><div class="row-gap">${cancelBtn}${recoverBtn}</div></td>
    </tr>`;
  }

  function renderMissionsAndEvents() {
    if (!data) return;
    const summaryMissions = (data.missions || []).slice(0, 20);
    $('missions').querySelector('tbody').innerHTML = summaryMissions.length
      ? summaryMissions.map((m) => missionRowHtml(m, false)).join('')
      : '<tr><td colspan="7" class="muted">임무 기록이 없습니다.</td></tr>';

    const histSource = fullMissions || data.missions || [];
    const filtered = H.filterMissions(histSource, {
      robotId: $('histRobot').value,
      source: $('histSource').value,
      statusGroup: $('histStatus').value,
    });
    $('missionHistoryTable').querySelector('tbody').innerHTML = filtered.length
      ? filtered.map((m) => missionRowHtml(m, true)).join('')
      : '<tr><td colspan="8" class="muted">조건에 맞는 임무 이력이 없습니다.</td></tr>';

    const evRobot = $('evRobot').value;
    const evLevel = $('evLevel').value;
    const evFiltered = (data.events || []).filter((e) => {
      if (evRobot !== 'ALL' && (e.robot_id || '') !== evRobot) return false;
      if (evLevel === 'warning' && !['warning', 'error'].includes(e.level)) return false;
      if (evLevel === 'info' && e.level !== 'info') return false;
      return true;
    });
    $('events').innerHTML = evFiltered.length
      ? evFiltered
          .map((e) => `<div class="${H.esc(e.level)}">${new Date(e.ts * 1000).toLocaleTimeString()} [${H.esc(e.robot_id || '-')}] <strong>${H.esc(e.kind)}</strong>: ${H.esc(e.message)}</div>`)
          .join('')
      : '<div class="muted">조건에 맞는 이벤트가 없습니다.</div>';
  }

  async function fetchFullMissions() {
    try {
      const r = await api('/missions?limit=200');
      if (!r.ok) return;
      const j = await r.json();
      fullMissions = j.missions || [];
      renderMissionsAndEvents();
    } catch (e) { /* ignore */ }
  }

  function selectRobot(rid) {
    const changed = selectedRobotId !== rid;
    selectedRobotId = rid;
    if (changed) { selectedMissionId = ''; $('goalConfirmed').checked = false; }
    if ($('mapRobot').value !== rid && [...$('mapRobot').options].some((o) => o.value === rid)) {
      $('mapRobot').value = rid;
    }
    if ($('mrobot').value !== rid && [...$('mrobot').options].some((o) => o.value === rid)) {
      $('mrobot').value = rid;
    }
    $('leaseErr').textContent = '';
    $('recoverRobotErr').textContent = '';
    if (me && (me.role === 'operator' || me.role === 'admin')) {
      $('merr').textContent = '';
    }
    if (mapView) mapView.setPreviewGoal(null, null);
    renderAll();
    if (mapView) mapView.fitRobot();
    updateGoalReview();
  }

  function renderAll() {
    if (!data) return;
    const disconnected = registryState.offline || performance.now() - lastDashboardAt > 3000;
    const robots = (data.robots || []).map(r => disconnected ? { ...r, online: false } : r);
    const hasFleet = !disconnected;
    $('overviewFleetOnline').textContent = hasFleet ? `${robots.filter(r=>r.online).length} / ${robots.length}` : 'UNKNOWN';
    $('overviewFleetMissions').textContent = hasFleet ? `${(data.missions || []).filter(m=>!m.terminal).length} · 최근 조회 기준` : 'UNKNOWN';
    $('overviewFleetFaults').textContent = hasFleet ? `${robots.filter(r=>r.online && H.classifySafetyGate(r).category==='FAULT').length} · 미확인 ${robots.filter(r=>!r.online || ['UNKNOWN','STALE'].includes(H.classifySafetyGate(r).category)).length}` : 'UNKNOWN';
    $('overviewFleetReconcile').textContent = hasFleet ? `${robots.filter(r=>r.reconciliation_required).length} 대` : 'UNKNOWN';
    const missions = data.missions || [];
    if (!robots.some((r) => r.robot_id === selectedRobotId)) {
      selectedRobotId = robots[0] ? robots[0].robot_id : '';
    }
    const sel = robots.find((r) => r.robot_id === selectedRobotId) || null;
    const rec = !disconnected && sel?.online ? sel.state?.recording : null;
    const recStatus = rec?.status || 'UNKNOWN';
    $('recordingStatus').textContent = `${selectedRobotId || '로봇 미선택'} · ${recStatus}${rec?.session_id ? ` · 세션 ${rec.session_id}` : ''} · ${rec?.detail || ''}${rec?.returncode != null ? ` · 종료 코드 ${rec.returncode}` : ''}`;
    $('recordingPath').textContent = rec?.run_dir ? `저장 경로: ${rec.run_dir}${rec.bag_bytes != null ? ` · ${rec.bag_bytes} bytes` : ''}` : '저장 경로 미확인';
    const recRole = me && (me.role === 'operator' || me.role === 'admin');
    $('recordingStart').disabled = !recRole || recordingBusy || !rec?.enabled || !['IDLE','VERIFIED','FAILED'].includes(recStatus);
    $('recordingStop').disabled = !recRole || recordingBusy || !rec?.enabled || recStatus !== 'RECORDING' || !rec.session_id;
    const slam = !disconnected && sel?.online ? sel.state?.slam : null;
    $('slamStatus').textContent = `${selectedRobotId || '미선택'} · ${sel?.state?.mapping_mode || 'IDLE'} · ${slam?.status || 'UNKNOWN'} · ${slam?.session_id || '세션 없음'} · ${slam?.detail || ''}`;
    $('slamPath').textContent = slam?.save_path ? `지도: ${slam.save_path} · ${slam.map_version_id || 'Registry 미확인'} · ${slam.floor_id || '층 미확인'} · posegraph ${slam.posegraph_saved ? (slam.posegraph_registered ? '저장·등록' : '저장·미등록') : '미확인'}` : '지도 미저장';
    $('slamRecording').textContent = `녹화: ${recStatus} · ${rec?.session_id || '세션 없음'}${slam?.session_id && rec?.session_id === slam.session_id ? ' · 동일 세션' : ''}`;
    $('slamStart').disabled = !recRole || slamBusy || !slam?.enabled || !['IDLE','STOPPED','COMPLETED','FAILED'].includes(slam.status);
    $('slamSave').disabled = !recRole || slamBusy || !slam?.enabled || slam.status !== 'RUNNING';
    $('slamStop').disabled = !recRole || slamBusy || !slam?.enabled || !['RUNNING','DEGRADED','STARTING','ERROR'].includes(slam.status);

    // Alert badge summary
    const recCount = robots.filter((r) => r.reconciliation_required).length;
    const faultCount = robots.filter((r) => H.classifySafetyGate(r).category === 'FAULT').length;
    if (recCount + faultCount > 0) {
      $('alertSummary').classList.remove('hidden');
      $('alertSummary').textContent = `주의: 고장 ${faultCount} · 복구필요 ${recCount}`;
    } else {
      $('alertSummary').classList.add('hidden');
    }

    renderWorkspace(robots, missions);
    renderFleetStrip(robots, missions);
    renderConsole(sel, missions, disconnected);
    renderSelectedRobotDetail(sel);
    renderLeaseAndReadiness(sel);
    renderPreflight(sel);
    renderRmfAndMatrix(robots, fullMissions || missions);
    renderMissionsAndEvents();

    // Update map view (preserve Map Registry preview mode across 1Hz telemetry ticks)
    const mapRid = $('mapRobot').value || selectedRobotId;
    const mapRobot = robots.find((r) => r.robot_id === mapRid) || sel;
    if (!registryState.preview) {
      if (mapRobot && mapRobot.map_meta) {
        const meta = mapRobot.map_meta;
        const st = mapRobot.state || {};
        const flags = [];
        if (!mapRobot.online) flags.push('OFFLINE 과거');
        if (st.map && st.map.status === 'STALE') flags.push('STALE');
        if ((st.map_version_id && st.localization_map_version_id && st.map_version_id !== st.localization_map_version_id) || st.map_verified === false) {
          flags.push('버전불일치/미검증');
        }
        const flagSuffix = flags.length ? ` · [${flags.join(', ')}]` : '';
        $('mapinfo').textContent = `v${mapRobot.map_version || ''} · ${meta.width}x${meta.height} @${meta.resolution}m · yaw ${H.fmt((meta.origin_yaw || 0) * 180 / Math.PI, 1)}°${flagSuffix}`;
        $('mapinfo').className = flags.length ? 'warn small' : 'muted small';
        const liveOpt = $('mapVersionSelect') && $('mapVersionSelect').querySelector('option[value="__live__"]');
        if (liveOpt) {
          liveOpt.textContent = `라이브 지도 v${mapRobot.map_version || '-'} (${st.map_version_id || 'unregistered'})`;
        }
      } else {
        $('mapinfo').textContent = '지도 없음 (NO_MAP)';
        $('mapinfo').className = 'muted small';
        const liveOpt = $('mapVersionSelect') && $('mapVersionSelect').querySelector('option[value="__live__"]');
        if (liveOpt) {
          liveOpt.textContent = '라이브 지도 (로봇 탑재 Agent · NO_MAP)';
        }
      }
    }
    updateMissionFormBlockState();
    if (mapView) mapView.update(mapRobot, missions.filter(m => m.robot_id === mapRobot?.robot_id));
    updateGoalReview();
  }

  async function recordAction(action) {
    const robot = (data?.robots || []).find(r => r.robot_id === selectedRobotId);
    const rec = robot?.state?.recording;
    const activeSlam = robot?.state?.slam;
    if (action === 'start' && ['STARTING','RUNNING','SAVING','DEGRADED'].includes(activeSlam?.status) && $('recordingSessionId').value.trim() && $('recordingSessionId').value.trim() !== activeSlam.session_id) {
      $('recordingError').className = 'bad small'; $('recordingError').textContent = '실행 중인 SLAM과 동일한 Session ID를 사용하세요.'; return;
    }
    const sid = action === 'start' ? ($('recordingSessionId').value.trim() || (['STARTING','RUNNING','SAVING','DEGRADED'].includes(activeSlam?.status) ? activeSlam.session_id : '') || 'rec_' + uuid().replace(/-/g, '')) : rec?.session_id;
    if (!robot || !sid || recordingBusy) return;
    recordingBusy = true; $('recordingError').textContent = ''; renderAll();
    try {
      const res = await api(`/robots/${encodeURIComponent(robot.robot_id)}/recording/${action}`, {method:'POST', body:JSON.stringify({session_id:sid})});
      const body = await res.json();
      if (!res.ok) throw new Error(body.error?.message || `HTTP ${res.status}`);
      $('recordingError').className = 'muted small'; $('recordingError').textContent = '요청 접수 · 위 상태에서 결과 확인';
    } catch (e) { $('recordingError').className = 'bad small'; $('recordingError').textContent = e.message; }
    finally { recordingBusy = false; await tick(); }
  }
  $('recordingStart').addEventListener('click', () => recordAction('start'));
  $('recordingStop').addEventListener('click', () => recordAction('stop'));
  async function slamAction(action) {
    const robot=(data?.robots||[]).find(r=>r.robot_id===selectedRobotId), slam=robot?.state?.slam, rec=robot?.state?.recording;
    const sid=action==='start' ? (['STARTING','RECORDING'].includes(rec?.status) ? rec.session_id : 'slam_'+uuid().replace(/-/g,'')) : slam?.session_id;
    if(!robot || !sid || slamBusy)return;
    slamBusy=true; $('slamError').textContent=''; renderAll();
    try {const r=await api(`/robots/${encodeURIComponent(robot.robot_id)}/slam/${action}`,{method:'POST',body:JSON.stringify({session_id:sid, floor_id:registryState.selectedFloorId})});const j=await r.json();if(!r.ok)throw Error(j.error?.message||`HTTP ${r.status}`);$('slamError').className='muted small';$('slamError').textContent='요청 접수 · 위 상태에서 결과 확인';}
    catch(e){$('slamError').className='bad small';$('slamError').textContent=e.message;} finally {slamBusy=false;await tick();}
  }
  ['start','save','stop'].forEach(action=>$('slam'+action[0].toUpperCase()+action.slice(1)).addEventListener('click',()=>slamAction(action)));

  async function tick() {
    if (tickBusy) return;
    tickBusy = true;
    try { await fetchDashboard(); if (window.MappingPanel) await window.MappingPanel.refresh(); if (window.MapSavePanel) await window.MapSavePanel.refresh(); } finally { tickBusy = false; }
  }

  async function fetchDashboard() {
    let r;
    try {
      r = await api('/dashboard', { timeoutMs: 2500 });
    } catch (e) {
      $('srv').textContent = '서버 연결 끊김';
      $('srv').className = 'bad small';
      if (!registryState.offline) {
        registryState.offline = true;
        renderRegistryUi();
      }
      renderAll();
      return;
    }
    if (!r.ok) { registryState.offline = true; renderAll(); renderRegistryUi(); return; }
    const wasOffline = registryState.offline;
    registryState.offline = false;
    data = await r.json();
    lastDashboardAt = performance.now();
    const sv = data.server;
    $('srv').textContent = `서버 ${new Date(sv.time * 1000).toLocaleTimeString()} · boot ${sv.boot_id} · v${sv.version}`;
    $('srv').className = 'muted small';

    const ids = (data.robots || []).map((x) => x.robot_id);
    fillSelect($('mapRobot'), ids, false);
    fillSelect($('mrobot'), ids, false);
    fillSelect($('histRobot'), ids, true);
    fillSelect($('evRobot'), ids, true);

    for (const x of data.robots || []) {
      if (x.map_version) loadMap(x.robot_id, x.map_version);
    }
    if ((!registryState.initialized && !registryState.loading) || wasOffline) {
      fetchRegistryHierarchy();
    } else if (wasOffline) {
      renderRegistryUi();
    }
    renderAll();
  }

  // Click handlers
  $('robots').addEventListener('click', (e) => {
    const card = e.target.closest('[data-robot]');
    if (card) selectRobot(card.dataset.robot);
  });
  $('fleetMatrixTable').addEventListener('click', (e) => {
    const tr = e.target.closest('[data-pick-robot]');
    if (tr) selectRobot(tr.dataset.pickRobot);
  });
  $('mapRobot').onchange = () => selectRobot($('mapRobot').value);
  $('mrobot').onchange = () => selectRobot($('mrobot').value);
  $('mapBuilding').onchange = async () => {
    registryState.selectedBuildingId = $('mapBuilding').value;
    const bSel = findBuilding(registryState.selectedBuildingId);
    const firstFloor = (bSel && bSel.floors && bSel.floors[0]) ? bSel.floors[0].floor_id : '';
    registryState.selectedFloorId = firstFloor;
    await selectRegistryMapVersion('__live__');
    await fetchFloorMaps(firstFloor);
  };
  $('mapLevel').onchange = async () => {
    registryState.selectedFloorId = $('mapLevel').value;
    await selectRegistryMapVersion('__live__');
    await fetchFloorMaps(registryState.selectedFloorId);
  };
  $('mapArea').onchange = async () => {
    registryState.selectedAreaId = $('mapArea').value;
    await selectRegistryMapVersion('__live__');
  };
  $('mapVersionSelect').onchange = () => {
    selectRegistryMapVersion($('mapVersionSelect').value);
  };
  $('mapRegistryRefresh').onclick = () => {
    fetchRegistryHierarchy();
  };
  if (window.MappingPanel) {
    window.MappingPanel.init({
      api,
      getMe: () => me,
      getRobot: () => (data?.robots || []).find((r) => r.robot_id === selectedRobotId) || null,
      getSelection: () => ({ floorId: registryState.selectedFloorId, areaId: registryState.selectedAreaId, mapVersionId: registryState.selectedMapVersionId }),
    });
  }
  if (window.MapSavePanel) {
    window.MapSavePanel.init({
      api,
      getMe: () => me,
      getRobot: () => (data?.robots || []).find((r) => r.robot_id === selectedRobotId) || null,
      getSelection: () => ({ floorId: registryState.selectedFloorId, areaId: registryState.selectedAreaId }),
      getTargetText: () => {
        const b = findBuilding(registryState.selectedBuildingId);
        const f = findFloor(b, registryState.selectedFloorId);
        if (!f) return '<span class="warn">등록 대상 층이 없습니다. 위에서 건물·층(필요하면 영역)을 선택하거나 아래 "지도 등록" 패널에서 먼저 만드세요.</span>';
        const a = ((f.areas) || []).find((x) => x.area_id === registryState.selectedAreaId);
        return `등록 대상: <span class="mono">${H.esc([b && b.facility_name, b && (b.name || b.building_id), f.level_name || f.floor_id, a && (a.name || a.area_id)].filter(Boolean).join(' › '))}</span>${a ? '' : ' · 영역 미지정'}`;
      },
      onRegistered: () => { fetchRegistryHierarchy(); },
    });
  }
  if (window.RegistryUpload) {
    window.RegistryUpload.init({
      api,
      getMe: () => me,
      getFacilities: () => registryFacilities || [],
      getSelection: () => ({
        buildingId: registryState.selectedBuildingId,
        floorId: registryState.selectedFloorId,
        areaId: registryState.selectedAreaId,
        mapVersionId: registryState.selectedMapVersionId,
      }),
      getAreas: () => {
        const b = findBuilding(registryState.selectedBuildingId);
        const f = findFloor(b, registryState.selectedFloorId);
        return (f && f.areas) || [];
      },
      getBuildingInfo: () => findBuilding(registryState.selectedBuildingId),
      getFloorInfo: () => {
        const b = findBuilding(registryState.selectedBuildingId);
        const f = findFloor(b, registryState.selectedFloorId);
        return f ? { building: (b && (b.name || b.building_id)) || '', level: f.level_name || f.floor_id, floorId: f.floor_id } : null;
      },
      refresh: async () => { await fetchRegistryHierarchy(); },
    });
  }

  ['histRobot', 'histSource', 'histStatus', 'evRobot', 'evLevel'].forEach((id) => {
    $(id).onchange = renderMissionsAndEvents;
  });
  $('histRefresh').onclick = fetchFullMissions;

  // Lease actions
  async function doLease(action) {
    $('leaseErr').textContent = '';
    const rid = selectedRobotId || $('mrobot').value;
    if (!rid) {
      $('leaseErr').textContent = '대상 로봇이 없습니다';
      return;
    }
    const ttl = Number($('leaseTtl').value);
    const note = $('leaseNote').value.trim();
    try {
      const r = await api(`/robots/${encodeURIComponent(rid)}/lock`, {
        method: 'POST',
        body: JSON.stringify({ action, ttl_s: ttl, note }),
      });
      if (!r.ok) {
        const j = await r.json().catch(() => ({}));
        $('leaseErr').textContent = (j.error && j.error.message) || `오류 ${r.status}`;
      }
    } catch (e) {
      $('leaseErr').textContent = '요청 실패: ' + e.message;
    }
    await tick();
  }

  $('leaseAcquire').onclick = () => doLease('acquire');
  $('leaseRenew').onclick = () => doLease('renew');
  $('leaseRelease').onclick = () => doLease('release');

  // Localhost mock-only locked path: the Agent's selector independently checks
  // the graph, Gate, watchdog, lease and both Nav2 status streams.
  const manualHost = document.createElement('section');
  manualHost.className = 'card';
  $('leaseCard').after(manualHost);
  const manualPanel = window.ManualControlPanel.mount(manualHost, {
    request: api,
    refreshEvidence: () => fetchDashboard(),
    isActive: () => activeTab === 'control' && !showAllPanels,
    getContext: () => {
      const robot = data?.robots?.find(r => r.robot_id === selectedRobotId);
      const agent = robot?.agent || {}, state = robot?.state || {};
      const arbiter = state.velocity_arbiter || {};
      const lockedPath = agent.mode === 'mock' && agent.simulated === true &&
        agent.locked_velocity_arbiter === true && arbiter.motor_output === false;
      const ready = lockedPath && arbiter.ready === true;
      const age = typeof robot?.state_age_s === 'number' && lastDashboardAt
        ? robot.state_age_s * 1000 + performance.now() - lastDashboardAt : Infinity;
      return {robotId:selectedRobotId, canControl:!!me && ['operator','admin'].includes(me.role),
        online:robot?.online === true, agentReady:robot?.reconciled === true && !robot?.reconciliation_required,
        motorLocked:lockedPath, leaseOwned:!!me && robot?.lease?.owner === me.name,
        leaseEpoch:robot?.lease?.epoch, mode:robot?.control_mode || 'UNKNOWN',
        operatingMode:robot?.operating_mode ? robot.operating_mode.mode : null,
        blocker:lockedPath && !ready ? (arbiter.blocker || 'STARTUP') : '',
        navIdleProven:ready, gateNormal:ready, evidenceAgeMs:age, agentBootId:agent.boot_id,
        serverBootId:data?.server?.boot_id};
    }
  });
  // Evidence refresh while a manual session is active (see ManualControlPanel.active): every 300 ms instead of the 1 s poll,
  // so the panel's 1000 ms evidence bound is not crossed by the poll period alone. Only /dashboard (runs even while the 1 s tick
  // is refreshing the mapping panels).
  let evidenceBusy = false;
  setInterval(async () => {
    if (evidenceBusy || typeof manualPanel.active !== 'function' || !manualPanel.active()) return;
    evidenceBusy = true;
    try { await fetchDashboard(); } catch (_) { /* the panel's own checks stop the session on stale evidence */ } finally { evidenceBusy = false; }
  }, 300);
  // Operating mode card (IDLE / MANUAL / MAPPING / NAVIGATION) directly under the Lease card.
  const modeHost = document.createElement('section');
  modeHost.className = 'card';
  modeHost.id = 'operatingModeCard';
  $('leaseCard').after(modeHost);
  window.OperatingModePanel.mount(modeHost, {
    request: api,
    refresh: () => tick(),
    getMe: () => me,
    getRobot: () => data?.robots?.find(r => r.robot_id === (selectedRobotId || $('mrobot').value)) || null,
  });
  // Saved-map navigation card (map version -> LOAD -> initial pose -> localization -> goal check) under the mode card.
  const navHost = document.createElement('section');
  navHost.className = 'card';
  navHost.id = 'navigationCard';
  modeHost.after(navHost);
  window.NavigationPanel.mount(navHost, {
    request: api,
    refresh: () => tick(),
    getMe: () => me,
    getRobot: () => data?.robots?.find(r => r.robot_id === (selectedRobotId || $('mrobot').value)) || null,
    getGoal: () => ({ x: +$('mx').value, y: +$('my').value, yaw: +$('myaw').value }),
  });
  // Admin Gate reset card (2026-11-21): status, fault reasons, Gate/server readiness, request result. Server enforces the admin role.
  const gateResetHost = document.createElement('section');
  gateResetHost.className = 'card';
  gateResetHost.id = 'gateResetCard';
  navHost.after(gateResetHost);
  window.GateResetPanel.mount(gateResetHost, {
    request: api,
    getMe: () => me,
    getRobot: () => data?.robots?.find(r => r.robot_id === (selectedRobotId || $('mrobot').value)) || null,
  });
  // Phase 4: robot readiness and services (admin actions are server- and supervisor-checked).
  const systemHost = document.createElement('section');
  systemHost.className = 'card';
  systemHost.id = 'systemReadinessCard';
  gateResetHost.after(systemHost);
  if (window.SystemPanel) window.SystemPanel.mount(systemHost, {
    request: api,
    getMe: () => me,
    getRobot: () => data?.robots?.find(r => r.robot_id === (selectedRobotId || $('mrobot').value)) || null,
  });
  // Phase 5: visitor guide administration (service open/close, destinations, admin visitor links + QR).
  const guideHost = document.createElement('section');
  guideHost.className = 'card';
  guideHost.id = 'guideAdminCard';
  systemHost.after(guideHost);
  if (window.GuidePanel) window.GuidePanel.mount(guideHost, {
    request: api,
    getMe: () => me,
    getRobot: () => data?.robots?.find(r => r.robot_id === (selectedRobotId || $('mrobot').value)) || null,
    getGoal: () => ({ x: +$('mx').value, y: +$('my').value, yaw: +$('myaw').value }),
  });
  $('leaseForceRelease').onclick = () => {
    const rid = selectedRobotId || $('mrobot').value;
    const robotObj = data && (data.robots || []).find((x) => x.robot_id === rid);
    const owner = robotObj && robotObj.lease ? robotObj.lease.owner : '없음';
    if (!window.confirm(`[관리자 확인] ${rid}의 현재 제어권 Lease(소유자: ${owner})를 강제 해제하시겠습니까? 진행 중인 운영/RMF 제어권과 충돌할 수 있습니다.`)) {
      return;
    }
    doLease('force_release');
  };

  // Mission create
  $('mform').onsubmit = async (ev) => {
    ev.preventDefault();
    $('merr').textContent = '';
    if (!$('goalConfirmed').checked) { $('merr').textContent = '선택 로봇과 목표 좌표를 먼저 확인하세요.'; return; }
    const pre = goalPrecheck();
    if (!pre.ok) { $('merr').textContent = '목표 사전 검사 실패: ' + pre.reasons.join(', '); return; }
    if (registryState.preview) {
      $('merr').textContent = '레지스트리 미리보기 모드에서는 NavigateToPose 임무를 전송할 수 없습니다. 라이브 지도로 전환하세요.';
      return;
    }
    const rid = $('mrobot').value;
    const robotObj = data && (data.robots || []).find((x) => x.robot_id === rid);
    const body = {
      robot_id: rid,
      type: 'navigate_to_pose',
      goal: { frame_id: 'map', x: +$('mx').value, y: +$('my').value, yaw_rad: +$('myaw').value },
      note: $('mnote').value.trim(),
      lease_epoch: robotObj && robotObj.lease ? robotObj.lease.epoch : null,
      idempotency_key: 'ui-' + uuid(),
    };
    try {
      const r = await api('/missions', { method: 'POST', body: JSON.stringify(body) });
      if (!r.ok) {
        const j = await r.json().catch(() => ({}));
        $('merr').textContent = (j.error && j.error.message) || `오류 ${r.status}`;
      } else {
        if (mapView) mapView.setPreviewGoal(null, null);
        closeGoalDrawer();
      }
    } catch (e) {
      $('merr').textContent = '임무 생성 요청 실패';
    }
    await tick();
    if (fullMissions) await fetchFullMissions();
  };

  // Mission cancel & admin mission recover
  async function handleMissionTableClick(ev) {
    const cancelId = ev.target.dataset && ev.target.dataset.cancel;
    if (cancelId) {
      $('merr').textContent = '';
      try {
        const r = await api(`/missions/${encodeURIComponent(cancelId)}/cancel`, { method: 'POST', body: '{}' });
        if (!r.ok) {
          const j = await r.json().catch(() => ({}));
          $('merr').textContent = (j.error && j.error.message) || r.status;
        }
      } catch (e) {
        $('merr').textContent = '취소 요청 결과 불명확: 연결 복구 후 임무 상태를 확인하세요';
      }
      await tick();
      if (fullMissions) await fetchFullMissions();
      return;
    }

    const recId = ev.target.dataset && ev.target.dataset.recoverMission;
    if (recId) {
      const note = window.prompt('RECONCILIATION_REQUIRED 복구 메모 (현장/Nav2 active_count=0 검증 내용, 10자 이상):', '');
      if (note === null) return;
      try {
        const r = await api(`/missions/${encodeURIComponent(recId)}/recover`, {
          method: 'POST',
          body: JSON.stringify({ note }),
        });
        if (!r.ok) {
          const j = await r.json().catch(() => ({}));
          const msg = '복구 거부: ' + ((j.error && j.error.message) || r.status);
          $('merr').textContent = msg;
          window.alert(msg);
        }
      } catch (e) {
        $('merr').textContent = '복구 요청 실패';
        window.alert('복구 요청 실패');
      }
      await tick();
      if (fullMissions) await fetchFullMissions();
    }
  }

  $('missions').onclick = handleMissionTableClick;
  $('missionHistoryTable').onclick = handleMissionTableClick;

  // Admin robot-level reconciliation block recovery
  $('recoverRobotBtn').onclick = async () => {
    $('recoverRobotErr').textContent = '';
    const rid = selectedRobotId;
    if (!rid) {
      $('recoverRobotErr').textContent = '대상 로봇을 선택하세요';
      return;
    }
    const note = $('recoverRobotNote').value.trim();
    if (!window.confirm(`[관리자 확인] ${rid}의 RECONCILIATION_REQUIRED 차단을 해제하시겠습니까?\nNav2 active_count=0 및 실차 정지 상태를 직접 확인한 경우에만 진행하세요.`)) {
      return;
    }
    try {
      const r = await api(`/robots/${encodeURIComponent(rid)}/recover`, {
        method: 'POST',
        body: JSON.stringify({ note }),
      });
      if (!r.ok) {
        const j = await r.json().catch(() => ({}));
        $('recoverRobotErr').textContent = (j.error && j.error.message) || `오류 ${r.status}`;
      } else {
        $('recoverRobotNote').value = '';
      }
    } catch (e) {
      $('recoverRobotErr').textContent = '복구 요청 실패';
    }
    await tick();
  };

  setInterval(() => { if (data && performance.now() - lastDashboardAt > 3000) renderAll(); }, 500);

  window.addEventListener('popstate', e => { activeTab = e.state?.tab || 'overview'; showAllPanels = false; applyTabVisibility(); });
  function navigate(name) { if (name !== activeTab) history.pushState({tab:name}, '', '#'+name); activeTab=name; }
  function openPanel(name) {
    navigate(name); showAllPanels = false; applyTabVisibility();
    if (name === 'rmf') fetchFullMissions();
  }
  document.querySelectorAll('[data-open]').forEach(btn => {
    btn.addEventListener('click', () => openPanel(btn.dataset.open));
  });
  let activeSensorTab = 'rgb';
  let lastSensorSeq = {};
  let sensorBlobUrls = {};
  function updateSensorTabVisibility() {
    $('sensorRgb').classList.toggle('hidden', activeSensorTab !== 'rgb');
    if ($('sensorLidar')) $('sensorLidar').classList.toggle('hidden', activeSensorTab !== 'lidar');
    if ($('sensorDepth')) $('sensorDepth').classList.toggle('hidden', activeSensorTab !== 'depth');
    $('sensorUnavailable').classList.add('hidden');
  }
  document.querySelectorAll('[data-sensor]').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('[data-sensor]').forEach(b => b.classList.toggle('active', b === btn));
      activeSensorTab = btn.dataset.sensor || 'rgb';
      updateSensorTabVisibility();
    });
  });
  async function fetchBlobUrl(key, url) {
    try {
      const res = await api(url);
      if (!res.ok) return null;
      const blob = await res.blob();
      const obj = URL.createObjectURL(blob);
      if (sensorBlobUrls[key]) URL.revokeObjectURL(sensorBlobUrls[key]);
      sensorBlobUrls[key] = obj;
      return obj;
    } catch { return null; }
  }
  async function pollLiveSensors() {
    const rid = selectedRobotId || (data && data.robots && data.robots[0] && data.robots[0].robot_id) || '';
    if (!token || !rid) return;
    try {
      const res = await api(`/robots/${encodeURIComponent(rid)}/sensors`);
      if (!res.ok) return;
      const s = await res.json();
      if (!s.available) {
        if ($('mapLayerNote')) $('mapLayerNote').textContent = '센서 스트림 대기 중 (Costmap · LiDAR · 경로 · Depth)';
        return;
      }
      const fresh = s.age_s != null && s.age_s <= 5.0;
      const seqChanged = lastSensorSeq[rid] !== s.seq;
      lastSensorSeq[rid] = s.seq;
      if ($('mapLayerNote')) {
        const lp = (s.lidar_points || []).length;
        const gp = (s.global_plan || []).length;
        const fr = (s.frontiers || []).length;
        $('mapLayerNote').textContent = `LIVE 센서(${s.age_s}s): LiDAR ${lp}pts · 경로 ${gp}pts · Frontier ${fr}pts · Costmap ${s.has_costmap ? 'ON' : 'OFF'}`;
      }
      let costmapImg = undefined;
      if (s.has_costmap && seqChanged) {
        const u = await fetchBlobUrl('costmap', `/robots/${encodeURIComponent(rid)}/costmap.png`);
        if (u) {
          costmapImg = await new Promise((resolve) => {
            const im = new Image(); im.onload = () => resolve(im); im.onerror = () => resolve(null); im.src = u;
          });
        }
      }
      if (mapView) mapView.setSensorData(rid, s, costmapImg);
      if (s.has_lidar_img && seqChanged) {
        const u = await fetchBlobUrl('lidar', `/robots/${encodeURIComponent(rid)}/lidar.png`);
        if (u && $('lidarImg')) {
          $('lidarImg').src = u;
          if ($('lidarOverlay')) $('lidarOverlay').classList.add('hidden');
        }
      }
      if ($('lidarMeta')) {
        const sm = s.summary || {};
        $('lidarMeta').textContent = `LiDAR ${(s.lidar_points || []).length}pts · 최근접 ${sm.scan_min_m != null ? sm.scan_min_m + 'm' : '--'} · 지연 ${s.age_s}s`;
      }
      if (s.has_rgb) {
        const aiLive = $('aiState') && $('aiState').classList.contains('on') && !$('aiState').textContent.includes('Fleet RGB');
        if (!aiLive) {
          if (seqChanged || !$('aiImg')?.getAttribute('src')) {
            const u = await fetchBlobUrl('rgb', `/robots/${encodeURIComponent(rid)}/rgb.jpg`);
            if (u && $('aiImg')) $('aiImg').src = u;
          }
          if ($('aiOverlay')) $('aiOverlay').classList.add('hidden');
          const rm = s.rgb_meta || {};
          if ($('spatialRgbStatus')) $('spatialRgbStatus').textContent = `LIVE RGB ${rm.width || 640}×${rm.height || 480} (${s.age_s}s · Fleet 센서 API)`;
          if ($('aiState')) {
            $('aiState').textContent = fresh ? 'LIVE (Fleet RGB)' : 'STALE (Fleet RGB)';
            $('aiState').className = 'badge ' + (fresh ? 'on' : 'warn');
          }
        }
      }
      if (s.has_depth && seqChanged) {
        const u = await fetchBlobUrl('depth', `/robots/${encodeURIComponent(rid)}/depth.jpg`);
        if (u) {
          if ($('depthImg')) { $('depthImg').src = u; if ($('depthOverlay')) $('depthOverlay').classList.add('hidden'); }
          if ($('spatialDepthImg')) {
            $('spatialDepthImg').src = u;
            if ($('spatialDepthBox')) $('spatialDepthBox').classList.remove('hidden');
            if ($('spatialDepthEmpty')) $('spatialDepthEmpty').classList.add('hidden');
          }
        }
      }
      const dm = s.depth_meta || {};
      const dTxt = `Depth ${dm.width || 640}×${dm.height || 400} (${dm.encoding || '16UC1'}) · 중앙 ${dm.center_m != null ? dm.center_m + 'm' : '--'} · 0.2~4.0m (${s.age_s}s)`;
      if ($('depthMeta')) $('depthMeta').textContent = dTxt;
      if ($('spatialDepthMeta')) $('spatialDepthMeta').textContent = dTxt;
      if ($('spatialDepthBadge')) {
        $('spatialDepthBadge').textContent = fresh && s.has_depth ? 'LIVE' : 'STALE';
        $('spatialDepthBadge').className = 'badge ' + (fresh && s.has_depth ? 'on' : 'warn');
      }
    } catch {}
  }
  setInterval(pollLiveSensors, 1000);
  $('robots').addEventListener('keydown', e => {
    if (e.key === 'Enter' || e.key === ' ') { const item = e.target.closest('[data-robot]'); if (item) { e.preventDefault(); selectRobot(item.dataset.robot); } }
  });
  function renderConsole(r, missions, disconnected) {
    const s = r?.state || {}, sg = s.safety_gate || {}, active = s.execution?.active;
    $('overviewSource').textContent = r?.agent?.simulated === true ? 'SIMULATED' : r ? (r.agent?.mode || 'UNKNOWN') : 'UNKNOWN';
    const pose = s.pose || {};
    const poseValid = r?.online === true && r.state_age_s != null && r.state_age_s <= 2 && pose.status === 'OK' && pose.frame_id === 'map' && s.localization_status === 'CONFIRMED' && s.map_verified === true && s.map_version_id && s.localization_map_version_id === s.map_version_id;
    $('spatialSummary').textContent = `${r?.robot_id || '로봇 미선택'} · 실제 건물/층 UNKNOWN · ${registryState.preview ? '로봇 라이브 보고(미리보기 지도에 표시 안 함)' : '라이브 보고'} · 위치 ${poseValid ? `x ${H.fmt(pose.x)}, y ${H.fmt(pose.y)} m (${pose.frame_id})` : '미확정'} · Localization ${s.localization_status || 'UNKNOWN'} · 계획 경로 UNAVAILABLE · 궤적 현재 세션`;
    const mission = missions.find(m => m.robot_id === r?.robot_id && !m.terminal && m.mission_id === active?.mission_id)
      || missions.find(m => m.robot_id === r?.robot_id && !m.terminal);
    const fresh = r?.online === true && Number.isFinite(r.state_age_s) && r.state_age_s >= 0 && r.state_age_s <= 2 && !disconnected;
    const gate = H.classifySafetyGate(r);
    const blocked = sg.movement_permitted === false || sg.cmd_blocked === true || sg.fault_latched === true || gate.category === 'FAULT' || r?.reconciliation_required || s.map_verified === false;
    const set = (id, text, tone='muted') => { $(id).textContent = text; $(id).className = tone; };
    set('globalRobotState', disconnected ? 'OFFLINE · 서버 단절' : r ? (r.online ? (fresh ? '● ONLINE' : '◷ STALE') : '○ OFFLINE') : 'UNKNOWN', fresh ? 'ok' : 'warn');
    set('robotConnection', disconnected ? 'OFFLINE · 서버 단절' : r ? (r.online ? (fresh ? '● ONLINE' : '◷ STALE') : '○ OFFLINE') : 'UNKNOWN', 'badge ' + (fresh ? 'on' : r ? 'bad' : 'neutral'));
    set('statusMode', (s.operation_mode || 'UNKNOWN') + (fresh ? '' : ' · 과거/미수신'));
    const gateFresh = fresh && Number.isFinite(sg.age_s) && sg.age_s >= 0 && sg.age_s + r.state_age_s <= 2 && sg.rx_fresh !== false && sg.source_fresh !== false;
    const gateText = !gateFresh ? 'UNKNOWN · 신선도 확인 필요' : gate.category === 'NORMAL' ? 'NORMAL 보고 · 증거 미확인' : gate.label;
    const gateTone = gateFresh && gate.category !== 'NORMAL' ? gate.tone : 'muted';
    set('statusGate', gateText, gateTone);
    // No authenticated attestation producer is exposed by this Fleet API. Never turn a
    // collection of telemetry fields into a positive authorization in the browser.
    set('statusMovement', !fresh ? '미확인 · 최신 증거 없음' : blocked ? '⊘ 차단' : '미확인 · 최종 승인 미제공', blocked && fresh ? 'bad' : 'warn');
    set('statusReconcile', r?.reconciliation_required ? '⚠ RECONCILIATION_REQUIRED' : r?.reconciled && fresh ? '동기화됨' : 'UNKNOWN', r?.reconciliation_required ? 'bad' : 'muted');
    set('statusMission', mission?.mission_id || '활성 임무 없음');
    $('statusMission').title = mission?.mission_id || '';
    const metric = (title, val, unit) => `<div class="metric"><small>${H.esc(title)}</small><strong>${H.esc(val)}</strong><span>${H.esc(unit)}</span></div>`;
    const bat = s.battery || {};
    $('robotTelemetry').innerHTML = `<div class="telemetry-grid">${metric('배터리', fresh && Number.isFinite(bat.voltage_v) ? H.fmt(bat.voltage_v, 1) + ' V' : '미수신', '백분율 미제공')}${metric('주행 속도', '미수신', '속도 API 미제공')}</div><div class="sensor-health">${[
      ['CPU / 메모리', 'UNAVAILABLE'], ['온도', 'UNAVAILABLE'],
      ['LiDAR', fresh ? s.lidar?.status || 'UNKNOWN' : 'UNKNOWN'],
      ['Depth', fresh ? s.depth_camera?.status || 'UNKNOWN' : 'UNKNOWN'],
      ['IMU', 'UNAVAILABLE'], ['Odometry', fresh && sg.wheel_odom_stale === true ? 'STALE' : 'UNKNOWN'],
    ].map(([k,v]) => `<div><span>${H.esc(k)}</span><strong class="${H.toneForStatus(v)}">${v === 'OK' ? '✓ ' : '· '}${H.esc(v)}</strong></div>`).join('')}</div>`;
    const standby = fresh && s.standby_status?.mode === 'STANDBY' && s.standby_status.drive_ready === false
      ? s.standby_status : null;
    if (standby) {
      set('statusMode', 'STANDBY');
      set('statusMovement', standby.motor_locked === true ? '⊘ 차단 · 모터 잠금 관측' : '⊘ 차단 · 잠금 미확인', 'bad');
    }
    const streamRows = [
      ['LiDAR', '/scan'], ['IMU', '/imu'], ['Odometry', '/odom'], ['베이스 상태', '/limo_status'],
      ['RGB 원본', '/camera/color/image_raw'], ['Depth 원본', '/camera/depth/image_raw'],
      ['Depth 점군', '/camera/depth/points'], ['Depth 필터 출력', '/depth_obstacles/points'],
      ['로컬 Costmap', '/local_costmap/costmap'], ['글로벌 Costmap', '/global_costmap/costmap'],
      ['SLAM 지도', '/map'], ['Safety Gate', '/stall_detector/status_json'],
    ].map(([label, topic]) => {
      const st = standby?.streams?.[topic];
      const rx = {FRESH:'최신 수신', STALE:'갱신 중단', NO_SAMPLE:'미수신'}[st?.rx] || '미확인';
      return [label, st ? `발행자 ${st.publishers} · ${rx}` : rx];
    });
    const nav = standby?.nav2;
    const costmaps = standby?.depth_costmap_subscribers;
    const rtt = fresh ? s.communication?.previous_state_post_rtt_ms : null;
    streamRows.push(
      ['Supervisor', standby?.ownership === 'SUPERVISOR' ? `스택 실행 보고 · PID ${standby.stack_pid}` : '소유권 미확인'],
      ['Depth 필터', standby?.depth_filter_code ? `${standby.depth_filter_code} · 장애물 점 ${standby.depth_marked_points ?? '미확인'}` : '미확인'],
      ['Depth → Costmap', !costmaps ? '연결 미확인' : ['/local_costmap','/global_costmap'].every(x => costmaps.includes(x)) ? '두 Costmap 구독 확인 · 장애물 반영은 별도 검증' : '구독 누락'],
      ['Nav2', nav && Object.keys(nav).length ? Object.entries(nav).map(([k,v]) => `${k}: ${v ? 'active 관측' : '미확인'}`).join(' · ') : '상태 미확인'],
      ['Agent 연결', !r ? '미확인' : fresh ? '최신 보고 수신' : r.online ? '보고 지연' : '연결 끊김'],
      ['직전 Agent 요청 왕복', Number.isFinite(rtt) ? `${H.fmt(rtt,1)} ms` : '미측정']
    );
    $('robotTelemetry').insertAdjacentHTML('beforeend', '<div class="sensor-health">' +
      streamRows.map(([key, value]) => `<div><span>${H.esc(key)}</span><strong>${H.esc(value)}</strong></div>`).join('') + '</div>');
    const faults = H.collectFaultReasons(r);
    $('safetyHeadline').textContent = !fresh ? 'UNKNOWN · 최신 상태 미수신' : r?.reconciliation_required ? '⚠ RECONCILIATION_REQUIRED · 재조정 필요' : blocked ? '⊘ 차단 · ' + (faults.join(' · ') || gate.label) : '이동 승인 미확인 · Gate 보고값과 별도';
    $('safetyHeadline').className = !fresh ? 'warn' : blocked ? 'bad' : 'muted';
    if (standby) {
      const missing = Array.isArray(standby.missing) ? standby.missing.join(' · ') : '';
      $('safetyHeadline').textContent = standby.stack_ready
        ? '센서·주행 스택 준비 · 모터 잠금 유지'
        : 'STANDBY 준비 실패 · ' + (missing || '구성 확인 필요');
      $('safetyHeadline').className = standby.stack_ready ? 'muted' : 'bad';
    }
    $('gateSummary').innerHTML = `<span class="${gateTone}">${H.esc(gateText)}</span><p>${H.esc(faults.slice(0,2).join(' · ') || '차단 원인 보고 없음')}</p><p class="muted">Attestation: UNAVAILABLE<br>Lease: ${H.esc(fresh && r?.lease ? r.lease.owner + ' · ' + H.fmt(r.lease.remaining_s, 0) + 's' : '미보유 / 미확인')}</p>`;
    const kv = (k,v) => `<dt>${H.esc(k)}</dt><dd>${H.esc(v ?? 'UNAVAILABLE')}</dd>`;
    set('currentMissionState', mission?.status || 'IDLE', 'badge ' + (mission ? (['RECONCILIATION_REQUIRED','FAILED'].includes(mission.status) ? 'bad' : mission.status === 'CANCELING' ? 'warn' : 'progress') : 'neutral'));
    const exec = active?.mission_id === mission?.mission_id ? active : null;
    const cmd = exec?.command_id || mission?.commands?.find(c => c.type === 'navigate_to_pose')?.command_id;
    const elapsed = mission && Number.isFinite(mission.created_at) && Number.isFinite(data.server?.time) ? Math.max(0, Math.floor(data.server.time - mission.created_at)) + ' s (생성 후)' : 'UNAVAILABLE';
    $('currentMission').innerHTML = mission ? `<dl class="mission-kv">${kv('Mission ID', mission.mission_id)}${kv('목표',mission.goal ? `${H.fmt(mission.goal.x)}, ${H.fmt(mission.goal.y)} m · ${H.fmt(mission.goal.yaw_rad)} rad` : null)}${kv('경과',elapsed)}${kv('남은 거리',mission.progress?.distance_remaining_m != null ? H.fmt(mission.progress.distance_remaining_m) + ' m' : null)}</dl>` : '<div class="mission-empty">◇<br>활성 임무가 없습니다</div>';
    const inspected = missions.find(m => m.mission_id === selectedMissionId && m.robot_id === r?.robot_id) || mission;
    const stageStatus = inspected?.status || '';
    $('operationStage').innerHTML = ['QUEUED','DISPATCHED','ACCEPTED','EXECUTING'].map((stage, i) => `<div class="stage ${stageStatus === stage ? 'current' : ''}"><span>${i + 1}</span>${stage}</div>`).join('') + (stageStatus && !['QUEUED','DISPATCHED','ACCEPTED','EXECUTING'].includes(stageStatus) ? `<div class="stage current"><span>•</span>${H.esc(stageStatus)}</div>` : '');
    $('operationMission').innerHTML = inspected ? `<dl class="mission-kv">${kv('Mission ID', inspected.mission_id)}${kv('상태',inspected.status)}${kv('Goal UUID', inspected.mission_id === active?.mission_id ? active.goal_uuid : null)}${kv('Command ID', inspected.commands?.find(c=>c.type==='navigate_to_pose')?.command_id)}${kv('출처',inspected.created_by)}${kv('목표',inspected.goal ? `${H.fmt(inspected.goal.x)}, ${H.fmt(inspected.goal.y)} m` : null)}${kv('상세',inspected.detail || '보고 없음')}</dl>` : '활성 임무 없음';
    const canCtl = me && ['operator','admin'].includes(me.role);
    $('newMission').disabled = !canCtl || disconnected;
    $('operationNewMission').disabled = !canCtl || disconnected;
    document.querySelectorAll('[data-goal-drawer]').forEach(b => { b.disabled = !canCtl || disconnected; });
    $('newMission').title = !canCtl ? 'operator/admin 권한 필요' : disconnected ? '서버 연결 확인 필요' : '임무 입력 및 Lease 관리';
    $('quickCancel').disabled = !canCtl || disconnected || !mission || mission.status === 'CANCELING';
    $('quickCancel').title = !canCtl ? 'operator/admin 권한 필요' : '선택 로봇의 현재 Goal 취소 요청';
    $('quickCancel').dataset.cancel = mission?.mission_id || '';
    $('operationCancelTarget').textContent = mission ? `취소 대상: ${mission.mission_id} · ${mission.status}` : '활성 대상 없음';
    $('quickActionStatus').textContent = $('merr').textContent || (mission?.status === 'CANCELING' ? '취소 요청 중 · 정지 완료가 아닙니다' : '');
    const events = (data.events || []).filter(e => !r || !e.robot_id || e.robot_id === r.robot_id).slice(0,3);
    $('recentEvents').innerHTML = events.length ? events.map(e => {
      const tone = e.level === 'error' ? 'bad' : e.level === 'warning' ? 'warn' : 'muted';
      const date = new Date(e.ts * 1000);
      return `<div class="event-line ${tone}"><span>${e.level === 'error' ? '⊘' : e.level === 'warning' ? '⚠' : '•'}</span><time datetime="${date.toISOString()}" title="${date.toISOString()}">${H.esc(date.toLocaleString('ko-KR',{hour12:false}))}</time><span class="event-text" title="${H.esc(e.message)}">${H.esc(e.kind)} · ${H.esc(e.message)}</span></div>`;
    }).join('') : '<p class="muted small">수신된 이벤트가 없습니다.</p>';
    $('diagnosticRaw').textContent = JSON.stringify({server: data.server, state_age_s:r?.state_age_s, agent:r?.agent, safety_gate:sg, execution:s.execution || null, journal:s.journal || 'UNAVAILABLE — API 미제공', attestation:s.attestation || 'UNAVAILABLE — 인증된 생산자 미연동', state:s}, null, 2);
  }
  $('quickCancel').onclick = handleMissionTableClick;
  function updateGoalReview() {
    const chk = goalPrecheck();
    const cell = chk.cell ? ` · 셀(${chk.cell.u}, ${chk.cell.v}) ${chk.cell.class}` : '';
    $('goalReview').textContent = `${$('mrobot').value || '로봇 미선택'} · map · x ${$('mx').value}, y ${$('my').value} m · yaw ${$('myaw').value} rad${cell}${registryState.preview ? ' · 저장 지도 미리보기: 제출 차단' : ''} · 사전 검사 ${chk.ok ? '통과' : '차단: ' + chk.reasons.join(', ')}`;
    $('goalReview').className = 'goal-review ' + (chk.ok ? '' : 'warn');
    updateMissionFormBlockState();
  }
  function openGoalDrawer() {
    if (!me || !['operator','admin'].includes(me.role)) return;
    openPanel('control');
    drawerReturnFocus = document.activeElement;
    $('goalConfirmed').checked = false;
    $('goalDrawerBackdrop').classList.remove('hidden');
    $('main').inert = true;
    updateGoalReview();
    $('goalDrawerClose').focus();
  }
  function closeGoalDrawer() {
    const wasOpen = !$('goalDrawerBackdrop').classList.contains('hidden');
    $('goalDrawerBackdrop').classList.add('hidden');
    $('main').inert = false;
    if (wasOpen && drawerReturnFocus?.isConnected) drawerReturnFocus.focus();
  }
  document.querySelectorAll('[data-goal-drawer]').forEach(b => b.addEventListener('click', openGoalDrawer));
  $('newMission').addEventListener('click', openGoalDrawer);
  $('goalDrawerClose').onclick = closeGoalDrawer;
  $('goalDrawerBackdrop').addEventListener('click', e => { if (e.target === $('goalDrawerBackdrop')) closeGoalDrawer(); });
  $('goalDrawer').addEventListener('keydown', e => {
    if (e.key === 'Escape') { e.preventDefault(); closeGoalDrawer(); }
    if (e.key !== 'Tab') return;
    const focusables = [...$('goalDrawer').querySelectorAll('button,input,select,summary,[tabindex]')].filter(el => !el.disabled && el.getClientRects().length);
    const first = focusables[0], last = focusables[focusables.length-1];
    if (e.shiftKey && document.activeElement === first) { e.preventDefault(); last?.focus(); }
    else if (!e.shiftKey && document.activeElement === last) { e.preventDefault(); first?.focus(); }
  });
  ['mx','my','myaw','mrobot'].forEach(id => $(id).addEventListener('input', () => { $('goalConfirmed').checked=false; updateGoalReview(); }));
  $('goalConfirmed').onchange = updateGoalReview;
  $('cameraCollapse').onclick = () => {
    const collapsed = $('spatialWorkspace').classList.toggle('cameras-collapsed');
    $('cameraCollapse').textContent = collapsed ? '카메라 펼치기' : '카메라 접기';
    $('cameraCollapse').setAttribute('aria-expanded', String(!collapsed));
    if (mapView) requestAnimationFrame(()=>mapView.resize());
  };
  $('cameraSize').oninput = e => { $('spatialWorkspace').style.setProperty('--camera-share',e.target.value+'%'); if(mapView)requestAnimationFrame(()=>mapView.resize()); };
  function renderWorkspace(robots, missions) {
    $('workspaceRobotCount').textContent=robots.length+' 대';
    const list = robots.map(r => {
      const fresh=r.online && Number.isFinite(r.state_age_s) && r.state_age_s<=2;
      const activeCount=missions.filter(m=>m.robot_id===r.robot_id&&!m.terminal).length;
      return `<button type="button" class="workspace-robot ${r.robot_id===selectedRobotId?'selected':''}" data-workspace-robot="${H.esc(r.robot_id)}"><strong>${H.esc(r.robot_id)}</strong><small>${r.agent?.simulated ? 'SIMULATED' : H.esc(r.agent?.mode || 'UNKNOWN')}</small><span><b class="${fresh?'ok':'warn'}">${fresh?'● ONLINE':r.online?'◷ STALE':'○ OFFLINE'}</b><small>활성 임무 ${activeCount}</small></span></button>`;
    }).join('') || '<p class="muted small">등록된 로봇 없음</p>';
    $('workspaceRobots').innerHTML=list; $('operationRobots').innerHTML=list;
    const selectedMissions=missions.filter(m=>m.robot_id===selectedRobotId);
    $('workspaceTaskCount').textContent=selectedMissions.length+' 건';
    $('workspaceTasks').innerHTML=selectedMissions.length ? selectedMissions.map(m=>`<button type="button" class="workspace-task ${m.mission_id===selectedMissionId?'selected':''}" data-workspace-task="${H.esc(m.mission_id)}"><strong>${H.esc(m.mission_id)}</strong><span class="st-${H.esc(m.status)}">${H.esc(m.status)}</span><small>${H.esc(m.created_by || 'UNKNOWN')} · ${H.esc(m.detail || '')}</small></button>`).join('') : '<div class="workspace-empty">◇<br>조회된 임무가 없습니다.<small>선택 로봇의 실제 임무가 표시됩니다.</small></div>';
    const r=robots.find(x=>x.robot_id===selectedRobotId);
    const st=r?.state || {};
    const fresh=r?.online && r.state_age_s!=null && r.state_age_s<=2;
    $('workspaceRobotFacts').textContent=`${selectedRobotId || '미선택'} · ${fresh ? st.operation_mode || 'UNKNOWN' : 'UNKNOWN'} · 실제 건물/층 UNKNOWN · Localization ${fresh ? st.localization_status || 'UNKNOWN' : 'UNKNOWN'}`;
  }
  ['workspaceRobots','operationRobots'].forEach(id=>$(id).addEventListener('click',e=>{ const b=e.target.closest('[data-workspace-robot]'); if(b)selectRobot(b.dataset.workspaceRobot); }));
  $('workspaceTasks').addEventListener('click',e=>{const b=e.target.closest('[data-workspace-task]');if(!b)return;selectedMissionId=b.dataset.workspaceTask;renderAll();if(mapView)mapView.fitRobot();});


  start();
})();
