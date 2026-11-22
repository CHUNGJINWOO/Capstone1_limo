'use strict';
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.FleetHelpers = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const GATE_FAULT_STATES = new Set([
    'STALL_LATCHED',
    'FAILSAFE_SENSOR_TIMEOUT',
    'FAILSAFE_ICP_DEGRADED',
    'FAULT',
    'ERROR',
  ]);

  function esc(s) {
    return String(s ?? '').replace(/[&<>"']/g, function (c) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
    });
  }

  function fmt(v, d) {
    if (d === undefined) d = 2;
    if (v === null || v === undefined || v === '' || Number.isNaN(Number(v))) return '-';
    return Number(v).toFixed(d);
  }

  function fmtAge(sec) {
    if (sec === null || sec === undefined || Number.isNaN(Number(sec))) return 'UNKNOWN';
    return Number(sec).toFixed(1) + ' s 전';
  }

  /** The server keeps the LAST reported state with the age_s it had at report time. When the robot is offline (or the state is old)
      those values are history: show them as such and add the time since the report to every displayed sensor age. */
  const STALE_AFTER_S = 3;
  function staleInfo(r) {
    const age = r && typeof r.state_age_s === 'number' && Number.isFinite(r.state_age_s) ? r.state_age_s : null;
    const stale = !r || !r.online || age === null || age > STALE_AFTER_S;
    let label = '';
    if (stale) {
      if (age === null) label = '과거 표본';
      else if (age >= 120) label = `과거 표본 · ${Math.round(age / 60)}분 전`;
      else label = `과거 표본 · ${Math.round(age)}초 전`;
    }
    return { stale, ageS: age, label };
  }
  function shownAge(reportedAge, r) {
    if (reportedAge === null || reportedAge === undefined || Number.isNaN(Number(reportedAge))) return reportedAge;
    const info = staleInfo(r);
    return info.stale && info.ageS !== null ? Number(reportedAge) + info.ageS : Number(reportedAge);
  }

  function toneForStatus(s) {
    if (!s) return 'muted';
    if (['OK', 'NORMAL', 'READY', 'ONLINE', 'LOCKED', 'CONFIRMED', 'UPDATING', 'STATIC', 'SUCCEEDED', 'ACTIVE_GOAL'].includes(s)) return 'ok';
    if (['STALE', 'VOLTAGE_ONLY', 'SIM_GATED', 'CANCELED', 'CANCELING', 'EXECUTING', 'ACCEPTED', 'DISPATCHED', 'QUEUED'].includes(s)) return 'warn';
    if (['NO_DATA', 'UNKNOWN', 'UNAVAILABLE', 'NO_MAP'].includes(s)) return 'muted';
    return 'bad';
  }

  function classifySafetyGate(robot) {
    if (!robot) {
      return { category: 'UNKNOWN', rawStatus: 'UNKNOWN', label: 'UNKNOWN', tone: 'muted', staleOrOffline: true };
    }
    const state = robot.state || {};
    const sg = (state && typeof state.safety_gate === 'object' && state.safety_gate) || {};
    const raw = sg.status || 'UNKNOWN';
    const offline = !robot.online;
    if (raw === 'NO_DATA' || raw === 'UNKNOWN') {
      return {
        category: 'UNKNOWN',
        rawStatus: raw,
        label: offline ? 'OFFLINE (마지막: ' + raw + ')' : raw,
        tone: offline ? 'bad' : 'muted',
        staleOrOffline: true,
      };
    }
    if (GATE_FAULT_STATES.has(raw) || sg.fault_latched === true || sg.cmd_blocked === true) {
      return {
        category: 'FAULT',
        rawStatus: raw,
        label: offline ? 'OFFLINE (마지막 고장: ' + raw + ')' : '고장 · ' + raw,
        tone: 'bad',
        staleOrOffline: offline,
      };
    }
    if (raw === 'STALE' || sg.rx_fresh === false || sg.source_fresh === false) {
      return {
        category: 'STALE',
        rawStatus: raw,
        label: offline ? 'OFFLINE (마지막: ' + raw + ')' : '지연/만료 · ' + raw,
        tone: offline ? 'bad' : 'warn',
        staleOrOffline: true,
      };
    }
    if (raw === 'NORMAL') {
      return {
        category: 'NORMAL',
        rawStatus: raw,
        label: offline ? 'OFFLINE (마지막: NORMAL)' : '정상 · NORMAL',
        tone: offline ? 'bad' : 'ok',
        staleOrOffline: offline,
      };
    }
    return {
      category: 'UNKNOWN',
      rawStatus: raw,
      label: offline ? 'OFFLINE (' + raw + ')' : String(raw),
      tone: offline ? 'bad' : 'warn',
      staleOrOffline: offline,
    };
  }

  function classifyMotorPath(robot) {
    const mp = ((robot && robot.state && robot.state.safety_gate) || {}).motor_path || {};
    const status = mp.status || 'UNKNOWN';
    if (status === 'LOCKED') return { status: 'LOCKED', label: 'LOCKED (모터 잠금)', tone: 'ok', publishers: mp.cmd_vel_publishers || [] };
    if (status === 'SIM_GATED') return { status: 'SIM_GATED', label: 'SIM_GATED (시뮬 안전 게이트)', tone: 'warn', publishers: mp.cmd_vel_publishers || [] };
    if (status === 'ARMED') return { status: 'ARMED', label: 'ARMED (/cmd_vel 발행 감지)', tone: 'bad', publishers: mp.cmd_vel_publishers || [] };
    return { status: 'UNKNOWN', label: 'UNKNOWN', tone: 'muted', publishers: mp.cmd_vel_publishers || [] };
  }

  function classifyAllowMissions(robot) {
    const agent = (robot && robot.agent) || {};
    if (agent.missions_enabled === false) {
      return { value: false, label: 'false (임무 차단 · 안전 기본)', tone: 'ok' };
    }
    if (agent.missions_enabled === true) {
      return { value: true, label: 'true (임무 허용)', tone: 'warn' };
    }
    return { value: null, label: 'UNKNOWN', tone: 'muted' };
  }

  function collectFaultReasons(robot) {
    if (!robot || !robot.state) return [];
    const s = robot.state;
    const sg = (s.safety_gate && typeof s.safety_gate === 'object') ? s.safety_gate : {};
    const out = [];
    const seen = new Set();
    function pushItem(prefix, item) {
      if (item === null || item === undefined || item === '') return;
      const text = prefix ? prefix + ': ' + String(item) : String(item);
      if (!seen.has(text)) {
        seen.add(text);
        out.push(text);
      }
    }
    (Array.isArray(s.errors) ? s.errors : []).forEach(function (e) { pushItem('error', e); });
    (Array.isArray(sg.errors) ? sg.errors : []).forEach(function (e) { pushItem('gate_error', e); });
    (Array.isArray(sg.fault_reasons) ? sg.fault_reasons : []).forEach(function (e) { pushItem('fault', e); });
    (Array.isArray(sg.last_fault_reasons) ? sg.last_fault_reasons : []).forEach(function (e) { pushItem('last_fault', e); });
    (Array.isArray(sg.movement_block_reasons) ? sg.movement_block_reasons : []).forEach(function (e) { pushItem('block', e); });
    if (Array.isArray(sg.missing_fields) && sg.missing_fields.length) {
      pushItem('missing_gate_fields', sg.missing_fields.join(','));
    }
    if (sg.wheel_odom_stale) pushItem('stale', 'wheel_odom_stale');
    if (sg.est_odom_stale) pushItem('stale', 'est_odom_stale');
    if (sg.depth_stale) pushItem('stale', 'depth_stale');
    if (sg.epoch_regressed) pushItem('safety', 'gate fault_epoch 역행 감지');
    if (sg.source_clock_regressed) pushItem('safety', 'gate source_clock 역행 감지');
    if (out.length === 0 && (GATE_FAULT_STATES.has(sg.status) || sg.fault_latched === true || sg.cmd_blocked === true)) {
      pushItem('gate_status', sg.status || (sg.fault_latched ? 'FAULT_LATCHED' : 'CMD_BLOCKED'));
    }
    return out;
  }

  function evaluateSafetyReadiness(robot, currentUserName) {
    if (!robot) return { readyForMission: false, blockers: ['로봇 미선택'], checks: [] };
    const s = robot.state || {};
    const a = robot.agent || {};
    const sg = (s.safety_gate && typeof s.safety_gate === 'object') ? s.safety_gate : {};
    const ex = (s.execution && typeof s.execution === 'object') ? s.execution : {};
    const ng = (ex.nav2_goals && typeof ex.nav2_goals === 'object') ? ex.nav2_goals : {};
    const lease = robot.lease || null;

    const checks = [
      { key: 'online', label: 'Agent 연결 (ONLINE)', ok: robot.online === true, detail: robot.online ? 'ONLINE' : 'OFFLINE' },
      { key: 'reconciled', label: '세션 동기화 & 복구 차단 없음', ok: robot.reconciled === true && !robot.reconciliation_required, detail: robot.reconciliation_required ? 'RECONCILIATION_REQUIRED' : (robot.reconciled ? 'reconciled' : '미동기화') },
      { key: 'contract', label: 'Safety Contract v1 (Protocol v2)', ok: a.protocol_version === 2 && a.safety_contract_version === 1 && Boolean(a.boot_id), detail: `proto=${a.protocol_version ?? '?'} / contract=${a.safety_contract_version ?? '?'}` },
      { key: 'gate', label: 'Safety Gate 정상 & 이동 허용', ok: sg.status === 'NORMAL' && sg.fault_latched === false && sg.cmd_blocked === false && sg.movement_permitted === true, detail: `status=${sg.status || 'NO_DATA'}, permitted=${sg.movement_permitted ?? 'UNKNOWN'}` },
      { key: 'opmode', label: '운용 모드 (NAVIGATION)', ok: s.operation_mode === 'NAVIGATION' && Number.isInteger(s.operation_revision), detail: `${s.operation_mode || 'UNKNOWN'} (rev ${s.operation_revision ?? '-'})` },
      { key: 'map', label: '지도 검증 (map_verified)', ok: s.map_verified === true && Boolean(s.map_version_id) && Number.isInteger(s.map_epoch), detail: `${s.map_version_id || 'NONE'} (epoch ${s.map_epoch ?? '-'}, verified=${s.map_verified ?? 'UNKNOWN'})` },
      { key: 'loc', label: '위치추정 확정 (CONFIRMED)', ok: s.localization_status === 'CONFIRMED' && Boolean(s.map_version_id) && s.localization_map_version_id === s.map_version_id && Number.isInteger(s.localization_epoch), detail: `${s.localization_status || 'UNKNOWN'} (epoch ${s.localization_epoch ?? '-'})` },
      { key: 'nav2', label: 'Nav2 유휴 & 상태 배열 신선', ok: ng.known === true && ng.received_status_array === true && ng.active_count === 0 && ex.active === null, detail: `known=${ng.known ?? 'UNKNOWN'}, active=${ng.active_count ?? 'UNKNOWN'}` },
      { key: 'lease', label: '현재 사용자 제어권 Lease 보유', ok: Boolean(lease && currentUserName && lease.owner === currentUserName && Number.isInteger(lease.epoch)), detail: lease ? `${lease.owner} (epoch ${lease.epoch}, ${fmt(lease.remaining_s, 1)}s)` : '미획득' },
    ];

    const blockers = checks.filter(function (c) { return !c.ok; }).map(function (c) { return c.label + ': ' + c.detail; });
    return {
      readyForMission: blockers.length === 0,
      blockers: blockers,
      checks: checks,
    };
  }

  function classifyMissionSource(m) {
    if (!m) return 'UNKNOWN';
    if (m.created_by === 'rmf-adapter' || String(m.idempotency_key || '').startsWith('rmf:')) return 'RMF';
    return 'FLEET_UI';
  }

  function classifyFleetOrigin(robot, missions) {
    if (!robot) return { label: 'UNKNOWN', kind: 'unknown' };
    if (robot.lease && robot.lease.owner === 'rmf-adapter') {
      return { label: 'Open-RMF 제어 중 (rmf-adapter)', kind: 'rmf' };
    }
    const recentRmf = (missions || []).find(function (m) {
      return m.robot_id === robot.robot_id && !m.terminal && classifyMissionSource(m) === 'RMF';
    });
    if (recentRmf) return { label: 'Open-RMF 임무 실행 중', kind: 'rmf' };
    if (robot.lease && robot.lease.owner) return { label: '웹/직접 제어 (' + robot.lease.owner + ')', kind: 'direct' };
    return { label: 'Fleet 대기 (제어권 미점유)', kind: 'idle' };
  }

  function filterMissions(missions, filters) {
    const f = filters || {};
    return (missions || []).filter(function (m) {
      if (f.robotId && f.robotId !== 'ALL' && m.robot_id !== f.robotId) return false;
      if (f.source && f.source !== 'ALL' && classifyMissionSource(m) !== f.source) return false;
      if (f.statusGroup && f.statusGroup !== 'ALL') {
        if (f.statusGroup === 'ACTIVE' && m.terminal) return false;
        if (f.statusGroup === 'FAILED_GROUP' && !['FAILED', 'REJECTED', 'EXPIRED', 'RECONCILIATION_REQUIRED'].includes(m.status)) return false;
        if (f.statusGroup === 'SUCCEEDED' && m.status !== 'SUCCEEDED') return false;
        if (f.statusGroup === 'CANCELED' && !['CANCELED', 'CANCELING'].includes(m.status)) return false;
        if (f.statusGroup === 'RECONCILIATION_REQUIRED' && m.status !== 'RECONCILIATION_REQUIRED') return false;
      }
      return true;
    });
  }

  // Map coordinate helpers (supports non-zero origin_yaw and view pan/zoom/rotation)
  function gridOrigin(meta) {
    return {
      x: Number(meta.origin_x || 0),
      y: Number(meta.origin_y || 0),
      yaw: Number(meta.origin_yaw || 0),
    };
  }

  function gridToWorld(meta, u, v) {
    const o = gridOrigin(meta);
    const c = Math.cos(o.yaw);
    const s = Math.sin(o.yaw);
    const res = Number(meta.resolution || 0.05);
    return {
      x: o.x + res * (c * u - s * v),
      y: o.y + res * (s * u + c * v),
    };
  }

  function worldToScreen(view, p) {
    const c = Math.cos(view.rotation || 0);
    const s = Math.sin(view.rotation || 0);
    const x = p.x - view.x;
    const y = p.y - view.y;
    return {
      x: view.width / 2 + view.panX + view.zoom * (c * x - s * y),
      y: view.height / 2 + view.panY - view.zoom * (s * x + c * y),
    };
  }

  function screenToWorld(view, p) {
    const c = Math.cos(view.rotation || 0);
    const s = Math.sin(view.rotation || 0);
    const x = (p.x - view.width / 2 - view.panX) / view.zoom;
    const y = -(p.y - view.height / 2 - view.panY) / view.zoom;
    return {
      x: view.x + c * x + s * y,
      y: view.y - s * x + c * y,
    };
  }

  return {
    GATE_FAULT_STATES,
    esc,
    fmt,
    fmtAge,
    toneForStatus,
    classifySafetyGate,
    classifyMotorPath,
    staleInfo,
    shownAge,
    classifyAllowMissions,
    collectFaultReasons,
    evaluateSafetyReadiness,
    classifyMissionSource,
    classifyFleetOrigin,
    filterMissions,
    gridOrigin,
    gridToWorld,
    worldToScreen,
    screenToWorld,
  };
});
