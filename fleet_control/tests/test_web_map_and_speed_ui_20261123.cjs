'use strict';
/* Phase 4 deterministic unit & mock-DOM tests for Web Map Visualization and Speed Control UI (2026-11-23).
 * Runs offline in Node.js (zero external npm packages, zero ROS/robot/18190 access). */
const test = require('node:test');
const assert = require('node:assert/strict');
const path = require('node:path');

const STATIC_DIR = path.resolve(__dirname, '../server/fleet_server/static');
const H = require(path.join(STATIC_DIR, 'state_helpers.js'));
globalThis.FleetHelpers = H;
const FleetMapView = require(path.join(STATIC_DIR, 'map_view.js'));
const SpeedControlPanel = require(path.join(STATIC_DIR, 'speed_control_panel.js'));

function makeMockCanvas(width = 640, height = 480) {
  const ops = [];
  const ctx = {
    fillStyle: '',
    strokeStyle: '',
    lineWidth: 1,
    font: '',
    globalAlpha: 1,
    imageSmoothingEnabled: true,
    setTransform: () => {},
    fillRect: (...a) => ops.push(['fillRect', ...a]),
    clearRect: () => {},
    save: () => ops.push(['save']),
    restore: () => ops.push(['restore']),
    translate: (x, y) => ops.push(['translate', x, y]),
    scale: (x, y) => ops.push(['scale', x, y]),
    rotate: (r) => ops.push(['rotate', r]),
    beginPath: () => ops.push(['beginPath']),
    closePath: () => ops.push(['closePath']),
    moveTo: (x, y) => ops.push(['moveTo', x, y]),
    lineTo: (x, y) => ops.push(['lineTo', x, y]),
    arc: (x, y, r) => ops.push(['arc', x, y, r]),
    stroke: () => ops.push(['stroke']),
    fill: () => ops.push(['fill']),
    setLineDash: () => {},
    drawImage: () => ops.push(['drawImage']),
    fillText: (text, x, y) => ops.push(['fillText', String(text), x, y]),
    measureText: (t) => ({ width: String(t).length * 7 }),
  };
  const canvas = {
    width,
    height,
    parentElement: {
      clientWidth: width,
      clientHeight: height,
      getBoundingClientRect: () => ({ width, height, left: 0, top: 0 }),
    },
    getContext: () => ctx,
    addEventListener: () => {},
    getBoundingClientRect: () => ({ width, height, left: 0, top: 0 }),
  };
  return { canvas, ops };
}

function makeRobot(overrides = {}) {
  const base = {
    robot_id: 'limo-01',
    online: true,
    state_age_s: 0.4,
    reconciled: true,
    reconciliation_required: false,
    lease: { owner: 'op1', epoch: 3, remaining_s: 45 },
    agent: { protocol_version: 2, safety_contract_version: 1, boot_id: 'boot-abc-123' },
    map_meta: { width: 200, height: 100, resolution: 0.05, origin_x: -5.0, origin_y: -2.5, origin_yaw: 0.0 },
    state: {
      operation_mode: 'NAVIGATION',
      localization_status: 'CONFIRMED',
      map_verified: true,
      map_version_id: 'map_v1',
      localization_map_version_id: 'map_v1',
      pose: { status: 'OK', frame_id: 'map', x: 1.25, y: -0.75, yaw_rad: Math.PI / 4 },
      velocity: { status: 'OK', linear_x_mps: 0.12, angular_z_rps: -0.15, source: 'odom' },
      safety_gate: { status: 'NORMAL', fault_latched: false, cmd_blocked: false, movement_permitted: true },
      motion_watchdog: { state: 'MOTION_AUTHORIZED' },
    },
  };
  return Object.assign({}, base, overrides, {
    state: Object.assign({}, base.state, overrides.state || {}),
    agent: Object.assign({}, base.agent, overrides.agent || {}),
  });
}

test('1. Map coordinate transforms: origin, resolution, yaw, Y-inversion, and screen zoom/pan/rotation', () => {
  const meta = { width: 200, height: 100, resolution: 0.05, origin_x: -2.0, origin_y: 1.5, origin_yaw: Math.PI / 6 };
  const sample = { x: 1.35, y: 2.80 };

  const g = H.worldToGrid(meta, sample.x, sample.y);
  const wBack = H.gridToWorld(meta, g.u, g.v);
  assert.ok(Math.abs(wBack.x - sample.x) < 1e-9);
  assert.ok(Math.abs(wBack.y - sample.y) < 1e-9);

  // PNG image pixel (py=0 at top, py=height at bottom) Y-inversion check
  const pxTopLeft = H.imagePxToWorld(meta, 0, 0);
  const gridTopLeft = H.gridToWorld(meta, 0, meta.height);
  assert.ok(Math.abs(pxTopLeft.x - gridTopLeft.x) < 1e-9);
  assert.ok(Math.abs(pxTopLeft.y - gridTopLeft.y) < 1e-9);

  const imgPx = H.worldToImagePx(meta, sample.x, sample.y);
  const fromImg = H.imagePxToWorld(meta, imgPx.px, imgPx.py);
  assert.ok(Math.abs(fromImg.x - sample.x) < 1e-9);
  assert.ok(Math.abs(fromImg.y - sample.y) < 1e-9);

  // Screen <-> World with zoom, pan, rotation, and Y-up -> Y-down flip
  const view = { x: 0.5, y: -0.5, zoom: 80, rotation: -Math.PI / 4, panX: 30, panY: -20, width: 640, height: 480 };
  const sp = H.worldToScreen(view, sample);
  const wp = H.screenToWorld(view, sp);
  assert.ok(Math.abs(wp.x - sample.x) < 1e-9);
  assert.ok(Math.abs(wp.y - sample.y) < 1e-9);

  // With rotation=0, larger world y MUST map to smaller screen y (Y-inversion)
  const v0 = { x: 0, y: 0, zoom: 50, rotation: 0, panX: 0, panY: 0, width: 640, height: 480 };
  assert.ok(H.worldToScreen(v0, { x: 0, y: 1 }).y < H.worldToScreen(v0, { x: 0, y: 0 }).y);
});

test('2. Pose classification & rendering: OK vs STALE vs UNAVAILABLE never draws stale pose as normal', () => {
  const { canvas, ops } = makeMockCanvas();
  const mv = new FleetMapView(canvas);
  mv.setMapImage('limo-01', 'v1', { width: 200, height: 100 });

  // Normal OK pose
  const rOk = makeRobot();
  ops.length = 0;
  mv.update(rOk, []);
  const vsOk = mv.getVisualState();
  assert.equal(vsOk.poseState.status, 'OK');
  assert.equal(vsOk.poseState.source, 'LOCALIZATION');
  assert.ok(ops.some((op) => op[0] === 'rotate' && Math.abs(op[1] - Math.PI / 4) < 1e-9));

  // STALE pose (state_age_s > 2) -> must NOT draw pose arrow, must show STALE warning
  const rStale = makeRobot({ state_age_s: 4.5 });
  ops.length = 0;
  mv.update(rStale, []);
  const vsStale = mv.getVisualState();
  assert.equal(vsStale.poseState.status, 'STALE');
  assert.equal(vsStale.poseState.pose, null);
  assert.ok(ops.some((op) => op[0] === 'fillText' && op[1].includes('로봇 위치 STALE')));
  assert.ok(!ops.some((op) => op[0] === 'rotate' && Math.abs(op[1] - Math.PI / 4) < 1e-9));

  // UNAVAILABLE pose (wrong frame_id 'odom' or NO_DATA) -> must show UNAVAILABLE warning
  const rBadFrame = makeRobot({ state: { pose: { status: 'OK', frame_id: 'odom', x: 1, y: 1, yaw_rad: 0 } } });
  ops.length = 0;
  mv.update(rBadFrame, []);
  const vsUnavail = mv.getVisualState();
  assert.equal(vsUnavail.poseState.status, 'UNAVAILABLE');
  assert.ok(ops.some((op) => op[0] === 'fillText' && op[1].includes('로봇 위치 UNAVAILABLE')));
});

test('3. Nav2 planned path, waypoints, Fit to Route, and Goal cancellation clearing', () => {
  const { canvas, ops } = makeMockCanvas();
  const mv = new FleetMapView(canvas);
  const r = makeRobot();
  const activeMission = {
    mission_id: 'm-1',
    robot_id: 'limo-01',
    terminal: false,
    goal: { frame_id: 'map', x: 3.0, y: 1.0, yaw_rad: 0.5 },
    waypoints: [
      { frame_id: 'map', x: 1.5, y: 0.0, label: 'W1' },
      { frame_id: 'map', x: 2.2, y: 0.6, label: 'W2' },
    ],
  };
  mv.setSensorData('limo-01', {
    available: true,
    age_s: 0.3,
    global_plan: [[1.25, -0.75], [1.5, 0.0], [2.2, 0.6], [3.0, 1.0]],
  });
  ops.length = 0;
  mv.update(r, [activeMission]);
  const vs = mv.getVisualState();
  assert.equal(vs.plan.status, 'OK');
  assert.equal(vs.plan.points.length, 4);
  assert.equal(vs.waypoints.length, 2);
  assert.ok(ops.some((op) => op[0] === 'fillText' && op[1] === 'W1'));
  assert.ok(ops.some((op) => op[0] === 'fillText' && op[1] === 'W2'));
  assert.equal(mv.fitRoute(), true);

  // Cancel goal + clear plan -> plan and waypoints disappear cleanly
  mv.setSensorData('limo-01', { available: true, age_s: 0.2, global_plan: [] });
  mv.update(r, [{ ...activeMission, terminal: true }]);
  const vsCleared = mv.getVisualState();
  assert.equal(vsCleared.plan.status, 'UNAVAILABLE');
  assert.equal(vsCleared.waypoints.length, 0);
  assert.equal(mv.fitRoute(), false);
});

test('4. Trail accumulation, memory bound, clearTrail, and STALE immunity', () => {
  const { canvas } = makeMockCanvas();
  const mv = new FleetMapView(canvas, { maxTrailPoints: 15 });
  for (let i = 0; i < 30; i++) {
    mv.update(makeRobot({ state: { pose: { status: 'OK', frame_id: 'map', x: i * 0.1, y: 0.0, yaw_rad: 0 } } }), []);
  }
  assert.equal(mv.getTrailStats('limo-01').count, 15);
  assert.equal(mv.getTrailStats('limo-01').max, 15);

  // STALE pose must NOT append to trail
  const lastBefore = mv.trails['limo-01'][mv.trails['limo-01'].length - 1];
  mv.update(makeRobot({ state_age_s: 9.0, state: { pose: { status: 'STALE', frame_id: 'map', x: 99.0, y: 99.0, yaw_rad: 0 } } }), []);
  assert.deepEqual(mv.trails['limo-01'][mv.trails['limo-01'].length - 1], lastBefore);

  // Clear trail resets count to 0
  mv.clearTrail('limo-01');
  assert.equal(mv.getTrailStats('limo-01').count, 0);
});

test('5. Phase C SpeedControlPanel: dynamic Agent ceilings, ARBITER/WATCHDOG gating, statuses, and error codes', () => {
  const slArbiter = {
    enforced_by: 'ARBITER',
    ceiling: {
      manual: { linear_mps: 0.10, angular_rps: 0.30 },
      auto: { linear_mps: 0.12, angular_rps: 0.40, reverse_mps: 0.10 },
    },
  };
  const cManual = SpeedControlPanel.extractScopeCeiling(slArbiter, 'manual');
  const cAuto = SpeedControlPanel.extractScopeCeiling(slArbiter, 'auto');
  assert.deepEqual(cManual, { linear_mps: 0.10, angular_rps: 0.30, reverse_mps: null });
  assert.deepEqual(cAuto, { linear_mps: 0.12, angular_rps: 0.40, reverse_mps: 0.10 });
  assert.equal(SpeedControlPanel.extractScopeCeiling({}, 'auto'), null);

  // Dynamic ceiling & minimum validation (no hardcoded 0.15/0.50 ceiling!)
  assert.equal(SpeedControlPanel.validateSpeedInput(0.11, 0.35, cAuto).ok, true);
  assert.equal(SpeedControlPanel.validateSpeedInput(0.13, 0.35, cAuto).code, 'speed_exceeds_ceiling');
  assert.equal(SpeedControlPanel.validateSpeedInput(0.005, 0.10, cAuto).code, 'speed_below_minimum');
  assert.equal(SpeedControlPanel.validateSpeedInput(0.05, 0.01, cAuto).code, 'speed_below_minimum');
  assert.equal(SpeedControlPanel.validateSpeedInput(0.05, 0.10, null).code, 'agent_ceiling_unknown');

  // Enforcement point gating: ARBITER ok, null blocked, WATCHDOG rejects bare boolean flags!
  assert.equal(SpeedControlPanel.evaluateEnforcement({ enforced_by: 'ARBITER' }).supported, true);
  assert.equal(SpeedControlPanel.evaluateEnforcement({ enforced_by: null }).supported, false);
  assert.equal(SpeedControlPanel.evaluateEnforcement({ enforced_by: 'WATCHDOG' }).supported, false);
  assert.equal(SpeedControlPanel.evaluateEnforcement({ enforced_by: 'WATCHDOG', watchdog_clamp_verified: true }).supported, false);
  assert.equal(SpeedControlPanel.evaluateEnforcement({ enforced_by: 'WATCHDOG', enforcement_verified: true, enforcement_ready: true }).supported, false);

  const validCtx = {
    robotId: 'limo-01',
    serverBootId: 'srv-1',
    online: true,
    stateAgeS: 0.3,
    canControl: true,
    reconciliationRequired: false,
    protocolVersion: 2,
    safetyContractVersion: 1,
    agentBootId: 'boot-1',
    leaseOwned: true,
    leaseEpoch: 5,
    operationMode: 'NAVIGATION',
    watchdogAuthorized: true,
  };
  const validWdEvidence = {
    enforced_by: 'WATCHDOG',
    watchdog_evidence: {
      server_verified: true,
      agent_verified: true,
      watchdog_boot_id: 'wd-boot-99',
      agent_boot_id: 'boot-1',
      output_gid: '0102030405060708',
      evidence_age_s: 0.2,
    },
  };
  assert.equal(SpeedControlPanel.evaluateEnforcement(validWdEvidence, validCtx).supported, true);
  // Missing server_verified, stale evidence, boot mismatch, or unauthorized watchdog MUST block
  assert.equal(SpeedControlPanel.evaluateEnforcement({ ...validWdEvidence, watchdog_evidence: { ...validWdEvidence.watchdog_evidence, server_verified: false } }, validCtx).supported, false);
  assert.equal(SpeedControlPanel.evaluateEnforcement({ ...validWdEvidence, watchdog_evidence: { ...validWdEvidence.watchdog_evidence, agent_boot_id: 'other-boot' } }, validCtx).supported, false);
  assert.equal(SpeedControlPanel.evaluateEnforcement({ ...validWdEvidence, watchdog_evidence: { ...validWdEvidence.watchdog_evidence, evidence_age_s: 5.0 } }, validCtx).supported, false);
  assert.equal(SpeedControlPanel.evaluateEnforcement(validWdEvidence, { ...validCtx, watchdogAuthorized: false }).supported, false);

  assert.equal(SpeedControlPanel.evaluateSpeedGate(validCtx, 'auto', slArbiter).allowed, true);
  assert.equal(SpeedControlPanel.evaluateSpeedGate(validCtx, 'auto', { enforced_by: null, ceiling: slArbiter.ceiling }).code, 'agent_cannot_enforce');
  assert.equal(SpeedControlPanel.evaluateSpeedGate(validCtx, 'auto', { enforced_by: 'ARBITER' }).code, 'agent_ceiling_unknown');
  assert.equal(SpeedControlPanel.evaluateSpeedGate({ ...validCtx, leaseOwned: false }, 'auto', slArbiter).code, 'not_lease_owner');
  assert.equal(SpeedControlPanel.evaluateSpeedGate({ ...validCtx, safetyContractVersion: 0 }, 'auto', slArbiter).code, 'agent_protocol');
  assert.equal(SpeedControlPanel.evaluateSpeedGate({ ...validCtx, agentBootId: '' }, 'auto', slArbiter).code, 'agent_protocol');

  // Status classification: PENDING / APPLIED / REJECTED / EXPIRED / STALE
  assert.equal(SpeedControlPanel.classifySpeedStatus({ status: 'PENDING', approved: false, generation: 2 }).status, 'PENDING');
  assert.equal(SpeedControlPanel.classifySpeedStatus({ status: 'PENDING', approved: false }).approved, false);
  assert.equal(SpeedControlPanel.classifySpeedStatus({ status: 'APPLIED', approved: true, stale: false, generation: 2 }).approved, true);
  // Stale/disconnect/restart MUST NOT claim '낮은 값이 계속 적용 중' without verified state
  const staleCls = SpeedControlPanel.classifySpeedStatus({ status: 'APPLIED', approved: true, stale: true, stale_reason: 'AGENT_RESTARTED', agent_report_age_s: null });
  assert.equal(staleCls.status, 'STALE');
  assert.equal(staleCls.approved, false);
  assert.ok(staleCls.text.includes('적용 상태 미확인'));
  assert.ok(!staleCls.text.includes('낮은 값이 계속 적용 중'));
  const offlineCls = SpeedControlPanel.classifySpeedStatus({ status: 'APPLIED', approved: true, stale: false }, { online: false, stateAgeS: 10 });
  assert.equal(offlineCls.status, 'STALE');
  assert.equal(offlineCls.approved, false);
  assert.ok(offlineCls.text.includes('적용 상태 미확인'));
  assert.ok(!offlineCls.text.includes('낮은 값이 계속 적용 중'));
  assert.equal(SpeedControlPanel.classifySpeedStatus({ status: 'REJECTED', reason: 'EXCEEDS_CEILING' }).status, 'REJECTED');
  assert.equal(SpeedControlPanel.classifySpeedStatus({ status: 'EXPIRED', reason: 'AGENT_NO_CONFIRMATION' }).status, 'EXPIRED');

  // Safety limiting classification
  assert.equal(SpeedControlPanel.classifySafetyLimiting({ online: true, stateAgeS: 0.2, motorLocked: true }).status, 'MOTOR_LOCKED');
  assert.equal(SpeedControlPanel.classifySafetyLimiting({ online: true, stateAgeS: 0.2, gateBlocked: true }).status, 'LIMITED_BY_SAFETY_GATE');
  assert.equal(SpeedControlPanel.classifySafetyLimiting({ online: true, stateAgeS: 0.2, watchdogAuthorized: false }).status, 'LIMITED_BY_WATCHDOG');
  assert.equal(SpeedControlPanel.classifySafetyLimiting({ online: true, stateAgeS: 5.0 }).status, 'STALE_TELEMETRY');
  assert.equal(SpeedControlPanel.classifySafetyLimiting({ online: true, stateAgeS: 0.2, watchdogAuthorized: true }).status, 'NORMAL');
});

test('6. verifyAppliedMatch: approved=true MUST NOT show APPLIED if robot_id, boot, scope, generation, or applied values mismatch', () => {
  const req = {
    robotId: 'limo-01',
    agentBootId: 'boot-1',
    scope: 'auto',
    generation: 7,
    requestId: 'req-7',
    requestedLinear: 0.09,
    requestedAngular: 0.25,
  };
  const ctx = { robotId: 'limo-01', agentBootId: 'boot-1' };
  const exact = {
    robot_id: 'limo-01',
    agent_boot_id: 'boot-1',
    status: 'APPLIED',
    approved: true,
    stale: false,
    scope: 'auto',
    generation: 7,
    request_id: 'req-7',
    applied: { linear_mps: 0.09, angular_rps: 0.25, generation: 7 },
  };
  assert.equal(SpeedControlPanel.verifyAppliedMatch(req, exact, ctx).ok, true);
  // Mismatched robot_id
  assert.equal(SpeedControlPanel.verifyAppliedMatch(req, { ...exact, robot_id: 'limo-02' }, ctx).ok, false);
  // Mismatched agent_boot_id (response or current context)
  assert.equal(SpeedControlPanel.verifyAppliedMatch(req, { ...exact, agent_boot_id: 'boot-old' }, ctx).ok, false);
  assert.equal(SpeedControlPanel.verifyAppliedMatch(req, exact, { ...ctx, agentBootId: 'boot-restarted' }).ok, false);
  // Mismatched scope
  assert.equal(SpeedControlPanel.verifyAppliedMatch(req, { ...exact, scope: 'manual' }, ctx).ok, false);
  // Mismatched generation (top-level or applied.generation)
  assert.equal(SpeedControlPanel.verifyAppliedMatch(req, { ...exact, generation: 6 }, ctx).ok, false);
  assert.equal(SpeedControlPanel.verifyAppliedMatch(req, { ...exact, applied: { ...exact.applied, generation: 6 } }, ctx).ok, false);
  // Mismatched applied speed values
  assert.equal(SpeedControlPanel.verifyAppliedMatch(req, { ...exact, applied: { ...exact.applied, linear_mps: 0.05 } }, ctx).ok, false);
  assert.equal(SpeedControlPanel.verifyAppliedMatch(req, { ...exact, applied: { ...exact.applied, angular_rps: 0.15 } }, ctx).ok, false);
  // Missing applied object
  assert.equal(SpeedControlPanel.verifyAppliedMatch(req, { ...exact, applied: null }, ctx).ok, false);
});
