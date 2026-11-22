import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const H = require('../server/fleet_server/static/state_helpers.js');

// 1. Safety Gate classification
assert.equal(H.classifySafetyGate(null).category, 'UNKNOWN');
assert.equal(H.classifySafetyGate({ online: false, state: { safety_gate: { status: 'NORMAL' } } }).staleOrOffline, true);
assert.equal(H.classifySafetyGate({ online: true, state: { safety_gate: { status: 'NORMAL', fault_latched: false, cmd_blocked: false } } }).category, 'NORMAL');
assert.equal(H.classifySafetyGate({ online: true, state: { safety_gate: { status: 'STALL_LATCHED', fault_latched: true } } }).category, 'FAULT');
assert.equal(H.classifySafetyGate({ online: true, state: { safety_gate: { status: 'FAILSAFE_SENSOR_TIMEOUT' } } }).category, 'FAULT');
assert.equal(H.classifySafetyGate({ online: true, state: { safety_gate: { status: 'NORMAL', rx_fresh: false } } }).category, 'STALE');
assert.equal(H.classifySafetyGate({ online: true, state: {} }).category, 'UNKNOWN');

// 2. Motor path & allow_missions
assert.equal(H.classifyMotorPath({ state: { safety_gate: { motor_path: { status: 'LOCKED' } } } }).tone, 'ok');
assert.equal(H.classifyMotorPath({ state: { safety_gate: { motor_path: { status: 'ARMED', cmd_vel_publishers: ['bad_node'] } } } }).tone, 'bad');
assert.equal(H.classifyMotorPath({}).status, 'UNKNOWN');
assert.equal(H.classifyAllowMissions({ agent: { missions_enabled: false } }).value, false);
assert.equal(H.classifyAllowMissions({ agent: { missions_enabled: true } }).value, true);
assert.equal(H.classifyAllowMissions({}).label, 'UNKNOWN');

// 3. Safety readiness (Safety Context v1 alignment)
const readyRobot = {
  robot_id: 'pcu',
  online: true,
  reconciled: true,
  reconciliation_required: false,
  state_age_s: 0.2,
  agent: { protocol_version: 2, safety_contract_version: 1, boot_id: 'b1' },
  lease: { owner: 'operator', epoch: 4, remaining_s: 19.0 },
  state: {
    operation_mode: 'NAVIGATION',
    operation_revision: 1,
    map_version_id: 'map-v1',
    map_epoch: 1,
    map_verified: true,
    localization_status: 'CONFIRMED',
    localization_map_version_id: 'map-v1',
    localization_epoch: 1,
    safety_gate: { status: 'NORMAL', fault_latched: false, cmd_blocked: false, movement_permitted: true },
    execution: { active: null, nav2_goals: { known: true, received_status_array: true, active_count: 0 } },
  },
};
assert.equal(H.evaluateSafetyReadiness(readyRobot, 'operator').readyForMission, true);
assert.equal(H.evaluateSafetyReadiness(readyRobot, 'other').readyForMission, false);

const faultRobot = JSON.parse(JSON.stringify(readyRobot));
faultRobot.state.safety_gate.status = 'FAILSAFE_ICP_DEGRADED';
faultRobot.state.safety_gate.fault_reasons = ['icp_inliers_low'];
faultRobot.state.safety_gate.errors = ['GATE_FAULT_LATCHED'];
assert.equal(H.evaluateSafetyReadiness(faultRobot, 'operator').readyForMission, false);
assert.ok(H.collectFaultReasons(faultRobot).includes('fault: icp_inliers_low'));
assert.ok(H.collectFaultReasons(faultRobot).includes('gate_error: GATE_FAULT_LATCHED'));

// 4. Open-RMF origin & mission filtering
const mRmf = { mission_id: 'm_1', robot_id: 'pcu', created_by: 'rmf-adapter', idempotency_key: 'rmf:pcu:1', status: 'FAILED', terminal: true };
const mUi = { mission_id: 'm_2', robot_id: 'pcu', created_by: 'operator', idempotency_key: 'ui-1', status: 'EXECUTING', terminal: false };
assert.equal(H.classifyMissionSource(mRmf), 'RMF');
assert.equal(H.classifyMissionSource(mUi), 'FLEET_UI');
assert.equal(H.filterMissions([mRmf, mUi], { source: 'RMF' }).length, 1);
assert.equal(H.filterMissions([mRmf, mUi], { statusGroup: 'FAILED_GROUP' }).length, 1);
assert.equal(H.filterMissions([mRmf, mUi], { statusGroup: 'ACTIVE' }).length, 1);
assert.equal(H.classifyFleetOrigin({ robot_id: 'pcu', lease: { owner: 'rmf-adapter' } }, []).kind, 'rmf');

// 5. Coordinate round-trip with non-zero yaw and view rotation
const view = { x: 1.5, y: -2.0, zoom: 50, rotation: 0.35, panX: 18, panY: -12, width: 640, height: 480 };
const pt = { x: 3.25, y: 1.75 };
const scr = H.worldToScreen(view, pt);
const back = H.screenToWorld(view, scr);
assert.ok(Math.abs(back.x - pt.x) < 1e-9);
assert.ok(Math.abs(back.y - pt.y) < 1e-9);

const meta = { origin_x: -3.0, origin_y: -4.0, origin_yaw: 0.3, resolution: 0.05 };
const w = H.gridToWorld(meta, 10, 0);
assert.ok(Math.abs(w.x - (-3.0 + 0.5 * Math.cos(0.3))) < 1e-9);
assert.ok(Math.abs(w.y - (-4.0 + 0.5 * Math.sin(0.3))) < 1e-9);

console.log('ALL_JS_HELPER_TESTS_PASS');
