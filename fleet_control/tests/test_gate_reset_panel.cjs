'use strict';
// Pure view helpers of the admin Gate reset card. The button state is a convenience; the server enforces the role (tested in Python).
const assert = require('assert');
const P = require('../server/fleet_server/static/gate_reset_panel.js');
const admin = { name: 'admin', role: 'admin' }, oper = { name: 'operator', role: 'operator' };
const ready = { gate: { status: 'FAILSAFE_SENSOR_TIMEOUT', report_age_s: 0.2, view: { state: 'FAILSAFE_SENSOR_TIMEOUT', fault_reasons: [], last_fault_reasons: ['cmd_input:NOT_EXCLUSIVE'],
  cmd_input: { state: 'NOT_EXCLUSIVE', sticky: 'NOT_EXCLUSIVE', publishers: ['/fleet_motion_watchdog:010f'] }, reset_readiness: { enforced: true, ready: true, blockers: [] } } },
  server_blockers: [], request: null };
let v = P.view(ready, admin);
assert.strictEqual(v.can, true); assert.strictEqual(v.supported, true);
assert.ok(v.rows.find(r => r[0] === '입력 발행자')[1].includes('Reset 필요'));
assert.strictEqual(P.view(ready, oper).can, false);                                  // not admin
v = P.view({ ...ready, server_blockers: ['ROLE_NOT_ADMIN', 'MODE_NOT_IDLE:MANUAL'] }, admin);
assert.strictEqual(v.can, false); assert.deepStrictEqual(v.serverBlockers, ['관리자 권한 필요', '운용 모드 MANUAL (IDLE 필요)']);
const notReady = JSON.parse(JSON.stringify(ready)); notReady.gate.view.reset_readiness = { enforced: true, ready: false, blockers: ['INPUT_WRITERS:2', 'MOTION_COMMANDED'] };
v = P.view(notReady, admin); assert.strictEqual(v.can, false);
assert.deepStrictEqual(v.gateBlockers, ['입력 발행자 2개 (유령 endpoint 포함 가능)', '속도 명령 수신 중']);
const unsupported = JSON.parse(JSON.stringify(ready)); delete unsupported.gate.view.reset_readiness;
v = P.view(unsupported, admin); assert.strictEqual(v.can, false); assert.strictEqual(v.supported, false);
v = P.view({ ...ready, request: { status: 'DELIVERED' } }, admin); assert.strictEqual(v.pending, true);
v = P.view({ ...ready, request: { status: 'REFUSED', reason: 'AGENT_PRECHECK:publisher_identity', failed_checks: ['publisher_identity'] } }, admin);
assert.ok(v.request.startsWith('거부')); assert.ok(v.request.includes('publisher_identity'));
v = P.view(null, admin);                                                              // nothing reported: no invented values
assert.strictEqual(v.can, false); assert.ok(v.rows.every(r => r[1] === '미보고' || r[1] === '없음'));
// 2026-11-21: retryability and the Gate's own last decision
const nr2 = JSON.parse(JSON.stringify(ready)); nr2.gate.view.reset_readiness = { enforced: true, ready: false, blockers: ['MOTION_COMMANDED'] };
nr2.gate.view.last_reset = { result: 'REFUSED', blockers: ['MOTION_COMMANDED'] }; nr2.gate.view.reset_refusals = 2;
v = P.view(nr2, admin); assert.strictEqual(v.retry.code, 'WAIT'); assert.strictEqual(v.can, false);
assert.ok(v.rows.find(r => r[0] === 'Gate 최근 Reset 판단')[1].includes('거부')); assert.strictEqual(v.rows.find(r => r[0] === 'Gate Reset 거부 횟수')[1], '2');
v = P.view({ ...ready, server_blockers: ['MODE_NOT_IDLE:MANUAL', 'WATCHDOG_NOT_AUTHORIZED:LOCKED'] }, admin); assert.strictEqual(v.retry.code, 'ACTION');
v = P.view({ ...ready, server_blockers: ['GATE_NOT_FAULTED'] }, admin); assert.strictEqual(v.retry.code, 'NONE');
assert.strictEqual(P.view(ready, admin).retry.code, 'READY');
assert.strictEqual(P.kind('INPUT_WRITERS:2'), 'wait'); assert.strictEqual(P.kind('SENSOR_FAULT:scan'), 'action');
console.log('PASS gate reset panel: admin/role gating, Gate + server blockers, unsupported Gate, pending/result text, retryability, Gate last decision, no invented values');
