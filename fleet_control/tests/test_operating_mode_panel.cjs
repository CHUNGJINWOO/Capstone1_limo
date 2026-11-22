'use strict';
// Pure checks for static/operating_mode_panel.js (no browser, no HTTP).
const assert = require('node:assert/strict');
const P = require('../server/fleet_server/static/operating_mode_panel.js');
const me = { name: 'operator', role: 'operator' };
const base = () => ({ robot_id: 'R', online: true, lease: { owner: 'operator', epoch: 3, remaining_s: 40 },
  agent: { mode: 'mock', simulated: true, manual_web_bench: true, mapping_worker: true, missions_enabled: true },
  operating_mode: { mode: 'IDLE', reason: 'initial', transition: null }, state: {} });

let r = base(); let v = P.view(r, me);
assert.equal(v.badge, '대기 IDLE');
assert.ok(v.buttons.MANUAL.enabled && v.buttons.MAPPING.enabled && v.buttons.NAVIGATION.enabled);
assert.ok(v.rows.find(([k, x]) => k === 'Watchdog' && x === '미보고'), 'missing data is shown as not reported');

r = base(); r.lease = null; v = P.view(r, me);
assert.match(v.buttons.MANUAL.why, /Lease/);
r = base(); v = P.view(r, { name: 'viewer', role: 'viewer' });
assert.match(v.buttons.MAPPING.why, /권한/);

r = base(); r.agent.mode = 'real'; r.agent.simulated = false; v = P.view(r, me);
assert.match(v.buttons.MAPPING.why, /실차/);

r = base(); r.operating_mode = { mode: 'TRANSITIONING', transition: { from: 'MAPPING', to: 'MANUAL', state: 'VERIFYING_IDLE', blocker: 'motion watchdog not authorized (LOCKED)' } };
v = P.view(r, me);
assert.ok(Object.values(v.buttons).every(b => !b.enabled), 'no request while a transition runs');
assert.match(v.steps, /정지·Goal 종료 검증/); assert.match(v.steps, /watchdog/);

r = base(); r.operating_mode = { mode: 'FAULT', reason: 'x', transition: { from: 'MAPPING', to: 'MANUAL', state: 'FAILED', reason: 'mapping session UNKNOWN (reconciliation required)' } };
v = P.view(r, me);
assert.equal(v.tone, 'bad'); assert.match(v.steps, /자동 재시도 없음/); assert.ok(v.buttons.IDLE.enabled);

r = base(); r.state = { execution: { active: { owner: 'frontier', mission_id: 's1', goal_uuid: 'abcdef0123' }, nav_idle: { ok: false, reason: 'NAV2_GOAL_ACTIVE' } },
  velocity_arbiter: { source: 'NAV2', selected_linear: 0.12, selected_turn: 0.1, operating_mode: 'MAPPING', deadman: false, blocker: null },
  motion_watchdog: { state: 'MOTION_AUTHORIZED' }, safety_gate: { status: 'NORMAL' }, mapping_worker: { enabled: true, status: 'MAPPING', goal: { status: 'ACCEPTED' } } };
v = P.view(r, me);
const row = k => v.rows.find(([n]) => n === k)[1];
assert.match(row('활성 Nav2 Goal'), /frontier/); assert.match(row('속도 출처'), /NAV2/); assert.equal(row('Manual Deadman'), '해제');
assert.match(row('Mapping Worker'), /MAPPING/);
console.log('PASS operating mode panel: gating reasons, transition steps, FAILED/FAULT, status rows, no invented values');
