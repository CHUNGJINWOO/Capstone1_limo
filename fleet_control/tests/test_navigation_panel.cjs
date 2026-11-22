'use strict';
// Pure checks for static/navigation_panel.js (no browser, no HTTP).
const assert = require('node:assert/strict');
const P = require('../server/fleet_server/static/navigation_panel.js');
const me = { name: 'operator', role: 'operator' };
const robot = (over = {}) => Object.assign({ robot_id: 'R', online: true, lease: { owner: 'operator', epoch: 2 },
  agent: { navigation_localization: true }, operating_mode: { mode: 'NAVIGATION', transition: { state: 'ACTIVE' } }, navigation: null, state: {} }, over);
let s = P.steps(robot({ operating_mode: { mode: 'MANUAL', transition: { state: 'ACTIVE' } } }), me);
assert.match(s.load, /NAVIGATION/); assert.ok(s.pose && s.goal, 'later steps blocked too');
s = P.steps(robot(), me); assert.equal(s.load, ''); assert.match(s.pose, /지도를 로딩/);
s = P.steps(robot({ navigation: { map_version_id: 'm', verification_status: 'UNVERIFIED', map_verified: false, localized: true } }), me);
assert.match(s.goal, /UNVERIFIED/);
s = P.steps(robot({ navigation: { map_version_id: 'm', verification_status: 'VERIFIED', map_verified: true, localized: false } }), me);
assert.match(s.goal, /CONFIRMED/);
s = P.steps(robot({ navigation: { map_version_id: 'm', verification_status: 'VERIFIED', map_verified: true, localized: true } }), me);
assert.equal(s.goal, '');
const v = P.view(robot({ state: { localization: { status: 'CONFIRMED', match_ratio: 0.83, planner_allow_unknown: false, nav_path: { points: [[0, 0], [1, 1]], age_s: 0.4 } } } }), me);
const row = k => v.rows.find(([n]) => n === k)[1];
assert.match(row('스캔/지도 일치'), /83%/); assert.match(row('계획기 미지 영역'), /불허/); assert.match(row('Nav2 경로'), /2점/);
assert.equal(P.view(robot(), me).rows.find(([n]) => n === 'AMCL')[1], '미보고');
assert.match(P.goalText({ ok: false, reason: 'UNKNOWN_AREA' }), /미지 영역/);
assert.match(P.goalText({ ok: true, clearance_m: 0.2, localized: false }), /제출 불가/);
console.log('PASS navigation panel: step gating (mode, map, localization, UNVERIFIED), rows, goal reasons, no invented values');
