import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const M = require('../server/fleet_server/static/mapping_panel.js');
const H = require('../server/fleet_server/static/state_helpers.js');

const ctx = (o = {}) => ({ canControl: true, online: true, workerEnabled: true, realAgent: false, leaseOwned: true, busy: false, missionActive: false, ...o });
const sess = (state, cmd) => ({ state, session_id: 'abcdef123456', command: cmd });

// ---- allowedActions: the reason is always a visible, specific sentence
let a = M.allowedActions(sess('IDLE'), ctx());
assert.equal(a.START.enabled, true);
assert.equal(a.PAUSE.enabled, false);
assert.match(a.PAUSE.why, /현재 상태 IDLE에서 불가/);
a = M.allowedActions(sess('IDLE'), ctx({ canControl: false }));
assert.ok(Object.values(a).every((v) => !v.enabled && v.why === '운영자 권한 필요'));
a = M.allowedActions(sess('IDLE'), ctx({ online: false }));
assert.equal(a.START.why, '로봇 오프라인');
a = M.allowedActions(sess('IDLE'), ctx({ workerEnabled: false }));
assert.match(a.START.why, /web_mapping_enabled/);                       // SIM/mock agent without a worker
a = M.allowedActions(sess('IDLE'), ctx({ workerEnabled: false, realAgent: true }));
assert.match(a.START.why, /실차 Agent/);
assert.match(a.START.why, /차단 3곳/);
a = M.allowedActions(sess('MAPPING', { status: 'QUEUED', action: 'START' }), ctx());
assert.ok(Object.values(a).every((v) => !v.enabled && v.why === '이전 명령 처리 중'));
a = M.allowedActions(sess('IDLE'), ctx({ leaseOwned: false }));
assert.equal(a.START.why, 'Lease 미보유');
a = M.allowedActions(sess('IDLE'), ctx({ missionActive: true }));
assert.equal(a.START.why, '이동 임무 종료 필요');

// ---- groupReasons: each distinct reason once, with the actions it affects
const g = M.groupReasons(M.allowedActions(sess('MAPPING'), ctx()));
assert.deepEqual(g.map((x) => x.actions.join(',')), ['START,RESUME,SAVE']);          // same sentence -> one group
assert.ok(g.every((x) => x.why.includes('MAPPING')));

// ---- realRobotBlocks: only for real agents, always lists the 3 blocks
assert.equal(M.realRobotBlocks(null), null);
assert.equal(M.realRobotBlocks({ agent: { mode: 'mock', simulated: true, mapping_worker: true } }), null);
const rb = M.realRobotBlocks({ agent: { mode: 'real', simulated: false, mapping_worker: false } });
assert.equal(rb.length, 3);
assert.deepEqual(rb.map((x) => x.id), ['ui', 'agent', 'server']);
assert.ok(rb.every((x) => x.active));
assert.match(rb[2].text, /mapping_real_blocked/);
assert.match(rb[0].text, /mapping_worker=false/);
assert.equal(M.realRobotBlocks({ agent: { mode: 'sim', simulated: false } }).length, 3);      // simulated=false alone is real

// ---- commandPhase: sending / accepted / running / succeeded / failed / timeout / rejected
const now = 1000;
assert.equal(M.commandPhase(null, now).phase, 'none');
assert.equal(M.commandPhase(null, now, { sending: true }).phase, 'sending');
const rej = M.commandPhase(null, now, { httpError: 'operation_state_conflict: UNKNOWN' });
assert.equal(rej.phase, 'rejected'); assert.equal(rej.tone, 'bad'); assert.match(rej.detail, /UNKNOWN/);
let p = M.commandPhase(sess('STARTING', { status: 'QUEUED', action: 'START', expires_at: now + 17 }), now);
assert.equal(p.phase, 'accepted'); assert.match(p.detail, /17초/);
p = M.commandPhase(sess('STARTING', { status: 'ACCEPTED', action: 'START', expires_at: now + 5 }), now);
assert.equal(p.phase, 'running'); assert.match(p.detail, /5초/);
assert.equal(M.commandPhase(sess('MAPPING', { status: 'COMPLETED', action: 'START' }), now).phase, 'succeeded');
p = M.commandPhase({ state: 'MAPPING', detail: 'frontier process exited', command: { status: 'FAILED', action: 'RESUME' } }, now);
assert.equal(p.phase, 'failed'); assert.match(p.detail, /frontier process exited/);
p = M.commandPhase(sess('UNKNOWN', { status: 'UNKNOWN', action: 'PAUSE' }), now);
assert.equal(p.phase, 'timeout'); assert.equal(p.tone, 'bad'); assert.match(p.detail, /자동 재개 없음/);
assert.equal(M.commandPhase(sess('IDLE', { status: 'WEIRD', action: 'X' }), now).phase, 'unknown');
// expired window never shows a negative countdown
assert.match(M.commandPhase(sess('STARTING', { status: 'QUEUED', action: 'START', expires_at: now - 50 }), now).detail, /약 0초/);
// a local rejection / sending state wins over the server's older command status
assert.equal(M.commandPhase(sess('MAPPING', { status: 'COMPLETED', action: 'START' }), now, { sending: true }).phase, 'sending');

// ---- phaseHtml: escapes, marks the current step
const html = M.phaseHtml(M.commandPhase(sess('UNKNOWN', { status: 'UNKNOWN', action: '<b>X</b>' }), now));
assert.ok(!html.includes('<b>X</b>'));
assert.ok(html.includes('&lt;b&gt;X&lt;/b&gt;'));
assert.ok(/class="step cur bad"/.test(html));
assert.match(M.phaseHtml({ phase: 'none' }), /명령 없음/);

// ---- startOptions keeps its contract (CONTINUE needs a stored parent)
const read = (m) => (id) => m[id] ?? '';
assert.throws(() => M.startOptions({ mapVersionId: '__live__' }, read({ webMapMode: 'CONTINUE_MAP' })), /부모/);
assert.equal(M.startOptions({}, read({ webMapMode: 'NEW_MAP', webMapMaxGoals: '10' })).limits.max_goals, 10);

// ---- stale display helpers (state_helpers.js)
assert.equal(H.staleInfo({ online: true, state_age_s: 0.4 }).stale, false);
assert.equal(H.staleInfo({ online: true, state_age_s: 5 }).stale, true);                       // online flag lags, data is old
let si = H.staleInfo({ online: false, state_age_s: 4348 });
assert.equal(si.stale, true); assert.match(si.label, /72분 전/);
assert.match(H.staleInfo({ online: false, state_age_s: 30 }).label, /30초 전/);
assert.equal(H.staleInfo({ online: true }).stale, true);                                       // no age at all is not "live"
assert.equal(H.staleInfo(null).stale, true);
// the age shown for a sensor of an offline robot = age at report + time since report
assert.equal(H.shownAge(0.05, { online: false, state_age_s: 100 }), 100.05);
assert.equal(H.shownAge(0.05, { online: true, state_age_s: 0.4 }), 0.05);
assert.equal(H.shownAge(null, { online: false, state_age_s: 100 }), null);

console.log('ALL_MAPPING_PANEL_JS_TESTS_PASS');
