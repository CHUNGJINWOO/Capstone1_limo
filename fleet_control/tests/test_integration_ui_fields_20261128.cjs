// Integration UI fields (2026-11-28): mode_motion, dry-run missions, API errors, Gate generation evidence. Pure helper tests (node, no browser).
const assert = require('assert');
const H = require('../server/fleet_server/static/state_helpers.js');
let n = 0; const ok = (name, fn) => { fn(); n++; console.log('PASS ' + name); };
const robot = (over) => Object.assign({ online: true, agent: {}, state: { safety_gate: {} }, operating_mode: { mode: 'NAVIGATION' } }, over);
ok('mission endpoint is separate for a dry-run Agent', () => {
  assert.strictEqual(H.missionEndpoint(robot({ agent: { real_nav_dry_run: true } })), '/dry-run-missions');
  assert.strictEqual(H.missionEndpoint(robot({ agent: { real_nav_dry_run: false } })), '/missions');
  assert.strictEqual(H.missionEndpoint(robot({ agent: { real_nav_dry_run: 'true' } })), '/missions', 'only a strict true selects the dry-run endpoint');
  assert.strictEqual(H.missionEndpoint(null), '/missions');
});
ok('dry-run is never shown as real motion', () => {
  const d = H.classifyDryRun(robot({ agent: { real_nav_dry_run: true } }));
  assert.strictEqual(d.dryRun, true); assert.strictEqual(d.realMotionAuthorized, false); assert.ok(d.label.includes('실제 주행 아님'));
  assert.strictEqual(H.classifyDryRun(robot()).realMotionAuthorized, false, 'the UI never claims real motion authorization');
});
ok('mode_motion distinguishes server mode from motion permission', () => {
  const v = H.classifyModeMotion(robot({ mode_motion: { verified: true, blocker: null, mode: 'MANUAL' } }));
  assert.strictEqual(v.status, 'VERIFIED'); assert.strictEqual(v.motionPermitted, false, 'verified is one input, never "motion permitted"');
  const u = H.classifyModeMotion(robot({ mode_motion: { verified: false, blocker: 'the Watchdog does not confirm MANUAL', hard: true, mode: 'MANUAL' } }));
  assert.strictEqual(u.status, 'UNVERIFIED'); assert.strictEqual(u.tone, 'bad'); assert.ok(u.label.includes('차단')); assert.strictEqual(u.blocker, 'the Watchdog does not confirm MANUAL');
  assert.strictEqual(H.classifyModeMotion(robot({ mode_motion: { verified: null } })).status, 'NOT_APPLICABLE');
  assert.strictEqual(H.classifyModeMotion(robot()).status, 'NOT_APPLICABLE', 'a legacy server without the field');
});
ok('error codes are explained', () => {
  for (const c of ['mode_unverified', 'dry_run_agent', 'not_dry_run_agent']) assert.ok(H.explainApiError(c, 'x').length > 20 && H.explainApiError(c, 'x') !== 'x', c);
  assert.strictEqual(H.explainApiError('something_else', 'server text'), 'server text');
});
ok('Gate evidence is labelled unauthenticated and shows reset blockers', () => {
  const g = H.classifyGateEvidence(robot({ state: { safety_gate: { fault_epoch: 3, fault_latched: true, boot_id: 'abcdef0123456789', reset_view: { boot_id: 'abcdef0123456789', reset_readiness: { enforced: true, ready: false, blockers: ['INPUT_SILENT', 'INPUT_WRITERS:2'] }, last_reset: { result: 'REFUSED' } } } } }));
  assert.strictEqual(g.authenticated, false); assert.ok(g.label.includes('비인증')); assert.ok(g.label.includes('운동 허가가 아닙니다'));
  assert.strictEqual(g.faultEpoch, 3); assert.strictEqual(g.faultLatched, true); assert.strictEqual(g.resetReady, false); assert.deepStrictEqual(g.resetBlockers, ['INPUT_SILENT', 'INPUT_WRITERS:2']);
  const none = H.classifyGateEvidence(robot());
  assert.strictEqual(none.faultEpoch, null); assert.strictEqual(none.faultLatched, null); assert.strictEqual(none.resetEnforced, false);
  const bad = H.classifyGateEvidence(robot({ state: { safety_gate: { fault_epoch: '3', fault_latched: 'false' } } }));
  assert.strictEqual(bad.faultEpoch, null, 'malformed values are not trusted'); assert.strictEqual(bad.faultLatched, null);
});
ok('mode buttons: a dry-run Agent may enter NAVIGATION, a plain missions-disabled Agent may not', () => {
  const P = require('../server/fleet_server/static/operating_mode_panel.js');
  const fn = P.buttons;
  const me = { name: 'op', role: 'operator' };
  const mk = (agent) => ({ online: true, lease: { owner: 'op', epoch: 1 }, agent, operating_mode: { mode: 'IDLE' } });
  const call = (agent) => fn(mk(agent), me);
  const dry = call({ real_nav_dry_run: true, missions_enabled: false, mapping_worker: true, simulated: true, mode: 'mock', velocity_selector: 'WATCHDOG_SELECTOR' });
  const plain = call({ missions_enabled: false, mapping_worker: true, simulated: true, mode: 'mock' });
  assert.strictEqual(dry.NAVIGATION.enabled, true); assert.strictEqual(plain.NAVIGATION.enabled, false);
  assert.strictEqual(dry.MANUAL.enabled, true, 'selector-shaped Agent: manual path is the Watchdog selector'); assert.strictEqual(plain.MANUAL.enabled, false);
});
console.log(`${n}/6 PASS`);
