import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const P = require('../server/fleet_server/static/map_save_panel.js');

const ok = { canControl: true, robot: true, online: true, agentCapable: true, capAvailable: true, capReason: '', leaseOwned: true, floorId: 'fl-1', labelValid: true, activeSave: false, busy: false };

// ---- blockReasons: ordered, specific, empty when sendable
assert.deepEqual(P.blockReasons(ok), []);
assert.match(P.blockReasons({ ...ok, canControl: false })[0], /운영자 권한/);
assert.deepEqual(P.blockReasons({ ...ok, robot: false }).length >= 1, true);
assert.match(P.blockReasons({ ...ok, online: false })[0], /오프라인/);
assert.match(P.blockReasons({ ...ok, agentCapable: false })[0], /map_save_enabled/);
assert.match(P.blockReasons({ ...ok, capAvailable: false, capReason: 'no live /map received (SLAM is not running)' })[0], /SLAM is not running/);
assert.match(P.blockReasons({ ...ok, leaseOwned: false }).join('|'), /Lease/);
assert.match(P.blockReasons({ ...ok, floorId: '' }).join('|'), /층/);
assert.match(P.blockReasons({ ...ok, labelValid: false }).join('|'), /지도 이름/);
assert.match(P.blockReasons({ ...ok, activeSave: true }).join('|'), /진행 중/);
assert.match(P.blockReasons({ ...ok, busy: true }).join('|'), /보내는 중/);
// an offline robot does not also complain about the lease (one root cause at a time)
assert.ok(!P.blockReasons({ ...ok, online: false, leaseOwned: false }).join('|').includes('Lease'));
// reasons accumulate when several independent things are missing
assert.ok(P.blockReasons({ ...ok, floorId: '', labelValid: false }).length >= 2);

// ---- label rule matches the server (letter/digit first, . _ - allowed, 64 max)
assert.ok(P.ID_RE.test('h107_20261120')); assert.ok(P.ID_RE.test('A.b-c_1'));
assert.ok(!P.ID_RE.test('_x')); assert.ok(!P.ID_RE.test('with space')); assert.ok(!P.ID_RE.test('')); assert.ok(!P.ID_RE.test('a'.repeat(65)));

// ---- savePhase: every state distinguishable
const now = 1000;
assert.equal(P.savePhase(null, now).phase, 'none');
assert.equal(P.savePhase(null, now, { sending: true }).phase, 'sending');
const rej = P.savePhase(null, now, { httpError: 'save_in_progress: another map save is still running' });
assert.equal(rej.phase, 'rejected'); assert.equal(rej.tone, 'bad'); assert.match(rej.detail, /save_in_progress/);
let p = P.savePhase({ status: 'QUEUED', expires_at: now + 200 }, now);
assert.equal(p.phase, 'queued'); assert.equal(p.index, 0); assert.match(p.detail, /200초/);
assert.equal(P.savePhase({ status: 'ACCEPTED', expires_at: now + 90 }, now).index, 1);
assert.match(P.savePhase({ status: 'SAVING_RASTER' }, now).label, /이미지/);
assert.match(P.savePhase({ status: 'SAVING_POSEGRAPH' }, now).label, /Pose Graph/);
assert.equal(P.savePhase({ status: 'UPLOADING' }, now).index, 4);
p = P.savePhase({ status: 'REGISTERED', result: { map_version_id: 'm#1' } }, now);
assert.equal(p.phase, 'succeeded'); assert.equal(p.tone, 'ok'); assert.match(p.label, /UNVERIFIED/); assert.equal(p.detail, 'm#1');
p = P.savePhase({ status: 'FAILED', error_code: 'raster_failed', detail: 'raster: map_saver_cli exited' }, now);
assert.equal(p.phase, 'failed'); assert.match(p.detail, /지도 이미지.*저장에 실패/); assert.match(p.detail, /map_saver_cli exited/);
p = P.savePhase({ status: 'TIMEOUT', error_code: 'timeout' }, now);
assert.equal(p.phase, 'timeout'); assert.match(p.label, /타임아웃/);
p = P.savePhase({ status: 'UNKNOWN', error_code: 'agent_restarted' }, now);
assert.equal(p.phase, 'unknown'); assert.match(p.detail, /자동 재개 없음/);
assert.equal(P.savePhase({ status: 'UNKNOWN', error_code: 'robot_offline' }, now).phase, 'unknown');
assert.equal(P.savePhase({ status: 'WEIRD' }, now).phase, 'unknown');
// the countdown never goes negative
assert.match(P.savePhase({ status: 'UPLOADING', expires_at: now - 50 }, now).detail, /약 0초/);
// a local send/rejection wins over an older server record
assert.equal(P.savePhase({ status: 'REGISTERED' }, now, { sending: true }).phase, 'sending');

// ---- errorText: friendly for known codes, raw for unknown
assert.match(P.errorText('unknown_pixels_misclassified', 'x'), /임계값/);
assert.match(P.errorText('upload_failed', ''), /파일은 로봇에 남아/);
assert.equal(P.errorText('brand_new_code', 'm'), 'brand_new_code (m)');
assert.equal(P.errorText('', ''), '실패');

// ---- stepperHtml: current step marked, escapes, failure replaces the last step
const h = P.stepperHtml(P.savePhase({ status: 'SAVING_POSEGRAPH', expires_at: now + 10 }, now));
assert.match(h, /class="step cur warn">Pose Graph 저장/);
assert.equal((h.match(/class="step past"/g) || []).length, 3);
const bad = P.stepperHtml(P.savePhase({ status: 'FAILED', error_code: '<img onerror=x>', detail: '<b>' }, now));
assert.ok(!bad.includes('<img') && !bad.includes('<b>'));
assert.match(bad, /class="step cur bad">실패/);
assert.match(P.stepperHtml(P.savePhase({ status: 'TIMEOUT' }, now)), /미확인/);
assert.match(P.stepperHtml({ phase: 'none' }), /저장 기록 없음/);
assert.match(P.stepperHtml(P.savePhase({ status: 'REGISTERED', result: {} }, now)), /class="step cur ok">등록/);

// ---- filesRows / consistencyRows
const save = { manifest: { files: {
  yaml: { size: 121, sha256: 'a'.repeat(64), saved_start: 1795147000.1, saved_end: 1795147000.9 },
  pgm: { size: 27176, sha256: 'b'.repeat(64), saved_start: 1795147000.1, saved_end: 1795147000.9 },
  posegraph: { size: 4967401, sha256: 'c'.repeat(64), saved_start: 1795147001.1, saved_end: 1795147002.5 },
  data: { size: 0, sha256: null, saved_start: 1795147001.1, saved_end: 1795147002.5, problem: 'empty file' } } },
  consistency: { same_snapshot: 'NOT_ESTABLISHED', gap_between_raster_end_and_posegraph_start_s: 0.2, live_map_unchanged_during_save: true,
                 robot_stationary_during_save: false, robot_moved_m: 0.12, robot_rotated_rad: 0.01 } };
const rows = P.filesRows(save);
assert.equal(rows.length, 4); assert.equal(rows[0][0], 'map.yaml'); assert.match(rows[1][1], /27176 B · bbbbbbbbbbbbbbbb…/);
assert.match(rows[3][1], /⚠ empty file/); assert.match(rows[3][1], /해시 없음/);
assert.deepEqual(P.filesRows(null), []); assert.deepEqual(P.filesRows({ manifest: {} }), []);
const cr = Object.fromEntries(P.consistencyRows(save));
assert.match(cr['두 저장물의 일치'], /확정할 수 없음/);
assert.match(cr['저장 사이 간격'], /0\.2 s/);
assert.equal(cr['저장 중 라이브 지도 변화 없음'], '예');
assert.match(cr['저장 중 로봇 정지'], /아니오.*0\.12 m/);
assert.match(Object.fromEntries(P.consistencyRows({ consistency: { same_snapshot: 'LIKELY_NOT_PROVEN' } }))['두 저장물의 일치'], /증명되지 않음/);
assert.deepEqual(P.consistencyRows(null), []);

console.log('ALL_MAP_SAVE_PANEL_JS_TESTS_PASS');
