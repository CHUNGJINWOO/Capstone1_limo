'use strict';
// Phase 4 system panel pure helpers: every readiness level and reason code maps to operator text (unknown codes shown verbatim).
const assert = require('assert');
const P = require('../server/fleet_server/static/system_panel.js');
assert.deepStrictEqual(P.levelView({ level: 'READY' }), { text: '준비 완료', tone: 'ok', code: 'READY' });
assert.strictEqual(P.levelView({ level: 'FAULT' }).tone, 'bad');
assert.strictEqual(P.levelView(null).code, 'UNKNOWN');
assert.strictEqual(P.reasonText('SERVICE_FAILED:stack'), '서비스 실패(재시도 필요) · stack');
assert.strictEqual(P.reasonText('GATE_FAULT:FAILSAFE_SENSOR_TIMEOUT'), 'Safety Gate 장애 래치 · FAILSAFE_SENSOR_TIMEOUT');
assert.strictEqual(P.reasonText('STALE:/scan'), '데이터 끊김 · /scan');
assert.strictEqual(P.reasonText('SOMETHING_NEW'), 'SOMETHING_NEW');
// Codex 2026-11-21: service actions carry the epoch of the lease THIS admin holds; anything else sends null (server refuses)
assert.strictEqual(P.leaseEpoch({ lease: { owner: 'admin', epoch: 7 } }, { name: 'admin' }), 7);
assert.strictEqual(P.leaseEpoch({ lease: { owner: 'operator', epoch: 7 } }, { name: 'admin' }), null);
assert.strictEqual(P.leaseEpoch({ lease: null }, { name: 'admin' }), null);
assert.strictEqual(P.leaseEpoch({ lease: { owner: 'admin', epoch: '7' } }, { name: 'admin' }), null);
assert.strictEqual(P.reasonText('SERVICE_MISSING:stack'), '필수 서비스 보고 없음 · stack');
const Q = require('../server/fleet_server/static/qr_encode.js');
assert.strictEqual(Q.matrix('hello').length, 21);                       // version 1
assert.strictEqual(Q.matrix('x'.repeat(200)).length, 57);               // version 10
assert.throws(() => Q.matrix('x'.repeat(300)));
assert.ok(Q.svg('http://127.0.0.1:8080/visitor.html#t=abc').startsWith('<svg'));
console.log('PASS system panel helpers + QR encoder structure (decoding not verified here)');
