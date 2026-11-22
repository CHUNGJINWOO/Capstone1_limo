import assert from 'node:assert/strict';
import { createRequire } from 'node:module';

const require = createRequire(import.meta.url);
const U = require('../server/fleet_server/static/registry_upload.js');

// 1. summarizeYaml
const yOk = U.summarizeYaml('image: map.pgm\nresolution: 0.05\norigin: [-1.5, -2.0, 0.0]\nframe_id: map\n');
assert.equal(yOk.image, 'map.pgm');
assert.equal(yOk.resolution, 0.05);
assert.deepEqual(yOk.origin, [-1.5, -2.0, 0.0]);
assert.equal(yOk.frame_id, 'map');
assert.equal(yOk.warnings.length, 0);

const yYaw = U.summarizeYaml('image: other.pgm\nresolution: 0.05\norigin: [0, 0, 0.25]\n');
assert.ok(yYaw.warnings.some((w) => w.includes('map.pgm')));
assert.ok(yYaw.warnings.some((w) => w.includes('yaw')));

const yBad = U.summarizeYaml('resolution: -1\norigin: [1, 2]\nframe_id: "bad space"\n');
assert.ok(yBad.warnings.some((w) => w.includes('resolution')));
assert.ok(yBad.warnings.some((w) => w.includes('origin')));
assert.ok(yBad.warnings.some((w) => w.includes('frame_id')));

// 2. pgmHeader
const pgm = new Uint8Array(Buffer.from('P5\n# comment\n36 28\n255\nXYZ'));
assert.deepEqual(U.pgmHeader(pgm), { width: 36, height: 28, maxval: 255 });
assert.equal(U.pgmHeader(new Uint8Array(Buffer.from('P2\n10 10\n255\n'))), null);
assert.equal(U.pgmHeader(new Uint8Array([1, 2, 3])), null);

// 3. bytesToB64
assert.equal(U.bytesToB64(new Uint8Array(Buffer.from('hello'))), 'aGVsbG8=');

// 4. validateInputs
assert.ok(U.validateInputs({}).length >= 4);
assert.equal(
  U.validateInputs({
    floorId: 'fl-l1',
    label: 'hall-a-v1',
    frameId: 'map',
    yaml: { size: 120 },
    pgm: { size: 1024 },
  }).length,
  0,
);
assert.ok(
  U.validateInputs({
    floorId: 'fl-l1',
    label: '../bad',
    yaml: { size: 10 },
    pgm: { size: 10 },
  }).some((e) => e.includes('지도 이름')),
);
assert.ok(
  U.validateInputs({
    floorId: 'fl-l1',
    label: 'ok-v1',
    yaml: { size: 10 },
    pgm: { size: 10 },
    posegraphData: { size: 10 },
  }).some((e) => e.includes('posegraph.data')),
);
assert.ok(
  U.validateInputs({
    floorId: 'fl-l1',
    label: 'ok-v1',
    yaml: { size: 300 * 1024 },
    pgm: { size: 10 },
  }).some((e) => e.includes('map.yaml')),
);

// 5. describeError
assert.ok(U.describeError(0, null).includes('연결'));
assert.ok(U.describeError(401, {}).includes('인증'));
assert.ok(U.describeError(403, {}).includes('operator/admin'));
assert.ok(U.describeError(409, { error: { message: 'conflict' } }).includes('충돌'));
assert.ok(U.describeError(413, {}).includes('너무 큽니다'));
assert.ok(U.describeError(422, { error: { code: 'checksum_mismatch' } }).includes('체크섬 불일치'));
assert.ok(U.describeError(503, {}).includes('다른 지도 업로드'));

// 6. describeResult
const r201 = U.describeResult(201, {
  map_version: {
    map_version_id: 'm1#abcdef123456',
    label: 'm1',
    verification_status: 'UNVERIFIED',
    frame_id: 'map',
    resolution: 0.05,
    origin: [-1, -2, 0],
    width: 36,
    height: 28,
    bundle_sha256: 'bsha',
    sha256: { content_sha256: 'csha', yaml_sha256: 'ysha', pgm_sha256: 'psha' },
  },
});
assert.equal(r201.kind, 'ok');
assert.ok(r201.title.includes('UNVERIFIED'));
assert.ok(r201.rows.some(([k, v]) => k === '지도 버전 ID' && v === 'm1#abcdef123456'));

const rDup = U.describeResult(200, { idempotent: true, map_version: { map_version_id: 'm1#abcdef123456' } });
assert.equal(rDup.kind, 'warn');
assert.ok(rDup.title.includes('중복'));

const rReplay = U.describeResult(201, { idempotent_replay: true, map_version: { map_version_id: 'm1#abcdef123456' } });
assert.equal(rReplay.kind, 'warn');
assert.ok(rReplay.title.includes('멱등 키'));

// 7. thresholdCheck / thresholdMessage: same rule as the server and register_saved_map.py
function trinaryPgm(w, h, unknownCells) {
  const px = new Uint8Array(w * h).fill(254);
  for (let i = 0; i < w; i++) { px[i] = 0; px[(h - 1) * w + i] = 0; }
  for (let i = 0; i < unknownCells; i++) px[w + 3 + i] = 205;
  return new Uint8Array([...Buffer.from(`P5\n${w} ${h}\n255\n`), ...px]);
}
const yml = (free, occ = '0.65', negate = 0) => `image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: ${negate}\noccupied_thresh: ${occ}\nfree_thresh: ${free}\n`;
const tp = trinaryPgm(10, 8, 6);
let c = U.thresholdCheck(yml('0.25'), tp);               // map_saver_cli default
assert.equal(c.judged, true); assert.equal(c.trinary, true); assert.equal(c.issue, true); assert.equal(c.fixable, true);
assert.equal(c.derived, 0.196); assert.equal(c.unknownCells, 6); assert.equal(c.pUnknown, 0.19608);
assert.equal(c.knownCells, 80 - 6);
c = U.thresholdCheck(yml('0.196'), tp);
assert.equal(c.issue, false);
c = U.thresholdCheck(yml('0.25', '0.65', 1), tp);        // negate=1: unknown p = 0.80 > occupied 0.65, not fixable by free_thresh
assert.equal(c.issue, true); assert.equal(c.fixable, false);
const scale = new Uint8Array([...Buffer.from('P5\n4 4\n255\n'), ...new Uint8Array(16).fill(100)]);
c = U.thresholdCheck(yml('0.25'), scale);                // not a trinary map: not judged as an issue
assert.equal(c.trinary, false); assert.equal(c.issue, false);
assert.equal(U.thresholdCheck('image: map.pgm\n', tp).judged, false);                 // no thresholds in yaml
assert.equal(U.thresholdCheck(yml('0.25'), new Uint8Array(Buffer.from('P5\n10 8\n255\nshort'))).judged, false);   // truncated pixels
assert.equal(U.pgmDataOffset(new Uint8Array(Buffer.from('P5\n# c\n3 2\n255\nabcdef'))), 15);
assert.equal(U.thresholdMessage(U.thresholdCheck(yml('0.25'), tp)).kind, 'warn');
assert.match(U.thresholdMessage(U.thresholdCheck(yml('0.25'), tp)).text, /보정본 등록/);
assert.equal(U.thresholdMessage(U.thresholdCheck(yml('0.196'), tp)).kind, 'ok');
assert.equal(U.thresholdMessage(U.thresholdCheck(yml('0.25', '0.65', 1), tp)).kind, 'bad');
assert.equal(U.thresholdMessage(null).text, '');

// 8. validateInputs: lineage + threshold rules
const file = (n) => ({ size: n });
const base = { floorId: 'f', label: 'h107_v1', yaml: file(100), pgm: file(1000) };
assert.deepEqual(U.validateInputs({ ...base, mode: 'IMPORTED', policy: 'reject' }), []);
assert.ok(U.validateInputs({ ...base, mode: 'CONTINUE_MAP', parentId: '__live__' }).some((e) => e.includes('CONTINUE_MAP')));
assert.ok(U.validateInputs({ ...base, mode: 'CONTINUE_MAP', parentId: '' }).some((e) => e.includes('CONTINUE_MAP')));
assert.deepEqual(U.validateInputs({ ...base, mode: 'CONTINUE_MAP', parentId: 'mv#123' }), []);
const issue = U.thresholdCheck(yml('0.25'), tp);
assert.ok(U.validateInputs({ ...base, policy: 'reject', thresholdCheck: issue }).some((e) => e.includes('임계값')));
assert.deepEqual(U.validateInputs({ ...base, policy: 'derive', thresholdCheck: issue }), []);
assert.ok(U.validateInputs({ ...base, policy: 'derive', thresholdCheck: U.thresholdCheck(yml('0.25', '0.65', 1), tp) }).some((e) => e.includes('자동 보정')));

// 9. describeError: threshold refusal and area conflict
const e422 = U.describeError(422, { error: { code: 'policy_violation', message: 'x', detail: { code: 'unknown_pixels_misclassified', free_thresh: 0.25, p_unknown: 0.19608, unknown_cells: 7, fixable_by_free_thresh: true, derived_free_thresh: 0.196 } } });
assert.match(e422, /임계값 안전 검사 거부/); assert.match(e422, /0\.196/); assert.match(e422, /원본 보존/);
assert.match(U.describeError(422, { error: { code: 'policy_violation', detail: { code: 'unknown_pixels_misclassified', free_thresh: 0.25, p_unknown: 0.2, unknown_cells: 1, fixable_by_free_thresh: false } } }), /자동 보정할 수 없/);
assert.match(U.describeError(409, { error: { code: 'conflict', message: 'area belongs to a different floor than floor_id' } }), /영역 충돌/);
assert.match(U.describeError(409, { error: { code: 'conflict', message: 'same label' } }), /충돌/);

// 10. describeResult + evidenceRows: what the server stored
const dr = U.describeResult(201, { map_version: { map_version_id: 'm#1', label: 'l', verification_status: 'UNVERIFIED' }, area_id: 'h107',
  lineage: { mapping_mode: 'NEW_MAP', parent_map_version_id: null }, yaml_adjustment: { free_thresh_from: 0.25, free_thresh_to: 0.196 } });
assert.ok(dr.rows.some((r) => r[0] === '영역' && r[1] === 'h107'));
assert.ok(dr.rows.some((r) => r[0] === 'YAML 보정' && r[1].includes('0.25 → 0.196')));
const rows = U.evidenceRows({ verification_status: 'UNVERIFIED', is_floor_active: false, area: { area_id: 'h107', name: 'H107 강의실' },
  lineage: { mapping_mode: 'NEW_MAP', parent_map_version_id: null, session_id: 's1' },
  verification_evidence: { uploader: 'op', note: '한 자리', client_checksums_provided: true,
    location: { facility: '배재대학교', building: 'H관', floor: '1F', area: 'H107 강의실', coverage: 'PARTIAL_FLOOR' },
    grid_counts_pgm: { known_cells: 866, known_area_m2: 2.17, free: 851, occupied: 15, unknown: 26295 },
    yaml_adjustment: { free_thresh_from: 0.25, free_thresh_to: 0.196, original_yaml_sha256: 'aaa', derived_yaml_sha256: 'bbb' } } });
const rm = Object.fromEntries(rows);
assert.match(rm['위치(운영자 선언)'], /배재대학교 › H관 › 1F › H107 강의실 \(층 일부\)/);
assert.match(rm['확인된 칸 / 면적'], /866칸 · 약 2\.17 m²/);
assert.match(rm['YAML'], /보정본 등록: free_thresh 0\.25 → 0\.196/);
assert.equal(rm['원본 YAML sha256'], 'aaa'); assert.equal(rm['등록된 YAML sha256'], 'bbb');
assert.match(rm['검증 상태'], /UNVERIFIED · 층 활성 지도 아님/);
assert.match(rm['클라이언트 체크섬'], /일치/);
assert.deepEqual(U.evidenceRows(null), []);

console.log('ALL_REGISTRY_UPLOAD_JS_TESTS_PASS');
