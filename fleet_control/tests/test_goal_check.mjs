// Node checks for static/goal_check.js (pixel <-> map transform, occupancy classes, clearance, geofence).
// Uses the real 2026-11-19 C103 PGM when present (read-only), plus synthetic grids for origin yaw and row flip.
import { createRequire } from 'module';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';
const require = createRequire(import.meta.url);
const here = path.dirname(fileURLToPath(import.meta.url));
const G = require(path.join(here, '..', 'server', 'fleet_server', 'static', 'goal_check.js'));
const H = require(path.join(here, '..', 'server', 'fleet_server', 'static', 'state_helpers.js'));
let pass = 0, fail = 0;
const check = (name, ok, extra = '') => { ok ? pass++ : fail++; console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${name} ${extra}`); };

// 1) synthetic 10x6 grid, row 0 of the IMAGE = top = +y. One occupied cell at grid (u=2, v=4).
{
  const meta = { width: 10, height: 6, resolution: 0.5, origin_x: -1, origin_y: -2, origin_yaw: 0 };
  const px = new Uint8Array(60).fill(255);
  px[(6 - 1 - 4) * 10 + 2] = 0;                    // grid v=4 -> image row 1
  const w = { x: -1 + 2.25 * 0.5, y: -2 + 4.25 * 0.5 };
  const g = G.worldToGrid(meta, w.x, w.y);
  check('worldToGrid inverse of origin + resolution', g.u === 2 && g.v === 4, JSON.stringify(g));
  check('row flip: occupied cell found at its world position', G.classify(G.cellValue(meta, px, g.u, g.v)) === 'OCCUPIED');
  check('outside map', G.evaluate(meta, px, 10, 10).reasons.includes('지도 경계 밖'));
}
// 2) origin yaw: gridToWorld (renderer helper) and worldToGrid must round-trip.
{
  const meta = { width: 40, height: 30, resolution: 0.05, origin_x: 1.2, origin_y: -0.7, origin_yaw: 0.6 };
  let ok = true;
  for (const [u, v] of [[0, 0], [5, 7], [39, 29], [12, 3]]) {
    const w = H.gridToWorld(meta, u + 0.5, v + 0.5);
    const g = G.worldToGrid(meta, w.x, w.y);
    if (g.u !== u || g.v !== v) ok = false;
  }
  check('origin_yaw round trip (gridToWorld <-> worldToGrid)', ok);
}
// 3) geofence
check('pointInPolygon inside', G.pointInPolygon(0.5, 0.5, [[0, 0], [1, 0], [1, 1], [0, 1]]));
check('pointInPolygon outside', !G.pointInPolygon(1.5, 0.5, [[0, 0], [1, 0], [1, 1], [0, 1]]));

// 4) real C103 map (PGM P5, values 0/205/254).
const pgmPath = path.join(here, '..', '..', 'local_data', 'raw_results', 'limo_runs', '20261119_auto_map_depth', 'map', 'map.pgm');
if (fs.existsSync(pgmPath)) {
  const buf = fs.readFileSync(pgmPath);
  const hdr = buf.toString('latin1', 0, 64).split(/\s+/);
  const width = +hdr[1], height = +hdr[2];
  const body = buf.subarray(buf.length - width * height);
  const meta = { width, height, resolution: 0.05, origin_x: -8.69, origin_y: -16.5, origin_yaw: 0 };
  // Robot start pose of that run (map frame origin) lies in a free aisle.
  const start = G.evaluate(meta, body, 0.0, 0.0);
  check('C103: run start pose is FREE', start.cell.class === 'FREE', JSON.stringify(start.cell));
  // Count classes via the same accessor to prove the image orientation and thresholds.
  let f = 0, o = 0, u = 0;
  for (let v = 0; v < height; v++) for (let c = 0; c < width; c++) {
    const k = G.classify(G.cellValue(meta, body, c, v)); if (k === 'FREE') f++; else if (k === 'OCCUPIED') o++; else u++;
  }
  check('C103: class counts preserved (free 70229 / occ 4575 / unknown 87564)', f === 70229 && o === 4575 && u === 87564, `${f}/${o}/${u}`);
  // Find a FREE cell adjacent to an occupied cell: must be rejected by the clearance check.
  let found = null;
  for (let v = 1; v < height - 1 && !found; v++) for (let c = 1; c < width - 1 && !found; c++) {
    if (G.cellValue(meta, body, c, v) >= 250 && G.cellValue(meta, body, c + 1, v) === 0) found = [c, v];
  }
  const w = H.gridToWorld(meta, found[0] + 0.5, found[1] + 0.5);
  const near = G.evaluate(meta, body, w.x, w.y);
  check('C103: free cell next to an obstacle is rejected by clearance', !near.ok && near.counts.occ > 0, near.reasons.join('; '));
  check('C103: unknown space rejected', !G.evaluate(meta, body, -8.0, -16.0).ok);
  const fence = [[-1, -1], [1, -1], [1, 1], [-1, 1]];
  check('C103: geofence applied', G.evaluate(meta, body, 0, 0, { geofence: fence }).reasons.every((r) => r !== '허용 영역(Geofence) 밖'));
} else {
  console.log('  [SKIP] C103 map not present');
}
console.log(`TOTAL ${pass}/${pass + fail} passed`);
process.exit(fail ? 1 : 0);
