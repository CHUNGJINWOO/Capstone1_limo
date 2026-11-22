'use strict';
/* Client-side NavigateToPose goal pre-check against the displayed OccupancyGrid image.
   Image encoding (agent pngmap.py and registry preview): free=255, occupied=0, unknown=other; image row 0 = top = +y.
   This is an operator aid only: the server and the robot (Safety Context, localization, Nav2 costmaps) stay authoritative. */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.GoalCheck = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  const ROBOT_RADIUS_M = 0.22;      // Nav2 robot_radius in the deployed configs
  const MARGIN_M = 0.10;

  /** world (map frame) -> grid cell (u = column from origin, v = row from origin, +v = +y), honours origin yaw. */
  function worldToGrid(meta, x, y) {
    const ox = Number(meta.origin_x || 0), oy = Number(meta.origin_y || 0), yaw = Number(meta.origin_yaw || 0);
    const res = Number(meta.resolution);
    const dx = x - ox, dy = y - oy, c = Math.cos(yaw), s = Math.sin(yaw);
    return { u: Math.floor((c * dx + s * dy) / res), v: Math.floor((-s * dx + c * dy) / res) };
  }

  /** pixels: Uint8 grey values, row-major, image row 0 = top (= grid row height-1). */
  function cellValue(meta, pixels, u, v) {
    if (u < 0 || v < 0 || u >= meta.width || v >= meta.height) return null;
    return pixels[(meta.height - 1 - v) * meta.width + u];
  }

  function classify(value) {
    if (value === null || value === undefined) return 'OUT_OF_MAP';
    if (value >= 250) return 'FREE';
    if (value === 0) return 'OCCUPIED';
    return 'UNKNOWN';
  }

  function evaluate(meta, pixels, x, y, opts = {}) {
    const reasons = [];
    if (!meta || !pixels || !(Number(meta.resolution) > 0)) return { ok: false, reasons: ['지도 미수신'], cell: null };
    if (!Number.isFinite(x) || !Number.isFinite(y)) return { ok: false, reasons: ['좌표가 숫자가 아님'], cell: null };
    const g = worldToGrid(meta, x, y);
    const cls = classify(cellValue(meta, pixels, g.u, g.v));
    if (cls !== 'FREE') reasons.push({ OUT_OF_MAP: '지도 경계 밖', OCCUPIED: '점유 셀', UNKNOWN: '미관측(UNKNOWN) 셀' }[cls]);
    const clearance = (opts.robotRadius ?? ROBOT_RADIUS_M) + (opts.margin ?? MARGIN_M);
    const r = Math.ceil(clearance / meta.resolution);
    let occ = 0, unk = 0, out = 0;
    if (cls === 'FREE') {
      for (let dv = -r; dv <= r; dv++) for (let du = -r; du <= r; du++) {
        if (du * du + dv * dv > r * r) continue;
        const c = classify(cellValue(meta, pixels, g.u + du, g.v + dv));
        if (c === 'OCCUPIED') occ++; else if (c === 'UNKNOWN') unk++; else if (c === 'OUT_OF_MAP') out++;
      }
      if (occ) reasons.push(`반경 ${clearance.toFixed(2)} m 안에 점유 셀 ${occ}개(차체 0.22 m + 여유 ${(clearance - 0.22).toFixed(2)} m)`);
      if (unk) reasons.push(`반경 ${clearance.toFixed(2)} m 안에 미관측 셀 ${unk}개`);
      if (out) reasons.push('반경 안이 지도 경계를 벗어남');
    }
    if (opts.geofence && opts.geofence.length >= 3 && !pointInPolygon(x, y, opts.geofence)) reasons.push('허용 영역(Geofence) 밖');
    return { ok: reasons.length === 0, reasons, cell: { u: g.u, v: g.v, class: cls }, clearance_m: clearance, counts: { occ, unk, out } };
  }

  function pointInPolygon(x, y, poly) {
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const [xi, yi] = poly[i], [xj, yj] = poly[j];
      if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
    }
    return inside;
  }

  /** Browser only: decode an <img>/ImageBitmap into grey pixels. */
  function pixelsFromImage(img, width, height) {
    const cv = document.createElement('canvas');
    cv.width = width; cv.height = height;
    const ctx = cv.getContext('2d', { willReadFrequently: true });
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(img, 0, 0, width, height);
    const rgba = ctx.getImageData(0, 0, width, height).data;
    const out = new Uint8Array(width * height);
    for (let i = 0; i < out.length; i++) out[i] = rgba[i * 4];
    return out;
  }

  return { worldToGrid, cellValue, classify, evaluate, pointInPolygon, pixelsFromImage, ROBOT_RADIUS_M, MARGIN_M };
});
