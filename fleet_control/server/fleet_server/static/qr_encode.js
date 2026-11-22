'use strict';
/* Minimal QR Code encoder (ISO/IEC 18004): byte mode, error correction level M, versions 1-10 (up to 213 bytes, enough for a
   visitor link). No dependency, no network. Structure follows the well-known reference algorithm (function patterns, RS ECC with
   block interleaving, zig-zag placement, 8 masks with penalty scoring). Verified by decoding the output with OpenCV
   (tests/test_qr_encode.py). Exposes QrEncode.matrix(text) -> boolean[][] and QrEncode.svg(text, scale). */
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.QrEncode = api;
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  // version -> [raw codewords, blocks, ecc codewords per block] for level M
  const M = { 1: [26, 1, 10], 2: [44, 1, 16], 3: [70, 1, 26], 4: [100, 2, 18], 5: [134, 2, 24], 6: [172, 4, 16], 7: [196, 4, 18],
    8: [242, 4, 22], 9: [292, 5, 22], 10: [346, 5, 26] };
  const ALIGN = { 1: [], 2: [6, 18], 3: [6, 22], 4: [6, 26], 5: [6, 30], 6: [6, 34], 7: [6, 22, 38], 8: [6, 24, 42], 9: [6, 26, 46], 10: [6, 28, 50] };
  const bit = (x, i) => ((x >>> i) & 1) !== 0;
  function gfMul(x, y) { let z = 0; for (let i = 7; i >= 0; i--) { z = (z << 1) ^ ((z >>> 7) * 0x11D); z ^= ((y >>> i) & 1) * x; } return z & 0xFF; }
  function rsDivisor(deg) {
    const r = new Array(deg).fill(0); r[deg - 1] = 1; let root = 1;
    for (let i = 0; i < deg; i++) { for (let j = 0; j < deg; j++) { r[j] = gfMul(r[j], root); if (j + 1 < deg) r[j] ^= r[j + 1]; } root = gfMul(root, 0x02); }
    return r;
  }
  function rsRemainder(data, div) {
    const r = div.map(() => 0);
    for (const b of data) { const f = b ^ r.shift(); r.push(0); div.forEach((c, i) => { r[i] ^= gfMul(c, f); }); }
    return r;
  }
  function encodeData(bytes, ver) {
    const [raw, nb, ecc] = M[ver]; const cap = raw - nb * ecc; const bits = [];
    const put = (v, n) => { for (let i = n - 1; i >= 0; i--) bits.push((v >>> i) & 1); };
    put(4, 4); put(bytes.length, ver < 10 ? 8 : 16); bytes.forEach(b => put(b, 8));
    put(0, Math.min(4, cap * 8 - bits.length)); while (bits.length % 8) bits.push(0);
    const data = []; for (let i = 0; i < bits.length; i += 8) data.push(parseInt(bits.slice(i, i + 8).join(''), 2));
    for (let p = 0xEC; data.length < cap; p ^= 0xEC ^ 0x11) data.push(p);
    // blocks + ECC, interleaved
    const shortLen = Math.floor(raw / nb), nShort = nb - raw % nb, div = rsDivisor(ecc), blocks = [];
    for (let i = 0, k = 0; i < nb; i++) {
      const dat = data.slice(k, k + shortLen - ecc + (i < nShort ? 0 : 1)); k += dat.length;
      const e = rsRemainder(dat, div); if (i < nShort) dat.push(0); blocks.push(dat.concat(e));
    }
    const out = [];
    for (let i = 0; i < blocks[0].length; i++) blocks.forEach((b, j) => { if (i !== shortLen - ecc || j >= nShort) out.push(b[i]); });
    return out;
  }
  function build(ver, codewords, mask) {
    const n = ver * 4 + 17, mod = [], fn = [];
    for (let y = 0; y < n; y++) { mod.push(new Array(n).fill(false)); fn.push(new Array(n).fill(false)); }
    const set = (x, y, v) => { mod[y][x] = v; fn[y][x] = true; };
    for (let i = 0; i < n; i++) { set(6, i, i % 2 === 0); set(i, 6, i % 2 === 0); }
    const finder = (cx, cy) => { for (let dy = -4; dy <= 4; dy++) for (let dx = -4; dx <= 4; dx++) {
      const d = Math.max(Math.abs(dx), Math.abs(dy)), x = cx + dx, y = cy + dy;
      if (x >= 0 && x < n && y >= 0 && y < n) set(x, y, d !== 2 && d !== 4); } };
    finder(3, 3); finder(n - 4, 3); finder(3, n - 4);
    const al = ALIGN[ver], L = al.length;
    for (let i = 0; i < L; i++) for (let j = 0; j < L; j++) {
      if ((i === 0 && j === 0) || (i === 0 && j === L - 1) || (i === L - 1 && j === 0)) continue;
      for (let dy = -2; dy <= 2; dy++) for (let dx = -2; dx <= 2; dx++) set(al[i] + dx, al[j] + dy, Math.max(Math.abs(dx), Math.abs(dy)) !== 1);
    }
    const format = () => {                       // level M = 0b00
      const data = (0 << 3) | mask; let rem = data;
      for (let i = 0; i < 10; i++) rem = (rem << 1) ^ ((rem >>> 9) * 0x537);
      const b = ((data << 10) | rem) ^ 0x5412;
      for (let i = 0; i <= 5; i++) set(8, i, bit(b, i));
      set(8, 7, bit(b, 6)); set(8, 8, bit(b, 7)); set(7, 8, bit(b, 8));
      for (let i = 9; i < 15; i++) set(14 - i, 8, bit(b, i));
      for (let i = 0; i < 8; i++) set(n - 1 - i, 8, bit(b, i));
      for (let i = 8; i < 15; i++) set(8, n - 15 + i, bit(b, i));
      set(8, n - 8, true);
    };
    format();
    if (ver >= 7) {
      let rem = ver; for (let i = 0; i < 12; i++) rem = (rem << 1) ^ ((rem >>> 11) * 0x1F25);
      const b = (ver << 12) | rem;
      for (let i = 0; i < 18; i++) { const a = n - 11 + i % 3, c = Math.floor(i / 3); set(a, c, bit(b, i)); set(c, a, bit(b, i)); }
    }
    let i = 0;
    for (let right = n - 1; right >= 1; right -= 2) {
      if (right === 6) right = 5;
      for (let vert = 0; vert < n; vert++) for (let j = 0; j < 2; j++) {
        const x = right - j, up = ((right + 1) & 2) === 0, y = up ? n - 1 - vert : vert;
        if (!fn[y][x] && i < codewords.length * 8) { mod[y][x] = bit(codewords[i >>> 3], 7 - (i & 7)); i++; }
      }
    }
    const MASK = [(x, y) => (x + y) % 2 === 0, (x, y) => y % 2 === 0, x => x % 3 === 0, (x, y) => (x + y) % 3 === 0,
      (x, y) => (Math.floor(x / 3) + Math.floor(y / 2)) % 2 === 0, (x, y) => (x * y) % 2 + (x * y) % 3 === 0,
      (x, y) => ((x * y) % 2 + (x * y) % 3) % 2 === 0, (x, y) => ((x + y) % 2 + (x * y) % 3) % 2 === 0][mask];
    for (let y = 0; y < n; y++) for (let x = 0; x < n; x++) if (!fn[y][x] && MASK(x, y)) mod[y][x] = !mod[y][x];
    return mod;
  }
  function penalty(m) {
    const n = m.length; let p = 0, dark = 0;
    const runs = line => { let s = 0, c = 1; for (let i = 1; i <= n; i++) { if (i < n && line[i] === line[i - 1]) c++; else { if (c >= 5) s += 3 + c - 5; c = 1; } } return s; };
    for (let y = 0; y < n; y++) { p += runs(m[y]); p += runs(m.map(r => r[y])); }
    for (let y = 0; y < n - 1; y++) for (let x = 0; x < n - 1; x++) { const c = m[y][x]; if (c === m[y][x + 1] && c === m[y + 1][x] && c === m[y + 1][x + 1]) p += 3; }
    const pat = [true, false, true, true, true, false, true];
    const finderLike = (get) => { let s = 0; for (let a = 0; a < n; a++) for (let b = 0; b + 7 <= n; b++) {
      if (!pat.every((v, k) => get(a, b + k) === v)) continue;
      const before = b >= 4 && [1, 2, 3, 4].every(k => !get(a, b - k)), after = b + 11 <= n && [7, 8, 9, 10].every(k => !get(a, b + k));
      if (before || after) s += 40; } return s; };
    p += finderLike((r, c) => m[r][c]) + finderLike((c, r) => m[r][c]);
    m.forEach(r => r.forEach(v => { if (v) dark++; }));
    p += Math.floor(Math.abs(dark * 20 - n * n * 10) / (n * n)) * 10;
    return p;
  }
  function matrix(text) {
    const bytes = Array.from(new TextEncoder().encode(String(text)));
    let ver = 1;
    while (ver <= 10 && 4 + (ver < 10 ? 8 : 16) + bytes.length * 8 > (M[ver][0] - M[ver][1] * M[ver][2]) * 8) ver++;
    if (ver > 10) throw new Error('text too long for a version-10 QR code');
    const cw = encodeData(bytes, ver);
    let best = null, bestP = Infinity;
    for (let mask = 0; mask < 8; mask++) { const m = build(ver, cw, mask), p = penalty(m); if (p < bestP) { best = m; bestP = p; } }
    return best;
  }
  function svg(text, scale = 6) {
    const m = matrix(text), n = m.length, q = 4, size = (n + 2 * q) * scale; let d = '';
    m.forEach((row, y) => row.forEach((v, x) => { if (v) d += `M${(x + q) * scale},${(y + q) * scale}h${scale}v${scale}h-${scale}z`; }));
    return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${size} ${size}" width="${size}" height="${size}" role="img" aria-label="QR code"><rect width="100%" height="100%" fill="#fff"/><path d="${d}" fill="#000"/></svg>`;
  }
  return { matrix, svg };
});
