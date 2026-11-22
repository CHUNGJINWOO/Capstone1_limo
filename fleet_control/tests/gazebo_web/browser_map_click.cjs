'use strict';
// Real dashboard, real map click (test-only, 2026-11-21). For each target {name, x, y, expect}:
//   hover three canvas points and read the UI's own cursor readout ("커서: x … · y … m") -> solve screen->world (affine)
//   -> click the screen point of (x, y) on the live map -> goal drawer opens with mx/my -> read the review / pre-check text
//   -> tick "확인" and press "임무 생성" when enabled -> read the error line, then for expect=submit wait until the mission
//   panel shows a terminal status. No page internals are touched: only what an operator sees and clicks.
// usage: node browser_map_click.cjs URL TOKEN_FILE TARGETS_JSON   (one JSON line per event on stdout, wall clock)
const {chromium} = require('playwright');
const fs = require('fs');
const [url, tokenFile, targetsJson] = process.argv.slice(2);
const targets = JSON.parse(targetsJson);
const out = (kind, extra = {}) => console.log(JSON.stringify({kind, t: Date.now() / 1000, ...extra}));
let lastMission = null;
(async () => {
  const token = fs.readFileSync(tokenFile, 'utf8').trim();
  const browser = await chromium.launch({executablePath: '/usr/bin/google-chrome', headless: true, args: ['--no-sandbox']});
  try {
    const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
    page.on('pageerror', e => out('pageerror', {message: e.message}));
    page.on('response', async r => {
      const path = new URL(r.url()).pathname;
      if (path.endsWith('/missions') && r.request().method() === 'POST') {
        let body = null; try { body = await r.json(); } catch (_) {}
        out('mission_post', {status: r.status(), mission_id: body && body.mission_id, error: body && body.error});
        if (body && body.mission_id) lastMission = body.mission_id;
      }
    });
    await page.addInitScript(t => sessionStorage.setItem('fleet_token', t), token);
    await page.goto(url, {waitUntil: 'domcontentloaded'});
    await page.locator('#navTabs [data-tab=control]').click({timeout: 10000});
    const cv = page.locator('#map');
    await cv.waitFor({state: 'visible', timeout: 15000});
    await page.waitForTimeout(4000);                                   // live map image + robot state
    const cursor = async (sx, sy) => {
      await page.mouse.move(sx, sy); await page.waitForTimeout(120);
      const txt = await page.locator('#mapCursor').textContent();
      const m = /x\s*(-?[\d.]+)\s*·\s*y\s*(-?[\d.]+)/.exec(txt || '');
      return m ? [Number(m[1]), Number(m[2])] : null;
    };
    const closeDrawer = async () => {
      if (await page.locator('#goalDrawerBackdrop').isHidden()) return;
      await page.locator('#goalDrawerClose').click(); await page.waitForTimeout(300);
    };
    // expand {candidates:[[x,y],...]}: an operator tries spots until the dashboard's own pre-check accepts one
    const expanded = [];
    for (const tg of targets) {
      if (Array.isArray(tg.candidates)) tg.candidates.forEach((c, i) => expanded.push({...tg, x: c[0], y: c[1], try: i, group: tg.name}));
      else expanded.push(tg);
    }
    const accepted = new Set();
    for (const tg of expanded) {
      if (tg.group && accepted.has(tg.group)) continue;
      await closeDrawer();
      const box = await cv.boundingBox();
      const ax = box.x + box.width * 0.35, ay = box.y + box.height * 0.35, d = Math.min(box.width, box.height) * 0.3;
      const p0 = await cursor(ax, ay), p1 = await cursor(ax + d, ay), p2 = await cursor(ax, ay + d);
      if (!p0 || !p1 || !p2) { out('target', {name: tg.name, error: 'cursor readout unavailable'}); continue; }
      // world = p0 + A * (screen - s0); A columns from the two probes
      const a11 = (p1[0] - p0[0]) / d, a21 = (p1[1] - p0[1]) / d, a12 = (p2[0] - p0[0]) / d, a22 = (p2[1] - p0[1]) / d;
      const det = a11 * a22 - a12 * a21;
      const wx = tg.x - p0[0], wy = tg.y - p0[1];
      const sx = ax + (a22 * wx - a12 * wy) / det, sy = ay + (-a21 * wx + a11 * wy) / det;
      const inside = sx > box.x + 2 && sx < box.x + box.width - 2 && sy > box.y + 2 && sy < box.y + box.height - 2;
      if (!inside) { out('target', {name: tg.name, error: 'target outside the visible canvas', screen: [sx, sy]}); continue; }
      const seen = await cursor(sx, sy);
      await page.mouse.down(); await page.mouse.up();
      await page.waitForTimeout(500);
      const drawerOpen = !(await page.locator('#goalDrawerBackdrop').isHidden());
      const mx = await page.locator('#mx').inputValue(), my = await page.locator('#my').inputValue();
      const confirm = page.locator('#goalConfirmed');
      if (drawerOpen) { await confirm.check().catch(() => {}); await page.waitForTimeout(300); }
      const review = await page.locator('#goalReview').textContent();
      const submitEnabled = drawerOpen && await page.locator('#mcreate').isEnabled();
      let merr = '', submitted = false;
      const doSubmit = submitEnabled && (tg.expect !== 'submit' || !tg.group || true);
      if (tg.group && !submitEnabled && tg.expect === 'submit') {   // pre-check refused this spot: record it and try the next one
        out('try', {name: tg.name, try: tg.try, want: [tg.x, tg.y], review}); continue;
      }
      if (doSubmit) { await page.locator('#mcreate').click(); submitted = true; if (tg.group) accepted.add(tg.group); await page.waitForTimeout(1500); }
      merr = (await page.locator('#merr').textContent()) || '';
      out('target', {name: tg.name, expect: tg.expect, want: [tg.x, tg.y], cursor_at_click: seen, drawer_open: drawerOpen, mx, my,
                     review, submit_enabled: submitEnabled, submitted, merr});
      if (tg.expect === 'submit' && submitted && !merr) {
        const end = Date.now() + (tg.timeout_s || 150) * 1000; let last = '';
        // the active-mission panel empties when the mission ends; the mission table row keeps the terminal status
        let active = '';
        while (Date.now() < end) {
          active = (await page.locator('#operationMission').textContent()) || active;
          // the newest row of the dashboard's own mission table (or the row of the captured id); the POST response body can arrive late
          last = await page.evaluate(id => { const rows = [...document.querySelectorAll('#missions tbody tr')];
                                             const row = (id && rows.find(r => r.textContent.includes(id))) || rows[0]; return row ? row.textContent : ''; }, lastMission);
          if (/SUCCEEDED|FAILED|CANCELED|ABORTED|REJECTED|RECONCILIATION_REQUIRED/.test(last)) break;
          await page.waitForTimeout(1000);
        }
        out('mission_ui', {name: tg.name, mission_id: lastMission, text: last.slice(0, 400), active_panel_last: active.slice(0, 200)});
      }
      await closeDrawer();
    }
    out('end');
  } finally { await browser.close(); }
})().catch(e => { out('error', {message: String(e && e.stack || e)}); process.exit(1); });
