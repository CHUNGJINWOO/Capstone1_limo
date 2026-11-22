'use strict';
// DIAGNOSTIC (test-only, 2026-11-21): repeated pointer-joystick cycles on the REAL dashboard manual panel, with timestamps of
// every step the input passes in the browser: pointer events on the stick (observed with capture-phase listeners, the panel's
// own handlers are untouched), panel status text changes, every manual-bench request/response (action, seq, deadman, latency,
// HTTP status) and every /dashboard poll (the panel's safety-evidence source). One JSON line per event on stdout; wall clock
// (Date.now()/1000) so the Python harness can join it with Fleet/Agent/Gate/Gazebo records.
// usage: node browser_pointer_diag.cjs URL TOKEN_FILE CYCLES HOLD_MS GAP_MS [extra|none]
//   extra = also exercise pointercancel (synthetic), pointer leaving the stick while held, and STOP button while held.
const {chromium} = require('playwright');
const fs = require('fs');
const [url, tokenFile, cyclesArg, holdArg, gapArg, extraArg] = process.argv.slice(2);
const CYCLES = Number(cyclesArg || 10), HOLD = Number(holdArg || 1500), GAP = Number(gapArg || 800);
const out = (kind, extra = {}) => console.log(JSON.stringify({kind, t: Date.now() / 1000, ...extra}));
(async () => {
  const token = fs.readFileSync(tokenFile, 'utf8').trim();
  const browser = await chromium.launch({executablePath: '/usr/bin/google-chrome', headless: true, args: ['--no-sandbox']});
  try {
    const page = await browser.newPage({viewport: {width: 1280, height: 900}});
    page.on('pageerror', e => out('pageerror', {message: e.message}));
    const pending = new Map();
    page.on('request', r => {
      const u = r.url();
      if (u.includes('/manual-bench')) {
        let b = {}; try { b = JSON.parse(r.postData() || '{}'); } catch (_) {}
        pending.set(r, Date.now() / 1000);
        out('bench_req', {action: b.action, seq: b.seq, deadman: b.deadman, linear: b.linear, turn: b.turn});
      } else if (u.endsWith('/api/v1/dashboard')) { pending.set(r, Date.now() / 1000); out('dash_req'); }
    });
    const done = async (r, failed) => {
      if (!pending.has(r)) return;
      const t0 = pending.get(r); pending.delete(r);
      const resp = failed ? null : await r.response();
      const kind = r.url().includes('/manual-bench') ? 'bench_resp' : 'dash_resp';
      let b = {}; try { b = JSON.parse(r.postData() || '{}'); } catch (_) {}
      out(kind, {action: b.action, seq: b.seq, status: resp ? resp.status() : null, failed: failed ? (r.failure() || {}).errorText : null,
                 latency_ms: Math.round((Date.now() / 1000 - t0) * 1000)});
    };
    page.on('requestfinished', r => done(r, false)); page.on('requestfailed', r => done(r, true));
    await page.exposeFunction('__diag', (kind, extra) => out(kind, extra || {}));
    await page.addInitScript(t => sessionStorage.setItem('fleet_token', t), token);
    await page.goto(url, {waitUntil: 'domcontentloaded'});
    await page.locator('#navTabs [data-tab=control]').click({timeout: 10000});
    const start = page.locator('[data-mc=start]');
    await start.waitFor({state: 'visible', timeout: 10000});
    // observation only: capture-phase listeners + status MutationObserver (the panel's handlers run unchanged)
    await page.evaluate(() => {
      const stick = document.querySelector('[data-mc=stick]'), st = document.querySelector('[data-mc=status]');
      for (const type of ['pointerdown', 'pointerup', 'pointercancel', 'gotpointercapture', 'lostpointercapture', 'pointerleave'])
        stick.addEventListener(type, e => window.__diag('ptr_' + type, {id: e.pointerId, primary: e.isPrimary, buttons: e.buttons}), {capture: true, passive: true});
      new MutationObserver(() => window.__diag('status', {text: st.textContent})).observe(st, {childList: true, characterData: true, subtree: true});
      window.addEventListener('blur', () => window.__diag('window_blur'));
    });
    await page.locator('[data-mc=source]').selectOption('pointer');
    const enabled = () => page.waitForFunction(() => !document.querySelector('[data-mc=start]').disabled, null, {timeout: 20000});
    const ready = () => page.waitForFunction(() => document.querySelector('[data-mc=status]').textContent.includes('준비'), null, {timeout: 5000});
    let box = await page.locator('[data-mc=stick]').boundingBox();
    let cx = box.x + box.width / 2, cy = box.y + box.height / 2;
    const hit = (x, y) => page.evaluate(([x, y]) => { const e = document.elementFromPoint(x, y); return e ? (e.getAttribute('data-mc') || e.tagName) : null; }, [x, y]);
    for (let i = 0; i < CYCLES; i++) {
      try { await enabled(); } catch (_) { out('mark', {mark: 'start_disabled', cycle: i}); await page.waitForTimeout(GAP); continue; }
      await start.click(); try { await ready(); } catch (_) { out('mark', {mark: 'not_ready', cycle: i}); }
      const before = {x: Math.round(cx), y: Math.round(cy)};
      await page.locator('[data-mc=stick]').scrollIntoViewIfNeeded();         // as an operator would: the stick must be on screen
      box = await page.locator('[data-mc=stick]').boundingBox(); cx = box.x + box.width / 2; cy = box.y + box.height / 2;   // layout after Start
      out('mark', {mark: 'stick_box', cycle: i, box: [Math.round(box.x), Math.round(box.y), Math.round(box.width), Math.round(box.height)], centre_before: before,
                   hit: await hit(cx, cy - 0.8 * box.height / 2)});
      await page.mouse.move(cx, cy - 0.8 * box.height / 2);
      await page.mouse.down(); out('mark', {mark: 'down', cycle: i});
      await page.waitForTimeout(HOLD / 2);
      await page.mouse.move(cx - 0.3 * box.width / 2, cy - 0.7 * box.height / 2, {steps: 4}); out('mark', {mark: 'moved', cycle: i});
      await page.waitForTimeout(HOLD / 2);
      await page.mouse.up(); out('mark', {mark: 'up', cycle: i});
      await page.waitForTimeout(GAP);
    }
    if (extraArg === 'extra') {
      box = await page.locator('[data-mc=stick]').boundingBox(); cx = box.x + box.width / 2; cy = box.y + box.height / 2;
      // pointer leaves the stick area while held (capture must keep it), then up outside
      await enabled(); await start.click(); await ready().catch(() => {});
      await page.mouse.move(cx, cy - 0.5 * box.height / 2); await page.mouse.down(); out('mark', {mark: 'leave_down'});
      await page.mouse.move(cx, cy - 2.5 * box.height, {steps: 6}); out('mark', {mark: 'leave_outside'}); await page.waitForTimeout(800);
      await page.mouse.up(); out('mark', {mark: 'leave_up'}); await page.waitForTimeout(GAP);
      // STOP button while the pointer is held (touch-style second input is not possible with one mouse: STOP via keyboard focus)
      await enabled(); await start.click(); await ready().catch(() => {});
      await page.mouse.move(cx, cy - 0.5 * box.height / 2); await page.mouse.down(); out('mark', {mark: 'stopbtn_down'}); await page.waitForTimeout(700);
      await page.evaluate(() => document.querySelector('[data-mc=stop]').click()); out('mark', {mark: 'stopbtn_click'}); await page.waitForTimeout(800);
      await page.mouse.up(); out('mark', {mark: 'stopbtn_up'}); await page.waitForTimeout(GAP);
      // synthetic pointercancel while held (touch cancel)
      await enabled(); await start.click(); await ready().catch(() => {});
      await page.mouse.move(cx, cy - 0.5 * box.height / 2); await page.mouse.down(); out('mark', {mark: 'cancel_down'}); await page.waitForTimeout(700);
      await page.evaluate(() => { const s = document.querySelector('[data-mc=stick]');
        s.dispatchEvent(new PointerEvent('pointercancel', {pointerId: 1, isPrimary: true, bubbles: true})); });
      out('mark', {mark: 'cancel_sent'}); await page.waitForTimeout(800); await page.mouse.up(); out('mark', {mark: 'cancel_up'});
    }
    out('mark', {mark: 'end'});
  } finally { await browser.close(); }
})().catch(e => { out('error', {message: String(e && e.stack || e)}); process.exit(1); });
