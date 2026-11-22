'use strict';
// Drives the real dashboard manual panel against an ephemeral loopback Fleet server.
// Prints one JSON line per phase mark (wall-clock seconds) for the Python harness to correlate with
// /cmd_vel_locked. The Gamepad is a synthetic navigator.getGamepads() object (no physical device).
// Token is read from a file and never logged.
const {chromium} = require('playwright');
const fs = require('fs');
const path = require('path');
const [url, tokenFile, outDir] = process.argv.slice(2);
const mark = (name, extra = {}) => console.log(JSON.stringify({mark: name, t: Date.now() / 1000, ...extra}));
(async () => {
  const token = fs.readFileSync(tokenFile, 'utf8').trim();
  const browser = await chromium.launch({executablePath: '/usr/bin/google-chrome', headless: true, args: ['--no-sandbox']});
  try {
    const page = await browser.newPage({viewport: {width: 1280, height: 900}});
    page.on('pageerror', e => mark('pageerror', {message: e.message}));
    await page.addInitScript(t => {
      sessionStorage.setItem('fleet_token', t);
      const pad = {index: 0, id: 'synthetic-pad', connected: true, timestamp: 1, mapping: 'standard',
        axes: [0, 0, 0, 0], buttons: Array.from({length: 12}, () => ({pressed: false, touched: false, value: 0}))};
      window.__pad = pad;
      window.__padLive = false;
      Object.defineProperty(Navigator.prototype, 'getGamepads', {value: () => [window.__pad], configurable: true});
      setInterval(() => { if (window.__padLive) pad.timestamp = performance.now(); }, 30);
    }, token);
    await page.goto(url, {waitUntil: 'domcontentloaded'});
    await page.locator('#navTabs [data-tab=control]').click({timeout: 10000});
    const start = page.locator('[data-mc=start]');
    const status = () => page.locator('[data-mc=status]').textContent();
    const block = () => page.locator('[data-mc=block]').textContent();
    const enabled = () => page.waitForFunction(() => !document.querySelector('[data-mc=start]').disabled, null, {timeout: 20000});
    const ready = () => page.waitForFunction(() => document.querySelector('[data-mc=status]').textContent.includes('준비'), null, {timeout: 5000});
    await start.waitFor({state: 'visible', timeout: 10000});
    await enabled();
    mark('panel_ready', {block: await block()});

    // Keyboard: W + A with Space as Deadman.
    await page.locator('[data-mc=source]').selectOption('keyboard');
    await enabled(); await start.click(); await ready();
    await page.keyboard.down('KeyW'); await page.keyboard.down('KeyA'); await page.keyboard.down('Space');
    mark('kb_on'); await page.waitForTimeout(900);
    await page.keyboard.up('Space'); mark('kb_off', {status: await status()});
    await page.waitForTimeout(700);
    await page.keyboard.up('KeyA'); await page.keyboard.up('KeyW');

    // Pointer / touch joystick: press up-left of centre, hold, release.
    await page.locator('[data-mc=source]').selectOption('pointer');
    await enabled(); await start.click(); await ready();
    const box = await page.locator('[data-mc=stick]').boundingBox();
    const cx = box.x + box.width / 2, cy = box.y + box.height / 2;
    await page.mouse.move(cx - 0.4 * box.width / 2, cy - 0.8 * box.height / 2);
    await page.mouse.down(); mark('ptr_on'); await page.waitForTimeout(900);
    await page.mouse.up(); mark('ptr_off', {status: await status()});
    await page.waitForTimeout(700);

    // Gamepad: left stick forward + slight right, L1 (button 4) as Deadman; timestamps keep updating.
    await page.locator('[data-mc=source]').selectOption('gamepad');
    await page.evaluate(() => { window.__padLive = true; });
    await enabled(); await start.click(); await ready();
    await page.evaluate(() => { const p = window.__pad; p.axes[0] = 0.5; p.axes[1] = -1; p.buttons[4] = {pressed: true, touched: true, value: 1}; });
    mark('pad_on'); await page.waitForTimeout(900);
    await page.evaluate(() => { window.__pad.buttons[4] = {pressed: false, touched: false, value: 0}; });
    mark('pad_off'); await page.waitForTimeout(300); mark('pad_status', {status: await status()});
    await page.waitForTimeout(400);

    // Gamepad freeze: timestamp stops updating while Deadman is held -> the panel must stop the session.
    await enabled(); await page.evaluate(() => { const p = window.__pad; p.axes[0] = 0; p.axes[1] = 0; });
    await start.click(); await ready();
    await page.evaluate(() => { const p = window.__pad; p.axes[1] = -1; p.buttons[4] = {pressed: true, touched: true, value: 1}; });
    mark('padfreeze_on'); await page.waitForTimeout(600);
    await page.evaluate(() => { window.__padLive = false; });
    mark('padfreeze_frozen'); await page.waitForTimeout(800);
    mark('padfreeze_status', {status: await status()});
    await page.evaluate(() => { window.__pad.buttons[4] = {pressed: false, touched: false, value: 0}; window.__padLive = true; });

    await page.screenshot({path: path.join(outDir, 'manual_panel_desktop.png'), fullPage: false});
    // Phone width: the panel must fit without horizontal page scroll.
    await page.setViewportSize({width: 375, height: 812});
    await page.waitForTimeout(400);
    const overflow = await page.evaluate(() => {
      const panel = document.querySelector('.manual-control');
      const r = panel ? panel.getBoundingClientRect() : null;
      return {docScroll: document.documentElement.scrollWidth, viewport: window.innerWidth,
              panelLeft: r && r.left, panelRight: r && r.right};
    });
    await page.locator('.manual-control').scrollIntoViewIfNeeded();
    await page.screenshot({path: path.join(outDir, 'manual_panel_mobile.png'), fullPage: false});
    mark('mobile_layout', overflow);
    mark('done', {block: await block()});
  } finally {
    await browser.close();
  }
})().catch(e => { mark('error', {message: e.message}); process.exitCode = 1; });
