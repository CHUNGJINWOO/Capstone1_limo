'use strict';
// Drives the real dashboard against the isolated mode E2E stack: Lease via the UI, mode buttons on the operating-mode
// card, waits for the server-reported transition result, reads the status rows, screenshots desktop and 375 px.
// Token is read from a file and never logged.
const {chromium} = require('playwright');
const fs = require('fs'); const path = require('path');
const [url, tokenFile, outDir] = process.argv.slice(2);
const mark = (name, extra = {}) => console.log(JSON.stringify({mark: name, t: Date.now() / 1000, ...extra}));
(async () => {
  const token = fs.readFileSync(tokenFile, 'utf8').trim();
  const browser = await chromium.launch({executablePath: '/usr/bin/google-chrome', headless: true, args: ['--no-sandbox']});
  try {
    const page = await browser.newPage({viewport: {width: 1440, height: 1000}});
    const errors = []; page.on('pageerror', e => errors.push(e.message));
    await page.addInitScript(t => sessionStorage.setItem('fleet_token', t), token);
    await page.goto(url, {waitUntil: 'domcontentloaded'});
    await page.locator('#navTabs [data-tab=control]').click({timeout: 10000});
    const card = page.locator('#operatingModeCard');
    await card.waitFor({state: 'visible', timeout: 10000});
    await page.fill('#leaseTtl', '120'); await page.click('#leaseAcquire');
    await page.waitForFunction(() => !document.querySelector('[data-om-mode="MANUAL"]').disabled, null, {timeout: 15000});
    const rows = async () => page.$$eval('#operatingModeCard dl div', ds => Object.fromEntries(ds.map(d => [d.querySelector('dt').textContent, d.querySelector('dd').textContent])));
    const go = async target => {
      const t0 = Date.now();
      await page.click(`[data-om-mode="${target}"]`);
      await page.waitForFunction(tg => {
        const s = document.querySelector('#operatingModeCard [data-om="steps"]').textContent;
        return s.includes(`→ ${tg}`) && (s.includes('ACTIVE') || s.includes('FAILED'));
      }, target, {timeout: 45000});
      const steps = await page.locator('#operatingModeCard [data-om="steps"]').textContent();
      const badge = await page.locator('#operatingModeCard [data-om="badge"]').textContent();
      mark('mode_' + target, {ms: Date.now() - t0, badge, steps: steps.replace(/\s+/g, ' ').slice(0, 200), rows: await rows()});
    };
    await go('MANUAL');
    const manualBlock = await page.locator('[data-mc="block"]').textContent();
    mark('manual_panel', {block: manualBlock});
    await go('MAPPING');
    mark('manual_panel_in_mapping', {block: await page.locator('[data-mc="block"]').textContent()});
    await page.screenshot({path: path.join(outDir, 'mode_card_desktop.png'), fullPage: false});
    await card.screenshot({path: path.join(outDir, 'mode_card_only.png')});
    await go('IDLE');
    await page.setViewportSize({width: 375, height: 812}); await page.waitForTimeout(500);
    await card.scrollIntoViewIfNeeded();
    const overflow = await page.evaluate(() => ({doc: document.documentElement.scrollWidth, vw: window.innerWidth}));
    await page.screenshot({path: path.join(outDir, 'mode_card_mobile.png'), fullPage: false});
    mark('mobile', overflow);
    mark('done', {errors});
  } finally { await browser.close(); }
})().catch(e => { mark('error', {message: e.message}); process.exitCode = 1; });
