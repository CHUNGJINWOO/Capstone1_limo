'use strict';
// Phase 4 (test-only): the real dashboard shows the readiness card; a page reload (operator reconnect) shows the same level.
// usage: node browser_system_panel.cjs URL TOKEN_FILE OUT_DIR   -> last stdout line: JSON {level, services, reasons, reload_same}
const {chromium} = require('playwright');
const fs = require('fs'), path = require('path');
const [url, tokenFile, outDir] = process.argv.slice(2);
(async () => {
  const token = fs.readFileSync(tokenFile, 'utf8').trim();
  const browser = await chromium.launch({executablePath: '/usr/bin/google-chrome', headless: true, args: ['--no-sandbox']});
  try {
    const page = await browser.newPage({viewport: {width: 1280, height: 1000}});
    const errors = []; page.on('pageerror', e => errors.push(e.message));
    await page.addInitScript(t => sessionStorage.setItem('fleet_token', t), token);
    const read = async () => {
      await page.goto(url, {waitUntil: 'domcontentloaded'});
      await page.locator('#navTabs [data-tab=control]').click({timeout: 10000});
      const lv = page.locator('#systemReadinessCard [data-sp=level]');
      await page.waitForFunction(() => { const e = document.querySelector('#systemReadinessCard [data-sp=level]'); return e && e.dataset.level; }, null, {timeout: 15000});
      await lv.scrollIntoViewIfNeeded();
      return {level: await lv.getAttribute('data-level'), services: await page.locator('#systemReadinessCard tr[data-service]').count(),
              reasons: await page.locator('#systemReadinessCard [data-sp=reasons] li[data-code]').evaluateAll(es => es.map(e => e.dataset.code)),
              buttons: await page.locator('#systemReadinessCard button[data-act]').count()};
    };
    const a = await read();
    await page.locator('#systemReadinessCard').screenshot({path: path.join(outDir, 'system_card.png')});
    const b = await read();
    console.log(JSON.stringify({...a, reload_same: a.level === b.level && a.services === b.services, after_reload: b, errors}));
  } finally { await browser.close(); }
})().catch(e => { console.error(String(e && e.stack || e)); process.exit(1); });
