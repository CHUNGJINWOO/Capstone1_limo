'use strict';
// Drives the real dashboard Gate reset card (test-only, loopback Fleet). argv: url tokenFile outDir role click(0|1)
// Prints one JSON line: card text, button disabled, whether it clicked. Token is read from a file and never logged.
const {chromium} = require('playwright');
const fs = require('fs'); const path = require('path');
const [url, tokenFile, outDir, role, click] = process.argv.slice(2);
(async () => {
  const token = fs.readFileSync(tokenFile, 'utf8').trim();
  const browser = await chromium.launch({executablePath: '/usr/bin/google-chrome', headless: true, args: ['--no-sandbox']});
  try {
    const page = await (await browser.newContext({viewport: {width: 1366, height: 1100}})).newPage();
    await page.addInitScript(t => sessionStorage.setItem('fleet_token', t), token);
    page.on('dialog', d => d.accept());
    await page.goto(url, {waitUntil: 'domcontentloaded'});
    await page.locator('#navTabs [data-tab=control]').click({timeout: 15000});
    const card = page.locator('#gateResetCard');
    await card.waitFor({timeout: 20000});
    await page.waitForFunction(() => { const c = document.querySelector('#gateResetCard'); return c && /Gate 상태/.test(c.innerText) && !/미보고 *\n*현재/.test(''); }, null, {timeout: 20000});
    await page.waitForTimeout(2500);
    const btn = card.locator('[data-gr="reset"]');
    const out = {role, text: await card.innerText(), disabled: await btn.isDisabled(), clicked: false};
    await card.screenshot({path: path.join(outDir, `gate_reset_card_${role}.png`)});
    if (click === '1' && !out.disabled) { await btn.click(); out.clicked = true; await page.waitForTimeout(4000); out.after = await card.innerText();
      await card.screenshot({path: path.join(outDir, `gate_reset_card_${role}_after.png`)}); }
    console.log(JSON.stringify(out));
  } finally { await browser.close(); }
})().catch(e => { console.log(JSON.stringify({error: String(e)})); process.exit(1); });
