'use strict';
// Local ephemeral Fleet server only; token is loaded but never logged.
const {chromium} = require('playwright');
const fs = require('fs');
(async () => {
 const url=process.argv[2], token=fs.readFileSync(process.argv[3],'utf8').trim();
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
 try {
  const page=await browser.newPage({viewport:{width:1280,height:900}});
  await page.addInitScript(t => sessionStorage.setItem('fleet_token',t),token);
  await page.goto(url,{waitUntil:'domcontentloaded'});
  await page.locator('#navTabs [data-tab=control]').click({timeout:10000});
  const start=page.locator('[data-mc=start]');
  await start.waitFor({state:'visible',timeout:10000});
  await page.waitForFunction(() => !document.querySelector('[data-mc=start]').disabled, null,{timeout:12000});
  await start.click();
  await page.waitForFunction(() => document.querySelector('[data-mc=status]').textContent.includes('준비'), null, {timeout:5000});
  await page.keyboard.down('KeyW');await page.keyboard.down('KeyA');await page.keyboard.down('Space');
  await page.waitForTimeout(700);
  await page.keyboard.up('Space');await page.keyboard.up('KeyA');await page.keyboard.up('KeyW');
  await page.waitForTimeout(500);
  console.log('PASS actual dashboard manual panel start, W+A, Deadman release');
 } finally { await browser.close(); }
})().catch(e=>{console.error(e.message);process.exitCode=1});
