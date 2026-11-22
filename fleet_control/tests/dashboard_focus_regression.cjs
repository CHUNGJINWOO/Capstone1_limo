// Run against dashboard_preview.py only; info file proves the isolated fixture URL.
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const { chromium } = require('playwright');
(async () => {
 const info = JSON.parse(fs.readFileSync(process.argv[2], 'utf8'));
 assert.match(info.url, /^http:\/\/127\.0\.0\.1:\d+$/);
 assert.match(info.runtime, /\/limo-dashboard-ui-/);
 const output = process.argv[3] || '/tmp';
 const browser = await chromium.launch({executablePath:process.env.CHROME_BIN || '/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
 const page = await browser.newPage({viewport:{width:1920,height:1080}, timezoneId:'Asia/Seoul'});
 const errors=[]; page.on('pageerror',e=>errors.push(e.message));
 page.on('console',m=>{ if(m.type()==='error') errors.push(m.text()); });
 await page.goto(info.url); await page.locator('#tok').fill(fs.readFileSync(path.join(info.token_dir,'operator.token'),'utf8').trim()); await page.locator('#go').click();
 await page.locator('#mapRobot option').first().waitFor({state:'attached'});
 await page.waitForTimeout(1500);
 const checks=[];
 async function check(name, fn){ await fn(); checks.push(name);console.log('PASS',name); }
 async function shot(name){ await page.screenshot({path:path.join(output,`limo-ui-${name}.png`),fullPage:true}); }
 async function overview(){await page.locator('[data-tab=overview]').click();}
 async function scenario(name){fs.writeFileSync(info.scenario,name); await page.waitForTimeout(name==='offline'?4500:1600);}
 await check('authenticated assets and console', async()=>assert.deepEqual(errors,[]));
 for(const [w,h] of [[1920,1080],[1600,900],[1366,768],[390,844]]){
  await page.setViewportSize({width:w,height:h}); await page.waitForTimeout(250); await page.locator('#mapFit').click();
  await check(`overflow ${w}x${h}`,async()=>{
   const dim=await page.evaluate(()=>({w:document.documentElement.scrollWidth,h:document.documentElement.scrollHeight,vw:innerWidth,vh:innerHeight}));
   assert.ok(dim.w<=dim.vw,JSON.stringify(dim)); if(w>1100) assert.ok(dim.h<=dim.vh,JSON.stringify(dim));
  }); await shot(`${w}x${h}`);
 }
 await page.setViewportSize({width:1366,height:768});
 for (const [w,h] of [[1536,864],[1280,720],[1024,768]]) {
  await page.setViewportSize({width:w,height:h});
  await check(`effective viewport / zoom equivalent ${w}x${h}`, async()=>{assert.ok(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));});
 }
 await page.setViewportSize({width:1366,height:768});
 await check('zoom and fit',async()=>{await page.locator('#mapSettingsToggle').click();const before=await page.locator('#mapScale').textContent();await page.locator('#mapZoomIn').click();await page.waitForTimeout(100);assert.notEqual(await page.locator('#mapScale').textContent(),before);await page.locator('#mapZoomOut').click();await page.locator('#mapSettingsToggle').click();await page.locator('#mapFit').click();});
 await check('robot selection',async()=>{await page.locator('#mapRobot').selectOption('UI-FIXTURE-02');assert.equal(await page.locator('#mrobot').inputValue(),'UI-FIXTURE-02');await page.locator('#mapRobot').selectOption('UI-FIXTURE-01');});
 await check('sensor tabs expose unavailable honestly',async()=>{await page.locator('[data-tab=media]').click();await page.locator('[data-sensor=depth]').click();assert.match(await page.locator('#sensorUnavailable').textContent(),/UNAVAILABLE/);await page.locator('[data-sensor=lidar]').click();await page.locator('[data-sensor=rgb]').click();await overview();});
 await check('Map Registry preview blocks goals',async()=>{await page.locator('#mapSettingsToggle').click();await page.locator('#mapVersionSelect').selectOption(info.map_version);await page.waitForTimeout(900);assert.match(await page.locator('#mapPreviewBanner').textContent(),/UNVERIFIED/);await page.locator('#map').click({position:{x:80,y:80}});await page.locator('[data-tab=control]').click();assert.ok(await page.locator('#mcreate').isDisabled());await overview();await page.locator('#mapVersionSelect').selectOption('__live__');await page.locator('#mapSettingsToggle').click();});
 await check('Lease expiry is visible',async()=>{await page.locator('[data-tab=control]').click();await page.locator('#leaseTtl').fill('5');await page.locator('#leaseAcquire').click();await page.waitForTimeout(6200);assert.match(await page.locator('#leaseStatusBadge').textContent(),/미점유/);assert.match(await page.locator('#gateSummary').textContent(),/미보유/);await page.locator('#leaseTtl').fill('30');await overview();});
 await check('Lease, create and cancel through real API',async()=>{
  await page.locator('[data-tab=control]').click();await page.locator('#leaseAcquire').click();await page.waitForTimeout(700);assert.match(await page.locator('#leaseStatusBadge').textContent(),/내 제어권/);
  await page.locator('#mx').fill('1.0');await page.locator('#my').fill('0.5');
  const response=page.waitForResponse(r=>r.url().endsWith('/api/v1/missions') && r.request().method()==='POST');await page.locator('#mcreate').click();assert.equal((await response).status(),201);
  await overview();await page.waitForTimeout(500);await shot('active-mission');assert.ok(await page.locator('#quickCancel').isEnabled());
  const cancel=page.waitForResponse(r=>r.url().endsWith('/cancel'));await page.locator('#quickCancel').click();assert.equal((await cancel).status(),202);await page.waitForTimeout(300);await shot('cancel-requested');
 });
 for(const name of ['fault','gate-stale','no-attestation','unverified','offline','reconciliation']){
  await scenario(name); await check(name+' fail closed display',async()=>{const label=await page.locator('#statusMovement').textContent();assert.match(label,name==='offline'?/미확인/:/차단/);assert.doesNotMatch(await page.locator('#statusGate').textContent(),/^정상/);}); await shot(name);
 }
 await check('all detail tabs retained',async()=>{for(const name of ['control','maps','safety','rmf','media','history','settings','docs']){await page.locator(`[data-tab=${name}]`).click();assert.ok(await page.locator(`[data-panel=${name}]`).isVisible());}await overview();});
 await scenario('normal');
 await check('viewer cannot operate',async()=>{
  await page.locator('#logout').click();await page.locator('#tok').fill(fs.readFileSync(path.join(info.token_dir,'viewer.token'),'utf8').trim());await page.locator('#go').click();await page.waitForTimeout(600);
  assert.ok(await page.locator('#newMission').isDisabled());assert.ok(await page.locator('#quickCancel').isDisabled());await page.locator('[data-tab=control]').click();assert.ok(await page.locator('#mcreate').isDisabled());assert.ok(await page.locator('#leaseAcquire').isDisabled());await overview();
 });
 await check('dashboard network failure becomes unknown',async()=>{
  await page.route('**/api/v1/dashboard',route=>route.abort());await page.waitForTimeout(3600);assert.match(await page.locator('#statusMovement').textContent(),/미확인/);assert.match(await page.locator('#robotConnection').textContent(),/OFFLINE/);await shot('server-offline');await page.unroute('**/api/v1/dashboard');
 });
 // Network abort above is intentional; all prior browser exceptions must remain absent.
 const unexpected=errors.filter(e=>!e.includes('net::ERR_FAILED'));
 assert.deepEqual(unexpected,[]);
 fs.writeFileSync(path.join(output,'limo-ui-browser-results.json'),JSON.stringify({checks,errors,expectedNetworkFailure:true},null,2));
 await browser.close();
})().catch(e=>{console.error(e);process.exit(1)});
