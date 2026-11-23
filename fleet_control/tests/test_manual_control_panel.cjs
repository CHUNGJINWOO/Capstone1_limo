'use strict';
const assert = require('node:assert/strict');
const path = require('node:path');
const { chromium } = require('playwright');
const source = path.resolve(__dirname, '../server/fleet_server/static/manual_control_panel.js');
const css = path.resolve(__dirname, '../server/fleet_server/static/manual_control_panel.css');
(async () => {
 const browser=await chromium.launch({executablePath:'/usr/bin/google-chrome',headless:true,args:['--no-sandbox']});
 const page=await browser.newPage({viewport:{width:1280,height:800}});
 const errors=[]; page.on('pageerror', e=>errors.push(e.message));
 await page.setContent('<main id="panel"></main><input id="text" /><div id="outside"></div>');
 await page.addScriptTag({path:source}); await page.addStyleTag({path:css});
 await page.evaluate(() => {
   window.ctx={evidenceAgeMs:0,canControl:true,online:true,agentReady:true,motorLocked:true,leaseOwned:true,leaseEpoch:1,mode:'IDLE',navIdleProven:true,gateNormal:true,robotId:'test',agentBootId:'a1',serverBootId:'s1'};
   window.active=true;window.messages=[];window.fail=false;window.fakePad=null;window.testId=0;
   Object.defineProperty(window,'isSecureContext',{configurable:true,value:true});
   Object.defineProperty(navigator,'getGamepads',{configurable:true,value:()=>window.fakePad?[window.fakePad]:[]});
   window.panel=ManualControlPanel.mount(document.querySelector('#panel'),{getContext:()=>window.ctx,isActive:()=>window.active,
      request:async(path,options)=>{let v=JSON.parse(options.body);if(v.action==='sample' && (v.input_format !== 'normalized_axes_v1' || 'ticket' in v)) throw Error('wrong Bench input format');window.messages.push({path,...v});if(window.fail) throw Error('disconnected');return new Response(JSON.stringify(v.action==='start'?{session_id:'s'+(++window.testId)}:{received:v.seq}),{status:200,headers:{'Content-Type':'application/json'}})}});
 });
 const get=()=>page.evaluate(()=>window.messages);
 const reset=()=>page.evaluate(()=>{window.messages=[];window.fail=false;window.active=true;Object.assign(window.ctx,{evidenceAgeMs:0,canControl:true,online:true,agentReady:true,motorLocked:true,leaseOwned:true,leaseEpoch:1,mode:'IDLE',navIdleProven:true,gateNormal:true,agentBootId:'a1',serverBootId:'s1'});});
 const start=async()=>{await page.locator('[data-mc=start]').click();await page.waitForTimeout(120);};
 const stop=async()=>{await page.locator('[data-mc=stop]').click();await page.waitForTimeout(100);};
 const check=(label,fn)=>Promise.resolve().then(fn).then(()=>console.log('PASS',label));
 await check('입력 방향, 범위, NaN·Inf 거부',async()=>{
  const v=await page.evaluate(()=>{const M=ManualControlPanel;return {k:M.keyboardAxes(new Set(['KeyW','KeyA','Space'])),arrows:M.keyboardAxes(new Set(['ArrowDown','ArrowRight','Space'])),p:M.pointerAxes(200,0,{left:0,top:0,width:200,height:200}),bad:M.pointerAxes(NaN,0,{left:0,top:0,width:1,height:1}),nan:M.gamepadAxes({connected:true,axes:[NaN,0],buttons:[0,0,0,0,{}]}),inf:M.gamepadAxes({connected:true,axes:[Infinity,0],buttons:[0,0,0,0,{}]})}});
  assert.deepEqual(v.k,{linear:1,turn:1,deadman:true});assert.deepEqual(v.arrows,{linear:-1,turn:-1,deadman:true});assert.ok(v.p.linear>0&&v.p.turn<0&&Math.hypot(v.p.linear,v.p.turn)<=1.00001);assert.equal(v.bad,null);assert.equal(v.nan,null);assert.equal(v.inf,null);
 });
 await check('Lease·Mapping·Nav2 Goal·Gate·Agent·증거 거부',async()=>{
  const bad=await page.evaluate(()=>{let fields={leaseOwned:false,mode:'MAPPING',navIdleProven:false,gateNormal:false,agentReady:false,motorLocked:false,evidenceAgeMs:1001};return Object.entries(fields).map(([k,v])=>ManualControlPanel.admissibility({...window.ctx,[k]:v}));});assert.ok(bad.every(Boolean));
 });
 await check('키보드 Deadman과 방향, 해제 시 세션 중지',async()=>{
  await reset();await start();await page.keyboard.down('KeyW');await page.keyboard.down('KeyA');await page.keyboard.down('Space');await page.waitForTimeout(190);let m=await get();assert.ok(m.some(x=>x.action==='sample'&&x.linear===1&&x.turn===1&&x.deadman));await page.keyboard.up('Space');await page.keyboard.up('KeyW');await page.keyboard.up('KeyA');await page.waitForTimeout(100);m=await get();assert.equal(m.at(-1).action,'stop');assert.match(await page.locator('[data-mc=values]').textContent(),/0\.00/);
 });
 await check('텍스트 입력 포커스에서 명령 차단',async()=>{
  await reset();await start();await page.locator('#text').focus();await page.keyboard.down('KeyW');await page.keyboard.down('Space');await page.waitForTimeout(120);await page.keyboard.up('Space');await page.keyboard.up('KeyW');const m=await get();assert.ok(m.some(x=>x.action==='stop'));assert.ok(!m.some(x=>x.action==='sample'&&x.deadman));
 });
 await check('마우스 누름·이동·해제',async()=>{
  await reset();await page.locator('[data-mc=source]').selectOption('pointer');await start();const b=await page.locator('[data-mc=stick]').boundingBox();await page.mouse.move(b.x+b.width/2,b.y+b.height/2);await page.mouse.down();await page.mouse.move(b.x+b.width/2,b.y+4);await page.waitForTimeout(180);assert.ok((await get()).some(x=>x.action==='sample'&&x.linear>.8&&x.deadman));await page.mouse.up();await page.waitForTimeout(100);assert.equal((await get()).at(-1).action,'stop');
 });
 await check('Gamepad L1과 연결 해제',async()=>{
  await reset();await page.locator('[data-mc=source]').selectOption('gamepad');await page.evaluate(()=>{window.fakePad={connected:true,index:0,id:'test-pad',axes:[0,0],buttons:[{},{},{},{},{pressed:false}],timestamp:1};});await start();await page.evaluate(()=>{window.fakePad.axes=[-.5,-.75];window.fakePad.buttons[4].pressed=true;window.fakePad.timestamp=2;});await page.waitForTimeout(150);assert.ok((await get()).some(x=>x.action==='sample'&&x.deadman&&x.linear===.75&&x.turn===.5));await page.evaluate(()=>{window.fakePad.buttons[4].pressed=false;window.fakePad.timestamp=3});await page.waitForTimeout(120);assert.equal((await get()).at(-1).action,'stop');
  await reset();await start();await page.evaluate(()=>{window.fakePad=null});await page.waitForTimeout(120);assert.equal((await get()).at(-1).action,'stop');
 });
 await check('탭 전환·서버 재시작·연결 손실은 자동 재개 금지',async()=>{
  for(const what of ['tab','boot','agent','lease','mapping','nav','network','blur','visibility']) {await reset();await page.locator('[data-mc=source]').selectOption('keyboard');await start();if(what==='tab')await page.evaluate(()=>window.active=false);if(what==='boot')await page.evaluate(()=>window.ctx.serverBootId='s2');if(what==='agent')await page.evaluate(()=>window.ctx.agentBootId='a2');if(what==='lease')await page.evaluate(()=>window.ctx.leaseOwned=false);if(what==='mapping')await page.evaluate(()=>window.ctx.mode='MAPPING');if(what==='nav')await page.evaluate(()=>window.ctx.navIdleProven=false);if(what==='network')await page.evaluate(()=>window.fail=true);if(what==='blur')await page.evaluate(()=>window.dispatchEvent(new Event('blur')));if(what==='visibility')await page.evaluate(()=>{Object.defineProperty(document,'hidden',{configurable:true,value:true});document.dispatchEvent(new Event('visibilitychange'));delete document.hidden});await page.waitForTimeout(250);const first=(await get()).filter(x=>x.action==='start').length;await page.waitForTimeout(200);assert.equal((await get()).filter(x=>x.action==='start').length,first);assert.match(await page.locator('[data-mc=values]').textContent(),/Deadman 해제/);}
 });
 await check('모바일 터치 방향·해제 및 반응형 너비',async()=>{
  await reset();await page.setViewportSize({width:375,height:812});await page.locator('[data-mc=source]').selectOption('pointer');await start();const stick=page.locator('[data-mc=stick]');await stick.scrollIntoViewIfNeeded();const b=await stick.boundingBox();const cdp=await page.context().newCDPSession(page);await cdp.send('Emulation.setTouchEmulationEnabled',{enabled:true});await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:b.x+b.width/2,y:b.y+5}]});await page.waitForTimeout(180);assert.ok((await get()).some(x=>x.action==='sample'&&x.deadman&&x.linear>.8));await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await page.waitForTimeout(100);assert.equal((await get()).at(-1).action,'stop');assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);await cdp.detach();
 });
 assert.deepEqual(errors,[]);await page.evaluate(()=>window.panel.destroy());await browser.close();
})().catch(e=>{console.error(e);process.exitCode=1});
