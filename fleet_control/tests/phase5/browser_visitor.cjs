'use strict';
// Visitor page in a phone-size headless Chrome (test-only). One JSON line per event on stdout.
// usage: node browser_visitor.cjs BASE_URL VISITOR_TOKEN SPEC_JSON
//   SPEC: {request: "<destination name>"|null, cancel_on: "IN_PROGRESS"|null, timeout_s: 150, session_tag: "a"}
const {chromium} = require('playwright');
const [base, token, specJson] = process.argv.slice(2);
const spec = JSON.parse(specJson || '{}');
const out = (kind, extra = {}) => console.log(JSON.stringify({kind, t: Date.now() / 1000, ...extra}));
(async () => {
  const browser = await chromium.launch({executablePath: '/usr/bin/google-chrome', headless: true, args: ['--no-sandbox']});
  try {
    const ctx = await browser.newContext({viewport: {width: 390, height: 844}, isMobile: true, hasTouch: true, deviceScaleFactor: 2});
    const page = await ctx.newPage(); const errors = [];
    page.on('pageerror', e => errors.push(e.message));
    await page.goto(`${base}/visitor.html#t=${token}`, {waitUntil: 'domcontentloaded'});
    out('url_after_load', {url: page.url(), token_in_url: page.url().includes(token)});
    await page.waitForFunction(() => !document.getElementById('vPick').classList.contains('hidden') || !document.getElementById('vLink').classList.contains('hidden')
                                     || !document.getElementById('vReq').classList.contains('hidden'), null, {timeout: 15000});
    await page.waitForTimeout(800);
    const list = await page.$$eval('#vList li[data-dest]', ls => ls.map(l => ({name: l.querySelector('b').textContent, available: l.dataset.available === 'true',
      why: (l.querySelector('.v-why') || {}).textContent || null})));
    const layout = await page.evaluate(() => ({docW: document.documentElement.scrollWidth, vw: innerWidth,
      operatorControls: !!document.querySelector('[data-mc], #gateResetCard, #systemReadinessCard, [data-tab]')}));
    out('list', {service: await page.locator('#vService').textContent(), floors: await page.$$eval('#vFloor option', o => o.map(x => x.textContent)),
                 destinations: list, layout, link_problem: !(await page.locator('#vLink').isHidden())});
    await page.screenshot({path: `${spec.shot || '/tmp'}/visitor_list_${spec.session_tag || 'a'}.png`, fullPage: true}).catch(() => {});
    if (!spec.request) { out('end', {errors}); return; }
    const btn = page.locator('#vList li[data-dest]', {hasText: spec.request}).locator('button[data-go]');
    if (await btn.isDisabled()) { out('request', {name: spec.request, sent: false, reason: 'button disabled'}); out('end', {errors}); return; }
    await btn.click(); out('request', {name: spec.request, sent: true});
    const seen = []; const end = Date.now() + (spec.timeout_s || 150) * 1000; let cancelled = false;
    while (Date.now() < end) {
      const err = await page.locator('#vError').textContent();
      const st = await page.locator('#vReqStatus').getAttribute('data-status').catch(() => null);
      const txt = await page.locator('#vReqStatus').textContent().catch(() => '');
      if (st && (!seen.length || seen[seen.length - 1].status !== st)) { seen.push({status: st, text: txt, t: Date.now() / 1000}); out('status', {status: st, text: txt, reason: await page.locator('#vReqReason').textContent()}); }
      if (!st && err) { out('error_line', {text: err}); break; }
      if (spec.cancel_on && st === spec.cancel_on && !cancelled) {
        page.once('dialog', d => d.accept()); await page.locator('#vCancel').click(); cancelled = true; out('cancel_clicked', {at: st});
      }
      if (['COMPLETED', 'CANCELED', 'FAILED', 'UNAVAILABLE'].includes(st)) break;
      await page.waitForTimeout(500);
    }
    // reload: the page restores the same request (same browser session)
    await page.reload({waitUntil: 'domcontentloaded'}); await page.waitForTimeout(2500);
    out('after_reload', {status: await page.locator('#vReqStatus').getAttribute('data-status').catch(() => null),
                         steps_done: await page.$$eval('#vSteps li.done', l => l.length)});
    await page.screenshot({path: `${spec.shot || '/tmp'}/visitor_status_${spec.session_tag || 'a'}.png`, fullPage: true}).catch(() => {});
    out('end', {errors, seen});
  } finally { await browser.close(); }
})().catch(e => { out('fatal', {message: String(e && e.stack || e)}); process.exit(1); });
