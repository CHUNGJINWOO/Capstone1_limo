'use strict';
/* Real isolated headless Chrome E2E test for Phase C Web Map Visualization & Speed Limit Integration (2026-11-23).
 * Exercises POST(202 PENDING) -> GET polling -> APPLIED / REJECTED / EXPIRED / STALE, dynamic Agent ceilings,
 * enforced_by (null / WATCHDOG unverified / ARBITER), /odom velocity states, Lease/Agent/Server changes, and map/trail/plan regression.
 * Uses local loopback ephemeral port (127.0.0.1:0) + isolated Chrome user-data-dir.
 * Zero protected ports (no 8080/18190), zero real LIMO, zero ROS processes. */
const test = require('node:test');
const assert = require('node:assert/strict');
const http = require('node:http');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { spawn } = require('node:child_process');

const STATIC_DIR = path.resolve(__dirname, '../server/fleet_server/static');
const CHROME_BIN = ['/usr/bin/google-chrome', '/usr/bin/chromium', '/usr/bin/chromium-browser'].find((p) => fs.existsSync(p));

const TINY_PNG = Buffer.from(
  'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=',
  'base64'
);

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'application/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.png': 'image/png',
  '.json': 'application/json; charset=utf-8',
};

function createMockFleetServer() {
  const mockState = {
    disconnected: false,
    online: true,
    state_age_s: 0.3,
    server_boot_id: 'srv-1',
    agent_boot_id: 'boot-chrome-1',
    operation_mode: 'MANUAL',
    lease_owner: 'op1',
    lease_epoch: 4,
    pose_x: 1.0,
    pose_y: 0.5,
    pose_yaw: 0.5,
    pose_status: 'OK',
    plan: [[1.0, 0.5], [1.5, 0.8], [2.0, 1.0], [2.5, 1.2]],
    mission_terminal: false,
    vel_status: 'OK',
    vel_linear: 0.06,
    vel_angular: -0.10,
    sensor_seq: 1,
    // Phase C speed-limits state
    enforced_by: null, // starts as real robot (null -> blocked), then WATCHDOG unverified, then ARBITER
    watchdog_clamp_verified: false,
    ceiling: {
      manual: { linear_mps: 0.10, angular_rps: 0.30 },
      auto: { linear_mps: 0.12, angular_rps: 0.40, reverse_mps: 0.10 },
    },
    speedGen: 0,
    speedView: {
      generation: 0,
      request_id: null,
      status: 'NONE',
      approved: false,
      reason: 'NO_REQUEST',
      scope: null,
      requested: null,
      applied: null,
      effective: {
        manual: { linear_mps: 0.10, angular_rps: 0.30 },
        auto: { linear_mps: 0.12, angular_rps: 0.40 },
      },
      stale: false,
      stale_reason: null,
      source: 'Agent Arbiter clamp (설정값, 실차 미검증)',
      agent_report_age_s: 0.2,
    },
    nextPollResolution: 'APPLIED', // 'APPLIED' | 'REJECTED' | 'EXPIRED'
    pollsUntilResolve: 1,
    postLog: [],
  };

  function readJson(req) {
    return new Promise((resolve) => {
      let data = '';
      req.on('data', (c) => { data += c; });
      req.on('end', () => {
        try { resolve(JSON.parse(data || '{}')); } catch { resolve({}); }
      });
    });
  }

  function currentSpeedView() {
    return {
      ...mockState.speedView,
      enforced_by: mockState.enforced_by,
      watchdog_clamp_verified: mockState.watchdog_clamp_verified,
      ceiling: mockState.ceiling,
    };
  }

  const server = http.createServer(async (req, res) => {
    const urlPath = (req.url || '/').split('?')[0];
    if (urlPath.startsWith('/api/v1/')) {
      if (mockState.disconnected) {
        res.writeHead(503, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ error: { message: 'mock server disconnected' } }));
        return;
      }
      if (urlPath === '/api/v1/me') {
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ name: 'op1', role: 'operator' }));
        return;
      }
      if (urlPath === '/api/v1/robots/limo-01/speed-limits') {
        if (req.method === 'POST') {
          const body = await readJson(req);
          mockState.postLog.push(body);
          if (!mockState.enforced_by) {
            res.writeHead(409, { 'Content-Type': 'application/json' });
            res.end(JSON.stringify({ error: { code: 'agent_cannot_enforce', message: 'cannot enforce' } }));
            return;
          }
          mockState.speedGen += 1;
          mockState.speedView = {
            ...mockState.speedView,
            generation: mockState.speedGen,
            request_id: body.client_request_id || `req-${mockState.speedGen}`,
            status: 'PENDING',
            approved: false,
            reason: 'PENDING_AGENT_APPLY',
            scope: body.scope || 'auto',
            requested: { max_linear_mps: body.max_linear_mps, max_angular_rps: body.max_angular_rps },
            stale: false,
            stale_reason: null,
          };
          mockState._remainingPolls = mockState.pollsUntilResolve;
          res.writeHead(202, { 'Content-Type': 'application/json' });
          res.end(JSON.stringify(currentSpeedView()));
          return;
        }
        // GET polling: advance PENDING -> nextPollResolution after configured polls
        if (mockState.speedView.status === 'PENDING') {
          mockState._remainingPolls = (mockState._remainingPolls || 1) - 1;
          if (mockState._remainingPolls <= 0) {
            const reqd = mockState.speedView.requested;
            const sc = mockState.speedView.scope;
            if (mockState.nextPollResolution === 'APPLIED') {
              mockState.speedView = {
                ...mockState.speedView,
                status: 'APPLIED',
                approved: true,
                reason: 'APPLIED',
                applied: {
                  linear_mps: reqd.max_linear_mps,
                  angular_rps: reqd.max_angular_rps,
                  generation: mockState.speedGen,
                  lease_epoch: mockState.lease_epoch,
                  age_s: 0.1,
                },
                effective: {
                  ...mockState.speedView.effective,
                  [sc]: { linear_mps: reqd.max_linear_mps, angular_rps: reqd.max_angular_rps },
                },
              };
            } else if (mockState.nextPollResolution === 'REJECTED') {
              mockState.speedView = {
                ...mockState.speedView,
                status: 'REJECTED',
                approved: false,
                reason: 'EXCEEDS_CEILING',
              };
            } else if (mockState.nextPollResolution === 'MISMATCHED_APPLIED') {
              // Server/Agent returns approved=true but applied values differ from requested!
              mockState.speedView = {
                ...mockState.speedView,
                status: 'APPLIED',
                approved: true,
                reason: 'APPLIED',
                applied: {
                  linear_mps: 0.03,
                  angular_rps: reqd.max_angular_rps,
                  generation: mockState.speedGen,
                  lease_epoch: mockState.lease_epoch,
                  age_s: 0.1,
                },
              };
            } else if (mockState.nextPollResolution === 'EXPIRED') {
              mockState.speedView = {
                ...mockState.speedView,
                status: 'EXPIRED',
                approved: false,
                reason: 'AGENT_NO_CONFIRMATION',
              };
            }
          }
        }
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify(currentSpeedView()));
        return;
      }
      if (urlPath === '/api/v1/dashboard') {
        const velObj = mockState.vel_status === 'UNAVAILABLE'
          ? { status: 'UNAVAILABLE', linear_x_mps: null, angular_z_rps: null, source: '/odom' }
          : { status: mockState.vel_status, linear_x_mps: mockState.vel_linear, angular_z_rps: mockState.vel_angular, source: '/odom' };
        const robot = {
          robot_id: 'limo-01',
          online: mockState.online,
          state_age_s: mockState.state_age_s,
          reconciled: true,
          reconciliation_required: false,
          control_mode: mockState.operation_mode,
          operating_mode: { mode: mockState.operation_mode, revision: 1 },
          lease: mockState.lease_owner ? { owner: mockState.lease_owner, epoch: mockState.lease_epoch, remaining_s: 55 } : null,
          agent: { protocol_version: 2, safety_contract_version: 1, boot_id: mockState.agent_boot_id, mode: 'mock', simulated: true, locked_velocity_arbiter: true, missions_enabled: true },
          map_version: 1,
          map_meta: { width: 100, height: 100, resolution: 0.05, origin_x: -2.5, origin_y: -2.5, origin_yaw: 0 },
          state: {
            operation_mode: mockState.operation_mode,
            operation_revision: 1,
            localization_status: 'CONFIRMED',
            localization_epoch: 1,
            localization_map_version_id: 'map_v1',
            map_version_id: 'map_v1',
            map_epoch: 1,
            map_verified: true,
            pose: { status: mockState.pose_status, frame_id: 'map', x: mockState.pose_x, y: mockState.pose_y, yaw_rad: mockState.pose_yaw },
            velocity: velObj,
            speed_limits: currentSpeedView(),
            safety_gate: { status: 'NORMAL', age_s: 0.1, rx_fresh: true, source_fresh: true, fault_latched: false, cmd_blocked: false, movement_permitted: true, motor_path: { status: 'SIM_GATED', cmd_vel_publishers: [] } },
            motion_watchdog: { state: 'MOTION_AUTHORIZED' },
            velocity_arbiter: { motor_output: false, ready: true },
            execution: { active: null, nav2_goals: { known: true, received_status_array: true, active_count: 0 } },
            battery: { voltage_v: 12.4 },
          },
        };
        const missions = [{
          mission_id: 'm-e2e-1',
          robot_id: 'limo-01',
          status: mockState.mission_terminal ? 'CANCELED' : 'EXECUTING',
          terminal: mockState.mission_terminal,
          created_by: 'op1',
          created_at: Date.now() / 1000,
          goal: { frame_id: 'map', x: 2.5, y: 1.2, yaw_rad: 0.3 },
          waypoints: [{ frame_id: 'map', x: 1.5, y: 0.8, label: 'W1' }],
        }];
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ server: { boot_id: mockState.server_boot_id }, robots: [robot], missions, events: [] }));
        return;
      }
      if (urlPath === '/api/v1/robots/limo-01/sensors') {
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({
          available: true,
          seq: mockState.sensor_seq,
          age_s: 0.2,
          global_plan: mockState.plan,
          frontiers: [[2.0, 1.5]],
          lidar_points: [[1.2, 0.6]],
          has_costmap: false,
          has_rgb: false,
          has_depth: false,
          has_lidar_img: false,
        }));
        return;
      }
      if (urlPath === '/api/v1/robots/limo-01/map.png') {
        res.writeHead(200, { 'Content-Type': 'image/png' });
        res.end(TINY_PNG);
        return;
      }
      if (urlPath.startsWith('/api/v1/map-registry/')) {
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ buildings: [], floors: [], areas: [], versions: [] }));
        return;
      }
      res.writeHead(200, { 'Content-Type': 'application/json' });
      res.end(JSON.stringify({ ok: true }));
      return;
    }

    const rel = urlPath === '/' ? 'index.html' : urlPath.replace(/^\/+/, '');
    const filePath = path.join(STATIC_DIR, path.basename(rel));
    if (fs.existsSync(filePath) && fs.statSync(filePath).isFile()) {
      res.writeHead(200, { 'Content-Type': MIME[path.extname(filePath)] || 'application/octet-stream' });
      fs.createReadStream(filePath).pipe(res);
      return;
    }
    res.writeHead(404);
    res.end('Not Found');
  });

  return { server, mockState };
}

class CDPClient {
  constructor(wsUrl) {
    this.ws = new WebSocket(wsUrl);
    this.id = 0;
    this.pending = new Map();
    this.errors = [];
    this.ready = new Promise((resolve, reject) => {
      this.ws.addEventListener('open', resolve);
      this.ws.addEventListener('error', reject);
    });
    this.ws.addEventListener('message', (ev) => {
      const msg = JSON.parse(String(ev.data));
      if (msg.id && this.pending.has(msg.id)) {
        const { resolve, reject } = this.pending.get(msg.id);
        this.pending.delete(msg.id);
        if (msg.error) reject(new Error(msg.error.message));
        else resolve(msg.result);
      } else if (msg.method === 'Runtime.exceptionThrown') {
        this.errors.push(msg.params?.exceptionDetails?.text || 'Runtime.exceptionThrown');
      }
    });
  }
  send(method, params = {}) {
    const id = ++this.id;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.ws.send(JSON.stringify({ id, method, params }));
    });
  }
  async evalJS(expression) {
    const res = await this.send('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true });
    if (res.exceptionDetails) throw new Error(res.exceptionDetails.text);
    return res.result?.value;
  }
  close() {
    try { this.ws.close(); } catch (_) {}
  }
}

async function waitUntil(fn, timeoutMs = 8000, intervalMs = 100) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    const val = await fn();
    if (val) return val;
    await new Promise((r) => setTimeout(r, intervalMs));
  }
  throw new Error('Timed out waiting for condition');
}

test('Phase C Isolated Headless Chrome E2E: POST(202 PENDING)->GET polling, ARBITER/WATCHDOG gating, /odom velocity, and map regression', { skip: !CHROME_BIN }, async () => {
  const { server, mockState } = createMockFleetServer();
  await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
  const port = server.address().port;
  const userDataDir = fs.mkdtempSync(path.join(os.tmpdir(), 'fleet-ui-phasec-'));
  const chrome = spawn(CHROME_BIN, [
    '--headless=new',
    '--no-sandbox',
    '--disable-gpu',
    '--disable-dev-shm-usage',
    '--remote-debugging-port=0',
    `--user-data-dir=${userDataDir}`,
    'about:blank',
  ], { stdio: ['ignore', 'pipe', 'pipe'] });

  let cdp = null;
  try {
    const wsBase = await new Promise((resolve, reject) => {
      let buf = '';
      const timer = setTimeout(() => reject(new Error('Chrome DevTools port timeout: ' + buf)), 10000);
      chrome.stderr.on('data', (chunk) => {
        buf += chunk.toString();
        const m = buf.match(/DevTools listening on (ws:\/\/127\.0\.0\.1:\d+\/\S+)/);
        if (m) { clearTimeout(timer); resolve(m[1]); }
      });
      chrome.on('exit', (code) => { clearTimeout(timer); reject(new Error('Chrome exited early: ' + code)); });
    });

    const debugPort = new URL(wsBase).port;
    const targets = await (await fetch(`http://127.0.0.1:${debugPort}/json/list`)).json();
    const pageTarget = targets.find((t) => t.type === 'page') || targets[0];
    cdp = new CDPClient(pageTarget.webSocketDebuggerUrl);
    await cdp.ready;
    await cdp.send('Page.enable');
    await cdp.send('Runtime.enable');

    // Load dashboard and sign in with mock operator token
    await cdp.send('Page.navigate', { url: `http://127.0.0.1:${port}/` });
    await waitUntil(() => cdp.evalJS(`document.readyState === 'complete' && Boolean(document.getElementById('speedControlHost'))`));
    await cdp.evalJS(`sessionStorage.setItem('fleet_token', 'op-tok'); location.reload();`);
    await waitUntil(() => cdp.evalJS(`(document.getElementById('spatialSummary')?.textContent || '').includes('OK (4pts)')`));

    // 1) Map & Trail regression check
    assert.equal(await cdp.evalJS(`!document.getElementById('spatialRoute').disabled`), true);
    mockState.pose_x = 1.3; mockState.pose_y = 0.7;
    await waitUntil(() => cdp.evalJS(`(document.getElementById('spatialSummary')?.textContent || '').includes('궤적 2/600')`));
    await cdp.evalJS(`document.getElementById('mapClearTrail').click()`);
    await waitUntil(() => cdp.evalJS(`(document.getElementById('spatialSummary')?.textContent || '').includes('궤적 0/600')`));

    // 2) Real robot (enforced_by: null) -> MUST block both manual and auto UI
    const realBlocked = await cdp.evalJS(`({
      manualDisabled: document.querySelector('[data-spd="manualApply"]')?.disabled,
      autoDisabled: document.querySelector('[data-spd="autoApply"]')?.disabled,
      enforced: document.querySelector('[data-spd="enforcedBy"]')?.textContent,
    })`);
    assert.equal(realBlocked.manualDisabled, true);
    assert.equal(realBlocked.autoDisabled, true);
    assert.ok(realBlocked.enforced.includes('강제 지점 없음'));

    // 3) Future WATCHDOG with bare boolean flag (no corroborated evidence) -> MUST stay blocked
    mockState.enforced_by = 'WATCHDOG';
    mockState.watchdog_clamp_verified = true;
    await waitUntil(() => cdp.evalJS(`(document.querySelector('[data-spd="enforcedBy"]')?.textContent || '').includes('WATCHDOG')`));
    assert.equal(await cdp.evalJS(`document.querySelector('[data-spd="autoApply"]')?.disabled`), true);

    // 4) ARBITER enabled -> dynamic ceiling from Agent (auto: 0.12/0.40) enables UI
    mockState.enforced_by = 'ARBITER';
    await waitUntil(() => cdp.evalJS(`document.querySelector('[data-spd="autoApply"]')?.disabled === false`));
    assert.equal(await cdp.evalJS(`document.querySelector('[data-spd="autoLin"]')?.max`), '0.12');

    // 5) POST(202 PENDING) -> GET polling -> APPLIED (auto scope)
    mockState.nextPollResolution = 'APPLIED';
    mockState.pollsUntilResolve = 1;
    await cdp.evalJS(`
      document.querySelector('[data-spd="autoLin"]').value = '0.09';
      document.querySelector('[data-spd="autoAng"]').value = '0.25';
      document.querySelector('[data-spd="autoApply"]').click();
    `);
    // Immediately after POST 202, must show PENDING (not yet APPLIED)
    await waitUntil(() => cdp.evalJS(`(document.querySelector('[data-spd="result"]')?.textContent || '').includes('[PENDING]')`));
    // After 1s GET poll returns approved:true -> transitions to APPLIED
    await waitUntil(() => cdp.evalJS(`(document.querySelector('[data-spd="result"]')?.textContent || '').includes('[APPLIED]')`));
    const appliedMeta = await cdp.evalJS(`document.querySelector('[data-spd="metaApplied"]')?.textContent`);
    assert.ok(appliedMeta.includes('0.09') && appliedMeta.includes('0.25'), 'appliedMeta=' + appliedMeta);
    assert.equal(mockState.postLog[0].protocol_version, 2);
    assert.equal(mockState.postLog[0].safety_contract_version, 1);
    assert.equal(mockState.postLog[0].lease_epoch, 4);
    assert.equal(mockState.postLog[0].scope, 'auto');

    // 6) Manual scope POST -> GET APPLIED syncs Gamepad max velocity inputs
    await cdp.evalJS(`
      document.querySelector('[data-spd="manualLin"]').value = '0.07';
      document.querySelector('[data-spd="manualAng"]').value = '0.22';
      document.querySelector('[data-spd="manualApply"]').click();
    `);
    await waitUntil(() => cdp.evalJS(`document.getElementById('gpMaxLinear')?.value === '0.07' && document.getElementById('gpMaxTurn')?.value === '0.22'`));

    // 7) REJECTED via GET poll
    mockState.nextPollResolution = 'REJECTED';
    await cdp.evalJS(`
      document.querySelector('[data-spd="autoLin"]').value = '0.08';
      document.querySelector('[data-spd="autoApply"]').click();
    `);
    await waitUntil(() => cdp.evalJS(`(document.querySelector('[data-spd="result"]')?.textContent || '').includes('[REJECTED]')`));

    // 8) EXPIRED via GET poll
    mockState.nextPollResolution = 'EXPIRED';
    await cdp.evalJS(`
      document.querySelector('[data-spd="autoLin"]').value = '0.07';
      document.querySelector('[data-spd="autoApply"]').click();
    `);
    await waitUntil(() => cdp.evalJS(`(document.querySelector('[data-spd="result"]')?.textContent || '').includes('[EXPIRED]')`));

    // 8b) GET returns approved=true but applied velocity mismatches requested -> MUST NOT show APPLIED!
    mockState.nextPollResolution = 'MISMATCHED_APPLIED';
    await cdp.evalJS(`
      document.querySelector('[data-spd="autoLin"]').value = '0.08';
      document.querySelector('[data-spd="autoApply"]').click();
    `);
    await waitUntil(() => cdp.evalJS(`(document.querySelector('[data-spd="result"]')?.textContent || '').includes('적용 응답 불일치')`));
    const mismatchRes = await cdp.evalJS(`document.querySelector('[data-spd="result"]')?.textContent`);
    assert.ok(mismatchRes.includes('[REJECTED]') && !mismatchRes.includes('[APPLIED]'), mismatchRes);

    // 9) Agent restart (boot_id change) marks applied config STALE as unconfirmed (NEVER claims lower speed still applied)
    mockState.nextPollResolution = 'APPLIED';
    await cdp.evalJS(`document.querySelector('[data-spd="autoApply"]').click()`);
    await waitUntil(() => cdp.evalJS(`(document.querySelector('[data-spd="result"]')?.textContent || '').includes('[APPLIED]')`));
    mockState.agent_boot_id = 'boot-chrome-restarted-2';
    await waitUntil(() => cdp.evalJS(`(document.querySelector('[data-spd="result"]')?.textContent || '').includes('[STALE]')`));
    const restartRes = await cdp.evalJS(`document.querySelector('[data-spd="result"]')?.textContent`);
    assert.ok(restartRes.includes('미확인') && !restartRes.includes('낮은 값이 계속 적용 중'), restartRes);

    // 10) /odom velocity telemetry: OK -> STALE (dimmed .vel-stale) -> UNAVAILABLE (no numbers)
    assert.equal(await cdp.evalJS(`document.querySelector('[data-spd="obsVel"]')?.textContent`), 'v 0.06 m/s · w -0.10 rad/s');
    mockState.vel_status = 'STALE';
    await waitUntil(() => cdp.evalJS(`document.querySelector('[data-spd="obsVel"]')?.classList.contains('vel-stale')`));
    mockState.vel_status = 'UNAVAILABLE';
    await waitUntil(() => cdp.evalJS(`document.querySelector('[data-spd="obsVel"]')?.textContent === 'UNAVAILABLE'`));

    // 11) Server disconnect -> unconfirmed state -> reconnect clean recovery
    mockState.disconnected = true;
    await waitUntil(() => cdp.evalJS(`(document.getElementById('globalRobotState')?.textContent || '').includes('OFFLINE')`));
    mockState.disconnected = false;
    await waitUntil(() => cdp.evalJS(`(document.getElementById('globalRobotState')?.textContent || '').includes('ONLINE')`));

    assert.deepEqual(cdp.errors, [], 'Unexpected browser runtime exceptions: ' + JSON.stringify(cdp.errors));
  } finally {
    if (cdp) cdp.close();
    const exited = new Promise((resolve) => chrome.once('exit', resolve));
    chrome.kill('SIGTERM');
    await Promise.race([exited, new Promise((r) => setTimeout(r, 1500))]);
    server.close();
    try { fs.rmSync(userDataDir, { recursive: true, force: true, maxRetries: 5, retryDelay: 100 }); } catch (_) {}
  }
});
