'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

async function main() {
  const elements = new Map();
  for (const [id, value] of Object.entries({
    gpDevice: '0', gpRobot: 'iso', gpLinear: 0, gpTurn: 1, gpDeadman: 0,
    gpCalV: '-1,0,1', gpCalW: '-1,0,1', gpMaxLinear: .1, gpMaxTurn: .3, gpDeadzone: .12
  })) elements.set(id, {value, valueAsNumber: value});
  const lookup = id => {
    if (!elements.has(id)) elements.set(id, {addEventListener() {}});
    return elements.get(id);
  };
  const pad = {connected: true, index: 0, id: 'mock', axes: [0, 0],
    buttons: [{pressed: false}], timestamp: 1};
  const requests = [];
  let tick;
  vm.runInNewContext(fs.readFileSync(path.join(__dirname,
    '../server/fleet_server/static/gamepad_panel.js'), 'utf8'), {
    $: lookup,
    window: {isSecureContext: true, addEventListener() {}},
    document: {hidden: false, hasFocus: () => true, addEventListener() {}},
    navigator: {getGamepads: () => [pad]},
    AbortSignal,
    performance,
    setInterval: callback => { tick = callback; },
    api: async (_path, options) => {
      if (!options.body) return {ok: true, json: async () => ({lease: {epoch: 7}})};
      const body = JSON.parse(options.body);
      requests.push(body);
      return {ok: true, json: async () => body.action === 'start'
        ? {session_id: 'session'} : {received: body.seq}};
    }
  });
  await lookup('gamepadForm').onsubmit({preventDefault() {}});
  assert.deepEqual(requests.shift(), {action: 'start', epoch: 7});
  pad.axes = [.75, -.5];
  pad.buttons[0].pressed = true;
  pad.timestamp++;
  await tick();
  await tick();
  assert.deepEqual(requests[0], {action: 'sample', epoch: 7, session_id: 'session',
    input_format: 'normalized_axes_v1', seq: 1, deadman: true, linear: .75, turn: -.5});
  assert.equal(requests[1].seq, 2);
  pad.buttons[0].pressed = false;
  await tick();
  assert.deepEqual(requests[2], {action: 'stop', epoch: 7, session_id: 'session'});
  console.log('PASS Gamepad v5 payload, increasing sequence, and Deadman STOP');
}
main().catch(error => { console.error(error); process.exitCode = 1; });
