'use strict';
// Uses the shipped input converter; no DOM, HTTP, or physical device.
const assert = require('node:assert/strict');
const M = require('../server/fleet_server/static/manual_control_panel.js');
const k = (...keys) => M.keyboardAxes(new Set(keys));
assert.deepEqual(k('KeyW','Space'), {linear:1,turn:0,deadman:true});
assert.deepEqual(k('KeyW','KeyA','Space'), {linear:1,turn:1,deadman:true});
assert.deepEqual(k('ArrowUp','ArrowRight','Space'), {linear:1,turn:-1,deadman:true});
assert.equal(k('KeyW').deadman, false);
const stick = M.pointerAxes(100,0,{left:0,top:0,width:200,height:200});
assert.equal(stick.linear,1); assert.equal(stick.deadman,true);
const pad=M.gamepadAxes({connected:true,axes:[-.5,-1],buttons:[null,null,null,null,{pressed:true}]});
assert.deepEqual(pad,{linear:1,turn:.5,deadman:true});
assert.equal(M.gamepadAxes({connected:true,axes:[NaN,0],buttons:[null,null,null,null,{}]}),null);
console.log('PASS: keyboard, combined direction, deadman, pointer geometry, gamepad axes, NaN');
console.log('Scope: pure input functions only; no event release, physical controller, browser, HTTP or ROS.');
