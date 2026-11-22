import test from 'node:test';import assert from 'node:assert/strict';import {FrameGate,alignedResult} from '../web/geometry.mjs';
const h=(v={})=>({boot:'a',frame_id:'rgb',sequence:1,stamp_ns:'1794572440657407536',capture_wall_ms:1000,...v});
test('old frames rejected before decode',()=>assert.equal(new FrameGate().accept(h(),1800),false));
test('duplicates and out of order rejected',()=>{const g=new FrameGate();assert(g.accept(h(),1001));assert(!g.accept(h(),1002));assert(!g.accept(h({sequence:0}),1003));});
test('new boot may reset sequence; retired boot never returns',()=>{const g=new FrameGate();assert(g.accept(h({sequence:8}),1001));assert(g.accept(h({boot:'b'}),1002));assert(!g.accept(h({sequence:9}),1003));});
test('future or invalid clock not reported fresh',()=>{assert(!new FrameGate().accept(h(),500));assert(!new FrameGate().accept(h({capture_wall_ms:NaN}),1001));});
test('nanoseconds must remain exact strings',()=>assert(!new FrameGate().accept(h({stamp_ns:1794572440657407536}),1001)));
test('YOLO needs exact complete frame key',()=>{const a=h();a.result_key=['a','rgb',1,'rgb',a.stamp_ns];assert(alignedResult(a));assert(!alignedResult({...a,stamp_ns:'1794572440657407537'}));assert(!alignedResult({...a,frame_id:'other'}));assert(!alignedResult({...a,boot:'old'}));});
