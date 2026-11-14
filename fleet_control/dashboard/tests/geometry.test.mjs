import test from'node:test';import assert from'node:assert/strict';
import{gridToWorld,worldToGrid,worldToScreen,screenToWorld,compose,depthColor}from'../web/geometry.mjs';
const near=(a,b)=>assert.ok(Math.abs(a-b)<1e-9,`${a} != ${b}`);
test('OccupancyGrid origin yaw rotates cells, not ROS coordinates',()=>{const g={origin:{x:10,y:20,yaw:Math.PI/2},resolution:.5};const p=gridToWorld(g,2,0);near(p.x,10);near(p.y,21);});
test('rolling odom origin composes map TF and grid yaw',()=>{const g={origin:{x:1,y:2,yaw:.4},map_from_frame:{x:10,y:20,yaw:.3},resolution:.05};for(let i=0;i<100;i++){const p=gridToWorld(g,i*.7,i*.2),q=worldToGrid(g,p.x,p.y);near(q.u,i*.7);near(q.v,i*.2);}});
test('view rotation, zoom and pan are invertible',()=>{const v={width:1440,height:960,x:3,y:-2,panX:120,panY:-70,zoom:85,rotation:.78};for(let i=0;i<100;i++){const p={x:i/7,y:-i/11},q=screenToWorld(v,worldToScreen(v,p));near(p.x,q.x);near(p.y,q.y);}});
test('PNG top row corresponds to ROS highest grid row',()=>{const g={origin:{x:0,y:0,yaw:0},resolution:1,height:10};const pngTop=gridToWorld(g,0,g.height-0),pngBottom=gridToWorld(g,0,g.height-10);near(pngTop.y,10);near(pngBottom.y,0);});
test('display rotation never mutates original map origin',()=>{const origin=Object.freeze({x:3,y:4,yaw:.7}),g=Object.freeze({origin,resolution:.1});gridToWorld(g,4,6);assert.equal(origin.yaw,.7);});
test('invalid depth is distinct from valid near/far distances',()=>{assert.deepEqual(depthColor(NaN,.2,4),depthColor(0,.2,4));assert.notDeepEqual(depthColor(1,.2,4),depthColor(3,.2,4));assert.notDeepEqual(depthColor(1,.2,4),depthColor(1,.2,2));});
