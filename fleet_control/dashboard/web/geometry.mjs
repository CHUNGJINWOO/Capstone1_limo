export function compose(a,b){const c=Math.cos(a.yaw),s=Math.sin(a.yaw);return{x:a.x+c*b.x-s*b.y,y:a.y+s*b.x+c*b.y,yaw:a.yaw+b.yaw};}
export function gridOrigin(g){return compose(g.map_from_frame||{x:0,y:0,yaw:0},g.origin);}
export function gridToWorld(g,u,v){const o=gridOrigin(g),c=Math.cos(o.yaw),s=Math.sin(o.yaw);return{x:o.x+g.resolution*(c*u-s*v),y:o.y+g.resolution*(s*u+c*v)};}
export function worldToGrid(g,x,y){const o=gridOrigin(g),c=Math.cos(o.yaw),s=Math.sin(o.yaw),dx=x-o.x,dy=y-o.y;return{u:(c*dx+s*dy)/g.resolution,v:(-s*dx+c*dy)/g.resolution};}
export function worldToScreen(view,p){const c=Math.cos(view.rotation),s=Math.sin(view.rotation),x=p.x-view.x,y=p.y-view.y;return{x:view.width/2+view.panX+view.zoom*(c*x-s*y),y:view.height/2+view.panY-view.zoom*(s*x+c*y)};}
export function screenToWorld(view,p){const c=Math.cos(view.rotation),s=Math.sin(view.rotation),x=(p.x-view.width/2-view.panX)/view.zoom,y=-(p.y-view.height/2-view.panY)/view.zoom;return{x:view.x+c*x+s*y,y:view.y-s*x+c*y};}
export function depthColor(v,lo,hi){if(!Number.isFinite(v)||v<=0)return[21,28,39,255];const t=Math.max(0,Math.min(1,(v-lo)/(hi-lo))),stops=[[60,73,150],[64,185,186],[231,206,110],[227,110,80]],p=t*3,i=Math.min(2,Math.floor(p)),f=p-i;return stops[i].map((x,j)=>Math.round(x*(1-f)+stops[i+1][j]*f)).concat(255);}

// Keep Mission Manager/Agent connectivity distinct from fresh read-only ROS pose.
export function liveRobots(data){
 const robots=data.mission?.robots||[],v=data.visualization||{};
 if(data.profile!=='live'||v.kind!=='ROS_LIVE')return robots;
 return robots.map(robot=>{
  if(robot.robot_id!==v.robot_id)return robot;
  const p=v.robot_pose,state={...(robot.state||{})},g=v.layers?.map;
  const valid=p&&['x','y','yaw_rad'].every(k=>Number.isFinite(p[k]));
  if(valid)state.pose={...p,frame_id:'map',status:v.connected?'OK':'STALE',age_s:v.receive_age_s,source:'READONLY_TF'};
  if(!valid&&g)state.pose=null;
  if(g)state.map={...state.map,status:g.status,source:'READONLY_ROS'};
  return {...robot,state};
 });
}
