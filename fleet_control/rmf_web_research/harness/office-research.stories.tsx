// Capstone research harness only: official components + explicit offline fixtures.
import React from 'react';
import type {Meta,StoryObj} from '@storybook/react-vite';
import {BehaviorSubject} from 'rxjs';
import {Box,Typography,Paper} from '@mui/material';
import {RmfApiProvider} from '../src/hooks';
import {MockRmfApi} from '../src/utils/test-utils.test';
import {Map} from '../src/components/map/map';
import {officeMap} from '../src/components/map/test-utils.test';
import {RobotDataGridTable} from '../src/components/robots/robot-table-datagrid';
import {RobotSummary} from '../src/components/robots/robot-summary';
import {TaskTable} from '../src/components/tasks/task-table';
import {TaskTimeline} from '../src/components/tasks/task-timeline';
import {makeTaskState} from '../src/components/tasks/test-data.test';
import FakeTrajectoryManager from '../src/services/__mocks__/robot-trajectory-manager';
const task=makeTaskState('OFFLINE_RESEARCH_TASK');
task.status='underway' as any; task.assigned_to={group:'research_fleet',name:'demo_robot_1'};
const now=Date.now();task.unix_millis_start_time=now-120000; task.unix_millis_finish_time=now+180000;task.estimate_millis=180000;
function OfficeFixture(){
 const api=React.useMemo(()=>{
  const a=new MockRmfApi();const map=structuredClone(officeMap);
  const second=structuredClone(map.levels[0]);second.name='L2_OFFLINE_COPY';map.levels.push(second);
  const fleet={name:'research_fleet',robots:{
   demo_robot_1:{name:'demo_robot_1',status:'working',task_id:task.booking.id,battery:.72,unix_millis_time:now,location:{map:'L1',x:10.2,y:-3.1,yaw:0},commission:{dispatch_tasks:true,direct_tasks:true,idle_behavior:true}},
   demo_robot_2:{name:'demo_robot_2',status:'idle',task_id:'',battery:.48,unix_millis_time:now,location:{map:'L1',x:16.8,y:-6.8,yaw:1.2},commission:{dispatch_tasks:true,direct_tasks:true,idle_behavior:true}}
  }};
  a.buildingMapObs=new BehaviorSubject(map); a.fleetsObs=new BehaviorSubject([fleet]) as any;
  a.fleetStateObsStore.research_fleet=new BehaviorSubject(fleet) as any;
  a.taskStateObsStore[task.booking.id]=new BehaviorSubject(task);
  a.trajectoryManager=new FakeTrajectoryManager() as any;
  return a;
 },[]);
 const rows=[{fleet:'research_fleet',name:'demo_robot_1',status:'working',battery:.72,level:'L1',lastUpdateTime:now,estFinishTime:now+180000},{fleet:'research_fleet',name:'demo_robot_2',status:'idle',battery:.48,level:'L1',lastUpdateTime:now}];
 return <RmfApiProvider value={api}><Box sx={{p:1,bgcolor:'#edf0f4',minHeight:'100vh'}}>
  <Typography sx={{p:1,bgcolor:'#9a5a06',color:'white'}}>OFFLINE FIXTURE · Official RMF Web components · stored Office map + synthetic robots/tasks + official trajectory mock · NO ROS / NO LIVE CONTROL</Typography>
  <Box sx={{display:'grid',gridTemplateColumns:'minmax(400px, 1.4fr) minmax(350px, 1fr)',gap:1,mt:1}}>
   <Paper sx={{height:620,position:'relative',overflow:'hidden'}}><Map defaultMapLevel="L1" defaultZoom={6} defaultRobotZoom={20} attributionPrefix="Open-RMF · OFFLINE FIXTURE"/></Paper>
   <Box><Paper sx={{height:260}}><RobotDataGridTable robots={rows as any}/></Paper><Paper sx={{mt:1,overflow:'auto'}}><TaskTable tasks={[task]}/></Paper><Paper sx={{mt:1,p:1}}><TaskTimeline taskState={task}/></Paper></Box>
  </Box>
 </Box></RmfApiProvider>;
}
function RobotFixture(){
 const api=React.useMemo(()=>{const a=new MockRmfApi();a.fleetStateObsStore.research_fleet=new BehaviorSubject({name:'research_fleet',robots:{demo_robot_1:{name:'demo_robot_1',status:'working',task_id:task.booking.id,battery:.72,unix_millis_time:now,location:{map:'L1',x:10,y:-3,yaw:0},commission:{dispatch_tasks:true,direct_tasks:true,idle_behavior:true}}}}) as any;a.taskStateObsStore[task.booking.id]=new BehaviorSubject(task);return a;},[]);
 return <RmfApiProvider value={api}><Typography>OFFLINE FIXTURE — no backend, no command connection</Typography><RobotSummary robot={{fleet:'research_fleet',name:'demo_robot_1'} as any} onClose={()=>{}}/></RmfApiProvider>;
}
export default {title:'Capstone Offline/Office Research',parameters:{layout:'fullscreen'}} satisfies Meta;
export const MapRobotsTasks:StoryObj={render:()=> <OfficeFixture/>};
export const PopulatedRobotSummary:StoryObj={render:()=> <RobotFixture/>};
