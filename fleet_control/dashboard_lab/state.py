"""Read models only. Fixture missions never reach Mission Manager or a robot."""
import copy, math, time
import numpy as np
from dashboard.core import Store,pack

class ReadModel:
 def __init__(self,hub,recorded=None):self.hub=hub;self.recorded=recorded;self.store=Store('test');self.sequence=0;self.trails={};self.last_update=0;self.last_mode=None
 def update(self):
  now=time.time();mode=self.hub.mode
  if self.last_mode!=mode:self.store=Store('test');self.trails={};self.sequence=0;self.last_mode=mode
  self.sequence+=1
  if mode=='LIVE':return {'profile':'live','source_mode':'LIVE','source_label':'LIVE · 로봇 연결 없음 / 읽기 전용 수신 미설정','mission':{'connected':False,'robots':[],'missions':[],'events':[]},'visualization':{'kind':None,'connected':False,'layers':{},'paths':{},'dynamic_obstacles':{'status':'UNIMPLEMENTED'}},'rmf':{'status':'UNAVAILABLE','registered':None},'video':{'fresh':False},'capabilities':{'mission_create':False,'mission_cancel':False,'motor_control':False}}
  t=now-self.hub.origin;replay=mode=='REPLAY' and self.recorded is not None
  robots=[];missions=[];events=[]
  for i in range(2):
   rid=f'sim-{i+1}';online=(i==0);phase=t/22+i if online else float(i);x=math.cos(phase)*1.6+i*.8;y=math.sin(phase)*1.1
   path=self.trails.setdefault(rid,[])
   if not path or math.dist(path[-1],[x,y])>.02:path.append([x,y]);self.trails[rid]=path[-1000:]
   robots.append({'robot_id':rid,'online':online,'last_seen_age_s':.1 if online else 42.+t,'agent':{'mode':'simulation','simulated':True},'state':{'pose':{'x':x,'y':y,'yaw_rad':(phase+math.pi)%(2*math.pi)-math.pi,'frame_id':'map','status':'OK' if online else 'STALE','age_s':.1 if online else 42},'map':{'status':'REPLAY' if replay else 'SIMULATION'},'nav2':{'status':'SIMULATION'},'safety_gate':{'status':'UNAVAILABLE'},'battery':{'voltage_v':None}},'source':'SIMULATION_FIXTURE'})
   missions.append({'mission_id':f'fixture_{i+1}','robot_id':rid,'status':'EXECUTING' if online else 'QUEUED','terminal':False,'created_at':self.hub.origin-90-i*10,'updated_at':now-2,'goal':{'frame_id':'map','x':2.2-i,'y':1.3+i*.6},'source':'SIMULATION_FIXTURE','history':[{'at':self.hub.origin-90,'status':'QUEUED'},{'at':self.hub.origin-88,'status':'ACCEPTED'},{'at':self.hub.origin-87,'status':'EXECUTING'}] if online else [{'at':self.hub.origin-100,'status':'QUEUED'}]})
  missions.append({'mission_id':'fixture_finished','robot_id':'sim-1','status':'SUCCEEDED','terminal':True,'created_at':self.hub.origin-180,'updated_at':self.hub.origin-120,'goal':{'frame_id':'map','x':-1,'y':.5},'source':'SIMULATION_FIXTURE','history':[{'at':self.hub.origin-180,'status':'QUEUED'},{'at':self.hub.origin-175,'status':'EXECUTING'},{'at':self.hub.origin-120,'status':'SUCCEEDED'}]})
  for i in range(30):events.append({'id':i,'ts':self.hub.origin-i*7,'kind':['mission','link','sensor','adapter'][i%4],'severity':['INFO','WARNING','ERROR'][i%3],'robot_id':'sim-1' if i%2==0 else 'sim-2','message':['임무 상태 갱신 (시험)','영상 지연 감지 (시험)','센서 시험 이벤트','Adapter 등록 상태 시험'][i%4],'source':'SIMULATION_FIXTURE'})
  if replay:
   p=copy.deepcopy(self.recorded);p.update(kind='GAZEBO_REPLAY',session='lab-replay',sequence=self.sequence)
   for g in p.get('grids',{}).values():g['source_age_s']=None
   p['paths']={};p.pop('depth',None)
  else:
   a=np.zeros((110,140),np.int8);a[[0,-1],:]=100;a[:,[0,-1]]=100;a[30:80,65:70]=100;a[30:35,30:100]=100
   base={'width':140,'height':110,'resolution':.05,'origin':{'x':-3.5,'y':-2.75,'yaw':.22},'frame_id':'map','stamp_ns':time.time_ns(),'source_age_s':0,'cells_zlib':pack(a.tobytes())}
   costs=a.copy();costs[40:45,40:45]=80
   p={'kind':'SYNTHETIC_ROS_TEST','session':'lab-simulation','sequence':self.sequence,'grids':{'map':base,'global_costmap':dict(base,cells_zlib=pack(costs.tobytes())),'local_costmap':dict(base,cells_zlib=pack(costs.tobytes()),frame_id='odom',map_from_frame={'x':0,'y':0,'yaw':0})},'paths':{}}
  r=robots[0]['state']['pose'];goal=missions[0]['goal'];plan=[[r['x']+(goal['x']-r['x'])*v/20,r['y']+(goal['y']-r['y'])*v/20] for v in range(21)]
  p['paths']={'global_plan':{'points':plan,'frame_id':'map','stamp_ns':time.time_ns(),'source_age_s':0,'renderable':not replay,'source':'SIMULATION_FIXTURE'},'trail':{'points':self.trails['sim-1'],'frame_id':'map','source_age_s':0,'renderable':not replay}}
  # Saved real map has a different SLAM session. Do not plot fixture robot paths on it.
  if replay:
   for rob in robots:rob['state']['pose']=None
   for mission in missions:mission['goal']['frame_id']='simulation_map_unregistered'
  p.update(robot_id='sim-1',robot_pose=r if not replay else None,ros_time_ns=time.time_ns());self.store.ingest(p);visual=self.store.snapshot()
  return {'profile':'test','source_mode':mode,'source_label':('REPLAY · 저장 실차 지도/Depth + Gazebo RGB · 임무/로봇은 별도 SIMULATION 예제' if replay else 'SIMULATION · 합성 지도/Costmap/로봇/임무 + 저장 Gazebo RGB 재생'),'mission':{'connected':True,'robots':robots,'missions':missions,'events':events,'source':'SIMULATION_FIXTURE'},'visualization':visual,'rmf':{'status':'SIMULATION','registered':True,'fleet':'fixture_fleet','robots':['sim-1','sim-2'],'last_seen_s':.1,'source':'SIMULATION_FIXTURE','note':'실제 RMF 등록 검증 아님'},'video':{},'capabilities':{'mission_create':False,'mission_cancel':False,'motor_control':False},'fixture_packet':p,'robot_trails':copy.deepcopy(self.trails)}
