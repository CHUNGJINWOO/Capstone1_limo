"""SIM_ONLY SLAM Toolbox supervisor; no motor, Nav2, or Explore process."""
import fcntl,hashlib,json,math,os,signal,subprocess,threading,time
from pathlib import Path
from rclpy.time import Time
from .recording_session import SESSION_ID

class SlamControl:
 def __init__(self,root,robot):
  self.root=Path(root).resolve();self.root.mkdir(parents=True,mode=0o700,exist_ok=True)
  self.guard=(self.root/'.slam.lock').open('a');fcntl.flock(self.guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
  self.robot=robot;self.lock=threading.RLock();self.proc=None;self.log=None;self.started=0.;self.state_file=self.root/'current.json';self.manual_mode=False;self.stop_observations={}
  self.state={'enabled':True,'status':'IDLE','session_id':None,'detail':'SIM_ONLY'}
  if self.state_file.exists():
   try:
    self.state=json.loads(self.state_file.read_text())
    if self.state.get('status') not in ('STOPPED','COMPLETED','FAILED','IDLE'):
     self.state.update(status='UNKNOWN',detail='Agent restarted; process ownership unknown');self._save()
   except Exception:self.state={'enabled':True,'status':'UNKNOWN','session_id':None,'detail':'state unreadable'}
 def enable_manual_stop(self):
  from geometry_msgs.msg import Twist
  from nav_msgs.msg import Odometry,OccupancyGrid
  from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
  self.manual_mode=True;self.manual_grid=None
  def cmd(m):
   with self.lock:self.stop_observations['cmd']=(time.monotonic(),float(m.linear.x),float(m.angular.z))
  def gt(m):
   with self.lock:self.stop_observations['gt']=(time.monotonic(),math.hypot(float(m.twist.twist.linear.x),float(m.twist.twist.linear.y)),float(m.twist.twist.angular.z))
  def grid(m):
   q=m.info.origin.orientation;p=m.info.origin.position
   with self.lock:self.manual_grid={'width':m.info.width,'height':m.info.height,'resolution':float(m.info.resolution),'origin':[float(p.x),float(p.y),math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],'cells':list(m.data),'arrival':time.monotonic()}
  self.robot.create_subscription(OccupancyGrid,'/map',grid,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
  self.robot.create_subscription(Twist,'/cmd_vel',cmd,20)
  self.robot.create_subscription(Odometry,'/ground_truth/world_odom',gt,20)
 def _confirm_manual_stop(self,timeout=12):
  started=time.monotonic();deadline=started+timeout;zero_since=None
  while time.monotonic()<deadline:
   now=time.monotonic()
   with self.lock:
    cmd=self.stop_observations.get('cmd');gt=self.stop_observations.get('gt')
   pubs=self.robot.get_publishers_info_by_topic('/cmd_vel')
   valid=(len(pubs)==1 and pubs[0].node_name=='stall_detector_node' and cmd and gt
          and cmd[0]>=started and gt[0]>=started and now-cmd[0]<.5 and now-gt[0]<.5
          and abs(cmd[1])<.001 and abs(cmd[2])<.001 and abs(gt[1])<.01 and abs(gt[2])<.02)
   if valid:
    if zero_since is None:zero_since=now
    if now-zero_since>=1.:return True
   else:zero_since=None
   time.sleep(.05)
  return False
 def _finish_manual(self,sid,floor,cid,ack):
  if not self._confirm_manual_stop():
   with self.lock:self.set('ERROR','stop confirmation failed; input remains disabled')
   ack('FAILED','stop confirmation failed');return
  if not isinstance(floor,str) or not floor:
   with self.lock:self.set('ERROR','stopped; registered floor required for saving')
   ack('FAILED','registered floor required for saving');return
  with self.lock:self.set('SAVING','Gate output and Gazebo stop confirmed')
  result=[]
  try:self._save_map(sid,floor,cid,lambda *args:result.append(args),manual=True)
  except Exception as exc:
   with self.lock:self.set('ERROR','save failed: '+(type(exc).__name__+': '+str(exc))[:240])
   ack('FAILED','save failed');return
  if not result or result[-1][0]!='SUCCEEDED':
   ack('FAILED',result[-1][1] if result else 'save failed');return
  with self.lock:self.set('STOPPING','map registered; stopping SLAM')
  self._stop(ack,manual=True)
 def _save(self):
  p=self.root/'.current.tmp';fd=os.open(p,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600)
  with os.fdopen(fd,'w') as f:json.dump(self.state,f);f.flush();os.fsync(f.fileno())
  os.replace(p,self.state_file);fd=os.open(self.root,os.O_RDONLY|os.O_DIRECTORY)
  try:os.fsync(fd)
  finally:os.close(fd)
 def set(self,status,detail,**kw):
  self.state.update(status=status,detail=detail,**kw);self._save()
 def _owned_map_visible(self):
  pubs=self.robot.get_publishers_info_by_topic('/map')
  return len(pubs)==1 and getattr(pubs[0],'node_name',None)=='slam_toolbox'
 def snapshot(self):
  with self.lock:
   if self.proc and self.state['status'] in ('STARTING','RUNNING','DEGRADED'):
    if self.proc.poll() is not None:self.set('FAILED','SLAM process exited',returncode=self.proc.returncode)
    elif self.state['status'] in ('RUNNING','DEGRADED'):
     s=self.robot.snapshot();m=s.get('map') or {};pose=s.get('pose') or {};lidar=s.get('lidar') or {}
     fresh=(isinstance(m.get('last_message_age_s'),(int,float)) and 0<=m['last_message_age_s']<=12 and pose.get('status')=='OK' and isinstance(pose.get('age_s'),(int,float)) and 0<=pose['age_s']<2 and lidar.get('status')=='OK')
     if not fresh and self.state['status']=='RUNNING':self.set('DEGRADED','map, TF or scan is stale')
     elif fresh and self.state['status']=='DEGRADED':self.set('RUNNING','fresh map, scan and TF observed')
   return dict(self.state)
 def command(self,cmd,ack):
  sid=(cmd.get('payload') or {}).get('session_id');typ=cmd.get('type')
  if not isinstance(sid,str) or not SESSION_ID.fullmatch(sid):ack('REJECTED','invalid session');return
  with self.lock:
   st=self.snapshot();status=st['status']
   if typ=='slam_start':
    if status not in ('IDLE','STOPPED','COMPLETED','FAILED') or sid==st.get('session_id'):ack('REJECTED','active, unknown or used session');return
    graph_nodes=self.robot.get_node_names_and_namespaces();nodes={n for n,ns in graph_nodes}
    blocked={'slam_toolbox','map_server','amcl','explore','explore_node'}
    try:
     # An existing map->odom, including a non-AMCL broadcaster, conflicts with SLAM.
     if self.robot.tfb.can_transform('map','odom',Time()):
      raise RuntimeError('map->odom already present')
     actions={'navigate_to_pose','navigate_through_poses','follow_path','spin','backup'}
     services={name for node,ns in graph_nodes for name,_ in self.robot.get_service_names_and_types_by_node(node,ns)}
     service_actions={name.split('/_action/send_goal')[0].lstrip('/') for name in services if name.endswith('/_action/send_goal')}
     if service_actions-actions:raise RuntimeError('unobserved action server present')
     present={a for a in actions if self.robot.count_publishers('/'+a+'/_action/status')}
     if service_actions-present:raise RuntimeError('action status publisher unavailable')
     if present:
      gate=self.robot._gate_evaluation(); ev=gate.get('nav_evidence') or {}; servers=ev.get('servers') or {}
      if (gate.get('contract_version') != 'gate-status/1.2-sim' or gate.get('trust') != 'UNAUTHENTICATED_SIM_ONLY'
          or gate.get('contract_met') is not True or gate.get('r7_invalid') is not False
          or not gate.get('rx_fresh') or not gate.get('source_fresh') or not gate.get('single_publisher_graph')
          or ev.get('scope_version') != 1 or ev.get('observed') is not True or ev.get('all_idle_proven') is not True
          or ev.get('active_uuids') or not present.issubset(servers)):
       raise RuntimeError('Nav2 idle evidence unavailable or active goal')
      for name in present:
       record=servers[name]; pubs=self.robot.get_publishers_info_by_topic('/'+name+'/_action/status')
       if (record.get('verdict') not in ('IDLE_PROVEN_EL','IDLE_PROVEN_ES') or len(pubs)!=1
           or bytes(pubs[0].endpoint_gid).hex()[:16]!=record.get('writer_gid')):
        raise RuntimeError('Nav2 status ownership or idle evidence ambiguous')
     if getattr(self.robot,'active',None):raise RuntimeError('Agent goal active')
    except Exception as e:
     self.set(status,'safety inspection failed: '+str(e));ack('REJECTED','safety inspection failed: '+str(e));return
    if nodes & blocked or self.robot.count_publishers('/map'):
     self.set(status,'SLAM/localization/map producer already detected');ack('REJECTED','SLAM/localization/map producer already detected');return
    directory=self.root/sid
    try:
     directory.mkdir(mode=0o700);self.set('STARTING','launch requested',session_id=sid,save_path=None,map_version_id=None,floor_id=None,posegraph_saved=False,posegraph_registered=False,returncode=None)
     self.log=(directory/'slam.log').open('xb');self.started=time.monotonic()
     self.proc=subprocess.Popen(['ros2','launch','slam_toolbox','online_async_launch.py','use_sim_time:='+str(self.robot._use_sim_time()).lower()],stdin=subprocess.DEVNULL,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
    except Exception as e:self.set('UNKNOWN','launch outcome unknown: '+type(e).__name__);ack('RECONCILIATION_REQUIRED','launch unknown');return
    ack('ACCEPTED','launch requested');threading.Thread(target=self._wait_ready,args=(ack,),daemon=True).start()
   elif typ=='slam_save':
    if sid!=st.get('session_id') or status!='RUNNING':ack('REJECTED','matching running session required');return
    floor=(cmd.get('payload') or {}).get('floor_id');cid=cmd.get('command_id')
    if not isinstance(floor,str) or not floor:ack('REJECTED','registered floor required');return
    self.set('SAVING','save requested');ack('ACCEPTED','saving');threading.Thread(target=self._save_map,args=(sid,floor,cid,ack),daemon=True).start()
   elif typ=='slam_stop' and self.manual_mode:
    floor=(cmd.get('payload') or {}).get('floor_id');cid=cmd.get('command_id')
    if sid!=st.get('session_id') or status not in ('RUNNING','DEGRADED','STARTING','ERROR'):ack('REJECTED','matching session required');return
    self.set('STOPPING','manual input disabled; confirming zero output and Gazebo stop');ack('ACCEPTED','stopping and saving');threading.Thread(target=self._finish_manual,args=(sid,floor,cid,ack),daemon=True).start()
   elif typ=='slam_stop':
    if sid!=st.get('session_id') or status not in ('RUNNING','DEGRADED','STARTING'):ack('REJECTED','matching owned session required');return
    self.set('STOP_REQUESTED','stopping');ack('ACCEPTED','stop requested');threading.Thread(target=self._stop,args=(ack,),daemon=True).start()
   else:ack('REJECTED','unknown command')
 def _wait_ready(self,ack):
  deadline=time.monotonic()+45
  while time.monotonic()<deadline:
   time.sleep(.2)
   with self.lock:
    if self.state['status']!='STARTING':return
    if self.proc.poll() is not None:self.set('FAILED','SLAM process exited');ack('FAILED','SLAM exited');return
    s=self.robot.snapshot();m=s.get('map') or {};pose=s.get('pose') or {};lidar=s.get('lidar') or {}
    if (self.robot.t.get('map',0)>=self.started and self._owned_map_visible() and
        m.get('frame_id')=='map' and m.get('known_cells',0)>0 and isinstance(m.get('last_message_age_s'),(int,float)) and 0<=m['last_message_age_s']<3 and pose.get('status')=='OK' and isinstance(pose.get('age_s'),(int,float)) and 0<=pose['age_s']<2 and lidar.get('status')=='OK'):
     self.set('RUNNING','fresh map, scan and TF observed');ack('SUCCEEDED','SLAM ready');return
  with self.lock:self.set('UNKNOWN','SLAM readiness timeout; process may still run')
  ack('RECONCILIATION_REQUIRED','readiness unknown')
 def _save_map(self,sid,floor,cid,ack,manual=False):
  directory=self.root/sid
  if manual:
   directory=directory/'manual_final';directory.mkdir(mode=0o700,exist_ok=True)
   candidate=directory/'map'
   if (candidate.with_suffix('.yaml').exists() or candidate.with_suffix('.pgm').exists()) and not (directory/'validated.json').is_file():
    n=1
    while True:
     retry=self.root/sid/('manual_final_retry_'+str(n))
     try:retry.mkdir(mode=0o700);directory=retry;break
     except FileExistsError:n+=1
  prefix=directory/'map'
  exists=[prefix.with_suffix(ext).is_file() for ext in ('.yaml','.pgm')]
  if any(exists) and (not manual or not all(exists)):
   with self.lock:self.set('ERROR' if manual else 'RUNNING','existing or incomplete map not overwritten')
   ack('FAILED' if manual else 'REJECTED','existing or incomplete map not overwritten');return
  try:
   source=None
   if manual and all(exists):
    marker=json.loads((directory/'validated.json').read_text())
    if any(hashlib.sha256(prefix.with_suffix(ext).read_bytes()).hexdigest()!=marker[ext] for ext in ('.yaml','.pgm')):raise RuntimeError('saved map changed after validation')
   if manual and not all(exists):
    with self.lock:source=dict(self.manual_grid) if self.manual_grid else None
    if not source or time.monotonic()-source['arrival']>12 or any(v not in (-1,0,100) for v in source['cells']):raise RuntimeError('fresh trinary SLAM grid required')
   if self.proc is None or self.proc.poll() is not None or not self._owned_map_visible():raise RuntimeError('owned SLAM map unavailable')
   if not all(exists):
    args=['ros2','run','nav2_map_server','map_saver_cli','-f',str(prefix)]
    # Nav2's PGM unknown value 205 must lie between the YAML thresholds.
    if manual:args.extend(['--free','0.1'])
    res=subprocess.run(args,capture_output=True,timeout=30)
    if res.returncode or not prefix.with_suffix('.yaml').is_file() or not prefix.with_suffix('.pgm').is_file() or not prefix.with_suffix('.pgm').stat().st_size:raise RuntimeError('save failed')
   if manual and source is not None:
    from PIL import Image
    import yaml
    with self.lock:latest=self.manual_grid
    if not latest or any(latest[k]!=source[k] for k in ('width','height','resolution','origin','cells')):raise RuntimeError('SLAM grid changed during save; saved files retained')
    with Image.open(prefix.with_suffix('.pgm')) as image:
     if image.size!=(source['width'],source['height']):raise RuntimeError('saved dimensions differ from SLAM grid')
     px=list(image.convert('L').getdata());w=source['width'];h=source['height']
    decoded=[100 if v==0 else 0 if v>=254 else -1 if v==205 else -999 for y in range(h-1,-1,-1) for v in px[y*w:(y+1)*w]]
    if decoded!=source['cells']:raise RuntimeError('saved pixels differ from SLAM grid')
    meta=yaml.safe_load(prefix.with_suffix('.yaml').read_text());meta['resolution']=source['resolution'];meta['origin']=source['origin']
    prefix.with_suffix('.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
    (directory/'validated.json').write_text(json.dumps({ext:hashlib.sha256(prefix.with_suffix(ext).read_bytes()).hexdigest() for ext in ('.yaml','.pgm')}))
   if self.proc.poll() is not None or not self._owned_map_visible():raise RuntimeError('SLAM stopped during save')
   graph=False
   try:
    if all(prefix.with_suffix(ext).is_file() for ext in ('.posegraph','.data')):raise FileExistsError('existing posegraph preserved')
    serialized=subprocess.run(['ros2','service','call','/slam_toolbox/serialize_map','slam_toolbox/srv/SerializePoseGraph',json.dumps({'filename':str(prefix)})],capture_output=True,text=True,timeout=20)
    graph=(serialized.returncode==0 and 'result=0' in serialized.stdout and all(prefix.with_suffix(ext).is_file() and prefix.with_suffix(ext).stat().st_size>0 for ext in ('.posegraph','.data')))
   except (OSError,subprocess.TimeoutExpired):pass
   result=self.upload_map(sid,floor,cid,prefix.with_suffix('.yaml'),prefix.with_suffix('.pgm'),prefix if graph else None)
   with self.lock:self.set('SAVING' if manual else 'RUNNING','map saved; registry UNVERIFIED',save_path=str(prefix)+'.yaml',map_version_id=result['map_version']['map_version_id'],floor_id=floor,posegraph_saved=graph,posegraph_registered=bool(result['map_version'].get('sha256',{}).get('posegraph_sha256')))
   ack('SUCCEEDED','map files saved and registered UNVERIFIED')
  except Exception as e:
   existing=str(prefix)+'.yaml' if prefix.with_suffix('.yaml').is_file() else None
   detail=(type(e).__name__+': '+str(e))[:240]
   with self.lock:self.set('ERROR' if manual else 'RUNNING','save or registration failed: '+detail,save_path=existing)
   ack('FAILED','map save failed')
 def _stop(self,ack,manual=False):
  try:
   os.killpg(self.proc.pid,signal.SIGINT);rc=self.proc.wait(timeout=20)
   deadline=time.monotonic()+30
   while time.monotonic()<deadline:
    if 'slam_toolbox' not in {n for n,ns in self.robot.get_node_names_and_namespaces()} and self.robot.count_publishers('/map')==0:break
    time.sleep(.2)
   else:raise RuntimeError('SLAM node still visible')
   with self.lock:self.set('COMPLETED' if manual else 'STOPPED','launch exited and SLAM node absent',returncode=rc)
   ack('SUCCEEDED','SLAM launch exited')
  except Exception as e:
   with self.lock:self.set('UNKNOWN','stop outcome unknown: '+type(e).__name__+': '+str(e))
   ack('RECONCILIATION_REQUIRED','stop outcome unknown')
 def close(self):
  if self.log:self.log.close()
  self.guard.close()
