"""Own only this visualization's processes; keep existing field services untouched."""
import argparse,json,os,secrets,signal,socket,subprocess,sys,time
from pathlib import Path

HERE=Path(__file__).resolve().parent;FLEET=HERE.parent
STATE=Path(os.environ.get('DASHBOARD_STATE',str(Path.home()/'.local/state/capstone-dashboard-visualization')))
def identity(pid):
 try:return Path(f'/proc/{pid}/stat').read_text().rpartition(') ')[2].split()[19]
 except OSError:return None

def write_private(path,value):
 path.write_text(json.dumps(value,indent=2)+'\n');path.chmod(0o600)
def token(name):
 p=STATE/name
 if not p.exists():p.write_text(secrets.token_urlsafe(32)+'\n');p.chmod(0o600)
 return str(p)
def spawn(name,command,env,registry):
 with (STATE/(name+'.log')).open('ab') as f:
  p=subprocess.Popen(command,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True,stdin=subprocess.DEVNULL)
 registry[name]={'pid':p.pid,'start_ticks':identity(p.pid)};write_private(STATE/'processes.json',registry)
def main():
 ap=argparse.ArgumentParser();ap.add_argument('action',choices=['start','stop','status']);ap.add_argument('--source-config');ap.add_argument('--video-project');ap.add_argument('--yolo-project');args=ap.parse_args()
 if args.action in ['stop','status']:
  registry=json.loads((STATE/'processes.json').read_text()) if (STATE/'processes.json').exists() else {}
  if args.action=='status':
   for n,p in registry.items():print(n,p['pid'],'RUNNING' if identity(p['pid'])==p['start_ticks'] else 'STOPPED')
   return
  for n,p in reversed(list(registry.items())):
   if identity(p['pid'])==p['start_ticks']:os.kill(p['pid'],signal.SIGINT)
  time.sleep(2)
  for n,p in reversed(list(registry.items())):
   if identity(p['pid'])==p['start_ticks']:
    try:os.kill(p['pid'],signal.SIGTERM)
    except ProcessLookupError:pass
  print('Stopped only registered visualization processes');return
 if not args.source_config:ap.error('--source-config must point to a private local JSON file (never Git)')
 for port in [18210,18211,18220,18221]:
  s=socket.socket();s.settimeout(.2)
  busy=s.connect_ex(('127.0.0.1',port))==0;s.close()
  if busy:raise SystemExit('Refusing occupied visualization port '+str(port))
 STATE.mkdir(mode=0o700,parents=True,exist_ok=True);STATE.chmod(0o700)
 source=json.loads(Path(args.source_config).read_text());registry={}
 video=Path(args.video_project).expanduser() if args.video_project else Path.home()/'.local/share/robot-sim-projects/limo_video_ai_preverify'
 yolo=Path(args.yolo_project).expanduser() if args.yolo_project else Path.home()/'.local/share/robot-sim-projects/yolo_gazebo_limo'
 cfg={'viewer_token_file':token('viewer.token'),'bridge_token_file':token('bridge.token'),'port':18210,'profile':'live',
      **{k:source[k] for k in ['mission_url','mission_token_file','ai_url','ai_token_file']}}
 write_private(STATE/'live.json',cfg)
 test={k:v for k,v in cfg.items() if not k.startswith('mission_')};test.update(port=18211,profile='test',ai_url='http://127.0.0.1:18221',ai_token_file=token('rgb_viewer.token'))
 write_private(STATE/'test.json',test)
 topics={'selected_frontier':'/test/selected_frontier','explore':'/test/explore'};write_private(STATE/'test.topics.json',topics)
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',PYTHONPATH=str(FLEET),ROS_DOMAIN_ID='180',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(STATE/'ros_logs'))
 spawn('dashboard_live',[sys.executable,'-u','-m','dashboard.server','--config',str(STATE/'live.json')],env,registry)
 spawn('dashboard_test',[sys.executable,'-u','-m','dashboard.server','--config',str(STATE/'test.json')],env,registry)
 from shlex import quote
 prefix='source /opt/ros/humble/setup.bash; exec '
 spawn('synthetic_ros',['bash','-c',prefix+quote(sys.executable)+' -u '+quote(str(HERE/'tests/ros_fixture.py'))],env,registry)
 command=[sys.executable,'-u','-m','dashboard.bridge_node','--endpoint','http://127.0.0.1:18211/api/bridge','--token-file',str(STATE/'bridge.token'),'--kind','SYNTHETIC_ROS_TEST','--topics',str(STATE/'test.topics.json'),'--explore-json','--robot-id','viz-test']
 spawn('test_readonly_bridge',['bash','-c',prefix+' '.join(map(quote,command))],env,registry)
 if (yolo/'venv/bin/python').exists() and (video/'server/ai_server.py').exists():
  yolo_env=dict(env,YOLO_CONFIG_DIR=str(STATE/'ultralytics_config'))
  # Reuse the previously validated YOLO venv's normal site policy. Its typing_extensions
  # dependency is in the user's existing site; no package installation is performed.
  yolo_env.pop('PYTHONNOUSERSITE',None)
  spawn('replay_yolo_host',[str(yolo/'venv/bin/python'),'-u',str(video/'server/ai_server.py'),'--port','18220','--http-port','18221','--sender-token-file',token('rgb_sender.token'),'--viewer-token-file',str(STATE/'rgb_viewer.token'),'--weights',str(yolo/'weights/yolo11n.pt'),'--trust-sender-clock','1'],yolo_env,registry)
  spawn('recorded_gazebo_rgb',[sys.executable,'-u',str(HERE/'tests/replay_rgb.py'),'--project',str(video),'--token-file',str(STATE/'rgb_sender.token')],env,registry)
 print('Live dashboard: http://127.0.0.1:18210/')
 print('Synthetic ROS / recorded Gazebo RGB test: http://127.0.0.1:18211/')
 print('Login: select the viewer.token file from the private DASHBOARD_STATE folder. No token printed.')
if __name__=='__main__':main()
