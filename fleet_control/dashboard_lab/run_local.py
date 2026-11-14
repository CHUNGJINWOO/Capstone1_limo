"""Own only registered lab processes; never inspect/connect to a real robot."""
import argparse,json,os,secrets,signal,socket,subprocess,sys,time
from pathlib import Path
STATE=Path(os.environ.get('DASHBOARD_LAB_STATE',str(Path.home()/'.local/state/capstone-dashboard-lab')))
ROOT=Path(__file__).resolve().parent;FLEET=ROOT.parent

def identity(pid):
 try:
  fields=Path(f'/proc/{pid}/stat').read_text().rpartition(') ')[2].split();return None if fields[0]=='Z' else fields[19]
 except OSError:return None

def private(path,data):path.write_text(json.dumps(data,indent=2)+'\n');path.chmod(0o600)
def main():
 ap=argparse.ArgumentParser();ap.add_argument('action',choices=['start','status','stop']);ap.add_argument('--stay',action='store_true');ap.add_argument('--python',default=sys.executable);ap.add_argument('--assets',default=str(Path.home()/'.local/share/robot-sim-projects/limo_video_ai_preverify/assets/replay_frames'));ap.add_argument('--recorded',default=str(STATE/'recorded_map_depth.json') if (STATE/'recorded_map_depth.json').exists() else None);ap.add_argument('--mode',choices=['SIMULATION','REPLAY','LIVE'],default='SIMULATION');a=ap.parse_args();registry=STATE/'processes.json'
 if a.action!='start':
  entries=json.loads(registry.read_text()) if registry.exists() else {}
  for name,p in entries.items():
   same=identity(p['pid'])==p['start_ticks'];print(name,'RUNNING' if same else 'STOPPED')
   if a.action=='stop' and same:os.kill(p['pid'],signal.SIGTERM)
  return
 for port in [18310,18311,18313]:
  s=socket.socket();s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);s.bind(('127.0.0.1',port));s.close()
 STATE.mkdir(parents=True,mode=0o700,exist_ok=True);STATE.chmod(0o700)
 for n in ['viewer.token','lab.token','baseline-bridge.token']:
  p=STATE/n
  if not p.exists():p.write_text(secrets.token_urlsafe(32)+'\n');p.chmod(0o600)
 config={'port':18311,'profile':'test','viewer_token_file':str(STATE/'viewer.token'),'bridge_token_file':str(STATE/'baseline-bridge.token'),'mission_url':'http://127.0.0.1:18310','mission_token_file':str(STATE/'viewer.token'),'ai_url':'http://127.0.0.1:18310','ai_token_file':str(STATE/'viewer.token')};private(STATE/'baseline.json',config)
 env=dict(os.environ,PYTHONNOUSERSITE='1',PYTHONPATH=str(FLEET),PYTHONDONTWRITEBYTECODE='1');commands={'optimized':[a.python,'-u','-m','dashboard_lab.server','--state',str(STATE),'--assets',a.assets,'--mode',a.mode,'--baseline-url','http://127.0.0.1:18311','--baseline-token-file',str(STATE/'baseline-bridge.token')], 'baseline':[a.python,'-u','-m','dashboard_lab.baseline_server','--config',str(STATE/'baseline.json')]}
 if a.recorded:commands['optimized']+=['--recorded',a.recorded]
 if (STATE/'inference.json').exists():commands['optimized']+=['--inference',str(STATE/'inference.json')]
 entries={}
 for name,command in commands.items():
  with (STATE/(name+'.log')).open('ab') as log:p=subprocess.Popen(command,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  entries[name]={'pid':p.pid,'start_ticks':identity(p.pid)};private(registry,entries)
 print('Optimized http://127.0.0.1:18310/ | baseline http://127.0.0.1:18311/')
 print('Viewer or local-test login: choose viewer.token / lab.token in private state folder.',flush=True)
 if a.stay:
  try:
   while all(identity(p['pid'])==p['start_ticks'] for p in entries.values()):time.sleep(1)
  except KeyboardInterrupt:pass
  finally:
   for p in entries.values():
    if identity(p['pid'])==p['start_ticks']:
     try:os.kill(p['pid'],signal.SIGTERM)
     except ProcessLookupError:pass

if __name__=='__main__':main()
