"""Run isolated real Fleet Server + Robot Agent + rosbag + Chromium smoke."""
import argparse,json,os,signal,subprocess,sys,tempfile,threading,time,urllib.request,urllib.error
from pathlib import Path
from types import SimpleNamespace
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'fleet_control/server'))
from fleet_server.auth import Auth,init_tokens
from fleet_server.db import DB
from fleet_server.service import Fleet
from fleet_server.http_api import Servers
from fleet_server.map_registry import MapRegistry

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);a=ap.parse_args();d=Path(a.output);d.mkdir(mode=0o700)
 if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not os.environ.get('ROS_DOMAIN_ID'):raise SystemExit('isolated ROS domain required')
 init_tokens(str(d),str(d/'tokens'),['TEST-REC'])
 fleet=Fleet(DB(str(d/'fleet.db')),SimpleNamespace(offline_after_s=6,command_ttl_s=120,resend_after_s=1))
 registry=MapRegistry(str(d/'registry.db'),str(d/'maps'))
 registry.create_facility('test-facility','Test Facility');registry.create_building('test-building','test-facility','Test Building');registry.create_floor('test-floor','test-building','L1')
 srv=Servers(fleet,Auth(str(d)),['127.0.0.1'],0,map_registry=registry);server_thread=threading.Thread(target=srv.serve_forever,daemon=True);server_thread.start()
 port=srv.srvs[0].server_address[1]
 info={'url':f'http://127.0.0.1:{port}','token_dir':str(d/'tokens'),'output':str(d)};(d/'info.json').write_text(json.dumps(info))
 env=dict(os.environ);env['PYTHONPATH']=str(root/'fleet_control/agent/limo_robot_agent')+os.pathsep+str(root/'fleet_control/server')+os.pathsep+env.get('PYTHONPATH','')
 log=(d/'agent.log').open('wb')
 args=[sys.executable,'-m','limo_robot_agent.agent_node','--ros-args','-p',f'server_url:={info["url"]}','-p',f'token_file:={d}/tokens/agent-TEST-REC.token','-p','robot_id:=TEST-REC','-p','mode:=sim','-p','simulated:=true','-p','allow_missions:=false','-p','slam_sim_enabled:=true','-p',f'slam_root:={d}/slam','-p',f'state_dir:={d}/agent-state','-p','period_s:=0.3','-p','recording_enabled:=true','-p',f'recording_root:={d}/recordings','-p',f'recorder_script:={root}/capstone_ws/src/limo_capstone/scripts/run_recorder.py','-p',f'verifier_script:={root}/fleet_control/scripts/verify_bag_copy.py']
 publog=(d/'scan.log').open('wb')
 publisher=subprocess.Popen([sys.executable,str(root/'fleet_control/tests/slam_synthetic_publisher.py')],env=env,stdout=publog,stderr=subprocess.STDOUT)
 agent=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT)
 try:
  out=subprocess.run(['node',str(root/'fleet_control/tests/slam_web_ros_e2e.cjs'),str(d/'info.json')],env=env,text=True,capture_output=True,timeout=150)
  (d/'browser.log').write_text(out.stdout+out.stderr);print(out.stdout+out.stderr,flush=True)
  if out.returncode:
   print('evidence:',d,'browser_exit:',out.returncode,flush=True);return out.returncode
  token=(d/'tokens/operator.token').read_text().strip()
  def request(path,body=None):
   req=urllib.request.Request(info['url']+'/api/v1'+path,data=None if body is None else json.dumps(body).encode(),headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
   with urllib.request.urlopen(req,timeout=4) as response:return json.load(response)
  def wait_for(fn,timeout=45):
   deadline=time.monotonic()+timeout
   while time.monotonic()<deadline:
    try:
     if fn():return
    except Exception:pass
    time.sleep(.3)
   raise RuntimeError('state did not converge')
  seq=fleet.robots['TEST-REC'].get('seq',0)
  srv.stop_evt.set();server_thread.join(timeout=12)
  if server_thread.is_alive():raise RuntimeError('server did not stop')
  srv=Servers(fleet,Auth(str(d)),['127.0.0.1'],port,map_registry=registry)
  server_thread=threading.Thread(target=srv.serve_forever,daemon=True);server_thread.start()
  wait_for(lambda:fleet.robots['TEST-REC'].get('seq',0)>seq)
  print('PASS Agent reconnected after loopback server restart',flush=True)
  # DDS graph discovery may retain the stopped map_server for its lease duration.
  time.sleep(25)
  request('/robots/TEST-REC/slam/start',{'session_id':'restart_slam'})
  wait_for(lambda:(fleet.robots['TEST-REC'].get('state') or {}).get('slam',{}).get('status')=='RUNNING',timeout=60)
  childfile=Path(f'/proc/{agent.pid}/task/{agent.pid}/children')
  matches=[]
  for pid in childfile.read_text().split():
   try:
    cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
    if b'ros2' in cmd and b'slam_toolbox' in cmd and os.getpgid(int(pid))==int(pid):matches.append(int(pid))
   except (FileNotFoundError,ProcessLookupError):pass
  if len(matches)!=1:raise RuntimeError('test SLAM child identity ambiguous')
  child=matches[0];started=Path(f'/proc/{child}/stat').read_text().split()[21]
  agent.send_signal(signal.SIGINT);agent.wait(timeout=15)
  agent=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT)
  try:
   wait_for(lambda:(fleet.robots['TEST-REC'].get('state') or {}).get('slam',{}).get('status')=='UNKNOWN')
   unknown=(fleet.robots['TEST-REC']['state']['slam'])
   if unknown.get('map_version_id') is not None or unknown.get('floor_id') is not None:raise RuntimeError('new session inherited old map metadata')
   try:request('/robots/TEST-REC/slam/start',{'session_id':'after_restart'})
   except urllib.error.HTTPError as e:
    if e.code!=409:raise
   else:raise RuntimeError('UNKNOWN SLAM allowed a new start')
   print('PASS Agent restart -> SLAM UNKNOWN and duplicate blocked',flush=True)
  finally:
   p=Path(f'/proc/{child}')
   if p.exists() and p.joinpath('stat').read_text().split()[21]==started and os.getpgid(child)==child:os.killpg(child,signal.SIGINT)
  print('evidence:',d,'browser_exit:',out.returncode,flush=True)
  return 0
 finally:
  if agent.poll() is None:
   childfile=Path(f'/proc/{agent.pid}/task/{agent.pid}/children')
   if childfile.exists():
    for pid in childfile.read_text().split():
     try:
      cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
      if b'run_recorder.py' in cmd and str(d/'recordings').encode() in cmd:os.kill(int(pid),signal.SIGINT)
      elif b'ros2' in cmd and b'slam_toolbox' in cmd and os.getpgid(int(pid))==int(pid):os.killpg(int(pid),signal.SIGINT)
     except (FileNotFoundError,ProcessLookupError):pass
   agent.send_signal(signal.SIGINT)
  try:
   agent.wait(timeout=15);print('agent final exit code:',agent.returncode,flush=True)
  except subprocess.TimeoutExpired:print('agent shutdown timeout; not force killed')
  if publisher.poll() is None:publisher.send_signal(signal.SIGINT)
  try:publisher.wait(timeout=10)
  except subprocess.TimeoutExpired:print('publisher shutdown timeout')
  publog.close();log.close();srv.stop_evt.set();registry.close()
if __name__=='__main__':sys.exit(main())
