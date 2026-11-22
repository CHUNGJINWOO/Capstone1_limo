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
 ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--restart-signal',choices=('SIGINT','SIGKILL'),default='SIGINT');a=ap.parse_args();d=Path(a.output);d.mkdir(mode=0o700)
 if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or not os.environ.get('ROS_DOMAIN_ID'):raise SystemExit('isolated ROS domain required')
 init_tokens(str(d),str(d/'tokens'),['TEST-REC'])
 fleet=Fleet(DB(str(d/'fleet.db')),SimpleNamespace(offline_after_s=6,command_ttl_s=120,resend_after_s=1))
 registry=MapRegistry(str(d/'registry.db'),str(d/'maps'))
 srv=Servers(fleet,Auth(str(d)),['127.0.0.1'],0,map_registry=registry);threading.Thread(target=srv.serve_forever,daemon=True).start()
 port=srv.srvs[0].server_address[1]
 info={'url':f'http://127.0.0.1:{port}','token_dir':str(d/'tokens'),'output':str(d)};(d/'info.json').write_text(json.dumps(info))
 env=dict(os.environ);env['PYTHONPATH']=str(root/'fleet_control/agent/limo_robot_agent')+os.pathsep+str(root/'fleet_control/server')+os.pathsep+env.get('PYTHONPATH','')
 log=(d/'agent.log').open('wb')
 args=[sys.executable,'-m','limo_robot_agent.agent_node','--ros-args','-p',f'server_url:={info["url"]}','-p',f'token_file:={d}/tokens/agent-TEST-REC.token','-p','robot_id:=TEST-REC','-p','mode:=sim','-p','simulated:=true','-p','allow_missions:=false','-p',f'state_dir:={d}/agent-state','-p','period_s:=0.3','-p','recording_enabled:=true','-p',f'recording_root:={d}/recordings','-p',f'recorder_script:={root}/capstone_ws/src/limo_capstone/scripts/run_recorder.py','-p',f'verifier_script:={root}/fleet_control/scripts/verify_bag_copy.py']
 agent=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT)
 try:
  out=subprocess.run(['node',str(root/'fleet_control/tests/recording_web_ros_e2e.cjs'),str(d/'info.json')],env=env,text=True,capture_output=True,timeout=150)
  (d/'browser.log').write_text(out.stdout+out.stderr);print(out.stdout+out.stderr,flush=True)
  if out.returncode:
   print('evidence:',d,'browser_exit:',out.returncode,flush=True);return out.returncode
  token=(d/'tokens/operator.token').read_text().strip()
  def request(path, body=None):
   req=urllib.request.Request(info['url']+'/api/v1'+path, data=None if body is None else json.dumps(body).encode(), headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
   with urllib.request.urlopen(req,timeout=4) as response:return json.load(response)
  def wait_for(fn,timeout=40):
   until=time.monotonic()+timeout
   while time.monotonic()<until:
    try:
     value=fn()
     if value:return value
    except Exception:pass
    time.sleep(.2)
   raise RuntimeError('timed out waiting for recording state')
  request('/robots/TEST-REC/recording/start',{'session_id':'restart_case'})
  wait_for(lambda: request('/robots/TEST-REC/recording')['recording'].get('status')=='RECORDING')
  wait_for(lambda: any(c['type']=='record_start' and c['session_id']=='restart_case' and c['status']=='SUCCEEDED' for c in request('/robots/TEST-REC/recording')['commands']))
  children=Path(f'/proc/{agent.pid}/task/{agent.pid}/children').read_text().split()
  matches=[int(pid) for pid in children if b'run_recorder.py' in Path(f'/proc/{pid}/cmdline').read_bytes()]
  if len(matches)!=1:raise RuntimeError('test recorder child identity ambiguous')
  child=matches[0];child_start=Path(f'/proc/{child}/stat').read_text().split()[21]
  agent.send_signal(getattr(signal,a.restart_signal));agent.wait(timeout=15)
  print('agent restart signal:',a.restart_signal,flush=True)
  print('agent first exit code:', agent.returncode, flush=True)
  proc=Path(f'/proc/{child}')
  if not proc.exists() or proc.joinpath('stat').read_text().split()[21]!=child_start:raise RuntimeError('owned recorder did not survive agent exit')
  print('PASS recorder survives agent shutdown',flush=True)
  agent=subprocess.Popen(args,env=env,stdout=log,stderr=subprocess.STDOUT)
  try:
   wait_for(lambda: request('/robots/TEST-REC/recording')['recording'].get('status')=='UNKNOWN')
   print('PASS real agent restart -> recording UNKNOWN',flush=True)
   try: request('/robots/TEST-REC/recording/start', {'session_id':'restart_duplicate'})
   except urllib.error.HTTPError as e:
    if e.code != 409:raise
    print('PASS UNKNOWN blocks new recording (409)',flush=True)
   else: raise RuntimeError('UNKNOWN allowed a new recording')
  finally:
   proc=Path(f'/proc/{child}')
   if proc.exists() and proc.joinpath('stat').read_text().split()[21]==child_start and b'run_recorder.py' in proc.joinpath('cmdline').read_bytes():os.kill(child,signal.SIGINT)
  print('evidence:',d,'browser_exit:',out.returncode,flush=True)
  return 0
 finally:
  if agent.poll() is None:agent.send_signal(signal.SIGINT)
  try:
   agent.wait(timeout=15);print('agent final exit code:',agent.returncode,flush=True)
  except subprocess.TimeoutExpired:print('agent shutdown timeout; not force killed')
  log.close();srv.stop_evt.set();registry.close()
  if agent.poll() is None or agent.returncode != 0:
   raise SystemExit('FAIL: final Agent shutdown was not clean')
if __name__=='__main__':sys.exit(main())
