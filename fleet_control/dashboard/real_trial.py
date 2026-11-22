"""Bounded approved trial: read-only ROS stdout over SSH to the already-running local gateway."""
import argparse,datetime,json,os,shlex,subprocess,time,urllib.request
from pathlib import Path
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--connection-config',required=True,help='0600 JSON containing ssh_target and optional setup_files')
parser.add_argument('--seconds',type=int,default=15,choices=range(1,16))
a=parser.parse_args();cp=Path(a.connection_config).expanduser()
if cp.stat().st_mode & 0o077:raise SystemExit('connection config must be 0600')
c=json.loads(cp.read_text());state=Path(os.environ.get('DASHBOARD_STATE',str(Path.home()/'.local/state/capstone-dashboard-visualization')));token=(state/'bridge.token').read_text().strip()
output=state/'real_trials'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S');output.mkdir(parents=True,mode=0o700,exist_ok=False)
def setup_path(p):return '"$HOME"/'+shlex.quote(p[2:]) if p.startswith('~/') else shlex.quote(p)
setups='\n'.join('source '+setup_path(p) for p in c.get('setup_files',['/opt/ros/humble/setup.bash']))

command='set -e\n'+setups+"\nexport ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1\nexport PYTHONPATH=\"$HOME/limo_dashboard_readonly_20261113${PYTHONPATH:+:$PYTHONPATH}\"\nexec timeout --signal=INT --kill-after=3 "+str(a.seconds)+"s python3 -u -m dashboard.bridge_node --stdout --topics \"$HOME/limo_dashboard_readonly_20261113/topics.json\" --kind ROS_LIVE --robot-id pcu --fps 1 --lock-check required\n"
start=time.monotonic();packets=[];last=None;failures=[]
with (output/'bridge-stderr.log').open('w') as err:
 p=subprocess.Popen(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=8',c['ssh_target'],'bash -s'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err,text=True,start_new_session=True)
 p.stdin.write(command);p.stdin.close()
 try:
  for line in p.stdout:
   try:payload=json.loads(line)
   except ValueError:continue
   if not isinstance(payload,dict) or payload.get('kind')!='ROS_LIVE':continue
   body=json.dumps(payload,allow_nan=False).encode();t=time.monotonic()
   request=urllib.request.Request('http://127.0.0.1:18210/api/bridge',data=body,headers={'Content-Type':'application/json','Authorization':'Bearer '+token})
   try:
    with urllib.request.urlopen(request,timeout=2) as r:reply=json.load(r)
    packets.append({'elapsed_s':round(time.monotonic()-start,3),'bytes':len(body),'local_sink_ack_ms':round((time.monotonic()-t)*1000,3),'sequence':reply['sequence']});last=payload
    if len(packets)==1:print('First actual ROS_LIVE packet accepted by existing 18210',flush=True)
   except Exception as e:failures.append(type(e).__name__)
 finally:
  try:rc=p.wait(timeout=8)
  except subprocess.TimeoutExpired:p.terminate();rc=p.wait(timeout=5)
elapsed=time.monotonic()-start
report={'duration_s':round(elapsed,2),'remote_exit':rc,'packets':len(packets),'payload_kB_s':round(sum(p['bytes'] for p in packets)/elapsed/1000,2),'failures':failures,'observations':packets}
if last:
 (output/'last_packet.json').write_text(json.dumps(last));(output/'last_packet.json').chmod(0o600)
 report.update(last_source=last['kind'],last_metrics=last['metrics'],unavailable=last['unavailable'],grid_shapes={k:{x:v[x] for x in ['width','height','resolution','origin','frame_id','map_from_frame','stamp_ns']} for k,v in last['grids'].items()},depth={k:v for k,v in last.get('depth',{}).items() if k!='values_f32le_zlib'})
(output/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ['last_metrics','grid_shapes','depth','observations']},indent=2),flush=True)

print('Private trial evidence:',output)
