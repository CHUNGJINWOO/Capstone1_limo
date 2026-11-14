"""Gazebo variation of the existing RMF E2E fixture; only owns its subprocess groups."""
import os,json,shlex,subprocess,time,math,re,signal,socket
from pathlib import Path
import psutil
import rmf_env as E
REFERENCE = os.environ.get('CAPSTONE_REFERENCE_WS')
if not REFERENCE or not Path(REFERENCE,'install/setup.bash').is_file():
    raise SystemExit('CAPSTONE_REFERENCE_WS must name the existing read-only built workspace')
E.PORT=int(os.environ.get('GAZEBO_TEST_PORT','18120'));E.BASE=f'http://127.0.0.1:{E.PORT}'
E.WS_PORT=int(os.environ.get('GAZEBO_TEST_WS_PORT','18121'))
E.ENV_RMF.update(ROS_DOMAIN_ID=os.environ.get('GAZEBO_RMF_DOMAIN','162'),PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1')
E.ENV_ROBOT.update(ROS_DOMAIN_ID=os.environ.get('GAZEBO_ROBOT_DOMAIN','163'),PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',IGN_PARTITION=f'capstone_rmf_{os.getpid()}')
OWNED={}
_original_kill=E.kill

def spawn(name,cmd,env,ros_rmf=False):
    prefix='source /opt/ros/humble/setup.bash; source '+shlex.quote(E.RMF_PREFIX if ros_rmf else str(Path(REFERENCE,'install/setup.bash')))+'; '
    p=subprocess.Popen(['bash','-c',prefix+cmd],env=env,stdout=open(os.path.join(E.T,name+'.log'),'a'),stderr=subprocess.STDOUT,start_new_session=True,cwd=E.T)
    E.PROCS[name]=p;OWNED[p.pid]=psutil.Process(p.pid).create_time();return p

def kill(name,sig=signal.SIGINT):
    p=E.PROCS.get(name)
    if p:
        try:
            for child in psutil.Process(p.pid).children(recursive=True):OWNED[child.pid]=child.create_time()
        except psutil.NoSuchProcess:pass
    _original_kill(name,sig)
E.spawn=spawn;E.kill=kill

def start_agent():
    spawn('agent',f'exec python3 -m limo_robot_agent.agent_node --ros-args -p server_url:={E.BASE} -p token_file:={E.SEC}/agent-pcu.token -p robot_id:=pcu -p mode:=sim -p simulated:=true -p allow_missions:=true -p allow_sim_safety_gate:=true -p use_sim_time:=true -p state_dir:={E.T}/agent_state',E.ENV_ROBOT)
E.start_agent=start_agent

def latest_evidence():
    try:
        with open(os.path.join(E.T,'gazebo_evidence.jsonl'),'rb') as f:
            f.seek(0,2);f.seek(max(0,f.tell()-65536));lines=f.read().splitlines()
        for line in reversed(lines):
            try:return json.loads(line)
            except ValueError:continue
    except OSError:pass
    return {}

def start_robot(speed=None):
    launch=os.path.join(E.FLEET,'gazebo','localized_sim.launch.py')
    spawn('gazebo','exec ros2 launch '+shlex.quote(launch),E.ENV_ROBOT)
    obs=os.path.join(E.FLEET,'gazebo','observe.py')
    spawn('observer',f'exec python3 {shlex.quote(obs)} {E.T}/gazebo_evidence.jsonl',E.ENV_ROBOT)
    def plant_ready():
        if E.PROCS['gazebo'].poll() is not None:raise RuntimeError('Gazebo launch exited; see private runtime log')
        return latest_evidence().get('gt') and latest_evidence().get('pose')
    assert E.wait(plant_ready,90),'Gazebo/AMCL not ready'
    assert E.wait(lambda:any('lifecycle_manager_navigation' in line and 'Managed nodes are active' in line for line in Path(E.T,'gazebo.log').read_text().splitlines()),90),'Nav2 lifecycle not active'
    assert E.wait(lambda:latest_evidence().get('gate',{}).get('state')=='NORMAL',45),'Safety Gate not NORMAL; no reset/goal allowed'
    start_agent()
E.start_robot=start_robot

def start_adapter():
    spawn('adapter',f'exec python3 -u -m limo_rmf_adapter.adapter_node --config {E.RMF}/config/fleet_gazebo.json --server-url {E.BASE} --token-file {E.SEC}/rmf-adapter.token --state-dir {E.T}/adapter_state --rmf-api-uri ws://127.0.0.1:{E.WS_PORT}',E.ENV_RMF,True)
E.start_adapter=start_adapter

def leftovers():
    result=[]
    for pid,created in OWNED.items():
        try:
            p=psutil.Process(pid)
            if p.create_time()==created and p.status()!=psutil.STATUS_ZOMBIE:result.append(pid)
        except psutil.NoSuchProcess:pass
    for port in [E.PORT,E.WS_PORT]:
        s=socket.socket();s.settimeout(.2)
        if s.connect_ex(('127.0.0.1',port))==0:result.append('port:'+str(port))
        s.close()
    return result

def goal_log():
    # Independent Nav2 status UUIDs must agree with the actual accepted-goal response audit.
    seen=latest_evidence().get('goals',{})
    try:audit=re.findall(r'NAV2_ACCEPTED command=(c_[a-f0-9]+) goal=([a-f0-9]+)',Path(E.T,'agent.log').read_text())
    except OSError:audit=[]
    matches={gid:cid for cid,gid in audit}
    ms=E.missions();bycid={c['command_id']:m for m in ms for c in m['commands']}
    out=[]
    for gid,info in seen.items():
        if gid not in matches:continue
        m=bycid.get(matches[gid])
        if m:out.append({'ev':'goal_received','goal_id':gid,'command_id':matches[gid],'t':info['first_seen'],'x':m['goal']['x'],'y':m['goal']['y']})
    return out

from rmf_env import *
