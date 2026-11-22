"""Use the Fleet Server, actual Robot Agent and Gate with isolated synthetic sensors."""
import collections,json,os,signal,subprocess,sys,tempfile,threading,time
from pathlib import Path
from types import SimpleNamespace

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray,String
from fleet_server.auth import Auth, init_tokens
from fleet_server.db import DB
from fleet_server.http_api import Servers
from fleet_server.service import Fleet
from limo_robot_agent.core import Http, HttpError

assert os.environ.get('ROS_DOMAIN_ID')=='214' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
here=Path(os.environ['MANUAL_FAULT_RESULTS']).resolve()
scenario=os.environ['MANUAL_FAULT_SCENARIO']
assert scenario in ('server_drop','agent_kill','server_restart','agent_restart')
private=Path(tempfile.mkdtemp(prefix='fleet-manual-bench-'))
fleet=Fleet(DB(str(private/'fleet.db')),SimpleNamespace(offline_after_s=6,command_ttl_s=120,resend_after_s=1))
init_tokens(str(private),str(private),['test-lock'])
server=Servers(fleet,Auth(str(private)),['127.0.0.1'],0)
worker=threading.Thread(target=server.serve_forever,daemon=True)
worker.start()
url='http://127.0.0.1:'+str(server.srvs[0].server_port)
def http(role):
    return Http(url,(private/(role+'.token')).read_text().strip())
operator=http('operator')
rclpy.init()
node=rclpy.create_node('bench_isolated_fixture')
messages={
  '/scan':LaserScan(range_min=.1,range_max=8.,ranges=[3.]*20),
  '/odom':Odometry(),'/odom_nav2':Odometry(),'/odometry/lidar_odom':Odometry(),
  '/diagnostics/f4_latency':Float32MultiArray(data=[0.,0.,0.,220.]),
  '/auto_mapping/depth_status':String(data='{"state":"ready","code":"DEPTH_READY"}'),
}
pub={k:node.create_publisher(type(m),k,10) for k,m in messages.items()}
raw=[]; locked=[]; status=[]
node.create_subscription(Twist,'/cmd_vel_raw',raw.append,10)
node.create_subscription(Twist,'/cmd_vel_locked',locked.append,10)
node.create_subscription(String,'/stall_detector/status_json',lambda m:status.append(json.loads(m.data)),10)
def pump(duration, odom=True, sender=None):
    end=time.monotonic()+duration; next_frame=0.; next_sensor=0.
    while time.monotonic()<end:
        now=time.monotonic()
        if sender and now>=next_frame:
            sender();next_frame=now+.06
        if now>=next_sensor:
            for k,p in pub.items():
                if odom or k!='/odom':p.publish(messages[k])
            next_sensor=now+.025
        rclpy.spin_once(node,timeout_sec=.002)
        time.sleep(.001)
path='/robots/test-lock/manual-bench'
def expect_http(code,f):
    try:f()
    except HttpError as ex:assert ex.status==code,ex
    else:raise AssertionError(f'expected HTTP {code}')
def bench_report():
    with fleet.lock:
        return (fleet.robots.get('test-lock',{}).get('state') or {}).get('manual_web_bench') or {}
def zero(messages):
    return all(not any(getattr(v,k) for v in (m.linear,m.angular) for k in 'xyz') for m in messages)
children=[]
try:
    jobs=[
        ('fixture_gate.log',[sys.executable,'experiments/real_auto_mapping_ready_20261113/src/limo_capstone/limo_capstone/stall_detector_node.py','--ros-args','-r','/cmd_vel:=/cmd_vel_locked','-p','estimator_odom_topic:=/odom_nav2']),
        ('fixture_agent.log',[sys.executable,'-m','limo_robot_agent.agent_node','--ros-args','-p','mode:=real','-p','manual_web_bench:=true','-p','allow_missions:=false','-p','server_url:='+url,'-p','token_file:='+str(private/'agent-test-lock.token'),'-p','robot_id:=test-lock','-p','state_dir:='+str(private/'agent_state'),'-p','period_s:=0.10']),
    ]
    for name,args in jobs:
        with (here/name).open('w') as f:children.append(subprocess.Popen(args,stdout=f,stderr=f))
    pump(3.5)
    assert all(p.poll() is None for p in children)
    assert status[-1]['state']=='NORMAL'
    lease=operator.post('/robots/test-lock/lock',{'action':'acquire','ttl_s':15})
    epoch=lease['epoch']
    expect_http(403,lambda: http('viewer').post(path,{'action':'start','epoch':epoch}))
    expect_http(409,lambda: http('admin').post(path,{'action':'start','epoch':epoch}))
    session=operator.post(path,{'action':'start','epoch':epoch})['session_id']
    expect_http(409,lambda: operator.post(path,{'action':'start','epoch':epoch}))
    seq=0
    def frame(v,w,pressed=True):
        def send():
            global seq
            seq+=1
            operator.post(path,{'action':'sample','epoch':epoch,'session_id':session,
                                'seq':seq,'linear':v,'turn':w,'deadman':pressed})
        return send
    pump(.65,sender=frame(1,.5))
    report=bench_report()
    print('forward/turn',report,'raw',len(raw),'locked',len(locked),flush=True)
    assert report.get('requested_linear')==.10 and report.get('requested_turn')==.15
    assert report.get('gate_zero_published') and len(raw)>0 and zero(raw+locked)
    pump(.4,sender=frame(-1,-1))
    report=bench_report()
    print('reverse clamp / left turn',report,flush=True)
    assert report.get('requested_linear')==0 and report.get('requested_turn')==-.30
    pump(.4,sender=frame(1,1,False))
    assert bench_report().get('status')=='deadman_released'
    pump(.4)
    print('browser silence',bench_report(),flush=True)
    assert bench_report().get('status')=='input_expired'
    expect_http(409,frame(1,0))
    session=operator.post(path,{'action':'start','epoch':epoch})['session_id']
    pump(.65,odom=False,sender=frame(1,0))
    print('wheel stale',status[-1]['state'],bench_report(),flush=True)
    assert status[-1]['wheel_odom_stale'] and bench_report().get('status')=='gate_or_odom_not_ready'
    pump(.5,sender=frame(0,0,False))
    conflict=node.create_publisher(Twist,'/cmd_vel_raw',10)
    pump(.5,sender=frame(1,0))
    print('competing publisher',bench_report(),flush=True)
    assert bench_report().get('status')=='velocity_source_conflict'
    node.destroy_publisher(conflict)
    pump(.4,sender=frame(1,0))
    operator.post(path,{'action':'stop','epoch':epoch,'session_id':session})
    pump(.35)
    assert bench_report().get('status')=='input_expired'
    assert zero(raw+locked) and node.count_publishers('/cmd_vel')==0
    print('PASS all existing Gate input/output samples zero and no motor publisher',len(raw),len(locked),flush=True)
    session=operator.post(path,{'action':'start','epoch':epoch})['session_id']
    pump(.45,sender=frame(1,.5))
    assert bench_report().get('status')=='active'
    if scenario in ('agent_kill','agent_restart'):
        children[1].kill();children[1].wait(timeout=3)
        pump(.8)
        assert children[1].returncode is not None
        assert zero(raw+locked) and node.count_publishers('/cmd_vel')==0
        print('PASS Agent SIGKILL: locked output stayed zero, motor publisher count 0',flush=True)
        if scenario=='agent_restart':
            with (here/'fixture_agent_restart.log').open('w') as f:
                children.append(subprocess.Popen(jobs[1][1],stdout=f,stderr=f))
            pump(3.5)
            assert children[-1].poll() is None
            expect_http(409,frame(1,0))
            assert zero(raw+locked) and node.count_publishers('/cmd_vel')==0
            print('PASS Agent restarted: old session refused; locked output stayed zero',flush=True)
    else:
        server.stop_evt.set();worker.join(timeout=4);assert not worker.is_alive()
        pump(.8)
        assert children[1].poll() is None
        assert zero(raw+locked) and node.count_publishers('/cmd_vel')==0
        print('PASS Fleet HTTP stop: locked output stayed zero, motor publisher count 0',flush=True)
        restarted=Fleet(DB(str(private/'fleet.db')),SimpleNamespace(offline_after_s=6,command_ttl_s=120,resend_after_s=1))
        assert restarted.manual_benches=={}
        print('PASS new Fleet instance restores no prior manual session',flush=True)
        if scenario=='server_restart':
            server=Servers(restarted,Auth(str(private)),['127.0.0.1'],int(url.rsplit(':',1)[1]))
            worker=threading.Thread(target=server.serve_forever,daemon=True);worker.start()
            pump(1.0)
            expect_http(409,frame(1,0))
            assert zero(raw+locked) and node.count_publishers('/cmd_vel')==0
            print('PASS Fleet HTTP restarted: old session refused; locked output stayed zero',flush=True)
finally:
    for child in children:
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=4)
        except subprocess.TimeoutExpired:child.kill();child.wait()
    server.stop_evt.set();worker.join(timeout=4)
    node.destroy_node();rclpy.shutdown()
