import importlib.util,math,os,signal,subprocess,sys,tempfile,time
from pathlib import Path
import rclpy,yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy,QoSProfile
from PIL import Image
# Run only in an isolated ROS domain with nav2_map_server installed.
rclpy.init();node=rclpy.create_node('map_test_fixture')
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub=node.create_publisher(OccupancyGrid,'/map',q)
m=OccupancyGrid();m.header.frame_id='map';m.header.stamp.sec=123;m.header.stamp.nanosec=456
m.info.width=3;m.info.height=2;m.info.resolution=.05
m.info.origin.position.x=-1.23456789;m.info.origin.position.y=2.34567891
m.info.origin.orientation.z=math.sin(.23/2);m.info.origin.orientation.w=math.cos(.23/2)
m.data=[-1,0,100,100,-1,0];pub.publish(m)
p=Path(tempfile.mkdtemp(prefix='map-third-regression-'))/'map'
code="""import importlib.util,sys
p=importlib.util.spec_from_file_location('manager',sys.argv[1]);m=importlib.util.module_from_spec(p);p.loader.exec_module(m)
print(m.save_frozen_grid(sys.argv[2]))
"""
script=Path(__file__).resolve().parents[1] / 'scripts' / 'auto_mapping_experiment_manager.py'
proc=subprocess.Popen([sys.executable,'-c',code,str(script),str(p)],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
end=time.monotonic()+15
while proc.poll() is None and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
out,err=proc.communicate(timeout=2);print(out,err)
assert proc.returncode==0
y=yaml.safe_load(p.with_suffix('.yaml').read_text());print(y)
assert y['origin'][0]==m.info.origin.position.x and y['origin'][1]==m.info.origin.position.y
assert math.isclose(y['origin'][2],.23,abs_tol=1e-12) and math.isclose(y['resolution'],m.info.resolution,abs_tol=1e-8)
assert y['free_thresh']==.196
assert list(Image.open(p.with_suffix('.pgm')).getdata())==[0,205,254,205,254,0]
print('same frozen grid, raster classification and full precision metadata PASS')
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav_msgs.srv import GetMap
server=subprocess.Popen(['ros2','run','nav2_map_server','map_server','--ros-args','-p','yaml_filename:='+str(p.with_suffix('.yaml'))],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
try:
    client=node.create_client(ChangeState,'/map_server/change_state')
    assert client.wait_for_service(timeout_sec=10)
    for transition in (Transition.TRANSITION_CONFIGURE,Transition.TRANSITION_ACTIVATE):
        req=ChangeState.Request();req.transition.id=transition
        future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        assert future.result() and future.result().success
    client2=node.create_client(GetMap,'/map_server/map');assert client2.wait_for_service(timeout_sec=10)
    future=client2.call_async(GetMap.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=10)
    loaded=future.result().map
    assert list(loaded.data)==list(m.data)
    assert loaded.info.width==m.info.width and loaded.info.height==m.info.height
    assert abs(loaded.info.resolution-m.info.resolution)<1e-8
    assert abs(loaded.info.origin.position.x-m.info.origin.position.x)<1e-10
    assert abs(loaded.info.origin.position.y-m.info.origin.position.y)<1e-10
    assert abs(loaded.info.origin.orientation.z-m.info.origin.orientation.z)<1e-10
    print('Nav2 map_server reload: all 6 cells, dimensions, resolution and origin PASS')
finally:
    server.terminate();server.wait(timeout=5)
node.destroy_node();rclpy.shutdown()
