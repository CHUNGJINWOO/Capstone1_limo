"""Run the public Arbiter class with a real Gate in an isolated ROS graph.

Only the test package snapshot on PYTHONPATH is used. No driver and no publisher
or subscriber on /cmd_vel is permitted. HTTP and Nav2 evidence are synthetic.
"""
import hashlib
import inspect
import json
import os
import signal
import subprocess
import sys
import time
import unittest
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from limo_robot_agent.manual_web_bench import ManualWebBench
from limo_robot_agent.velocity_arbiter import VelocityArbiter


class FakeHttp:
    def __init__(self):
        self.enabled = False
        self.deadman = True
    def post(self, path, body, timeout):
        assert path == '/agent/manual-bench'
        if not self.enabled:
            return {'sample': None}
        return {'sample': {'linear': .08, 'turn': .18, 'deadman': self.deadman,
                'session_id': 'fixture-session', 'seq': 1, 'ttl_s': .20}}


class Robot(Node):
    def __init__(self):
        super().__init__('fleet_robot_agent')
        self.active = None
        self.gate = None
        self.gate_time = 0.
        self.nav = {'publishers': 1, 'active_count': 0, 'received_status_array': True}
        self.follow_idle = True
        self.mapping_active = False  # The public arbiter does not consult this test field.
        self.create_subscription(String, '/stall_detector/status_json', self.on_gate, 10)
    def on_gate(self, msg):
        self.gate=json.loads(msg.data);self.gate_time=time.monotonic()
    def _age(self, key):
        return time.monotonic()-self.gate_time if key=='gate' and self.gate_time else None
    def _nav2_goals(self): return self.nav.copy()
    def follow_path_idle(self): return self.follow_idle
    def frontier_allowed(self, _): return False


class ArbiterLockedTest(unittest.TestCase):
    def test_nonzero_manual_selection_with_locked_gate(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '215')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        self.assertFalse(os.environ.get('ROS_DISCOVERY_SERVER'))
        # Pin the actual imported modules, not merely a similarly named staging file.
        for cls, expected in ((ManualWebBench, '89f9cfc4a9ad30d809d8df67ef0fe8c4db7cc86b6ded809f8f148545cc802743'),
                              (VelocityArbiter, 'ddcad3a50051e61b32185dee69936692cab9217af1b06d16ed5e91561d377740')):
            with open(inspect.getsourcefile(cls), 'rb') as source:
                self.assertEqual(hashlib.sha256(source.read()).hexdigest(), expected)
        root=os.environ['MANUAL_ARBITER_REPO']
        gate_source=os.path.join(root,'experiments/real_auto_mapping_ready_20261113/src/limo_capstone/limo_capstone/stall_detector_node.py')
        gate_log=os.environ['MANUAL_ARBITER_GATE_LOG']
        with open(gate_log,'w') as log:
            gate=subprocess.Popen([sys.executable,gate_source,'--ros-args','-r','/cmd_vel:=/cmd_vel_locked',
                    '-p','estimator_odom_topic:=/odom_nav2'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                self.exercise(gate)
            finally:
                if gate.poll() is None:
                    os.killpg(gate.pid,signal.SIGINT)
                    try:gate.wait(timeout=7)
                    except subprocess.TimeoutExpired:os.killpg(gate.pid,signal.SIGKILL);gate.wait()

    def exercise(self, gate):
        rclpy.init()
        robot=Robot();smoother=Node('velocity_smoother');controller=Node('controller_server');fixture=Node('arbiter_synthetic_sensors')
        selected=[]
        pubraw=smoother.create_publisher(Twist,'/cmd_vel_raw',10)
        def on_selected(m):
            selected.append((time.monotonic(),m.linear.x,m.angular.z));pubraw.publish(m)
        smoother.create_subscription(Twist,'/fleet/cmd_vel_selected',on_selected,10)
        controller.create_publisher(Twist,'/fleet/cmd_vel_nav',10)
        messages={'/scan':LaserScan(range_min=.1,range_max=8.,ranges=[3.]*20),'/odom':Odometry(),
                  '/odom_nav2':Odometry(),'/odometry/lidar_odom':Odometry(),
                  '/diagnostics/f4_latency':Float32MultiArray(data=[0.,0.,0.,220.]),
                  '/auto_mapping/depth_status':String(data='{"state":"ready","code":"DEPTH_READY"}')}
        pubs={topic:fixture.create_publisher(type(msg),topic,10) for topic,msg in messages.items()}
        http=FakeHttp();agent=SimpleNamespace(connected=True,http=http,robot_id='isolated',boot_id='test')
        bench=ManualWebBench(robot,agent);arbiter=VelocityArbiter(robot,agent,bench)
        nodes=(robot,smoother,controller,fixture)
        next_sensor=0.
        def pump(seconds):
            nonlocal next_sensor
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                self.assertIsNone(gate.poll(),'Gate process exited')
                if time.monotonic()>=next_sensor:
                    for topic,publisher in pubs.items():publisher.publish(messages[topic])
                    next_sensor=time.monotonic()+.025
                for node in nodes:rclpy.spin_once(node,timeout_sec=.002)
        def last():
            self.assertTrue(selected);return selected[-1][1:]
        try:
            pump(3.0)
            self.assertTrue(arbiter.graph_locked())
            self.assertEqual(robot.count_publishers('/cmd_vel'),0)
            self.assertEqual(robot.count_subscribers('/cmd_vel'),0)
            http.enabled=True;pump(.8)
            snap=arbiter.snapshot()
            self.assertEqual(snap['mode'],'MANUAL')
            self.assertAlmostEqual(last()[0],.08,places=5)
            self.assertAlmostEqual(last()[1],.18,places=5)
            self.assertGreater(robot.gate.get('cmd_vx_out',0.),0.)
            self.assertEqual(robot.count_publishers('/cmd_vel_locked'),1)
            self.assertEqual(robot.count_subscribers('/cmd_vel_locked'),0)
            print('PASS public ManualWebBench -> public VelocityArbiter: selected 0.08 m/s, 0.18 rad/s; actual Gate outputs to /cmd_vel_locked; /cmd_vel endpoints 0',flush=True)
            robot.nav['active_count']=1;pump(.35);self.assertEqual(last(),(0.,0.));print('PASS active Nav2 goal -> arbiter zero',flush=True)
            robot.nav['active_count']=0;robot.nav['received_status_array']=False;pump(.35);self.assertEqual(last(),(0.,0.));self.assertGreater(bench.snapshot()['linear'],0.);print('PASS missing Nav2 status -> arbiter zero; OBSERVED GAP: bench alone reports nonzero',flush=True)
            robot.nav['received_status_array']=True;robot.follow_idle=False;pump(.35);self.assertEqual(last(),(0.,0.));self.assertGreater(bench.snapshot()['linear'],0.);print('PASS missing FollowPath terminal evidence -> arbiter zero; OBSERVED GAP: bench alone reports nonzero',flush=True)
            robot.follow_idle=True;http.deadman=False;pump(.35);self.assertEqual(last(),(0.,0.));print('PASS deadman released -> arbiter zero',flush=True)
            http.deadman=True;pump(.35);self.assertGreater(last()[0],0.)
            http.enabled=False;pump(.4);self.assertEqual(last(),(0.,0.));print('PASS expired manual sample -> arbiter zero',flush=True)
            http.enabled=True;pump(.35);self.assertGreater(last()[0],0.)
            print('OBSERVED LIMIT: fake transport can reissue same session after timeout; arbiter alone does not fence resume',flush=True)
            robot.mapping_active=True;pump(.25)
            self.assertGreater(last()[0],0.)
            print('OBSERVED GAP: arbiter alone does not inspect mapping_active; public server admission must fence it',flush=True)
            agent.connected=False;pump(.4);self.assertEqual(last(),(0.,0.));print('PASS Agent disconnect -> arbiter zero',flush=True)
            self.assertEqual(robot.count_publishers('/cmd_vel'),0)
            self.assertEqual(robot.count_subscribers('/cmd_vel'),0)
        finally:
            bench.close()
            for node in reversed(nodes):node.destroy_node()
            rclpy.shutdown()

if __name__=='__main__':unittest.main(verbosity=2)
