"""Isolated ROS bench with a synthetic locked Gate, no physical driver or /cmd_vel publisher."""
import threading
import time
import unittest
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from limo_robot_agent.manual_web_bench import ManualWebBench
from limo_robot_agent.joystick_filter import JoystickConfig


class Robot(Node):
    def __init__(self):
        super().__init__('fleet_robot_agent')
        self.lock = threading.Lock()
        self.gate = {'state':'NORMAL', 'wheel_odom_stale':False, 'cmd_blocked':False}
        self.active = None
    def _age(self, _): return 0.01


class LockedROS(unittest.TestCase):
    def test_bench_only_sends_zero_and_timeout_zero(self):
        rclpy.init()
        robot = Robot(); gate = Node('stall_detector_node'); watch = Node('bench_watch')
        raw=[]; locked=[]
        lock_pub=gate.create_publisher(Twist,'/cmd_vel_locked',10)
        status_pub=gate.create_publisher(String,'/stall_detector/status_json',10)
        def relay(m):
            raw.append((m.linear.x,m.angular.z));lock_pub.publish(Twist())
        gate.create_subscription(Twist,'/cmd_vel_raw',relay,10)
        watch.create_subscription(Twist,'/cmd_vel_locked',lambda m:locked.append((m.linear.x,m.angular.z)),10)
        bench=ManualWebBench.__new__(ManualWebBench)
        bench.robot=robot
        session = dict(connected=True, reconciliation_required=False, server_boot_id='server',
                       protocol_version=2, safety_contract_version=1, contract_generation=0,
                       lease=dict(owner='operator', epoch=1, expires_at=time.time()+30))
        bench.agent=SimpleNamespace(is_execution_allowed=lambda:True, boot_id='agent',
                                    session_snapshot=lambda:session)
        bench.config=JoystickConfig(1,0,4,.10,.30,.25);bench.lock=threading.Lock()
        bench.sample=None;bench.publisher=None;bench.state={}
        def spin(seconds, use_bench=False):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                if use_bench: bench.tick()
                for node in (robot,gate,watch):rclpy.spin_once(node,timeout_sec=.005)
        try:
            spin(.8)
            self.assertEqual(robot.count_publishers('/cmd_vel'),0)
            self.assertIsNone(bench._graph_blocker())
            bench.sample={'linear':1.,'turn':.5,'deadman':True,'expires':time.monotonic()+2,'session_id':'s','authority':bench.authority(),'seq':1}
            spin(.35,True)
            self.assertEqual(bench.state['requested_linear'],.1)
            self.assertEqual(bench.state['requested_turn'],.15)
            self.assertTrue(raw and locked)
            self.assertTrue(all(v==(0.,0.) for v in raw+locked))
            bench.sample={'linear':1.,'turn':1.,'deadman':False,'expires':time.monotonic()+1,'session_id':'s','authority':bench.authority(),'seq':2}
            spin(.15,True)
            self.assertEqual((bench.state['requested_linear'],bench.state['requested_turn']),(0.,0.))
            bench.sample={'linear':1.,'turn':1.,'deadman':True,'expires':time.monotonic()-.01,'session_id':'s','authority':bench.authority(),'seq':3}
            spin(.15,True)
            self.assertEqual(bench.state['status'],'input_expired')
            self.assertEqual((bench.state['requested_linear'],bench.state['requested_turn']),(0.,0.))
            robot.mode = 'real'
            bench.sample = dict(bench.sample, expires=time.monotonic()+1, deadman=True)
            spin(.15,True)
            self.assertIsNone(bench.publisher)
            self.assertEqual(robot.count_publishers('/cmd_vel_raw'), 0)
            self.assertFalse(bench.state['motor_output'])
            robot.gate['state'] = 'FAILSAFE_SENSOR_TIMEOUT'
            bench.sample={'linear':1.,'turn':1.,'deadman':True,'expires':time.monotonic()+1,'session_id':'s','authority':bench.authority(),'seq':4}
            spin(.15,True)
            self.assertEqual(bench.state['status'],'gate_or_odom_not_ready')
            self.assertTrue(all(v==(0.,0.) for v in raw+locked))
            self.assertEqual(robot.count_publishers('/cmd_vel'),0)
        finally:
            if bench.publisher is not None:robot.destroy_publisher(bench.publisher)
            watch.destroy_node();gate.destroy_node();robot.destroy_node();rclpy.shutdown()

if __name__=='__main__':unittest.main(verbosity=2)
