"""Actual rclpy transport, simulated SLAM state; no Gate or robot driver."""
import time
import unittest
from types import SimpleNamespace
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from limo_robot_agent.manual_mapping_input import ManualMappingInput
from limo_robot_agent.joystick_filter import JoystickConfig

class ManualROS(unittest.TestCase):
 def test_joy_transport_watchdog_and_stop(self):
  rclpy.init(); node=Node('manual_mapping_transport_test'); state={'status':'IDLE'}
  node.execution_allowed=lambda:True
  adapter=ManualMappingInput(node,SimpleNamespace(snapshot=lambda:dict(state)),JoystickConfig(1,0,0,.15,.4,.2))
  adapter._motion_permitted=lambda:node.execution_allowed() is True
  pub=node.create_publisher(Joy,'/joy',10);received=[]
  sub=node.create_subscription(Twist,'/cmd_vel_pre_cm',lambda m:received.append((m.linear.x,m.angular.z)),10)
  def pump(duration, msg=None):
   end=time.monotonic()+duration
   while time.monotonic()<end:
    if msg:pub.publish(msg)
    rclpy.spin_once(node,timeout_sec=.01)
  try:
   msg=Joy();msg.axes=[.5,1.];msg.buttons=[1]
   pump(.4,msg);self.assertTrue(received);self.assertTrue(all(v==(0.,0.) for v in received))
   state['status']='RUNNING';received.clear();pump(.3,msg)
   self.assertTrue(any(abs(v-.15)<.001 and abs(w-.2)<.001 for v,w in received))
   pump(.35);self.assertEqual(received[-1],(0.,0.))
   msg.buttons=[0];pump(.1,msg);self.assertEqual(received[-1],(0.,0.))
   node.execution_allowed=lambda:False;msg.buttons=[1];pump(.1,msg);self.assertEqual(received[-1],(0.,0.))
   state['status']='STOP_REQUESTED';pump(.1,msg);self.assertEqual(received[-1],(0.,0.))
  finally:adapter.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':unittest.main(verbosity=2)
