"""Isolated ROS probe: GID changes are not Gate identity proof."""
import json,os,socket,sys,time,unittest
from pathlib import Path
if os.environ.get('CAPSTONE_PRIVATE_ROS_TEST')!='1' or {n for _,n in socket.if_nameindex()}!={'lo'}:
 raise SystemExit('refusing: private network namespace required')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent'))
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from limo_robot_agent.safety_protocol import GateObservationTracker

class Tests(unittest.TestCase):
 def test_publisher_replacement_gid_not_bound_to_boot(self):
  rclpy.init();sub=Node('isolated_gid_observer');pub=Node('isolated_gid_publisher');tracker=GateObservationTracker()
  topic='/isolated_gate_status'
  sub.create_subscription(String,topic,lambda m:tracker.on_message(json.loads(m.data)),10)
  def wait(pred):
   end=time.monotonic()+5
   while time.monotonic()<end:
    rclpy.spin_once(sub,timeout_sec=.05)
    if pred():return
   self.fail('timed out')
  try:
   p=pub.create_publisher(String,topic,10)
   wait(lambda:len(sub.get_publishers_info_by_topic(topic))==1)
   first=bytes(sub.get_publishers_info_by_topic(topic)[0].endpoint_gid)
   msg=String(data=json.dumps({'state':'NORMAL','boot_id':'same-claimed-boot','fault_epoch':1,'fault_latched':False,'cmd_blocked':False}))
   p.publish(msg);wait(lambda:tracker.last_msg is not None)
   pub.destroy_publisher(p);wait(lambda:len(sub.get_publishers_info_by_topic(topic))==0)
   p2=pub.create_publisher(String,topic,10);wait(lambda:len(sub.get_publishers_info_by_topic(topic))==1)
   second=bytes(sub.get_publishers_info_by_topic(topic)[0].endpoint_gid);self.assertNotEqual(first,second)
   old=tracker.rx_mono;p2.publish(msg);wait(lambda:tracker.rx_mono!=old)
   self.assertFalse(tracker.boot_changed);self.assertTrue(tracker.evaluate()['contract_met'])
   print('SAFETY_GAP: GID changed, claimed boot unchanged, tracker still contract_met',flush=True)
  finally:
   sub.destroy_node();pub.destroy_node();rclpy.shutdown()
if __name__=='__main__':unittest.main(verbosity=2)
