import importlib.util,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'agent/limo_robot_agent'))
from limo_robot_agent.slam_control import SlamControl

class Robot:
 def __init__(self,nodes=(),topics=(),services=(),pubs=0):self.nodes=nodes;self.topics=topics;self.services=services;self.pubs=pubs;self.tfb=SimpleNamespace(can_transform=lambda *a:False)
 def get_node_names_and_namespaces(self):return [(x,'/') for x in self.nodes]
 def get_topic_names_and_types(self):return [(x,[]) for x in self.topics]
 def get_service_names_and_types(self):return [(x,[]) for x in self.services]
 def get_service_names_and_types_by_node(self,node,ns):return [(x,[]) for x in self.services]
 def count_publishers(self,_):return self.pubs
 def get_publishers_info_by_topic(self,_):return [SimpleNamespace(node_name='slam_toolbox')] if self.pubs else []
 def _use_sim_time(self):return False
class Tests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory(prefix='slam-test-');self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
 def test_rejects_existing_graph(self):
  for robot in (Robot(nodes=['slam_toolbox']),Robot(nodes=['amcl']),Robot(pubs=1),Robot(nodes=['foreign'],services=['/follow_path/_action/send_goal'])):
   with self.subTest(robot=robot):
    control=SlamControl(self.root/str(id(robot)),robot);self.addCleanup(control.close);calls=[]
    control.command({'type':'slam_start','payload':{'session_id':'test'}},lambda *a:calls.append(a))
    self.assertEqual('REJECTED',calls[-1][0]);self.assertIsNone(control.proc)
 def test_nav2_idle_evidence_allows_start_and_active_goal_rejects(self):
  gid=bytes.fromhex('0102030405060708')
  for verdict,expected in [('IDLE_PROVEN_EL','ACCEPTED'),('ACTIVE_GOALS','REJECTED'),('UNOBSERVED:no_event_no_sample','REJECTED')]:
   with self.subTest(verdict=verdict):
    robot=Robot(nodes=['controller_server'],services=['/follow_path/_action/send_goal'])
    robot.count_publishers=lambda t: int(t=='/follow_path/_action/status')
    robot.get_publishers_info_by_topic=lambda t: [SimpleNamespace(endpoint_gid=gid)]
    robot._gate_evaluation=lambda: {'contract_version':'gate-status/1.2-sim','trust':'UNAUTHENTICATED_SIM_ONLY','contract_met':True,'r7_invalid':False,'rx_fresh':True,'source_fresh':True,'single_publisher_graph':True,'nav_evidence':{'scope_version':1,'observed':True,'all_idle_proven':verdict=='IDLE_PROVEN_EL','active_uuids':[],'servers':{'follow_path':{'verdict':verdict,'writer_gid':gid.hex()}}}}
    control=SlamControl(self.root/verdict.split(':')[0],robot);self.addCleanup(control.close);calls=[]
    with patch('limo_robot_agent.slam_control.subprocess.Popen',return_value=SimpleNamespace(pid=1)), patch('limo_robot_agent.slam_control.threading.Thread.start'):
     control.command({'type':'slam_start','payload':{'session_id':'test'}},lambda *a:calls.append(a))
    self.assertEqual(expected,calls[-1][0])
 def test_existing_transform_rejected(self):
  robot=Robot();robot.tfb=SimpleNamespace(can_transform=lambda *a:True)
  c=SlamControl(self.root,robot);self.addCleanup(c.close);calls=[]
  c.command({'type':'slam_start','payload':{'session_id':'test'}},lambda *a:calls.append(a))
  self.assertEqual('REJECTED',calls[-1][0]);self.assertIn('map->odom',calls[-1][1])
 def test_manual_finish_keeps_slam_alive_when_stop_unconfirmed(self):
  c=SlamControl(self.root,Robot());self.addCleanup(c.close);c.set('STOPPING','test',session_id='test');calls=[]
  with patch.object(c,'_confirm_manual_stop',return_value=False), patch.object(c,'_save_map') as save, patch.object(c,'_stop') as stop:
   c._finish_manual('test','floor','cmd',lambda *a:calls.append(a))
  self.assertEqual('ERROR',c.state['status']);self.assertEqual('FAILED',calls[-1][0]);save.assert_not_called();stop.assert_not_called()
 def test_manual_finish_missing_floor_keeps_input_disabled(self):
  c=SlamControl(self.root,Robot());self.addCleanup(c.close);c.set('STOPPING','test',session_id='test');calls=[]
  with patch.object(c,'_confirm_manual_stop',return_value=True),patch.object(c,'_save_map') as save,patch.object(c,'_stop') as stop:
   c._finish_manual('test',None,'cmd',lambda *a:calls.append(a))
  self.assertEqual('ERROR',c.state['status']);self.assertEqual('FAILED',calls[-1][0]);save.assert_not_called();stop.assert_not_called()
 def test_manual_finish_save_exception_remains_error(self):
  c=SlamControl(self.root,Robot());self.addCleanup(c.close);c.set('STOPPING','test',session_id='test');calls=[]
  with patch.object(c,'_confirm_manual_stop',return_value=True),patch.object(c,'_save_map',side_effect=OSError('disk error')),patch.object(c,'_stop') as stop:
   c._finish_manual('test','floor','cmd',lambda *a:calls.append(a))
  self.assertEqual('ERROR',c.state['status']);self.assertIn('disk error',c.state['detail']);self.assertEqual('FAILED',calls[-1][0]);stop.assert_not_called()
 def test_manual_finish_saves_before_stopping(self):
  c=SlamControl(self.root,Robot());self.addCleanup(c.close);c.set('STOPPING','test',session_id='test');order=[]
  def save(sid,floor,cid,ack,manual=False):order.append('save');ack('SUCCEEDED','saved')
  def stop(ack,manual=False):order.append('stop');ack('SUCCEEDED','done')
  with patch.object(c,'_confirm_manual_stop',return_value=True),patch.object(c,'_save_map',side_effect=save),patch.object(c,'_stop',side_effect=stop):
   c._finish_manual('test','floor','cmd',lambda *a:None)
  self.assertEqual(['save','stop'],order)
 def test_failed_map_save_not_reported_success(self):
  c=SlamControl(self.root,Robot(pubs=1));self.addCleanup(c.close);(self.root/'s').mkdir();c.set('SAVING','test',session_id='s');c.proc=SimpleNamespace(poll=lambda:None);calls=[]
  with patch('limo_robot_agent.slam_control.subprocess.run',return_value=SimpleNamespace(returncode=1)):
   c._save_map('s','floor','cmd',lambda *a:calls.append(a))
  self.assertEqual('FAILED',calls[-1][0]);self.assertNotIn('map_version_id',c.state)
 def test_restart_preserves_unknown(self):
  c=SlamControl(self.root,Robot());c.set('RUNNING','test',session_id='s');c.close()
  d=SlamControl(self.root,Robot());self.addCleanup(d.close);self.assertEqual('UNKNOWN',d.snapshot()['status']);calls=[]
  d.command({'type':'slam_start','payload':{'session_id':'new'}},lambda *a:calls.append(a));self.assertEqual('REJECTED',calls[-1][0])
 def test_new_session_does_not_inherit_old_map(self):
  c=SlamControl(self.root,Robot());self.addCleanup(c.close)
  c.set('STOPPED','old',session_id='old',save_path='/old/map.yaml',map_version_id='old#123',floor_id='old-floor',posegraph_saved=True,posegraph_registered=True,returncode=0)
  calls=[]
  with patch('limo_robot_agent.slam_control.subprocess.Popen',return_value=SimpleNamespace(pid=1)), patch('limo_robot_agent.slam_control.threading.Thread.start'):
   c.command({'type':'slam_start','payload':{'session_id':'new'}},lambda *a:calls.append(a))
  self.assertEqual('ACCEPTED',calls[-1][0]);self.assertIsNone(c.state['map_version_id']);self.assertIsNone(c.state['floor_id']);self.assertFalse(c.state['posegraph_saved'])
if __name__=='__main__':unittest.main()
