"""Supplemental synthetic regression; preserves all legacy tests unchanged."""
import copy
import unittest
import test_r3_r4_server_safety as fixture
from fleet_server.auth import Principal
from fleet_server.service import ApiError


class Regression(fixture.Tests):
 def test_recovery_refuses_unreliable_goal_evidence(self):
  mission,_=self.create(); self.f._deliver('r'); self.fault()
  original=copy.deepcopy(self.f.robots['r']['state'])
  invalid=[{'known':True,'active_count':0},
           {'known':True,'active_count':0,'received_status_array':False,'age_s':0},
           {'known':True,'active_count':0,'received_status_array':True,'age_s':3},
           {'known':True,'active_count':-1,'received_status_array':True,'age_s':0},
           {'known':True,'active_count':False,'received_status_array':True,'age_s':0}]
  for evidence in invalid:
   with self.subTest(evidence=evidence):
    self.f.robots['r']['reconciled']=True
    self.f.robots['r']['state']['execution']['nav2_goals']=evidence
    with self.assertRaises(ApiError):
     self.f.recover_mission(mission['mission_id'],Principal('admin','admin'),'verified no active goal')
    self.assertEqual(self.f.mission(mission['mission_id'])['status'],'RECONCILIATION_REQUIRED')
  self.f.robots['r']['state']=original
  self.f.robots['r']['reconciled']=True
  with self.assertRaises(ApiError) as refused:
   self.f.recover_mission(mission['mission_id'],Principal('admin','admin'),'verified no active goal')
  self.assertEqual(refused.exception.status,409)
  self.assertEqual(self.f.mission(mission['mission_id'])['status'],'RECONCILIATION_REQUIRED')

 def test_lease_owner_conflict_and_renewal(self):
  with self.assertRaises(ApiError):
   self.f.lease_action('r',Principal('other','operator'),'acquire')
  renewed=self.f.lease_action('r',Principal('op','operator'),'renew')
  self.assertEqual(renewed['epoch'],self.lease)
  with self.assertRaises(ApiError):
   self.f.create_mission('r','navigate_to_pose',fixture.GOAL,'other',created_by='other',lease_epoch=self.lease)

if __name__=='__main__': unittest.main(verbosity=2)
