"""SIM_ONLY limited trust model for the Nav2-internal FollowPath child of the Agent's own NavigateToPose goal.
Synthetic (no Gazebo): pure classification in Agent safety_protocol and the Server re-derivation. These tests show that the child is attributed
ONLY in the narrow case and that every other active UUID stays foreign; they do NOT prove that external goals are blocked on a real robot."""
import copy
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'agent/limo_robot_agent'), str(ROOT / 'server')]
from limo_robot_agent import safety_protocol as sp
from fleet_server import safety as server_safety

OWN, CHILD, OTHER, OTHER2 = 'a' * 32, 'b' * 32, 'c' * 32, 'd' * 32
GID = '0123456789abcdef'
ACTIONS = ('navigate_to_pose', 'navigate_through_poses', 'follow_path', 'spin', 'backup')


def evidence(nav=(), fp=(), spin=(), through=(), backup=()):
    def rec(ids):
        return {'verdict': 'ACTIVE_GOALS' if ids else 'IDLE_PROVEN_EL', 'active_uuids': sorted(ids), 'writer_gid': GID}
    servers = {'navigate_to_pose': rec(nav), 'navigate_through_poses': rec(through), 'follow_path': rec(fp), 'spin': rec(spin), 'backup': rec(backup)}
    ids = sorted({*nav, *fp, *spin, *through, *backup})
    return {'scope_version': 1, 'observed': True, 'all_idle_proven': not ids, 'active_uuids': ids, 'servers': servers}


def gate(ev):
    return {'contract_version': 'gate-status/1.2-sim', 'trust': 'UNAUTHENTICATED_SIM_ONLY', 'single_publisher_graph': True, 'contract_met': True,
            'rx_fresh': True, 'source_fresh': True, 'safety_state': 'NORMAL', 'age_s': 0.05, 'nav_evidence': ev}


def direct(ids):
    return {'source': 'navigate_to_pose/_action/status', 'known': True, 'publishers': 1, 'active_count': len(ids), 'received_status_array': True,
            'age_s': 0.05, 'last_change_age_s': 0.05, 'active_goal_uuids': list(ids), 'foreign_active_goal_uuids': [u for u in ids if u != OWN]}


class Classify(unittest.TestCase):
    def test_single_child_with_provenance_is_subordinate(self):
        sub, foreign, basis = sp.sim_classify_nav2_goals(evidence(nav=[OWN], fp=[CHILD]), OWN, lambda c: c == CHILD)
        self.assertEqual((sub, foreign), ([CHILD], []))
        self.assertEqual(basis, 'FOLLOW_PATH_CHILD_OF_OWNED_NAVIGATE_TO_POSE')

    def test_uuid_difference_alone_is_not_enough(self):
        for ok in (None, lambda c: False):
            sub, foreign, basis = sp.sim_classify_nav2_goals(evidence(nav=[OWN], fp=[CHILD]), OWN, ok)
            self.assertEqual((sub, foreign, basis), ([], [CHILD], 'CHILD_PROVENANCE_UNCONFIRMED'))

    def test_two_followpath_goals_are_a_handover_only_with_provenance_and_short_overlap(self):
        ev = evidence(nav=[OWN], fp=[CHILD, OTHER])
        sub, foreign, basis = sp.sim_classify_nav2_goals(ev, OWN, lambda c: True, lambda pair: True)
        self.assertEqual((sorted(sub), foreign), (sorted([CHILD, OTHER]), [])); self.assertTrue(basis.endswith('_HANDOVER'))
        for ok, ov in ((lambda c: True, None), (lambda c: True, lambda p: False), (lambda c: c == CHILD, lambda p: True), (None, lambda p: True)):
            sub, foreign, _ = sp.sim_classify_nav2_goals(ev, OWN, ok, ov)
            self.assertEqual((sub, sorted(foreign)), ([], sorted([CHILD, OTHER])))

    def test_three_followpath_goals_stay_foreign(self):
        sub, foreign, _ = sp.sim_classify_nav2_goals(evidence(nav=[OWN], fp=[CHILD, OTHER, OTHER2]), OWN, lambda c: True, lambda p: True)
        self.assertEqual((sub, sorted(foreign)), ([], sorted([CHILD, OTHER, OTHER2])))

    def test_second_navigate_to_pose_goal_is_foreign_and_child_not_attributed(self):
        sub, foreign, _ = sp.sim_classify_nav2_goals(evidence(nav=[OWN, OTHER], fp=[CHILD]), OWN, lambda c: True)
        self.assertEqual(sub, []); self.assertEqual(sorted(foreign), sorted([CHILD, OTHER]))

    def test_followpath_without_owned_goal_is_foreign(self):
        sub, foreign, _ = sp.sim_classify_nav2_goals(evidence(fp=[CHILD]), OWN, lambda c: True)
        self.assertEqual((sub, foreign), ([], [CHILD]))
        sub, foreign, _ = sp.sim_classify_nav2_goals(evidence(nav=[OTHER], fp=[CHILD]), OWN, lambda c: True)
        self.assertEqual(sub, []); self.assertEqual(sorted(foreign), sorted([OTHER, CHILD]))

    def test_other_active_action_blocks_attribution(self):
        for kw in ({'spin': [OTHER]}, {'backup': [OTHER]}, {'through': [OTHER]}):
            sub, foreign, _ = sp.sim_classify_nav2_goals(evidence(nav=[OWN], fp=[CHILD], **kw), OWN, lambda c: True)
            self.assertEqual(sub, []); self.assertIn(OTHER, foreign); self.assertIn(CHILD, foreign)

    def test_no_child_no_foreign(self):
        self.assertEqual(sp.sim_classify_nav2_goals(evidence(nav=[OWN]), OWN, lambda c: True)[:2], ([], []))


class Evidence(unittest.TestCase):
    def run_ev(self, ev, direct_ids, owned_age=0.5, ok=lambda c: True, owned=OWN):
        return sp.sim_r7_nav_evidence(gate(ev), ACTIONS, direct(direct_ids), owned, owned_age_s=owned_age, subordinate_ok=ok)

    def test_child_does_not_make_goals_foreign(self):
        r = self.run_ev(evidence(nav=[OWN], fp=[CHILD]), [OWN])
        self.assertEqual(r['foreign_active_goal_uuids'], []); self.assertEqual(r['subordinate_active_goal_uuids'], [CHILD])
        self.assertEqual(sorted(r['active_goal_uuids']), sorted([OWN, CHILD])); self.assertEqual(r['active_count'], 2)

    def test_unconfirmed_child_is_foreign(self):
        r = self.run_ev(evidence(nav=[OWN], fp=[CHILD]), [OWN], ok=lambda c: False)
        self.assertEqual(r['foreign_active_goal_uuids'], [CHILD]); self.assertEqual(r['subordinate_active_goal_uuids'], [])

    def test_pending_evidence_window(self):
        idle = evidence()
        r = self.run_ev(idle, [OWN], owned_age=0.3)
        self.assertIsNotNone(r); self.assertEqual(r['owned_pending_evidence'], OWN); self.assertEqual(r['active_goal_uuids'], []); self.assertEqual(r['foreign_active_goal_uuids'], [])
        self.assertIsNone(self.run_ev(idle, [OWN], owned_age=sp.SIM_PENDING_EVIDENCE_GRACE_S + 0.1), 'expired grace must be UNKNOWN, not normal')
        self.assertIsNone(self.run_ev(idle, [OWN], owned_age=None))

    def test_pending_window_never_excuses_other_uuid(self):
        self.assertIsNone(self.run_ev(evidence(), [OTHER], owned_age=0.2))
        self.assertIsNone(self.run_ev(evidence(), [OWN, OTHER], owned_age=0.2))
        self.assertIsNone(self.run_ev(evidence(fp=[CHILD]), [OWN], owned_age=0.2))   # evidence not idle: the lag excuse does not apply

    def test_direct_sample_contradicting_evidence_is_unknown(self):
        self.assertIsNone(self.run_ev(evidence(nav=[OTHER]), [OWN]))

    def test_direct_foreign_seen_by_agent_is_still_foreign(self):
        r = self.run_ev(evidence(nav=[OWN, OTHER]), [OWN, OTHER])
        self.assertEqual(r['foreign_active_goal_uuids'], [OTHER])

    def test_movement_permission_reasons(self):
        r = self.run_ev(evidence(nav=[OWN, OTHER]), [OWN, OTHER])
        self.assertTrue(r['foreign_active_goal_uuids'])


class ServerRederivation(unittest.TestCase):
    def goals(self, ev, sub, foreign=()):
        return {'gate_nav_evidence': ev, 'subordinate_active_goal_uuids': list(sub), 'foreign_active_goal_uuids': list(foreign)}

    def test_valid_and_invalid(self):
        ev = evidence(nav=[OWN], fp=[CHILD])
        self.assertTrue(server_safety.sim_subordinate_valid(self.goals(ev, [CHILD]), OWN))
        self.assertTrue(server_safety.sim_subordinate_valid(self.goals(ev, []), OWN))                       # nothing claimed
        self.assertFalse(server_safety.sim_subordinate_valid(self.goals(ev, [CHILD]), None))                # no owned goal known to the server
        self.assertFalse(server_safety.sim_subordinate_valid(self.goals(ev, [OTHER]), OWN))                 # claimed UUID is not what the Gate saw
        self.assertFalse(server_safety.sim_subordinate_valid(self.goals(ev, [CHILD], [OTHER]), OWN))
        self.assertTrue(server_safety.sim_subordinate_valid(self.goals(evidence(nav=[OWN], fp=[CHILD, OTHER]), [CHILD, OTHER]), OWN))      # hand-over pair
        self.assertFalse(server_safety.sim_subordinate_valid(self.goals(evidence(nav=[OWN], fp=[CHILD, OTHER]), [CHILD]), OWN))           # claimed only one of two
        self.assertFalse(server_safety.sim_subordinate_valid(self.goals(evidence(nav=[OWN], fp=[CHILD, OTHER, OTHER2]), [CHILD, OTHER, OTHER2]), OWN))
        self.assertFalse(server_safety.sim_subordinate_valid(self.goals(evidence(nav=[OWN, OTHER], fp=[CHILD]), [CHILD]), OWN))
        self.assertFalse(server_safety.sim_subordinate_valid(self.goals(evidence(nav=[OWN], fp=[CHILD], spin=[OTHER]), [CHILD]), OWN))
        self.assertFalse(server_safety.sim_subordinate_valid(self.goals(evidence(nav=[OTHER], fp=[CHILD]), [CHILD]), OWN))
        bad = copy.deepcopy(ev); bad['servers']['follow_path']['active_uuids'] = []
        self.assertFalse(server_safety.sim_subordinate_valid(self.goals(bad, [CHILD]), OWN))


if __name__ == '__main__':
    unittest.main()
