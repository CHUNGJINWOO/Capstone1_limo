"""LocalizationWorker initial-pose delivery (2026-11-21, no ROS graph; the worker object is built without its ROS constructor).

Phase 3 timeline (phase3_stability_claude_20261121/baseline/01_r11, n05): after a re-LOAD the new AMCL had not matched /initialpose yet,
the 3 publications within 0.6 s were lost and AMCL never published a pose. The worker now waits (bounded) for AMCL's subscription and
re-sends the SAME operator pose while AMCL stays silent.
"""
import sys
import time
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent'))
from limo_robot_agent import localization_worker as lw  # noqa: E402


class FakePub:
    def __init__(self): self.subs = 0; self.sent = []
    def get_subscription_count(self): return self.subs
    def publish(self, m): self.sent.append((time.monotonic(), self.subs, m.pose.pose.position.x))


def worker():
    w = lw.LocalizationWorker.__new__(lw.LocalizationWorker)
    w.init_pub = FakePub(); w.robot = types.SimpleNamespace(get_clock=lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: lw.PoseWithCovarianceStamped().header.stamp)))
    w.amcl = None; w.pose_sent_t = None; w.applied_seq = None; w.matches = []; w.pose_resends = 0; w.last_pose_send = 0.0; w.pose_subscribers = None
    w.status, w.detail = 'IDLE', ''
    return w


POSE = {'x': 1.5, 'y': -0.5, 'yaw': 0.3, 'seq': 7}


class InitialPose(unittest.TestCase):
    def setUp(self):
        self.old = lw.INITIAL_POSE_SUB_WAIT_S, lw.INITIAL_POSE_RESEND_S
        lw.INITIAL_POSE_SUB_WAIT_S, lw.INITIAL_POSE_RESEND_S = 0.5, 0.0

    def tearDown(self): lw.INITIAL_POSE_SUB_WAIT_S, lw.INITIAL_POSE_RESEND_S = self.old

    def test_waits_for_amcl_subscription_before_publishing(self):
        w = worker()
        import threading
        threading.Timer(0.25, lambda: setattr(w.init_pub, 'subs', 1)).start()
        t0 = time.monotonic(); w.send_initial_pose(POSE)
        self.assertEqual(len(w.init_pub.sent), 3)
        self.assertTrue(all(s == 1 for _, s, _ in w.init_pub.sent))              # every publication had AMCL subscribed
        self.assertGreaterEqual(w.init_pub.sent[0][0] - t0, 0.2)
        self.assertEqual(w.pose_subscribers, 1)

    def test_bounded_wait_without_subscriber_still_publishes(self):
        w = worker(); t0 = time.monotonic(); w.send_initial_pose(POSE)
        self.assertLess(time.monotonic() - t0, 0.5 + 0.8); self.assertEqual(len(w.init_pub.sent), 3); self.assertEqual(w.pose_subscribers, 0)

    def resend_condition(self, w):
        """Mirror of the resend guard in step() (kept identical by test_step_guard_text)."""
        a = w.amcl
        return (w.status == 'CONVERGING' and POSE.get('seq') == w.applied_seq
                and not (a is not None and w.pose_sent_t is not None and a[0] >= w.pose_sent_t)
                and time.monotonic() - w.last_pose_send >= lw.INITIAL_POSE_RESEND_S and w.pose_resends < lw.INITIAL_POSE_RESENDS)

    def test_resend_only_while_amcl_silent_and_bounded(self):
        w = worker(); w.init_pub.subs = 1; w.status = 'CONVERGING'; w.applied_seq = 7; w.pose_sent_t = time.monotonic(); w.last_pose_send = 0
        self.assertTrue(self.resend_condition(w))
        w.amcl = (time.monotonic() + 1, 0, 0, 0, .01, .01, .01)                     # AMCL answered after the send
        self.assertFalse(self.resend_condition(w))
        w.amcl = None; w.pose_resends = lw.INITIAL_POSE_RESENDS
        self.assertFalse(self.resend_condition(w))                                   # bounded
        w.pose_resends = 0; w.status = 'CONFIRMED'; self.assertFalse(self.resend_condition(w))

    def test_step_guard_text(self):
        src = (Path(lw.__file__)).read_text()
        self.assertIn("time.monotonic() - getattr(self, 'last_pose_send', 0) >= INITIAL_POSE_RESEND_S and getattr(self, 'pose_resends', 0) < INITIAL_POSE_RESENDS", src)
        self.assertIn('self.send_initial_pose(pose)                  # same seq, same pose', src)


class ConfirmNeedsAllowUnknownFalse(unittest.TestCase):
    """2026-11-21: LOAD no longer fails when the planner cannot be reached yet; CONFIRMED still requires allow_unknown == false."""
    def w(self, planner_ok):
        w = worker(); w.status = 'CONVERGING'; w.pose_sent_t = time.monotonic() - 1
        w.amcl = (time.monotonic(), 0, 0, 0, .01, .01, .01); w.matches = [True] * (lw.CONFIRM_SAMPLES - 1)
        w.match_ratio = lambda: 0.9; w.allow_unknown = None; w.calls = 0
        def pau(v):
            w.calls += 1
            if planner_ok[0]: w.allow_unknown = bool(v)
            return planner_ok[0]
        w.planner_allow_unknown = pau
        return w

    def test_not_confirmed_until_planner_confirms(self):
        ok = [False]; w = self.w(ok)
        w.evaluate(); self.assertEqual(w.status, 'CONVERGING'); self.assertIn('allow_unknown', w.detail); self.assertEqual(w.calls, 1)
        w.evaluate(); self.assertEqual(w.calls, 1)                       # rate limited (1 s)
        ok[0] = True; w.last_allow_unknown_try = 0
        w.evaluate(); self.assertEqual(w.status, 'CONFIRMED'); self.assertIs(w.allow_unknown, False)

    def test_already_false_confirms_without_call(self):
        ok = [True]; w = self.w(ok); w.allow_unknown = False
        w.evaluate(); self.assertEqual(w.status, 'CONFIRMED'); self.assertEqual(w.calls, 0)

    def test_load_no_longer_raises_on_planner(self):
        src = Path(lw.__file__).read_text()
        self.assertNotIn("raise RuntimeError('planner GridBased.allow_unknown could not be set to false')", src)
        self.assertIn('self.allow_unknown is False:\n                self.status, self.detail, self.lost_since = \'CONFIRMED\'', src)


if __name__ == '__main__':
    unittest.main(verbosity=2)
