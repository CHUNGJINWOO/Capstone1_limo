"""frontier_policy (2026-11-21): the Gazebo Office stall case and the visited / failed / exhausted / blocked rules."""
import sys, unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'agent' / 'limo_robot_agent'))
from limo_robot_agent import frontier_policy as fp   # noqa: E402

META = {'resolution': 0.05, 'origin_x': -5.0, 'origin_y': -5.0}


def grid(fill=-1):
    return np.full((200, 200), fill, dtype=np.int8)       # 10 x 10 m, origin (-5, -5)


def cell(x, y):
    return int((y + 5.0) / 0.05), int((x + 5.0) / 0.05)


def free_disc(g, cx, cy, r):
    for y in np.arange(cy - r, cy + r, 0.05):
        for x in np.arange(cx - r, cx + r, 0.05):
            if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                g[cell(x, y)] = 0


class Policy(unittest.TestCase):
    def test_gazebo_stall_case_single_near_candidate_gives_reposition(self):
        # first scan only: free disc of 1.8 m around the robot, everything else unknown; one candidate 0.25 m away
        g = grid(); free_disc(g, 0, 0, 1.8)
        xy, skipped = fp.pick_candidate([(0.20, 0.15)], (0.0, 0.0), [], 100.)
        self.assertIsNone(xy); self.assertEqual(skipped, {'TOO_CLOSE': 1})
        rp, score = fp.pick_reposition(g, META, (0.0, 0.0), [], 100.)
        self.assertIsNotNone(rp); self.assertGreater(score, 0)
        d = (rp[0] ** 2 + rp[1] ** 2) ** .5
        self.assertTrue(fp.REPOSITION_MIN <= d <= fp.REPOSITION_MAX, d)    # far enough for SLAM's 0.2 m update threshold

    def test_far_candidate_is_used_in_ranking_order(self):
        xy, _ = fp.pick_candidate([(0.1, 0.0), (2.0, 0.0), (3.0, 0.0)], (0.0, 0.0), [], 0.)
        self.assertEqual(xy, (2.0, 0.0))

    def test_visited_and_failed_rules(self):
        out = [{'x': 2.0, 'y': 0.0, 'status': 'SUCCEEDED', 't': 0.}]
        self.assertEqual(fp.candidate_verdict((2.2, 0.1), (0., 0.), out, 10.), 'VISITED')
        out = [{'x': 2.0, 'y': 0.0, 'status': 'FAILED', 't': 0.}]
        self.assertEqual(fp.candidate_verdict((2.1, 0.0), (0., 0.), out, 5.), 'FAILED_RECENTLY')
        self.assertIsNone(fp.candidate_verdict((2.1, 0.0), (0., 0.), out, 5. + fp.RETRY_AFTER_S))   # one retry allowed later
        out.append({'x': 2.05, 'y': 0.0, 'status': 'CANCELED', 't': 30.})
        self.assertEqual(fp.candidate_verdict((2.1, 0.0), (0., 0.), out, 500.), 'FAILED_TWICE')       # blocked for good

    def test_exhausted_when_everything_known(self):
        g = grid(0)
        g[0, :] = 100
        self.assertEqual(fp.pick_reposition(g, META, (0.0, 0.0), [], 0.), (None, 0))

    def test_reposition_keeps_clearance_and_stays_in_known_free_space(self):
        g = grid(); free_disc(g, 0, 0, 1.8)
        for y in np.arange(-2, 2, 0.05):
            g[cell(1.0, y)] = 100                                      # wall at x = 1.0
        rp, _ = fp.pick_reposition(g, META, (0.0, 0.0), [], 0.)
        self.assertIsNotNone(rp)
        self.assertTrue(abs(rp[0] - 1.0) > fp.CLEARANCE_M - 0.05, rp)
        r, c = cell(*rp); self.assertEqual(int(g[r, c]), 0)

    def test_consecutive_failures(self):
        o = [{'status': s} for s in ('SUCCEEDED', 'FAILED', 'CANCELED', 'REJECTED')]
        self.assertEqual(fp.consecutive_failures(o), 3)
        self.assertEqual(fp.consecutive_failures(o + [{'status': 'SUCCEEDED'}]), 0)


if __name__ == '__main__':
    unittest.main()
