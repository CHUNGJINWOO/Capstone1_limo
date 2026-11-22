"""Frontier goal selection for the web mapping session (pure functions, no ROS) - 2026-11-21.

Problem it fixes (Gazebo Office, GATE_STALL_GAZEBO_20261121.md §6.1): the first SLAM scan of an open hall gave ONE candidate
~0.25 m from the robot. Reaching it moved the robot less than SLAM's minimum_travel_distance (0.2 m) / heading (0.2 rad), so the
map - and therefore the candidate - did not change, and the worker never re-offered an attempted cell: exploration stalled
with the operator seeing nothing but a running session.

Rules (each decision is reported, so "blocked" and "nothing left" are distinguishable from a stall):
  1. candidates closer than MIN_GOAL_DIST to the robot are skipped (they cannot make SLAM add a scan);
  2. a candidate within VISITED_RADIUS of a goal that SUCCEEDED is skipped (it was reached and is still a frontier: unresolvable);
  3. a candidate within FAILED_RADIUS of a goal that FAILED/CANCELED/REJECTED is skipped for RETRY_AFTER_S after the first
     failure and for good after MAX_FAILS failures there (blocked / unreachable);
  4. no usable candidate -> a REPOSITION goal: a known-free cell 0.6-1.5 m away with clearance, scored by the unknown cells
     around it (moves the robot enough for SLAM to add scans);
  5. nothing usable at all -> EXHAUSTED; MAX_CONSECUTIVE_FAILS failed goals in a row -> GOALS_FAILING (probably blocked).
The worker still never claims completion: EXHAUSTED / GOALS_FAILING only stop offering goals; the operator decides SAVE / STOP.
"""
import math

MIN_GOAL_DIST = 0.6
VISITED_RADIUS = 0.5
FAILED_RADIUS = 0.5
RETRY_AFTER_S = 20.0
MAX_FAILS = 2
MAX_CONSECUTIVE_FAILS = 3
MAX_REPOSITIONS = 12
REPOSITION_MIN, REPOSITION_MAX = 0.6, 1.5
CLEARANCE_M = 0.30
INFO_RADIUS_M = 1.0
DONE_FAIL = ('FAILED', 'CANCELED', 'REJECTED')


def _near(a, b, r):
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= r


def candidate_verdict(xy, robot_xy, outcomes, now):
    """None = usable, otherwise the skip reason. outcomes: list of dicts {x, y, status, t}."""
    if not all(math.isfinite(v) for v in xy):
        return 'NOT_FINITE'
    if robot_xy is not None and _near(xy, robot_xy, MIN_GOAL_DIST):
        return 'TOO_CLOSE'
    if any(o['status'] == 'SUCCEEDED' and _near(xy, (o['x'], o['y']), VISITED_RADIUS) for o in outcomes):
        return 'VISITED'
    fails = [o for o in outcomes if o['status'] in DONE_FAIL and _near(xy, (o['x'], o['y']), FAILED_RADIUS)]
    if len(fails) >= MAX_FAILS:
        return 'FAILED_TWICE'
    if fails and now - max(o['t'] for o in fails) < RETRY_AFTER_S:
        return 'FAILED_RECENTLY'
    return None


def pick_candidate(cands, robot_xy, outcomes, now):
    """First usable candidate in the node's ranking order; returns (xy or None, {reason: count})."""
    skipped = {}
    for xy in cands:
        why = candidate_verdict(xy, robot_xy, outcomes, now)
        if why is None:
            return xy, skipped
        skipped[why] = skipped.get(why, 0) + 1
    return None, skipped


def pick_reposition(grid, meta, robot_xy, outcomes, now):
    """grid: 2-D int array (row 0 = origin_y, OccupancyGrid order), values -1 unknown / 0..100. Returns (xy, score) or (None, 0)."""
    if grid is None or not meta or robot_xy is None:
        return None, 0
    res, ox, oy = meta['resolution'], meta['origin_x'], meta['origin_y']
    h, w = grid.shape
    rc = lambda x, y: (int((y - oy) / res), int((x - ox) / res))
    r0, c0 = rc(*robot_xy)
    reach, clear, info = int(REPOSITION_MAX / res) + 1, max(1, int(CLEARANCE_M / res)), max(1, int(INFO_RADIUS_M / res))
    best, best_score = None, 0
    step = max(1, int(0.1 / res))
    for r in range(max(0, r0 - reach), min(h, r0 + reach + 1), step):
        for c in range(max(0, c0 - reach), min(w, c0 + reach + 1), step):
            v = grid[r, c]
            if v < 0 or v >= 25:
                continue
            x, y = ox + (c + .5) * res, oy + (r + .5) * res
            d = math.hypot(x - robot_xy[0], y - robot_xy[1])
            if not REPOSITION_MIN <= d <= REPOSITION_MAX:
                continue
            win = grid[max(0, r - clear):r + clear + 1, max(0, c - clear):c + clear + 1]
            if (win >= 65).any() or (win < 0).any():
                continue          # needs clearance and must be inside known free space (planner allow_unknown may be off)
            if candidate_verdict((x, y), None, outcomes, now) is not None:
                continue
            around = grid[max(0, r - info):r + info + 1, max(0, c - info):c + info + 1]
            score = int((around < 0).sum())
            if score > best_score or (score == best_score and best is not None and d > math.hypot(best[0] - robot_xy[0], best[1] - robot_xy[1])):
                best, best_score = (round(x, 2), round(y, 2)), score
    return (best, best_score) if best_score > 0 else (None, 0)


def consecutive_failures(outcomes):
    n = 0
    for o in reversed(outcomes):
        if o['status'] in DONE_FAIL: n += 1
        else: break
    return n
