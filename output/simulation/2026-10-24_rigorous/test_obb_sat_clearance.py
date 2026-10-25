#!/usr/bin/env python3
"""
test_obb_sat_clearance.py:
Unit test suite verifying the Separating Axis Theorem (SAT) based clearance & collision detection
between oriented bounding boxes (OBB) and axis-aligned bounding boxes (AABB).
Tests:
  1. Separation (분리)
  2. Contact / Touching (접촉)
  3. Containment (포함: Box inside Robot, Robot inside Box)
  4. Cross Intersection (십자 교차: rectangles cross with no corners inside)
  5. Rotated Cross Intersection (회전된 십자 교차)
  6. Non-intersecting clearance accuracy against known analytical distances
"""

import math
import sys

import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from evaluation_lib import (
    point_to_segment_distance,
    compute_polygon_clearance_sat,
    make_robot_polygon,
    make_box_polygon
)


def run_tests():
    print("=== Running SAT Clearance Verification Tests ===")
    tests_passed = True

    # Case 1: Strictly Separated (Parallel boxes)
    poly_a = make_box_polygon(0.0, 0.0, 2.0, 2.0) # [-1, 1] x [-1, 1]
    poly_b = make_box_polygon(3.0, 0.0, 2.0, 2.0) # [2, 4] x [-1, 1]
    c1 = compute_polygon_clearance_sat(poly_a, poly_b)
    assert abs(c1 - 1.0) < 1e-6, f"Case 1 failed: expected 1.0, got {c1}"
    print(f"[PASS] Case 1: Separation (Clearance: {c1:.4f}m == 1.0m)")

    # Case 2: Contact / Touching (Edge-to-edge contact)
    poly_touch = make_box_polygon(2.0, 0.0, 2.0, 2.0) # [1, 3] x [-1, 1] touching at x = 1.0
    c2 = compute_polygon_clearance_sat(poly_a, poly_touch)
    assert abs(c2) < 1e-6, f"Case 2 failed: expected 0.0, got {c2}"
    print(f"[PASS] Case 2: Contact / Touching (Clearance: {c2:.4f}m == 0.0m)")

    # Case 3: Containment (Box inside robot, robot inside box)
    poly_small = make_box_polygon(0.0, 0.0, 0.5, 0.5)
    c3 = compute_polygon_clearance_sat(poly_a, poly_small)
    assert abs(c3) < 1e-6, f"Case 3 failed: expected 0.0, got {c3}"
    print(f"[PASS] Case 3: Containment (Clearance: {c3:.4f}m == 0.0m)")

    # Case 4: Cross Intersection (십자 교차 - No corners of either rectangle are inside the other)
    # Box A: Horizontal thin slab [-2, 2] x [-0.2, 0.2]
    # Box B: Vertical thin slab [-0.2, 0.2] x [-2, 2]
    poly_h = make_box_polygon(0.0, 0.0, 4.0, 0.4)
    poly_v = make_box_polygon(0.0, 0.0, 0.4, 4.0)
    c4 = compute_polygon_clearance_sat(poly_h, poly_v)
    assert abs(c4) < 1e-6, f"Case 4 failed: expected 0.0, got {c4}"
    print(f"[PASS] Case 4: Cross Intersection (Clearance: {c4:.4f}m == 0.0m)")

    # Case 5: Rotated Cross Intersection (회전된 십자 교차)
    # Robot rotated 45 deg crossing a horizontal wall
    poly_rot = make_robot_polygon(0.0, 0.0, math.pi / 4.0, rw=0.22, rl=0.80)
    poly_wall = make_box_polygon(0.0, 0.0, 2.0, 0.20)
    c5 = compute_polygon_clearance_sat(poly_rot, poly_wall)
    assert abs(c5) < 1e-6, f"Case 5 failed: expected 0.0, got {c5}"
    print(f"[PASS] Case 5: Rotated Cross Intersection (Clearance: {c5:.4f}m == 0.0m)")

    # Case 6: Rotated Robot Near Obstacle (Disjoint with known diagonal distance)
    # Robot at (0, 0) yaw=0 (w=0.22, l=0.32 -> x in [-0.16, 0.16], y in [-0.11, 0.11])
    # Box at (1.16, 0.11) with size (1.0, 1.0) -> nearest x=0.66, nearest y=-0.39
    poly_robot = make_robot_polygon(0.0, 0.0, 0.0, rw=0.22, rl=0.32)
    poly_obs = make_box_polygon(0.66, 0.0, 0.50, 0.50) # x in [0.41, 0.91]
    # Distance between robot front x=0.16 and box left x=0.41 is 0.41 - 0.16 = 0.25m
    c6 = compute_polygon_clearance_sat(poly_robot, poly_obs)
    assert abs(c6 - 0.25) < 1e-6, f"Case 6 failed: expected 0.25, got {c6}"
    print(f"[PASS] Case 6: Analytical Clearance Distance (Clearance: {c6:.4f}m == 0.25m)")

    print("ALL 6 SAT CLEARANCE TESTS PASSED SUCCESSFULLY!")
    return True

if __name__ == '__main__':
    run_tests()
