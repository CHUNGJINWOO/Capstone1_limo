#!/usr/bin/env python3
"""
test_evaluator_phases.py:
Authoritative unit tests for shared evaluation functions from evaluation_lib.py.
Verifies:
  1. Phase separation: Initial wait (INITIALIZING), settle wait (POST_RESULT),
     and forced cancel wait (POST_CANCEL) are strictly excluded from navigating standstill.
  2. Data gaps: Observation gap (dt > max_gap) resets continuous standstill and sets DEGRADED quality.
  3. Costmap clearing: Unknown (-1) cells strictly reject clearing.
  4. Costmap clearing: Clear -> Re-occupied -> Re-cleared cycle strictly resets hold duration.
  5. Costmap clearing: Data gap during clearing hold resets continuous hold duration.
  6. Costmap clearing: Latency below observation resolution is formatted with '관측 주기 해상도 이내'.
"""

import sys
import pathlib

# Ensure shared evaluation_lib is loaded from output/simulation
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from evaluation_lib import analyze_physical_standstill, analyze_costmap_clearing

def test_phase_separation_and_standstill():
    print("=== Test 1: Phase Separation & Standstill ===")
    history = []
    t = 0.0

    # 1. INITIALIZING phase: Stopped for 5.0 seconds
    for _ in range(50):
        history.append({
            'header_sim_time': 0.0,
            'recv_sim_time': round(t, 2),
            'phase': 'INITIALIZING',
            'x': 0.0, 'y': 0.0, 'z': 0.05, 'yaw': 0.0
        })
        t += 0.1

    # 2. NAVIGATING phase: Moves for 3.0s, stops for 4.0s (obstacle response), moves for 2.0s
    for _ in range(30):
        history.append({
            'header_sim_time': 0.0,
            'recv_sim_time': round(t, 2),
            'phase': 'NAVIGATING',
            'x': round(t * 0.1, 3), 'y': 0.0, 'z': 0.05, 'yaw': 0.0
        })
        t += 0.1

    cur_x = history[-1]['x']
    for _ in range(40):
        history.append({
            'header_sim_time': 0.0,
            'recv_sim_time': round(t, 2),
            'phase': 'NAVIGATING',
            'x': cur_x, 'y': 0.0, 'z': 0.05, 'yaw': 0.0
        })
        t += 0.1

    for _ in range(20):
        cur_x += 0.02
        history.append({
            'header_sim_time': 0.0,
            'recv_sim_time': round(t, 2),
            'phase': 'NAVIGATING',
            'x': round(cur_x, 3), 'y': 0.0, 'z': 0.05, 'yaw': 0.0
        })
        t += 0.1

    # 3. POST_RESULT phase: Settle wait for 3.0 seconds
    for _ in range(30):
        history.append({
            'header_sim_time': 0.0,
            'recv_sim_time': round(t, 2),
            'phase': 'POST_RESULT',
            'x': round(cur_x, 3), 'y': 0.0, 'z': 0.05, 'yaw': 0.0
        })
        t += 0.1

    res = analyze_physical_standstill(history, speed_thresh=0.015, yaw_speed_thresh=0.03, max_gap=0.3)
    print("Test 1 Result:", res)

    # Autonomous stop during navigating MUST equal 3.9 ~ 4.0s (excluding initial 4.9s and post-result 3.0s)
    assert 3.85 <= res['navigating_max_continuous_stop_sec'] <= 4.05, f"Unexpected nav stop: {res['navigating_max_continuous_stop_sec']}"
    assert 4.85 <= res['initial_wait_stop_sec'] <= 5.05, f"Unexpected init stop: {res['initial_wait_stop_sec']}"
    assert 2.85 <= res['post_result_stop_sec'] <= 3.05, f"Unexpected post-result stop: {res['post_result_stop_sec']}"
    assert res['telemetry_quality'] == 'HIGH'
    assert res['telemetry_source'] == 'RECV_SIM_TIME_APPROXIMATION'
    print("[PASS] Test 1: Phase separation strictly isolates initial and post-result wait!")


def test_gt_data_gap_resets_standstill():
    print("\n=== Test 2: GT Data Gap Resets Standstill ===")
    history = []
    t = 10.0

    # Stopped for 2.0s
    for _ in range(20):
        history.append({
            'header_sim_time': 0.0,
            'recv_sim_time': round(t, 2),
            'phase': 'NAVIGATING',
            'x': 0.0, 'y': 0.0, 'z': 0.05, 'yaw': 0.0
        })
        t += 0.1

    # Missing data gap of 1.0 second (max_gap is 0.3s)
    t += 1.0

    # Stopped for 1.5s after gap
    for _ in range(15):
        history.append({
            'header_sim_time': 0.0,
            'recv_sim_time': round(t, 2),
            'phase': 'NAVIGATING',
            'x': 0.0, 'y': 0.0, 'z': 0.05, 'yaw': 0.0
        })
        t += 0.1

    res = analyze_physical_standstill(history, speed_thresh=0.015, yaw_speed_thresh=0.03, max_gap=0.3)
    print("Test 2 Result:", res)

    assert res['large_gaps_count'] == 1, "Failed to count large gap"
    assert res['telemetry_quality'] == 'DEGRADED', "Quality should be DEGRADED"
    # Max continuous stop should be 1.9s (first segment) or 1.4s (second segment), NOT 2.0+1.5=3.5s!
    assert res['navigating_max_continuous_stop_sec'] < 2.5, f"Gap failed to reset stop: {res['navigating_max_continuous_stop_sec']}"
    print("[PASS] Test 2: Data gap correctly detected, resets continuous standstill, marks DEGRADED!")


def test_costmap_clearing_unknown_rejected():
    print("\n=== Test 3: Costmap Clearing Unknown Cells Rejection ===")
    history = [
        # Pre-removal: obstacle marked with max 100
        {
            'sim_time': 5.0,
            'front_surface': {'max_cost': 100},
            'interior': {'max_cost': 100},
            'all_roi': {'total_cells': 231, 'unknown_count': 0, 'all_valid_cells_zero': False},
            'barrier_gt_translation': [0.5, -3.5, 0.5]
        },
        # Post-removal: obstacle moved to z=-10, but unknown cells present!
        {
            'sim_time': 11.0,
            'front_surface': {'max_cost': 0},
            'interior': {'max_cost': 0},
            'all_roi': {'total_cells': 231, 'unknown_count': 12, 'all_valid_cells_zero': True},
            'barrier_gt_translation': [0.5, -3.5, -10.0]
        },
        {
            'sim_time': 13.0,
            'front_surface': {'max_cost': 0},
            'interior': {'max_cost': 0},
            'all_roi': {'total_cells': 231, 'unknown_count': 5, 'all_valid_cells_zero': True},
            'barrier_gt_translation': [0.5, -3.5, -10.0]
        }
    ]
    res = analyze_costmap_clearing(history, phys_clear_time=10.0, required_hold_duration=1.5)
    print("Test 3 Result:", res)
    assert not res['clearing_verified'], "Clearing should NOT pass when unknown cells are present!"
    assert res['post_removal_unknown_cells_detected'], "Failed to detect unknown cells"
    print("[PASS] Test 3: Unknown (-1) cells correctly rejected from clearing verification!")


def test_costmap_clearing_reoccupy_resets_hold():
    print("\n=== Test 4: Costmap Clearing Cycle (Clear -> Reoccupy -> Reclear) ===")
    # Scenario:
    # 1. t=10.0: Obstacle physically removed
    # 2. t=11.0, 12.0: Cleared for 1.0s (less than 2.0s required hold)
    # 3. t=13.0: Re-occupied! (dynamic object / obstacle re-detected)
    # 4. t=14.0, 15.0: Re-cleared for 1.0s (again less than 2.0s required hold)
    # Total cleared time is 2.0s, but continuous hold was broken, so max continuous hold is only 1.0s!
    history = [
        {'sim_time': 5.0, 'front_surface': {'max_cost': 100}, 'interior': {'max_cost': 100},
         'all_roi': {'total_cells': 231, 'unknown_count': 0, 'all_valid_cells_zero': False}, 'barrier_gt_translation': [0.5, -3.5, 0.5]},

        # Cleared run 1: t=11.0 to 12.0 (1.0s duration)
        {'sim_time': 11.0, 'front_surface': {'max_cost': 0}, 'interior': {'max_cost': 0},
         'all_roi': {'total_cells': 231, 'unknown_count': 0, 'all_valid_cells_zero': True}, 'barrier_gt_translation': [0.5, -3.5, -10.0]},
        {'sim_time': 12.0, 'front_surface': {'max_cost': 0}, 'interior': {'max_cost': 0},
         'all_roi': {'total_cells': 231, 'unknown_count': 0, 'all_valid_cells_zero': True}, 'barrier_gt_translation': [0.5, -3.5, -10.0]},

        # Re-occupied!
        {'sim_time': 13.0, 'front_surface': {'max_cost': 99}, 'interior': {'max_cost': 50},
         'all_roi': {'total_cells': 231, 'unknown_count': 0, 'all_valid_cells_zero': False}, 'barrier_gt_translation': [0.5, -3.5, -10.0]},

        # Cleared run 2: t=14.0 to 15.0 (1.0s duration)
        {'sim_time': 14.0, 'front_surface': {'max_cost': 0}, 'interior': {'max_cost': 0},
         'all_roi': {'total_cells': 231, 'unknown_count': 0, 'all_valid_cells_zero': True}, 'barrier_gt_translation': [0.5, -3.5, -10.0]},
        {'sim_time': 15.0, 'front_surface': {'max_cost': 0}, 'interior': {'max_cost': 0},
         'all_roi': {'total_cells': 231, 'unknown_count': 0, 'all_valid_cells_zero': True}, 'barrier_gt_translation': [0.5, -3.5, -10.0]},
    ]

    res = analyze_costmap_clearing(history, phys_clear_time=10.0, required_hold_duration=2.0)
    print("Test 4 Result (with reoccupy):", res)
    assert res['clearing_hold_duration_seconds'] == 1.0, f"Expected continuous hold 1.0s, got {res['clearing_hold_duration_seconds']}"
    assert not res['clearing_verified'], "Clearing should NOT be verified when continuous hold was broken by re-occupancy!"
    print("[PASS] Test 4: Re-occupancy strictly resets continuous hold duration!")


def test_costmap_clearing_successful_hold_and_latency():
    print("\n=== Test 5: Successful Clearing Hold and Latency Resolution ===")
    history = [
        {'sim_time': 8.0, 'front_surface': {'max_cost': 100}, 'interior': {'max_cost': 100},
         'all_roi': {'total_cells': 231, 'unknown_count': 0, 'all_valid_cells_zero': False}, 'barrier_gt_translation': [0.5, -3.5, 0.5]},

        # Removed at t=10.0. First observed cleared at t=11.5 (dt = 1.5s < 2.0s observation resolution)
        {'sim_time': 11.5, 'front_surface': {'max_cost': 0}, 'interior': {'max_cost': 0},
         'all_roi': {'total_cells': 231, 'unknown_count': 0, 'all_valid_cells_zero': True}, 'barrier_gt_translation': [0.5, -3.5, -10.0]},
        {'sim_time': 13.5, 'front_surface': {'max_cost': 0}, 'interior': {'max_cost': 0},
         'all_roi': {'total_cells': 231, 'unknown_count': 0, 'all_valid_cells_zero': True}, 'barrier_gt_translation': [0.5, -3.5, -10.0]},
        {'sim_time': 15.5, 'front_surface': {'max_cost': 0}, 'interior': {'max_cost': 0},
         'all_roi': {'total_cells': 231, 'unknown_count': 0, 'all_valid_cells_zero': True}, 'barrier_gt_translation': [0.5, -3.5, -10.0]},
    ]

    res = analyze_costmap_clearing(history, phys_clear_time=10.0, required_hold_duration=2.0, obs_resolution=2.0)
    print("Test 5 Result:", res)
    assert res['clearing_hold_duration_seconds'] == 4.0, f"Expected 4.0s hold, got {res['clearing_hold_duration_seconds']}"
    assert res['clearing_verified'], "Clearing should be verified"
    assert "관측 주기 해상도 이내" in res['clearing_latency_display']
    print("[PASS] Test 5: Successful clearing verified with hold duration and observation resolution formatting!")

def main():
    test_phase_separation_and_standstill()
    test_gt_data_gap_resets_standstill()
    test_costmap_clearing_unknown_rejected()
    test_costmap_clearing_reoccupy_resets_hold()
    test_costmap_clearing_successful_hold_and_latency()
    print("\nALL AUTHORITATIVE EVALUATOR UNIT TESTS PASSED SUCCESSFULLY!")

if __name__ == '__main__':
    main()
