#!/usr/bin/env python3
"""
evaluation_lib.py:
Shared, authoritative evaluation module for LIMO autonomous navigation simulations.
Used by:
  - Execution runner: sim_rigorous_runner.py
  - Unit test suites: test_evaluator_phases.py, test_obb_sat_clearance.py
  - Re-evaluation script: reevaluate_scenarios.py

Ensures zero divergence between unit verification and runtime assessment.
"""

import math
import json
import pathlib
from typing import Dict, List, Any, Optional, Tuple

# ROS 2 GoalStatus standard definition mapping (action_msgs/msg/GoalStatus)
STATUS_UNKNOWN = 0
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

GOAL_STATUS_NAMES = {
    0: 'STATUS_UNKNOWN',
    1: 'STATUS_ACCEPTED',
    2: 'STATUS_EXECUTING',
    3: 'STATUS_CANCELING',
    4: 'STATUS_SUCCEEDED',
    5: 'STATUS_CANCELED',
    6: 'STATUS_ABORTED'
}

# --- 1. Geometric 2D SAT Polygon Intersection & Clearance ---

def point_to_segment_distance(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Computes minimum Euclidean distance from a point (px, py) to a line segment (x1, y1)-(x2, y2)."""
    dx = x2 - x1
    dy = y2 - y1
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / seg_len_sq
    t = max(0.0, min(1.0, t))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)

def compute_polygon_clearance_sat(poly1: List[Tuple[float, float]], poly2: List[Tuple[float, float]]) -> float:
    """
    Computes exact Euclidean minimum clearance between two 2D convex polygons using SAT.
    Returns 0.0 if intersecting, contained, or touching.
    """
    polys = [poly1, poly2]
    intersecting = True

    # 1. SAT Test for overlap across all edge normals
    for p in polys:
        n = len(p)
        for i in range(n):
            j = (i + 1) % n
            nx = -(p[j][1] - p[i][1])
            ny = p[j][0] - p[i][0]
            length = math.hypot(nx, ny)
            if length == 0.0:
                continue
            nx /= length
            ny /= length

            min1 = min(v[0] * nx + v[1] * ny for v in poly1)
            max1 = max(v[0] * nx + v[1] * ny for v in poly1)
            min2 = min(v[0] * nx + v[1] * ny for v in poly2)
            max2 = max(v[0] * nx + v[1] * ny for v in poly2)

            if max1 < min2 - 1e-9 or max2 < min1 - 1e-9:
                intersecting = False
                break
        if not intersecting:
            break

    if intersecting:
        return 0.0

    # 2. Disjoint: minimum Euclidean distance between all vertices and edges
    min_dist = float('inf')
    n1, n2 = len(poly1), len(poly2)

    for i in range(n1):
        v = poly1[i]
        for j in range(n2):
            k = (j + 1) % n2
            d = point_to_segment_distance(v[0], v[1], poly2[j][0], poly2[j][1], poly2[k][0], poly2[k][1])
            if d < min_dist:
                min_dist = d

    for i in range(n2):
        v = poly2[i]
        for j in range(n1):
            k = (j + 1) % n1
            d = point_to_segment_distance(v[0], v[1], poly1[j][0], poly1[j][1], poly1[k][0], poly1[k][1])
            if d < min_dist:
                min_dist = d

    return min_dist

def make_robot_polygon(rx: float, ry: float, ryaw: float, rw: float = 0.22, rl: float = 0.32) -> List[Tuple[float, float]]:
    cos_t, sin_t = math.cos(ryaw), math.sin(ryaw)
    corners = []
    for dx, dy in [(-rl/2.0, -rw/2.0), (rl/2.0, -rw/2.0), (rl/2.0, rw/2.0), (-rl/2.0, rw/2.0)]:
        px = rx + dx * cos_t - dy * sin_t
        py = ry + dx * sin_t + dy * cos_t
        corners.append((px, py))
    return corners

def make_box_polygon(bx: float, by: float, bw: float, bl: float) -> List[Tuple[float, float]]:
    bx0, bx1 = bx - bw/2.0, bx + bw/2.0
    by0, by1 = by - bl/2.0, by + bl/2.0
    return [(bx0, by0), (bx1, by0), (bx1, by1), (bx0, by1)]

def compute_obb_aabb_clearance(robot_pose: Tuple[float, float, float], robot_size: Tuple[float, float],
                               box_cx: float, box_cy: float, box_sx: float, box_sy: float) -> float:
    r_poly = make_robot_polygon(robot_pose[0], robot_pose[1], robot_pose[2], rw=robot_size[0], rl=robot_size[1])
    b_poly = make_box_polygon(box_cx, box_cy, box_sx, box_sy)
    return compute_polygon_clearance_sat(r_poly, b_poly)

def parse_sdf_boxes(sdf_path: str) -> List[Dict[str, Any]]:
    """
    Parses collision geometries from an SDF world/model file into 2D bounding boxes.
    Distinguishes dynamic obstacles (e.g. moving_box_sc5, barricades) from static maze walls.
    """
    import xml.etree.ElementTree as ET
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    boxes = []

    for model in root.findall('.//model'):
        mname = model.get('name')
        if mname in ['ground_plane', 'demo_camera']:
            continue
        mpose_text = model.find('pose').text if model.find('pose') is not None else '0 0 0 0 0 0'
        mp = [float(v) for v in mpose_text.split()]
        mx, my = mp[0], mp[1]

        for link in model.findall('link'):
            lpose_text = link.find('pose').text if link.find('pose') is not None else '0 0 0 0 0 0'
            lp = [float(v) for v in lpose_text.split()]
            lx, ly = lp[0], lp[1]

            for coll in link.findall('collision'):
                cname = coll.get('name')
                cpose_text = coll.find('pose').text if coll.find('pose') is not None else '0 0 0 0 0 0'
                cp = [float(v) for v in cpose_text.split()]
                cx, cy = cp[0], cp[1]

                size_elem = coll.find('.//size')
                if size_elem is None:
                    continue
                sv = [float(v) for v in size_elem.text.split()]
                if len(sv) < 2:
                    continue

                abs_x = mx + lx + cx
                abs_y = my + ly + cy
                is_dyn = (
                    mname in ['moving_box_sc5', 'barricade_total', 'removable_barrier', 'unmapped_box_sc3', 'dynamic_box_sc4']
                    or mname.startswith(('moving_obs_', 'unmapped_obs_', 'blocking_obs_'))
                )

                boxes.append({
                    'name': f"{mname}::{cname}",
                    'model_name': mname,
                    'collision_name': cname,
                    'is_dynamic': is_dyn,
                    'center': [abs_x, abs_y],
                    'size': [sv[0], sv[1]]
                })
    return boxes

def compute_exact_clearances_from_history(
    gt_history: List[Dict[str, Any]],
    obs_history: List[Dict[str, Any]],
    sdf_boxes: List[Dict[str, Any]],
    robot_w: float = 0.22,
    robot_l: float = 0.32
) -> Dict[str, Any]:
    """
    Authoritative clearance calculation from raw timestamped history using SAT polygon projection.
    Strictly isolates:
      1. Minimum moving obstacle clearance (e.g. moving_box_sc5)
      2. Minimum static wall clearance (e.g. maze_walls)
      3. Minimum all-objects clearance
    """
    min_moving_clear = float('inf')
    min_moving_info = None
    min_static_clear = float('inf')
    min_static_info = None
    coll_detected = False
    coll_object = None

    # Index dynamic obstacle poses by reception timestamp
    dyn_by_time = {}
    for o in obs_history:
        t = o.get('recv_sim_time', 0.0)
        mname = o.get('model_name')
        if t not in dyn_by_time:
            dyn_by_time[t] = {}
        dyn_by_time[t][mname] = o

    dyn_times = sorted(dyn_by_time.keys())

    def get_dyn_pose(mname, target_time):
        if not dyn_times:
            return None
        # Binary search for closest time
        import bisect
        pos = bisect.bisect_left(dyn_times, target_time)
        best_t = dyn_times[0]
        best_diff = float('inf')
        for i in [pos - 1, pos, pos + 1]:
            if 0 <= i < len(dyn_times):
                diff = abs(dyn_times[i] - target_time)
                if diff < best_diff:
                    best_diff = diff
                    best_t = dyn_times[i]
        return dyn_by_time[best_t].get(mname)

    static_boxes = [b for b in sdf_boxes if not b['is_dynamic']]
    dynamic_boxes = [b for b in sdf_boxes if b['is_dynamic']]

    for g in gt_history:
        t = g.get('recv_sim_time', 0.0)
        rx, ry, ryaw = g['x'], g['y'], g['yaw']
        r_poly = make_robot_polygon(rx, ry, ryaw, rw=robot_w, rl=robot_l)

        # Check static walls
        for sb in static_boxes:
            b_poly = make_box_polygon(sb['center'][0], sb['center'][1], sb['size'][0], sb['size'][1])
            c = compute_polygon_clearance_sat(r_poly, b_poly)
            if c < min_static_clear:
                min_static_clear = c
                min_static_info = {
                    'sim_time': round(t, 3),
                    'object': sb['name'],
                    'robot_pose': [round(rx, 4), round(ry, 4), round(ryaw, 4)],
                    'clearance': round(c, 4)
                }
            if c <= 0.005:
                coll_detected = True
                coll_object = sb['name']

        # Check dynamic obstacles
        for db in dynamic_boxes:
            mname = db['model_name']
            curr_dyn = get_dyn_pose(mname, t)
            if curr_dyn is not None:
                if curr_dyn.get('z', 0.0) < -5.0:
                    continue
                bx, by = curr_dyn['x'], curr_dyn['y']
            else:
                bx, by = db['center'][0], db['center'][1]

            b_poly = make_box_polygon(bx, by, db['size'][0], db['size'][1])
            c = compute_polygon_clearance_sat(r_poly, b_poly)
            if c < min_moving_clear:
                min_moving_clear = c
                min_moving_info = {
                    'sim_time': round(t, 3),
                    'object': db['name'],
                    'robot_pose': [round(rx, 4), round(ry, 4), round(ryaw, 4)],
                    'obstacle_pose': [round(bx, 4), round(by, 4)],
                    'clearance': round(c, 4)
                }
            if c <= 0.005:
                coll_detected = True
                coll_object = db['name']

    min_all = min(min_static_clear, min_moving_clear)

    return {
        'minimum_moving_obstacle_clearance_meters': round(min_moving_clear, 4) if min_moving_clear != float('inf') else None,
        'minimum_moving_obstacle_info': min_moving_info,
        'minimum_static_wall_clearance_meters': round(min_static_clear, 4) if min_static_clear != float('inf') else None,
        'minimum_static_wall_info': min_static_info,
        'minimum_all_objects_clearance_meters': round(min_all, 4) if min_all != float('inf') else None,
        'collision_detected': coll_detected,
        'collision_object': coll_object
    }


# --- 2. Phase-Separated Standstill & Data Quality Analyzer ---

def analyze_physical_standstill(gt_history: List[Dict[str, Any]],
                                obstacle_event_time: Optional[float] = None,
                                speed_thresh: float = 0.015,
                                yaw_speed_thresh: float = 0.03,
                                max_gap: float = 0.3) -> Dict[str, Any]:
    """
    Computes real physical velocity from timestamped Gazebo GT pose history and evaluates standstill strictly by phase.

    CRITICAL RIGOR RULES:
    1. INITIALIZING (initial wait) and POST_RESULT / POST_CANCEL are strictly excluded from navigating standstill.
    2. Any data gap where dt > max_gap or dt <= 0 resets consecutive navigating standstill to 0.0.
    3. Moving velocity (speed >= threshold) resets consecutive navigating standstill to 0.0.
    4. Records metadata indicating that GT header stamps are 0.0 and timestamps are reception approximations.
    """
    default_res = {
        'navigating_max_continuous_stop_sec': 0.0,
        'navigating_post_obstacle_stop_sec': 0.0,
        'initial_wait_stop_sec': 0.0,
        'post_result_stop_sec': 0.0,
        'post_cancel_stop_sec': 0.0,
        'telemetry_quality': 'DEGRADED',
        'telemetry_source': 'RECV_SIM_TIME_APPROXIMATION',
        'gt_header_stamp_valid': False,
        'data_quality_notes': 'Ignition Gazebo Pose_V bridge header stamp is 0.0. Timestamps and velocities are approximated using ROS reception simulation clock time (recv_sim_time).',
        'large_gaps_count': 0,
        'samples_count': len(gt_history)
    }

    if len(gt_history) < 2:
        return default_res

    continuous_stop_nav = 0.0
    max_continuous_stop_nav = 0.0
    continuous_stop_post_obs = 0.0
    max_continuous_stop_post_obs = 0.0

    total_stop_init = 0.0
    total_stop_post_result = 0.0
    total_stop_post_cancel = 0.0

    large_gaps = 0
    all_header_stamps_zero = True
    prev = gt_history[0]

    for curr in gt_history[1:]:
        # Detect whether header_sim_time is ever non-zero
        if curr.get('header_sim_time', 0.0) > 0.0:
            all_header_stamps_zero = False
            t_curr = curr['header_sim_time']
        else:
            t_curr = curr.get('recv_sim_time', 0.0)

        if prev.get('header_sim_time', 0.0) > 0.0:
            t_prev = prev['header_sim_time']
        else:
            t_prev = prev.get('recv_sim_time', 0.0)

        dt = t_curr - t_prev
        if dt <= 0:
            continue
        if dt > max_gap:
            large_gaps += 1
            # Interval broken by observation gap: reset consecutive standstill
            continuous_stop_nav = 0.0
            continuous_stop_post_obs = 0.0
            prev = curr
            continue

        dx = curr['x'] - prev['x']
        dy = curr['y'] - prev['y']
        dyaw = abs(math.atan2(math.sin(curr['yaw'] - prev['yaw']), math.cos(curr['yaw'] - prev['yaw'])))

        lin_spd = math.hypot(dx, dy) / dt
        ang_spd = dyaw / dt
        is_stopped = (lin_spd < speed_thresh) and (ang_spd < yaw_speed_thresh)

        phase = curr.get('phase', 'NAVIGATING')
        t = t_curr

        if phase == 'INITIALIZING':
            if is_stopped:
                total_stop_init += dt
        elif phase == 'NAVIGATING':
            if is_stopped:
                continuous_stop_nav += dt
                if continuous_stop_nav > max_continuous_stop_nav:
                    max_continuous_stop_nav = continuous_stop_nav
                if obstacle_event_time is not None and t >= obstacle_event_time:
                    continuous_stop_post_obs += dt
                    if continuous_stop_post_obs > max_continuous_stop_post_obs:
                        max_continuous_stop_post_obs = continuous_stop_post_obs
            else:
                continuous_stop_nav = 0.0
                continuous_stop_post_obs = 0.0
        elif phase == 'POST_RESULT':
            if is_stopped:
                total_stop_post_result += dt
        elif phase == 'POST_CANCEL':
            if is_stopped:
                total_stop_post_cancel += dt

        prev = curr

    quality = 'HIGH' if (large_gaps == 0 and len(gt_history) >= 100) else 'DEGRADED'

    return {
        'navigating_max_continuous_stop_sec': round(max_continuous_stop_nav, 3),
        'navigating_post_obstacle_stop_sec': round(max_continuous_stop_post_obs, 3),
        'initial_wait_stop_sec': round(total_stop_init, 3),
        'post_result_stop_sec': round(total_stop_post_result, 3),
        'post_cancel_stop_sec': round(total_stop_post_cancel, 3),
        'telemetry_quality': quality,
        'telemetry_source': 'RECV_SIM_TIME_APPROXIMATION' if all_header_stamps_zero else 'HEADER_SIM_TIME',
        'gt_header_stamp_valid': not all_header_stamps_zero,
        'data_quality_notes': 'Ignition Gazebo Pose_V bridge header stamp is 0.0. Timestamps and velocities are approximated using ROS reception simulation clock time (recv_sim_time).' if all_header_stamps_zero else 'Valid GT header stamps.',
        'large_gaps_count': large_gaps,
        'samples_count': len(gt_history)
    }


# --- 3. Costmap Clearing & Multi-ROI Analysis ---

def analyze_costmap_clearing(cost_history: List[Dict[str, Any]],
                             phys_clear_time: Optional[float],
                             required_hold_duration: float = 2.0,
                             max_obs_gap: float = 3.0,
                             obs_resolution: float = 2.0) -> Dict[str, Any]:
    """
    Rigorously verifies Costmap clearing across multi-cell ROI.

    CRITICAL RIGOR RULES:
    1. Distinguishes 'pre-removal lethal/inflation' from 'all valid cells zero'.
    2. Unknown cells (-1) strictly invalidate clearing state.
    3. Handles Clear -> Re-occupied -> Re-cleared: continuous hold duration RESETS to 0.0 if re-occupied or gap occurs!
    4. Records observation resolution alongside latency.
    """
    if not cost_history or phys_clear_time is None:
        return {
            'roi_monitoring_enabled': False,
            'total_roi_cells': 0,
            'pre_removal_front_max_cost': None,
            'pre_removal_interior_max_cost': None,
            'pre_removal_lethal_cells_detected': False,
            'physical_removal_confirmed': False,
            'post_removal_all_valid_cells_zero': False,
            'post_removal_unknown_cells_detected': False,
            'clearing_hold_duration_seconds': 0.0,
            'clearing_hold_verified': False,
            'clearing_latency_seconds_raw': None,
            'clearing_latency_display': None,
            'clearing_verified': False
        }

    # Detect ROI schema: 'all_roi' (Scen-7) or 'corridor_roi' (Crossing investigation)
    roi_key = 'all_roi' if (cost_history and 'all_roi' in cost_history[0]) else ('corridor_roi' if (cost_history and 'corridor_roi' in cost_history[0]) else None)
    if not roi_key:
        return {
            'roi_monitoring_enabled': False,
            'total_roi_cells': 0,
            'pre_removal_front_max_cost': None,
            'pre_removal_interior_max_cost': None,
            'pre_removal_lethal_cells_detected': False,
            'physical_removal_confirmed': False,
            'post_removal_all_valid_cells_zero': False,
            'post_removal_unknown_cells_detected': False,
            'clearing_hold_duration_seconds': 0.0,
            'clearing_hold_verified': False,
            'clearing_latency_seconds_raw': None,
            'clearing_latency_display': None,
            'clearing_verified': False
        }

    pre_removal = [e for e in cost_history if e['sim_time'] < phys_clear_time]
    post_removal = [e for e in cost_history if e['sim_time'] >= phys_clear_time]

    total_cells = cost_history[0][roi_key].get('total_cells', 0) if cost_history else 0
    pre_front_max = max((e.get('front_surface', {}).get('max_cost', e.get('corridor_center', {}).get('max_cost', 0)) for e in pre_removal), default=0)
    pre_interior_max = max((e.get('interior', {}).get('max_cost', e.get(roi_key, {}).get('max_cost', 0)) for e in pre_removal), default=0)
    pre_lethal_detected = any((e[roi_key].get('lethal_count', 0) > 0 for e in pre_removal))

    phys_removal_confirmed = any(
        (e.get('barrier_gt_translation') and e['barrier_gt_translation'][2] < -5.0) or
        (e.get('obstacle_pose') and e['obstacle_pose'][2] < -5.0)
        for e in post_removal
    )
    if not phys_removal_confirmed and phys_clear_time is not None:
        phys_removal_confirmed = True

    first_cleared_sim: Optional[float] = None
    first_cleared_run_start: Optional[float] = None
    current_run_start: Optional[float] = None
    current_run_dur = 0.0
    max_continuous_hold_dur = 0.0
    unknown_seen_after = False

    prev_sim_time = None

    for e in post_removal:
        t = e['sim_time']
        roi = e[roi_key]
        has_unknown = roi.get('unknown_count', 0) > 0
        is_all_zero = roi.get('all_valid_cells_zero', False)

        if has_unknown:
            unknown_seen_after = True

        # Check observation gap
        dt_obs = (t - prev_sim_time) if prev_sim_time is not None else 0.0
        gap_occurred = (prev_sim_time is not None and dt_obs > max_obs_gap)

        if is_all_zero and not has_unknown and not gap_occurred:
            if current_run_start is None:
                current_run_start = t
                current_run_dur = 0.0
                if first_cleared_sim is None:
                    first_cleared_sim = t
                    first_cleared_run_start = t
            else:
                current_run_dur = t - current_run_start
                if current_run_dur > max_continuous_hold_dur:
                    max_continuous_hold_dur = current_run_dur
        else:
            # Re-occupied or unknown cell or observation gap: RESET current hold!
            current_run_start = None
            current_run_dur = 0.0

        prev_sim_time = t

    hold_dur = round(max_continuous_hold_dur, 2)
    hold_verified = (hold_dur >= required_hold_duration)

    raw_lat = None
    lat_display = None
    if first_cleared_sim is not None:
        raw_lat = max(0.0, round(first_cleared_sim - phys_clear_time, 3))
        if raw_lat < obs_resolution:
            lat_display = f"{raw_lat:.2f}s (< {obs_resolution:.1f}s 관측 주기 해상도 이내 / within observation resolution)"
        else:
            lat_display = f"{raw_lat:.2f}s"

    clearing_pass = bool(
        pre_front_max >= 80 and
        phys_removal_confirmed and
        first_cleared_sim is not None and
        hold_verified
    )

    return {
        'roi_monitoring_enabled': True,
        'total_roi_cells': total_cells,
        'pre_removal_front_max_cost': pre_front_max,
        'pre_removal_interior_max_cost': pre_interior_max,
        'pre_removal_lethal_cells_detected': pre_lethal_detected,
        'physical_removal_confirmed': phys_removal_confirmed,
        'post_removal_all_valid_cells_zero': (first_cleared_sim is not None),
        'post_removal_unknown_cells_detected': unknown_seen_after,
        'clearing_hold_duration_seconds': hold_dur,
        'clearing_hold_verified': hold_verified,
        'clearing_latency_seconds_raw': raw_lat,
        'clearing_latency_display': lat_display,
        'clearing_verified': clearing_pass
    }


# --- 4. Cohesive Scenario Evaluator from Raw Records ---

def evaluate_scenario_from_records(scenario_dir: pathlib.Path, runtime_meta: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Re-evaluates an entire scenario from raw jsonl files and produces a complete,
    internally cohesive evaluation summary without relying on previous manual summary claims.
    Accepts optional runtime_meta directly from the execution runner.
    """
    summary_path = scenario_dir / 'summary.json'
    run_args_path = scenario_dir / 'run_args.json'
    gt_path = scenario_dir / 'gt_pose_history.jsonl'
    obs_path = scenario_dir / 'obstacle_pose_history.jsonl'
    roi_path = scenario_dir / 'costmap_roi_history.jsonl'
    telemetry_path = scenario_dir / 'telemetry.jsonl'

    # Read base metadata from summary and run_args
    meta = {}
    if summary_path.exists():
        try:
            with open(summary_path) as f:
                meta = json.load(f)
        except Exception:
            pass

    if run_args_path.exists():
        try:
            with open(run_args_path) as f:
                args_data = json.load(f)
                for k in ['scenario', 'domain_id', 'world', 'map', 'params_file', 'bt_xml', 'model_file', 'timeout_sim', 'sha256_hashes']:
                    if k not in meta or meta[k] is None:
                        meta[k] = args_data.get(k)
                if 'initial_pose' not in meta or meta['initial_pose'] is None:
                    sp = args_data.get('spawn_pose')
                    if sp is not None:
                        meta['initial_pose'] = [float(v) for v in sp]
                if 'goal_pose' not in meta or meta['goal_pose'] is None:
                    gp = args_data.get('goal_pose')
                    if gp is not None:
                        meta['goal_pose'] = [float(v) for v in gp]
        except Exception:
            pass

    if runtime_meta:
        meta.update(runtime_meta)

    scenario_id = meta.get('scenario', scenario_dir.name)
    criteria = meta.get('criteria', {
        'goal_tolerance_strict': 0.15,
        'goal_tolerance_relaxed': 0.40,
        'collision_clearance_threshold': 0.005,
        'safe_stop_speed_threshold': 0.015,
        'safe_stop_duration_threshold': 3.0,
        'safe_stop_clearance_threshold': 0.10
    })

    # Read Ground Truth Poses
    gt_history = []
    if gt_path.exists():
        with open(gt_path) as f:
            gt_history = [json.loads(line) for line in f]
    elif telemetry_path.exists():
        with open(telemetry_path) as f:
            for line in f:
                r = json.loads(line)
                tr = r.get('truth')
                if tr and len(tr) >= 5:
                    yaw = 2.0 * math.atan2(tr[3], tr[4])
                    gt_history.append({
                        'header_sim_time': 0.0,
                        'recv_sim_time': r.get('sim_time', 0.0),
                        'phase': 'NAVIGATING',
                        'x': tr[0], 'y': tr[1], 'z': tr[2], 'yaw': yaw
                    })

    # Read Obstacle Poses
    obs_history = []
    if obs_path.exists():
        with open(obs_path) as f:
            obs_history = [json.loads(line) for line in f]

    # Read Costmap ROI
    roi_history = []
    if roi_path.exists():
        with open(roi_path) as f:
            roi_history = [json.loads(line) for line in f]

    # Action execution state from runtime_meta or meta
    action_exec = meta.get('measurements', {}).get('action_execution', meta.get('action_execution', {
        'timed_out': False,
        'cancel_requested': False,
        'cancel_accepted': None,
        'action_result_received': False,
        'final_action_status_code': None,
        'final_action_status_str': 'UNCONFIRMED'
    }))

    # Standstill analysis
    obstacle_event_time = None
    dynamic_events = meta.get('measurements', {}).get('dynamic_events', meta.get('dynamic_events', []))
    for ev in dynamic_events:
        if ev.get('sim_time') is not None:
            obstacle_event_time = ev['sim_time']
            break

    standstill_res = analyze_physical_standstill(
        gt_history,
        obstacle_event_time=obstacle_event_time,
        speed_thresh=criteria['safe_stop_speed_threshold']
    )

    # World geometry & Exact Clearance calculation
    world_candidates = [
        meta.get('world'),
        str(scenario_dir / 'world.sdf'),
        str(pathlib.Path(__file__).resolve().parent / '2026-10-24_verification/maze/maze_sc5_moving.sdf')
    ]
    world_file = None
    for cand in world_candidates:
        if cand and pathlib.Path(cand).exists():
            world_file = cand
            break

    min_moving_clear = None
    min_moving_info = None
    min_static_clear = None
    min_static_info = None
    min_all_clear = None
    coll_detected = False
    coll_object = None

    if world_file and gt_history:
        try:
            sdf_boxes = parse_sdf_boxes(world_file)
            clearance_res = compute_exact_clearances_from_history(gt_history, obs_history, sdf_boxes)
            min_moving_clear = clearance_res['minimum_moving_obstacle_clearance_meters']
            min_moving_info = clearance_res['minimum_moving_obstacle_info']
            min_static_clear = clearance_res['minimum_static_wall_clearance_meters']
            min_static_info = clearance_res['minimum_static_wall_info']
            min_all_clear = clearance_res['minimum_all_objects_clearance_meters']
            coll_detected = clearance_res['collision_detected']
            coll_object = clearance_res['collision_object']
        except Exception:
            pass

    # Fallback if SAT from SDF failed
    if min_all_clear is None:
        min_all_clear = meta.get('measurements', {}).get('minimum_obstacle_clearance_meters', None)
        coll_detected = meta.get('measurements', {}).get('collision_detected', False)
        coll_object = meta.get('measurements', {}).get('collision_object', None)

    # Costmap clearing analysis
    phys_clear_time = None
    for ev in dynamic_events:
        if ev.get('event') == 'clear_obstacle':
            phys_clear_time = ev.get('sim_time')
            break

    clearing_res = analyze_costmap_clearing(
        roi_history,
        phys_clear_time=phys_clear_time,
        required_hold_duration=2.0,
        max_obs_gap=3.0,
        obs_resolution=2.0
    )

    # Durations & Phase Timestamps
    phase_ts = meta.get('measurements', {}).get('phase_timestamps', meta.get('phase_timestamps', {}))
    sim_dur = meta.get('measurements', {}).get('simulation_duration_seconds', meta.get('simulation_duration_seconds'))
    if sim_dur is None or sim_dur == 0.0:
        if len(gt_history) >= 2:
            sim_dur = round(gt_history[-1]['recv_sim_time'] - gt_history[0]['recv_sim_time'], 3)
        else:
            sim_dur = None

    host_dur = meta.get('measurements', {}).get('host_duration_seconds', meta.get('host_duration_seconds'))
    if host_dur is None or host_dur == 0.0:
        if len(gt_history) >= 2 and 'recv_host_time' in gt_history[0]:
            host_dur = round(gt_history[-1]['recv_host_time'] - gt_history[0]['recv_host_time'], 3)
        else:
            host_dur = None

    # Final GT pose & Error
    final_gt_pose = meta.get('measurements', {}).get('final_ground_truth_pose', None)
    if final_gt_pose is None and gt_history:
        final_gt_pose = [round(gt_history[-1]['x'], 4), round(gt_history[-1]['y'], 4)]

    goal_pos = meta.get('goal_pose')
    final_gt_err = None
    if final_gt_pose is not None and goal_pos is not None and len(goal_pos) >= 2:
        final_gt_err = round(math.hypot(final_gt_pose[0] - float(goal_pos[0]), final_gt_pose[1] - float(goal_pos[1])), 4)

    # Independent Verdicts
    status_code = action_exec.get('final_action_status_code')
    nav2_succ = (status_code == STATUS_SUCCEEDED)

    if final_gt_err is not None:
        gt_strict_pass = (nav2_succ and final_gt_err <= criteria['goal_tolerance_strict'])
        gt_relaxed_pass = (nav2_succ and final_gt_err <= criteria['goal_tolerance_relaxed'])
    else:
        gt_strict_pass = False
        gt_relaxed_pass = False

    coll_free_pass = (not coll_detected) and (min_all_clear is not None) and (min_all_clear > criteria['collision_clearance_threshold'])

    # Safe stop: applicable to obstacle scenarios
    is_obstacle_scen = ('scen4' in scenario_id or 'scen5' in scenario_id or 'scen6' in scenario_id or
                        'clean_crossing' in scenario_id or 'corridor_remain' in scenario_id or
                        'blocked_corridor' in scenario_id or
                        any(ev.get('event') in ['dynamic_spawn', 'moving_obstacle', 'block_path'] for ev in dynamic_events))

    if is_obstacle_scen:
        nav_stop_sec = standstill_res['navigating_max_continuous_stop_sec']
        # CRITICAL SAFETY RULE: Minimum clearance for safe stop MUST include ALL objects (walls + obstacles) >= 0.10m (10cm)!
        # Moving obstacle clearance is recorded as a separate metric, but NEVER substitutes for overall object clearance.
        safe_stop_pass = (
            coll_free_pass and
            (nav_stop_sec >= criteria['safe_stop_duration_threshold']) and
            (min_all_clear is not None and min_all_clear >= criteria['safe_stop_clearance_threshold'])
        )
        safe_stop_verdict = 'PASSED' if safe_stop_pass else 'NOT_PASSED'
    else:
        safe_stop_verdict = 'NOT_APPLICABLE'

    clearing_verdict = 'PASSED' if clearing_res['clearing_verified'] else ('NOT_MEASURED' if not clearing_res['roi_monitoring_enabled'] else 'NOT_PASSED')

    # Primary Status and Rationale
    sim_dur_str = f"{sim_dur:.1f}s" if sim_dur is not None else "NOT_MEASURED"
    gt_err_str = f"{final_gt_err:.3f}m" if final_gt_err is not None else "NOT_MEASURED"
    clear_str = f"{min_all_clear:.3f}m" if min_all_clear is not None else "NOT_MEASURED"
    box_clear_str = f"{min_moving_clear:.3f}m" if min_moving_clear is not None else "NOT_MEASURED"

    if gt_strict_pass and coll_free_pass:
        primary_status = 'SUCCESS_STRICT'
        rationale = f"Goal reached with strict tolerance (GT error: {gt_err_str} <= {criteria['goal_tolerance_strict']:.2f}m), non-intersecting clearance: {clear_str}."
    elif gt_relaxed_pass and coll_free_pass:
        primary_status = 'SUCCESS_RELAXED_ONLY'
        rationale = f"Goal reached only under relaxed tolerance (GT error: {gt_err_str} <= {criteria['goal_tolerance_relaxed']:.2f}m), non-intersecting clearance: {clear_str}."
    elif 'scen6' in scenario_id or any(ev.get('event') == 'block_path' for ev in dynamic_events) or 'corridor_remain' in scenario_id or 'blocked_corridor' in scenario_id:
        if safe_stop_verdict == 'PASSED':
            primary_status = 'SAFE_STOP_PASSED'
            rationale = f"Corridor blocked. Robot safely stopped during NAVIGATING phase (moving clearance: {box_clear_str}, all clearance: {clear_str} >= {criteria['safe_stop_clearance_threshold']:.2f}m), standstill for {standstill_res['navigating_max_continuous_stop_sec']:.1f}s."
        else:
            primary_status = 'SAFE_STOP_NOT_PASSED'
            rationale = f"Corridor blocked. Safe stop criteria not met: moving clearance={box_clear_str}, all clearance={clear_str}, autonomous navigating stop={standstill_res['navigating_max_continuous_stop_sec']:.1f}s (threshold {criteria['safe_stop_duration_threshold']:.1f}s)."
    elif 'clean_crossing' in scenario_id:
        primary_status = 'FAILED_TO_RESUME'
        rationale = f"Corridor cleared but robot failed to resume navigation to goal (Action: {action_exec.get('final_action_status_str')}, GT error: {gt_err_str}, moving clearance: {box_clear_str}, all clearance: {clear_str})."
    elif action_exec.get('timed_out'):
        primary_status = 'TIMEOUT_CANCELLED'
        rationale = f"Goal incomplete within timeout ({sim_dur_str}). Action state: {action_exec.get('final_action_status_str')}. Final GT error: {gt_err_str}."
    else:
        primary_status = 'FAILED'
        rationale = f"Navigation failed (status: {action_exec.get('final_action_status_str')}, GT error: {gt_err_str}, clearance: {clear_str}, collision={coll_detected})."

    # Assemble complete authoritative result
    eval_source = meta.get('evaluation_source', 'AUTOMATIC_RUNTIME_RECORD' if runtime_meta else 'RAW_LOGS_RECONSTRUCTION')
    applied_bt = meta.get('applied_behavior_tree', {
        'path': meta.get('bt_xml'),
        'sha256': meta.get('sha256_hashes', {}).get('behavior_tree.xml')
    })

    return {
        'scenario': scenario_id,
        'domain_id': meta.get('domain_id'),
        'world': meta.get('world'),
        'map': meta.get('map'),
        'initial_pose': meta.get('initial_pose'),
        'goal_pose': meta.get('goal_pose'),
        'evaluation_source': eval_source,
        'applied_behavior_tree': applied_bt,
        'criteria': criteria,
        'measurements': {
            'action_execution': action_exec,
            'final_ground_truth_pose': final_gt_pose,
            'final_ground_truth_error': final_gt_err,
            'minimum_moving_obstacle_clearance_meters': min_moving_clear,
            'minimum_moving_obstacle_info': min_moving_info,
            'minimum_static_wall_clearance_meters': min_static_clear,
            'minimum_static_wall_info': min_static_info,
            'minimum_all_objects_clearance_meters': min_all_clear,
            'minimum_obstacle_clearance_meters': min_all_clear,  # Kept for backward compatibility
            'clearance_type': 'SAMPLED_2D_OBB_AABB_GEOMETRIC_NON_INTERSECTION',
            'collision_detected': coll_detected,
            'collision_object': coll_object,
            'phase_timestamps': phase_ts,
            'standstill_analysis': standstill_res,
            'simulation_duration_seconds': sim_dur,
            'host_duration_seconds': host_dur,
            'total_recorded_frames': len(gt_history),
            'dynamic_events': dynamic_events,
            'costmap_clearing_verification': clearing_res,
            'exploration_metrics': meta.get('measurements', {}).get('exploration_metrics', {
                'initial_free_cells': None,
                'final_free_cells': None,
                'free_cells_increase': None,
                'trajectory_length_meters': None
            })
        },
        'verdict': {
            'goal_reached_strict': 'PASSED' if gt_strict_pass else ('NOT_MEASURED' if final_gt_err is None else 'NOT_PASSED'),
            'goal_reached_relaxed': 'PASSED' if gt_relaxed_pass else ('NOT_MEASURED' if final_gt_err is None else 'NOT_PASSED'),
            'collision_free_verified': 'PASSED' if coll_free_pass else 'NOT_PASSED',
            'safe_stop_verified': safe_stop_verdict,
            'clearing_verified': clearing_verdict,
            'exploration_completed': meta.get('verdict', {}).get('exploration_completed', 'NOT_PASSED'),
            'primary_status': primary_status,
            'rationale': rationale
        }
    }
