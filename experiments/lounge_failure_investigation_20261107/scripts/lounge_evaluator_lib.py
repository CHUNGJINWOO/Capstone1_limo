#!/usr/bin/env python3
"""
lounge_evaluator_lib.py:
Core evaluation library for lounge simulation failure analysis.
Provides:
- 2D Oriented Bounding Box (OBB) Separating Axis Theorem (SAT) collision & clearance calculation
- 3D underpass / overhead clearance and curb contact checks
- Failure identification (Table Underpass, Skid Entrapment, Glass Wall Collision, Stall, Oscillation)
- Automatic Root Cause Classification Matrix
"""
import math
import numpy as np

class FailureCategory:
    SUCCESS = "SUCCESS"
    TABLE_UNDERPASS_COLLISION = "TABLE_UNDERPASS_COLLISION"
    CHAIR_SKID_ENTRAPMENT = "CHAIR_SKID_ENTRAPMENT"
    GLASS_WALL_COLLISION = "GLASS_WALL_COLLISION"
    NARROW_CORRIDOR_OSCILLATION = "NARROW_CORRIDOR_OSCILLATION"
    PLANNER_PATH_THROUGH_BARRIER = "PLANNER_PATH_THROUGH_BARRIER"
    STALL_UNRESPONSIVE = "STALL_UNRESPONSIVE"

class RootCause:
    NONE = "NONE"
    SENSOR_BLIND_SPOT = "SENSOR_BLIND_SPOT"                  # LiDAR z=0.16m plane cannot see z<0.05m or z>0.21m
    COSTMAP_LAYER_MISSING = "COSTMAP_LAYER_MISSING"          # Absence of 3D VoxelLayer or Pointcloud layer
    FOOTPRINT_INFLATION_DEFECT = "FOOTPRINT_INFLATION_DEFECT"# Circular footprint allows corner collision in tight gaps
    PLANNER_LOGIC_DEFECT = "PLANNER_LOGIC_DEFECT"            # Absence of keepout filter allows path through underpass
    ENV_OPTICAL_MISMATCH = "ENV_OPTICAL_MISMATCH"            # Laser penetrates transparent glass partition
    SAFETY_BEHAVIOR_DEFECT = "SAFETY_BEHAVIOR_DEFECT"        # No hardware bumper / no stall e-stop

class LoungeEvaluator:
    def __init__(self):
        # LIMO Pro physical body specifications (meters)
        self.robot_length = 0.32
        self.robot_width = 0.22
        self.robot_height = 0.20
        self.ground_clearance = 0.025 # 25mm clearance
        self.lidar_height = 0.16      # 160mm beam height

        # Lounge Furniture Geometries in Map/World Frame:
        # Table 1: cx=-2.5, cy=1.8, w=1.40, d=0.80, reinf_z=0.21
        # Table 2: cx=0.5, cy=1.8, w=1.40, d=0.80, reinf_z=0.21
        # Table 3: cx=-2.5, cy=0.0, w=1.40, d=0.80, reinf_z=0.21
        self.tables = [
            {"id": "table_1", "cx": -2.5, "cy": 1.8, "w": 1.40, "d": 0.80, "reinf_z": 0.21, "yaw": 0.0},
            {"id": "table_2", "cx": 0.5, "cy": 1.8, "w": 1.40, "d": 0.80, "reinf_z": 0.21, "yaw": 0.0},
            {"id": "table_3", "cx": -2.5, "cy": 0.0, "w": 1.40, "d": 0.80, "reinf_z": 0.21, "yaw": 0.0},
            {"id": "vip_table", "cx": 1.5, "cy": -3.0, "w": 0.80, "d": 1.40, "reinf_z": 0.21, "yaw": 1.5708},
        ]

        # Sled-base chair target in corridor and tables
        # Chair bottom skid rails: height z=0.035m (higher than robot ground clearance 0.025m)
        self.target_sled_chair = {
            "id": "corridor_sled_chair_target",
            "cx": 4.2, "cy": 2.0, "w": 0.42, "d": 0.42, "skid_z": 0.035, "yaw": 0.0
        }

        # Glass partition: y = -1.5m, x in [-1.3, 5.9], physical collision box
        self.glass_wall = {
            "id": "glass_wall",
            "y": -1.5, "x_min": -1.3, "x_max": 5.9, "z_min": 0.0, "z_max": 1.8, "thickness": 0.02
        }

    def get_robot_obb_corners(self, rx, ry, ryaw):
        """Returns 4 corners of the robot footprint."""
        hl = self.robot_length / 2.0
        hw = self.robot_width / 2.0
        cos_y = math.cos(ryaw)
        sin_y = math.sin(ryaw)

        local_corners = [
            (hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)
        ]
        return [
            (rx + dx * cos_y - dy * sin_y, ry + dx * sin_y + dy * cos_y)
            for dx, dy in local_corners
        ]

    def check_sat_overlap(self, corners_a, corners_b):
        """Separating Axis Theorem for two 2D convex polygons."""
        polys = [corners_a, corners_b]
        for poly in polys:
            for i in range(len(poly)):
                p1 = poly[i]
                p2 = poly[(i + 1) % len(poly)]
                edge = (p2[0] - p1[0], p2[1] - p1[1])
                # Normal vector
                axis = (-edge[1], edge[0])
                length = math.hypot(axis[0], axis[1])
                if length < 1e-6:
                    continue
                axis = (axis[0] / length, axis[1] / length)

                # Project poly a
                proj_a = [p[0] * axis[0] + p[1] * axis[1] for p in corners_a]
                proj_b = [p[0] * axis[0] + p[1] * axis[1] for p in corners_b]

                if max(proj_a) < min(proj_b) or max(proj_b) < min(proj_a):
                    return False # Separating axis found -> No overlap
        return True # Overlap on all axes

    def compute_min_distance(self, rx, ry, corners_box):
        """Approximate minimum distance from robot center to obstacle box edges."""
        min_d = float('inf')
        for i in range(len(corners_box)):
            p1 = corners_box[i]
            p2 = corners_box[(i + 1) % len(corners_box)]
            # Point to segment distance
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-6:
                d = math.hypot(rx - p1[0], ry - p1[1])
            else:
                t = max(0.0, min(1.0, ((rx - p1[0]) * dx + (ry - p1[1]) * dy) / seg_len_sq))
                proj_x = p1[0] + t * dx
                proj_y = p1[1] + t * dy
                d = math.hypot(rx - proj_x, ry - proj_y)
            if d < min_d:
                min_d = d
        return max(0.0, min_d - max(self.robot_length, self.robot_width) / 2.0)

    def check_table_underpass_violation(self, rx, ry, ryaw):
        """
        Determines whether the robot entered under a table.
        Under the table, the robot top (0.20m) interferes with the reinforcement frame (0.21m).
        """
        robot_corners = self.get_robot_obb_corners(rx, ry, ryaw)
        for tbl in self.tables:
            cx, cy, w, d = tbl["cx"], tbl["cy"], tbl["w"], tbl["d"]
            tbl_corners = [
                (cx + w/2, cy + d/2), (cx + w/2, cy - d/2),
                (cx - w/2, cy - d/2), (cx - w/2, cy + d/2)
            ]
            if self.check_sat_overlap(robot_corners, tbl_corners):
                # Check if robot center is within inner underpass zone
                if abs(rx - cx) < (w/2.0 - 0.15) and abs(ry - cy) < (d/2.0 - 0.10):
                    return True, tbl["id"]
        return False, None

    def check_skid_chair_contact(self, rx, ry, ryaw):
        """
        Determines whether the robot is traversing over the sled-base chair's
        near-ground skid frame (height 35mm), causing bottom-out chassis hang.
        """
        robot_corners = self.get_robot_obb_corners(rx, ry, ryaw)
        ch = self.target_sled_chair
        cx, cy, w, d = ch["cx"], ch["cy"], ch["w"], ch["d"]
        chair_corners = [
            (cx + w/2, cy + d/2), (cx + w/2, cy - d/2),
            (cx - w/2, cy - d/2), (cx - w/2, cy + d/2)
        ]
        if self.check_sat_overlap(robot_corners, chair_corners):
            dist = self.compute_min_distance(rx, ry, chair_corners)
            return True, dist
        return False, self.compute_min_distance(rx, ry, chair_corners)

    def check_glass_wall_collision(self, rx, ry, ryaw):
        """
        Determines if the robot collided with the glass wall at y = -1.5m.
        """
        gw = self.glass_wall
        y_wall = gw["y"]
        x_min, x_max = gw["x_min"], gw["x_max"]

        robot_corners = self.get_robot_obb_corners(rx, ry, ryaw)
        # Check if any robot corner crossed or touched the wall line y_wall
        y_coords = [c[1] for c in robot_corners]
        x_coords = [c[0] for c in robot_corners]

        if min(x_coords) <= x_max and max(x_coords) >= x_min:
            # Did the robot straddle or get closer than 0.05m to the glass wall?
            if min(y_coords) <= y_wall <= max(y_coords) or abs(ry - y_wall) < (self.robot_width/2.0 + 0.03):
                return True, abs(ry - y_wall)
        return False, abs(ry - y_wall)

    def classify_failure(self, scenario_id, result_dict):
        """
        Takes raw test telemetry and assigns failure category and root causes.
        """
        collided = result_dict.get("collision_occurred", False)
        stalled = result_dict.get("stall_detected", False)
        underpass = result_dict.get("table_underpass_detected", False)
        glass_coll = result_dict.get("glass_collision_detected", False)
        skid_coll = result_dict.get("skid_contact_detected", False)
        oscillations = result_dict.get("oscillation_count", 0)
        replans = result_dict.get("replan_count", 0)
        path_through_obstacle = result_dict.get("path_through_obstacle", False)

        if underpass:
            return {
                "category": FailureCategory.TABLE_UNDERPASS_COLLISION,
                "primary_cause": RootCause.SENSOR_BLIND_SPOT,
                "secondary_cause": RootCause.PLANNER_LOGIC_DEFECT,
                "mechanism": "2D LiDAR at z=0.16m sees empty space beneath tabletop; planner chose underpass; robot top/crossbar interference occurred."
            }
        elif glass_coll:
            return {
                "category": FailureCategory.GLASS_WALL_COLLISION,
                "primary_cause": RootCause.ENV_OPTICAL_MISMATCH,
                "secondary_cause": RootCause.SENSOR_BLIND_SPOT,
                "mechanism": "Optical transparency allowed laser beams to penetrate glass partition without return; costmap remained free; robot drove straight into physical barrier."
            }
        elif skid_coll and (stalled or collided):
            return {
                "category": FailureCategory.CHAIR_SKID_ENTRAPMENT,
                "primary_cause": RootCause.SENSOR_BLIND_SPOT,
                "secondary_cause": RootCause.COSTMAP_LAYER_MISSING,
                "mechanism": "Chair bottom sled rail (height 35mm) is lower than 2D LiDAR (160mm) and higher than robot clearance (25mm); wheels spun without forward progress."
            }
        elif oscillations > 5 or replans > 15:
            return {
                "category": FailureCategory.NARROW_CORRIDOR_OSCILLATION,
                "primary_cause": RootCause.FOOTPRINT_INFLATION_DEFECT,
                "secondary_cause": RootCause.PLANNER_LOGIC_DEFECT,
                "mechanism": "Narrow passageway (0.72m) with circular footprint (radius 0.19m) and inflation margin caused costmap conflict and DWB trajectory oscillation."
            }
        elif stalled:
            return {
                "category": FailureCategory.STALL_UNRESPONSIVE,
                "primary_cause": RootCause.SAFETY_BEHAVIOR_DEFECT,
                "secondary_cause": RootCause.FOOTPRINT_INFLATION_DEFECT,
                "mechanism": "Robot velocity commands issued but physical displacement stalled without recovery abort."
            }
        elif path_through_obstacle:
            return {
                "category": FailureCategory.PLANNER_PATH_THROUGH_BARRIER,
                "primary_cause": RootCause.PLANNER_LOGIC_DEFECT,
                "secondary_cause": RootCause.COSTMAP_LAYER_MISSING,
                "mechanism": "Global planner generated trajectory directly intersecting unmapped/unobserved obstacle structure."
            }
        else:
            return {
                "category": FailureCategory.SUCCESS,
                "primary_cause": RootCause.NONE,
                "secondary_cause": RootCause.NONE,
                "mechanism": "Navigation completed within tolerance without hazardous clearance or physical collision."
            }
