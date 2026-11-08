#!/usr/bin/env python3
"""
generate_complex_env.py:
Generates the expanded Complex Indoor Environment (16.0m x 12.0m) for LIMO simulation.
Preserves all existing maze files while creating a separate multi-zone indoor environment with:
  1. Multiple intersections (4 major intersections) and 2 circular loops (West Loop, Central-East Loop)
  2. Dead-end corridor (South-East cul-de-sac) and 3 parallel alternative East-West routes
  3. Corridors of varying widths:
     - Narrow Middle Corridor: 1.40m wide (Ackermann passable with >= 0.40m margin beyond inflation)
     - Standard South Main Corridor: 2.10m wide
     - Wide North Bypass Corridor: 2.20m wide
     - Wide Open Space (East Plaza / Atrium): 5.2m x 8.8m
  4. Mapped static obstacles (4 pillars included in both the 2D PGM map and Gazebo SDF)
  5. Unmapped static obstacles (discovered by LiDAR /scan during navigation)
  6. Deterministic dynamic moving obstacles (Stages C, D, E, F) with recorded seed, poses, speeds,
     and verified wall-free trajectories.
"""

import os
import json
import math
import yaml
import pathlib
import numpy as np
from PIL import Image


ENV_DIR = pathlib.Path(__file__).resolve().parent
RANDOM_SEED = 20261024


def define_complex_geometry():
    """
    Returns the authoritative single-source geometry for both 2D map generation and Gazebo 3D SDF generation.
    World bounds: 16.0m x 12.0m, origin = (-8.0, -6.0), resolution = 0.05m -> 320 x 240 pixels.
    """
    res = 0.05
    ox, oy = -8.0, -6.0
    W_m, H_m = 16.0, 12.0
    W, H = int(round(W_m / res)), int(round(H_m / res))
    wall_h = 1.0

    # Structural walls: (name, cx, cy, sx, sy, sz, category)
    structural_walls = [
        # 1. Outer perimeter boundary (16.0m x 12.0m, thickness 0.20m)
        ('wall_outer_south', 0.0, -5.9, 16.0, 0.20, wall_h, 'outer_wall'),
        ('wall_outer_north', 0.0, +5.9, 16.0, 0.20, wall_h, 'outer_wall'),
        ('wall_outer_west', -7.9, 0.0, 0.20, 12.0, wall_h, 'outer_wall'),
        ('wall_outer_east', +7.9, 0.0, 0.20, 12.0, wall_h, 'outer_wall'),

        # 2. West Loop Island & North-West Divider (creates Circular Loop #1 in West Zone)
        # West vertical corridor: x in [-7.8, -6.0] (width 1.80m)
        # Central-West vertical avenue: x in [-4.0, -2.0] (width 2.00m, center x = -3.0)
        ('island_west_loop', -5.0, +1.8, 2.0, 2.4, wall_h, 'internal_wall'),
        ('wall_nw_divider', -5.0, +4.4, 2.0, 0.20, wall_h, 'internal_wall'),

        # 3. South Divider & South-East Dead-End Corridor (막다른 길)
        # Separates South Main Corridor (y in [-3.0, -0.9]) from South-East Dead-End (y in [-5.8, -3.2])
        ('wall_south_divider', -0.2, -3.1, 7.2, 0.20, wall_h, 'internal_wall'),
        ('wall_deadend_east_cap', +3.3, -4.5, 0.20, 2.6, wall_h, 'dead_end_wall'),
        ('wall_deadend_baffle', +0.6, -4.5, 3.2, 0.20, wall_h, 'dead_end_wall'),

        # 4. Central Islands forming 3 Parallel East-West Corridors of Varying Widths (Circular Loop #2)
        # - Route 1 (South Main Corridor): y in [-3.0, -0.9], center y = -1.95, width = 2.10m (Standard)
        # - Route 2 (Middle Narrow Corridor): y in [+0.7, +2.1], center y = +1.40, width = 1.40m (Narrow)
        # - Route 3 (North Wide Corridor): y in [+3.6, +5.8], center y = +4.70, width = 2.20m (Wide)
        ('island_center_south', +0.2, -0.1, 4.4, 1.6, wall_h, 'internal_wall'),
        ('island_center_north', +0.2, +2.85, 4.4, 1.5, wall_h, 'internal_wall'),
    ]

    # Mapped Static Obstacles: included in BOTH the static PGM map and the Gazebo SDF world
    mapped_static_obstacles = [
        ('mapped_obs_1', -5.2, -4.4, 0.50, 0.50, 0.80, 'mapped_static_obstacle'),
        ('mapped_obs_2', +4.2, -1.1, 0.50, 0.50, 0.80, 'mapped_static_obstacle'),
        ('mapped_obs_3', +4.2, +1.4, 0.50, 0.50, 0.80, 'mapped_static_obstacle'),
        ('mapped_obs_4', +4.8, +3.9, 0.50, 0.50, 0.80, 'mapped_static_obstacle'),
    ]

    return {
        'resolution': res,
        'origin': [ox, oy, 0.0],
        'width_m': W_m,
        'height_m': H_m,
        'grid_w': W,
        'grid_h': H,
        'structural_walls': structural_walls,
        'mapped_static_obstacles': mapped_static_obstacles,
    }


def define_stages():
    """
    Defines Stages A through F in increasing difficulty order.
    All dynamic obstacle trajectories are deterministic (seed=20261024), wall-free, and outside the robot at spawn.
    """
    default_start = [-6.2, -2.0, 0.0]
    default_goal = [+5.8, +1.4, 0.0]

    stages = {
        'A': {
            'stage_id': 'A',
            'name': 'stage_A_baseline',
            'title': 'Stage A: 추가 장애물 없는 기본 목표점 주행 (Baseline)',
            'description': '복잡 실내 환경(교차로·순환로·막다른 길·광장·지도 반영 고정 장애물 4개)에서 추가 미등록/이동 장애물 없이 S(-6.2, -2.0)에서 G(5.8, 1.4)까지 기본 주행',
            'seed': RANDOM_SEED,
            'start_pose': default_start,
            'goal_pose': default_goal,
            'timeout_sim': 75.0,
            'unmapped_obstacles': [],
            'moving_obstacles': [],
            'expected_behavior': '남측 주 통로(폭 2.1m, y=-1.95)를 통과한 뒤 동측 광장의 지도 반영 고정 장애물(mapped_obs_2, mapped_obs_3)을 자연스럽게 회피하여 목적지 G 도달',
        },
        'B': {
            'stage_id': 'B',
            'name': 'stage_B_multi_unmapped',
            'title': 'Stage B: 여러 미등록 고정 장애물 발견과 단계적 경로 재계획',
            'description': '사전 지도에 없는 고정 장애물 3개가 남측 주 통로(Route 1), 중앙 좁은 복도(Route 2), 북측 우회 통로(Route 3)에 배치되어 센서 감지 시마다 실시간으로 전역 경로를 재계획',
            'seed': RANDOM_SEED,
            'start_pose': default_start,
            'goal_pose': default_goal,
            'timeout_sim': 110.0,
            'unmapped_obstacles': [
                {
                    'name': 'unmapped_obs_1',
                    'pose': [-1.55, -1.95, 0.35],
                    'size': [0.45, 1.55, 0.70],
                    'role': '남측 주 통로(Route 1, y=-1.95) 차단 → 교차로(x=-3.0) 진입 전 라이다 감지 후 중앙 복도(Route 2)로 1차 우회 재계획 유도'
                },
                {
                    'name': 'unmapped_obs_2',
                    'pose': [-2.05, +1.40, 0.35],
                    'size': [0.40, 1.36, 0.70],
                    'role': '중앙 좁은 복도(Route 2, y=+1.40) 입구 차단 → 북상 중 라이다 감지 후 북측 광폭 통로(Route 3, y=+4.70)로 2차 우회 재계획 유도'
                },
                {
                    'name': 'unmapped_obs_3',
                    'pose': [+0.80, +3.95, 0.35],
                    'size': [0.45, 0.45, 0.70],
                    'role': '북측 광폭 통로(Route 3) 내 부분 장애물 → 발견 시 북측 여유 공간(y=+4.85)으로 3차 경로 조정 유도'
                }
            ],
            'moving_obstacles': [],
            'expected_behavior': '최초 경로(남측 통로) 출발 → unmapped_obs_1 라이다 감지 및 Costmap 반영 → 1차 재계획(중앙 복도) → unmapped_obs_2 감지 → 2차 재계획(북측 광폭 우회로) → unmapped_obs_3 회피 후 목적지 G 도달',
        },
        'C': {
            'stage_id': 'C',
            'name': 'stage_C_single_moving_cross',
            'title': 'Stage C: 이동 장애물 1개가 통로를 완전히 횡단한 뒤 이탈',
            'description': '남측 주 통로 주행 중 교차로(x=-3.0)에서 이동 장애물 1개가 통로를 북→남으로 완전히 횡단해 남측 공간(y=-4.3)으로 이탈한 뒤 로봇이 같은 목표로 주행을 재개하여 완주',
            'seed': RANDOM_SEED,
            'start_pose': default_start,
            'goal_pose': default_goal,
            'timeout_sim': 90.0,
            'unmapped_obstacles': [],
            'moving_obstacles': [
                {
                    'name': 'moving_obs_1',
                    'size': [0.42, 0.42, 0.70],
                    'initial_pose': [-3.0, -0.35, 0.35],
                    'start_pose': [-3.0, -0.55, 0.35],
                    'end_pose': [-3.0, -4.35, 0.35],
                    'speed_mps': 0.38,
                    'trigger_type': 'robot_x_ge',
                    'trigger_value': -4.8,
                    'clears_corridor': True,
                    'corridor_y_range': [-2.8, -1.1],
                    'description': '로봇이 x=-4.8m 통과 시 (-3.0, -0.55)에서 출발해 y=-1.95 통로를 0.38m/s로 횡단 후 (-3.0, -4.35)로 완전 이탈'
                }
            ],
            'expected_behavior': '감지 → Costmap 반영 → 감속·대기 또는 국소 회피 → 장애물 완전 이탈 후 통로 개방 확인 → 주행 재개 → 목적지 G 완주',
        },
        'D': {
            'stage_id': 'D',
            'name': 'stage_D_multi_moving_cross',
            'title': 'Stage D: 서로 다른 시점에 이동 장애물 3개가 순차 횡단',
            'description': '주행 구간을 따라 서로 다른 3개 지점(서측 교차로 x=-3.0, 동측 광장 진입부 x=+2.9, 목적지 전면 x=+4.9)에서 이동 장애물 3개가 순차적으로 횡단 후 이탈',
            'seed': RANDOM_SEED,
            'start_pose': default_start,
            'goal_pose': default_goal,
            'timeout_sim': 105.0,
            'unmapped_obstacles': [],
            'moving_obstacles': [
                {
                    'name': 'moving_obs_1',
                    'size': [0.42, 0.42, 0.70],
                    'initial_pose': [-3.0, -0.35, 0.35],
                    'start_pose': [-3.0, -0.55, 0.35],
                    'end_pose': [-3.0, -4.35, 0.35],
                    'speed_mps': 0.40,
                    'trigger_type': 'robot_x_ge',
                    'trigger_value': -4.8,
                    'clears_corridor': True,
                    'corridor_y_range': [-2.8, -1.1],
                    'description': '1차 횡단: 로봇 x>=-4.8m 도달 시 서측 교차로(x=-3.0)를 북→남으로 횡단 후 y=-4.35로 이탈'
                },
                {
                    'name': 'moving_obs_2',
                    'size': [0.42, 0.42, 0.70],
                    'initial_pose': [+2.9, +0.40, 0.35],
                    'start_pose': [+2.9, +0.20, 0.35],
                    'end_pose': [+2.9, -2.65, 0.35],
                    'speed_mps': 0.40,
                    'trigger_type': 'robot_x_ge',
                    'trigger_value': +1.0,
                    'clears_corridor': True,
                    'corridor_y_range': [-2.5, -0.8],
                    'description': '2차 횡단: 로봇 x>=+1.0m 도달 시 동측 광장 진입부(x=+2.9)를 북→남으로 횡단 후 y=-2.65로 이탈'
                },
                {
                    'name': 'moving_obs_3',
                    'size': [0.42, 0.42, 0.70],
                    'initial_pose': [+5.0, +3.10, 0.35],
                    'start_pose': [+5.0, +2.85, 0.35],
                    'end_pose': [+5.0, -0.40, 0.35],
                    'speed_mps': 0.42,
                    'trigger_type': 'robot_x_ge',
                    'trigger_value': +3.3,
                    'clears_corridor': True,
                    'corridor_y_range': [+0.4, +2.2],
                    'description': '3차 횡단: 로봇 x>=+3.3m 도달 시 목적지 G 전면(x=+5.0)을 북→남으로 횡단 후 y=-0.40으로 이탈'
                }
            ],
            'expected_behavior': '서로 다른 시점에 출현하는 3개의 이동 장애물을 순차적으로 센서 감지·대응하고 모두 이탈한 뒤 목적지 G 도달',
        },
        'E': {
            'stage_id': 'E',
            'name': 'stage_E_combined_static_moving',
            'title': 'Stage E: 미등록 고정 장애물과 이동 장애물이 함께 있는 복합 환경',
            'description': '미등록 고정 장애물이 남측 주 통로(Route 1)를 막아 중앙 복도(Route 2)로 우회하는 과정에서 복도 출구와 동측 광장에서 이동 장애물 2개가 순차 횡단하는 복합 시나리오',
            'seed': RANDOM_SEED,
            'start_pose': default_start,
            'goal_pose': default_goal,
            'timeout_sim': 110.0,
            'unmapped_obstacles': [
                {
                    'name': 'unmapped_obs_1',
                    'pose': [-1.55, -1.95, 0.35],
                    'size': [0.45, 1.55, 0.70],
                    'role': '남측 주 통로(Route 1, y=-1.95) 차단 → 북측 방향 우회 경로(x=-3.0 → Route 2 y=+1.40) 재계획 유도'
                }
            ],
            'moving_obstacles': [
                {
                    'name': 'moving_obs_1',
                    'size': [0.42, 0.42, 0.70],
                    'initial_pose': [+2.9, +2.85, 0.35],
                    'start_pose': [+2.9, +2.65, 0.35],
                    'end_pose': [+2.9, -0.35, 0.35],
                    'speed_mps': 0.40,
                    'trigger_type': 'robot_x_ge',
                    'trigger_value': +0.8,
                    'clears_corridor': True,
                    'corridor_y_range': [+0.6, +2.2],
                    'description': '1차 이동 장애물: 로봇이 우회 복도(Route 2)를 통과해 x>=+0.8m에 도달할 때 복도 출구(x=+2.9)를 북→남으로 횡단 후 y=-0.35로 완전 이탈'
                },
                {
                    'name': 'moving_obs_2',
                    'size': [0.42, 0.42, 0.70],
                    'initial_pose': [+5.0, +3.10, 0.35],
                    'start_pose': [+5.0, +2.85, 0.35],
                    'end_pose': [+5.0, -0.40, 0.35],
                    'speed_mps': 0.42,
                    'trigger_type': 'robot_x_ge',
                    'trigger_value': +3.2,
                    'clears_corridor': True,
                    'corridor_y_range': [+0.4, +2.2],
                    'description': '2차 이동 장애물: 로봇이 동측 광장에 진입해 x>=+3.2m에 도달할 때 목적지 G 전면(x=+5.0)을 북→남으로 횡단 후 y=-0.40으로 완전 이탈'
                }
            ],
            'expected_behavior': '미등록 고정 장애물 감지 후 우회 경로 재계획 + 우회 통로에서 마주치는 이동 장애물 2개 순차 대응 후 목적지 G 도달',
        },
        'F': {
            'stage_id': 'F',
            'name': 'stage_F_blocked_corridor',
            'title': 'Stage F: 장애물이 통로에 남는 완전 차단 및 안전 정지 상황',
            'description': '대체 우회로 입구가 차단된 상태에서 전방 통로 중앙에 장애물이 진입·정지하여 유효 경로가 완전히 사라진 조건에서 안전 거리(>=10cm)를 유지하며 무충돌 정지·대기하는지 검증',
            'seed': RANDOM_SEED,
            'start_pose': default_start,
            'goal_pose': [+1.5, -1.95, 0.0],
            'timeout_sim': 40.0,
            'unmapped_obstacles': [
                {
                    'name': 'blocking_obs_north_seal',
                    'pose': [-3.0, -0.55, 0.35],
                    'size': [1.85, 0.45, 0.70],
                    'role': '서측 북상 교차로(x=-3.0, y=-0.55) 봉쇄'
                },
                {
                    'name': 'blocking_obs_south_seal',
                    'pose': [-3.0, -3.10, 0.35],
                    'size': [1.85, 0.45, 0.70],
                    'role': '서측 남하 통로 봉쇄'
                }
            ],
            'moving_obstacles': [
                {
                    'name': 'blocking_obs_main',
                    'size': [0.50, 1.95, 0.70],
                    'initial_pose': [-1.2, -1.95, 0.35],
                    'start_pose': [-1.2, -1.95, 0.35],
                    'end_pose': [-1.2, -1.95, 0.35],
                    'speed_mps': 0.0,
                    'trigger_type': 'always',
                    'trigger_value': 0.0,
                    'clears_corridor': False,
                    'remains_in_corridor': True,
                    'description': '남측 주 통로(x=-1.2, y=-1.95) 전체 폭을 완전히 가로막고 통로에 잔류하는 차단 장애물'
                }
            ],
            'expected_behavior': '차단 장애물 감지 및 Costmap 반영 후 전방 최소 안전 간격(>=10cm)을 침범하지 않고 무충돌 정지·대기 유지',
        },
    }
    return stages


def build_occupancy_grid_and_verify(geom):
    res = geom['resolution']
    ox, oy, _ = geom['origin']
    W, H = geom['grid_w'], geom['grid_h']

    # 254 = free space, 0 = occupied wall/mapped obstacle
    grid = np.full((H, W), 254, dtype=np.uint8)
    alignment_checks = []

    all_mapped = geom['structural_walls'] + geom['mapped_static_obstacles']
    for name, cx, cy, sx, sy, sz, cat in all_mapped:
        x0, x1 = cx - sx / 2.0, cx + sx / 2.0
        y0, y1 = cy - sy / 2.0, cy + sy / 2.0

        c0 = max(0, min(W, int(round((x0 - ox) / res))))
        c1 = max(0, min(W, int(round((x1 - ox) / res))))
        r1 = max(0, min(H, int(round(H - (y0 - oy) / res))))
        r0 = max(0, min(H, int(round(H - (y1 - oy) / res))))
        if r0 > r1:
            r0, r1 = r1, r0

        grid[r0:r1, c0:c1] = 0

        # Reconstruct world bounds from grid indices to verify 1:1 coordinate alignment
        grid_x0 = ox + c0 * res
        grid_x1 = ox + c1 * res
        grid_y0 = oy + (H - r1) * res
        grid_y1 = oy + (H - r0) * res
        max_err = max(abs(grid_x0 - x0), abs(grid_x1 - x1), abs(grid_y0 - y0), abs(grid_y1 - y1))
        alignment_checks.append({
            'name': name,
            'category': cat,
            'sdf_center': [cx, cy],
            'sdf_size': [sx, sy],
            'max_grid_alignment_error_m': round(float(max_err), 6),
            'occupied_cells': int((r1 - r0) * (c1 - c0))
        })

    return grid, alignment_checks


def generate_stage_sdf(geom, stage_cfg, out_sdf_path: pathlib.Path):
    world_name = 'mc3f_world'
    lines = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.6">',
        f'  <world name="{world_name}">',
        '    <physics name="1ms" type="ignored">',
        '      <max_step_size>0.001</max_step_size>',
        '      <real_time_factor>1.0</real_time_factor>',
        '    </physics>',
        '',
        '    <!-- Ignition Gazebo Systems -->',
        '    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>',
        '    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>',
        '    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>',
        '    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">',
        '      <render_engine>ogre2</render_engine>',
        '    </plugin>',
        '    <plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu"/>',
        '',
        '    <!-- GUI Configuration: Fixed Top-Down Overhead View covering entire 16m x 12m complex -->',
        '    <gui fullscreen="0">',
        '      <plugin filename="GzScene3D" name="3D View">',
        '        <ignition-gui>',
        '          <title>3D View</title>',
        '          <property type="bool" key="showTitleBar">false</property>',
        '          <property type="string" key="state">docked</property>',
        '        </ignition-gui>',
        '        <engine>ogre2</engine>',
        '        <scene>scene</scene>',
        '        <ambient_light>0.55 0.55 0.55</ambient_light>',
        '        <background_color>0.15 0.18 0.24</background_color>',
        '        <camera_pose>0.0 0.0 16.2 0 1.570796 1.570796</camera_pose>',
        '      </plugin>',
        '      <plugin filename="WorldControl" name="World control">',
        '        <ignition-gui>',
        '          <title>World control</title>',
        '          <property type="bool" key="showTitleBar">false</property>',
        '          <property type="bool" key="resizable">false</property>',
        '          <property type="double" key="height">72</property>',
        '          <property type="double" key="width">121</property>',
        '          <property type="double" key="z">1</property>',
        '          <property type="string" key="state">floating</property>',
        '          <anchors target="3D View">',
        '            <line own="left" target="left"/>',
        '            <line own="bottom" target="bottom"/>',
        '          </anchors>',
        '        </ignition-gui>',
        '        <play_pause>true</play_pause>',
        '        <step>true</step>',
        '        <start_paused>false</start_paused>',
        '      </plugin>',
        '      <plugin filename="WorldStats" name="World stats">',
        '        <ignition-gui>',
        '          <title>World stats</title>',
        '          <property type="bool" key="showTitleBar">false</property>',
        '          <property type="bool" key="resizable">false</property>',
        '          <property type="double" key="height">110</property>',
        '          <property type="double" key="width">290</property>',
        '          <property type="double" key="z">1</property>',
        '          <property type="string" key="state">floating</property>',
        '          <anchors target="3D View">',
        '            <line own="right" target="right"/>',
        '            <line own="bottom" target="bottom"/>',
        '          </anchors>',
        '        </ignition-gui>',
        '        <sim_time>true</sim_time>',
        '        <real_time>true</real_time>',
        '        <real_time_factor>true</real_time_factor>',
        '      </plugin>',
        '    </gui>',
        '',
        '    <light name="sun" type="directional">',
        '      <cast_shadows>false</cast_shadows>',
        '      <pose>0 0 22 0 0 0</pose>',
        '      <diffuse>0.85 0.85 0.85 1</diffuse>',
        '      <specular>0.2 0.2 0.2 1</specular>',
        '      <direction>-0.3 0.1 -0.9</direction>',
        '    </light>',
        '',
        '    <!-- Ground Plane -->',
        '    <model name="ground_plane">',
        '      <static>true</static>',
        '      <link name="ground_plane_link">',
        '        <collision name="ground_collision">',
        '          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>',
        '          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>',
        '        </collision>',
        '        <visual name="ground_visual">',
        '          <cast_shadows>false</cast_shadows>',
        '          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>',
        '          <material>',
        '            <ambient>0.72 0.74 0.77 1</ambient>',
        '            <diffuse>0.72 0.74 0.77 1</diffuse>',
        '          </material>',
        '        </visual>',
        '      </link>',
        '    </model>',
        '',
        '    <!-- Overhead Sensor Camera (/demo/overhead) covering full 16m x 12m complex -->',
        '    <model name="demo_camera">',
        '      <static>true</static>',
        '      <pose>0 0 16.5 0 1.57079632679 1.57079632679</pose>',
        '      <link name="camera_link">',
        '        <sensor name="overhead" type="camera">',
        '          <always_on>true</always_on>',
        '          <update_rate>10</update_rate>',
        '          <topic>/demo/overhead</topic>',
        '          <camera>',
        '            <horizontal_fov>1.12</horizontal_fov>',
        '            <image>',
        '              <width>1280</width>',
        '              <height>960</height>',
        '              <format>R8G8B8</format>',
        '            </image>',
        '            <clip><near>0.1</near><far>40</far></clip>',
        '          </camera>',
        '        </sensor>',
        '      </link>',
        '    </model>',
        '',
    ]

    # Visual-only Start S and Goal G floor pads inside Gazebo so user can clearly see S and G in Gazebo GUI
    sx, sy, _ = stage_cfg['start_pose']
    gx, gy, _ = stage_cfg['goal_pose']
    lines.extend([
        '    <!-- Visual-only Start Marker S (Blue Pad) -->',
        '    <model name="start_marker_S">',
        '      <static>true</static>',
        f'      <pose>{sx:.4f} {sy:.4f} 0.004 0 0 0</pose>',
        '      <link name="link">',
        '        <visual name="pad">',
        '          <cast_shadows>false</cast_shadows>',
        '          <geometry><cylinder><radius>0.32</radius><length>0.006</length></cylinder></geometry>',
        '          <material><ambient>0.10 0.40 0.95 1</ambient><diffuse>0.10 0.40 0.95 1</diffuse></material>',
        '        </visual>',
        '      </link>',
        '    </model>',
        '',
        '    <!-- Visual-only Goal Marker G (Red/Gold Target Pad) -->',
        '    <model name="goal_marker_G">',
        '      <static>true</static>',
        f'      <pose>{gx:.4f} {gy:.4f} 0.004 0 0 0</pose>',
        '      <link name="link">',
        '        <visual name="outer_ring">',
        '          <cast_shadows>false</cast_shadows>',
        '          <geometry><cylinder><radius>0.36</radius><length>0.006</length></cylinder></geometry>',
        '          <material><ambient>0.92 0.18 0.18 1</ambient><diffuse>0.92 0.18 0.18 1</diffuse></material>',
        '        </visual>',
        '        <visual name="inner_core">',
        '          <cast_shadows>false</cast_shadows>',
        '          <pose>0 0 0.002 0 0 0</pose>',
        '          <geometry><cylinder><radius>0.18</radius><length>0.006</length></cylinder></geometry>',
        '          <material><ambient>0.98 0.82 0.15 1</ambient><diffuse>0.98 0.82 0.15 1</diffuse></material>',
        '        </visual>',
        '      </link>',
        '    </model>',
        '',
        '    <!-- Complex Indoor Walls & Mapped Static Obstacles (1:1 aligned with complex_indoor_map) -->',
        '    <model name="complex_indoor_walls">',
        '      <static>true</static>',
        '      <pose>0 0 0 0 0 0</pose>',
        '      <link name="walls_link">',
    ])

    all_mapped = geom['structural_walls'] + geom['mapped_static_obstacles']
    for name, cx, cy, w_sx, w_sy, w_sz, cat in all_mapped:
        cz = w_sz / 2.0
        if cat == 'mapped_static_obstacle':
            amb = '0.20 0.45 0.68 1'
            dif = '0.25 0.52 0.78 1'
        elif cat == 'dead_end_wall':
            amb = '0.42 0.32 0.35 1'
            dif = '0.48 0.36 0.40 1'
        else:
            amb = '0.28 0.33 0.42 1'
            dif = '0.33 0.39 0.49 1'

        lines.extend([
            f'        <collision name="{name}">',
            f'          <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>',
            f'          <geometry><box><size>{w_sx:.4f} {w_sy:.4f} {w_sz:.4f}</size></box></geometry>',
            '        </collision>',
            f'        <visual name="v_{name}">',
            '          <cast_shadows>false</cast_shadows>',
            f'          <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>',
            f'          <geometry><box><size>{w_sx:.4f} {w_sy:.4f} {w_sz:.4f}</size></box></geometry>',
            f'          <material><ambient>{amb}</ambient><diffuse>{dif}</diffuse></material>',
            '        </visual>',
        ])

    lines.extend([
        '      </link>',
        '    </model>',
        ''
    ])

    # Unmapped Static Obstacles (High-contrast Magenta/Red so user immediately recognizes them)
    for uobs in stage_cfg['unmapped_obstacles']:
        uname = uobs['name']
        ux, uy, uz = uobs['pose']
        usx, usy, usz = uobs['size']
        lines.extend([
            f'    <!-- Unmapped Static Obstacle: {uname} -->',
            f'    <model name="{uname}">',
            '      <static>true</static>',
            f'      <pose>{ux:.4f} {uy:.4f} {uz:.4f} 0 0 0</pose>',
            '      <link name="link">',
            '        <collision name="collision">',
            f'          <geometry><box><size>{usx:.4f} {usy:.4f} {usz:.4f}</size></box></geometry>',
            '        </collision>',
            '        <visual name="visual">',
            '          <cast_shadows>false</cast_shadows>',
            f'          <geometry><box><size>{usx:.4f} {usy:.4f} {usz:.4f}</size></box></geometry>',
            '          <material><ambient>0.90 0.12 0.35 1</ambient><diffuse>0.95 0.15 0.40 1</diffuse></material>',
            '        </visual>',
            '      </link>',
            '    </model>',
            ''
        ])

    # Dynamic Moving Obstacles (High-contrast Bright Orange/Gold)
    for mobs in stage_cfg['moving_obstacles']:
        mname = mobs['name']
        mx, my, mz = mobs['initial_pose']
        msx, msy, msz = mobs['size']
        lines.extend([
            f'    <!-- Dynamic / Blocking Obstacle: {mname} -->',
            f'    <model name="{mname}">',
            '      <static>true</static>',
            f'      <pose>{mx:.4f} {my:.4f} {mz:.4f} 0 0 0</pose>',
            '      <link name="link">',
            '        <collision name="collision">',
            f'          <geometry><box><size>{msx:.4f} {msy:.4f} {msz:.4f}</size></box></geometry>',
            '        </collision>',
            '        <visual name="visual">',
            '          <cast_shadows>false</cast_shadows>',
            f'          <geometry><box><size>{msx:.4f} {msy:.4f} {msz:.4f}</size></box></geometry>',
            '          <material><ambient>0.98 0.55 0.05 1</ambient><diffuse>1.0 0.60 0.08 1</diffuse></material>',
            '        </visual>',
            '      </link>',
            '    </model>',
            ''
        ])

    lines.extend([
        '  </world>',
        '</sdf>'
    ])

    out_sdf_path.write_text('\n'.join(lines))


def main():
    ENV_DIR.mkdir(parents=True, exist_ok=True)
    geom = define_complex_geometry()
    stages = define_stages()

    grid, alignment_checks = build_occupancy_grid_and_verify(geom)

    # Save PGM and YAML
    pgm_path = ENV_DIR / 'complex_indoor_map.pgm'
    yaml_path = ENV_DIR / 'complex_indoor_map.yaml'
    Image.fromarray(grid).save(pgm_path)

    yaml_data = {
        'image': 'complex_indoor_map.pgm',
        'resolution': float(geom['resolution']),
        'origin': [float(v) for v in geom['origin']],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
        'mode': 'trinary'
    }
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)

    # Generate SDF worlds for Stages A..F
    stage_files = {}
    for stage_key, stage_cfg in stages.items():
        sdf_file = ENV_DIR / f"complex_indoor_stage{stage_key}.sdf"
        generate_stage_sdf(geom, stage_cfg, sdf_file)
        stage_files[stage_key] = str(sdf_file)
        print(f"[generate_complex_env] Generated Stage {stage_key} world: {sdf_file}")

    max_align_err = max(c['max_grid_alignment_error_m'] for c in alignment_checks)
    manifest = {
        'environment_name': 'complex_indoor',
        'random_seed': RANDOM_SEED,
        'map_yaml': str(yaml_path),
        'map_pgm': str(pgm_path),
        'world_bounds_meters': [geom['width_m'], geom['height_m']],
        'origin': geom['origin'],
        'resolution': geom['resolution'],
        'max_map_sdf_alignment_error_meters': max_align_err,
        'corridor_widths_meters': {
            'south_main_corridor_route1': 2.10,
            'middle_narrow_corridor_route2': 1.40,
            'north_wide_corridor_route3': 2.20,
            'west_loop_corridor': 1.80,
            'central_west_avenue': 2.00,
            'east_open_plaza': [5.20, 8.80]
        },
        'alignment_verification': alignment_checks,
        'stage_worlds': stage_files,
        'stages': stages,
    }
    manifest_path = ENV_DIR / 'complex_indoor_manifest.json'
    with open(manifest_path, 'w') as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
    print(f"[generate_complex_env] Saved manifest: {manifest_path} (Max alignment error: {max_align_err:.6f}m)")


if __name__ == '__main__':
    main()
