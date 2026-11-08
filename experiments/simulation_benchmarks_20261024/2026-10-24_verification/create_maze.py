#!/usr/bin/env python3
"""
create_maze.py:
Generates an indoor maze environment for LIMO autonomous navigation validation.
Produces both:
  1) 2D ROS occupancy map (YAML + PGM)
  2) Gazebo Fortress SDF world (maze_world.sdf)
from the exact same geometric definition to guarantee 100% 1:1 alignment.
"""

import os
import yaml
import numpy as np
from PIL import Image

def build_maze():
    # World bounding: 12.0m x 10.0m
    # Origin: [-6.0, -5.0, 0]
    # Resolution: 0.05m -> Width: 240, Height: 200
    res = 0.05
    ox, oy = -6.0, -5.0
    W_m, H_m = 12.0, 10.0
    W, H = int(round(W_m / res)), int(round(H_m / res))

    # Initialize occupancy grid with free space (254)
    # Background outside maze boundary will be unknown (205) or wall (0)
    grid = np.full((H, W), 254, dtype=np.uint8)

    # List of wall boxes in world coordinates: [cx, cy, sx, sy, height]
    # cz will be height / 2.0
    wall_height = 1.0
    walls = []

    def add_box(cx, cy, sx, sy):
        walls.append((cx, cy, sx, sy, wall_height))
        # Rasterize onto grid
        # World to pixel:
        # px = (x - ox) / res
        # py = H - (y - oy) / res
        x0, x1 = cx - sx/2.0, cx + sx/2.0
        y0, y1 = cy - sy/2.0, cy + sy/2.0

        c0 = max(0, min(W, int(round((x0 - ox) / res))))
        c1 = max(0, min(W, int(round((x1 - ox) / res))))
        r1 = max(0, min(H, int(round(H - (y0 - oy) / res))))
        r0 = max(0, min(H, int(round(H - (y1 - oy) / res))))

        if r0 > r1:
            r0, r1 = r1, r0
        grid[r0:r1, c0:c1] = 0

    # 1. Outer Boundary Walls (12m x 10m outer perimeter, thickness 0.2m)
    # South wall: y = -4.9
    add_box(0.0, -4.9, 12.0, 0.2)
    # North wall: y = 4.9
    add_box(0.0, 4.9, 12.0, 0.2)
    # West wall: x = -5.9
    add_box(-5.9, 0.0, 0.2, 10.0)
    # East wall: x = 5.9
    add_box(5.9, 0.0, 0.2, 10.0)

    # 2. Main Internal Corridors and Rooms
    #
    # Region 1: Start Zone (South-West)
    # Robot spawns at (-4.5, -3.5) facing +X (East).
    # South wall is at y = -4.9.
    # Divider wall separating start corridor from Loop A:
    # Wall from x = -5.8 to -2.8, y = -2.2, thickness 0.2m
    add_box(-4.3, -2.2, 3.0, 0.2)

    # East wall of Loop A: x = -2.7, y from -2.2 to 2.2
    add_box(-2.7, 0.0, 0.2, 4.2)

    # Loop A (North-West cyclic loop):
    # Enclosed area between x = [-5.8, -2.7] and y = [-2.2, 4.8]
    # With a central block at x = -4.2, y = 1.0, size 1.6 x 2.2
    # This forms a complete 1.2m wide navigable loop around the central block!
    add_box(-4.2, 1.0, 1.4, 2.0)

    # Divider between Loop A and North Corridor:
    # Gap exists at top: y = [3.5, 4.8] -> corridor connecting Loop A to North-East.
    # Gap exists at bottom: y = [-2.2, -1.0] -> opening to West corridor.

    # 3. Intersection 1: (-2.0, -3.5)
    # South corridor runs from x = -5.8 to +5.8 along y = -3.5.
    # Divider between South corridor and Dead-End room:
    # Wall from x = -1.5 to +3.5, y = -2.2, thickness 0.2m
    add_box(1.0, -2.2, 5.0, 0.2)

    # Dead-End (South-East):
    # Corridor along y = -3.5 goes to x = 4.8, but x = 5.0 has a blocking end wall!
    # Dead-end wall at x = 5.0, y from -4.8 to -2.2
    add_box(5.0, -3.5, 0.2, 2.6)
    # Result: Going east from (-2.0, -3.5) leads straight into a dead end!

    # 4. Central Section (North of Intersection 1):
    # Corridor going North from (-2.0, -3.5) through gap x = [-2.6, -1.4] along y = [-2.2, -0.6]
    # Opens into Intersection 2 at (-2.0, 0.0).

    # 5. Dual Alternate Route (Detour Area in Central-East):
    # From Intersection 2 (-2.0, 0.0), paths branch East into two routes:
    # Central Divider Island: x = [ -0.8, 2.4 ], y = [ -0.4, 0.8 ] (size: 3.2m x 1.2m)
    add_box(0.8, 0.2, 3.2, 1.2)

    # Route 1 (South branch): Narrow corridor!
    # Corridor runs between y = -2.1 and Central Island bottom y = -0.4.
    # To make it a realistic narrow corridor (폭 0.9m), add a partial guide wall:
    add_box(0.8, -1.4, 3.2, 0.2)
    # The gap between y = -1.3 and y = -0.4 is 0.9m -> Narrow Corridor!

    # Route 2 (North branch): Wide detour route!
    # Runs between Central Island top y = 0.8 and North wall / divider y = 2.4.
    # Gap is 1.6m wide.
    # Fixed obstacle 1 in wide route: A pillar at (0.8, 1.6) size 0.4m x 0.4m
    add_box(0.8, 1.6, 0.4, 0.4)

    # 6. North-East Room & Goal Area:
    # North-East corridor leads to Goal Area at (4.0, 2.5).
    # Divider wall between North-East room and Central:
    add_box(3.2, 3.4, 0.2, 2.8)

    # Fixed obstacle 2 in East room: Column at (4.2, 0.2), size 0.5m x 0.5m
    add_box(4.2, 0.2, 0.5, 0.5)

    # Fixed obstacle 3: Column at (4.0, -3.0), size 0.4m x 0.4m inside dead-end room
    add_box(4.0, -3.0, 0.4, 0.4)

    return grid, walls, ox, oy, res, W, H

def export_maze(output_dir):
    os.makedirs(output_dir, exist_ok=True)
    grid, walls, ox, oy, res, W, H = build_maze()

    # 1. Save PGM
    pgm_path = os.path.join(output_dir, 'maze_map.pgm')
    im = Image.fromarray(grid)
    im.save(pgm_path)
    print(f"[create_maze] Saved map image: {pgm_path} ({W}x{H})")

    # 2. Save YAML
    yaml_path = os.path.join(output_dir, 'maze_map.yaml')
    yaml_data = {
        'image': 'maze_map.pgm',
        'resolution': float(res),
        'origin': [float(ox), float(oy), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
        'mode': 'trinary'
    }
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
    print(f"[create_maze] Saved map metadata: {yaml_path}")

    # 3. Generate Gazebo Fortress SDF world
    sdf_path = os.path.join(output_dir, 'maze_world.sdf')
    world_name = 'mc3f_world'

    lines = []
    lines.append('<?xml version="1.0" ?>')
    lines.append('<sdf version="1.6">')
    lines.append(f'  <world name="{world_name}">')
    lines.append('    <physics name="1ms" type="ignored">')
    lines.append('      <max_step_size>0.001</max_step_size>')
    lines.append('      <real_time_factor>1.0</real_time_factor>')
    lines.append('    </physics>')
    lines.append('')
    lines.append('    <!-- Essential Ignition Gazebo 6 Systems -->')
    lines.append('    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>')
    lines.append('    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>')
    lines.append('    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>')
    lines.append('    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">')
    lines.append('      <render_engine>ogre2</render_engine>')
    lines.append('    </plugin>')
    lines.append('    <plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu"/>')
    lines.append('')
    lines.append('    <!-- Directional Light -->')
    lines.append('    <light name="sun" type="directional">')
    lines.append('      <cast_shadows>false</cast_shadows>')
    lines.append('      <pose>0 0 20 0 0 0</pose>')
    lines.append('      <diffuse>0.8 0.8 0.8 1</diffuse>')
    lines.append('      <specular>0.2 0.2 0.2 1</specular>')
    lines.append('      <direction>-0.5 0.1 -0.9</direction>')
    lines.append('    </light>')
    lines.append('')
    lines.append('    <!-- Ground Plane (covers 40m x 40m) -->')
    lines.append('    <model name="ground_plane">')
    lines.append('      <static>true</static>')
    lines.append('      <link name="ground_plane_link">')
    lines.append('        <collision name="ground_collision">')
    lines.append('          <geometry>')
    lines.append('            <plane>')
    lines.append('              <normal>0 0 1</normal>')
    lines.append('              <size>40 40</size>')
    lines.append('            </plane>')
    lines.append('          </geometry>')
    lines.append('          <surface>')
    lines.append('            <friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>')
    lines.append('          </surface>')
    lines.append('        </collision>')
    lines.append('        <visual name="ground_visual">')
    lines.append('          <cast_shadows>false</cast_shadows>')
    lines.append('          <geometry>')
    lines.append('            <plane>')
    lines.append('              <normal>0 0 1</normal>')
    lines.append('              <size>40 40</size>')
    lines.append('            </plane>')
    lines.append('          </geometry>')
    lines.append('          <material>')
    lines.append('            <ambient>0.65 0.65 0.65 1</ambient>')
    lines.append('            <diffuse>0.65 0.65 0.65 1</diffuse>')
    lines.append('          </material>')
    lines.append('        </visual>')
    lines.append('      </link>')
    lines.append('    </model>')
    lines.append('')
    lines.append('    <!-- Overhead Demo Camera covering the full 12m x 10m maze -->')
    lines.append('    <model name="demo_camera">')
    lines.append('      <static>true</static>')
    lines.append('      <pose>0 0 13.0 0 1.57079632679 0</pose>')
    lines.append('      <link name="camera_link">')
    lines.append('        <sensor name="overhead" type="camera">')
    lines.append('          <always_on>true</always_on>')
    lines.append('          <update_rate>10</update_rate>')
    lines.append('          <topic>/demo/overhead</topic>')
    lines.append('          <camera>')
    lines.append('            <horizontal_fov>1.10</horizontal_fov>')
    lines.append('            <image>')
    lines.append('              <width>1280</width>')
    lines.append('              <height>960</height>')
    lines.append('              <format>R8G8B8</format>')
    lines.append('            </image>')
    lines.append('            <clip><near>0.1</near><far>40</far></clip>')
    lines.append('          </camera>')
    lines.append('        </sensor>')
    lines.append('      </link>')
    lines.append('    </model>')
    lines.append('')
    lines.append('    <!-- Maze Walls Model: 1:1 aligned with map frame -->')
    lines.append('    <model name="maze_walls">')
    lines.append('      <static>true</static>')
    lines.append('      <pose>0 0 0 0 0 0</pose>')
    lines.append('      <link name="walls_link">')

    for idx, (cx, cy, sx, sy, sz) in enumerate(walls):
        cz = sz / 2.0
        lines.append(f'        <collision name="c_{idx}">')
        lines.append(f'          <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>')
        lines.append('          <geometry>')
        lines.append(f'            <box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>')
        lines.append('          </geometry>')
        lines.append('        </collision>')
        lines.append(f'        <visual name="v_{idx}">')
        lines.append('          <cast_shadows>false</cast_shadows>')
        lines.append(f'          <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>')
        lines.append('          <geometry>')
        lines.append(f'            <box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>')
        lines.append('          </geometry>')
        lines.append('          <material>')
        lines.append('            <ambient>0.3 0.35 0.45 1</ambient>')
        lines.append('            <diffuse>0.35 0.4 0.5 1</diffuse>')
        lines.append('          </material>')
        lines.append('        </visual>')

    lines.append('      </link>')
    lines.append('    </model>')
    lines.append('  </world>')
    lines.append('</sdf>')

    with open(sdf_path, 'w') as f:
        f.write('\n'.join(lines))
    print(f"[create_maze] Saved SDF world: {sdf_path} with {len(walls)} wall elements.")

if __name__ == '__main__':
    from pathlib import Path
    out = str(Path(__file__).resolve().parent / 'maze')
    export_maze(out)
