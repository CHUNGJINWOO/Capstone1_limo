#!/usr/bin/env python3
"""
map_to_world.py: Generates a Gazebo Fortress (Ignition) SDF world from a 2D ROS map (YAML + PGM).
Maintains exact 1:1 coordinate alignment between Gazebo world frame and ROS 2 map frame.

Usage:
  python3 map_to_world.py --yaml <path_to_map.yaml> --output <path_to_output.sdf> [--wall-height 1.0]
"""

import os
import sys
import argparse
import yaml
import numpy as np
from PIL import Image

def generate_sdf(yaml_path, output_sdf_path, wall_height=1.0, world_name="mc3f_world"):
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Map YAML not found: {yaml_path}")

    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)

    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

    # Resolve PGM image path relative to YAML
    image_rel = meta['image']
    pgm_path = os.path.join(os.path.dirname(yaml_path), image_rel)
    if not os.path.exists(pgm_path):
        raise FileNotFoundError(f"Map PGM image not found: {pgm_path}")

    im = Image.open(pgm_path)
    arr = np.array(im)
    H, W = arr.shape

    # In standard ROS 2 occupancy grid (negate: 0):
    # pixel == 0 is OCCUPIED
    occ = (arr == 0)
    total_occupied_cells = int(occ.sum())

    # Deterministic horizontal-first greedy box merging to minimize box count
    # while preserving exact cell boundary geometry
    m = occ.copy()
    boxes = []
    for r in range(H):
        c = 0
        while c < W:
            if not m[r, c]:
                c += 1
                continue
            c1 = c
            while c1 < W and m[r, c1]:
                c1 += 1
            r1 = r + 1
            while r1 < H and m[r1, c:c1].all():
                r1 += 1
            m[r:r1, c:c1] = False
            boxes.append((r, c, r1, c1))
            c = c1

    print(f"[map_to_world] Input map: {yaml_path} ({W}x{H} cells, resolution: {res}m)")
    print(f"[map_to_world] Occupied cells: {total_occupied_cells} -> merged into {len(boxes)} boxes.")

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
    lines.append('    <!-- Ground Plane (covers 100m x 100m) -->')
    lines.append('    <model name="ground_plane">')
    lines.append('      <static>true</static>')
    lines.append('      <link name="ground_plane_link">')
    lines.append('        <collision name="ground_collision">')
    lines.append('          <geometry>')
    lines.append('            <plane>')
    lines.append('              <normal>0 0 1</normal>')
    lines.append('              <size>100 100</size>')
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
    lines.append('              <size>100 100</size>')
    lines.append('            </plane>')
    lines.append('          </geometry>')
    lines.append('          <material>')
    lines.append('            <ambient>0.6 0.6 0.6 1</ambient>')
    lines.append('            <diffuse>0.6 0.6 0.6 1</diffuse>')
    lines.append('          </material>')
    lines.append('        </visual>')
    lines.append('      </link>')
    lines.append('    </model>')
    lines.append('')
    lines.append('    <!-- Generated Wall Model: 1:1 aligned with map frame -->')
    lines.append('    <model name="mc3f_walls">')
    lines.append('      <static>true</static>')
    lines.append('      <pose>0 0 0 0 0 0</pose>')
    lines.append('      <link name="walls_link">')

    half_h = wall_height / 2.0
    for idx, (r0, c0, r1, c1) in enumerate(boxes):
        sx = (c1 - c0) * res
        sy = (r1 - r0) * res
        sz = wall_height
        cx = ox + (c0 + c1) * 0.5 * res
        cy = oy + (H - (r0 + r1) * 0.5) * res
        cz = half_h

        lines.append(f'        <collision name="c_{idx}">')
        lines.append(f'          <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>')
        lines.append('          <geometry>')
        lines.append(f'            <box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>')
        lines.append('          </geometry>')
        lines.append('        </collision>')
        lines.append(f'        <visual name="v_{idx}">')
        lines.append(f'          <cast_shadows>false</cast_shadows>')
        lines.append(f'          <pose>{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0</pose>')
        lines.append('          <geometry>')
        lines.append(f'            <box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>')
        lines.append('          </geometry>')
        lines.append('          <material>')
        lines.append('            <ambient>0.7 0.7 0.7 1</ambient>')
        lines.append('            <diffuse>0.7 0.7 0.7 1</diffuse>')
        lines.append('          </material>')
        lines.append('        </visual>')

    lines.append('      </link>')
    lines.append('    </model>')
    lines.append('  </world>')
    lines.append('</sdf>')

    os.makedirs(os.path.dirname(os.path.abspath(output_sdf_path)), exist_ok=True)
    with open(output_sdf_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"[map_to_world] Successfully wrote world file: {output_sdf_path}")

def main():
    parser = argparse.ArgumentParser(description="Convert ROS 2 2D occupancy map to Gazebo Fortress SDF world.")
    parser.add_argument("--yaml", required=True, help="Path to input map YAML file.")
    parser.add_argument("--output", required=True, help="Path to output SDF file.")
    parser.add_argument("--wall-height", type=float, default=1.0, help="Wall height in meters (default: 1.0).")
    parser.add_argument("--world-name", default="mc3f_world", help="Name of world element in SDF.")

    args = parser.parse_args()
    generate_sdf(args.yaml, args.output, args.wall_height, args.world_name)

if __name__ == '__main__':
    main()
