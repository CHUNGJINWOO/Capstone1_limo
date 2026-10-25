#!/usr/bin/env python3
"""
create_scenario_worlds.py:
Generates specialized Gazebo Fortress SDF worlds for Scenarios 3, 4, 5, 6, 7.
All worlds are based on the identical maze wall geometry, adding specific obstacles.
"""

import os
import pathlib

BASE_DIR = pathlib.Path(__file__).resolve().parent / 'maze'
MAZE_SDF = BASE_DIR / 'maze_world.sdf'

def get_base_sdf_content():
    return MAZE_SDF.read_text()

def insert_model(sdf_content, model_xml):
    pos = sdf_content.rfind('  </world>')
    return sdf_content[:pos] + model_xml + '\n' + sdf_content[pos:]

def generate_scenario_worlds():
    base_sdf = get_base_sdf_content()

    # Scenario 3: Unmapped fixed obstacle in narrow corridor (x=0.0, y=-0.85)
    # Causes robot to dynamically replan and take the wide north detour route!
    sc3_obstacle = '''
    <!-- Scenario 3: Unmapped static obstacle blocking the narrow corridor -->
    <model name="unmapped_box_sc3">
      <static>true</static>
      <pose>0.0 -0.85 0.25 0 0 0</pose>
      <link name="box_link">
        <collision name="collision">
          <geometry><box><size>0.45 0.45 0.50</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.45 0.45 0.50</size></box></geometry>
          <material>
            <ambient>0.9 0.2 0.1 1</ambient>
            <diffuse>0.9 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    '''
    sc3_sdf = insert_model(base_sdf, sc3_obstacle)
    (BASE_DIR / 'maze_sc3_unmapped.sdf').write_text(sc3_sdf)
    print("Generated maze_sc3_unmapped.sdf")

    # Scenario 4: Dynamic appearance obstacle (initially hidden at z=-10.0, then appears)
    sc4_obstacle = '''
    <!-- Scenario 4: Dynamically appearing obstacle (starts hidden underground) -->
    <model name="dynamic_box_sc4">
      <static>true</static>
      <pose>-2.5 -3.5 -10.0 0 0 0</pose>
      <link name="box_link">
        <collision name="collision">
          <geometry><box><size>0.45 0.45 0.50</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.45 0.45 0.50</size></box></geometry>
          <material>
            <ambient>0.9 0.5 0.1 1</ambient>
            <diffuse>0.9 0.5 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    '''
    sc4_sdf = insert_model(base_sdf, sc4_obstacle)
    (BASE_DIR / 'maze_sc4_dynamic.sdf').write_text(sc4_sdf)
    print("Generated maze_sc4_dynamic.sdf")

    # Scenario 5: Moving obstacle crossing the corridor
    sc5_obstacle = '''
    <!-- Scenario 5: Moving obstacle crossing the corridor -->
    <model name="moving_box_sc5">
      <static>true</static>
      <pose>0.0 -1.6 0.25 0 0 0</pose>
      <link name="box_link">
        <collision name="collision">
          <geometry><box><size>0.35 0.35 0.50</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.35 0.35 0.50</size></box></geometry>
          <material>
            <ambient>0.1 0.3 0.9 1</ambient>
            <diffuse>0.1 0.3 0.9 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    '''
    sc5_sdf = insert_model(base_sdf, sc5_obstacle)
    (BASE_DIR / 'maze_sc5_moving.sdf').write_text(sc5_sdf)
    print("Generated maze_sc5_moving.sdf")

    # Scenario 6: Completely blocked path (single entrance to dead-end corridor blocked)
    sc6_obstacle = '''
    <!-- Scenario 6: Total corridor blockage on the single entrance to dead-end corridor -->
    <model name="barricade_total">
      <static>true</static>
      <pose>0.5 -3.5 0.50 0 0 0</pose>
      <link name="barr_link">
        <collision name="collision"><geometry><box><size>0.60 1.60 1.00</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>0.60 1.60 1.00</size></box></geometry>
          <material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>
    '''
    sc6_sdf = insert_model(base_sdf, sc6_obstacle)
    (BASE_DIR / 'maze_sc6_blocked.sdf').write_text(sc6_sdf)
    print("Generated maze_sc6_blocked.sdf")

    # Scenario 7: Blocked obstacle that gets cleared during navigation
    sc7_obstacle = '''
    <!-- Scenario 7: Temporary blockage on dead-end corridor removed at runtime -->
    <model name="removable_barrier">
      <static>true</static>
      <pose>0.5 -3.5 0.50 0 0 0</pose>
      <link name="barr_link">
        <collision name="collision"><geometry><box><size>0.60 1.60 1.00</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>0.60 1.60 1.00</size></box></geometry>
          <material><ambient>0.9 0.6 0.1 1</ambient><diffuse>0.9 0.6 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>
    '''
    sc7_sdf = insert_model(base_sdf, sc7_obstacle)
    (BASE_DIR / 'maze_sc7_clearing.sdf').write_text(sc7_sdf)
    print("Generated maze_sc7_clearing.sdf")

if __name__ == '__main__':
    generate_scenario_worlds()
