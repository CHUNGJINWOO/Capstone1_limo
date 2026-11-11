#!/usr/bin/env python3
"""
generate_lounge_world.py:
Generates the parameterized SDF simulation world and individual models
for LIMO Pro lounge obstacle and failure reproduction experiments.

Parameters customizable:
- Table dimensions (top width, depth, leg clearance, reinforcement bar height)
- Chair dimensions (seat height, skid frame height, leg clearance)
- Glass partition properties (length, position, doorway width)
- Corridor clearances (narrow corridor width, wide corridor width)
"""
import argparse
import math
import os
from pathlib import Path

BASE_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107')
MODELS_DIR = BASE_DIR / 'models'
WORLDS_DIR = BASE_DIR / 'worlds'

def generate_table_sdf(top_w=1.40, top_d=0.80, top_h=0.75, leg_r=0.02, reinf_h=0.21, reinf_thick=0.035):
    """Generates SDF string for 4-person table with apron and lower crossbars."""
    leg_x = (top_w / 2.0) - 0.10
    leg_y = (top_d / 2.0) - 0.07
    leg_len = top_h - 0.03
    leg_cz = leg_len / 2.0

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="lounge_table">
    <static>true</static>
    <link name="table_link">
      <!-- Table Top -->
      <visual name="top_visual">
        <pose>0 0 {top_h - 0.015:.4f} 0 0 0</pose>
        <geometry><box><size>{top_w:.3f} {top_d:.3f} 0.03</size></box></geometry>
        <material><ambient>0.65 0.55 0.45 1</ambient><diffuse>0.70 0.60 0.50 1</diffuse></material>
      </visual>
      <collision name="top_collision">
        <pose>0 0 {top_h - 0.015:.4f} 0 0 0</pose>
        <geometry><box><size>{top_w:.3f} {top_d:.3f} 0.03</size></box></geometry>
      </collision>

      <!-- Upper Apron (under top restriction) -->
      <visual name="apron_visual">
        <pose>0 0 {top_h - 0.05:.4f} 0 0 0</pose>
        <geometry><box><size>{top_w - 0.10:.3f} {top_d - 0.10:.3f} 0.04</size></box></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
      <collision name="apron_collision">
        <pose>0 0 {top_h - 0.05:.4f} 0 0 0</pose>
        <geometry><box><size>{top_w - 0.10:.3f} {top_d - 0.10:.3f} 0.04</size></box></geometry>
      </collision>

      <!-- 4 Legs (LiDAR at z=0.16m sees only these 4 cylinders) -->
      <visual name="leg1_vis"><pose>{leg_x:.3f} {leg_y:.3f} {leg_cz:.3f} 0 0 0</pose><geometry><cylinder><radius>{leg_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual>
      <collision name="leg1_col"><pose>{leg_x:.3f} {leg_y:.3f} {leg_cz:.3f} 0 0 0</pose><geometry><cylinder><radius>{leg_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry></collision>

      <visual name="leg2_vis"><pose>{leg_x:.3f} {-leg_y:.3f} {leg_cz:.3f} 0 0 0</pose><geometry><cylinder><radius>{leg_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual>
      <collision name="leg2_col"><pose>{leg_x:.3f} {-leg_y:.3f} {leg_cz:.3f} 0 0 0</pose><geometry><cylinder><radius>{leg_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry></collision>

      <visual name="leg3_vis"><pose>{-leg_x:.3f} {leg_y:.3f} {leg_cz:.3f} 0 0 0</pose><geometry><cylinder><radius>{leg_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual>
      <collision name="leg3_col"><pose>{-leg_x:.3f} {leg_y:.3f} {leg_cz:.3f} 0 0 0</pose><geometry><cylinder><radius>{leg_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry></collision>

      <visual name="leg4_vis"><pose>{-leg_x:.3f} {-leg_y:.3f} {leg_cz:.3f} 0 0 0</pose><geometry><cylinder><radius>{leg_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual>
      <collision name="leg4_col"><pose>{-leg_x:.3f} {-leg_y:.3f} {leg_cz:.3f} 0 0 0</pose><geometry><cylinder><radius>{leg_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry></collision>

      <!-- Lower Reinforcement Frame: height z={reinf_h:.3f}m (LIMO top collides, 2D LiDAR ray misses) -->
      <visual name="reinf_bar_vis"><pose>0 0 {reinf_h:.3f} 0 0 0</pose><geometry><box><size>{top_w - 0.20:.3f} {reinf_thick:.3f} {reinf_thick:.3f}</size></box></geometry><material><ambient>0.25 0.25 0.25 1</ambient></material></visual>
      <collision name="reinf_bar_col"><pose>0 0 {reinf_h:.3f} 0 0 0</pose><geometry><box><size>{top_w - 0.20:.3f} {reinf_thick:.3f} {reinf_thick:.3f}</size></box></geometry></collision>

      <!-- Side Reinforcement Braces -->
      <visual name="side_brace_l"><pose>0 {leg_y:.3f} {reinf_h:.3f} 0 0 0</pose><geometry><box><size>{reinf_thick:.3f} {leg_y*2:.3f} {reinf_thick:.3f}</size></box></geometry><material><ambient>0.25 0.25 0.25 1</ambient></material></visual>
      <collision name="side_brace_l_col"><pose>0 {leg_y:.3f} {reinf_h:.3f} 0 0 0</pose><geometry><box><size>{reinf_thick:.3f} {leg_y*2:.3f} {reinf_thick:.3f}</size></box></geometry></collision>

      <visual name="side_brace_r"><pose>0 {-leg_y:.3f} {reinf_h:.3f} 0 0 0</pose><geometry><box><size>{reinf_thick:.3f} {leg_y*2:.3f} {reinf_thick:.3f}</size></box></geometry><material><ambient>0.25 0.25 0.25 1</ambient></material></visual>
      <collision name="side_brace_r_col"><pose>0 {-leg_y:.3f} {reinf_h:.3f} 0 0 0</pose><geometry><box><size>{reinf_thick:.3f} {leg_y*2:.3f} {reinf_thick:.3f}</size></box></geometry></collision>
    </link>
  </model>
</sdf>
"""

def generate_chair_sdf(seat_w=0.42, seat_d=0.42, seat_h=0.44, skid_z=0.035, skid_r=0.012):
    """Generates SDF string for sled-base chair with near-ground skid rails."""
    leg_x = (seat_w / 2.0) - 0.04
    leg_y = (seat_d / 2.0) - 0.04
    leg_len = seat_h - (skid_z * 2)

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="lounge_chair_sled">
    <static>true</static>
    <link name="chair_link">
      <visual name="seat_vis"><pose>0 0 {seat_h:.3f} 0 0 0</pose><geometry><box><size>{seat_w:.3f} {seat_d:.3f} 0.04</size></box></geometry><material><ambient>0.2 0.4 0.6 1</ambient><diffuse>0.2 0.4 0.6 1</diffuse></material></visual>
      <collision name="seat_col"><pose>0 0 {seat_h:.3f} 0 0 0</pose><geometry><box><size>{seat_w:.3f} {seat_d:.3f} 0.04</size></box></geometry></collision>

      <visual name="back_vis"><pose>{-leg_x:.3f} 0 {seat_h + 0.24:.3f} 0 0 0</pose><geometry><box><size>0.04 {seat_d - 0.04:.3f} 0.44</size></box></geometry><material><ambient>0.2 0.4 0.6 1</ambient><diffuse>0.2 0.4 0.6 1</diffuse></material></visual>
      <collision name="back_col"><pose>{-leg_x:.3f} 0 {seat_h + 0.24:.3f} 0 0 0</pose><geometry><box><size>0.04 {seat_d - 0.04:.3f} 0.44</size></box></geometry></collision>

      <!-- Legs -->
      <visual name="leg_fl"><pose>{leg_x:.3f} {leg_y:.3f} {seat_h/2.0:.3f} 0 0 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry><material><ambient>0.15 0.15 0.15 1</ambient></material></visual>
      <collision name="leg_fl_col"><pose>{leg_x:.3f} {leg_y:.3f} {seat_h/2.0:.3f} 0 0 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry></collision>

      <visual name="leg_fr"><pose>{leg_x:.3f} {-leg_y:.3f} {seat_h/2.0:.3f} 0 0 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry><material><ambient>0.15 0.15 0.15 1</ambient></material></visual>
      <collision name="leg_fr_col"><pose>{leg_x:.3f} {-leg_y:.3f} {seat_h/2.0:.3f} 0 0 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry></collision>

      <visual name="leg_rl"><pose>{-leg_x:.3f} {leg_y:.3f} {seat_h/2.0:.3f} 0 0 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry><material><ambient>0.15 0.15 0.15 1</ambient></material></visual>
      <collision name="leg_rl_col"><pose>{-leg_x:.3f} {leg_y:.3f} {seat_h/2.0:.3f} 0 0 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry></collision>

      <visual name="leg_rr"><pose>{-leg_x:.3f} {-leg_y:.3f} {seat_h/2.0:.3f} 0 0 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry><material><ambient>0.15 0.15 0.15 1</ambient></material></visual>
      <collision name="leg_rr_col"><pose>{-leg_x:.3f} {-leg_y:.3f} {seat_h/2.0:.3f} 0 0 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_len:.3f}</length></cylinder></geometry></collision>

      <!-- SKID BASE: Rails near ground (height z={skid_z:.3f}m, radius={skid_r:.3f}m) -->
      <visual name="sled_l"><pose>0 {leg_y:.3f} {skid_z:.3f} 0 1.570796 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_x*2:.3f}</length></cylinder></geometry><material><ambient>0.15 0.15 0.15 1</ambient></material></visual>
      <collision name="sled_l_col"><pose>0 {leg_y:.3f} {skid_z:.3f} 0 1.570796 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_x*2:.3f}</length></cylinder></geometry></collision>

      <visual name="sled_r"><pose>0 {-leg_y:.3f} {skid_z:.3f} 0 1.570796 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_x*2:.3f}</length></cylinder></geometry><material><ambient>0.15 0.15 0.15 1</ambient></material></visual>
      <collision name="sled_r_col"><pose>0 {-leg_y:.3f} {skid_z:.3f} 0 1.570796 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_x*2:.3f}</length></cylinder></geometry></collision>

      <!-- Front connecting curb bar -->
      <visual name="sled_cross"><pose>{leg_x:.3f} 0 {skid_z:.3f} 1.570796 0 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_y*2:.3f}</length></cylinder></geometry><material><ambient>0.15 0.15 0.15 1</ambient></material></visual>
      <collision name="sled_cross_col"><pose>{leg_x:.3f} 0 {skid_z:.3f} 1.570796 0 0</pose><geometry><cylinder><radius>{skid_r:.3f}</radius><length>{leg_y*2:.3f}</length></cylinder></geometry></collision>
    </link>
  </model>
</sdf>
"""

def generate_full_lounge_world(narrow_w=0.72, wide_w=1.80, glass_door_w=1.10):
    """Generates the complete lounge SDF world containing tables, chairs, glass walls, and corridors."""
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="lounge_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Ignition Gazebo Systems -->
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu"/>

    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.92 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.2 0.2 -0.95</direction>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_coll">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name="ground_vis">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <material>
            <ambient>0.82 0.82 0.85 1</ambient>
            <diffuse>0.82 0.82 0.85 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Overhead Camera for Monitoring & Ground-Truth Verification -->
    <model name="overhead_camera_model">
      <static>true</static>
      <pose>0.0 0.0 8.5 0 1.57079632679 1.57079632679</pose>
      <link name="overhead_link">
        <sensor name="overhead" type="camera">
          <always_on>true</always_on>
          <update_rate>15</update_rate>
          <topic>/lounge/overhead</topic>
          <camera>
            <horizontal_fov>1.25</horizontal_fov>
            <image><width>1280</width><height>960</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>25</far></clip>
          </camera>
        </sensor>
      </link>
    </model>

    <!-- LOUNGE ROOM STRUCTURE (12m x 9m) -->
    <model name="lounge_architecture">
      <static>true</static>
      <link name="walls_link">
        <!-- North Outer Wall (Y = 4.5m) -->
        <collision name="wall_n"><pose>0 4.5 1.0 0 0 0</pose><geometry><box><size>12.2 0.2 2.0</size></box></geometry></collision>
        <visual name="wall_n_v"><pose>0 4.5 1.0 0 0 0</pose><geometry><box><size>12.2 0.2 2.0</size></box></geometry><material><ambient>0.3 0.35 0.4 1</ambient></material></visual>

        <!-- South Outer Wall (Y = -4.5m) -->
        <collision name="wall_s"><pose>0 -4.5 1.0 0 0 0</pose><geometry><box><size>12.2 0.2 2.0</size></box></geometry></collision>
        <visual name="wall_s_v"><pose>0 -4.5 1.0 0 0 0</pose><geometry><box><size>12.2 0.2 2.0</size></box></geometry><material><ambient>0.3 0.35 0.4 1</ambient></material></visual>

        <!-- West Outer Wall (X = -6.0m) -->
        <collision name="wall_w"><pose>-6.0 0 1.0 0 0 0</pose><geometry><box><size>0.2 9.2 2.0</size></box></geometry></collision>
        <visual name="wall_w_v"><pose>-6.0 0 1.0 0 0 0</pose><geometry><box><size>0.2 9.2 2.0</size></box></geometry><material><ambient>0.3 0.35 0.4 1</ambient></material></visual>

        <!-- East Outer Wall (X = 6.0m) -->
        <collision name="wall_e"><pose>6.0 0 1.0 0 0 0</pose><geometry><box><size>0.2 9.2 2.0</size></box></geometry></collision>
        <visual name="wall_e_v"><pose>6.0 0 1.0 0 0 0</pose><geometry><box><size>0.2 9.2 2.0</size></box></geometry><material><ambient>0.3 0.35 0.4 1</ambient></material></visual>

        <!-- Mid Solid Dividing Wall (West portion of Y = -1.5m, X in [-6.0, -2.5]) -->
        <collision name="wall_mid_w"><pose>-4.25 -1.5 1.0 0 0 0</pose><geometry><box><size>3.5 0.2 2.0</size></box></geometry></collision>
        <visual name="wall_mid_w_v"><pose>-4.25 -1.5 1.0 0 0 0</pose><geometry><box><size>3.5 0.2 2.0</size></box></geometry><material><ambient>0.3 0.35 0.4 1</ambient></material></visual>

        <!-- Door frame pillar (X = -2.5 + door_w) -->
        <!-- Doorway opening is between X = -2.5 and X = -2.5 + {glass_door_w:.2f} (Clear open passageway) -->
        <collision name="door_post"><pose>{-2.5 + glass_door_w + 0.05:.3f} -1.5 1.0 0 0 0</pose><geometry><box><size>0.10 0.2 2.0</size></box></geometry></collision>
        <visual name="door_post_v"><pose>{-2.5 + glass_door_w + 0.05:.3f} -1.5 1.0 0 0 0</pose><geometry><box><size>0.10 0.2 2.0</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual>

        <!-- Narrow Corridor Restrictor Partition (X = 2.5m, Y in [1.5, 4.5]) -->
        <!-- Forms narrow corridor of width {narrow_w:.2f}m between table cluster and wall -->
        <collision name="corridor_wall_1"><pose>2.6 3.0 1.0 0 0 0</pose><geometry><box><size>0.15 3.0 2.0</size></box></geometry></collision>
        <visual name="corridor_wall_1_v"><pose>2.6 3.0 1.0 0 0 0</pose><geometry><box><size>0.15 3.0 2.0</size></box></geometry><material><ambient>0.4 0.45 0.5 1</ambient></material></visual>
      </link>
    </model>

    <!-- GLASS WALL PARTITION (Y = -1.5m, X from {-2.5 + glass_door_w + 0.10:.2f} to 5.9m) -->
    <!-- Total length ~ 7.2m. Physical collision blocks robot; transparent visual allows LiDAR rays to penetrate -->
    <model name="lounge_glass_wall">
      <static>true</static>
      <pose>1.80 -1.5 0.0 0 0 0</pose>
      <link name="glass_link">
        <collision name="glass_collision">
          <pose>0 0 0.90 0 0 0</pose>
          <geometry><box><size>7.0 0.02 1.80</size></box></geometry>
        </collision>
        <visual name="glass_visual">
          <pose>0 0 0.90 0 0 0</pose>
          <geometry><box><size>7.0 0.01 1.80</size></box></geometry>
          <material>
            <ambient>0.4 0.7 0.9 0.25</ambient>
            <diffuse>0.4 0.7 0.9 0.25</diffuse>
            <specular>0.9 0.9 0.9 0.8</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ======================================================== -->
    <!-- FURNITURE PLACEMENT (TABLES & SLED-BASE CHAIRS)          -->
    <!-- ======================================================== -->

    <!-- TABLE 1: Main Discussion Table (X = -2.5, Y = 1.8) -->
    <!-- Scenario A Target Obstacle: Robot will be tasked to navigate from North to South through it -->
    <include>
      <name>table_1</name>
      <uri>model://lounge_table</uri>
      <pose>-2.5 1.8 0 0 0 0</pose>
    </include>

    <!-- Chairs for Table 1 -->
    <include>
      <name>chair_1_north_1</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>-2.9 2.35 0 0 0 -1.570796</pose>
    </include>
    <include>
      <name>chair_1_north_2</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>-2.1 2.35 0 0 0 -1.570796</pose>
    </include>
    <include>
      <name>chair_1_south_1</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>-2.9 1.25 0 0 0 1.570796</pose>
    </include>
    <include>
      <name>chair_1_south_2</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>-2.1 1.25 0 0 0 1.570796</pose>
    </include>

    <!-- TABLE 2: Middle Lounge Table (X = 0.5, Y = 1.8) -->
    <include>
      <name>table_2</name>
      <uri>model://lounge_table</uri>
      <pose>0.5 1.8 0 0 0 0</pose>
    </include>
    <include>
      <name>chair_2_north_1</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>0.1 2.35 0 0 0 -1.570796</pose>
    </include>
    <include>
      <name>chair_2_north_2</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>0.9 2.35 0 0 0 -1.570796</pose>
    </include>
    <include>
      <name>chair_2_south_1</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>0.1 1.25 0 0 0 1.570796</pose>
    </include>
    <include>
      <name>chair_2_south_2</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>0.9 1.25 0 0 0 1.570796</pose>
    </include>

    <!-- TABLE 3: Secondary Work Table (X = -2.5, Y = 0.0) -->
    <!-- Positioned close to Table 1, creating narrow passage between chair backs -->
    <include>
      <name>table_3</name>
      <uri>model://lounge_table</uri>
      <pose>-2.5 0.0 0 0 0 0</pose>
    </include>
    <include>
      <name>chair_3_n1</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>-2.9 0.55 0 0 0 -1.570796</pose>
    </include>
    <include>
      <name>chair_3_n2</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>-2.1 0.55 0 0 0 -1.570796</pose>
    </include>
    <include>
      <name>chair_3_s1</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>-2.9 -0.55 0 0 0 1.570796</pose>
    </include>
    <include>
      <name>chair_3_s2</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>-2.1 -0.55 0 0 0 1.570796</pose>
    </include>

    <!-- ISOLATED SLED CHAIR IN CORRIDOR (Scenario B Target Obstacle) -->
    <!-- Placed at (X = 4.2, Y = 2.0) right along the main wide corridor -->
    <!-- Robot attempting to drive north-south will graze or cross its near-ground sled rails -->
    <include>
      <name>corridor_sled_chair_target</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>4.2 2.0 0 0 0 0</pose>
    </include>

    <!-- GLASS ROOM INTERIOR FURNITURE (Inside VIP Room at Y < -1.5m) -->
    <!-- Visible through the glass wall -->
    <include>
      <name>vip_table</name>
      <uri>model://lounge_table</uri>
      <pose>1.5 -3.0 0 0 0 1.570796</pose>
    </include>
    <include>
      <name>vip_chair_1</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>0.8 -3.0 0 0 0 0</pose>
    </include>
    <include>
      <name>vip_chair_2</name>
      <uri>model://lounge_chair_sled</uri>
      <pose>2.2 -3.0 0 0 0 3.14159</pose>
    </include>

  </world>
</sdf>
"""

def main():
    parser = argparse.ArgumentParser(description="Generate Lounge Simulation Environment SDF")
    parser.add_argument('--reinf-h', type=float, default=0.21, help="Table lower reinforcement bar height in meters (default: 0.21m)")
    parser.add_argument('--skid-z', type=float, default=0.035, help="Chair bottom skid frame height in meters (default: 0.035m)")
    parser.add_argument('--narrow-w', type=float, default=0.72, help="Narrow corridor width in meters (default: 0.72m)")
    parser.add_argument('--door-w', type=float, default=1.10, help="Glass doorway clear width in meters (default: 1.10m)")
    args = parser.parse_args()

    table_sdf_path = MODELS_DIR / 'lounge_table' / 'model.sdf'
    chair_sdf_path = MODELS_DIR / 'lounge_chair_sled' / 'model.sdf'
    world_sdf_path = WORLDS_DIR / 'lounge_evaluation_world.sdf'

    table_sdf = generate_table_sdf(reinf_h=args.reinf_h)
    with open(table_sdf_path, 'w') as f:
        f.write(table_sdf)
    print(f"[OK] Wrote table model SDF -> {table_sdf_path}")

    chair_sdf = generate_chair_sdf(skid_z=args.skid_z)
    with open(chair_sdf_path, 'w') as f:
        f.write(chair_sdf)
    print(f"[OK] Wrote chair model SDF -> {chair_sdf_path}")

    world_sdf = generate_full_lounge_world(narrow_w=args.narrow_w, glass_door_w=args.door_w)
    with open(world_sdf_path, 'w') as f:
        f.write(world_sdf)
    print(f"[OK] Wrote full lounge world SDF -> {world_sdf_path}")

if __name__ == '__main__':
    main()
