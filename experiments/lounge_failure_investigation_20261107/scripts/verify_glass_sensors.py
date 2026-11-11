#!/usr/bin/env python3
"""
verify_glass_sensors.py:
Step 3 Verification:
Evaluates 3 distinct glass conditions in Gazebo Fortress with actual /scan topic:
1. Condition 1: Opaque Wall (Ground Truth Solid Barrier)
2. Condition 2: Visually Semi-Transparent Wall (Alpha = 0.25 with box visual)
3. Condition 3: LiDAR-Invisible Transparent Assumption (Visual omitted, Collision active)

Measures:
- Valid vs Inf/Out-of-range beam count in forward 30-degree cone (-15 to +15 deg)
- Minimum measured range
- Whether glass wall is detected in LiDAR scan
Saves results to logs/step3_glass_sensor_results.json.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

BASE_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/lounge_failure_investigation_20261107')
MODELS_DIR = BASE_DIR / 'models'
WORLDS_DIR = BASE_DIR / 'worlds'
LOGS_DIR = BASE_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
except ImportError:
    print("[ERROR] ROS 2 Humble environment not sourced!")
    sys.exit(1)

def kill_proc(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        p.wait(timeout=3)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            p.wait(timeout=1)
        except Exception:
            pass

class ScanCollector(Node):
    def __init__(self):
        super().__init__('scan_audit_collector')
        self.scans = []
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        self.scans.append(msg)

def create_glass_test_world(condition_name, sdf_path):
    """Generates standalone SDF world containing the target wall condition at y = 1.5m."""
    if condition_name == "opaque":
        wall_visual = """
        <visual name="v_wall">
          <pose>0 1.5 0.9 0 0 0</pose>
          <geometry><box><size>4.0 0.05 1.8</size></box></geometry>
          <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
        </visual>"""
    elif condition_name == "reflective": # Semi-transparent visual present
        wall_visual = """
        <visual name="v_wall">
          <pose>0 1.5 0.9 0 0 0</pose>
          <geometry><box><size>4.0 0.02 1.8</size></box></geometry>
          <material><ambient>0.4 0.7 0.9 0.25</ambient><diffuse>0.4 0.7 0.9 0.25</diffuse></material>
        </visual>"""
    else: # invisible: visual intentionally omitted
        wall_visual = """<!-- Visual omitted so gpu_lidar rays penetrate through -->"""

    content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="glass_test_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry><material><ambient>0.8 0.8 0.8 1</ambient></material></visual>
      </link>
    </model>

    <!-- Background solid wall at y = 5.0m to catch penetrating rays -->
    <model name="background_wall">
      <static>true</static>
      <link name="bg_link">
        <collision name="c"><pose>0 5.0 1.0 0 0 0</pose><geometry><box><size>10.0 0.2 2.0</size></box></geometry></collision>
        <visual name="v"><pose>0 5.0 1.0 0 0 0</pose><geometry><box><size>10.0 0.2 2.0</size></box></geometry><material><ambient>0.2 0.3 0.4 1</ambient></material></visual>
      </link>
    </model>

    <!-- Target Wall at y = 1.5m (Distance from robot = 1.5m) -->
    <model name="test_wall">
      <static>true</static>
      <link name="wall_link">
        <collision name="c_wall">
          <pose>0 1.5 0.9 0 0 0</pose>
          <geometry><box><size>4.0 0.05 1.8</size></box></geometry>
        </collision>
        {wall_visual}
      </link>
    </model>
  </world>
</sdf>
"""
    with open(sdf_path, 'w') as f:
        f.write(content)

def test_condition(cond_name):
    world_path = WORLDS_DIR / f"test_glass_{cond_name}.sdf"
    create_glass_test_world(cond_name, world_path)
    robot_sdf = MODELS_DIR / 'limo_eval' / 'model.sdf'

    env = os.environ.copy()
    mesh_path = '/home/cjinw/simulator-research/limo_sim_ws/install/limo_description/share'
    env['IGN_GAZEBO_RESOURCE_PATH'] = f"{str(MODELS_DIR)}:{mesh_path}:{str(WORLDS_DIR)}"
    env['GZ_SIM_RESOURCE_PATH'] = f"{str(MODELS_DIR)}:{mesh_path}:{str(WORLDS_DIR)}"
    env['SDF_PATH'] = str(MODELS_DIR)

    # Launch gazebo with robot spawned at (0, 0), yaw = 1.5708 (Facing north towards y = 1.5m)
    launch_cmd = [
        'ros2', 'launch',
        str(BASE_DIR / 'launch' / 'lounge_sim.launch.py'),
        f'world:={str(world_path)}',
        'world_name:=glass_test_world',
        f'model_file:={str(robot_sdf)}',
        'spawn_x:=0.0', 'spawn_y:=0.0', 'spawn_yaw:=1.5708',
        'headless:=true'
    ]


    print(f"\n[STEP 3] Testing Glass Condition: {cond_name.upper()}...", flush=True)
    sim_proc = subprocess.Popen(launch_cmd, env=env, start_new_session=True)
    time.sleep(5.0)

    node = ScanCollector()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8.0 and len(node.scans) < 10:
        rclpy.spin_once(node, timeout_sec=0.1)

    kill_proc(sim_proc)

    if not node.scans:
        print(f"[FAIL] No /scan collected for {cond_name}!", flush=True)
        return {"condition": cond_name, "status": "NO_SCAN"}

    scan = node.scans[-1]
    # Center beam is straight ahead (index ~ 180 out of 360)
    mid = len(scan.ranges) // 2
    cone = scan.ranges[mid - 15 : mid + 15] # 30-degree forward cone
    valid_cone = [r for r in cone if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
    inf_cone = sum(1 for r in cone if math.isinf(r) or r >= scan.range_max)

    min_r = min(valid_cone) if valid_cone else None
    mean_r = (sum(valid_cone)/len(valid_cone)) if valid_cone else None

    # Target wall is at exactly 1.5m from spawn (0,0) minus lidar offset (0.103m) -> ~1.397m
    # Background wall is at 5.0m minus offset -> ~4.897m
    detected_glass = False
    if min_r is not None and 1.2 <= min_r <= 1.6:
        detected_glass = True

    res = {
        "condition": cond_name,
        "total_beams": len(scan.ranges),
        "cone_valid_beams": len(valid_cone),
        "cone_inf_beams": inf_cone,
        "cone_min_range_m": round(min_r, 3) if min_r else None,
        "cone_mean_range_m": round(mean_r, 3) if mean_r else None,
        "target_detected_at_1_5m": detected_glass,
        "penetrated_to_background": (min_r is not None and min_r > 3.0)
    }

    print(f"  Result: Detected at 1.5m = {detected_glass} | Min Range = {min_r}m | Penetrated = {res['penetrated_to_background']}", flush=True)
    return res

def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    results = []
    for cond in ["opaque", "reflective", "invisible"]:
        r = test_condition(cond)
        results.append(r)
        time.sleep(2)

    out_file = LOGS_DIR / "step3_glass_sensor_results.json"
    with open(out_file, 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=2)
    print(f"\n[STEP 3 COMPLETED] Saved glass sensor results to: {out_file}\n", flush=True)

if __name__ == '__main__':
    main()
