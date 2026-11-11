#!/usr/bin/env python3
"""
create_baseline_map.py:
Runs Gazebo Fortress simulation with complex_indoor_stageA.sdf,
launches online slam_toolbox with use_sim_time:=true,
drives the robot to map a portion of the environment (West Loop & South Corridor),
and serializes the map into:
  - complex_indoor_baseline.posegraph
  - complex_indoor_baseline.data
  - complex_indoor_baseline.yaml
  - complex_indoor_baseline.pgm

Coordinate Transformation (Fixed Pre-Test):
  Robot starts at World Pose: (x_w=-6.2, y_w=-2.0, yaw_w=0.0)
  Initial SLAM Map Origin corresponds to robot spawn pose.
  Therefore:
    x_map = x_world - (-6.2) = x_world + 6.2
    y_map = y_world - (-2.0) = y_world + 2.0
    yaw_map = yaw_world - 0.0 = yaw_world
"""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph


EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification')
MAP_PREFIX = EXP_DIR / 'maps/complex_indoor_baseline'
WORLD_SDF = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf')
SLAM_CONFIG = EXP_DIR / 'config/slam_params_sim.yaml'


def kill_proc_group(proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=3)
        except Exception:
            pass


class MapCreationDriver(Node):
    def __init__(self):
        super().__init__('map_creation_driver')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_received = False
        self.scan_received = False
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 1)
        self.serialize_cli = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')

    def on_map(self, msg):
        self.map_received = True

    def on_scan(self, msg):
        self.scan_received = True

    def drive(self, vx, wz, duration_sec):
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        end_time = time.monotonic() + duration_sec
        while time.monotonic() < end_time and rclpy.ok():
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        # stop
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        rclpy.spin_once(self, timeout_sec=0.1)


def main():
    print("[1/5] Launching Gazebo Fortress...")
    sim_cmd = [
        'ros2', 'launch', 'limo_capstone', 'limo_mc3f_sim.launch.py',
        f'world:={WORLD_SDF}',
        'spawn_x:=-6.2',
        'spawn_y:=-2.0',
        'spawn_yaw:=0.0',
        'headless:=true',
        'rviz:=false'
    ]
    sim_proc = subprocess.Popen(sim_cmd, start_new_session=True)

    print("[2/5] Launching SLAM Toolbox...")
    slam_cmd = [
        'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
        'use_sim_time:=true',
        f'slam_params_file:={SLAM_CONFIG}'
    ]
    slam_proc = subprocess.Popen(slam_cmd, start_new_session=True)

    rclpy.init()
    node = MapCreationDriver()

    try:
        print("[3/5] Waiting for /scan and /map...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.scan_received and node.map_received:
                print("  -> /scan and /map received!")
                break
        else:
            raise RuntimeError("Timeout waiting for /scan or /map from simulation")

        # Let SLAM process initial scans
        time.sleep(2.0)

        print("[4/5] Driving robot to map West Loop and South Corridor...")
        # Step A: Forward down South Corridor (x: -6.2 -> -3.5)
        node.drive(vx=0.25, wz=0.0, duration_sec=11.0)
        time.sleep(1.0)

        # Step B: Turn left into Central-West avenue (facing +y)
        node.drive(vx=0.15, wz=0.40, duration_sec=4.0)
        time.sleep(1.0)

        # Step C: Drive along Central-West avenue (y: -2.0 -> 0.5)
        node.drive(vx=0.20, wz=0.0, duration_sec=8.0)
        time.sleep(1.0)

        # Step D: Turn left towards West corridor (facing -x)
        node.drive(vx=0.15, wz=0.40, duration_sec=4.0)
        time.sleep(1.0)

        # Step E: Drive across above island_west_loop
        node.drive(vx=0.20, wz=0.0, duration_sec=6.0)
        time.sleep(1.0)

        # Step F: Turn left towards South Corridor (facing -y)
        node.drive(vx=0.15, wz=0.40, duration_sec=4.0)
        time.sleep(1.0)

        # Step G: Drive back down West corridor
        node.drive(vx=0.20, wz=0.0, duration_sec=8.0)
        time.sleep(1.0)

        # Step H: Turn back to face East (yaw ~ 0)
        node.drive(vx=0.15, wz=0.40, duration_sec=4.0)
        time.sleep(2.0)

        print("[5/5] Serializing posegraph and saving map files...")
        MAP_PREFIX.parent.mkdir(parents=True, exist_ok=True)

        # Wait for service
        if not node.serialize_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Service /slam_toolbox/serialize_map not available")

        req = SerializePoseGraph.Request()
        req.filename = str(MAP_PREFIX)
        fut = node.serialize_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("Failed to serialize posegraph")
        print(f"  -> Posegraph serialized to {MAP_PREFIX}.posegraph / .data")

        # Save standard 2D map (yaml + pgm)
        map_saver_cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', str(MAP_PREFIX),
            '--ros-args', '-p', 'use_sim_time:=true'
        ]
        res = subprocess.run(map_saver_cmd, capture_output=True, text=True, timeout=15)
        print("  -> map_saver_cli returned:", res.returncode)
        if res.returncode != 0:
            print("map_saver stderr:", res.stderr)

        # Verify files
        for ext in ('.posegraph', '.data', '.yaml', '.pgm'):
            f = Path(str(MAP_PREFIX) + ext)
            if not f.is_file() or f.stat().st_size == 0:
                raise RuntimeError(f"Expected map file missing or empty: {f}")
            print(f"  [OK] {f.name} ({f.stat().st_size} bytes)")

        print("=== Baseline Map Generation Complete! ===")

    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Cleaning up SLAM and Gazebo processes...")
        kill_proc_group(slam_proc)
        kill_proc_group(sim_proc)
        time.sleep(2.0)


if __name__ == '__main__':
    main()
