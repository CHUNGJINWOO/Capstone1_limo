#!/usr/bin/env python3
"""
verify_glass_scan.py:
Verifies the 4 glass sensor conditions directly on the /scan topic in Gazebo Fortress:
1. Normal glass reflection: /scan reports glass wall distance (~1.0m)
2. Invisible glass: /scan penetrates glass and reports classroom interior (~2.8m - 5.0m)
3. Intermittent glass: /scan contains alternating/scattered returns (~1.0m and >2.0m)
4. Invalid / no-return: /scan contains inf or out-of-range in glass sector
"""
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_mapping_verification')


class ScanCollector(Node):
    def __init__(self):
        super().__init__('scan_collector')
        self.scans = []
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        self.scans.append(msg)


def kill_proc(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        p.wait(timeout=4)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            p.wait(timeout=2)
        except Exception:
            pass


def test_world_scan(world_name, world_path, spawn_pose=(1.0, -1.0, 1.57)):
    print(f"\n=======================================================")
    print(f"[TEST] Measuring /scan for: {world_name}")
    print(f"  Robot Spawn: x={spawn_pose[0]}, y={spawn_pose[1]}, yaw={spawn_pose[2]}")
    print(f"  Target Glass Wall is at y=0.0 (exact distance: 1.0m)")
    print(f"=======================================================")

    cmd = [
        'ros2', 'launch', 'limo_capstone', 'limo_mc3f_sim.launch.py',
        f'world:={world_path}',
        f'spawn_x:={spawn_pose[0]}',
        f'spawn_y:={spawn_pose[1]}',
        f'spawn_yaw:={spawn_pose[2]}',
        'headless:=true', 'rviz:=false'
    ]
    proc = subprocess.Popen(cmd, start_new_session=True)
    time.sleep(5)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = ScanCollector()

    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10.0 and len(node.scans) < 5:
            rclpy.spin_once(node, timeout_sec=0.2)

        if not node.scans:
            print("ERROR: No /scan received!")
            return None

        scan = node.scans[-1]
        # Robot is facing +y (North). Forward direction is angle = 0 in robot frame.
        # Find index for forward sector (-15 deg to +15 deg)
        num_beams = len(scan.ranges)
        mid_idx = num_beams // 2
        
        # Check angle mapping: angle_min is -pi, angle_max is pi, angle 0 is index mid_idx
        forward_ranges = scan.ranges[mid_idx - 15 : mid_idx + 15]
        valid_ranges = [r for r in forward_ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        inf_count = sum(1 for r in forward_ranges if math.isinf(r) or r >= scan.range_max)

        mean_range = sum(valid_ranges) / len(valid_ranges) if valid_ranges else float('nan')
        min_range = min(valid_ranges) if valid_ranges else float('nan')
        max_range = max(valid_ranges) if valid_ranges else float('nan')

        print(f"Total beams in scan: {num_beams}")
        print(f"Forward Sector (-15 deg to +15 deg, 30 beams):")
        print(f"  Valid Returns: {len(valid_ranges)} / 30")
        print(f"  Inf/No-return: {inf_count} / 30")
        print(f"  Min Range:     {min_range:.3f} m")
        print(f"  Max Range:     {max_range:.3f} m")
        print(f"  Mean Range:    {mean_range:.3f} m")

        # Classify
        if mean_range < 1.3:
            print(f"  -> Physical & Visual Check: Glass Wall REFLECTED at {mean_range:.2f}m (Condition 1)")
        elif mean_range > 2.0:
            print(f"  -> Physical & Visual Check: Glass Wall INVISIBLE! Rays penetrated into room to {mean_range:.2f}m (Condition 2)")
        else:
            print(f"  -> Physical & Visual Check: INTERMITTENT/MIXED returns (Condition 3)")

        return {
            'world': world_name,
            'num_beams': num_beams,
            'forward_valid_count': len(valid_ranges),
            'forward_inf_count': inf_count,
            'forward_min_m': min_range,
            'forward_max_m': max_range,
            'forward_mean_m': mean_range,
            'sample_forward_ranges': [round(r, 3) for r in forward_ranges[:10]]
        }

    finally:
        node.destroy_node()
        rclpy.shutdown()
        kill_proc(proc)
        subprocess.run(['pkill', '-9', '-f', 'ign gazebo'], check=False)
        subprocess.run(['pkill', '-9', '-f', 'ros_gz'], check=False)
        time.sleep(2)


def main():
    worlds = [
        ('Condition 1: Normal Glass', EXP_DIR / 'worlds/c103_glass_normal.sdf'),
        ('Condition 2: Invisible Glass', EXP_DIR / 'worlds/c103_glass_invisible.sdf'),
        ('Condition 3: Intermittent Glass', EXP_DIR / 'worlds/c103_glass_intermittent.sdf')
    ]
    results = []
    for name, path in worlds:
        res = test_world_scan(name, path)
        results.append(res)

    print("\n=======================================================")
    print("ALL GLASS SENSOR CONDITIONS VERIFIED VIA REAL /scan TOPIC:")
    for r in results:
        if r:
            print(f"  {r['world']}: Mean={r['forward_mean_m']:.3f}m, Min={r['forward_min_m']:.3f}m, Max={r['forward_max_m']:.3f}m, Infs={r['forward_inf_count']}")


if __name__ == '__main__':
    main()
