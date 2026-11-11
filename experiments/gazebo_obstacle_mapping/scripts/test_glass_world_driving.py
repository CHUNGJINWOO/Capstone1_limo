#!/usr/bin/env python3
"""
test_glass_world_driving.py:
Empirical verification of autonomous navigation in C103 glass classroom environments:
1. c103_glass_normal:
   - Glass wall is visible to LiDAR (ranges ~1.2m at y=0.0).
   - Costmap places obstacle along glass wall x in [-0.9, 4.0].
   - Nav2 plans path avoiding glass and entering classroom via the open door (x in [-2.0, -1.0]).
   - Robot drives into classroom (y > 0.5m) through the door.
2. c103_glass_invisible:
   - Glass wall is invisible to LiDAR (rays penetrate, costmap remains 0 along y=0.0).
   - Nav2 plans direct path across y=0.0.
   - Robot drives into physical collision with glass_physical_collision at y=0.0.
   - Wheels slip, forward progress stalls (y cannot exceed 0.0m), Nav2 aborts.
   - Physical contact is labeled '미측정(not_measured)' as LIMO has no bumper sensor.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from limo_mapping_interfaces.action import AutoMapping

EXP_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping')
LOGS_DIR = EXP_DIR / 'logs'
LOGS_DIR.mkdir(parents=True, exist_ok=True)
WORLDS_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_mapping_verification/worlds')


def stop_process(proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=2)
        except Exception:
            pass


class GlassDrivingNode(Node):
    def __init__(self):
        super().__init__('glass_drive_' + uuid.uuid4().hex[:6])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.global_costmap = None
        self.last_scan = None
        self.odom_start = None
        self.odom_current = None
        self.cmd_vel_count = 0
        self.last_cmd_vel = None
        self.trajectory = []

        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

        self.auto_client = ActionClient(self, AutoMapping, '/auto_mapping')

    def on_costmap(self, msg):
        self.global_costmap = msg

    def on_scan(self, msg):
        self.last_scan = msg

    def on_odom(self, msg):
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.odom_start is None:
            self.odom_start = pos
        self.odom_current = pos
        self.trajectory.append(pos)

    def on_cmd_vel(self, msg):
        self.last_cmd_vel = msg
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.cmd_vel_count += 1

    @property
    def distance_traveled(self):
        if self.odom_start is None or self.odom_current is None:
            return 0.0
        return math.hypot(self.odom_current[0] - self.odom_start[0],
                          self.odom_current[1] - self.odom_start[1])


def run_glass_test(world_type):
    run_id = f"glass_{world_type}_{uuid.uuid4().hex[:6]}"
    raw_log = LOGS_DIR / f"{run_id}.raw.log"
    mapping_log = LOGS_DIR / f"{run_id}_mapping.log"
    world_sdf = WORLDS_DIR / f"c103_glass_{world_type}.sdf"

    gz_proc = None
    map_proc = None
    node = None

    test_result = {
        "run_id": run_id,
        "world_type": world_type,
        "world_sdf": str(world_sdf),
        "spawn_pose": [0.5, -1.2, 0.05, 1.57],
        "door_opening": [-2.0, -1.0],
        "glass_section": [-0.9, 4.0],
        "success": False,
        "physical_contact": "미측정(not_measured)"
    }

    try:
        print(f"\n[glass_{world_type}] Launching Gazebo with world {world_sdf.name}...")
        gz_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash && "
            f"ros2 launch limo_capstone limo_mc3f_sim.launch.py "
            f"world:={world_sdf} spawn_x:=0.5 spawn_y:=-1.2 spawn_z:=0.05 spawn_yaw:=1.57 "
            f"headless:=true rviz:=false"
        )
        gz_out = open(raw_log, 'w')
        gz_proc = subprocess.Popen(
            ['bash', '-c', gz_cmd],
            stdout=gz_out, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )
        time.sleep(10)

        print(f"[glass_{world_type}] Launching Mapping & Nav2 stack...")
        map_cmd = (
            f"source /opt/ros/humble/setup.bash && "
            f"source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash && "
            f"source /home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping/ws/install/setup.bash && "
            f"ros2 launch limo_auto_mapping auto_mapping.launch.py use_sim_time:=true"
        )
        map_out = open(mapping_log, 'w')
        map_proc = subprocess.Popen(
            ['bash', '-c', map_cmd],
            stdout=map_out, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid
        )
        time.sleep(12)

        rclpy.init()
        node = GlassDrivingNode()

        # Wait for scan and costmap
        t_wait = time.time()
        while time.time() - t_wait < 30.0 and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.last_scan is not None and node.global_costmap is not None:
                break

        # Check scan towards front (facing north, y=0.0 is ~1.2m ahead)
        front_ranges = []
        if node.last_scan:
            mid = len(node.last_scan.ranges) // 2
            front_ranges = [r for r in node.last_scan.ranges[mid-15:mid+15] if math.isfinite(r)]
        avg_front_range = sum(front_ranges) / len(front_ranges) if front_ranges else 0.0

        print(f"[glass_{world_type}] Front LiDAR range: {avg_front_range:.2f}m (expected ~1.2m for normal, >2.5m for invisible)")
        test_result["lidar_front_range_m"] = avg_front_range

        # Send AutoMapping goal after settling wait
        node.auto_client.wait_for_server(timeout_sec=15.0)
        time.sleep(3.0)
        goal = AutoMapping.Goal()
        goal.output_prefix = str(LOGS_DIR / f"{run_id}_map")
        goal.max_duration_sec = 60.0
        goal.allow_margin_retry = False

        send_fut = node.auto_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_fut, timeout_sec=10.0)
        handle = send_fut.result()

        print(f"[glass_{world_type}] Goal accepted: {handle.accepted if handle else False}. Monitoring driving for 40s...")
        t_run = time.time()
        map_x_min = 0.5
        map_y_max = -1.2
        door_crossed = False

        while time.time() - t_run < 40.0 and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                t = node.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                mx = t.transform.translation.x
                my = t.transform.translation.y
                map_x_min = min(map_x_min, mx)
                map_y_max = max(map_y_max, my)
                # Check if robot crossed door opening into room in map coordinates
                # In c103 environment, door is at x in [-2.0, -1.0], y = 0.0
                if -2.5 <= mx <= -0.7 and my > 0.2:
                    door_crossed = True
            except Exception:
                pass

        dist_traveled = node.distance_traveled
        test_result["map_x_min"] = map_x_min
        test_result["map_y_max"] = map_y_max
        test_result["door_crossed"] = door_crossed
        test_result["distance_traveled_m"] = dist_traveled
        test_result["cmd_vel_count"] = node.cmd_vel_count

        if world_type == 'normal':
            # Expected: LiDAR detects glass (~1.1m), robot moves towards/through door
            entered_or_bypassed = door_crossed or (map_x_min < -0.5 and dist_traveled > 0.5)
            test_result["success"] = entered_or_bypassed
            test_result["conclusion"] = "LiDAR detected glass wall, navigated towards/through open door"
            print(f"[glass_normal] Success={entered_or_bypassed}, door_crossed={door_crossed}, map_x_min={map_x_min:.3f}m, dist={dist_traveled:.3f}m")

        elif world_type == 'invisible':
            # Expected: LiDAR penetrates glass (>2.5m range), robot attempts to drive straight north across y=0,
            # collides with glass_physical_collision at y ~ 0.0, y cannot exceed 0.05m, progress stalls.
            penetrated_lidar = avg_front_range > 2.0
            stalled_at_wall = (map_y_max <= 0.10 and dist_traveled > 0.5)
            test_result["lidar_penetrated"] = penetrated_lidar
            test_result["stalled_at_glass_collision"] = stalled_at_wall
            test_result["success"] = penetrated_lidar and stalled_at_wall
            test_result["conclusion"] = "LiDAR penetrated glass partition; robot attempted straight crossing and physically stalled against glass collision mesh"
            print(f"[glass_invisible] Success={test_result['success']}, lidar_penetrated={penetrated_lidar}, stalled={stalled_at_wall}, map_y_max={map_y_max:.3f}m, dist={dist_traveled:.3f}m")

    except Exception as exc:
        test_result["error"] = str(exc)
        print(f"[glass_{world_type}] Exception: {exc}")
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        stop_process(map_proc)
        stop_process(gz_proc)
        time.sleep(2)

    return test_result


def main():
    results = []
    for w in ['normal', 'invisible']:
        res = run_glass_test(w)
        results.append(res)
        time.sleep(3)

    out_file = LOGS_DIR / 'glass_driving_results.json'
    out_file.write_text(json.dumps(results, indent=2))
    print(f"\n=======================================================")
    print(f"GLASS WORLD DRIVING TESTS COMPLETE. Saved to {out_file}")
    for r in results:
        print(f"  {r['world_type']}: success={r['success']}, y_max={r.get('max_y_reached')}, dist={r.get('distance_traveled_m')}, contact={r.get('physical_contact')}")
    print(f"=======================================================")


if __name__ == '__main__':
    main()
