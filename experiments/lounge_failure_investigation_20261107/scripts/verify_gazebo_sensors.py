#!/usr/bin/env python3
"""
verify_gazebo_sensors.py:
Step 2 Verification:
Launches the Lounge World and LIMO model in Gazebo Fortress.
Verifies and records:
- Topic availability: /clock, /scan, /odom, /tf
- Exact publishing frequencies (Hz) and timestamp intervals
- Actual robot locomotion (verifying odom displacement under cmd_vel)
Saves results to logs/step2_sensor_verification.json.
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
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage
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

class SensorAuditNode(Node):
    def __init__(self):
        super().__init__('sensor_audit_node')
        self.clock_times = []
        self.scan_times = []
        self.odom_times = []
        self.tf_times = []
        self.odom_poses = []
        self.last_scan_msg = None

        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def on_clock(self, msg):
        self.clock_times.append(time.monotonic())

    def on_scan(self, msg):
        self.scan_times.append(time.monotonic())
        self.last_scan_msg = msg

    def on_odom(self, msg):
        self.odom_times.append(time.monotonic())
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.odom_poses.append((time.monotonic(), pos[0], pos[1]))

    def on_tf(self, msg):
        self.tf_times.append(time.monotonic())

    def calc_hz(self, timestamps):
        if len(timestamps) < 2:
            return 0.0
        duration = timestamps[-1] - timestamps[0]
        return (len(timestamps) - 1) / duration if duration > 0 else 0.0

def main():
    world_sdf = WORLDS_DIR / 'lounge_evaluation_world.sdf'
    robot_sdf = MODELS_DIR / 'limo_eval' / 'model.sdf'

    env = os.environ.copy()
    mesh_path = '/home/cjinw/simulator-research/limo_sim_ws/install/limo_description/share'
    env['IGN_GAZEBO_RESOURCE_PATH'] = f"{str(MODELS_DIR)}:{mesh_path}:{str(WORLDS_DIR)}"
    env['GZ_SIM_RESOURCE_PATH'] = f"{str(MODELS_DIR)}:{mesh_path}:{str(WORLDS_DIR)}"
    env['SDF_PATH'] = str(MODELS_DIR)

    launch_cmd = [
        'ros2', 'launch',
        str(BASE_DIR / 'launch' / 'lounge_sim.launch.py'),
        f'world:={str(world_sdf)}',
        f'model_file:={str(robot_sdf)}',
        'spawn_x:=0.0', 'spawn_y:=0.0', 'spawn_yaw:=0.0',
        'headless:=true'
    ]


    print("[STEP 2] Launching Gazebo Fortress for Sensor and Locomotion Audit...", flush=True)
    sim_proc = subprocess.Popen(launch_cmd, env=env, start_new_session=True)
    time.sleep(6.0)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = SensorAuditNode()

    # Phase 1: Passive Sensor Collection for 5 seconds
    print("[STEP 2] Sampling sensor topic frequencies for 5 seconds...", flush=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Phase 2: Locomotion verification (send cmd_vel for 3 seconds)
    print("[STEP 2] Testing locomotion: sending cmd_vel linear.x = 0.25 m/s...", flush=True)
    t_drive = time.monotonic()
    initial_odom = node.odom_poses[-1] if node.odom_poses else None
    cmd = Twist()
    cmd.linear.x = 0.25
    while time.monotonic() - t_drive < 3.0:
        node.cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)

    final_odom = node.odom_poses[-1] if node.odom_poses else None
    kill_proc(sim_proc)

    # Calculate metrics
    clock_hz = node.calc_hz(node.clock_times)
    scan_hz = node.calc_hz(node.scan_times)
    odom_hz = node.calc_hz(node.odom_times)
    tf_hz = node.calc_hz(node.tf_times)

    moved_dist = 0.0
    if initial_odom and final_odom:
        moved_dist = math.hypot(final_odom[1] - initial_odom[1], final_odom[2] - initial_odom[2])

    scan_valid_beams = 0
    scan_range_min = float('nan')
    if node.last_scan_msg:
        valid = [r for r in node.last_scan_msg.ranges if math.isfinite(r) and node.last_scan_msg.range_min <= r <= node.last_scan_msg.range_max]
        scan_valid_beams = len(valid)
        scan_range_min = min(valid) if valid else float('nan')

    audit_result = {
        "timestamp": time.time(),
        "topics": {
            "/clock": {"active": len(node.clock_times) > 0, "hz": round(clock_hz, 2), "samples": len(node.clock_times)},
            "/scan": {"active": len(node.scan_times) > 0, "hz": round(scan_hz, 2), "samples": len(node.scan_times),
                      "valid_beams": scan_valid_beams, "min_range": round(scan_range_min, 3) if math.isfinite(scan_range_min) else None},
            "/odom": {"active": len(node.odom_times) > 0, "hz": round(odom_hz, 2), "samples": len(node.odom_times)},
            "/tf": {"active": len(node.tf_times) > 0, "hz": round(tf_hz, 2), "samples": len(node.tf_times)}
        },
        "locomotion": {
            "initial_odom": initial_odom,
            "final_odom": final_odom,
            "moved_distance_m": round(moved_dist, 3),
            "locomotion_verified": moved_dist > 0.30
        }
    }

    out_file = LOGS_DIR / "step2_sensor_verification.json"
    with open(out_file, 'w', encoding='utf-8') as f:
        json.dump(audit_result, f, indent=2)

    print("\n=======================================================", flush=True)
    print("STEP 2 SENSOR & LOCOMOTION AUDIT RESULTS:", flush=True)
    print(f"  /clock: active={audit_result['topics']['/clock']['active']}, Hz={clock_hz:.1f}")
    print(f"  /scan:  active={audit_result['topics']['/scan']['active']}, Hz={scan_hz:.1f}, Valid Beams={scan_valid_beams}/360")
    print(f"  /odom:  active={audit_result['topics']['/odom']['active']}, Hz={odom_hz:.1f}")
    print(f"  /tf:    active={audit_result['topics']['/tf']['active']}, Hz={tf_hz:.1f}")
    print(f"  Locomotion Distance: {moved_dist:.3f} m (Verified: {audit_result['locomotion']['locomotion_verified']})")
    print(f"  Saved to: {out_file}")
    print("=======================================================\n", flush=True)

if __name__ == '__main__':
    main()
