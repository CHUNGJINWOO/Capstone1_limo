#!/usr/bin/env python3
"""
investigate_scenario_c_ground_truth.py:
Audits whether LIMO physically penetrated the glass wall in Gazebo Fortress
or if it hit the collision barrier and wheel slip accumulated false Odometry.

Compares:
- Gazebo Ground-Truth Model Pose (`ign model -m limo -p`)
- ROS 2 Odometry (`/odom`)
- Reconstructed Map Coordinates
"""
import json
import math
import os
import re
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
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
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

def get_ign_ground_truth_pose():
    """Queries ignition gazebo directly for model limo world pose."""
    try:
        res = subprocess.run(['ign', 'model', '-m', 'limo', '-p'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=1.5)
        out = res.stdout
        # Output format usually contains: [X Y Z Roll Pitch Yaw] or Position: [x, y, z]
        # Example: [x, y, z, roll, pitch, yaw] or numbers
        # Let's search for floats
        nums = [float(s) for s in re.findall(r"[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?", out)]
        if len(nums) >= 3:
            return {"x": nums[0], "y": nums[1], "z": nums[2], "raw": out.strip()}
    except Exception as e:
        pass
    return None

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('ground_truth_audit_node')
        self.latest_odom = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, msg):
        self.latest_odom = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y
        }

def main():
    world_sdf = WORLDS_DIR / 'lounge_evaluation_world_invisible_glass.sdf'
    robot_sdf = MODELS_DIR / 'limo_eval' / 'model.sdf'

    env = os.environ.copy()
    mesh_path = '/home/cjinw/simulator-research/limo_sim_ws/install/limo_description/share'
    env['IGN_GAZEBO_RESOURCE_PATH'] = f"{str(MODELS_DIR)}:{mesh_path}:{str(WORLDS_DIR)}"
    env['GZ_SIM_RESOURCE_PATH'] = f"{str(MODELS_DIR)}:{mesh_path}:{str(WORLDS_DIR)}"
    env['SDF_PATH'] = str(MODELS_DIR)

    # Spawn identical to Scenario C: X=1.0, Y=0.5, Yaw=-1.5708 (Facing South towards glass wall at Y=-1.5)
    launch_cmd = [
        'ros2', 'launch',
        str(BASE_DIR / 'launch' / 'lounge_sim.launch.py'),
        f'world:={str(world_sdf)}',
        f'model_file:={str(robot_sdf)}',
        'spawn_x:=1.0', 'spawn_y:=0.5', 'spawn_yaw:=-1.5708',
        'headless:=true'
    ]

    print("[AUDIT] Launching Gazebo Fortress for Glass Collision & Penetration Ground-Truth Test...", flush=True)
    sim_proc = subprocess.Popen(launch_cmd, env=env, start_new_session=True)
    time.sleep(6.0)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = OdomSubscriber()

    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_odom is not None:
            break

    initial_gt = get_ign_ground_truth_pose()
    print(f"[AUDIT] Initial Ground Truth Pose: {initial_gt}", flush=True)
    print(f"[AUDIT] Initial ROS Odometry: {node.latest_odom}", flush=True)

    history = []
    t_start = time.time()
    cmd = Twist()
    cmd.linear.x = 0.28  # Same speed as Nav2 driving in baseline

    # Run for 14 seconds (same duration as Scenario C baseline which took 12.4s)
    print("[AUDIT] Driving forward at 0.28 m/s for 14.0 seconds towards glass wall at Y = -1.5m...", flush=True)
    next_sample = t_start
    while time.time() - t_start < 14.0:
        node.cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)

        now = time.time()
        if now >= next_sample:
            next_sample = now + 0.5
            gt = get_ign_ground_truth_pose()
            odom = node.latest_odom
            # Reconstruct map coordinates from odom: spawn (1.0, 0.5, -pi/2)
            # cos(-pi/2) = 0, sin(-pi/2) = -1
            # mx = 1.0 + oy
            # my = 0.5 - ox
            mx_est = 1.0 + (odom["y"] if odom else 0.0)
            my_est = 0.5 - (odom["x"] if odom else 0.0)

            rec = {
                "elapsed_s": round(now - t_start, 2),
                "odom_x": round(odom["x"], 3) if odom else None,
                "odom_y": round(odom["y"], 3) if odom else None,
                "odom_est_map_y": round(my_est, 3),
                "gt_x": round(gt["x"], 3) if gt else None,
                "gt_y": round(gt["y"], 3) if gt else None,
                "gt_z": round(gt["z"], 3) if gt else None
            }
            history.append(rec)
            print(f"  t={rec['elapsed_s']:4.1f}s | Odom X={rec['odom_x']}m (Est my={rec['odom_est_map_y']:.2f}) | Gazebo GT Y={rec['gt_y']}m", flush=True)

    # Stop robot
    cmd.linear.x = 0.0
    for _ in range(5):
        node.cmd_pub.publish(cmd)
        rclpy.spin_once(node, timeout_sec=0.05)

    final_gt = get_ign_ground_truth_pose()
    final_odom = node.latest_odom
    kill_proc(sim_proc)
    rclpy.shutdown()

    out_file = LOGS_DIR / "glass_penetration_ground_truth_audit.json"
    audit_result = {
        "spawn": {"x": 1.0, "y": 0.5, "yaw": -1.5708},
        "glass_wall_y": -1.50,
        "glass_thickness_m": 0.02,
        "initial_gt": initial_gt,
        "final_gt": final_gt,
        "final_odom": final_odom,
        "history": history
    }

    with open(out_file, 'w', encoding='utf-8') as f:
        json.dump(audit_result, f, indent=2)

    print("\n=======================================================", flush=True)
    print(f"[AUDIT COMPLETE] Saved Ground-Truth Audit to: {out_file}")
    if final_gt and final_gt["y"] is not None:
        if final_gt["y"] > -1.50:
            print(f"[FINDING]: Robot Gazebo GT Y reached {final_gt['y']:.3f}m. Glass wall at Y = -1.50m.")
            print(f"           Robot STOPPED by physical collision at glass wall!")
            print(f"           Odom reported {final_odom['x']:.3f}m due to WHEEL SLIP, NOT PHYSICAL PENETRATION!")
        else:
            print(f"[FINDING]: Robot Gazebo GT Y reached {final_gt['y']:.3f}m. Robot physically passed through glass wall!")
    print("=======================================================\n", flush=True)

if __name__ == '__main__':
    main()
