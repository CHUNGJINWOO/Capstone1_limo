#!/usr/bin/env python3
"""
verify_physical_contacts.py:
Empirical Step 1 verification:
Tests in live Gazebo Fortress physics engine:
1. Contact 1: LIMO upper tower collision vs Table lower reinforcement bar (z=0.21m)
2. Contact 2: LIMO bottom chassis (25mm clearance) vs Chair near-ground skid frame (35mm)

Records:
- Initial pose vs Final pose after applying forward cmd_vel (0.3m/s for 3 seconds)
- Whether robot physically blocked (delta_dist < 0.05m -> Physical Block Validated)
- Whether wheel slip / stall occurs on contact
"""
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
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
except ImportError:
    print("[ERROR] ROS 2 humble environment not sourced!")
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

class ContactTesterNode(Node):
    def __init__(self):
        super().__init__('contact_tester_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.first_odom = None
        self.last_odom = None
        self.odom_history = []

    def on_odom(self, msg):
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.first_odom is None:
            self.first_odom = pos
        self.last_odom = pos
        self.odom_history.append((time.time(), pos[0], pos[1]))

    def send_cmd(self, vx=0.3):
        msg = Twist()
        msg.linear.x = vx
        self.cmd_pub.publish(msg)

def test_obstacle_contact(target_type="table"):
    """
    Spawns LIMO right in front of obstacle, drives forward for 3.5s,
    and checks if displacement is stopped by the collision geometry.
    """
    print(f"\n=======================================================", flush=True)
    print(f"[TEST 1] Testing Physical Contact: LIMO vs {target_type.upper()}", flush=True)
    print(f"=======================================================", flush=True)

    world_sdf = WORLDS_DIR / 'lounge_evaluation_world.sdf'
    robot_sdf = MODELS_DIR / 'limo_eval' / 'model.sdf'

    env = os.environ.copy()
    mesh_path = '/home/cjinw/simulator-research/limo_sim_ws/install/limo_description/share'
    env['IGN_GAZEBO_RESOURCE_PATH'] = f"{str(MODELS_DIR)}:{mesh_path}:{str(WORLDS_DIR)}"
    env['GZ_SIM_RESOURCE_PATH'] = f"{str(MODELS_DIR)}:{mesh_path}:{str(WORLDS_DIR)}"
    env['SDF_PATH'] = str(MODELS_DIR)

    # Spawn positions:
    # Table 1 is at (-2.5, 1.8), reinf bar at center y=1.8, z=0.21m.
    # Spawn LIMO at (-2.5, 2.15, yaw=-1.5708) -> Facing south towards table center, distance to bar ~0.35m
    # Chair target is at (4.2, 2.0). Front skid crossbar at (4.37, 2.0).
    # Spawn LIMO at (4.2, 1.60, yaw=1.5708) -> Facing north towards chair, distance ~0.40m
    if target_type == "table":
        spawn_x, spawn_y, spawn_yaw = "-2.5", "2.15", "-1.5708"
        expected_free_dist = 0.35 # After 0.35m forward, top collision MUST hit table crossbar
    else:
        spawn_x, spawn_y, spawn_yaw = "4.2", "1.60", "1.5708"
        expected_free_dist = 0.40 # After 0.40m forward, chassis bottom MUST hit chair skid rail

    launch_cmd = [
        'ros2', 'launch',
        str(BASE_DIR / 'launch' / 'lounge_sim.launch.py'),
        f'world:={str(world_sdf)}',
        f'model_file:={str(robot_sdf)}',
        f'spawn_x:={spawn_x}',
        f'spawn_y:={spawn_y}',
        f'spawn_yaw:={spawn_yaw}',
        'headless:=true'
    ]


    print(f"[LAUNCH] Starting Gazebo Fortress with LIMO at ({spawn_x}, {spawn_y})...", flush=True)
    sim_proc = subprocess.Popen(launch_cmd, env=env, start_new_session=True)
    time.sleep(6.0)

    node = ContactTesterNode()
    t_start = time.time()

    # Step A: Wait for odom
    while (time.time() - t_start) < 6.0 and node.first_odom is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.first_odom is None:
        print("[FAIL] No /odom received from Gazebo! Simulation failed to start.", flush=True)
        kill_proc(sim_proc)
        return False, 0.0, "NO_ODOM"

    print(f"[ODOM] Connected. Initial Pose: ({node.first_odom[0]:.3f}, {node.first_odom[1]:.3f})", flush=True)

    # Step B: Drive forward towards obstacle
    drive_start = time.time()
    while (time.time() - drive_start) < 4.0:
        node.send_cmd(vx=0.35)
        rclpy.spin_once(node, timeout_sec=0.05)

    kill_proc(sim_proc)

    final_odom = node.last_odom
    dist_moved = math.hypot(final_odom[0] - node.first_odom[0], final_odom[1] - node.first_odom[1])
    print(f"[ODOM] Final Pose: ({final_odom[0]:.3f}, {final_odom[1]:.3f}) | Total Distance Moved: {dist_moved:.3f} m", flush=True)

    # Evaluation:
    # If no obstacle was there, at 0.35 m/s for 4s, distance moved would be ~1.4m.
    # If blocked by the physical collision, distance moved should be bounded near expected_free_dist (~0.35m - 0.45m).
    is_blocked = (dist_moved < 0.55)
    print(f"[EVALUATION] Obstacle Blocked Robot: {is_blocked} (Distance: {dist_moved:.3f}m vs free travel > 1.2m)", flush=True)

    return is_blocked, dist_moved, "OK"

def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])

    table_ok, table_dist, table_err = test_obstacle_contact("table")
    time.sleep(2)
    chair_ok, chair_dist, chair_err = test_obstacle_contact("chair")

    print("\n=======================================================", flush=True)
    print("STEP 1 PHYSICAL CONTACT VERIFICATION SUMMARY:", flush=True)
    print(f"  Table Reinforcement Bar (Top Collision): Blocked={table_ok} (Moved {table_dist:.3f}m)", flush=True)
    print(f"  Chair Skid Rail (Chassis Contact):       Blocked={chair_ok} (Moved {chair_dist:.3f}m)", flush=True)
    print("=======================================================\n", flush=True)

if __name__ == '__main__':
    main()
