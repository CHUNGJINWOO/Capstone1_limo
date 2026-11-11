#!/usr/bin/env python3
import os, sys, time, signal, subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped

EXP_DIR = "/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification"
MAP_PREFIX = f"{EXP_DIR}/maps/complex_indoor_baseline"
WORLD = "/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf"

def main():
    spawn_x, spawn_y, spawn_yaw = -6.5, 1.8, 1.57
    print(f"Spawning Gazebo at ({spawn_x}, {spawn_y}, {spawn_yaw})...")
    sim_proc = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD}", f"spawn_x:={spawn_x}", f"spawn_y:={spawn_y}", f"spawn_yaw:={spawn_yaw}",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(6)

    print("Launching auto_resume with localization_timeout_sec:=180.0...")
    resume_proc = subprocess.Popen([
        "ros2", "launch", "limo_auto_mapping", "auto_resume.launch.py",
        f"posegraph:={MAP_PREFIX}",
        "use_sim_time:=true",
        "localization_timeout_sec:=180.0"
    ], start_new_session=True)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node('loc_watcher')
    
    statuses = []
    ready_flags = []
    
    node.create_subscription(String, '/auto_mapping/resume_status', lambda m: statuses.append((time.monotonic(), m.data)), 10)
    node.create_subscription(Bool, '/auto_mapping/localization_ready', lambda m: ready_flags.append((time.monotonic(), m.data)), 10)

    start = time.monotonic()
    last_p = 0
    try:
        while time.monotonic() - start < 190.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if resume_proc.poll() is not None:
                print(f"auto_resume exited with code {resume_proc.returncode}")
                break
            if len(statuses) > last_p:
                for t, s in statuses[last_p:]:
                    print(f"[{t-start:6.1f}s] STATUS: {s}")
                last_p = len(statuses)
            if any(f[1] for f in ready_flags):
                print(f"*** READY DETECTED at {time.monotonic()-start:.1f}s! ***")
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for p in [resume_proc, sim_proc]:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
                p.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                    p.wait(timeout=2)
                except Exception:
                    pass

if __name__ == '__main__':
    main()
