#!/usr/bin/env python3
"""Inspect /amcl_pose and /scan timestamps and values during no-motion update."""
import subprocess, time, signal, os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

def main():
    WORLD_SDF = "/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf"
    MAP_YAML = "/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification/maps/complex_indoor_baseline.yaml"

    print("Starting Gazebo...")
    sim_proc = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD_SDF}", "spawn_x:=-5.5", "spawn_y:=-2.0", "spawn_yaw:=0.0",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)

    time.sleep(5)

    print("Starting Nav2 localization (map_server + amcl)...")
    loc_proc = subprocess.Popen([
        "ros2", "launch", "nav2_bringup", "localization_launch.py",
        f"map:={MAP_YAML}", "use_sim_time:=true", "autostart:=true"
    ], start_new_session=True)

    time.sleep(6)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node('timestamp_tester')
    
    last_scan = [None]
    last_pose = [None]
    
    node.create_subscription(LaserScan, '/scan', lambda m: last_scan.__setitem__(0, m), 1)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: last_pose.__setitem__(0, m), 1)
    
    nomotion_cli = node.create_client(Empty, '/request_nomotion_update')
    reinit_cli = node.create_client(Empty, '/reinitialize_global_localization')
    
    try:
        print("Waiting for nomotion service...")
        nomotion_cli.wait_for_service(timeout_sec=10)
        reinit_cli.wait_for_service(timeout_sec=10)
        
        print("Reinitializing global localization...")
        reinit_cli.call_async(Empty.Request())
        time.sleep(1)
        
        for i in range(5):
            print(f"Calling nomotion update {i+1}...")
            nomotion_cli.call_async(Empty.Request())
            for _ in range(15):
                rclpy.spin_once(node, timeout_sec=0.1)
            
            s = last_scan[0]
            p = last_pose[0]
            s_stamp = (s.header.stamp.sec + s.header.stamp.nanosec/1e9) if s else None
            p_stamp = (p.header.stamp.sec + p.header.stamp.nanosec/1e9) if p else None
            now_sec = node.get_clock().now().nanoseconds / 1e9
            print(f"  now={now_sec:.3f}, scan_stamp={s_stamp}, pose_stamp={p_stamp}")
            if s_stamp and p_stamp:
                print(f"  diff(pose - scan) = {p_stamp - s_stamp:.4f}s")
                print(f"  diff(now - pose) = {now_sec - p_stamp:.4f}s")
                print(f"  covariance[0]={p.pose.covariance[0]:.4f}, covariance[7]={p.pose.covariance[7]:.4f}")
            time.sleep(1)
            
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(os.getpgid(loc_proc.pid), signal.SIGINT)
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
        loc_proc.wait(timeout=5)
        sim_proc.wait(timeout=5)

if __name__ == '__main__':
    main()
