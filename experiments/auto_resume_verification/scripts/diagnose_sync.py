#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import subprocess, time, os, signal

def main():
    MAP = "/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification/maps/complex_indoor_baseline.yaml"
    WORLD = "/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf"
    
    sim = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD}", "spawn_x:=-6.5", "spawn_y:=1.8", "spawn_yaw:=1.57",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(5)
    
    loc = subprocess.Popen([
        "ros2", "launch", "nav2_bringup", "localization_launch.py",
        f"map:={MAP}", "use_sim_time:=true", "autostart:=true"
    ], start_new_session=True)
    time.sleep(5)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node('diag')
    
    scan = [None]
    pose = [None]
    node.create_subscription(LaserScan, '/scan', lambda m: scan.__setitem__(0, m), 10)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: pose.__setitem__(0, m), 10)
    
    nomotion = node.create_client(Empty, '/request_nomotion_update')
    reinit = node.create_client(Empty, '/reinitialize_global_localization')
    nomotion.wait_for_service(5)
    reinit.wait_for_service(5)
    
    reinit.call_async(Empty.Request())
    time.sleep(1)
    
    started_stamp = node.get_clock().now().nanoseconds / 1e9
    print(f"started_stamp (node.get_clock().now()) = {started_stamp:.4f}")
    
    for i in range(5):
        nomotion.call_async(Empty.Request())
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
            s, p = scan[0], pose[0]
            if s and p:
                s_stamp = s.header.stamp.sec + s.header.stamp.nanosec / 1e9
                p_stamp = p.header.stamp.sec + p.header.stamp.nanosec / 1e9
                print(f"Iter {i} spin: started={started_stamp:.3f}, p_stamp={p_stamp:.3f}, s_stamp={s_stamp:.3f}")
                print(f"   p <= started: {p_stamp <= started_stamp}")
                print(f"   abs(p - s) > 0.5: {abs(p_stamp - s_stamp) > 0.5} (diff={abs(p_stamp - s_stamp):.4f})")
                break
    
    node.destroy_node()
    rclpy.shutdown()
    for proc in [loc, sim]:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=4)
        except:
            pass

if __name__ == '__main__':
    main()
