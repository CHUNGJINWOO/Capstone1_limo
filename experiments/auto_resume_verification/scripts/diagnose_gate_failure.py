#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
import subprocess, time, os, signal, math

from limo_auto_mapping.localization import MapMatcher, planar_pose, covariance_ok, close_pose
from nav_msgs.msg import OccupancyGrid

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
    node = Node('gate_diag')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    scan = [None]
    pose = [None]
    grid = [None]
    node.create_subscription(LaserScan, '/scan', lambda m: scan.__setitem__(0, m), 10)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: pose.__setitem__(0, m), 10)
    node.create_subscription(OccupancyGrid, '/map', lambda m: grid.__setitem__(0, m), rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
    
    nomotion = node.create_client(Empty, '/request_nomotion_update')
    reinit = node.create_client(Empty, '/reinitialize_global_localization')
    nomotion.wait_for_service(5)
    reinit.wait_for_service(5)
    
    print("Waiting for map...")
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if grid[0]:
            break
            
    matcher = MapMatcher(grid[0], wall_tolerance=0.15, clearance=0.24)
    print("Reinitializing global...")
    reinit.call_async(Empty.Request())
    time.sleep(1)
    
    for step in range(1, 55):
        nomotion.call_async(Empty.Request())
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
            
        s, p = scan[0], pose[0]
        if s and p:
            cov = p.pose.covariance
            c_ok = covariance_ok(cov, 0.20, 0.15)
            std_x = math.sqrt(max(0, cov[0]))
            std_y = math.sqrt(max(0, cov[7]))
            std_yaw = math.sqrt(max(0, cov[35]))
            
            pl_pose = planar_pose(p.pose.pose)
            
            # Check TF
            tf_ok = False
            tf_pose = None
            try:
                t = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                tf_p = planar_pose(t.transform)
                tf_ok = close_pose(pl_pose, tf_p)
                tf_pose = tf_p
            except:
                pass
                
            # Check scan matches
            sm_res = None
            sm_pass = False
            laser = (0.0, 0.0, 0.0)
            try:
                lt = tf_buffer.lookup_transform('base_link', s.header.frame_id, rclpy.time.Time())
                laser = planar_pose(lt.transform)
            except:
                pass
                
            refined = matcher.refine_pose(pl_pose, s, laser)
            res = matcher.score(refined, s, laser)
            sm_pass = (res.beams >= 30 and res.hit_ratio >= 0.75 and res.known_ratio >= 0.80 and res.blocked_ratio <= 0.05 and res.sectors >= 3)
            
            print(f"Step {step:2d} pose=({pl_pose[0]:.2f}, {pl_pose[1]:.2f}, {pl_pose[2]:.2f}) | std=({std_x:.3f}, {std_y:.3f}, {std_yaw:.3f}) cov_ok={c_ok} | tf_ok={tf_ok} | sm_pass={sm_pass} (hit={res.hit_ratio:.2f}, known={res.known_ratio:.2f}, blk={res.blocked_ratio:.2f}, sec={res.sectors})")
            if c_ok and tf_ok and sm_pass:
                print(f"*** ALL CHECKS PASSED AT STEP {step}! ***")
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
