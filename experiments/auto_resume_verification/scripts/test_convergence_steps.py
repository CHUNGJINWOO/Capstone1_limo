#!/usr/bin/env python3
"""
Test AMCL global particle convergence over 60 seconds with stationary updates.
Inspects covariance[0], covariance[7], covariance[35], and estimated pose.
"""
import subprocess, time, signal, os, math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import yaml

def main():
    WORLD_SDF = "/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf"
    MAP_YAML = "/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification/maps/complex_indoor_baseline.yaml"
    NAV2_CONFIG = "/home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/src/limo_capstone/config/nav2_explore_params.yaml"

    cfg = yaml.safe_load(open(NAV2_CONFIG))
    cfg['amcl']['ros__parameters'].update({
        'use_sim_time': True,
        'set_initial_pose': False,
        'always_reset_initial_pose': True,
        'save_pose_rate': -1.0,
        'tf_broadcast': True,
        'max_particles': 20000,
        'min_particles': 2000,
        'update_min_d': 0.05,
        'update_min_a': 0.05
    })
    tmp_config = "/tmp/test_amcl_convergence.yaml"
    with open(tmp_config, 'w') as f:
        yaml.safe_dump(cfg, f)

    # Spawn robot at (-5.5, -2.0, 0.0) -> Map Truth: (0.70, 0.00, 0.00)
    print("Launching Gazebo...")
    sim_proc = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD_SDF}", "spawn_x:=-5.5", "spawn_y:=-2.0", "spawn_yaw:=0.0",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(5)

    print("Launching Nav2 localization...")
    loc_proc = subprocess.Popen([
        "ros2", "launch", "nav2_bringup", "localization_launch.py",
        f"map:={MAP_YAML}", f"params_file:={tmp_config}",
        "use_sim_time:=true", "autostart:=true"
    ], start_new_session=True)
    time.sleep(5)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node('convergence_tester')
    
    last_pose = [None]
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: last_pose.__setitem__(0, m), 1)
    
    nomotion_cli = node.create_client(Empty, '/request_nomotion_update')
    reinit_cli = node.create_client(Empty, '/reinitialize_global_localization')
    
    try:
        nomotion_cli.wait_for_service(timeout_sec=10)
        reinit_cli.wait_for_service(timeout_sec=10)
        
        print("Triggering global uniform reinitialization...")
        reinit_cli.call_async(Empty.Request())
        time.sleep(1)
        
        # Ground truth in map coordinates: (0.70, 0.00, 0.00)
        truth = (0.70, 0.00, 0.00)
        print(f"Tracking convergence towards Ground Truth: x={truth[0]:.2f}, y={truth[1]:.2f}, yaw={truth[2]:.2f}")
        print(f"{'Step':>4} | {'Pose X':>7} {'Pose Y':>7} {'Yaw':>7} | {'Err Pos':>7} {'Err Yaw':>7} | {'std_x':>6} {'std_y':>6} {'std_yaw':>7}")
        print("-" * 75)

        for step in range(1, 45):
            nomotion_cli.call_async(Empty.Request())
            # spin to receive update
            t_end = time.monotonic() + 0.8
            while time.monotonic() < t_end:
                rclpy.spin_once(node, timeout_sec=0.05)
            
            p = last_pose[0]
            if p is not None:
                q = p.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                px = p.pose.pose.position.x
                py = p.pose.pose.position.y
                err_pos = math.hypot(px - truth[0], py - truth[1])
                err_yaw = abs(math.atan2(math.sin(yaw - truth[2]), math.cos(yaw - truth[2])))
                cov = p.pose.covariance
                std_x = math.sqrt(max(0, cov[0]))
                std_y = math.sqrt(max(0, cov[7]))
                std_yaw = math.sqrt(max(0, cov[35]))
                print(f"{step:>4} | {px:7.3f} {py:7.3f} {yaw:7.3f} | {err_pos:7.3f} {err_yaw:7.3f} | {std_x:6.3f} {std_y:6.3f} {std_yaw:7.3f}")
                if std_x <= 0.20 and std_y <= 0.20 and std_yaw <= 0.15:
                    print(f"*** CONVERGED below max_xy_std=0.20, max_yaw_std=0.15 at step {step}! ***")
                    break
            else:
                print(f"{step:>4} | Waiting for pose...")
                
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(os.getpgid(loc_proc.pid), signal.SIGINT)
        os.killpg(os.getpgid(sim_proc.pid), signal.SIGINT)
        loc_proc.wait(timeout=5)
        sim_proc.wait(timeout=5)

if __name__ == '__main__':
    main()
