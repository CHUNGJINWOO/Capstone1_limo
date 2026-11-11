import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import subprocess, time, os, signal, math

def main():
    sim = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        "world:=/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf",
        "spawn_x:=-6.5", "spawn_y:=1.8", "spawn_yaw:=1.57", "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(5)
    
    loc = subprocess.Popen([
        "ros2", "launch", "nav2_bringup", "localization_launch.py",
        "map:=/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification/maps/complex_indoor_baseline.yaml",
        "use_sim_time:=true", "autostart:=true"
    ], start_new_session=True)
    time.sleep(5)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node('stamp_tester')
    
    poses = []
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: poses.append(m), 10)
    
    cli = node.create_client(Empty, '/request_nomotion_update')
    reinit = node.create_client(Empty, '/reinitialize_global_localization')
    cli.wait_for_service(5)
    reinit.wait_for_service(5)
    reinit.call_async(Empty.Request())
    time.sleep(1)
    
    try:
        print(f"{'Step':>4} | {'Cov[0]':>8} {'Cov[7]':>8} {'Cov[35]':>8} | {'std_x':>7} {'std_y':>7} {'std_yaw':>7}")
        print("-" * 65)
        for i in range(1, 61):
            cli.call_async(Empty.Request())
            t_end = time.monotonic() + 0.9
            while time.monotonic() < t_end:
                rclpy.spin_once(node, timeout_sec=0.05)
            if poses:
                p = poses[-1]
                cov = p.pose.covariance
                std_x = math.sqrt(max(0, cov[0]))
                std_y = math.sqrt(max(0, cov[7]))
                std_yaw = math.sqrt(max(0, cov[35]))
                print(f"{i:4d} | {cov[0]:8.4f} {cov[7]:8.4f} {cov[35]:8.4f} | {std_x:7.4f} {std_y:7.4f} {std_yaw:7.4f}")
                if std_x <= 0.20 and std_y <= 0.20 and std_yaw <= 0.15:
                    print(f"*** CONVERGED below threshold at step {i}! ***")
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for p in [loc, sim]:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
                p.wait(timeout=4)
            except:
                pass

if __name__ == '__main__':
    main()
