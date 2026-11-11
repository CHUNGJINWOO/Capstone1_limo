import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
import subprocess, time, os, signal

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
    
    scans, poses = [], []
    node.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), 10)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: poses.append(m), 10)
    
    cli = node.create_client(Empty, '/request_nomotion_update')
    reinit = node.create_client(Empty, '/reinitialize_global_localization')
    cli.wait_for_service(5)
    reinit.wait_for_service(5)
    reinit.call_async(Empty.Request())
    
    try:
        for i in range(10):
            cli.call_async(Empty.Request())
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
            now = node.get_clock().now().nanoseconds / 1e9
            s_stamp = (scans[-1].header.stamp.sec + scans[-1].header.stamp.nanosec/1e9) if scans else None
            p_stamp = (poses[-1].header.stamp.sec + poses[-1].header.stamp.nanosec/1e9) if poses else None
            cov = poses[-1].pose.covariance if poses else None
            print(f"Step {i}: now={now:.3f}, scan={s_stamp}, pose={p_stamp}")
            if s_stamp and p_stamp:
                print(f"   diff(pose - scan) = {p_stamp - s_stamp:.4f}, diff(now - pose) = {now - p_stamp:.4f}")
                print(f"   cov[0]={cov[0]:.4f}, cov[7]={cov[7]:.4f}, cov[35]={cov[35]:.4f}")
            time.sleep(1)
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
