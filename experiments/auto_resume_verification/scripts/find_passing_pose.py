import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
import subprocess, time, os, signal, math
from limo_auto_mapping.localization import MapMatcher, planar_pose, covariance_ok, close_pose

EXP_DIR = "/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification"
MAP = f"{EXP_DIR}/maps/complex_indoor_baseline.yaml"
WORLD = "/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf"

CANDIDATES = [
    ("Dock", -6.2, -2.0, 0.0),
    ("Dock_Forward", -5.5, -2.0, 0.0),
    ("Central_Junction", -3.5, -2.0, 0.0),
    ("West_Avenue_Mid", -3.5, 0.0, 1.57),
    ("North_Loop_Turn", -5.0, 3.5, 3.14),
]

def test_candidate(name, sx, sy, syaw):
    print(f"\nTesting Candidate: {name} at ({sx}, {sy}, {syaw})...")
    sim = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD}", f"spawn_x:={sx}", f"spawn_y:={sy}", f"spawn_yaw:={syaw}",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(5)
    
    loc = subprocess.Popen([
        "ros2", "launch", "nav2_bringup", "localization_launch.py",
        f"map:={MAP}", "use_sim_time:=true", "autostart:=true"
    ], start_new_session=True)
    time.sleep(5)

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node('candidate_tester')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    scan = [None]
    pose = [None]
    grid = [None]
    latched = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(LaserScan, '/scan', lambda m: scan.__setitem__(0, m), 10)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: pose.__setitem__(0, m), 10)
    node.create_subscription(OccupancyGrid, '/map', lambda m: grid.__setitem__(0, m), latched)
    
    nomotion = node.create_client(Empty, '/request_nomotion_update')
    reinit = node.create_client(Empty, '/reinitialize_global_localization')
    nomotion.wait_for_service(5)
    reinit.wait_for_service(5)
    
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if grid[0]: break
        
    matcher = MapMatcher(grid[0], wall_tolerance=0.15, clearance=0.24)
    reinit.call_async(Empty.Request())
    time.sleep(1)
    
    passed = False
    best_hit = 0.0
    final_step = 0
    truth = (sx + 6.2, sy + 2.0, syaw)
    
    for step in range(1, 45):
        nomotion.call_async(Empty.Request())
        t_end = time.monotonic() + 0.9
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)
        s, p = scan[0], pose[0]
        if s and p:
            cov = p.pose.covariance
            c_ok = covariance_ok(cov, 0.20, 0.15)
            pl_pose = planar_pose(p.pose.pose)
            from geometry_msgs.msg import Pose
            laser = (0.103, 0.0, 0.0)
            try:
                lt = tf_buffer.lookup_transform('base_link', s.header.frame_id, rclpy.time.Time())
                p_msg = Pose()
                p_msg.position.x = lt.transform.translation.x
                p_msg.position.y = lt.transform.translation.y
                p_msg.position.z = lt.transform.translation.z
                p_msg.orientation = lt.transform.rotation
                laser = planar_pose(p_msg)
            except:
                pass
            refined = matcher.refine_pose(pl_pose, s, laser)
            res = matcher.score(refined, s, laser)
            if res.hit_ratio > best_hit: best_hit = res.hit_ratio
            sm_pass = (res.beams >= 30 and res.hit_ratio >= 0.75 and res.known_ratio >= 0.80 and res.blocked_ratio <= 0.05 and res.sectors >= 3)
            pos_err = math.hypot(pl_pose[0] - truth[0], pl_pose[1] - truth[1])
            if c_ok and sm_pass:
                print(f"  Step {step}: SUCCESS! pose=({pl_pose[0]:.2f}, {pl_pose[1]:.2f}) pos_err={pos_err:.2f}m hit={res.hit_ratio:.2f}")
                passed = True
                final_step = step
                break
            if step % 5 == 0 or step == 1:
                print(f"  Step {step}: pose=({pl_pose[0]:.2f}, {pl_pose[1]:.2f}) truth=({truth[0]:.2f}, {truth[1]:.2f}) err={pos_err:.2f}m cov_ok={c_ok} hit={res.hit_ratio:.2f} blk={res.blocked_ratio:.2f} sec={res.sectors}")

    node.destroy_node()
    rclpy.shutdown()
    for proc in [loc, sim]:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=4)
        except:
            pass
    return passed, best_hit

def main():
    for name, sx, sy, syaw in CANDIDATES:
        passed, hit = test_candidate(name, sx, sy, syaw)
        if passed:
            print(f"\n===> FOUND WORKING LOCATION: {name} (sx={sx}, sy={sy}, syaw={syaw})! <===")
            break
        else:
            print(f"  Result: Failed (best_hit={hit:.2f})")

if __name__ == '__main__':
    main()
