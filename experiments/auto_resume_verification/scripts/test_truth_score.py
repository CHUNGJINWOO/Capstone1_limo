import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import subprocess, time, os, signal, math
from limo_auto_mapping.localization import MapMatcher, planar_pose

EXP_DIR = "/home/cjinw/school/2026-2/Capstone1_limo/experiments/auto_resume_verification"
MAP = f"{EXP_DIR}/maps/complex_indoor_baseline.yaml"
WORLD = "/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf"

def main():
    sx, sy, syaw = -6.2, -2.0, 0.0
    truth = (sx + 6.2, sy + 2.0, syaw) # (0.0, 0.0, 0.0)
    
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
    node = Node('truth_score_tester')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    scan = [None]
    grid = [None]
    latched = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(LaserScan, '/scan', lambda m: scan.__setitem__(0, m), 10)
    node.create_subscription(OccupancyGrid, '/map', lambda m: grid.__setitem__(0, m), latched)
    
    t0 = time.monotonic()
    while time.monotonic() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if grid[0] is not None and scan[0] is not None:
            break
        
    if grid[0] is None or scan[0] is None:
        print(f"Failed to receive scan or map: grid={grid[0] is not None}, scan={scan[0] is not None}")
        return

    matcher = MapMatcher(grid[0], wall_tolerance=0.15, clearance=0.24)
    s = scan[0]
    
    from geometry_msgs.msg import Pose
    laser = (0.0, 0.0, 0.0)
    for _ in range(10):
        try:
            lt = tf_buffer.lookup_transform('base_link', s.header.frame_id, rclpy.time.Time())
            p = Pose()
            p.position.x = lt.transform.translation.x
            p.position.y = lt.transform.translation.y
            p.position.z = lt.transform.translation.z
            p.orientation = lt.transform.rotation
            laser = planar_pose(p)
            break
        except Exception as exc:
            print(f"TF lookup error: {exc}")
            time.sleep(0.5)
            rclpy.spin_once(node, timeout_sec=0.1)
    print(f"Laser TF: {laser}")
        
    res = matcher.score(truth, s, laser)
    print(f"Ground Truth Score: hit_ratio={res.hit_ratio:.3f}, known_ratio={res.known_ratio:.3f}, blocked_ratio={res.blocked_ratio:.3f}, sectors={res.sectors}, beams={res.beams}")

    # Detailed ray trace analysis
    ct, st = math.cos(truth[2]), math.sin(truth[2])
    sx = truth[0] + ct*laser[0] - st*laser[1]
    sy = truth[1] + st*laser[0] + ct*laser[1]
    stride = max(1, len(s.ranges)//120)
    blocked_count = 0
    hit_count = 0
    total_valid = 0
    from limo_auto_mapping.policy import cost
    for i in range(0, len(s.ranges), stride):
        r = s.ranges[i]
        if not math.isfinite(r) or not max(s.range_min, 0.10) < r < s.range_max:
            continue
        total_valid += 1
        rel = laser[2] + s.angle_min + i * s.angle_increment
        th = truth[2] + rel
        dx, dy = math.cos(th), math.sin(th)
        hit = matcher.near_wall(sx + r*dx, sy + r*dy)
        if hit: hit_count += 1
        
        # Check blocking
        samples = max(1, math.ceil((r - matcher.tolerance) / max(0.10, grid[0].info.resolution)))
        ray_blocked = False
        blocked_dists = []
        for j in range(samples):
            dist = j * (r - matcher.tolerance) / samples
            val = cost(grid[0], (sx + dist*dx, sy + dist*dy))
            if val is not None and val >= 50:
                ray_blocked = True
                blocked_dists.append((dist, val))
        if ray_blocked:
            blocked_count += 1
            if blocked_count <= 5:
                print(f"  Blocked Ray {i}: r={r:.2f} angle={math.degrees(rel):.1f}deg hits={blocked_dists[:2]}")
    print(f"Total valid beams: {total_valid}, hits: {hit_count}, blocked: {blocked_count}")

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
