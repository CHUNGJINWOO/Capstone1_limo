import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import time, subprocess, os, signal
from pathlib import Path
from tf2_ros import Buffer, TransformListener

WORLD_SDF = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf')

def stop_process(proc):
    if proc is None: return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=2)
        except Exception: pass

rclpy.init()
class N(Node):
    def __init__(self):
        super().__init__('n')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.grid = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: setattr(self, 'grid', m), 1)

n = N()
# Spawn at x = -7.70, yaw = 0.0
sim = subprocess.Popen([
    "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
    f"world:={WORLD_SDF}", "spawn_x:=-7.70", "spawn_y:=0.0", "spawn_yaw:=0.0",
    "headless:=true", "rviz:=false"
], start_new_session=True)
time.sleep(10)
mapping = subprocess.Popen([
    "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
    "use_sim_time:=true"
], start_new_session=True)
time.sleep(12)

t0 = time.monotonic()
while time.monotonic() - t0 < 25.0:
    rclpy.spin_once(n, timeout_sec=0.2)
    if n.grid:
        g = n.grid
        info = g.info
        print(f"Costmap info: width={info.width}, height={info.height}, res={info.resolution}, origin=({info.origin.position.x:.3f}, {info.origin.position.y:.3f})")
        # Find distribution of costs
        from collections import Counter
        counts = Counter(g.data)
        print("Cost distribution:", sorted(counts.items()))
        
        # Check robot cell
        try:
            tf = n.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx, ry = tf.transform.translation.x, tf.transform.translation.y
            gx = int(round((rx - info.origin.position.x) / info.resolution))
            gy = int(round((ry - info.origin.position.y) / info.resolution))
            print(f"Robot map pose: ({rx:.4f}, {ry:.4f}) -> grid: ({gx}, {gy}) -> cost: {g.data[gy * info.width + gx]}")
            # Print a small 7x7 patch around robot
            print("Patch around robot (y from gy+3 down to gy-3):")
            for y in range(gy+3, gy-4, -1):
                row = [f"{g.data[y * info.width + x]:3d}" for x in range(gx-3, gx+4)]
                print(f"y={y:3d} (rel {y-gy:+2d}): " + " ".join(row))
            break
        except Exception as e:
            print(f"TF exception: {e}")

stop_process(mapping)
stop_process(sim)
n.destroy_node()
rclpy.shutdown()
