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
sim = subprocess.Popen([
    "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
    f"world:={WORLD_SDF}", "spawn_x:=-7.0", "spawn_y:=0.0", "spawn_yaw:=0.0",
    "headless:=true", "rviz:=false"
], start_new_session=True)
time.sleep(10)
mapping = subprocess.Popen([
    "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
    "use_sim_time:=true"
], start_new_session=True)
time.sleep(12)

t0 = time.monotonic()
while time.monotonic() - t0 < 15.0:
    rclpy.spin_once(n, timeout_sec=0.2)
    if n.grid:
        g = n.grid
        info = g.info
        c100 = [i for i, v in enumerate(g.data) if v == 100]
        c99 = [i for i, v in enumerate(g.data) if v == 99]
        print(f"Total cells: {len(g.data)}, count(100)={len(c100)}, count(99)={len(c99)}")
        # Check first 5 100 cells
        for idx in c100[:5]:
            gx = idx % info.width
            gy = idx // info.width
            wx = info.origin.position.x + gx * info.resolution
            wy = info.origin.position.y + gy * info.resolution
            print(f"Cost 100 at grid ({gx}, {gy}) -> map ({wx:.3f}, {wy:.3f})")
        break

stop_process(mapping)
stop_process(sim)
n.destroy_node()
rclpy.shutdown()
