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

def check(spawn_x):
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
        f"world:={WORLD_SDF}", f"spawn_x:={spawn_x}", "spawn_y:=0.0", "spawn_yaw:=0.0",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(10)
    mapping = subprocess.Popen([
        "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
        "use_sim_time:=true"
    ], start_new_session=True)
    time.sleep(12)

    t0 = time.monotonic()
    cost = None
    while time.monotonic() - t0 < 15.0:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.grid:
            try:
                tf = n.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                rx, ry = tf.transform.translation.x, tf.transform.translation.y
                info = n.grid.info
                gx = int(round((rx - info.origin.position.x) / info.resolution))
                gy = int(round((ry - info.origin.position.y) / info.resolution))
                if 0 <= gx < info.width and 0 <= gy < info.height:
                    cost = n.grid.data[gy * info.width + gx]
                    print(f"CHECK spawn_x={spawn_x}: rx={rx:.3f}, ry={ry:.3f}, gx={gx}, gy={gy}, cost={cost}", flush=True)
                    break
            except Exception:
                pass
    stop_process(mapping)
    stop_process(sim)
    n.destroy_node()
    rclpy.shutdown()
    time.sleep(3)
    return cost

if __name__ == '__main__':
    import sys
    check(float(sys.argv[1]))
