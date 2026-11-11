#!/usr/bin/env python3
import subprocess, time, sys, os, signal
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
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

class NodeP(Node):
    def __init__(self):
        super().__init__('probe_profile')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.costmap = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: setattr(self, 'costmap', m), 1)

def run(spawn_x):
    rclpy.init()
    node = NodeP()
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
    while time.monotonic() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tf = node.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx, ry = tf.transform.translation.x, tf.transform.translation.y
            if node.costmap:
                info = node.costmap.info
                gx = int(round((rx - info.origin.position.x) / info.resolution))
                gy = int(round((ry - info.origin.position.y) / info.resolution))
                row = []
                for x in range(max(0, gx-4), min(info.width, gx+5)):
                    row.append((x - gx, node.costmap.data[gy * info.width + x]))
                print(f"SPAWN_X={spawn_x} | ROBOT=({rx:.4f}, {ry:.4f}) | GRID=({gx}, {gy}) | X-PROFILE (dx, cost): {row}", flush=True)
                break
        except Exception:
            pass

    stop_process(mapping)
    stop_process(sim)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run(float(sys.argv[1]))
