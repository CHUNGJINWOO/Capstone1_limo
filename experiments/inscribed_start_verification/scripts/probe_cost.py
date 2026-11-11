#!/usr/bin/env python3
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

WORLD_SDF = Path('/home/cjinw/school/2026-2/Capstone1_limo/output/simulation/complex_indoor_env/complex_indoor_stageA.sdf')

def stop_process(proc):
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=2)
        except Exception:
            pass

class ProbeNode(Node):
    def __init__(self):
        super().__init__('probe_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.costmap = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, 1)

    def on_costmap(self, msg):
        self.costmap = msg

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            t = tf.transform.translation
            return (t.x, t.y)
        except Exception:
            return None

def probe(spawn_x, spawn_y=0.0):
    rclpy.init()
    node = ProbeNode()
    print(f"Testing spawn_x = {spawn_x}")
    sim = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD_SDF}", f"spawn_x:={spawn_x}", f"spawn_y:={spawn_y}", "spawn_yaw:=0.0",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(10)

    mapping = subprocess.Popen([
        "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
        "use_sim_time:=true"
    ], start_new_session=True)
    time.sleep(12)

    t0 = time.monotonic()
    center_cost = None
    robot_pose = None
    cell_info = None
    while time.monotonic() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        robot_pose = node.get_robot_pose()
        if robot_pose and node.costmap:
            info = node.costmap.info
            gx = int(round((robot_pose[0] - info.origin.position.x) / info.resolution))
            gy = int(round((robot_pose[1] - info.origin.position.y) / info.resolution))
            if 0 <= gx < info.width and 0 <= gy < info.height:
                center_cost = node.costmap.data[gy * info.width + gx]
                cell_info = (gx, gy, info.resolution, info.origin.position.x, info.origin.position.y)
                break

    print(f"Result for spawn_x={spawn_x}: pose={robot_pose}, center_cost={center_cost}, cell_info={cell_info}")
    stop_process(mapping)
    stop_process(sim)
    node.destroy_node()
    rclpy.shutdown()
    time.sleep(3)
    return center_cost

if __name__ == '__main__':
    sx = float(sys.argv[1]) if len(sys.argv) > 1 else -7.75
    probe(sx)
