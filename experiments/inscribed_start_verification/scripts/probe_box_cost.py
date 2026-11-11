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

def spawn_box(name, x, y, z=0.2, sx=0.2, sy=0.2, sz=0.4):
    sdf = (
        f"<sdf version='1.6'>"
        f"<model name='{name}'>"
        f"<static>true</static>"
        f"<link name='link'>"
        f"<visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></visual>"
        f"<collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>"
        f"</link>"
        f"</model>"
        f"</sdf>"
    )
    req = f'sdf: "{sdf}", pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}'
    cmd = [
        'ign', 'service', '-s', '/world/mc3f_world/create',
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '3000',
        '--req', req
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    return res.returncode == 0 and 'data: true' in res.stdout

def remove_box(name):
    req = f'name: "{name}", type: 2'
    cmd = [
        'ign', 'service', '-s', '/world/mc3f_world/remove',
        '--reqtype', 'ignition.msgs.Entity',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '3000',
        '--req', req
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    return res.returncode == 0 and 'data: true' in res.stdout

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

def test_box_cost(box_rel_x):
    rclpy.init()
    node = ProbeNode()
    robot_spawn_x = -7.0
    sim = subprocess.Popen([
        "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
        f"world:={WORLD_SDF}", f"spawn_x:={robot_spawn_x}", f"spawn_y:=0.0", "spawn_yaw:=0.0",
        "headless:=true", "rviz:=false"
    ], start_new_session=True)
    time.sleep(10)

    # Spawn box relative to robot
    box_x = robot_spawn_x + box_rel_x
    print(f"Spawning box at x={box_x}, y=0.0 (rel_x={box_rel_x})")
    spawn_ok = spawn_box('test_box', box_x, 0.0, z=0.2, sx=0.2, sy=0.2, sz=0.4)
    print(f"Spawn box result: {spawn_ok}")
    time.sleep(2)

    mapping = subprocess.Popen([
        "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
        "use_sim_time:=true"
    ], start_new_session=True)
    time.sleep(12)

    t0 = time.monotonic()
    center_cost = None
    robot_pose = None
    surrounding = {}
    while time.monotonic() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        robot_pose = node.get_robot_pose()
        if robot_pose and node.costmap:
            info = node.costmap.info
            gx = int(round((robot_pose[0] - info.origin.position.x) / info.resolution))
            gy = int(round((robot_pose[1] - info.origin.position.y) / info.resolution))
            if 0 <= gx < info.width and 0 <= gy < info.height:
                center_cost = node.costmap.data[gy * info.width + gx]
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < info.width and 0 <= ny < info.height:
                            surrounding[(dx, dy)] = node.costmap.data[ny * info.width + nx]
                break

    print(f"Result with box_rel_x={box_rel_x}: pose={robot_pose}, center_cost={center_cost}, surrounding={surrounding}")
    remove_box('test_box')
    stop_process(mapping)
    stop_process(sim)
    node.destroy_node()
    rclpy.shutdown()
    time.sleep(3)

if __name__ == '__main__':
    rel_x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
    test_box_cost(rel_x)
