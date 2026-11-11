#!/usr/bin/env python3
"""
test_exploration_run.py:
Verifies that c103_obstacle_exploration world runs with SLAM Toolbox + Nav2 + AutoMapping,
and that the robot moves and expands the map.
"""
import os
import sys
import time
import math
import signal
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image as ImageMsg
from tf2_ros import Buffer, TransformListener
from limo_mapping_interfaces.action import AutoMapping

WORLD_SDF = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping/worlds/c103_obstacle_exploration.sdf')
WS_DIR = Path('/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping/ws')

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

class ExplorationTestNode(Node):
    def __init__(self):
        super().__init__('test_exploration_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_map = None
        self.latest_costmap = None
        self.latest_cam = None
        self.odom_start = None
        self.odom_cur = None
        self.cmd_vel_count = 0
        
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 1)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, 1)
        self.create_subscription(ImageMsg, '/demo/overhead', self.on_cam, 1)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.auto_client = ActionClient(self, AutoMapping, '/auto_mapping')

    def on_map(self, msg): self.latest_map = msg
    def on_costmap(self, msg): self.latest_costmap = msg
    def on_cam(self, msg): self.latest_cam = msg
    def on_odom(self, msg):
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.odom_start is None: self.odom_start = pos
        self.odom_cur = pos
    def on_cmd_vel(self, msg):
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.cmd_vel_count += 1

    @property
    def distance_traveled(self):
        if self.odom_start is None or self.odom_cur is None: return 0.0
        return math.hypot(self.odom_cur[0] - self.odom_start[0], self.odom_cur[1] - self.odom_start[1])

def main():
    print("[1] Launching Gazebo...")
    gz_cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash && "
        f"ros2 launch limo_capstone limo_mc3f_sim.launch.py "
        f"world:={WORLD_SDF} spawn_x:=-3.0 spawn_y:=-1.1 spawn_z:=0.05 spawn_yaw:=0.0 "
        f"headless:=true rviz:=false"
    )
    gz_proc = subprocess.Popen(["bash", "-c", gz_cmd], preexec_fn=os.setsid)
    time.sleep(10)

    print("[2] Launching Camera Bridge...")
    bridge_cmd = "source /opt/ros/humble/setup.bash && ros2 run ros_gz_bridge parameter_bridge /demo/overhead@sensor_msgs/msg/Image[ignition.msgs.Image"
    bridge_proc = subprocess.Popen(["bash", "-c", bridge_cmd], preexec_fn=os.setsid)
    time.sleep(3)

    print("[3] Launching AutoMapping stack...")
    map_cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash && "
        f"source {WS_DIR}/install/setup.bash && "
        "ros2 launch limo_auto_mapping auto_mapping.launch.py use_sim_time:=true"
    )
    map_proc = subprocess.Popen(["bash", "-c", map_cmd], preexec_fn=os.setsid)
    time.sleep(12)

    rclpy.init()
    node = ExplorationTestNode()

    # Wait for map & costmap
    print("[4] Waiting for SLAM map and costmap...")
    t0 = time.time()
    while time.time() - t0 < 30.0 and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.latest_map is not None and node.latest_costmap is not None and node.latest_cam is not None:
            break

    print(f"Map received: {node.latest_map is not None}, Costmap: {node.latest_costmap is not None}, Camera: {node.latest_cam is not None}")
    if node.latest_map:
        known = sum(1 for v in node.latest_map.data if v >= 0)
        total = len(node.latest_map.data)
        print(f"Initial map: {known}/{total} cells known ({known*node.latest_map.info.resolution**2:.2f} m^2)")

    print("[5] Sending AutoMapping action goal...")
    if not node.auto_client.wait_for_server(timeout_sec=10.0):
        print("ERROR: Action server not ready")
        stop_process(map_proc); stop_process(bridge_proc); stop_process(gz_proc)
        return

    goal = AutoMapping.Goal()
    goal.output_prefix = "/tmp/test_map"
    goal.max_duration_sec = 40.0
    goal.allow_margin_retry = False
    send_fut = node.auto_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_fut, timeout_sec=10.0)
    handle = send_fut.result()
    print(f"Goal accepted: {handle.accepted if handle else False}")

    print("[6] Monitoring driving and map expansion for 30s...")
    t_start = time.time()
    while time.time() - t_start < 30.0 and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if int(time.time() - t_start) % 5 == 0:
            known = sum(1 for v in node.latest_map.data if v >= 0) if node.latest_map else 0
            res = node.latest_map.info.resolution if node.latest_map else 0.05
            print(f"t={time.time()-t_start:.1f}s: dist={node.distance_traveled:.3f}m, cmd_vel={node.cmd_vel_count}, map_area={known*res**2:.2f}m^2")

    print("[7] Final Check:")
    print(f"Final distance traveled: {node.distance_traveled:.3f}m")
    print(f"Final cmd_vel counts: {node.cmd_vel_count}")
    if node.latest_map:
        known = sum(1 for v in node.latest_map.data if v >= 0)
        res = node.latest_map.info.resolution
        print(f"Final mapped area: {known*res**2:.2f} m^2")

    rclpy.shutdown()
    stop_process(map_proc)
    stop_process(bridge_proc)
    stop_process(gz_proc)
    print("Done!")

if __name__ == '__main__':
    main()
