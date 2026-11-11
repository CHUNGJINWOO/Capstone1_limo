import os
import sys
import time
import subprocess
import signal
import pathlib
import numpy as np
from PIL import Image

WORLD_SDF = "/home/cjinw/school/2026-2/Capstone1_limo/experiments/gazebo_obstacle_mapping/worlds/c103_obstacle_exploration.sdf"

print("[Test] Launching Gazebo...")
gz_cmd = (
    "source /opt/ros/humble/setup.bash && "
    "source /home/cjinw/school/2026-2/Capstone1_limo/capstone_ws/install/setup.bash && "
    f"ros2 launch limo_capstone limo_mc3f_sim.launch.py world:={WORLD_SDF} spawn_x:=2.0 spawn_y:=-1.1 spawn_z:=0.05 spawn_yaw:=3.14159 headless:=true rviz:=false"
)
gz_proc = subprocess.Popen(["bash", "-c", gz_cmd], preexec_fn=os.setsid)

time.sleep(10)

print("[Test] Launching camera bridge...")
bridge_cmd = "source /opt/ros/humble/setup.bash && ros2 run ros_gz_bridge parameter_bridge /demo/overhead@sensor_msgs/msg/Image[ignition.msgs.Image"
bridge_proc = subprocess.Popen(["bash", "-c", bridge_cmd], preexec_fn=os.setsid)

time.sleep(4)

print("[Test] Capturing 1 image frame...")
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg

rclpy.init()
node = Node("camera_test_node")
frame_data = []

def cb(msg):
    frame_data.append(msg)

sub = node.create_subscription(ImageMsg, "/demo/overhead", cb, 1)

t0 = time.time()
while time.time() - t0 < 10.0 and not frame_data:
    rclpy.spin_once(node, timeout_sec=0.2)

rclpy.shutdown()

os.killpg(os.getpgid(bridge_proc.pid), signal.SIGINT)
os.killpg(os.getpgid(gz_proc.pid), signal.SIGINT)

if frame_data:
    msg = frame_data[0]
    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    img = Image.fromarray(arr)
    img.save("/tmp/c103_test_view.png")
    print(f"[Test] SUCCESS: Captured image {msg.width}x{msg.height} to /tmp/c103_test_view.png")
else:
    print("[Test] FAILED: No image received")
