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

def spawn_box(name, x, y, z=0.2, sx=0.1, sy=0.2, sz=0.3):
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

# Robot at x = -7.0, LiDAR at -6.90.
# Box at x = -6.65 (dist from LiDAR = 0.25m >= min_range 0.20m, dist from robot center = 0.35m)
# Wait: if dist from robot center is 0.35m, is it in inflation? inflation_radius is 0.30m, robot_radius is 0.19m!
# Box front face at x = -6.70 (box size sx=0.1, centered at -6.65, rear face at -6.70).
# Distance from robot center (-7.0) to rear face of box (-6.70) is 0.30m.
# What if box is at x = -6.70, rear face at -6.75?
# Distance from LiDAR (-6.90) to box rear face (-6.75) is 0.15m (below 0.20m min_range)!
# What if the box is placed at the side (e.g. y = +0.25, in front-left where FOV is clear and dist >= 0.20m)?
# At y = 0.25, dist = 0.25m < inflation_radius (0.30m)!
# What if y = 0.18? Distance from robot center is 0.18m < robot_radius (0.19m)!
# Distance from LiDAR (0.103, 0.0) to (0.0, 0.18):
# hypot(0.103, 0.18) = sqrt(0.0106 + 0.0324) = sqrt(0.043) = 0.207m!
# 0.207m >= 0.20m (EXACTLY above LiDAR min range 0.20m)!
# And distance to robot center is 0.18m <= 0.19m (robot_radius)!
# So cost at robot center is 99!
# And because it is at y = 0.18 (angle = 60 deg, well within +-120 deg FOV):
# When the box is removed, the laser ray at 60 deg shoots across the room and raytrace-clears the cell!
