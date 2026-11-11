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

def spawn_box(name, x, y, z=0.2, sx=0.1, sy=0.1, sz=0.3):
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

    def get_center_cost(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx, ry = tf.transform.translation.x, tf.transform.translation.y
            info = self.grid.info
            gx = int(round((rx - info.origin.position.x) / info.resolution))
            gy = int(round((ry - info.origin.position.y) / info.resolution))
            return self.grid.data[gy * info.width + gx]
        except Exception:
            return None

n = N()
sim = subprocess.Popen([
    "ros2", "launch", "limo_capstone", "limo_mc3f_sim.launch.py",
    f"world:={WORLD_SDF}", "spawn_x:=-7.0", "spawn_y:=0.0", "spawn_yaw:=0.0",
    "headless:=true", "rviz:=false"
], start_new_session=True)
time.sleep(10)

# Launch mapping first so SLAM initial map is clean
mapping = subprocess.Popen([
    "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
    "use_sim_time:=true"
], start_new_session=True)
time.sleep(12)

# NOW spawn dynamic box at (-6.99, 0.18)
print("Spawning dynamic box at (-6.99, 0.18)...")
spawn_box('dyn_box', -6.99, 0.18, z=0.2, sx=0.1, sy=0.1, sz=0.3)

# Wait for costmap to reflect cost >= 99
t1 = time.monotonic()
cost_with_box = None
while time.monotonic() - t1 < 10.0:
    rclpy.spin_once(n, timeout_sec=0.2)
    c = n.get_center_cost()
    if c is not None and c >= 99:
        cost_with_box = c
        print(f"Cost WITH dynamic box: {c} (t={time.monotonic()-t1:.2f}s)")
        break

# Wait 3s
time.sleep(3)

# Remove dynamic box
print("Removing dynamic box...")
remove_box('dyn_box')

# Wait for costmap to clear to < 99
t2 = time.monotonic()
cost_after_remove = None
while time.monotonic() - t2 < 10.0:
    rclpy.spin_once(n, timeout_sec=0.2)
    c = n.get_center_cost()
    if c is not None and c < 99:
        cost_after_remove = c
        print(f"Cost AFTER remove: {c} (took {time.monotonic()-t2:.2f}s)")
        break

stop_process(mapping)
stop_process(sim)
n.destroy_node()
rclpy.shutdown()
