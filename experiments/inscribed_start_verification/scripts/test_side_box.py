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

def spawn_box(name, x, y, z=0.5, sx=0.4, sy=0.4, sz=1.0):
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

mapping = subprocess.Popen([
    "ros2", "launch", "limo_auto_mapping", "auto_mapping.launch.py",
    "use_sim_time:=true"
], start_new_session=True)
time.sleep(12)

print("Waiting for TF and costmap...", flush=True)
while True:
    rclpy.spin_once(n, timeout_sec=0.2)
    if n.grid:
        c = n.get_center_cost()
        if c is not None:
            print(f"Initial center cost: {c}", flush=True)
            break
    time.sleep(0.2)

# Spawn box at x = -6.75, y = 0.35 (sx=0.4, sy=0.4, sz=1.0)
# South face is at y = 0.35 - 0.20 = 0.15m (< 0.19m robot_radius)
# Distance from laser (-6.897, 0) to south edge is ~0.25m (> 0.20m min range)
print("Spawning box at x=-6.75, y=0.35...", flush=True)
spawn_box('dyn_box', -6.75, 0.35, sx=0.4, sy=0.4, sz=1.0)

t0 = time.monotonic()
cost_with_box = None
while time.monotonic() - t0 < 8.0:
    rclpy.spin_once(n, timeout_sec=0.2)
    c = n.get_center_cost()
    if c is not None:
        cost_with_box = c
        # Also inspect max cost within 10 cells of robot
        tf = n.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        rx, ry = tf.transform.translation.x, tf.transform.translation.y
        info = n.grid.info
        gx = int(round((rx - info.origin.position.x) / info.resolution))
        gy = int(round((ry - info.origin.position.y) / info.resolution))
        nearby_costs = []
        for dy in range(-8, 9):
            row = []
            for dx in range(-8, 9):
                idx = (gy + dy) * info.width + (gx + dx)
                val = n.grid.data[idx] if 0 <= idx < len(n.grid.data) else -1
                row.append(val)
            nearby_costs.append(row)
        max_cost = max(max(r) for r in nearby_costs)
        print(f"t={time.monotonic()-t0:.1f}s | Center cost: {c} | Max nearby (within 8 cells / 0.4m): {max_cost}", flush=True)
        if c >= 99:
            break
    time.sleep(0.5)

print("Nearby patch at end of wait:")
for dy, row in zip(range(-8, 9), nearby_costs):
    print(f"dy={dy:+2d}: {[v for v in row]}", flush=True)

print("Removing box...", flush=True)
remove_box('dyn_box')

t1 = time.monotonic()
cost_cleared = None
while time.monotonic() - t1 < 8.0:
    rclpy.spin_once(n, timeout_sec=0.2)
    if n.grid:
        c = n.get_center_cost()
        if c is not None:
            cost_cleared = c
            print(f"t={time.monotonic()-t1:.1f}s | After remove cost: {c}", flush=True)
            if c < 99:
                print("SUCCESSFULLY CLEARED!", flush=True)
                break
    time.sleep(0.5)

stop_process(mapping)
stop_process(sim)
n.destroy_node()
rclpy.shutdown()
