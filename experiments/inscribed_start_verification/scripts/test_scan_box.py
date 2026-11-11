import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import subprocess, time, math

rclpy.init()
class ScanCheck(Node):
    def __init__(self):
        super().__init__('scan_check')
        self.min_dist = 999.0
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
    def cb(self, msg):
        valid = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        if valid:
            self.min_dist = min(valid)

sc = ScanCheck()

# Check scan without box
t0 = time.monotonic()
while time.monotonic() - t0 < 3.0:
    rclpy.spin_once(sc, timeout_sec=0.1)

print(f"Current min scan dist without box: {sc.min_dist}")

# Spawn box at x=-6.5, y=0.0
sdf = "<sdf version='1.6'><model name='dyn_box'><static>true</static><link name='l'><collision name='c'><geometry><box><size>0.4 0.4 1.0</size></box></geometry></collision><visual name='v'><geometry><box><size>0.4 0.4 1.0</size></box></geometry></visual></link></model></sdf>"
req = f'sdf: "{sdf}", pose: {{position: {{x: -6.5, y: 0.0, z: 0.5}}}}'
cmd = ['ign', 'service', '-s', '/world/mc3f_world/create', '--reqtype', 'ignition.msgs.EntityFactory', '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', req]
r = subprocess.run(cmd, capture_output=True, text=True)
print(f"Spawn result: {r.stdout.strip()}")

time.sleep(1.0)
sc.min_dist = 999.0
t1 = time.monotonic()
while time.monotonic() - t1 < 3.0:
    rclpy.spin_once(sc, timeout_sec=0.1)

print(f"Min scan dist with box at -6.5: {sc.min_dist}")

# Remove box
cmd_rm = ['ign', 'service', '-s', '/world/mc3f_world/remove', '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', 'name: "dyn_box", type: 2']
r_rm = subprocess.run(cmd_rm, capture_output=True, text=True)
print(f"Remove result: {r_rm.stdout.strip()}")

sc.destroy_node()
rclpy.shutdown()
