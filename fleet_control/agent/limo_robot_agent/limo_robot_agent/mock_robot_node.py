"""MOCK robot for the laptop (never used on the real LIMO). Provides what the agent reads from a navigation stack, clearly simulated:
  /map (from a saved PGM map), TF map->odom->base_footprint (moving pose), a navigate_to_pose ActionServer (straight-line motion, feedback, cancel),
  a synthetic /scan, a fake /limo_status battery voltage and a fake stall-gate status. It publishes NO depth status (the agent must show NO_DATA).
No velocity topic is published and no real hardware is touched."""
import json, math, os, threading, time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from PIL import Image
try:
    from limo_msgs.msg import LimoStatus
except Exception:
    LimoStatus = None


class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        d = self.declare_parameter
        d('map_yaml', ''); d('start_x', 0.0); d('start_y', 0.0); d('start_yaw', 0.0); d('speed_mps', 0.5); d('goal_tolerance_m', 0.15); d('goal_log', '')
        self.x, self.y, self.yaw = float(self.get_parameter('start_x').value), float(self.get_parameter('start_y').value), float(self.get_parameter('start_yaw').value)
        self.speed, self.tol = float(self.get_parameter('speed_mps').value), float(self.get_parameter('goal_tolerance_m').value)
        self.tfb = TransformBroadcaster(self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', qos)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pub_gate = self.create_publisher(String, '/stall_detector/status_json', 10)
        self.pub_limo = self.create_publisher(LimoStatus, '/limo_status', 10) if LimoStatus else None
        my = self.get_parameter('map_yaml').value
        if my: self._publish_map(my)
        self.create_timer(0.05, self._tf); self.create_timer(0.1, self._scan); self.create_timer(0.2, self._gate)
        self.act = ActionServer(self, NavigateToPose, 'navigate_to_pose', execute_callback=self._exec, goal_callback=self._goal_cb,
                                cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'mock robot up at ({self.x:.2f},{self.y:.2f}); map={my or "none"}')

    def _publish_map(self, yaml_path):
        meta = yaml.safe_load(open(yaml_path)); pgm = np.array(Image.open(os.path.join(os.path.dirname(yaml_path), meta['image'])))
        grid = np.full(pgm.shape, -1, np.int8); grid[pgm == 254] = 0; grid[pgm == 0] = 100; grid = np.flipud(grid)
        m = OccupancyGrid(); m.header.frame_id = 'map'; m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = float(meta['resolution']); m.info.width, m.info.height = pgm.shape[1], pgm.shape[0]
        m.info.origin.position.x, m.info.origin.position.y = float(meta['origin'][0]), float(meta['origin'][1]); m.info.origin.orientation.w = 1.0
        m.data = grid.flatten().tolist(); self.pub_map.publish(m)

    def _tf(self):
        now = self.get_clock().now().to_msg(); ts = []
        for p, c, x, y, yaw in (('map', 'odom', 0.0, 0.0, 0.0), ('odom', 'base_footprint', self.x, self.y, self.yaw)):
            t = TransformStamped(); t.header.stamp = now; t.header.frame_id = p; t.child_frame_id = c
            t.transform.translation.x, t.transform.translation.y = x, y; t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2); ts.append(t)
        self.tfb.sendTransform(ts)

    def _scan(self):
        s = LaserScan(); s.header.stamp = self.get_clock().now().to_msg(); s.header.frame_id = 'laser_link'; s.angle_min, s.angle_max = -math.pi, math.pi
        s.angle_increment = 2 * math.pi / 360; s.range_min, s.range_max = 0.1, 10.0; s.ranges = [3.0] * 360; self.pub_scan.publish(s)

    def _gate(self):
        self.pub_gate.publish(String(data=json.dumps({'state': 'NORMAL', 'cmd_blocked': False, 'wheel_odom_stale': False, 'est_odom_stale': False, 'depth_stale': False,
                                                      'stall_events': 0, 'failsafe_events': 0, 'icp_degraded_events': 0, 'operator_unlatch_count': 0, 'simulated': True})))
        if self.pub_limo:
            m = LimoStatus(); m.header.stamp = self.get_clock().now().to_msg(); m.battery_voltage = 12.3; self.pub_limo.publish(m)

    def _log(self, ev, **kw):
        path = self.get_parameter('goal_log').value
        if path:
            with open(path, 'a') as f: f.write(json.dumps(dict(t=time.time(), ev=ev, **kw)) + '\n')

    def _goal_cb(self, req):
        g = req.pose.pose.position; self._log('goal_received', x=g.x, y=g.y); return GoalResponse.ACCEPT

    def _exec(self, gh):
        g = gh.request.pose.pose; gx, gy = g.position.x, g.position.y; gyaw = 2 * math.atan2(g.orientation.z, g.orientation.w); t0 = time.monotonic(); fb = NavigateToPose.Feedback()
        while rclpy.ok():
            if gh.is_cancel_requested:
                self._log('goal_canceled'); gh.canceled(); return NavigateToPose.Result()
            dx, dy = gx - self.x, gy - self.y; dist = math.hypot(dx, dy)
            if dist <= self.tol:
                self.yaw = gyaw; self._log('goal_succeeded'); gh.succeed(); return NavigateToPose.Result()
            step = min(self.speed * 0.1, dist); self.x += dx / dist * step; self.y += dy / dist * step; self.yaw = math.atan2(dy, dx)
            fb.distance_remaining = float(dist); fb.navigation_time.sec = int(time.monotonic() - t0); gh.publish_feedback(fb); time.sleep(0.1)
        gh.abort(); return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args); n = MockRobot(); ex = MultiThreadedExecutor(num_threads=4); ex.add_node(n)
    try: ex.spin()
    except KeyboardInterrupt: pass
    finally:
        ex.shutdown(); n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
