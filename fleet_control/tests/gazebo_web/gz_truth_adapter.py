"""SIM ONLY: Gazebo ground truth (/ground_truth/world_odom, world frame) -> /e2e/ground_truth [x, y, yaw, v, w] in the SPAWN frame.

The web-mapping session starts SLAM at the spawn pose, so its map frame equals this spawn frame; the existing E2E harnesses read
/e2e/ground_truth the same way as from tests/mode_e2e/kinematic_sim.py. Test-only; never on a robot.
"""
import math, os
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0', '77')


def yaw(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init(); n = rclpy.create_node('gz_truth_adapter')
    sx, sy, syaw = (float(n.declare_parameter(k, 0.0).value) for k in ('spawn_x', 'spawn_y', 'spawn_yaw'))
    pub = n.create_publisher(Float64MultiArray, '/e2e/ground_truth', 10)
    c, s = math.cos(-syaw), math.sin(-syaw)
    prev = [None]
    def on(m):
        dx, dy = m.pose.pose.position.x - sx, m.pose.pose.position.y - sy
        th = math.atan2(math.sin(yaw(m.pose.pose.orientation) - syaw), math.cos(yaw(m.pose.pose.orientation) - syaw))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9; x, y = c * dx - s * dy, s * dx + c * dy
        v = w = 0.0
        if prev[0] and t > prev[0][0]:                # speed from successive sim-time poses (the GT node may not fill twist)
            dt = t - prev[0][0]; v = math.hypot(x - prev[0][1], y - prev[0][2]) / dt
            w = math.atan2(math.sin(th - prev[0][3]), math.cos(th - prev[0][3])) / dt
        if not prev[0] or t > prev[0][0]: prev[0] = (t, x, y, th)
        pub.publish(Float64MultiArray(data=[x, y, th, v, w]))
    n.create_subscription(Odometry, '/ground_truth/world_odom', on, 20)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass


if __name__ == '__main__':
    main()
