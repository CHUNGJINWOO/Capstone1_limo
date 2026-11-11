import sys, time, math, statistics, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n = Node('node_test'); src = {}; res = []
def on_src(m): src[(m.header.stamp.sec, m.header.stamp.nanosec)] = (m, time.time())
def on_out(m):
    k = (m.header.stamp.sec, m.header.stamp.nanosec); s = src.get(k)
    if s is None: res.append(('NO_MATCHING_INPUT_STAMP',)); return
    o, t_in = s; res.append((time.time() - t_in, len(m.ranges), m.header.frame_id == o.header.frame_id, m.range_min == o.range_min and m.range_max == o.range_max, abs(m.angle_max - o.angle_max) < 1e-5, m.scan_time == o.scan_time,
                             int(round((m.angle_max - m.angle_min) / m.angle_increment)) + 1 == len(m.ranges), len(m.intensities) in (0, len(m.ranges)), m.time_increment > 0))
n.create_subscription(LaserScan, '/scan', on_src, qos_profile_sensor_data); n.create_subscription(LaserScan, '/scan_slam', on_out, qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 24: rclpy.spin_once(n, timeout_sec=0.05)
bad = [r for r in res if len(r) == 1]; ok = [r for r in res if len(r) > 1]
print('input scans seen %d, normalized scans received %d, without a matching input stamp: %d' % (len(src), len(res), len(bad)))
print('output length set:', sorted({r[1] for r in ok}), '| frame_id same:', all(r[2] for r in ok), '| range_min/max same:', all(r[3] for r in ok), '| angle_max same:', all(r[4] for r in ok), '| scan_time same:', all(r[5] for r in ok), '| Karto expected == len:', all(r[6] for r in ok), '| intensities consistent:', all(r[7] for r in ok), '| time_increment>0:', all(r[8] for r in ok))
lat = [r[0] * 1000 for r in ok]; print('added latency (replay publish -> /scan_slam received): median %.2f ms, max %.2f ms' % (statistics.median(lat), max(lat)))
rclpy.shutdown()
