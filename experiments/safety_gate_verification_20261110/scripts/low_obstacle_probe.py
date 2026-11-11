#!/usr/bin/env python3
"""Read-only bench probe: does a low obstacle (chair sled rail, 0.023-0.047 m above the floor) survive the
depth chain and reach the costmaps?  The robot is spawned facing the rail at a known distance and never moves.

usage: low_obstacle_probe.py --rail-x R --rail-y0 Y0 --rail-y1 Y1 [--wait 20]
  R = distance from robot base_footprint origin to the rail along the robot x axis (m),
  Y0..Y1 = rail extent along the robot y axis (m).
Reports, in the base_footprint frame (the filter's `height_frame`):
  * floor level z_f as seen by the depth camera (median z of clearing_points ahead)
  * points of /camera/depth/points-derived clearing cloud and of the marked cloud (/depth_obstacles/points) near the rail
  * local / global costmap cost along the rail and just in front of it
  * 2D scan returns in the sector of the rail
Publishes nothing."""
import argparse, json, math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros

ap = argparse.ArgumentParser()
ap.add_argument('--rail-x', type=float, default=None); ap.add_argument('--rail-y0', type=float, default=0.0)
ap.add_argument('--rail-y1', type=float, default=0.0); ap.add_argument('--wait', type=float, default=25.0)
ap.add_argument('--corridor', type=float, nargs=3, default=None, help='X0 X1 HALFWIDTH of a free-floor box ahead (robot frame) for false-marking checks')
ap.add_argument('--out', default='')
a = ap.parse_args()


def quat_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


rclpy.init()
n = Node('low_obstacle_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
got = {}
be = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(PointCloud2, '/depth_obstacles/points', lambda m: got.__setitem__('marked', m), be)
n.create_subscription(PointCloud2, '/depth_obstacles/clearing_points', lambda m: got.__setitem__('clear', m), be)
n.create_subscription(LaserScan, '/scan', lambda m: got.__setitem__('scan', m), qos_profile_sensor_data)
for nm, t in (('local', '/local_costmap/costmap'), ('global', '/global_costmap/costmap')):
    n.create_subscription(OccupancyGrid, t, lambda m, nm=nm: got.__setitem__(nm, m), tl)
    n.create_subscription(OccupancyGrid, t, lambda m, nm=nm: got.__setitem__(nm, m), be)
t0 = time.monotonic()
while time.monotonic() - t0 < a.wait or not all(k in got for k in ('marked', 'clear', 'scan', 'local', 'global')):
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.monotonic() - t0 > 90:
        break
n.get_logger().info('collected: ' + ','.join(got))
# let the costmaps integrate several observation cycles
t1 = time.monotonic()
while time.monotonic() - t1 < 8:
    rclpy.spin_once(n, timeout_sec=0.1)

out = {}


def cloud_in_base(msg):
    pts = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=np.float64).reshape(-1, 3)
    tr = tfb.lookup_transform('base_footprint', msg.header.frame_id, rclpy.time.Time())
    t = tr.transform.translation; q = tr.transform.rotation
    R = quat_rot(q)
    return pts @ R.T + np.array([t.x, t.y, t.z]), (t.x, t.y, t.z)


# camera extrinsics as the TF tree defines them
cl, cam = cloud_in_base(got['clear'])
out['camera_height_in_base_footprint_tf_m'] = round(cam[2], 3)
ahead = cl[(cl[:, 0] > 0.45) & (np.abs(cl[:, 1]) < 0.12)]
# floor = the dominant low z level among points ahead
zf = float(np.percentile(ahead[:, 2], 10)) if len(ahead) else None
out['floor_level_z_in_filter_frame_m'] = round(zf, 3) if zf is not None else None
if len(ahead):
    hist, edges = np.histogram(ahead[:, 2], bins=np.arange(-0.10, 0.12, 0.01))
    out['z_histogram_points_ahead(0.01m_bins_from_-0.10)'] = hist.tolist()
out['depth_filter_params'] = {}
if a.rail_x is not None:
    RX, Y0, Y1 = a.rail_x, a.rail_y0, a.rail_y1
    near = lambda P: P[(P[:, 0] > RX - 0.12) & (P[:, 0] < RX + 0.12) & (P[:, 1] > Y0 - 0.12) & (P[:, 1] < Y1 + 0.12)]
    nc = near(cl)
    out['clearing_points_near_rail'] = int(len(nc))
    if len(nc):
        out['clearing_points_near_rail_z'] = dict(min=round(float(nc[:, 2].min()), 3), p50=round(float(np.median(nc[:, 2])), 3), max=round(float(nc[:, 2].max()), 3))
        above_floor = nc[nc[:, 2] > (zf + 0.012 if zf is not None else 0)]
        out['clearing_points_near_rail_above_floor+12mm'] = dict(n=int(len(above_floor)), z_max=round(float(above_floor[:, 2].max()), 3) if len(above_floor) else None,
                                                                 height_above_floor_max_m=round(float(above_floor[:, 2].max() - zf), 3) if len(above_floor) and zf is not None else None)
    mk, _ = cloud_in_base(got['marked']) if got['marked'].width > 0 else (np.zeros((0, 3)), None)
    out['marked_points_total'] = int(len(mk))
    nm_ = near(mk) if len(mk) else mk
    out['marked_points_near_rail'] = int(len(nm_))
    if len(nm_):
        out['marked_points_near_rail_z'] = dict(min=round(float(nm_[:, 2].min()), 3), max=round(float(nm_[:, 2].max()), 3))

    # costmaps: robot pose in the costmap frame, then sample cost along the rail
    for nm in ('local', 'global'):
        m = got[nm]; frame = m.header.frame_id
        tr = tfb.lookup_transform(frame, 'base_footprint', rclpy.time.Time())
        q = tr.transform.rotation; yaw = 2 * math.atan2(q.z, q.w)
        px, py = tr.transform.translation.x, tr.transform.translation.y
        info = m.info; arr = np.asarray(m.data, dtype=np.int16).reshape(info.height, info.width)

        def cost(rx, ry):
            wx = px + rx * math.cos(yaw) - ry * math.sin(yaw); wy = py + rx * math.sin(yaw) + ry * math.cos(yaw)
            i = int((wx - info.origin.position.x) / info.resolution); j = int((wy - info.origin.position.y) / info.resolution)
            return int(arr[j, i]) if 0 <= i < info.width and 0 <= j < info.height else None
        ys = np.linspace(Y0, Y1, 7)
        out[f'{nm}_costmap_frame'] = frame
        out[f'{nm}_cost_on_rail_line'] = [cost(RX, y) for y in ys]
        out[f'{nm}_cost_0.10m_before_rail'] = [cost(RX - 0.10, y) for y in ys]
        out[f'{nm}_cost_robot_center'] = cost(0.0, 0.0)
        out[f'{nm}_max_cost_in_rail_box(+-0.10m)'] = max(v for v in (cost(RX + dx, y + dy) for dx in (-0.1, 0, 0.1) for y in ys for dy in (-0.1, 0, 0.1)) if v is not None)

    sc = got['scan']; ang = sc.angle_min + np.arange(len(sc.ranges)) * sc.angle_increment
    r = np.array(sc.ranges)
    # scan frame -> assume laser frame ~ base_footprint heading; sector around the rail bearing
    lo = math.atan2(Y0, RX) - 0.10; hi = math.atan2(Y1, RX) + 0.10
    sel = (ang > lo) & (ang < hi) & np.isfinite(r) & (r > sc.range_min) & (r < sc.range_max)
    out['scan_sector_deg'] = [round(math.degrees(lo), 1), round(math.degrees(hi), 1)]
    out['scan_returns_in_sector'] = [round(float(x), 2) for x in np.sort(r[sel])[:12]]
    out['scan_expected_rail_range_m'] = round(RX, 2)

# ---- free-floor corridor (false marking) and overall costmap obstacle totals ----
mk2 = cloud_in_base(got['marked'])[0] if got['marked'].width > 0 else np.zeros((0, 3))
out['marked_points_total_all'] = int(len(mk2)); out['marked_z_min_all'] = round(float(mk2[:, 2].min()), 3) if len(mk2) else None
for nm in ('local', 'global'):
    m = got[nm]; arr = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    out[f'{nm}_cells_eq100'] = int((arr == 100).sum()); out[f'{nm}_cells_ge99'] = int((arr >= 99).sum()); out[f'{nm}_cells_free0'] = int((arr == 0).sum())
if a.corridor is not None:
    X0c, X1c, HW = a.corridor
    inbox = lambda P: P[(P[:, 0] >= X0c) & (P[:, 0] <= X1c) & (np.abs(P[:, 1]) <= HW)]
    cm = inbox(mk2); out['corridor_marked_points'] = int(len(cm))
    out['corridor_marked_z'] = dict(min=round(float(cm[:, 2].min()), 3), max=round(float(cm[:, 2].max()), 3)) if len(cm) else None
    cc = inbox(cl); out['corridor_floor_points_seen'] = int((cc[:, 2] < zf + 0.012).sum()) if len(cc) and zf is not None else 0
    for nm in ('local', 'global'):
        m = got[nm]; frame = m.header.frame_id
        tr = tfb.lookup_transform(frame, 'base_footprint', rclpy.time.Time())
        q = tr.transform.rotation; yaw = 2 * math.atan2(q.z, q.w); px, py = tr.transform.translation.x, tr.transform.translation.y
        info = m.info; arr = np.asarray(m.data, dtype=np.int16).reshape(info.height, info.width)
        bad = 0; tot = 0
        for rx in np.arange(X0c, X1c + 1e-6, 0.05):
            for ry in np.arange(-HW, HW + 1e-6, 0.05):
                wx = px + rx * math.cos(yaw) - ry * math.sin(yaw); wy = py + rx * math.sin(yaw) + ry * math.cos(yaw)
                i2 = int((wx - info.origin.position.x) / info.resolution); j2 = int((wy - info.origin.position.y) / info.resolution)
                if 0 <= i2 < info.width and 0 <= j2 < info.height:
                    tot += 1; bad += int(arr[j2, i2] >= 99)
        out[f'corridor_{nm}_cells_ge99'] = f'{bad}/{tot}'
print(json.dumps(out, indent=1))
if a.out:
    json.dump(out, open(a.out, 'w'), indent=1)
import os; os._exit(0)
