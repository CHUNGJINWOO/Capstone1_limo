#!/usr/bin/env python3
"""Robot-side measurement node for the integrated simulation (read-only; publishes nothing).
Records JSONL samples (wall time, sim time, ground-truth world pose, odometry-free distance travelled, scan min range, local/global costmap
evidence near the scenario obstacle, global plan, Safety Gate state, /cmd_vel) and a summary JSON on exit.
usage: sim_metrics.py OUT_PREFIX [--box X Y HALF] ...   (box = known obstacle in WORLD coordinates; map->world transform given by --map-from-world)
"""
import argparse, json, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

ap = argparse.ArgumentParser(); ap.add_argument('out'); ap.add_argument('--box', nargs=3, type=float, action='append', default=[], metavar=('X', 'Y', 'HALF'))
ap.add_argument('--robot-radius', type=float, default=0.155)   # circumscribed radius of the 0.28 x 0.13 m chassis
args = ap.parse_args()
rclpy.init(); n = Node('integrated_sim_metrics', parameter_overrides=[Parameter('use_sim_time', value=True)])
buf = Buffer(); TransformListener(buf, n)
S = dict(gt=None, dist=0.0, last_gt=None, scan_min=None, scan_min_all=9e9, gate=None, cmd=None, cmd_n=0, plan=None, plans=[], loc=None, glo=None,
         person=None, min_clear={}, min_person=9e9, first_wall=None, first_sim=None, collisions=0, in_collision=False, nogate=0, gate_states={})
BOXES = [(i, b) for i, b in enumerate(args.box)]


def gt(m):
    p = m.pose.pose.position; q = m.pose.pose.orientation; yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    if S['last_gt'] is not None: S['dist'] += math.hypot(p.x - S['last_gt'][0], p.y - S['last_gt'][1])
    S['last_gt'] = (p.x, p.y); S['gt'] = (p.x, p.y, yaw)
    for i, (bx, by, h) in BOXES:                       # distance from the robot centre to the (axis-aligned) box surface
        dx = max(abs(p.x - bx) - h, 0.0); dy = max(abs(p.y - by) - h, 0.0); d = math.hypot(dx, dy)
        S['min_clear'][i] = min(S['min_clear'].get(i, 9e9), d)
    near = any(math.hypot(dx_, dy_) < args.robot_radius for dx_, dy_ in [(max(abs(p.x - bx) - h, 0.0), max(abs(p.y - by) - h, 0.0)) for _, (bx, by, h) in BOXES])
    if S['person'] is not None:
        d = math.hypot(p.x - S['person'][0], p.y - S['person'][1]); S['min_person'] = min(S['min_person'], d)
        near = near or d < args.robot_radius + 0.22
    if near and not S['in_collision']: S['collisions'] += 1
    S['in_collision'] = near


def scan(m):
    r = [v for v in m.ranges if m.range_min < v < m.range_max]; S['scan_min'] = min(r) if r else None
    if r: S['scan_min_all'] = min(S['scan_min_all'], min(r))


def gate(m):
    try: S['gate'] = json.loads(m.data); st = S['gate'].get('state'); S['gate_states'][st] = S['gate_states'].get(st, 0) + 1
    except ValueError: pass


def cmd(m): S['cmd'] = (m.linear.x, m.angular.z); S['cmd_n'] += 1


def plan(m):
    pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
    if len(pts) < 2: return
    L = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]) for i in range(len(pts) - 1))
    S['plan'] = {'n': len(pts), 'len': L, 'goal': pts[-1]}; S['plans'].append({'t': time.time(), 'len': round(L, 3), 'goal': [round(v, 2) for v in pts[-1]], 'pts': [[round(a, 2), round(b, 2)] for a, b in pts[::6]]})


def person(m):
    for t in m.transforms:
        if t.child_frame_id.endswith('person_walker') or t.child_frame_id == 'person_walker': S['person'] = (t.transform.translation.x, t.transform.translation.y)


def grid_near(g, mx, my, r):
    """max cost in g within radius r of the map-frame point (mx,my)"""
    res = g.info.resolution; ox, oy = g.info.origin.position.x, g.info.origin.position.y; best = -1
    for j in range(int((my - r - oy) / res), int((my + r - oy) / res) + 1):
        for i in range(int((mx - r - ox) / res), int((mx + r - ox) / res) + 1):
            if 0 <= i < g.info.width and 0 <= j < g.info.height and (i * res + ox - mx) ** 2 + (j * res + oy - my) ** 2 <= r * r: best = max(best, g.data[j * g.info.width + i])
    return best


qos_l = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(Odometry, '/ground_truth/world_odom', gt, 50); n.create_subscription(LaserScan, '/scan', scan, 10)
n.create_subscription(String, '/stall_detector/status_json', gate, 10); n.create_subscription(Twist, '/cmd_vel', cmd, 20)
n.create_subscription(PointCloud2, '/depth_obstacles/points', lambda m: S.update(depth_pts=m.width * m.height, depth_max=max(S.get('depth_max', 0), m.width * m.height)), 5)
n.create_subscription(Path, '/plan', plan, 10); n.create_subscription(TFMessage, '/gz/dynamic_pose_info', person, 20)
n.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: S.__setitem__('loc', m), 1)
n.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: S.__setitem__('glo', m), 1)
f = open(args.out + '.jsonl', 'w'); last = 0.0
# box positions in MAP coordinates for the costmap evidence: world = (-(my+1.1), mx-0.6)  =>  mx = wy+0.6, my = -wx-1.1 (estimate; AMCL corrects the residual)
BOX_MAP = [(by + 0.6, -bx - 1.1) for _, (bx, by, _h) in BOXES]; cost_max = {'loc': {}, 'glo': {}}
try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
        if time.monotonic() - last < 0.5: continue
        last = time.monotonic(); wall = time.time(); sim = n.get_clock().now().nanoseconds / 1e9
        if S['first_wall'] is None and sim > 0: S['first_wall'], S['first_sim'] = wall, sim
        try:
            t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()); q = t.transform.rotation
            pose = [t.transform.translation.x, t.transform.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))]
        except Exception: pose = None
        ev = {}
        try:   # local costmap lives in the odom frame (rolling window): convert the map-frame box position with the current map->odom transform
            mo = buf.lookup_transform('odom', 'map', rclpy.time.Time()); tx, ty = mo.transform.translation.x, mo.transform.translation.y; qq = mo.transform.rotation
            yaw_mo = math.atan2(2 * (qq.w * qq.z + qq.x * qq.y), 1 - 2 * (qq.y * qq.y + qq.z * qq.z))
        except Exception: tx = ty = yaw_mo = None
        for k, key in (('loc', 'local'), ('glo', 'global')):
            if S[k] is not None:
                for i, (mx, my) in enumerate(BOX_MAP):
                    if k == 'loc':
                        if tx is None: continue
                        mx, my = tx + math.cos(yaw_mo) * mx - math.sin(yaw_mo) * my, ty + math.sin(yaw_mo) * mx + math.cos(yaw_mo) * my
                    v = grid_near(S[k], mx, my, 0.35); ev[f'{key}_box{i}'] = v; cost_max[k][i] = max(cost_max[k].get(i, -1), v)
        f.write(json.dumps(dict(wall=wall, sim=sim, gt=S['gt'], map_pose=pose, dist=round(S['dist'], 3), scan_min=S['scan_min'], gate=(S['gate'] or {}).get('state'), cmd=S['cmd'],
                                plan=S['plan'], person=S['person'], clear=S['min_clear'], costmap=ev, collisions=S['collisions'], depth_pts=S.get('depth_pts'))) + '\n'); f.flush()
except BaseException:   # SIGINT shuts the rclpy context down under spin_once; always fall through to the summary
    pass
summary = dict(wall_s=(time.time() - S['first_wall']) if S['first_wall'] else None, sim_s=(n.get_clock().now().nanoseconds / 1e9 - S['first_sim']) if S['first_sim'] else None,
               distance_m=round(S['dist'], 3), collisions=S['collisions'], min_clearance_to_box_m={str(k): round(v, 3) for k, v in S['min_clear'].items()},
               min_distance_to_person_m=None if S['min_person'] > 1e8 else round(S['min_person'], 3), scan_min_all_m=None if S['scan_min_all'] > 1e8 else round(S['scan_min_all'], 3),
               costmap_max_near_box={'local': cost_max['loc'], 'global': cost_max['glo']}, gate_states=S['gate_states'], depth_obstacle_points_max=S.get('depth_max'), cmd_vel_msgs=S['cmd_n'], plans=len(S['plans']), plan_lengths=[p['len'] for p in S['plans']][-200:])
if summary['wall_s'] and summary['sim_s']: summary['rtf_mean'] = round(summary['sim_s'] / summary['wall_s'], 3)
json.dump(summary, open(args.out + '_summary.json', 'w'), indent=1); json.dump(S['plans'][-400:], open(args.out + '_plans.json', 'w'))
try: n.destroy_node()
except Exception: pass
if rclpy.ok(): rclpy.shutdown()
