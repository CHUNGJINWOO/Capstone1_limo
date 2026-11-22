"""SIMULATION ONLY (2026-11-22, Claude): LiDAR + Depth obstacle integration in Gazebo, per target, with a LiDAR-only baseline.

Why fixtures: the rmf_demos Office furniture has collision meshes that are solid blocks much larger than the visual meshes
(OfficeChairGrey: hull floor-0.49 m over 0.71 x 0.69 m, visual at the LiDAR plane = a 4 cm column; AdjTable: block floor-0.81 m,
visual = thin legs + 3 cm foot rails). Gazebo LiDAR and depth cameras render VISUAL geometry only, so a contact with those hulls
is invisible to every sensor. Each target here is therefore a test fixture whose visual == collision geometry, appended to a
COPY of the Office world (the original world file is read only).

Targets (sizes in metres, heights above the floor; the LiDAR plane is about 0.161 m, the robot body top about 0.125 m):
  chair_base   5-spoke star base (spokes 0.33 long, 0.035 high at 0.040-0.075) + 0.05 column to a seat at 0.45-0.51
  chair_bars   4 legs 0.03 at a 0.40 square + low connecting bars at 0.08-0.11 + seat 0.45-0.50
  desk_frame   desk 1.00 wide x 0.50 deep: 4 legs 0.04, low front stretcher at 0.06-0.10, top 0.72-0.75
  low_box      0.40 x 0.30 x 0.08 box
  glass        0.012 x 1.40 x 1.00 pane, visual transparency 0.9 (Gazebo rendering, NOT a model of real glass optics)
  none         no fixture (control / free-space survey)

Configs (each a fresh isolated stack: ROS_LOCALHOST_ONLY=1, own domain 210-219, own IGN_PARTITION; no LIMO, no 18190, no /cmd_vel):
  depth        tests/gazebo_web/gazebo_world.launch.py + gazebo_depth/depth_sensors.launch.py + web_mapping_isolated/stack.launch.py
               with NAV2_PARAMS_OVERLAY = truth-localization fixture + depth_obstacles source in both costmaps, Gate r13 copy with
               depth_mode=required (gate_depth_mode.py wrapper). REQUIRED readiness before a run may PASS: depth_status DEPTH_READY,
               /depth_obstacles/points fresh (< 0.5 s), TF base_footprint<-depth_camera_link, both costmaps subscribed to
               /depth_obstacles/points. Otherwise the run is SKIP_DEPTH_NOT_READY (never PASS).
  lidar        same, without depth_sensors.launch.py, lidar costmap sources only, Gate depth_mode=disabled (comparison baseline)
  depth_fault  depth config; the depth bridge process is SIGSTOPped during the approach: the Gate must block (depth fault) - PASS
               means the robot stops, not that it reaches the goal.
Velocity path: velocity_smoother -> /cmd_vel_raw -> Gate -> /cmd_vel_locked -> drive timeout relay -> Gazebo (ARBITER=0, WATCHDOG=0).
Localization FIXTURE (as tests/gazebo_web/nav2_d3_targeted.py): odom = Gazebo truth, map->odom identity, rolling global costmap.

Per run (run.json + streams.jsonl): stationary detection before the goal (raw depth / filter / LiDAR points inside the fixture
volume, local and global costmap LETHAL cells inside the fixture footprint), detection vs range during the approach, Nav2 result,
truth trajectory, minimum 2D distance robot footprint <-> fixture collision boxes that reach below 0.22 m (body + sensors), contact
indicator (locked |vx| > 0.04 m/s with truth speed < 0.01 m/s for > 0.5 s), Gate states, depth status. Collision Monitor: N/A.

usage: python3 run_depth_targets.py --out <new dir> --config depth|lidar|depth_fault --targets a,b --repeat N [--domain 233]
       python3 run_depth_targets.py --out <new dir> --survey      (free-space scan at the test start pose, no fixture)
"""
import argparse, json, math, os, signal, subprocess, sys, threading, time
from pathlib import Path

HERE = Path(__file__).resolve().parent; FC = HERE.parents[1]; REPO = FC.parent
WORLD_SRC = REPO / 'experiments/office_limo_sim_20261113/office_limo.world'
GATE_R13 = REPO / 'local_data/raw_results/gate_r13_integ_claude_20261122/candidate_r13/stall_detector_node.py'
TRUTH_LOC = REPO / 'local_data/raw_results/gazebo_depth_claude_20261122/inputs/d3_targeted_truth_localization.yaml'
SPAWN = (0.0, 0.0)                # test room origin (world frame); the lane below is in this frame
ROOM = dict(x0=-1.0, x1=6.0, y0=-1.5, y1=1.5, h=0.6)   # enclosure interior, walls 0.03 m, visual == collision
FOOT_X, FOOT_Y = 0.145, 0.115
BODY_TOP = 0.22                   # body box top 0.125 m + LiDAR puck / camera mount; boxes starting below this can be hit
FLOOR_Z_BF = -0.045               # Gazebo floor in base_footprint (model origin) - measured value is recorded per run
# test lane in the SPAWN frame (yaw 0): start, fixture centre, goal (set from the --survey result, see GAZEBO_DEPTH_20261122.md)
# The Office world was abandoned for these runs (survey_d + smoke3): its furniture collision hulls are invisible to every sensor and
# no straight 3 m lane with +-0.9 m clearance exists near the spawn once those hulls are counted. The test world is a COPY of the
# Office world's physics/sensor/light setup with all models removed, a ground plane and a 7 x 3 m enclosure (GAZEBO_DEPTH_20261122.md).
LANE = {'start': (0.0, 0.0, 0.0), 'fixture': (2.0, 0.0), 'fixture_yaw': 0.0, 'goal': (4.0, 0.0)}


def box(name, cx, cy, z0, z1, sx, sy, yaw=0.0, transparent=False):
    geo = f'<geometry><box><size>{sx:.3f} {sy:.3f} {z1 - z0:.3f}</size></box></geometry>'
    mat = ('<material><ambient>0.6 0.8 0.9 0.1</ambient><diffuse>0.6 0.8 0.9 0.1</diffuse></material><transparency>0.9</transparency>'
           if transparent else '<material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>')
    return (f'<link name="{name}"><pose>{cx:.4f} {cy:.4f} {(z0 + z1) / 2:.4f} 0 0 {yaw:.4f}</pose>'
            f'<collision name="c">{geo}</collision><visual name="v">{geo}{mat}</visual></link>',
            dict(name=name, cx=cx, cy=cy, z0=z0, z1=z1, sx=sx, sy=sy, yaw=yaw))


def fixture_parts(target):
    """Parts in the fixture frame (x toward the approaching robot = +x of the lane reversed is handled by the caller)."""
    P = []
    if target == 'chair_base':
        for k in range(5):
            a = 2 * math.pi * k / 5; L = 0.33
            P.append(box(f'spoke{k}', L / 2 * math.cos(a), L / 2 * math.sin(a), 0.040, 0.075, L, 0.035, a))
        P.append(box('column', 0, 0, 0.075, 0.45, 0.05, 0.05)); P.append(box('seat', 0, 0, 0.45, 0.51, 0.45, 0.45))
    elif target == 'chair_bars':
        h = 0.20
        for i, (x, y) in enumerate([(h, h), (h, -h), (-h, h), (-h, -h)]): P.append(box(f'leg{i}', x, y, 0.0, 0.45, 0.03, 0.03))
        P += [box('bar_f', h, 0, 0.08, 0.11, 0.03, 0.40), box('bar_b', -h, 0, 0.08, 0.11, 0.03, 0.40),
              box('bar_l', 0, h, 0.08, 0.11, 0.40, 0.03), box('bar_r', 0, -h, 0.08, 0.11, 0.40, 0.03), box('seat', 0, 0, 0.45, 0.50, 0.45, 0.45)]
    elif target == 'desk_frame':
        dx, dy = 0.23, 0.48            # depth 0.50 along the lane, width 1.00 across it
        for i, (x, y) in enumerate([(dx, dy), (dx, -dy), (-dx, dy), (-dx, -dy)]): P.append(box(f'leg{i}', x, y, 0.0, 0.72, 0.04, 0.04))
        P += [box('stretcher', -dx, 0, 0.06, 0.10, 0.04, 0.96), box('top', 0, 0, 0.72, 0.75, 0.50, 1.00)]   # stretcher on the side facing the robot
    elif target == 'low_box':
        P.append(box('box', 0, 0, 0.0, 0.08, 0.30, 0.40))
    elif target == 'glass':
        P.append(box('pane', 0, 0, 0.0, 1.00, 0.012, 1.40, transparent=True))
    return P


def room_world():
    import xml.etree.ElementTree as ET
    root = ET.fromstring(WORLD_SRC.read_text()); world = root.find('world')
    keep = ''.join(ET.tostring(c, encoding='unicode') for c in world if c.tag in ('physics', 'plugin', 'scene', 'light'))
    head = f'<?xml version="1.0" ?>\n<sdf version="{root.get("version")}">\n'; m = type('M', (), {'group': lambda self, i: f'<world name="{world.get("name")}">'})()
    ground = ('<model name="ground_plane"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal>'
              '<size>40 40</size></plane></geometry></collision><visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size>'
              '</plane></geometry><material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual></link></model>')
    R = ROOM; walls = ''
    for k, (cx, cy, sx, sy) in {'n': ((R['x0'] + R['x1']) / 2, R['y1'] + .015, R['x1'] - R['x0'] + .06, .03),
                                's': ((R['x0'] + R['x1']) / 2, R['y0'] - .015, R['x1'] - R['x0'] + .06, .03),
                                'e': (R['x1'] + .015, 0, .03, R['y1'] - R['y0']), 'w': (R['x0'] - .015, 0, .03, R['y1'] - R['y0'])}.items():
        geo = f'<geometry><box><size>{sx:.3f} {sy:.3f} {R["h"]}</size></box></geometry>'
        walls += (f'<model name="room_wall_{k}"><static>true</static><pose>{cx:.3f} {cy:.3f} {R["h"] / 2} 0 0 0</pose><link name="l">'
                  f'<collision name="c">{geo}</collision><visual name="v">{geo}</visual></link></model>')
    return head + m.group(0) + keep + ground + walls + '</world>\n</sdf>\n'


def world_with_fixture(dst, target, fx, fy, fyaw=0.0):
    s = room_world(); i = s.rindex('</world>'); parts = fixture_parts(target); boxes = []
    if parts:
        links = ''.join(p[0] for p in parts); c, sn = math.cos(fyaw), math.sin(fyaw)
        s = s[:i] + f'<model name="depth_fixture_{target}"><static>true</static><pose>{fx:.4f} {fy:.4f} 0 0 0 {fyaw:.4f}</pose>{links}</model>\n' + s[i:]
        for _, b in parts: boxes.append(dict(b, cx=fx + c * b['cx'] - sn * b['cy'], cy=fy + sn * b['cx'] + c * b['cy'], yaw=b['yaw'] + fyaw))
    dst.write_text(s); return boxes


def in_box(x, y, z, b, m=0.03):
    c, s = math.cos(-b['yaw']), math.sin(-b['yaw']); dx, dy = x - b['cx'], y - b['cy']
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return abs(lx) <= b['sx'] / 2 + m and abs(ly) <= b['sy'] / 2 + m and (z is None or b['z0'] - m <= z <= b['z1'] + m)


def foot_clearance(px, py, pth, boxes):
    """min 2D distance between the robot footprint rectangle and the low boxes (sampled perimeters, 1 cm)."""
    low = [b for b in boxes if b['z0'] < BODY_TOP]
    if not low: return None
    def rect_pts(cx, cy, hx, hy, yaw, step=0.01):
        c, s = math.cos(yaw), math.sin(yaw); out = []
        n = max(2, int(2 * hx / step)); m = max(2, int(2 * hy / step))
        for i in range(n + 1):
            for yy in (-hy, hy): xx = -hx + 2 * hx * i / n; out.append((cx + c * xx - s * yy, cy + s * xx + c * yy))
        for j in range(m + 1):
            for xx in (-hx, hx): yy = -hy + 2 * hy * j / m; out.append((cx + c * xx - s * yy, cy + s * xx + c * yy))
        return out
    robot = rect_pts(px, py, FOOT_X, FOOT_Y, pth); best = 9.9
    for b in low:
        if any(in_box(x, y, None, b, 0.0) for x, y in robot): return 0.0
        for x, y in rect_pts(b['cx'], b['cy'], b['sx'] / 2, b['sy'] / 2, b['yaw']):
            c, s = math.cos(-pth), math.sin(-pth); dx, dy = x - px, y - py; lx, ly = c * dx - s * dy, s * dx + c * dy
            if abs(lx) <= FOOT_X and abs(ly) <= FOOT_Y: return 0.0
        for x, y in robot:
            for qx, qy in rect_pts(b['cx'], b['cy'], b['sx'] / 2, b['sy'] / 2, b['yaw'], 0.02): best = min(best, math.hypot(x - qx, y - qy))
    return best


def overlay(path, depth, layer='shared'):
    import yaml
    fix = yaml.safe_load(TRUTH_LOC.read_text())     # copy (sha recorded in the run MANIFEST) of the D3 session's fixture overlay
    if depth:
        src = dict(topic='/depth_obstacles/points', max_obstacle_height=0.40, min_obstacle_height=-0.05, clearing=True, marking=True,
                   data_type='PointCloud2', raytrace_max_range=3.0, raytrace_min_range=0.0, obstacle_max_range=2.5, obstacle_min_range=0.20)
        # the height band is applied by the filter (0.025-0.35 m above the floor); the costmap band is only widened by the SIM floor
        # offset so that it never cuts filter output (base_footprint 0.045 m above the Gazebo floor). Sources as in the depth profile.
        # layer 'shared'   = integrated_nav2_depth_slow_params.yaml as written: one obstacle_layer with scan + depth (LiDAR raytrace
        #                    clearing can erase depth-only marks once they leave the camera view).
        # layer 'separate' = CANDIDATE: a second nav2_costmap_2d::ObstacleLayer 'depth_layer' holding only depth_obstacles, so the
        #                    scan source clears only its own layer. Same depth source values. Not applied to any shared file.
        for cm, srcs in (('local_costmap', 'scan virtual_bumper'), ('global_costmap', 'scan')):
            rp = fix.setdefault(cm, {}).setdefault(cm, {}).setdefault('ros__parameters', {})
            if layer == 'shared':
                ol = rp.setdefault('obstacle_layer', {}); ol['observation_sources'] = srcs.replace('scan', 'scan depth_obstacles', 1); ol['depth_obstacles'] = dict(src)   # copy: no YAML alias
            else:
                plugins = rp.get('plugins') or (['obstacle_layer', 'inflation_layer'] if cm == 'local_costmap' else ['static_layer', 'obstacle_layer', 'inflation_layer'])
                rp['plugins'] = [x for x in plugins if x != 'inflation_layer'] + ['depth_layer', 'inflation_layer']
                rp['depth_layer'] = dict(plugin='nav2_costmap_2d::ObstacleLayer', enabled=True, observation_sources='depth_obstacles', depth_obstacles=dict(src))
    Path(path).write_text(yaml.safe_dump(fix, sort_keys=False)); return fix


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True); ap.add_argument('--config', default='depth', choices=['depth', 'lidar', 'depth_fault'])
    ap.add_argument('--targets', default='chair_base,chair_bars,desk_frame,low_box,glass'); ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--domain', default='210'); ap.add_argument('--depth-layer', default='shared', choices=['shared', 'separate']); ap.add_argument('--timeout', type=float, default=90.0); ap.add_argument('--survey', action='store_true')
    a = ap.parse_args()
    assert 210 <= int(a.domain) <= 219, 'domains 210-219 for this harness (agreed with the D3 session: it uses 200-204)'
    out = Path(a.out); out.mkdir()
    depth = a.config != 'lidar'
    ovl = out / 'nav2_overlay.yaml'; overlay(ovl, depth, a.depth_layer)
    targets = ['none'] if a.survey else a.targets.split(',')
    summary = []
    for r in range(a.repeat):
        for t in targets:
            res = run_case(out / f'{t}_{a.config}_r{r}', t, a, ovl, depth)
            summary.append(res)
            print(json.dumps({k: res.get(k) for k in ('target', 'config', 'result', 'nav2', 'min_clear_m', 'blocked_s', 'stationary', 'first_lethal_range_m')}), flush=True)
    (out / 'SUMMARY.json').write_text(json.dumps({'config': a.config, 'cases': summary}, indent=1))


def run_case(d, target, a, ovl, depth):
    d.mkdir(); st = LANE['start']; ws = (SPAWN[0] + st[0], SPAWN[1] + st[1], st[2])
    world = d / 'office_depth_fixture.world'
    boxes = world_with_fixture(world, target, SPAWN[0] + LANE['fixture'][0], SPAWN[1] + LANE['fixture'][1], LANE['fixture_yaw'])
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=a.domain, IGN_PARTITION=f'dep_{a.domain}_{os.getpid()}_{d.name}', IGN_IP='127.0.0.1',
               GZ_SPAWN=f'{ws[0]},{ws[1]},{ws[2]}', GZ_WORLD=str(world), GZ_DRIVE_TIMEOUT_S='0.2', USE_SIM_TIME='1', ARBITER='0', WATCHDOG='0',
               ODOM_ACTIVE_TOPIC='/ground_truth/odom', LIFECYCLE_DELAY_S='5', GATE_SCRIPT=str(HERE / 'gate_depth_mode.py'),
               GATE_SCRIPT_REAL=str(GATE_R13), GATE_DEPTH_MODE='required' if depth else 'disabled', NAV2_PARAMS_OVERLAY=str(ovl),
               ROS_LOG_DIR=str(d / 'roslog'), ROS2CLI_NO_DAEMON='1')
    for k in ('ROS_DISCOVERY_SERVER', 'GATE_INPUT_EXCL', 'GATE_WRITER_STATUS', 'GATE_RESET_PEER', 'GATE_REQ_WD'): env.pop(k, None)
    procs = {}
    def start_p(tag, cmd):
        procs[tag] = subprocess.Popen(cmd, env=env, stdout=(d / f'{tag}.log').open('x'), stderr=subprocess.STDOUT, start_new_session=True)
    start_p('sim', ['ros2', 'launch', str(FC / 'tests/gazebo_web/gazebo_world.launch.py')])
    time.sleep(4)
    if depth: start_p('depth', ['ros2', 'launch', str(HERE / 'depth_sensors.launch.py')])
    start_p('stack', ['ros2', 'launch', str(FC / 'tests/web_mapping_isolated/stack.launch.py')])
    start_p('tf', ['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--frame-id', 'map', '--child-frame-id', 'odom', '--ros-args', '-p', 'use_sim_time:=true'])
    os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=a.domain); os.environ.pop('ROS_DISCOVERY_SERVER', None)
    res = {'target': target, 'config': a.config, 'depth_layer': a.depth_layer if depth else None, 'run': d.name, 'fixture_boxes': boxes, 'lane_spawn_frame': LANE,
           'collision_monitor': 'N/A (not deployed in this stack or the real launch)', 'localization': 'FIXTURE: truth odom, map->odom identity'}
    try:
        res.update(drive(d, target, a, depth, boxes, ws, procs))
    except Exception as e:
        import traceback; res.update(result='HARNESS_ERROR', error=repr(e)[:300], trace=traceback.format_exc()[-1500:])
    finally:
        for p in reversed(list(procs.values())):
            try: os.killpg(p.pid, signal.SIGCONT); os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError: pass
        time.sleep(4)
        for p in procs.values():
            try: os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError: pass
    (d / 'run.json').write_text(json.dumps(res, indent=1))
    return res


def drive(d, target, a, depth, boxes, ws, procs):
    import numpy as np
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan, PointCloud2
    from std_msgs.msg import String
    from nav2_msgs.action import NavigateToPose
    from tf2_ros import Buffer, TransformListener
    rclpy.init(); n = rclpy.create_node('depth_targets_observer', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    tfb = Buffer(); TransformListener(tfb, n)
    S = dict(truth=None, gate=None, depth_status=None, depth_status_t=None, depth_pts_t=None, locked=(0.0, 0.0), counts={}, lethal={},
             floor_z=[], first_lethal=None, first_filter=None, approach=[])
    rows = (d / 'streams.jsonl').open('x'); lock = threading.Lock()
    def rec(k, **v):
        with lock: rows.write(json.dumps({'k': k, 't': round(time.monotonic(), 4), **v}) + '\n')
    st = LANE['start']
    def odom_to_world(x, y):                     # odom = case start pose frame (truth odom starts at the spawn pose)
        c, s = math.cos(ws[2]), math.sin(ws[2]); return ws[0] + c * x - s * y, ws[1] + s * x + c * y
    def robot_range():
        if not S['truth'] or not boxes: return None
        fx, fy = SPAWN[0] + LANE['fixture'][0], SPAWN[1] + LANE['fixture'][1]; return math.hypot(S['truth'][0] - fx, S['truth'][1] - fy)
    def on_truth(m):
        p = m.pose.pose; q = p.orientation; y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        now = time.monotonic(); S['truth'] = (p.position.x, p.position.y, y, v, now)
        if now - S.get('truth_rec', 0) < 0.1: return             # 10 Hz record + clearance (CPU bound)
        S['truth_rec'] = now; cl = foot_clearance(p.position.x, p.position.y, y, boxes)
        rec('truth', x=round(p.position.x, 4), y=round(p.position.y, 4), th=round(y, 4), v=round(v, 4), clear=None if cl is None else round(cl, 3))
    def cloud_counter(tag, frame_hint):
        def f(m):
            now = time.monotonic()
            if tag == 'filter':
                if S['depth_pts_t'] is not None: S['filter_gap_max'] = max(S.get('filter_gap_max', 0.0), now - S['depth_pts_t'])
                S['depth_pts_t'] = now
            if int(now * 4) % 2: return           # every other 0.25 s window: enough samples, bounded CPU
            try:
                tr = tfb.lookup_transform('odom', m.header.frame_id, rclpy.time.Time())
                bf = tfb.lookup_transform('base_footprint', m.header.frame_id, rclpy.time.Time())
            except Exception: return
            pts = np.frombuffer(bytes(m.data), dtype=np.float32).reshape(-1, m.point_step // 4)[:, :3]
            if len(pts) > 6000: pts = pts[:: len(pts) // 6000 + 1]
            def apply(t, P):
                q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
                R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                              [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
                tv = t.transform.translation; return P @ R.T + np.array([tv.x, tv.y, tv.z])
            Po = apply(tr, pts); Pb = apply(bf, pts)
            if tag == 'raw':                        # floor height in base_footprint: low percentile of points 0.4-1.5 m ahead
                sel = Pb[(Pb[:, 0] > 0.4) & (Pb[:, 0] < 1.5) & (np.abs(Pb[:, 1]) < 0.3)]
                if len(sel) > 50: S['floor_z'].append(float(np.percentile(sel[:, 2], 5)))
            hit = 0
            for x, y, z in Po:
                wx, wy = odom_to_world(float(x), float(y))
                if any(in_box(wx, wy, None, b) for b in boxes): hit += 1
            S['counts'][tag] = hit; rng = robot_range()
            rec(tag, n=int(len(pts)), in_fixture=hit, range=None if rng is None else round(rng, 3))
            if tag == 'filter' and hit >= 5 and S['first_filter'] is None and rng is not None: S['first_filter'] = round(rng, 3)
        return f
    def on_scan(m):
        if not S['truth']: return
        if a.survey and not S.get('scan_saved') and S['truth']:          # survey: one full scan + truth pose, for lane planning
            S['scan_saved'] = True; (d / 'survey_scan.json').write_text(json.dumps(dict(angle_min=m.angle_min, inc=m.angle_increment,
                ranges=[None if not math.isfinite(r) else round(r, 3) for r in m.ranges], rmax=m.range_max, truth_world=S['truth'][:3])))
        hit = 0
        for i, r in enumerate(m.ranges):
            if not (m.range_min <= r <= m.range_max): continue
            a_ = m.angle_min + i * m.angle_increment; px, py = 0.103 + r * math.cos(a_), r * math.sin(a_)
            c, s = math.cos(S['truth'][2]), math.sin(S['truth'][2])
            wx, wy = S['truth'][0] + c * px - s * py, S['truth'][1] + s * px + c * py
            if any(in_box(wx, wy, None, b) for b in boxes): hit += 1
        S['counts']['lidar'] = hit
        if int(time.monotonic() * 4) % 4 == 0: rec('scan', in_fixture=hit)
    def grid_counter(tag):
        def f(m):
            info = m.info; data = np.asarray(m.data, dtype=np.int16).reshape(info.height, info.width)
            ys, xs = np.nonzero(data >= 99); hit = 0
            for j, i in zip(ys, xs):
                ox = info.origin.position.x + (i + .5) * info.resolution; oy = info.origin.position.y + (j + .5) * info.resolution
                wx, wy = odom_to_world(ox, oy) if m.header.frame_id == 'odom' else odom_to_world(ox, oy)
                if any(in_box(wx, wy, None, b, 0.06) for b in boxes): hit += 1
            if a.survey and tag == 'global_costmap':                    # survey: keep the last grid (odom frame) for lane planning
                np.save(d / 'survey_global_costmap.npy', data); (d / 'survey_global_costmap.json').write_text(json.dumps(dict(
                    res=info.resolution, ox=info.origin.position.x, oy=info.origin.position.y, w=info.width, h=info.height, frame=m.header.frame_id, start_world=ws)))
            S['lethal'][tag] = hit; rng = robot_range(); rec(tag, lethal_in_fixture=hit, lethal_total=int(len(xs)), range=None if rng is None else round(rng, 3))
            if tag == 'local_costmap' and hit > 0 and S['first_lethal'] is None and rng is not None: S['first_lethal'] = round(rng, 3)
        return f
    def tw(k):
        def f(m):
            if k == 'locked': S['locked'] = (m.linear.x, m.angular.z)
            rec(k, vx=round(m.linear.x, 4), wz=round(m.angular.z, 4))
        return f
    def on_gate(m):
        try: g = json.loads(m.data)
        except Exception: return
        stt = g.get('state') or g.get('status')
        if stt != S['gate']: rec('gate', state=stt, reasons=g.get('fault_reasons'))
        if S.get('goal_t') and stt not in ('NORMAL', 'STALL_PENDING'): S['gate_fault_seen'] = True
        S['gate'] = stt
    def on_depth_status(m):
        S['depth_status'] = m.data; S['depth_status_t'] = time.monotonic()
        try: code = json.loads(m.data).get('code')
        except Exception: code = m.data[:40]
        if code != S.get('depth_code'): rec('depth_status', code=code); S.setdefault('depth_codes', []).append([round(time.monotonic(), 3), code])
        S['depth_code'] = code
    n.create_subscription(Odometry, '/ground_truth/world_odom', on_truth, 20)
    n.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    n.create_subscription(PointCloud2, '/camera/depth/points', cloud_counter('raw', 'depth_camera_link'), qos_profile_sensor_data)
    n.create_subscription(PointCloud2, '/depth_obstacles/points', cloud_counter('filter', None), qos_profile_sensor_data)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(OccupancyGrid, '/local_costmap/costmap', grid_counter('local_costmap'), latched)
    n.create_subscription(OccupancyGrid, '/global_costmap/costmap', grid_counter('global_costmap'), latched)
    for k, t in (('smoothed', '/cmd_vel_raw'), ('locked', '/cmd_vel_locked')): n.create_subscription(Twist, t, tw(k), 20)
    n.create_subscription(String, '/stall_detector/status_json', on_gate, 10)
    n.create_subscription(String, '/auto_mapping/depth_status', on_depth_status, 10)
    ex = MultiThreadedExecutor(3); ex.add_node(n); th = threading.Thread(target=ex.spin, daemon=True); th.start()
    out = {}
    def depth_ready():
        now = time.monotonic()
        ok_status = bool(S['depth_status']) and 'DEPTH_READY' in S['depth_status'] and now - (S['depth_status_t'] or -9) < 0.5
        fresh = S['depth_pts_t'] is not None and now - S['depth_pts_t'] < 0.5
        try: tfb.lookup_transform('base_footprint', 'depth_camera_link', rclpy.time.Time()); tf_ok = True
        except Exception: tf_ok = False
        subs = sorted(i.node_name for i in n.get_subscriptions_info_by_topic('/depth_obstacles/points'))
        cm_ok = any('local_costmap' in s for s in subs) and any('global_costmap' in s for s in subs)
        return dict(status=ok_status, fresh=fresh, tf=tf_ok, costmaps_subscribed=cm_ok, subscribers=subs)
    try:
        ac = ActionClient(n, NavigateToPose, 'navigate_to_pose'); t_end = time.monotonic() + 180
        while time.monotonic() < t_end:
            base = S['gate'] == 'NORMAL' and S['truth'] and ac.server_is_ready()
            if base and (not depth or all(v for k, v in depth_ready().items() if k != 'subscribers')): break
            time.sleep(.5)
        out['ready_s'] = round(180 - (t_end - time.monotonic()), 1); out['gate_at_start'] = S['gate']
        # Pre-test only: if the Gate latched during start-up (e.g. depth stale while the stack came up) and the inputs are healthy NOW,
        # clear it through its own operator unlatch service (recorded, max 3). Faults DURING the test are never cleared.
        out['pre_test_resets'] = []
        if S['gate'] != 'NORMAL' and ac.server_is_ready() and (not depth or all(v for k, v in depth_ready().items() if k != 'subscribers')):
            from std_srvs.srv import Trigger
            cli = n.create_client(Trigger, '/stall_detector/reset_latch')
            for _ in range(3):
                if S['gate'] == 'NORMAL' or not cli.wait_for_service(timeout_sec=3): break
                f = cli.call_async(Trigger.Request()); tq = time.monotonic()
                while not f.done() and time.monotonic() - tq < 5: time.sleep(.05)
                r_ = f.result() if f.done() else None
                out['pre_test_resets'].append([S['gate'], bool(r_ and r_.success), (r_.message[:80] if r_ else None)]); time.sleep(3.0)
        S['filter_gap_max'] = 0.0; gate_states_before = S['gate']
        out['depth_readiness'] = depth_ready() if depth else 'N/A (lidar config)'
        out['depth_codes_startup'] = S.get('depth_codes', [])[-10:]
        if not ac.server_is_ready() or S['gate'] != 'NORMAL': out['result'] = 'NOT_READY'; return out
        if depth and not all(v for k, v in out['depth_readiness'].items() if k != 'subscribers'):
            out['result'] = 'SKIP_DEPTH_NOT_READY'; return out
        w0, s0 = time.monotonic(), n.get_clock().now().nanoseconds * 1e-9
        time.sleep(6.0)                                                      # stationary: costmaps and filter settle
        out['rtf_stationary'] = round((n.get_clock().now().nanoseconds * 1e-9 - s0) / (time.monotonic() - w0), 3)
        out['load_avg'] = open('/proc/loadavg').read().split()[:3]
        out['stationary'] = dict(range_m=round(robot_range(), 3) if robot_range() else None, **{k: S['counts'].get(k) for k in ('raw', 'filter', 'lidar')},
                                 **{k + '_lethal': S['lethal'].get(k) for k in ('local_costmap', 'global_costmap')},
                                 floor_z_base_footprint=round(float(np.median(S['floor_z'])), 4) if S['floor_z'] else None,
                                 depth_status=S['depth_status'][:160] if S['depth_status'] else None)
        if a.survey:
            out['result'] = 'SURVEY'; return out
        g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'
        gx, gy = LANE['goal'][0] - st[0], LANE['goal'][1] - st[1]; c, s = math.cos(-st[2]), math.sin(-st[2])
        g.pose.pose.position.x, g.pose.pose.position.y = c * gx - s * gy, s * gx + c * gy; g.pose.pose.orientation.w = 1.0
        rec('goal_sent'); t0 = time.monotonic(); S['goal_t'] = t0; fut = ac.send_goal_async(g)
        while not fut.done() and time.monotonic() - t0 < 10: time.sleep(.05)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted: out['result'] = 'GOAL_REJECTED'; return out
        rfut = gh.get_result_async(); blocked = 0.0; blk_since = None; min_clear = 9.9; fault_t = None; stopped_after_fault = None
        while not rfut.done() and time.monotonic() - t0 < a.timeout:
            tr = S['truth']
            if tr:
                cl = foot_clearance(tr[0], tr[1], tr[2], boxes) if int(time.monotonic() * 20) % 2 == 0 else None
                if cl is not None: min_clear = min(min_clear, cl)
                if abs(S['locked'][0]) > 0.04 and tr[3] < 0.01: blk_since = blk_since or time.monotonic()
                else:
                    if blk_since and time.monotonic() - blk_since > 0.5: blocked += time.monotonic() - blk_since
                    blk_since = None
                rng = robot_range()
                if a.config == 'depth_fault' and fault_t is None and rng is not None and rng < 1.0:
                    os.killpg(procs['depth'].pid, signal.SIGSTOP); fault_t = time.monotonic(); rec('depth_fault_injected', range=round(rng, 3))
                if fault_t and stopped_after_fault is None and abs(S['locked'][0]) < 1e-6 and tr[3] < 0.01:
                    stopped_after_fault = round(time.monotonic() - fault_t, 3)
            if fault_t and time.monotonic() - fault_t > 8: break
            time.sleep(.05)
        status = rfut.result().status if rfut.done() else None
        out['nav2'] = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, f'STATUS_{status}' if status is not None else 'TIMEOUT_OR_STOPPED')
        if not rfut.done(): gh.cancel_goal_async(); time.sleep(2)
        out['duration_s'] = round(time.monotonic() - t0, 1); out['min_clear_m'] = None if min_clear > 9 else round(min_clear, 3)
        out['blocked_s'] = round(blocked, 2); out['first_filter_hit_range_m'] = S['first_filter']; out['first_lethal_range_m'] = S['first_lethal']
        out['gate_final'] = S['gate']; out['fault_injected'] = fault_t is not None; out['stop_after_fault_s'] = stopped_after_fault
        out['filter_gap_max_s'] = round(S.get('filter_gap_max', 0.0), 3); out['depth_codes'] = S.get('depth_codes', [])[-12:]
        out['gate_faulted_during_test'] = S.get('gate_fault_seen', False)
        gw = (SPAWN[0] + LANE['goal'][0], SPAWN[1] + LANE['goal'][1])
        if S['truth']: out['final_err_m'] = round(math.hypot(S['truth'][0] - gw[0], S['truth'][1] - gw[1]), 3)
        if a.config == 'depth_fault':
            out['result'] = 'PASS' if (fault_t and stopped_after_fault is not None and stopped_after_fault < 1.0 and (out['min_clear_m'] or 0) > 0) else 'FAIL'
        else:
            contact = (out['min_clear_m'] is not None and out['min_clear_m'] <= 0.0) or out['blocked_s'] > 0
            out['contact'] = contact
            safe = not contact and (out['min_clear_m'] is None or out['min_clear_m'] >= 0.05)
            # PASS_AVOIDED: reached the goal past the fixture without contact. PASS_STOPPED: no contact, goal not reached (the
            # planner/controller refused or stopped short). Both are safe; only PASS_AVOIDED shows an avoidance manoeuvre.
            out['result'] = ('PASS_AVOIDED' if out['nav2'] == 'SUCCEEDED' else 'PASS_STOPPED') if safe else 'FAIL'
            if safe and out['nav2'] != 'SUCCEEDED' and out['gate_faulted_during_test'] and (out['min_clear_m'] or 0) > 0.5:
                # 11-22 campaign: Gate STALL_LATCHED ~1.5 s after the goal, >1.3 m before the fixture (ICP lidar_speed ~0 while the
                # robot moved). The obstacle response was never exercised, so this is not a PASS.
                out['result'] = 'INCONCLUSIVE_GATE_STOP'
            elif depth and out['gate_faulted_during_test'] and safe and out['nav2'] != 'SUCCEEDED':
                out['result'] = 'FAIL_DEPTH_FAULT'      # stopped because the safety chain faulted, not because the obstacle was handled
        return out
    finally:
        rows.close(); ex.shutdown(); n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
