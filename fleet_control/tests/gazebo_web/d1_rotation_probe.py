"""SIMULATION ONLY (2026-11-22, Claude): D1 probe. Does /odometry/lidar_odom (ICP, lidar_scan_matcher_odom_node, NOT modified)
under-read PURE in-place rotation against Gazebo truth, and why?

record (Gazebo): isolated stack (ROS_LOCALHOST_ONLY=1, private domain, own IGN_PARTITION; no LIMO / 18190 / /cmd_vel):
  gazebo_world.launch.py + web_mapping_isolated/stack.launch.py (ICP node, odom relay, Nav2 idle, Gate r13 candidate; ARBITER=0, WATCHDOG=0).
  The probe is the single /cmd_vel_raw writer -> Gate -> /cmd_vel_locked -> drive relay -> Gazebo. Segments: in-place rotation at
  +w / -w for N turns, with stops. Recorded: every /scan (ranges, stamp), /ground_truth/world_odom, /odometry/lidar_odom,
  /diagnostics/f4_latency, /clock, wheel /odom, receive times (scan.npz + streams.jsonl).
replay (offline, no ROS): the node's OWN on_scan/icp_2d code (imported read-only from capstone_ws) re-run on the recorded scans with
  parameter variants (keyframe threshold, correspondence distance, iterations, laser offset sign, trimming), and compared with truth.
  Separates: algorithm/parameters (replay reproduces the live error), TF (offset sign variant), time sync (stamp vs truth alignment).

usage: python3 d1_rotation_probe.py record --out <new dir> [--domain 200] [--w 0.3,0.5] [--turns 2] [--spawn x,y,yaw]
       python3 d1_rotation_probe.py replay --run <dir> [--out <new json>]
"""
import argparse, json, math, os, signal, subprocess, sys, threading, time
from pathlib import Path

HERE = Path(__file__).resolve().parent; FC = HERE.parents[1]; REPO = FC.parent
GATE_R13 = REPO / 'local_data/raw_results/gate_r13_integ_claude_20261122/candidate_r13/stall_detector_node.py'
ICP_SRC = REPO / 'capstone_ws/src/limo_capstone/limo_capstone/lidar_scan_matcher_odom_node.py'


def unwrap(seq):
    out, prev, acc = [], None, 0.0
    for a in seq:
        if prev is not None: acc += math.atan2(math.sin(a - prev), math.cos(a - prev))
        out.append(acc); prev = a
    return out


def record(a):
    out = Path(a.out); out.mkdir()
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=a.domain, IGN_PARTITION=f'd1_{a.domain}_{os.getpid()}', IGN_IP='127.0.0.1',
               GZ_SPAWN=a.spawn, GZ_DRIVE_TIMEOUT_S='0.2', USE_SIM_TIME='1', ARBITER='0', WATCHDOG='0', LIFECYCLE_DELAY_S='5',
               GATE_SCRIPT=str(GATE_R13), ROS_LOG_DIR=str(out / 'roslog'), ROS2CLI_NO_DAEMON='1')
    for k in ('ROS_DISCOVERY_SERVER', 'GATE_INPUT_EXCL', 'GATE_WRITER_STATUS', 'GATE_RESET_PEER', 'GATE_REQ_WD', 'NAV2_PARAMS_OVERLAY', 'ODOM_ACTIVE_TOPIC'):
        env.pop(k, None)
    procs = []
    def start_p(tag, cmd):
        procs.append(subprocess.Popen(cmd, env=env, stdout=(out / f'{tag}.log').open('x'), stderr=subprocess.STDOUT, start_new_session=True))
    start_p('sim', ['ros2', 'launch', str(HERE / 'gazebo_world.launch.py')]); time.sleep(4)
    start_p('stack', ['ros2', 'launch', str(FC / 'tests/web_mapping_isolated/stack.launch.py')])
    os.environ.update(ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=a.domain); os.environ.pop('ROS_DISCOVERY_SERVER', None)
    try: meta = drive(out, a)
    finally:
        for p in reversed(procs):
            try: os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError: pass
        time.sleep(4)
        for p in procs:
            try: os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError: pass
    (out / 'meta.json').write_text(json.dumps(meta, indent=1))
    print(json.dumps(meta.get('live_summary', meta), indent=1))


def drive(out, a):
    import numpy as np, rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String, Float32MultiArray
    from rosgraph_msgs.msg import Clock
    rclpy.init(); n = rclpy.create_node('d1_rotation_probe')
    rows = (out / 'streams.jsonl').open('x'); lock = threading.Lock(); scans = []; st = {'gate': None, 'clock': None}
    def yaw(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    def stamp(h): return h.stamp.sec + h.stamp.nanosec * 1e-9
    def rec(k, **v):
        with lock: rows.write(json.dumps({'k': k, 'rx': round(time.monotonic(), 4), 'clk': st['clock'], **v}) + '\n')
    def on_scan(m):
        scans.append((stamp(m.header), time.monotonic(), np.asarray(m.ranges, dtype=np.float32)))
        if len(scans) == 1: st['scan_meta'] = {'angle_min': m.angle_min, 'inc': m.angle_increment, 'range_min': m.range_min, 'range_max': m.range_max, 'n': len(m.ranges)}
    n.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    n.create_subscription(Odometry, '/ground_truth/world_odom', lambda m: rec('truth', st=stamp(m.header), x=m.pose.pose.position.x, y=m.pose.pose.position.y, th=yaw(m.pose.pose.orientation)), 50)
    n.create_subscription(Odometry, '/odometry/lidar_odom', lambda m: rec('icp', st=stamp(m.header), x=m.pose.pose.position.x, y=m.pose.pose.position.y, th=yaw(m.pose.pose.orientation)), 50)
    n.create_subscription(Odometry, '/odom', lambda m: rec('wheel', st=stamp(m.header), th=yaw(m.pose.pose.orientation)), 50)
    n.create_subscription(Float32MultiArray, '/diagnostics/f4_latency', lambda m: rec('f4', d=[round(float(x), 3) for x in m.data]), 50)
    n.create_subscription(Clock, '/clock', lambda m: st.__setitem__('clock', round(m.clock.sec + m.clock.nanosec * 1e-9, 3)), 10)
    def on_gate(m):
        try: g = json.loads(m.data)
        except Exception: return
        s = g.get('state') or g.get('status')
        if s != st['gate']: rec('gate', state=s, reasons=g.get('fault_reasons'))
        st['gate'] = s
    n.create_subscription(String, '/stall_detector/status_json', on_gate, 10)
    pub = n.create_publisher(Twist, '/cmd_vel_raw', 10)
    ex = SingleThreadedExecutor(); ex.add_node(n); th = threading.Thread(target=ex.spin, daemon=True); th.start()
    meta = {'spawn': a.spawn, 'segments': []}
    def hold(wz, secs, label):
        rec('seg', label=label, wz=wz, secs=secs); end = time.monotonic() + secs
        while time.monotonic() < end:
            m = Twist(); m.angular.z = wz; pub.publish(m); time.sleep(0.05)
    try:
        t_end = time.monotonic() + 120
        while time.monotonic() < t_end and st['gate'] != 'NORMAL': time.sleep(.5)
        meta['gate_at_start'] = st['gate']
        if st['gate'] != 'NORMAL': meta['result'] = 'NOT_READY'; return meta
        hold(0.0, 6.0, 'settle')
        for w in [float(x) for x in a.w.split(',')]:
            for sgn in (1, -1):
                secs = a.turns * 2 * math.pi / w
                hold(sgn * w, secs, f'rot_{sgn * w:+.2f}'); hold(0.0, 5.0, 'stop')
                meta['segments'].append({'wz': sgn * w, 'secs': round(secs, 2)})
        meta['result'] = 'RECORDED'
    finally:
        ex.shutdown(); n.destroy_node(); rclpy.shutdown(); th.join(timeout=5); rows.close()
        if scans:
            np.savez_compressed(out / 'scans.npz', stamp=np.array([s[0] for s in scans]), rx=np.array([s[1] for s in scans]),
                                ranges=np.stack([s[2] for s in scans]))
            meta['scan_meta'] = st.get('scan_meta')
    meta['live_summary'] = analyse_live(out)
    return meta


def segments(R):
    segs, cur = [], None
    for r in R:
        if r['k'] == 'seg':
            if cur: cur['end'] = r['rx']; segs.append(cur)
            cur = {'label': r['label'], 'wz': r['wz'], 'start': r['rx']}
    if cur: cur['end'] = R[-1]['rx']; segs.append(cur)
    return [s for s in segs if s['label'].startswith('rot_')]


def seg_delta(series, s0, s1, key='rx'):
    xs = [r for r in series if s0 <= r[key] <= s1]
    if len(xs) < 2: return None
    u = unwrap([r['th'] for r in xs]); return u[-1] - u[0]


def analyse_live(out):
    R = [json.loads(l) for l in (out / 'streams.jsonl').open() if l.strip()]
    T = [r for r in R if r['k'] == 'truth']; I = [r for r in R if r['k'] == 'icp']; W = [r for r in R if r['k'] == 'wheel']
    f4 = [r['d'] for r in R if r['k'] == 'f4']
    res = []
    for s in segments(R):
        s0, s1 = s['start'], s['end'] + 3.0             # include the stop after the segment (settled values)
        dt_, di, dw = seg_delta(T, s0, s1), seg_delta(I, s0, s1), seg_delta(W, s0, s1)
        res.append({'seg': s['label'], 'truth_rad': round(dt_, 4) if dt_ else None, 'icp_rad': round(di, 4) if di else None,
                    'icp_err_pct': round(100 * (di - dt_) / dt_, 2) if dt_ and di else None,
                    'wheel_err_pct': round(100 * (dw - dt_) / dt_, 2) if dt_ and dw else None})
    lat = {'icp_compute_ms_p50': None}
    if f4:
        import statistics
        lat = {'icp_compute_ms_p50': round(statistics.median(d[0] for d in f4), 1), 'icp_compute_ms_max': round(max(d[0] for d in f4), 1),
               'sim_pub_delay_ms_p50': round(statistics.median(d[1] for d in f4), 1), 'sim_pub_delay_ms_max': round(max(d[1] for d in f4), 1),
               'scan_period_ms_p50': round(statistics.median(d[2] for d in f4), 1), 'inliers_min': min(d[3] for d in f4),
               'inliers_p50': statistics.median(d[3] for d in f4)}
    gates = [r.get('state') for r in R if r['k'] == 'gate']
    return {'segments': res, 'f4': lat, 'gate_states': gates}


class _Msg:      # minimal LaserScan stand-in for the node's scan_to_points
    def __init__(self, meta, ranges): self.ranges = ranges; self.angle_min = meta['angle_min']; self.angle_increment = meta['inc']; self.range_min = meta['range_min']; self.range_max = meta['range_max']


def replay(a):
    import numpy as np, importlib.util
    run = Path(a.run); meta = json.loads((run / 'meta.json').read_text()); smeta = meta['scan_meta']
    Z = np.load(run / 'scans.npz'); stamps, rx, ranges = Z['stamp'], Z['rx'], Z['ranges']
    spec = importlib.util.spec_from_file_location('icp_node_ro', ICP_SRC); mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    Node = mod.LidarScanMatcherOdomNode
    R = [json.loads(l) for l in (run / 'streams.jsonl').open() if l.strip()]
    T = [r for r in R if r['k'] == 'truth']
    def truth_at_stamp(t):                              # truth by sim stamp (time sync independent of receive order)
        best = min(T, key=lambda r: abs(r['st'] - t)); return best['th'], abs(best['st'] - t)
    variants = {
        'node_default': dict(off=0.103, it=18, md=0.30, kf=0.12, trim=85.0),
        'offset_sign_neg': dict(off=-0.103, it=18, md=0.30, kf=0.12, trim=85.0),
        'offset_zero': dict(off=0.0, it=18, md=0.30, kf=0.12, trim=85.0),
        'kf_0.30rad': dict(off=0.103, it=18, md=0.30, kf=0.30, trim=85.0),
        'kf_every_scan_0.0': dict(off=0.103, it=18, md=0.30, kf=0.0, trim=85.0),
        'iter_50': dict(off=0.103, it=50, md=0.30, kf=0.12, trim=85.0),
        'maxdist_0.15': dict(off=0.103, it=18, md=0.15, kf=0.12, trim=85.0),
        'maxdist_0.60': dict(off=0.103, it=18, md=0.60, kf=0.12, trim=85.0),
        'no_trim': dict(off=0.103, it=18, md=0.30, kf=0.12, trim=100.0),
        # 2026-11-22 second pass: combinations and a constant-velocity initial guess (replay loop only; the node is not modified)
        'kf_0.30_no_trim': dict(off=0.103, it=18, md=0.30, kf=0.30, trim=100.0),
        'cv_init': dict(off=0.103, it=18, md=0.30, kf=0.12, trim=85.0, cv=True),
        'cv_init_kf_0.30_no_trim': dict(off=0.103, it=18, md=0.30, kf=0.30, trim=100.0, cv=True),
    }
    if a.variants: variants = {k: v for k, v in variants.items() if k in a.variants.split(',')}
    out = {'run': str(run), 'n_scans': int(len(stamps)), 'variants': {}}
    for name, v in variants.items():
        node = Node.__new__(Node)                       # algorithm only: no rclpy node is created
        node.laser_x_offset, node.max_iter, node.max_dist = v['off'], v['it'], v['md']
        node.last_inliers = 350.0
        kf_pts, T_okf, T_kfc = None, np.eye(3), np.eye(3); yaws = []; last_step = np.eye(3); prev_oc = np.eye(3)
        src = Path(ICP_SRC).read_text()
        pct = v['trim']
        for i in range(len(stamps)):
            pts = node.scan_to_points(_Msg(smeta, ranges[i]))
            if len(pts) < 20: yaws.append(yaws[-1] if yaws else 0.0); continue
            if kf_pts is None: kf_pts = pts; yaws.append(0.0); continue
            if v.get('cv'): T_kfc = T_kfc @ last_step          # predict this scan's motion = the previous scan-to-scan step
            if pct >= 100.0:
                orig = np.percentile; np.percentile = lambda x, q: np.max(x)
                try: T_kfc = node.icp_2d(pts, kf_pts, T_kfc)
                finally: np.percentile = orig
            else: T_kfc = node.icp_2d(pts, kf_pts, T_kfc)
            T_oc = T_okf @ T_kfc
            last_step = np.linalg.inv(prev_oc) @ T_oc; prev_oc = T_oc.copy()
            dx, dy, dyaw = mod.mat_to_se2(T_kfc)
            if math.hypot(dx, dy) >= v['kf'] or abs(dyaw) >= v['kf']:
                kf_pts = pts; T_okf = T_oc.copy(); T_kfc = np.eye(3)
            yaws.append(mod.mat_to_se2(T_oc)[2])
        segs = []
        for s in segments(R):
            idx = [i for i in range(len(rx)) if s['start'] <= rx[i] <= s['end'] + 3.0]
            if len(idx) < 3: continue
            u = unwrap([yaws[i] for i in idx]); tr = unwrap([truth_at_stamp(stamps[i])[0] for i in idx])
            di, dt_ = u[-1] - u[0], tr[-1] - tr[0]
            segs.append({'seg': s['label'], 'truth_rad': round(dt_, 4), 'icp_rad': round(di, 4), 'err_pct': round(100 * (di - dt_) / dt_, 2) if dt_ else None})
        out['variants'][name] = {'params': v, 'segments': segs}
        print(name, [(x['seg'], x['err_pct']) for x in segs], flush=True)
    sync = [truth_at_stamp(t)[1] for t in stamps[:: max(1, len(stamps) // 200)]]
    out['truth_stamp_match_max_s'] = round(max(sync), 4)
    if a.out:
        with open(a.out, 'x') as f: json.dump(out, f, indent=1)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); sp = ap.add_subparsers(dest='cmd', required=True)
    r = sp.add_parser('record'); r.add_argument('--out', required=True); r.add_argument('--domain', default='200')
    r.add_argument('--w', default='0.3,0.5'); r.add_argument('--turns', type=float, default=2.0); r.add_argument('--spawn', default='10.09,-6.98,0')
    p = sp.add_parser('replay'); p.add_argument('--run', required=True); p.add_argument('--out', default=''); p.add_argument('--variants', default='')
    a = ap.parse_args()
    if a.cmd == 'record':
        assert 200 <= int(a.domain) <= 209
        record(a)
    else: replay(a)
