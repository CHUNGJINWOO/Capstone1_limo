#!/usr/bin/env python3
"""Fault injection on the REAL bringup launch's safety actions, isolated ROS, mock driver (G2 Phase A, 2026-11-23). Not a real-robot result.

The real launch (limo_integrated_auto_mapping_real.launch.py) cannot run here (vendor limo_base / ydlidar / dabai are not installed). This
harness imports that launch file, generates its LaunchDescription and runs ONLY these of its actions: the limo_base GroupAction's two
SetRemap rules (with a mock driver Node in place of the vendor include), the motion_watchdog Node and the stall_detector_node (Gate) Node.
So the driver remap, the Watchdog and the Gate parameters are the ones in the launch file; everything else is a stand-in:
  fake Agent (heartbeat + the REAL SpeedLimiter and WatchdogSpeedLink) + fake NavigateToPose action server (idle proof) + synthetic scan/odom,
  all in this process. HOME is redirected to a temporary directory, so the Watchdog's persisted speed limits never touch the user's home.
Run:  unshare -Urn  (ROS env, CAPSTONE_PRIVATE_ROS_TEST=1)  python3 real_launch_chain_faults_20261123.py OUT_DIR
"Delivery stops" below means: no further non-zero command reaches the driver's input topic. It says nothing about a physical motor
(vendor limo_base, STM32 command time-out and motor braking are NOT tested here).
"""
import json, math, os, signal, subprocess, sys, tempfile, threading, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REAL = ROOT / 'capstone_ws/src/limo_capstone/launch/limo_integrated_auto_mapping_real.launch.py'
MOCK = Path(__file__).resolve()
DRIVER = '/limo_driver/cmd_vel'
HOME = tempfile.mkdtemp(prefix='wd_home_')
STATE_FILE = os.path.join(HOME, '.config/limo_fleet/watchdog_speed_limits.json')
sys.path[:0] = [str(ROOT / 'fleet_control/agent/limo_robot_agent')]


def launch_child(unlocked):
    import importlib.util
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory as real
    from launch import LaunchDescription, LaunchService
    from launch.actions import GroupAction, SetLaunchConfiguration
    from launch_ros.actions import Node, SetRemap
    tmp = tempfile.mkdtemp()

    def fake(n):
        try:
            return real(n)
        except PackageNotFoundError:
            d = os.path.join(tmp, n); os.makedirs(os.path.join(d, 'launch'), exist_ok=True); return d
    spec = importlib.util.spec_from_file_location('real_launch', REAL); mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod); mod.get_package_share_directory = fake
    ld = mod.generate_launch_description()
    g0 = [g for g in ld.entities if isinstance(g, GroupAction)][0]            # the limo_base group (first GroupAction in the launch)
    remaps = [e for e in g0.get_sub_entities() if isinstance(e, SetRemap)]
    assert len(remaps) == 2, 'driver remap rules not found in the real launch'
    wd = [n for n in ld.entities if isinstance(n, Node) and n.node_executable == 'motion_watchdog']
    gate = [n for n in ld.entities if isinstance(n, Node) and n.node_executable == 'stall_detector_node']
    assert len(wd) == 1 and len(gate) == 1, (len(wd), len(gate))
    driver = GroupAction(actions=remaps + [Node(package=None, executable=str(MOCK), name='mock_limo_base', arguments=['--mock-driver'], output='log')])
    cfg = [SetLaunchConfiguration('allow_motor_drive', 'true' if unlocked else 'false'), SetLaunchConfiguration('use_depth_camera', 'false'),
           SetLaunchConfiguration('use_sim_time', 'false')]
    svc = LaunchService(); svc.include_launch_description(LaunchDescription(
        [a for a in ld.entities if type(a).__name__ == 'DeclareLaunchArgument'] + cfg + [driver, wd[0], gate[0]]))
    sys.exit(svc.run())


def mock_driver():
    import rclpy
    from geometry_msgs.msg import Twist
    rclpy.init(); n = rclpy.create_node('mock_limo_base')
    n.create_subscription(Twist, 'cmd_vel', lambda m: None, 10)        # relative name, like the vendor driver; the launch remap must move it
    rclpy.spin(n)


class Env:
    """Fake Agent + fake NavigateToPose server + synthetic sensors + recorder, in this process."""
    def __init__(self, domain_tag):
        import rclpy
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionServer
        from rclpy.executors import MultiThreadedExecutor
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        self.rclpy, self.Twist, self.String = rclpy, Twist, String
        rclpy.init(); self.n = rclpy.create_node('fleet_robot_agent')      # the real Agent's node name: the Watchdog's allow-list for /fleet/speed_limit
        n = self.n
        self.hb_on, self.hb_age, self.boot = True, 0.01, 'agent-boot-1'
        self.nav_node = rclpy.create_node('velocity_smoother')     # the allow-listed Watchdog input writer in the real launch
        self.nav_pub = self.nav_node.create_publisher(Twist, '/fleet/nav_cmd_vel', 10)
        self.hbp = n.create_publisher(String, '/fleet/agent_heartbeat', 10)
        self.scan, self.odom = n.create_publisher(LaserScan, '/scan', 10), n.create_publisher(Odometry, '/odom', 10)
        self.odom2, self.odom3 = n.create_publisher(Odometry, '/odom_nav2', 10), n.create_publisher(Odometry, '/odometry/lidar_odom', 10)
        self.rows, self.gate, self.wd, self.locked_rows, self.lock = [], {}, {}, [], threading.Lock()
        n.create_subscription(Twist, DRIVER, lambda m: self._rec(self.rows, m), 50)
        n.create_subscription(Twist, '/cmd_vel_locked', lambda m: self._rec(self.locked_rows, m), 50)
        n.create_subscription(String, '/stall_detector/status_json', lambda m: self._js(self.gate, m), 10)
        n.create_subscription(String, '/fleet/motion_watchdog/status', lambda m: self._js(self.wd, m), 10)
        self.srv = ActionServer(n, NavigateToPose, 'navigate_to_pose', lambda g: NavigateToPose.Result())
        self.send_nav, self.nav_v, self.nav_w, self.extra = True, 0.08, 0.0, []
        self.wd_status = None
        n.create_subscription(String, '/fleet/motion_watchdog/status', lambda m: setattr(self, 'wd_status', (time.monotonic(), json.loads(m.data))), 10)
        self.sess = {'connected': True, 'reconciliation_required': False, 'server_boot_id': 'sb1', 'agent_boot_id': self.boot,
                     'lease': {'epoch': 5, 'expires_at': time.time() + 3600}, 'protocol_version': 2, 'safety_contract_version': 1,
                     'contract_generation': 3, 'speed_limit_request': None,
                     'operating_mode': {'mode': 'MAPPING', 'revision': 1, 'epoch': 5}}      # the real launch's Watchdog has the mode gate: the fake Agent states MAPPING with an owned Frontier goal
        self.limiter = self.link = None
        self.make_agent(self.boot, fresh=True)
        n.create_timer(0.05, self.tick); n.create_timer(0.1, self.sensors)
        self.ex = MultiThreadedExecutor(4); self.ex.add_node(n); self.ex.add_node(self.nav_node)
        self.th = threading.Thread(target=self.ex.spin, daemon=True); self.th.start()

    def make_agent(self, boot, fresh=False):
        """(Re)start the Agent-side speed-limit machinery: a new boot id and a limiter that loads its own persisted file, like a restart."""
        from limo_robot_agent.speed_limits import SpeedLimiter
        from limo_robot_agent.watchdog_speed_link import WatchdogSpeedLink
        if self.link is not None:
            self.link.close()
        self.boot = boot; self.sess['agent_boot_id'] = boot; self.sess['speed_limit_request'] = None
        path = os.path.join(HOME, 'agent_speed_limits.json')
        if fresh and os.path.exists(path):
            os.remove(path)
        self.limiter = SpeedLimiter(path)
        agent = type('A', (), {'session_snapshot': lambda _s: dict(self.sess), 'is_execution_allowed': lambda _s, max_age_s=2.0: True})()
        robot = type('R', (), {'create_publisher': lambda _s, *a, **k: self.n.create_publisher(*a, **k),
                               'create_timer': lambda _s, *a, **k: self.n.create_timer(*a, **k), 'destroy_publisher': lambda _s, p: self.n.destroy_publisher(p),
                               'motion_watchdog': property(lambda _s: self.wd_status), 'active': {'owner': 'frontier', 'cancel_requested': False}})()
        self.link = WatchdogSpeedLink(robot, agent, self.limiter)

    def request(self, gen, scope, lin, ang, rid=None):
        self.sess['speed_limit_request'] = {'generation': gen, 'request_id': rid or f'r{gen}', 'scope': scope, 'max_linear_mps': lin, 'max_angular_rps': ang,
                                            'lease_epoch': self.sess['lease']['epoch'], 'server_boot_id': 'sb1', 'agent_boot_id': self.boot}

    def _rec(self, lst, m):
        with self.lock:
            lst.append((time.monotonic(), m.linear.x, m.angular.z))

    def _js(self, d, m):
        try:
            d.clear(); d.update(json.loads(m.data)); d['_t'] = time.monotonic()
        except ValueError:
            pass

    def tick(self):
        if self.hb_on:
            self.hbp.publish(self.String(data=json.dumps({'boot_id': self.boot, 'healthy': True, 'tasks_age_s': self.hb_age, 'goal_uuids': []})))
        if self.send_nav:
            t = self.Twist(); t.linear.x = self.nav_v; t.angular.z = self.nav_w; self.nav_pub.publish(t)

    def sensors(self):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        st = self.n.get_clock().now().to_msg()
        for p in (self.odom, self.odom2, self.odom3):
            o = Odometry(); o.header.stamp, o.header.frame_id, o.child_frame_id = st, 'odom', 'base_footprint'; o.pose.pose.orientation.w = 1.; p.publish(o)
        s = LaserScan(); s.header.stamp, s.header.frame_id = st, 'laser'; s.angle_min, s.angle_max = -math.pi, math.pi
        s.angle_increment = 2 * math.pi / 359; s.range_min, s.range_max, s.scan_time = .12, 10., .1
        s.ranges = [2. + .3 * math.cos(i * .021) for i in range(360)]; self.scan.publish(s)

    def nonzero_since(self, t, rows=None, thr=1e-6):
        with self.lock:
            return [r for r in (rows or self.rows) if r[0] >= t and (abs(r[1]) > thr or abs(r[2]) > thr)]

    def last_nonzero(self, rows=None):
        r = self.nonzero_since(0, rows)
        return r[-1][0] if r else None

    def close(self):
        self.ex.shutdown(); self.n.destroy_node(); self.nav_node.destroy_node()
        try:
            self.rclpy.shutdown()
        except Exception:
            pass


def children(pid):
    out = subprocess.run(['ps', '-eo', 'pid=,ppid=,args='], capture_output=True, text=True).stdout.splitlines()
    rows = [l.split(None, 2) for l in out]; kids, todo = {}, [str(pid)]
    while todo:
        p = todo.pop()
        for r in rows:
            if len(r) == 3 and r[1] == p and r[0] not in kids:
                kids[r[0]] = r[2]; todo.append(r[0])
    return kids


STACKS = []


@__import__('atexit').register
def _stop_all_stacks():                       # a crashed run must not leave launch processes behind
    for st in STACKS:
        st.stop()


class Stack:
    def __init__(self, unlocked=True):
        STACKS.append(self)
        self.p = subprocess.Popen([sys.executable, str(MOCK), '--launch', 'unlocked' if unlocked else 'locked'], env=dict(os.environ, HOME=HOME),
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def pid_of(self, key):
        for pid, args in children(self.p.pid).items():
            if key in args and 'ps -eo' not in args:
                return int(pid)

    def stop(self):
        try:
            os.killpg(self.p.pid, signal.SIGTERM); self.p.wait(15)
        except Exception:
            try: os.killpg(self.p.pid, signal.SIGKILL)
            except Exception: pass


def wait(pred, secs=20, step=.1):
    end = time.monotonic() + secs
    while time.monotonic() < end:
        v = pred()
        if v:
            return v
        time.sleep(step)
    return None


def rogue_json(topic, payload, secs, name):
    code = (f"import rclpy,time,json\nfrom std_msgs.msg import String\nrclpy.init();n=rclpy.create_node('{name}');p=n.create_publisher(String,'{topic}',10)\n"
            f"e=time.time()+{secs}\nwhile time.time()<e:\n p.publish(String(data=json.dumps({payload!r})));time.sleep(0.05)\n")
    return subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def rogue(topic, v, secs, name):
    code = (f"import rclpy,time\nfrom geometry_msgs.msg import Twist\nrclpy.init();n=rclpy.create_node('{name}');p=n.create_publisher(Twist,'{topic}',10)\n"
            f"e=time.time()+{secs}\nwhile time.time()<e:\n t=Twist();t.linear.x={v};p.publish(t);time.sleep(0.05)\n")
    return subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=False)
    res = []

    def check(name, ok, detail=''):
        res.append(dict(name=name, ok=bool(ok), detail=str(detail))); print(('PASS ' if ok else 'FAIL ') + name + (f' | {detail}' if detail else ''), flush=True)

    shared = []

    def up(unlocked=True, fresh=True):
        if not shared:
            shared.append(Env('x'))                      # one in-process Env (one action server) for all scenarios; only the launch is restarted
        env = shared[0]
        with env.lock:
            env.rows.clear(); env.locked_rows.clear()
        env.gate.clear(); env.wd.clear(); env.send_nav, env.hb_on, env.hb_age, env.nav_v, env.nav_w = True, True, 0.01, 0.08, 0.0
        if fresh:
            if os.path.exists(STATE_FILE):
                os.remove(STATE_FILE)
            env.make_agent('agent-boot-1', fresh=True)
        st = Stack(unlocked)
        ok = wait(lambda: env.wd.get('state') == 'MOTION_AUTHORIZED' and env.gate.get('status') in ('NORMAL', None) and env.gate.get('_t'), 40)
        return env, st, ok

    def down(env, st):
        st.stop(); time.sleep(1.5)

    # --- S0 locked default: authorized Watchdog, valid nav input, driver input must stay silent
    env, st, ok = up(False)
    check('S0 stack authorized (locked)', ok, env.wd.get('state'))
    time.sleep(2.0)
    subs = [i.node_name for i in env.n.get_subscriptions_info_by_topic(DRIVER)]; subs0 = env.n.count_subscribers('/cmd_vel')
    check('S0 mock driver subscribes the driver topic, not /cmd_vel (launch remap works)', 'mock_limo_base' in subs and subs0 == 0, f'driver topic subs={subs} /cmd_vel subs={subs0}')
    check('S0 locked: nothing reaches the driver input', not env.nonzero_since(0) and len(env.rows) == 0, f'driver rows={len(env.rows)}')
    check('S0 locked: Gate output goes to /cmd_vel_locked', bool(env.nonzero_since(0, env.locked_rows)), f'locked nonzero={len(env.nonzero_since(0, env.locked_rows))}')
    check('S0 locked: no publisher on /cmd_vel or the driver topic', env.n.count_publishers('/cmd_vel') == 0 and env.n.count_publishers(DRIVER) == 0)
    down(env, st)

    # --- S1.. unlocked (mock driver only, no hardware)
    env, st, ok = up(True)
    check('S1 stack authorized (unlocked)', ok, env.wd.get('state'))
    time.sleep(1.5)
    check('S1 legitimate path: Watchdog -> Gate -> driver delivers the nav command', len(env.nonzero_since(time.monotonic() - 1.0)) >= 5, f'rows in last 1s={len(env.nonzero_since(time.monotonic() - 1.0))}')
    pubs = [i.node_name for i in env.n.get_publishers_info_by_topic(DRIVER)]
    check('S1 only the Gate writes the driver input', pubs == ['stall_detector_node'], pubs)
    # S2 foreign /cmd_vel writer
    env.send_nav = False; time.sleep(1.0); t0 = time.monotonic(); r = rogue('/cmd_vel', 0.15, 3, 'rogue_cmd_vel'); time.sleep(3.2)
    check('S2 external /cmd_vel writer never reaches the driver input', not env.nonzero_since(t0), f'nonzero rows={len(env.nonzero_since(t0))}'); r.wait()
    # S3 foreign /cmd_vel_raw writer (Gate input) while Watchdog is silent
    t0 = time.monotonic(); r = rogue('/cmd_vel_raw', 0.15, 3, 'rogue_raw'); time.sleep(3.2); leaked = env.nonzero_since(t0); r.wait()
    check('S3 external /cmd_vel_raw writer is not passed (Gate input validation)', len(leaked) == 0, f'nonzero rows={len(leaked)}; gate input status={ (env.gate.get("input") or env.gate.get("cmd_input") or "")}'[:200])
    check('S3 Gate recorded the cause', 'NOT_EXCLUSIVE' in json.dumps(env.gate) or 'UNEXPECTED' in json.dumps(env.gate), 'faults in gate status json')
    down(env, st)

    # S4 foreign writer directly on the driver input (the one path the Gate cannot block): must be DETECTED, not prevented
    env, st, ok = up(True); env.send_nav = False; time.sleep(1.0)
    t0 = time.monotonic(); r = rogue(DRIVER, 0.15, 3, 'rogue_driver'); time.sleep(2.0); seen = json.dumps(env.gate.get('cmd_input')); time.sleep(1.2); reach = env.nonzero_since(t0); r.wait()
    check('S4 writer on the driver input reaches the driver (known limit: not preventable by the Gate)', len(reach) > 0, f'rows={len(reach)}')
    check('S4 ... and the Gate detects it', 'rogue_driver' in seen and 'output_topic_faults' in seen, seen[:300])
    down(env, st)

    # S5 Watchdog killed -> delivery stops
    env, st, ok = up(True); time.sleep(1.5); pid = st.pid_of('motion_watchdog')
    t0 = time.monotonic(); os.kill(pid, signal.SIGKILL); time.sleep(2.5)
    last = env.last_nonzero(); dt = (last - t0) if last and last > t0 else 0.0
    check('S5 Watchdog killed: non-zero delivery stops', not env.nonzero_since(t0 + 1.0), f'last non-zero {dt:.3f} s after the kill')
    # S5b restart of the Watchdog by hand (the launch does not respawn)
    env.send_nav = True; t1 = time.monotonic()
    w2 = subprocess.Popen(['ros2', 'run', 'limo_robot_agent', 'motion_watchdog'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True); time.sleep(8.0)
    check('S5b restarted Watchdog is NOT trusted by the Gate (restart => WRITER_RESTARTED/other fault, zero until operator reset)',
          not env.nonzero_since(t1 + 1.5), f'nonzero after restart={len(env.nonzero_since(t1 + 1.5))}; gate={json.dumps(env.gate)[:160]}')
    os.killpg(w2.pid, signal.SIGTERM); w2.wait(); down(env, st)

    # S6 Gate killed -> delivery stops
    env, st, ok = up(True); time.sleep(1.5); pid = st.pid_of('stall_detector_node'); t0 = time.monotonic(); os.kill(pid, signal.SIGKILL); time.sleep(2.5)
    last = env.last_nonzero(); dt = (last - t0) if last and last > t0 else 0.0
    check('S6 Gate killed: no further non-zero command is delivered (driver may hold its last value: unverified)', not env.nonzero_since(t0 + 0.5), f'last non-zero {dt:.3f} s after the kill')
    down(env, st)

    # S7 Agent exit (heartbeat stops) / S8 Agent control-loop stall / S9 unhealthy
    for tag, act in (('S7 Agent exit (heartbeat stops)', lambda e: setattr(e, 'hb_on', False)),
                     ('S8 Agent control loop stalled (tasks_age_s large)', lambda e: setattr(e, 'hb_age', 5.0)),
                     ('S9 nav input stops (Arbiter output ends, e.g. Lease released)', lambda e: setattr(e, 'send_nav', False))):
        env, st, ok = up(True); time.sleep(1.5); t0 = time.monotonic(); act(env); time.sleep(2.0)
        last = env.last_nonzero(); dt = (last - t0) if last and last > t0 else 0.0
        check(tag + ': delivery stops', not env.nonzero_since(t0 + 1.0), f'last non-zero {dt:.3f} s after the fault')
        down(env, st)

    # S10 Watchdog frozen (SIGSTOP) -> stale input
    env, st, ok = up(True); time.sleep(1.5); pid = st.pid_of('motion_watchdog'); t0 = time.monotonic(); os.kill(pid, signal.SIGSTOP); time.sleep(2.0)
    last = env.last_nonzero(); dt = (last - t0) if last and last > t0 else 0.0
    check('S10 Watchdog frozen: delivery stops', not env.nonzero_since(t0 + 1.0), f'last non-zero {dt:.3f} s after the freeze'); os.kill(pid, signal.SIGCONT); down(env, st)

    # S11 Watchdog replaced by an impostor of the same node name (graph change)
    env, st, ok = up(True); time.sleep(1.5); pid = st.pid_of('motion_watchdog'); t0 = time.monotonic(); os.kill(pid, signal.SIGKILL)
    r = rogue('/cmd_vel_raw', 0.15, 4, 'fleet_motion_watchdog'); time.sleep(4.2); leaked = env.nonzero_since(t0 + 1.0); r.wait()
    check('S11 impostor named fleet_motion_watchdog (no matching status/GID) is not passed', len(leaked) == 0, f'nonzero rows={len(leaked)}')
    down(env, st)

    # S12 Gate status delay: Gate frozen -> Watchdog loses Gate status -> failsafe, nothing delivered after resume until re-authorized
    env, st, ok = up(True); time.sleep(1.5); pid = st.pid_of('stall_detector_node'); t0 = time.monotonic(); os.kill(pid, signal.SIGSTOP); time.sleep(2.5)
    state_during = env.wd.get('state'); os.kill(pid, signal.SIGCONT); time.sleep(0.2)
    check('S12 Gate frozen (status delayed): Watchdog leaves MOTION_AUTHORIZED', state_during != 'MOTION_AUTHORIZED', f'state={state_during} reason={env.wd.get("reason")}')
    down(env, st)


    # ---------------- Watchdog speed clamp (G2): the enforcement point on a real robot. Real Watchdog node from the launch, real Agent limiter + link.
    def peak_since(env, t0):
        nz = env.nonzero_since(t0)
        return (max([r[1] for r in nz] or [0.0]), max([abs(r[2]) for r in nz] or [0.0]), len(nz))

    def authorized(env):
        return wait(lambda: env.wd.get('state') == 'MOTION_AUTHORIZED' and env.gate.get('_t'), 30)

    env, st, ok = up(True); check('C0 stack authorized with the Agent link keeping the Watchdog synchronised', ok, str(env.wd.get('reason')))
    sc = (env.wd_status or (0, {}))[1].get('speed_clamp') or {}
    check('C0 Watchdog reports the clamp enabled, synchronised, one allowed publisher, no fault', sc.get('enabled') and sc.get('synced') and sc.get('publisher_ok') and sc.get('fault') is None, json.dumps({k: sc.get(k) for k in ('enabled', 'synced', 'publisher_ok', 'fault', 'input_scope')}))
    wait(lambda: env.limiter.enforced_by == 'WATCHDOG', 10); check('C0 the Agent reports enforced_by WATCHDOG only now', env.limiter.enforced_by == 'WATCHDOG')
    env.nav_v, env.nav_w = 0.30, 0.0; time.sleep(1.0); t0 = time.monotonic(); time.sleep(2.0); p = peak_since(env, t0)
    check('C1 positive control: a 0.30 m/s Nav2 command flows and is cut to the static auto ceiling 0.15 (no dynamic limit yet)', p[2] > 20 and 0.149 <= p[0] <= 0.1501, str(p))
    env.nav_v, env.nav_w = 0.30, 0.30; time.sleep(1.0); t0 = time.monotonic(); time.sleep(1.5); p = peak_since(env, t0)
    print(f"   curvature (omega/v): input {0.30/0.30:.2f} 1/m -> output {(p[1]/p[0] if p[0] else float('nan')):.2f} 1/m (linear saturated, angular passed): saturation changes the path curvature")
    check('C1b one axis saturated, the other passed through (no proportional scaling)', abs(p[0] - 0.15) < 1e-3 and abs(p[1] - 0.30) < 1e-3, str(p))
    env.nav_v, env.nav_w = 0.0, 0.0; time.sleep(1.0)
    t_step = time.monotonic(); env.nav_v = 0.30; time.sleep(1.0); rows = [r for r in env.rows if r[0] >= t_step]
    step = max([abs(b[1] - a[1]) for a, b in zip(rows, rows[1:])] or [0.0]); first_nz = next((r for r in rows if r[1] > 1e-6), None)
    print(f"   acceleration: first non-zero sample {first_nz[1] if first_nz else None} m/s after a 0 -> 0.30 step, largest sample-to-sample change {step:.3f} m/s (no ramp added or removed by the clamp)")
    check('C1c the clamp adds no acceleration limit: the output steps straight to the clamped value', first_nz is not None and first_nz[1] >= 0.149, str(first_nz))
    env.nav_v, env.nav_w = 0.30, 0.0
    # request -> pending -> confirmed
    env.request(1, 'auto', 0.06, 0.20); seen = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 8 and (not seen or seen[-1] != 'APPLIED'):
        r = (env.limiter.last_result or {}).get('status')
        if r and (not seen or seen[-1] != r): seen.append(r)
        time.sleep(0.02)
    check('C2 request goes PENDING_ENFORCEMENT before APPLIED (Agent does not claim it applied before the Watchdog shows it)', seen[:2] == ['PENDING_ENFORCEMENT', 'APPLIED'], str(seen))
    wa = ((env.wd_status[1].get('speed_clamp') or {}).get('applied') or {}).get('auto') or {}
    check('C2 the Watchdog status shows the same generation, request id and values', (wa.get('generation'), wa.get('request_id'), wa.get('linear_mps'), wa.get('angular_rps')) == (1, 'r1', 0.06, 0.2), json.dumps(wa))
    env.sess['speed_limit_request'] = None; time.sleep(0.8); t0 = time.monotonic(); time.sleep(2.0); p = peak_since(env, t0)
    check('C2 with an approved limit of 0.06 m/s the driver input follows (flow continues)', p[2] > 20 and 0.059 <= p[0] <= 0.0601, str(p))
    env.request(2, 'auto', 0.20, 0.30); time.sleep(1.0); lr = env.limiter.last_result
    check('C3 a request above the ceiling is rejected by the Agent and the Watchdog value is unchanged', lr['status'] == 'REJECTED' and lr['reason'] == 'EXCEEDS_CEILING' and abs(((env.wd_status[1]['speed_clamp']['applied']['auto'] or {})['linear_mps']) - 0.06) < 1e-9, json.dumps(lr))
    env.sess['speed_limit_request'] = None
    # external writer on the speed-limit topic: the Watchdog must stop, not accept it
    t0 = time.monotonic(); r = rogue_json('/fleet/speed_limit', {'agent_boot_id': env.boot, 'binding': ['sb1', 5, 3], 'seq': 1, 'scopes': {'auto': {'max_linear_mps': 0.15, 'max_angular_rps': 0.5, 'generation': 99, 'request_id': 'evil'}}}, 4, 'rogue_limits')
    # 2026-11-26 (C4 investigation): a 20 Hz timeline from the rogue's start instead of one sleep, so a failure carries the real times (rogue appearance, Watchdog state changes, last fault, clamp sync, first/last forwarded sample)
    tl, last_key = [], None
    obs_n2, obs_first_any, load0 = None, None, os.getloadavg()
    while time.monotonic() - t0 < 3.0:
        try:
            npub = len(env.n.get_publishers_info_by_topic('/fleet/speed_limit'))     # an independent observer in this process: when does the graph show the second writer?
            if npub >= 2 and obs_n2 is None: obs_n2 = time.monotonic()
        except Exception: pass
        wst = dict(env.wd); sc = wst.get('speed_clamp') or {}; key = (wst.get('state'), wst.get('last_fault'), sc.get('publisher_ok'), sc.get('fault'), sc.get('synced'))
        if key != last_key: tl.append((round(time.monotonic() - t0, 2),) + key); last_key = key
        time.sleep(0.05)
    stopped = env.wd.get('state') != 'MOTION_AUTHORIZED'; late = env.nonzero_since(t0 + 1.5)
    fwd = env.nonzero_since(t0)
    sd = (env.wd_status[1].get('sl_diag') or {}) if env.wd_status else {}
    rid_t = (sd.get('first_msg_t') or {}).get('evil')
    print("   C4 diag: observer graph shows 2 writers at +%s s; Watchdog first saw n_pubs>=2 in a message at +%s s; first rogue message ('evil') reached the Watchdog at +%s s; Watchdog pubs-seen histogram %s; Watchdog tick gaps: max %.3f s, >150 ms: %s of %s; load1 %.2f -> %.2f"
          % (None if obs_n2 is None else round(obs_n2 - t0, 3), None if sd.get('first_n2_t') is None else round(sd['first_n2_t'] - t0, 3), None if rid_t is None else round(rid_t - t0, 3), sd.get('n_pubs_hist'),
             sd.get('tick_gap_max', 0), sd.get('tick_gaps_gt_150ms'), sd.get('ticks'), load0[0], os.getloadavg()[0]), flush=True)
    print(f"   C4 timeline (s after the rogue process was started; state, last_fault, clamp publisher_ok, clamp fault, synced): {tl}; first forwarded sample at +{(fwd[0][0] - t0) if fwd else None}, last at +{(fwd[-1][0] - t0) if fwd else None}, rogue alive: {r.poll() is None}", flush=True)
    wa = ((env.wd_status[1].get('speed_clamp') or {}).get('applied') or {}).get('auto') or {}
    check('C4 a second writer on /fleet/speed_limit: the Watchdog stops forwarding (SPEED_LIMIT_NOT_SYNCED) and never takes the raise', stopped and not late and wa.get('linear_mps') == 0.06, f'state={env.wd.get("state")} late_nonzero={len(late)} applied={wa.get("linear_mps")} fault={env.wd.get("last_fault")}'); r.wait()
    ok = authorized(env); time.sleep(1.0); t0 = time.monotonic(); time.sleep(2.0); p = peak_since(env, t0)
    check('C4 after the rogue is gone the Watchdog recovers (re-armed) with the limit still 0.06', ok and p[2] > 20 and p[0] <= 0.0601, str(p))
    down(env, st)

    # Watchdog restart while the Agent link is silent: no motion at all, and the restored limit is the LOWER one
    env, st, ok = up(True, fresh=True); env.nav_v = 0.30; env.request(1, 'auto', 0.06, 0.20); time.sleep(4.0); env.sess['speed_limit_request'] = None
    check('C5 setup: 0.06 applied and persisted by the Watchdog', os.path.exists(STATE_FILE) and json.load(open(STATE_FILE))['applied']['auto']['linear_mps'] == 0.06, STATE_FILE and str(os.path.exists(STATE_FILE)))
    env.link.timer.cancel(); st.stop(); time.sleep(1.5); env.rows.clear(); st = Stack(True); time.sleep(10.0)
    check('C5 Watchdog restarted with the Agent link silent: it stays out of MOTION_AUTHORIZED and nothing non-zero reaches the driver', env.wd.get('state') != 'MOTION_AUTHORIZED' and not env.nonzero_since(0), f'state={env.wd.get("state")} reason={env.wd.get("reason")} rows={len(env.rows)}')
    scs = (env.wd_status[1].get('speed_clamp') or {}); wa = (scs.get('applied') or {}).get('auto') or {}
    check('C5 the restored limit is the lower persisted one (0.06), marked unconfirmed/stale, never the ceiling', wa.get('linear_mps') == 0.06 and wa.get('stale') is True and scs.get('restored_from_file_unconfirmed') is True, json.dumps(wa))
    env.link.timer = env.n.create_timer(0.2, env.link.tick); ok = authorized(env); time.sleep(1.0); t0 = time.monotonic(); time.sleep(2.0); p = peak_since(env, t0)
    check('C5 once the Agent resumes, motion resumes and stays at or below 0.06', ok and p[2] > 20 and p[0] <= 0.0601, str(p))
    # Agent restart: new boot id, new limiter loaded from its file (stale)
    env.make_agent('agent-boot-2'); time.sleep(0.5); ok = authorized(env); t0 = time.monotonic(); time.sleep(2.5); p = peak_since(env, t0)
    check('C6 Agent restart (new boot, limiter reloaded as stale): the Watchdog keeps the lower limit and motion stays <= 0.06', ok and p[2] > 20 and p[0] <= 0.0601 and env.limiter.snapshot()['stale'] is True, f'{p} agent_stale={env.limiter.snapshot()["stale"]}')
    # lease change: old setting stale, a new request under the new lease raises it explicitly
    env.sess['lease'] = {'epoch': 6, 'expires_at': time.time() + 3600}; time.sleep(1.0); t0 = time.monotonic(); time.sleep(2.0); p = peak_since(env, t0)
    check('C7 lease change: output stays at the lower limit (no automatic restore)', p[2] > 20 and p[0] <= 0.0601, str(p))
    env.request(1, 'auto', 0.12, 0.40); time.sleep(4.0); env.sess['speed_limit_request'] = None; t0 = time.monotonic(); time.sleep(2.0); p = peak_since(env, t0)
    check('C7 a new request under the new lease raises it explicitly and the output follows (0.12)', p[2] > 20 and 0.119 <= p[0] <= 0.1201, str(p))
    # contention: limits changing while commands flow
    stop_evt = threading.Event(); gens = [10]
    def churn():
        while not stop_evt.is_set():
            gens[0] += 1; env.request(gens[0], 'auto', 0.03 + (gens[0] % 5) * 0.02, 0.1 + (gens[0] % 4) * 0.1); time.sleep(0.25)
    th = threading.Thread(target=churn); t0 = time.monotonic(); th.start(); time.sleep(6.0); stop_evt.set(); th.join(); env.sess['speed_limit_request'] = None
    mx = max([r[1] for r in env.rows if r[0] >= t0] or [0.0])
    check('C8 limits changing every 0.25 s while commands flow: output never above the highest approved value and never above the ceiling', mx <= 0.15 + 1e-9, f'max linear {mx:.3f} over {len([r for r in env.rows if r[0] >= t0])} samples')
    down(env, st)
    shared[0].close()
    bad = [r for r in res if not r['ok']]
    (out / 'result.json').write_text(json.dumps(dict(passed=len(res) - len(bad), failed=len(bad), results=res), indent=1))
    print(f'{len(res) - len(bad)}/{len(res)} PASS'); sys.exit(1 if bad else 0)


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == '--launch':
        launch_child(sys.argv[2] == 'unlocked')
    elif '--mock-driver' in sys.argv:
        mock_driver()
    else:
        main()
