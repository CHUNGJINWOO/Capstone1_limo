"""TEST ONLY timeline recorder for isolated Phase 3 runs (never on a robot). Writes one JSON line every 0.2 s to argv[3].

Usage: timeline_recorder.py <fleet_url> <operator_token_file> <out.jsonl>   (ROS_LOCALHOST_ONLY=1 and a private ROS_DOMAIN_ID required)
Fleet (HTTP): mode / transition / operation_revision, Agent and Gate boot ids, Gate status / fault epoch / block reasons / reset readiness,
  watchdog, arbiter, localization (status, match, AMCL pose + covariance), owned goal, server safety_context first failing check,
  gate_boot_reconciliation flag.
ROS (direct): /amcl_pose stamp age + covariance, /scan receipt and stamp age, TF map->base_footprint and odom->base_footprint ages,
  /cmd_vel_locked last sample / last non-zero time, NavigateToPose status array, publisher counts with graph-query latency,
  /stall_detector/status_json state, and this process's own timer jitter (callback-delay proxy).
"""
import json, math, os, sys, threading, time, urllib.request
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0', '77')
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

URL, TOKEN_FILE, OUTF = sys.argv[1], sys.argv[2], sys.argv[3]
TOKEN = open(TOKEN_FILE).read().strip()
TOPICS = ('/cmd_vel_raw', '/cmd_vel_locked', '/cmd_vel', '/navigate_to_pose/_action/status', '/stall_detector/status_json', '/scan', '/amcl_pose')


def fleet():
    t0 = time.monotonic()
    req = urllib.request.Request(URL + '/api/v1/robots', headers={'Authorization': 'Bearer ' + TOKEN})
    with urllib.request.urlopen(req, timeout=1.5) as r:
        robots = json.loads(r.read())['robots']
    return robots[0] if robots else {}, round((time.monotonic() - t0) * 1000, 1)


def yaw(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Rec:
    def __init__(self, n):
        self.n = n; self.lock = threading.Lock()
        self.amcl = None; self.scan = None; self.locked = None; self.locked_nz = None; self.nav = None; self.gate = None
        self.tf = tf2_ros.Buffer(cache_time=Duration(seconds=10)); self.tfl = tf2_ros.TransformListener(self.tf, n)
        n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        n.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        n.create_subscription(Twist, '/cmd_vel_locked', self.on_locked, 50)
        n.create_subscription(String, '/stall_detector/status_json', self.on_gate, 10)
        n.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_nav,
                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.f = open(OUTF, 'x'); self.last_tick = None
        n.create_timer(.2, self.tick)

    def now_ros(self): return self.n.get_clock().now().nanoseconds * 1e-9
    def on_amcl(self, m):
        c = m.pose.covariance
        with self.lock: self.amcl = (time.monotonic(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation), c[0], c[7], c[35])
    def on_scan(self, m):
        with self.lock: self.scan = (time.monotonic(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    def on_locked(self, m):
        now = time.monotonic()
        with self.lock:
            self.locked = (now, m.linear.x, m.angular.z)
            if abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6: self.locked_nz = now
    def on_gate(self, m):
        try: d = json.loads(m.data)
        except ValueError: return
        with self.lock: self.gate = (time.monotonic(), {k: d.get(k) for k in ('state', 'boot_id', 'fault_epoch', 'fault_reasons', 'last_fault_reasons', 'reset_readiness', 'last_reset', 'not_ready')} | {'cmd_input': (d.get('cmd_input') or {}).get('state')})
    def on_nav(self, m):
        with self.lock: self.nav = (time.monotonic(), [(bytes(s.goal_info.goal_id.uuid).hex()[:12], s.status) for s in m.status_list][-4:])

    def tf_age(self, target, source):
        try:
            tr = self.tf.lookup_transform(target, source, Time())
            return round(self.now_ros() - (tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9), 3)
        except Exception as e:
            return type(e).__name__

    def tick(self):
        mono = time.monotonic()
        jitter = None if self.last_tick is None else round((mono - self.last_tick - .2) * 1000, 1)
        self.last_tick = mono
        rec = {'t': round(time.time(), 3), 'mono': round(mono, 3), 'timer_jitter_ms': jitter}
        graph = {}
        for tp in TOPICS:
            t0 = time.monotonic()
            try:
                pubs = self.n.get_publishers_info_by_topic(tp)
                graph[tp] = [len(pubs), round((time.monotonic() - t0) * 1000, 2)]
            except Exception as e:
                graph[tp] = [type(e).__name__, None]
        rec['graph_pubs_ms'] = graph
        rec['tf_age'] = {'map>base': self.tf_age('map', 'base_footprint'), 'odom>base': self.tf_age('odom', 'base_footprint')}
        ros_now = self.now_ros()
        with self.lock:
            a, sc, lk, nz, nav, g = self.amcl, self.scan, self.locked, self.locked_nz, self.nav, self.gate
        rec['amcl'] = None if not a else {'rx_age': round(mono - a[0], 2), 'stamp_age': round(ros_now - a[1], 2), 'x': round(a[2], 3), 'y': round(a[3], 3),
                                          'yaw': round(a[4], 3), 'cov': [round(a[5], 4), round(a[6], 4), round(a[7], 4)]}
        rec['scan'] = None if not sc else {'rx_age': round(mono - sc[0], 3), 'stamp_age': round(ros_now - sc[1], 3)}
        rec['cmd_vel_locked'] = None if not lk else {'rx_age': round(mono - lk[0], 3), 'v': round(lk[1], 3), 'w': round(lk[2], 3),
                                                     'last_nonzero_age': None if nz is None else round(mono - nz, 3)}
        rec['nav_status'] = None if not nav else {'rx_age': round(mono - nav[0], 1), 'goals': nav[1]}
        rec['gate_topic'] = None if not g else dict(g[1], rx_age=round(mono - g[0], 3))
        try:
            r, ms = fleet(); st = r.get('state') or {}; om = r.get('operating_mode') or {}; gt = st.get('safety_gate') or {}
            loc = st.get('localization') or {}; ex = st.get('execution') or {}; act = ex.get('active') or {}
            rec['fleet'] = {'http_ms': ms, 'mode': om.get('mode'), 'transition': (om.get('transition') or {}).get('state'),
                            'operation_revision': st.get('operation_revision'), 'agent_boot': (r.get('agent') or {}).get('boot_id'),
                            'online': r.get('online'), 'reconciled': r.get('reconciled'), 'reconciliation_required': r.get('reconciliation_required'),
                            'gate': {k: gt.get(k) for k in ('status', 'boot_id', 'fault_epoch', 'fault_latched', 'movement_permitted', 'movement_block_reasons', 'r7_invalid')},
                            'watchdog': (st.get('motion_watchdog') or {}).get('state'), 'arbiter': (st.get('velocity_arbiter') or {}).get('blocker'),
                            'arbiter_source': (st.get('velocity_arbiter') or {}).get('source'),
                            'loc': {k: loc.get(k) for k in ('status', 'match_ratio', 'detail')}, 'owned_goal': act.get('goal_uuid'),
                            'nav2_active': (ex.get('nav2_goals') or {}).get('active_goal_uuids'),
                            'safety_context_reason': (r.get('safety_context_reason') or {}).get('reason'),
                            'gate_boot_reconciliation': r.get('gate_boot_reconciliation')}
        except Exception as e:
            rec['fleet'] = {'error': type(e).__name__}
        self.f.write(json.dumps(rec, default=str) + '\n'); self.f.flush()


rclpy.init(); node = rclpy.create_node('phase3_timeline_recorder'); Rec(node)
ex = MultiThreadedExecutor(num_threads=2); ex.add_node(node)
try: ex.spin()
except KeyboardInterrupt: pass
