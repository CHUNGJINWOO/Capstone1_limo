#!/usr/bin/env python3
"""Gate-level tests for the installed stall_detector_node using fake sensors (wall clock).
Each scenario starts a fresh gate process with given params on an isolated ROS_DOMAIN_ID."""
import json, os, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger

READY = json.dumps({'state': 'ready', 'code': 'DEPTH_READY'})


class Fake(Node):
    def __init__(self):
        super().__init__('fake_sensors')
        self.on = dict(scan=True, lidar=True, depth=True, wheel=True, est=True, icp=True)
        self.depth_status = READY
        self.icp_inliers = 300.0
        self.cmd = Twist()
        self.wheel_vx = 0.0
        self.lidar_x = 0.0
        self.lidar_moving = False
        self.state = None
        self.out = []
        q = qos_profile_sensor_data
        self.p_scan = self.create_publisher(LaserScan, '/scan', q)
        self.p_lidar = self.create_publisher(Odometry, '/odometry/lidar_odom', 20)
        self.p_pts = self.create_publisher(PointCloud2, '/depth_obstacles/points', q)
        self.p_dst = self.create_publisher(String, '/auto_mapping/depth_status', 10)
        self.p_odom = self.create_publisher(Odometry, '/odom', 20)
        self.p_est = self.create_publisher(Odometry, '/odom_nav2', 20)
        self.p_icp = self.create_publisher(Float32MultiArray, '/diagnostics/f4_latency', 10)
        self.p_cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_subscription(String, '/stall_detector/status_json', self.cb_s, 50)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.out.append((m.linear.x, m.angular.z)), 100)
        self.cli = self.create_client(Trigger, '/stall_detector/reset_latch')
        self.create_timer(0.1, self.tick10)
        self.create_timer(0.05, self.tick20)

    def cb_s(self, m):
        self.state = json.loads(m.data)['state']

    def tick10(self):
        if self.on['scan']:
            s = LaserScan(); s.header.frame_id = 'laser'; s.range_min = 0.1; s.range_max = 10.0
            s.ranges = [2.0] * 10; self.p_scan.publish(s)
        if self.on['lidar']:
            if self.lidar_moving:
                self.lidar_x += 0.02
            o = Odometry(); o.pose.pose.position.x = self.lidar_x; self.p_lidar.publish(o)
        if self.on['icp']:
            self.p_icp.publish(Float32MultiArray(data=[0.0, 0.0, 0.0, self.icp_inliers]))
        if self.on['depth']:
            self.p_pts.publish(PointCloud2())

    def tick20(self):
        if self.on['depth']:
            self.p_dst.publish(String(data=self.depth_status))
        if self.on['wheel']:
            o = Odometry(); o.twist.twist.linear.x = self.wheel_vx; self.p_odom.publish(o)
        if self.on['est']:
            self.p_est.publish(Odometry())
        if self.cmd.linear.x or self.cmd.angular.z:
            self.p_cmd.publish(self.cmd)


def spin(n, sec):
    t0 = time.monotonic()
    while time.monotonic() - t0 < sec:
        rclpy.spin_once(n, timeout_sec=0.01)


def reset(n):
    n.cli.wait_for_service(timeout_sec=3.0)
    f = n.cli.call_async(Trigger.Request())
    t0 = time.monotonic()
    while not f.done() and time.monotonic() - t0 < 3:
        rclpy.spin_once(n, timeout_sec=0.01)
    return f.result()


def start_gate(params):
    args = ['ros2', 'run', 'limo_capstone', 'stall_detector_node', '--ros-args']
    base = dict(enabled='true', require_operator_unlatch='true', startup_timeout_sec='5.0')
    base.update(params)
    for k, v in base.items():
        args += ['-p', f'{k}:={v}']
    return subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


RESULTS = []


def check(name, cond, detail=''):
    RESULTS.append((name, bool(cond)))
    print(f"  [{'PASS' if cond else 'FAIL'}] {name} {detail}", flush=True)


def scenario(title, params, body, sensors_on=None):
    print(f'== {title}  params={params}', flush=True)
    n = Fake()
    if sensors_on is not None:
        n.on.update(sensors_on)
    g = start_gate(params)
    try:
        body(n)
    finally:
        os.killpg(g.pid, 9); g.wait()
        n.destroy_node()
        time.sleep(0.5)


def nonzero(out):
    return sum(1 for vx, wz in out if abs(vx) > 1e-6 or abs(wz) > 1e-6)


# ---- scenarios ----------------------------------------------------------------------
F4 = dict(depth_mode='required', require_lidar_odom='true', estimator_odom_topic='/odom_nav2')


def s_arm_and_block(n):
    n.cmd.linear.x = 0.2                      # motion requested from the start
    spin(n, 2.5)
    check('INIT holds while motion requested', n.state == 'INITIALIZING', f'state={n.state}')
    check('INIT output is always 0', nonzero(n.out) == 0, f'nonzero={nonzero(n.out)}/{len(n.out)}')
    r = reset(n)
    check('reset refused during INIT', r is not None and not r.success, r.message if r else 'no resp')
    n.cmd = Twist(); spin(n, 1.5)
    check('arms to NORMAL once healthy and cmd=0', n.state == 'NORMAL', f'state={n.state}')
    n.out.clear(); n.cmd.linear.x = 0.1; n.lidar_moving = True; n.wheel_vx = 0.1; spin(n, 1.0)
    check('NORMAL passes commands', nonzero(n.out) > 0, f'nonzero={nonzero(n.out)}')


def mk_dropout(sensor, extra=None):
    def body(n):
        spin(n, 1.8)
        check('armed', n.state == 'NORMAL', f'state={n.state}')
        n.cmd.linear.x = 0.1; n.lidar_moving = True; n.wheel_vx = 0.1
        spin(n, 0.5)
        if sensor == 'depth_status':
            n.depth_status = json.dumps({'state': 'error', 'code': 'DEPTH_FILTER_OR_TF_ERROR'})
        else:
            n.on[sensor] = False
        spin(n, 1.0)
        check(f'{sensor} dropout -> FAILSAFE', n.state == 'FAILSAFE_SENSOR_TIMEOUT', f'state={n.state}')
        n.out.clear(); spin(n, 0.5)
        check('output 0 in FAILSAFE', nonzero(n.out) == 0, f'nonzero={nonzero(n.out)}')
        # restore the sensor: latch must persist (operator unlatch required)
        n.on[sensor if sensor != 'depth_status' else 'depth'] = True; n.depth_status = READY
        n.cmd = Twist(); n.lidar_moving = False; n.wheel_vx = 0.0
        spin(n, 1.5)
        check('latch persists after sensor recovers', n.state == 'FAILSAFE_SENSOR_TIMEOUT', f'state={n.state}')
        r = reset(n); spin(n, 0.1)
        check('reset -> INITIALIZING', r.success and n.state in ('INITIALIZING', 'NORMAL'), f'state={n.state}')
        spin(n, 1.5)
        check('re-arms to NORMAL', n.state == 'NORMAL', f'state={n.state}')
    return body


def s_reset_with_dead_sensor(n):
    spin(n, 1.8)
    n.cmd.linear.x = 0.1; n.lidar_moving = True; n.wheel_vx = 0.1; spin(n, 0.3)
    n.on['scan'] = False; spin(n, 1.0)
    n.cmd = Twist(); n.lidar_moving = False; n.wheel_vx = 0.0
    r = reset(n); spin(n, 1.5)
    check('reset with scan still dead stays INITIALIZING', n.state == 'INITIALIZING', f'state={n.state}')
    spin(n, 5.0)
    check('then startup timeout -> FAILSAFE', n.state == 'FAILSAFE_SENSOR_TIMEOUT', f'state={n.state}')


def mk_timeout(expect_state):
    def body(n):
        spin(n, 3.0)
        check('not armed at 3s', n.state == 'INITIALIZING', f'state={n.state}')
        spin(n, 3.0)
        check(f'timeout -> {expect_state}', n.state == expect_state, f'state={n.state}')
        check('output 0 throughout', nonzero(n.out) == 0, f'nonzero={nonzero(n.out)}')
    return body


def s_arms(n):
    spin(n, 2.0)
    check('arms to NORMAL', n.state == 'NORMAL', f'state={n.state}')


def s_icp(n):
    spin(n, 1.8)
    n.icp_inliers = 40.0; n.cmd.linear.x = 0.1; n.lidar_moving = True; n.wheel_vx = 0.1; spin(n, 1.0)
    check('ICP degraded + motion -> FAILSAFE_ICP_DEGRADED', n.state == 'FAILSAFE_ICP_DEGRADED', f'state={n.state}')


def s_stall(n):
    spin(n, 1.8)
    n.cmd.linear.x = 0.2; n.wheel_vx = 0.2; n.lidar_moving = False; spin(n, 2.5)
    check('physical stall -> STALL_LATCHED', n.state == 'STALL_LATCHED', f'state={n.state}')
    n.out.clear(); spin(n, 0.5)
    check('output 0 when latched', nonzero(n.out) == 0, f'nonzero={nonzero(n.out)}')


rclpy.init()
scenario('F4/F5 sim config: block during INIT, refuse reset, arm', F4, s_arm_and_block)
for s in ('scan', 'lidar', 'depth', 'depth_status'):
    scenario(f'runtime dropout: {s}', F4, mk_dropout(s))
scenario('reset while a sensor is still dead', F4, s_reset_with_dead_sensor)
scenario('ICP degraded (existing)', F4, s_icp)
scenario('physical stall (existing)', F4, s_stall)
# estimator-specific readiness
scenario('B0/F1b without ICP observer, no depth camera (lidar odom & ICP absent)',
         dict(depth_mode='disabled', require_lidar_odom='false', estimator_odom_topic='/odom_nav2'),
         s_arms, sensors_on=dict(lidar=False, icp=False, depth=False))
scenario('F4 config but lidar odom never arrives',
         F4, mk_timeout('FAILSAFE_SENSOR_TIMEOUT'), sensors_on=dict(lidar=False))
scenario('depth required but camera absent',
         F4, mk_timeout('FAILSAFE_SENSOR_TIMEOUT'), sensors_on=dict(depth=False))
scenario('depth optional, camera absent -> arms',
         dict(F4, depth_mode='optional'), s_arms, sensors_on=dict(depth=False))
scenario('estimator output (/odom_nav2) never arrives',
         F4, mk_timeout('FAILSAFE_SENSOR_TIMEOUT'), sensors_on=dict(est=False))
scenario('wheel /odom never arrives',
         F4, mk_timeout('FAILSAFE_SENSOR_TIMEOUT'), sensors_on=dict(wheel=False))
scenario('no estimator_odom_topic param (real-launch default) -> arms without /odom_nav2',
         dict(depth_mode='required', require_lidar_odom='true'), s_arms, sensors_on=dict(est=False))

# ---- v2: runtime odometry monitoring + real-launch semantics (require_operator_unlatch=false) ----
def s_real_timeout_stays(n):
    spin(n, 3.0)
    check('not armed at 3s', n.state == 'INITIALIZING', f'state={n.state}')
    spin(n, 3.0)
    check('timeout -> FAILSAFE (auto-clearing config)', n.state == 'FAILSAFE_SENSOR_TIMEOUT', f'state={n.state}')
    spin(n, 3.0)
    check('FAILSAFE does NOT clear itself while the stream never arrived', n.state == 'FAILSAFE_SENSOR_TIMEOUT', f'state={n.state}')
    check('output 0 throughout', nonzero(n.out) == 0, f'nonzero={nonzero(n.out)}')


def s_real_wheel_recovers(n):
    spin(n, 1.8)
    check('armed', n.state == 'NORMAL', f'state={n.state}')
    n.cmd.linear.x = 0.1; n.lidar_moving = True; n.wheel_vx = 0.1; spin(n, 0.5)
    n.on['wheel'] = False; spin(n, 1.0)
    check('wheel odom lost while moving -> FAILSAFE', n.state == 'FAILSAFE_SENSOR_TIMEOUT', f'state={n.state}')
    n.out.clear(); spin(n, 0.5)
    check('output 0', nonzero(n.out) == 0, f'nonzero={nonzero(n.out)}')
    n.on['wheel'] = True; n.cmd = Twist(); n.lidar_moving = False; n.wheel_vx = 0.0
    spin(n, 1.5)
    check('auto-clearing config recovers once odom is back and cmd=0', n.state == 'NORMAL', f'state={n.state}')


def s_idle_est_dropout(n):
    spin(n, 1.8)
    check('armed', n.state == 'NORMAL', f'state={n.state}')
    n.on['est'] = False; spin(n, 1.0)       # robot idle: no motion commanded
    check('estimator odom lost while idle -> FAILSAFE (established stream lost)', n.state == 'FAILSAFE_SENSOR_TIMEOUT', f'state={n.state}')


def s_est_not_watched(n):
    spin(n, 1.8)
    check('armed without estimator_odom_topic', n.state == 'NORMAL', f'state={n.state}')
    n.on['est'] = False; spin(n, 1.5)
    check('/odom_nav2 absence ignored when topic not configured', n.state == 'NORMAL', f'state={n.state}')
    n.on['wheel'] = False; spin(n, 1.0)
    check('wheel odom loss still caught', n.state == 'FAILSAFE_SENSOR_TIMEOUT', f'state={n.state}')


B0 = dict(depth_mode='disabled', require_lidar_odom='false', estimator_odom_topic='/odom_nav2')
for s_ in ('wheel', 'est'):
    scenario(f'runtime odometry dropout: {s_} (F4/F5 config)', F4, mk_dropout(s_))
    scenario(f'runtime odometry dropout: {s_} (B0/F1b/F3b config: no lidar odom, no depth)', B0, mk_dropout(s_),
             sensors_on=dict(lidar=False, icp=False, depth=False))
scenario('real-launch semantics: startup timeout does not self-clear',
         dict(F4, require_operator_unlatch='false'), s_real_timeout_stays, sensors_on=dict(lidar=False))
scenario('real-launch semantics: runtime wheel dropout then auto-recovery',
         dict(F4, require_operator_unlatch='false'), s_real_wheel_recovers)
scenario('idle estimator-odom loss', F4, s_idle_est_dropout)
scenario('no estimator_odom_topic configured', dict(depth_mode='required', require_lidar_odom='true'), s_est_not_watched)

ok = sum(1 for _, p in RESULTS if p)
print(f'\nTOTAL {ok}/{len(RESULTS)} passed')
sys.exit(0 if ok == len(RESULTS) else 1)
