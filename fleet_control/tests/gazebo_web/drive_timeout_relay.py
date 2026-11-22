"""SIMULATION ONLY (2026-11-21): Gate-independent lower-drive command timeout for the Gazebo LIMO.

Why: Ignition Fortress DiffDrive keeps executing the last /cmd_vel it received (no command timeout). When the Safety Gate process
stalls or dies it can publish neither motion nor a stop, so the simulated robot kept moving (GATE_STALL_GAZEBO_20261121.md §3).
A real drive has its own command timeout below the Gate (LIMO STM32: value NOT measured); this node models that layer in sim.

Position in the chain:  Gate r12 --/cmd_vel_locked--> [this process] --/sim/drive_cmd--> ros_gz_bridge --> Ignition /cmd_vel
  - forwards every Gate output immediately (no smoothing, no limits, no policy: the Gate stays the only safety decision maker);
  - its OWN steady-clock timer (not the Gate's, not /clock) checks the age of the last Gate output; older than `timeout_s`
    -> publishes zero at `zero_rate_hz` until a new Gate output arrives (Ignition holds the last command, so zeros are repeated);
  - runs in its own process (the sim launch group), so a Gate SIGSTOP/SIGKILL does not stop it.
Limits (by design, recorded in the report): if THIS process stops, Gazebo again holds the last command - the same single point a
real MCU timeout has. Not a real-robot component; the real-robot launch and firmware are untouched.
Status: /sim/drive_timeout/status (JSON, 10 Hz): state FORWARDING / TIMED_OUT / NO_INPUT, timeouts count, last input age.
"""
import json, os, time
import rclpy
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from std_msgs.msg import String

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0', '77'), 'isolated sim only'


class DriveTimeoutRelay:
    def __init__(self, node):
        self.n = node
        self.timeout = float(node.declare_parameter('timeout_s', 0.2).value)
        rate = float(node.declare_parameter('zero_rate_hz', 50.0).value)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.out = node.create_publisher(Twist, node.declare_parameter('output_topic', '/sim/drive_cmd').value, 10)
        self.stat = node.create_publisher(String, '/sim/drive_timeout/status', 10)
        node.create_subscription(Twist, node.declare_parameter('input_topic', '/cmd_vel_locked').value, self.on_cmd, 20)
        self.last_rx = None; self.timed_out = False; self.timeouts = 0; self.t_timeout = None; self.forwarded = 0
        node.create_timer(1.0 / rate, self.check, clock=steady)
        node.create_timer(0.1, self.publish_status, clock=steady)

    def on_cmd(self, msg):
        self.last_rx = time.monotonic(); self.forwarded += 1
        if self.timed_out:
            self.timed_out = False
            self.n.get_logger().warn('DRIVE_TIMEOUT cleared: Gate output resumed')
        self.out.publish(msg)

    def check(self):
        if self.last_rx is None:
            return                                   # nothing commanded yet: Ignition starts at rest
        age = time.monotonic() - self.last_rx
        if age > self.timeout:
            if not self.timed_out:
                self.timed_out = True; self.timeouts += 1; self.t_timeout = time.monotonic()
                self.n.get_logger().error(f'DRIVE_TIMEOUT: no Gate output for {age:.3f}s (> {self.timeout:.3f}s) -> zero')
            self.out.publish(Twist())

    def publish_status(self):
        age = None if self.last_rx is None else round(time.monotonic() - self.last_rx, 3)
        st = 'NO_INPUT' if self.last_rx is None else 'TIMED_OUT' if self.timed_out else 'FORWARDING'
        self.stat.publish(String(data=json.dumps({'state': st, 'timeout_s': self.timeout, 'input_age_s': age, 'timeouts': self.timeouts,
                                                  'forwarded': self.forwarded, 'pid': os.getpid(), 't': time.time()})))


def main():
    rclpy.init(); node = rclpy.create_node('sim_drive_timeout'); DriveTimeoutRelay(node)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass


if __name__ == '__main__':
    main()
