#!/usr/bin/env python3
"""
depth_obstacle_filter_node.py
Integrated DaBai Depth Camera Obstacle Filter for Nav2 Costmap:
- Subscribes to /camera/depth/points.
- Transforms points into height_frame (base_footprint).
- Filters points in vertical window [0.025m, 0.35m] to capture low crossbars / sled rails / hurdles.
- Publishes /depth_obstacles/points with RELIABLE QoS (compatible with Nav2 ObstacleLayer).
- Publishes /depth_obstacles/clearing_points and /auto_mapping/depth_status.
"""
import json
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from limo_depth_obstacles.filtering import filter_points, stamp_age


class DepthObstacleFilterNode(Node):
    def __init__(self):
        super().__init__('depth_obstacle_filter', automatically_declare_parameters_from_overrides=True)
        defaults = {
            'use_sim_time': False,
            'input_topic': '/camera/depth/points',
            'height_frame': 'base_footprint',
            'min_range': 0.20,
            'max_range': 2.50,
            'min_height': 0.025,
            'max_height': 0.35,
            'self_radius': 0.19,
            'processing_hz': 10.0,
            'data_timeout_sec': 0.38,
            'point_stride': 1
        }
        for name, value in defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, value)

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

        # Use RELIABLE QoS so Nav2 Costmap ObstacleLayer (which requests RELIABLE) connects without rejection
        qos_pub = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                             reliability=ReliabilityPolicy.RELIABLE)
        qos_sub = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT)

        self.obstacles = self.create_publisher(PointCloud2, '/depth_obstacles/points', qos_pub)
        self.clearing = self.create_publisher(PointCloud2, '/depth_obstacles/clearing_points', qos_pub)
        self.status = self.create_publisher(String, '/auto_mapping/depth_status', 10)

        # Allow subscribing to either best-effort or reliable
        self.create_subscription(PointCloud2, self.p('input_topic'), self.on_cloud, qos_sub)

        self.last_processed = float('-inf')
        self.last_valid = float('-inf')
        self.last_stamp = None
        self.state = {'state': 'error', 'code': 'DEPTH_MISSING', 'message': 'No validated cloud yet'}
        self.previous_code = None
        self.create_timer(0.05, self.publish_status)

        self.get_logger().info("DepthObstacleFilterNode initialized with RELIABLE publisher for Nav2 costmaps.")

    def p(self, name):
        return self.get_parameter(name).value

    def set_error(self, code, message):
        self.state = {'state': 'error', 'code': code, 'message': message}

    def on_cloud(self, message):
        now_mono = time.monotonic()
        if now_mono - self.last_processed < 1.0 / self.p('processing_hz'):
            return
        self.last_processed = now_mono

        try:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
            age = stamp_age(self.get_clock().now().nanoseconds / 1e9, stamp)
            if not message.header.frame_id or not (-0.2 <= age <= self.p('data_timeout_sec')):
                raise ValueError(f'Invalid frame/stamp: age={age:.3f}s')

            if self.last_stamp is not None and stamp <= self.last_stamp:
                self.set_error('DEPTH_REPEATED_STAMP', 'Camera stamp is not advancing')
                return

            transform = self.tf.lookup_transform(
                self.p('height_frame'), message.header.frame_id,
                rclpy.time.Time.from_msg(message.header.stamp)
            )

            raw = point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=False)
            points = np.column_stack([raw[name].ravel() for name in ('x', 'y', 'z')])

            stride = self.p('point_stride')
            marked, clear, counts = filter_points(
                points[::stride], transform.transform,
                self.p('min_range'), self.p('max_range'),
                self.p('min_height'), self.p('max_height'),
                self.p('self_radius')
            )

            if counts['valid_points'] == 0:
                self.set_error('DEPTH_NO_VALID_POINTS', 'No finite in-range camera observations')
                return

            marked_count = counts.get('marked_points', 0)
            if marked_count > 10:
                now_t = time.monotonic()
                if not hasattr(self, '_last_warn_t') or (now_t - self._last_warn_t > 1.5):
                    self._last_warn_t = now_t
                    self.get_logger().warn(
                        f"[3D OBSTACLE DETECTED] DaBai Depth marked {marked_count} low obstacle points in window [{self.p('min_height')}, {self.p('max_height')}]m"
                    )

            self.obstacles.publish(point_cloud2.create_cloud_xyz32(message.header, marked))
            self.clearing.publish(point_cloud2.create_cloud_xyz32(message.header, clear))
            self.last_valid = time.monotonic()
            self.last_stamp = stamp
            self.state = {
                'state': 'ready', 'code': 'DEPTH_READY', 'message': '', **counts,
                'source_frame': message.header.frame_id, 'source_stamp_sec': stamp
            }
        except Exception as exc:
            self.get_logger().error(f"[DEPTH FILTER/TF ERROR] {type(exc).__name__}: {exc}")
            self.set_error('DEPTH_FILTER_OR_TF_ERROR', f'{type(exc).__name__}: {exc}')

    def publish_status(self):
        payload = dict(self.state)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if payload['state'] == 'ready':
            if self.last_stamp is not None:
                age = stamp_age(now_sec, self.last_stamp)
                if not (-0.2 <= age <= self.p('data_timeout_sec')):
                    payload.update(state='error', code='DEPTH_STALE', message=f'Depth age={age:.3f}s')
                    self.state = payload
                    self.get_logger().error(f"[DEPTH SENSOR STALE] PointCloud age={age:.3f}s exceeds {self.p('data_timeout_sec')}s")
        if payload['code'] != self.previous_code:
            self.get_logger().info('DEPTH_STATUS ' + json.dumps(payload, ensure_ascii=False))
            self.previous_code = payload['code']

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
