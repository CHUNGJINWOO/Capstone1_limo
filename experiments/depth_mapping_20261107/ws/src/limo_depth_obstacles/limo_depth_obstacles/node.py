"""깊이 카메라 → 장애물/관측 광선 PointCloud2 + 자동 맵핑 상태 신호."""
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .filtering import filter_points, stamp_age


class DepthObstacleFilter(Node):
    def __init__(self):
        super().__init__('depth_obstacle_filter')
        defaults = {'input_topic': '/camera/depth/points', 'height_frame': 'base_footprint',
                    'min_range': 0.20, 'max_range': 2.50, 'min_height': 0.025,
                    'max_height': 0.35, 'self_radius': 0.19, 'processing_hz': 5.0,
                    'data_timeout_sec': 1.0, 'point_stride': 1}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        for name in ('min_range', 'max_range', 'min_height', 'max_height', 'self_radius',
                     'processing_hz', 'data_timeout_sec'):
            if not math.isfinite(self.p(name)):
                raise ValueError(name+' must be finite')
        if not (0 <= self.p('min_range') < self.p('max_range')
                and 0 <= self.p('min_height') < self.p('max_height') < .8
                and self.p('self_radius') >= 0 and self.p('processing_hz') > 0
                and self.p('data_timeout_sec') > 0 and isinstance(self.p('point_stride'), int)
                and not isinstance(self.p('point_stride'), bool) and self.p('point_stride') >= 1):
            raise ValueError('Invalid depth filter ranges/rates')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.obstacles = self.create_publisher(PointCloud2, '/depth_obstacles/points', qos)
        self.clearing = self.create_publisher(PointCloud2, '/depth_obstacles/clearing_points', qos)
        self.status = self.create_publisher(String, '/auto_mapping/depth_status', 10)
        self.create_subscription(PointCloud2, self.p('input_topic'), self.on_cloud, qos)
        self.last_processed = float('-inf')
        self.last_valid = float('-inf')
        self.last_stamp = None
        self.state = {'state': 'error', 'code': 'DEPTH_MISSING', 'message': 'No validated cloud yet'}
        self.previous_code = None
        self.create_timer(.2, self.publish_status)

    def p(self, name):
        return self.get_parameter(name).value

    def set_error(self, code, message):
        self.state = {'state': 'error', 'code': code, 'message': message}

    def on_cloud(self, message):
        if time.monotonic()-self.last_processed < 1/self.p('processing_hz'):
            return
        self.last_processed = time.monotonic()
        try:
            stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
            age = stamp_age(self.get_clock().now().nanoseconds/1e9, stamp)
            if not message.header.frame_id or not -.2 <= age <= self.p('data_timeout_sec'):
                raise ValueError(f'Invalid frame/stamp: age={age:.3f}s')
            if self.last_stamp is not None and stamp <= self.last_stamp:
                self.set_error('DEPTH_REPEATED_STAMP', 'Camera stamp is not advancing')
                return
            transform = self.tf.lookup_transform(self.p('height_frame'), message.header.frame_id,
                                                 rclpy.time.Time.from_msg(message.header.stamp))
            # 필요한 필드만 선택해 rgb 정수 필드와 XYZ float 자료형 혼용을 피한다.
            if message.row_step != message.width*message.point_step:
                raise ValueError('Padded cloud rows are not supported by this reader')
            raw = point_cloud2.read_points(message, field_names=('x', 'y', 'z'), skip_nans=False)
            points = np.column_stack([raw[name].ravel() for name in ('x', 'y', 'z')])
            marked, clear, counts = filter_points(points[::self.p('point_stride')], transform.transform,
                self.p('min_range'), self.p('max_range'), self.p('min_height'), self.p('max_height'),
                self.p('self_radius'))
            if counts['valid_points'] == 0:
                self.set_error('DEPTH_NO_VALID_POINTS', 'No finite in-range camera observations')
                return
            self.obstacles.publish(point_cloud2.create_cloud_xyz32(message.header, marked))
            self.clearing.publish(point_cloud2.create_cloud_xyz32(message.header, clear))
            self.last_valid, self.last_stamp = time.monotonic(), stamp
            self.state = {'state': 'ready', 'code': 'DEPTH_READY', 'message': '', **counts,
                          'source_frame': message.header.frame_id, 'source_stamp_sec': stamp}
        except Exception as exc:
            self.set_error('DEPTH_FILTER_OR_TF_ERROR', f'{type(exc).__name__}: {exc}')

    def publish_status(self):
        payload = dict(self.state)
        if payload['state'] == 'ready':
            age = stamp_age(self.get_clock().now().nanoseconds/1e9, self.last_stamp)
            if time.monotonic()-self.last_valid > self.p('data_timeout_sec') or not (
                    -.2 <= age <= self.p('data_timeout_sec')):
                payload.update(state='error', code='DEPTH_STALE', message=f'Depth age={age:.3f}s')
        if payload['code'] != self.previous_code:
            self.get_logger().info('DEPTH_STATUS '+json.dumps(payload, ensure_ascii=False))
            self.previous_code = payload['code']
        self.status.publish(String(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
