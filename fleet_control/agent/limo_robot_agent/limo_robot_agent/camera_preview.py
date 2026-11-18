"""Bounded read-only snapshots from ROS camera topics."""
import base64
import io
import threading
import time

import numpy as np
from PIL import Image
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image as RosImage


def render_frame(kind, message):
    width, height, step = message.width, message.height, message.step
    if not (0 < width * height <= 4_000_000 and len(message.data) == step * height <= 16_000_000):
        raise ValueError('invalid frame size')
    stride = max(1, (max(width, height) + 479) // 480)
    if kind == 'depth':
        types = {'16UC1': 'u2', '32FC1': 'f4'}
        fmt = types.get(message.encoding)
        if not fmt:
            raise ValueError('unsupported depth encoding')
        dtype = np.dtype(('>' if message.is_bigendian else '<') + fmt)
        if step < width * dtype.itemsize:
            raise ValueError('invalid row stride')
        values = np.ndarray((height, width), dtype=dtype, buffer=message.data,
                            strides=(step, dtype.itemsize))[::stride, ::stride].astype(np.float32)
        if fmt == 'u2':
            values *= .001
        valid = np.isfinite(values) & (values > 0)
        levels = np.zeros(values.shape, dtype=np.uint8)
        levels[valid] = (255 - 254 * np.clip(values[valid] / 5, 0, 1)).astype(np.uint8)
        image = Image.fromarray(levels)
    else:
        order = {'rgb8': [0, 1, 2], 'bgr8': [2, 1, 0], 'rgba8': [0, 1, 2], 'bgra8': [2, 1, 0]}
        channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(message.encoding)
        if channels is None or step < width * channels:
            raise ValueError('unsupported color encoding or row stride')
        pixels = np.ndarray((height, width, channels), dtype=np.uint8, buffer=message.data,
                            strides=(step, channels, 1))[::stride, ::stride]
        image = Image.fromarray(pixels[..., 0] if channels == 1 else pixels[..., order[message.encoding]])
    output = io.BytesIO()
    image.save(output, 'JPEG', quality=78)
    return output.getvalue()


class CameraPreview:
    def __init__(self, robot, agent, rgb_topic, depth_topic):
        self.robot, self.agent = robot, agent
        self.frames = {}
        self.last = {}
        self.stop = threading.Event()
        for kind, topic in (('rgb', rgb_topic), ('depth', depth_topic)):
            robot.create_subscription(RosImage, topic, lambda msg, k=kind: self.receive(k, msg),
                                      qos_profile_sensor_data)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def receive(self, kind, message):
        self.frames[kind] = time.monotonic(), message

    def run(self):
        while not self.stop.wait(.4):
            if not self.agent.connected:
                continue
            for kind, (received, message) in self.frames.copy().items():
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                age = self.robot.get_clock().now().nanoseconds * 1e-9 - stamp
                if (received == self.last.get(kind) or time.monotonic() - received > 1
                        or stamp <= 0 or not -.2 <= age < 2):
                    continue
                self.last[kind] = received
                try:
                    payload = base64.b64encode(render_frame(kind, message)).decode('ascii')
                    self.agent.http.post('/agent/camera', {
                        'robot_id': self.agent.robot_id, 'agent_boot_id': self.agent.boot_id,
                        'kind': kind, 'source_age_s': max(0., age), 'jpeg_base64': payload
                    }, timeout=.7)
                except Exception:
                    continue

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
