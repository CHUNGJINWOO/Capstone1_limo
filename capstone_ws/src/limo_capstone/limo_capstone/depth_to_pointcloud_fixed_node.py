#!/usr/bin/env -S python3 -s
"""
depth_to_pointcloud_fixed_node.py
Fixes the 2nd-Stage Verification TF disconnection bug where depth points were published
with frame_id='depth_link' while no TF existed for 'depth_link'.
Here we publish with frame_id='depth_camera_link', paired with a static TF broadcaster
(base_link -> depth_camera_link -> depth_link) in benchmark_sim.launch.py.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, PointCloud2, PointField


class DepthToPointCloudFixedNode(Node):
    def __init__(self):
        super().__init__(
            'depth_to_pointcloud_fixed_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('output_frame_id', 'depth_camera_link')
        self.output_frame_id = self.get_parameter('output_frame_id').value

        self.sub = self.create_subscription(Image, '/camera/depth_image', self.on_image, 5)
        self.pub = self.create_publisher(PointCloud2, '/camera/depth/points', 5)

        # Match model.sdf depth_camera horizontal_fov=1.15 rad, 320x240
        self.width = 320
        self.height = 240
        hfov = 1.15
        self.fx = (self.width * 0.5) / np.tan(hfov * 0.5)
        self.fy = self.fx
        self.cx = 160.0
        self.cy = 120.0

        u = np.arange(self.width)
        v = np.arange(self.height)
        uu, vv = np.meshgrid(u, v)
        self.x_factors = (uu - self.cx) / self.fx
        self.y_factors = (vv - self.cy) / self.fy

        self.get_logger().info(f"DepthToPointCloudFixedNode started with frame_id='{self.output_frame_id}'.")

    def on_image(self, msg: Image):
        if msg.encoding != '32FC1':
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        stride = 2
        d_sub = depth[::stride, ::stride]
        valid = np.isfinite(d_sub) & (d_sub > 0.20) & (d_sub < 3.0)

        z = d_sub[valid]
        if len(z) == 0:
            return
        x = self.x_factors[::stride, ::stride][valid] * z
        y = self.y_factors[::stride, ::stride][valid] * z

        # Camera link coordinates: X forward, Y left, Z up
        points = np.column_stack((z, -x, -y)).astype(np.float32)

        cloud_msg = PointCloud2()
        cloud_msg.header = msg.header
        cloud_msg.header.frame_id = self.output_frame_id
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.point_step = 12
        cloud_msg.row_step = 12 * len(points)
        cloud_msg.data = points.tobytes()
        self.pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudFixedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
