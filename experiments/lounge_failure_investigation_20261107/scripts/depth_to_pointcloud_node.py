#!/usr/bin/env python3
"""
depth_to_pointcloud_node.py:
Converts 32FC1 /camera/depth_image from Gazebo Fortress into sensor_msgs/PointCloud2.
Publishes to /camera/depth/points for consumption by limo_depth_obstacles.
"""
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header

class DepthToPointCloudNode(Node):
    def __init__(self):
        super().__init__(
            'depth_to_pointcloud_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.sub = self.create_subscription(Image, '/camera/depth_image', self.on_image, 5)
        self.pub = self.create_publisher(PointCloud2, '/camera/depth/points', 5)

        # Camera intrinsics (standard Gazebo 320x240, 1.047 rad horizontal FOV)
        # width = 320, height = 240
        # hfov = 1.04719755 rad (60 deg) -> fx = (width / 2) / tan(hfov / 2) = 160 / tan(30 deg) = 277.128
        self.width = 320
        self.height = 240
        self.fx = 277.13
        self.fy = 277.13
        self.cx = 160.0
        self.cy = 120.0

        # Precompute pixel grids
        u = np.arange(self.width)
        v = np.arange(self.height)
        self.uu, self.vv = np.meshgrid(u, v)
        self.x_factors = (self.uu - self.cx) / self.fx
        self.y_factors = (self.vv - self.cy) / self.fy

        self.get_logger().info("DepthToPointCloudNode initialized.")

    def on_image(self, msg: Image):
        # Gazebo emits 32FC1 float32
        if msg.encoding != '32FC1':
            return

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        
        # Subsample by stride 2 for real-time efficiency (160x120 = 19200 points)
        stride = 2
        d_sub = depth[::stride, ::stride]
        valid = np.isfinite(d_sub) & (d_sub > 0.15) & (d_sub < 3.5)

        z = d_sub[valid]
        x = self.x_factors[::stride, ::stride][valid] * z
        y = self.y_factors[::stride, ::stride][valid] * z

        if len(z) == 0:
            return

        # Stack into N x 3
        points = np.column_stack((z, -x, -y)).astype(np.float32) # Optical frame to camera link (X forward, Y left, Z up)

        # Build PointCloud2 msg
        cloud_msg = PointCloud2()
        cloud_msg.header = msg.header
        cloud_msg.header.frame_id = 'depth_link'
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.is_dense = True
        cloud_msg.is_bigendian = False

        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        cloud_msg.point_step = 12
        cloud_msg.row_step = 12 * len(points)
        cloud_msg.data = points.tobytes()

        self.pub.publish(cloud_msg)

def main():
    rclpy.init()
    node = DepthToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
