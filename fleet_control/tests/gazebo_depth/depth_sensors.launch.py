"""SIMULATION ONLY (2026-11-22, Claude): the existing Depth obstacle path for the Gazebo web/test stack, started NEXT TO
tests/gazebo_web/gazebo_world.launch.py (which is not modified and has no depth path).

Reuses, unchanged:
  Gazebo limo_eval_diff depth_camera (model.sdf: /camera/depth_image, 320x240 R_FLOAT32, hfov 1.15, clip 0.2-3.0 m, pose 0.12 0 0.13)
  -> ros_gz_bridge (own process 'gz_depth_bridge', so a depth outage can be injected alone)
  -> limo_capstone depth_to_pointcloud_fixed_node (/camera/depth/points, frame depth_camera_link)
  -> limo_depth_obstacles depth_obstacle_filter_node (/depth_obstacles/points, /auto_mapping/depth_status), installed package as is.
TF: the sim URDF (limo_diff_fortress.urdf) has no camera frame, so the static TFs of the 2026-11-13 office launch are reused:
  base_link -> depth_camera_link (0.12, 0, -0.02) [base_link is 0.15 above base_footprint; camera 0.13 above the model origin],
  depth_camera_link -> depth_link (identity).
Height band, SIM ONLY: base_footprint is the model origin, 0.045 m above the Gazebo floor, while the filter treats base_footprint z=0
as the floor. The real calibrated band (0.025-0.35 m above the floor, FIELD_LOCKED_READINESS_20261120 sec. 18) is therefore given as
min_height -0.02 / max_height 0.305 here (the same -0.045 shift as the 2026-11-13 office launch's min_height -0.02).
processing_hz 20 as in the 2026-11-13 office launch (the filter throttles on wall time). Nothing here publishes velocity.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim = {'use_sim_time': True}
    return LaunchDescription([
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_depth_bridge', output='screen', parameters=[sim],
             arguments=['/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_depth_camera_link', parameters=[sim],
             arguments=['--x', '0.12', '--y', '0', '--z', '-0.02', '--frame-id', 'base_link', '--child-frame-id', 'depth_camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', name='static_tf_depth_link_alias', parameters=[sim],
             arguments=['--frame-id', 'depth_camera_link', '--child-frame-id', 'depth_link']),
        Node(package='limo_capstone', executable='depth_to_pointcloud_fixed_node', name='depth_to_pointcloud_fixed_node', output='screen',
             parameters=[sim]),
        Node(package='limo_depth_obstacles', executable='depth_obstacle_filter_node', name='depth_obstacle_filter', output='screen',
             parameters=[dict(sim, height_frame='base_footprint', processing_hz=20.0, min_height=-0.02, max_height=0.305)]),
    ])
