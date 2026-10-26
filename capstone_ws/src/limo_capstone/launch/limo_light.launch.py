from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Base (limo_base) with odom TF enabled
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('limo_base'), '/launch/limo_base.launch.py'
        ]),
        launch_arguments={'pub_odom_tf': 'true'}.items()
    )

    # 2. Robot TF Description (load_urdf -> base_link to laser_link)
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('limo_description'), '/launch/load_urdf.launch.py'
        ])
    )

    # 3. YDLidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ydlidar_ros2_driver'), '/launch/ydlidar.launch.py'
        ])
    )

    return LaunchDescription([
        base_launch,
        urdf_launch,
        lidar_launch
    ])
