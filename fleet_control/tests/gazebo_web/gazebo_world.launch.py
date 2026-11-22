"""SIMULATION ONLY (2026-11-21): Ignition Fortress office world + LIMO (limo_eval_diff) for the web control chain.

Reuses the existing assets of experiments/safety_chain_fix_20261113/office (world, model, URDF, normalizer, ground-truth node).
Command path: Gate output /cmd_vel_locked -> drive_timeout_relay.py (Gate-independent lower timeout, GZ_DRIVE_TIMEOUT_S) -> /sim/drive_cmd -> bridge
-> Gazebo's DiffDrive /cmd_vel (GZ_DRIVE_TIMEOUT_S=0: bridge directly from /cmd_vel_locked, DiffDrive holds the last command).
ROS /cmd_vel therefore keeps NO endpoint, as in every isolated configuration; the real-robot launch is not involved.
Nothing here starts SLAM, AMCL, map_server or Nav2: the Agent's MappingWorker / LocalizationWorker and tests/web_mapping_isolated
stack.launch.py are the only producers. /scan has its own bridge process (gz_scan_bridge) so a LiDAR outage can be injected alone.
Ignition transport is limited to IGN_IP=127.0.0.1 and the IGN_PARTITION of the run (set by the caller); DDS is ROS_LOCALHOST_ONLY=1.
Env: GZ_SPAWN="x,y,yaw" (default 10.09,-6.98,0 = Office patrol_A1, the pose of the verified 2026-11-13 office run), GZ_WORLD (default
office_limo.world), RMF_DEMOS_HOME (default ~/rmf_demos_humble: the Office world's models and door plugins, as in that run).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

REPO = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..'))


def generate_launch_description():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1' and os.environ.get('IGN_PARTITION'), 'isolated run only'
    cap = get_package_share_directory('limo_capstone'); desc = get_package_share_directory('limo_description')
    world = os.environ.get('GZ_WORLD') or os.path.join(REPO, 'experiments/office_limo_sim_20261113/office_limo.world')
    model = os.path.join(cap, 'models', 'limo_eval_diff', 'model.sdf')
    urdf = open(os.path.join(cap, 'urdf', 'limo_diff_fortress.urdf')).read()
    sx, sy, syaw = (os.environ.get('GZ_SPAWN') or '10.09,-6.98,0').split(',')
    rmf = os.environ.get('RMF_DEMOS_HOME') or os.path.expanduser('~/rmf_demos_humble')
    res = ':'.join([os.path.join(cap, 'models'), os.path.dirname(desc), os.path.dirname(world), os.path.join(rmf, 'models'),
                    os.path.join(rmf, 'ws/install/rmf_demos_assets/share/rmf_demos_assets/models'),
                    os.path.join(rmf, 'ws/install/rmf_demos_maps/share/rmf_demos_maps/maps/office/models')])
    pre = os.path.join(rmf, 'prefix')
    gz_env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': ':'.join(os.path.join(pre, 'opt/ros/humble/lib', d) for d in ('rmf_robot_sim_gz_plugins', 'rmf_building_sim_gz_plugins')),
              'LD_LIBRARY_PATH': ':'.join([os.path.join(pre, d) for d in ('opt/ros/humble/lib/x86_64-linux-gnu', 'usr/lib/x86_64-linux-gnu', 'opt/ros/humble/lib',
                                                                          'opt/ros/humble/opt/rmf_traffic/lib')] + [os.environ.get('LD_LIBRARY_PATH', '')])}
    sim = {'use_sim_time': True}
    # GZ_DRIVE_TIMEOUT_S (default 0.2): Gate-independent lower-drive timeout process between /cmd_vel_locked and the bridge
    # (drive_timeout_relay.py). 0 = direct bridge from /cmd_vel_locked (Ignition DiffDrive then HOLDS the last command).
    drive_timeout = float(os.environ.get('GZ_DRIVE_TIMEOUT_S', '0.2') or 0)
    drive_in = '/sim/drive_cmd' if drive_timeout > 0 else '/cmd_vel_locked'
    out = [SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', res), SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', res),
           SetEnvironmentVariable('IGN_IP', '127.0.0.1'),
           ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-s', world, '-v', '2'], output='screen', additional_env=gz_env),
           Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher', parameters=[{'robot_description': urdf}, sim]),
           TimerAction(period=3.0, actions=[Node(package='ros_gz_sim', executable='create', output='screen',
                                                 arguments=['-world', 'sim_world', '-file', model, '-name', 'limo', '-x', sx, '-y', sy, '-z', '0.05', '-Y', syaw])]),
           Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen', parameters=[sim],
                arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                           '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                           '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                           '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                           '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                           '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                           '/world/sim_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
                remappings=[('/cmd_vel', drive_in), ('/odom', '/odom_gz_raw'), ('/imu', '/imu_gz_raw'), ('/tf', '/gz/wheel_tf_raw'),
                            ('/world/sim_world/dynamic_pose/info', '/gz/dynamic_pose_info')]),
           Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_scan_bridge', output='screen', parameters=[sim],
                arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']),
           Node(package='limo_capstone', executable='sensor_normalizer_node', name='sensor_normalizer_node', parameters=[sim]),
           Node(package='limo_capstone', executable='ground_truth_publisher_node', name='ground_truth_publisher_node', parameters=[sim]),
           ExecuteProcess(cmd=['python3', os.path.join(os.path.dirname(os.path.realpath(__file__)), 'gz_truth_adapter.py'), '--ros-args',
                               '-p', 'use_sim_time:=true', '-p', f'spawn_x:={float(sx)!r}', '-p', f'spawn_y:={float(sy)!r}', '-p', f'spawn_yaw:={float(syaw)!r}'], output='screen')]
    if drive_timeout > 0:
        out.append(ExecuteProcess(cmd=['python3', os.path.join(os.path.dirname(os.path.realpath(__file__)), 'drive_timeout_relay.py'), '--ros-args',
                                       '-p', f'timeout_s:={drive_timeout!r}'], output='screen'))
    return LaunchDescription(out)
