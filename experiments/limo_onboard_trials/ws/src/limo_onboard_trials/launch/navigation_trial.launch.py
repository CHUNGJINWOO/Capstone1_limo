"""USB 기준·BT만 변경·RPP+BT 변경을 원본 수정 없이 각각 실행한다."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from limo_onboard_trials.trial_config import configure, validate_map


def start(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = Path(get_package_share_directory('limo_onboard_trials'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))/'launch'
    profile, mode = arg('profile'), arg('mode')
    if mode not in ('navigation', 'mapping'):
        raise ValueError('mode must be navigation or mapping')
    if arg('use_sim_time').lower() not in ('true', 'false'):
        raise ValueError('use_sim_time must be true or false')
    source, rewrites = configure(share, profile, arg('backup'), arg('collision_time'))
    params = RewrittenYaml(source_file=source, root_key='',
                           param_rewrites=rewrites, convert_types=True)
    common = {'params_file': params, 'use_sim_time': arg('use_sim_time'),
              'autostart': 'true', 'use_composition': 'false'}
    if mode == 'navigation':
        # 저장 지도 주행: 기존 LIMO 지도 파일을 읽으며 지도를 새로 복사하지 않는다.
        filename = arg('map')
        if not filename:
            filename = str(Path(get_package_share_directory('limo_capstone'))/'maps/MC_3F_10_15.yaml')
        common['map'] = validate_map(filename)
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav2/'bringup_launch.py')),
            launch_arguments=common.items())]
    if arg('map'):
        raise ValueError('mapping mode does not load a static map; leave map empty')
    # 맵핑 비교: AMCL/정적 map_server를 실행하지 않고 SLAM과 Nav2 navigation만 실행한다.
    return [
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[str(share/'config/slam_params.yaml'),
                         {'use_sim_time': arg('use_sim_time').lower() == 'true'}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2/'navigation_launch.py')),
                                 launch_arguments=common.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='usb', description='usb, bt, or rpp_bt'),
        DeclareLaunchArgument('mode', default_value='navigation', description='navigation or mapping'),
        DeclareLaunchArgument('map', default_value='', description='Existing LIMO map YAML; navigation only'),
        DeclareLaunchArgument('backup', default_value='15', description='Custom BT backup distance: 15 or 30 cm'),
        DeclareLaunchArgument('collision_time', default_value='', description='Optional RPP prediction horizon in seconds'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=start)])
