"""SLAM + Nav2 + idle action server. Robot driver/URDF/LiDAR run separately."""
from pathlib import Path
import math
import shutil
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    # 기존 LIMO 설정 파일을 재사용해 동일한 YAML 사본을 만들지 않는다.
    share = Path(get_package_share_directory('limo_capstone'))
    own_share = Path(get_package_share_directory('limo_auto_mapping'))
    # 저속 설정을 기본으로 하되 원본 설정과 사용자 지정 파일도 선택할 수 있다.
    profile = LaunchConfiguration('nav2_profile').perform(context)
    custom_params = LaunchConfiguration('nav2_params_file').perform(context)
    if profile not in ('slow', 'original'):
        raise RuntimeError('nav2_profile must be slow or original')
    params = (Path(custom_params).expanduser() if custom_params else
              own_share / 'config/nav2_mapping_slow.yaml' if profile == 'slow' else
              share / 'config/nav2_explore_params.yaml')
    if not params.is_file():
        raise RuntimeError(f'Nav2 parameter file does not exist: {params}')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    config = yaml.safe_load((share / 'config/slam_params.yaml').read_text())['slam_toolbox']['ros__parameters']
    config['use_sim_time'] = use_sim_time
    config['mode'] = 'mapping'
    graph = LaunchConfiguration('posegraph').perform(context)
    pose = LaunchConfiguration('start_pose').perform(context)
    gate = LaunchConfiguration('require_localization_ready').perform(context).lower()
    if gate not in ('true', 'false'):
        raise RuntimeError('require_localization_ready must be true or false')
    if graph:
        path = Path(graph).expanduser()
        if not path.is_absolute() or not all(Path(str(path) + ext).is_file()
                                             for ext in ('.posegraph', '.data')):
            raise RuntimeError('posegraph must be an absolute prefix of existing .posegraph/.data files')
        values = yaml.safe_load(pose)
        if (not isinstance(values, list) or len(values) != 3
                or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in values)):
            raise RuntimeError('Resuming requires start_pose:="[x, y, yaw]" in map coordinates')
        config['map_file_name'] = str(path)
        config['map_start_pose'] = [float(x) for x in values]
        config['map_start_at_dock'] = False
    elif pose:
        raise RuntimeError('start_pose requires posegraph')
    nav = Path(get_package_share_directory('nav2_bringup')) / 'launch/navigation_launch.py'
    depth_enabled = LaunchConfiguration('use_depth_camera', default='false').perform(context).lower()
    if depth_enabled not in ('true', 'false'):
        raise RuntimeError('use_depth_camera must be true or false')
    depth_actions = []
    if depth_enabled == 'true':
        from limo_depth_obstacles.configuration import add_camera_layer
        depth_share = Path(get_package_share_directory('limo_depth_obstacles'))
        filter_path = Path(LaunchConfiguration('depth_params_file', default='').perform(context)
                           or depth_share/'config/depth_obstacles.yaml').expanduser()
        filtering = yaml.safe_load(filter_path.read_text())['depth_obstacle_filter']['ros__parameters']
        generated = add_camera_layer(yaml.safe_load(params.read_text()), filtering)
        directory = Path(tempfile.mkdtemp(prefix='limo_depth_nav2_'))
        params = directory/'nav2.yaml'
        params.write_text(yaml.safe_dump(generated, sort_keys=False))
        def cleanup(_context):
            shutil.rmtree(directory, ignore_errors=True)  # 이 launch가 생성한 임시 폴더만 정리한다.
            return []
        depth_actions = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(depth_share/'launch/depth_obstacles.launch.py')),
            launch_arguments={'use_sim_time': str(use_sim_time).lower(),
                'filter_params_file': str(filter_path),
                'start_camera_driver': LaunchConfiguration('start_camera_driver', default='true'),
                'camera_extrinsics_file': LaunchConfiguration('camera_extrinsics_file', default='')}.items()),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]
    return [
        LogInfo(msg=f'Auto mapping Nav2 parameters: {params} (profile={profile})'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[config], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav)), launch_arguments={
            'params_file': str(params),
            'use_sim_time': str(use_sim_time).lower(), 'autostart': 'true',
            'use_composition': 'False'}.items()),
        Node(package='limo_auto_mapping', executable='mapping_server',
             parameters=[{'use_sim_time': use_sim_time,
                          'require_depth_ready': depth_enabled == 'true',
                          'require_localization_ready': gate == 'true'}], output='screen')] + depth_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('nav2_profile', default_value='slow',
                             description='slow: 저속 시험 설정, original: 기존 LIMO 설정'),
        DeclareLaunchArgument('nav2_params_file', default_value='',
                             description='Optional Nav2 YAML override; takes precedence over profile'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_depth_camera', default_value='false',
                             description='Enable depth filter, camera costmap layers and health gate'),
        DeclareLaunchArgument('start_camera_driver', default_value='true'),
        DeclareLaunchArgument('depth_params_file', default_value=''),
        DeclareLaunchArgument('camera_extrinsics_file', default_value=''),
        DeclareLaunchArgument('posegraph', default_value='', description='Serialized graph prefix; no extension'),
        DeclareLaunchArgument('start_pose', default_value='', description='Required map-frame [x,y,yaw] when resuming'),
        DeclareLaunchArgument('require_localization_ready', default_value='false',
                             description='Internal gate used by auto_resume; manual startup stays unchanged'),
        OpaqueFunction(function=start)])
