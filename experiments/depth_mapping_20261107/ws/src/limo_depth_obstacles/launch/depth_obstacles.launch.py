"""기존 astra 카메라 구성 재사용. base→camera TF를 추측해서 생성하지 않는다."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from limo_depth_obstacles.configuration import checked_extrinsics


def start(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    own = Path(get_package_share_directory('limo_depth_obstacles'))
    params = Path(arg('filter_params_file') or own/'config/depth_obstacles.yaml').expanduser()
    if not params.is_file():
        raise RuntimeError('Depth filter parameter file does not exist: '+str(params))
    use_sim = arg('use_sim_time').lower() == 'true'
    if arg('start_camera_driver').lower() not in ('true', 'false'):
        raise ValueError('start_camera_driver must be true or false')
    actions = [Node(package='limo_depth_obstacles', executable='depth_obstacle_filter',
                    name='depth_obstacle_filter', parameters=[str(params), {'use_sim_time': use_sim}],
                    output='screen')]
    if arg('start_camera_driver').lower() == 'true':
        astra = Path(get_package_share_directory('astra_camera'))
        config = yaml.safe_load((astra/'params/dabai_params.yaml').read_text())
        # optical 내부 TF는 드라이버가 담당한다. 차체→camera_link 연결은 별도 실측/기존 URDF 필요.
        config.update({'publish_tf': True, 'depth_align': False, 'enable_color': False,
                       'uvc_camera.enable': False, 'use_sim_time': use_sim})
        actions.append(ComposableNodeContainer(
            name='mapping_depth_camera', namespace='', package='rclcpp_components',
            executable='component_container', output='screen', composable_node_descriptions=[
                ComposableNode(package='astra_camera', plugin='astra_camera::OBCameraNodeFactory',
                               name='camera', namespace='camera', parameters=[config]),
                ComposableNode(package='astra_camera', plugin='astra_camera::PointCloudXyzNode',
                               name='point_cloud_xyz', namespace='camera')]))
    if arg('camera_extrinsics_file'):
        extrinsics = yaml.safe_load(Path(arg('camera_extrinsics_file')).expanduser().read_text())
        actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                            name='mapping_camera_extrinsics', arguments=checked_extrinsics(extrinsics),
                            parameters=[{'use_sim_time': use_sim}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_camera_driver', default_value='true'),
        DeclareLaunchArgument('filter_params_file', default_value=''),
        DeclareLaunchArgument('camera_extrinsics_file', default_value='',
                              description='Optional measured TF; omit if a correct TF already exists'),
        OpaqueFunction(function=start)])
