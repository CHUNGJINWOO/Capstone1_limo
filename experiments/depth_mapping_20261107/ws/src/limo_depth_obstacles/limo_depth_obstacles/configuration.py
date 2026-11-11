"""기존 LiDAR 설정을 보존하고 전용 카메라 voxel 레이어만 덧붙인다."""
from copy import deepcopy
import math


def add_camera_layer(config, filtering):
    result = deepcopy(config)
    for key in ('local_costmap', 'global_costmap'):
        params = result[key][key]['ros__parameters']
        plugins = params['plugins']
        if not isinstance(plugins, list) or 'inflation_layer' not in plugins or 'depth_camera_layer' in plugins:
            raise ValueError('Expected a plugin list with inflation_layer and no existing depth_camera_layer')
        # 관측을 먼저 합치고 나중에 inflation을 적용한다. 기존 반경·속도는 바꾸지 않는다.
        plugins.insert(plugins.index('inflation_layer'), 'depth_camera_layer')
        near, far = filtering['min_range'], filtering['max_range']
        params['depth_camera_layer'] = {
            'plugin': 'nav2_costmap_2d::VoxelLayer', 'enabled': True,
            'publish_voxel_map': False, 'origin_z': 0.0, 'z_resolution': 0.05,
            'z_voxels': 16, 'mark_threshold': 0, 'unknown_threshold': 15,
            'combination_method': 1, 'footprint_clearing_enabled': True,
            'max_obstacle_height': 10.0, 'observation_sources': 'depth_mark depth_clear',
            'depth_mark': {'topic': '/depth_obstacles/points', 'data_type': 'PointCloud2',
                           'marking': True, 'clearing': False, 'observation_persistence': 0.0,
                           'min_obstacle_height': -10.0, 'max_obstacle_height': 10.0,
                           'obstacle_min_range': near, 'obstacle_max_range': far},
            # 원래의 카메라 좌표를 유지하므로 광선 원점이 base_link로 잘못 옮겨지지 않는다.
            # NaN/무한대는 가짜 최대거리 광선으로 바꾸지 않는다.
            'depth_clear': {'topic': '/depth_obstacles/clearing_points', 'data_type': 'PointCloud2',
                            'marking': False, 'clearing': True, 'observation_persistence': 0.0,
                            'min_obstacle_height': -10.0, 'max_obstacle_height': 10.0,
                            'raytrace_min_range': 0.0, 'raytrace_max_range': far}}
    return result


def checked_extrinsics(config):
    if config.get('calibrated') is not True:
        raise ValueError('Camera extrinsics must be measured and calibrated: true')
    parent, child = config.get('parent_frame'), config.get('child_frame')
    if not parent or not child or parent == child or child != 'camera_link':
        raise ValueError('Expected distinct parent and camera_link child frames')
    names = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
    if not all(isinstance(config.get(n), (float, int)) and not isinstance(config[n], bool)
               and math.isfinite(config[n]) for n in names):
        raise ValueError('Camera extrinsics need six finite measured values')
    return ['--frame-id', parent, '--child-frame-id', child] + [
        item for n in names for item in ('--'+n, str(config[n]))]
