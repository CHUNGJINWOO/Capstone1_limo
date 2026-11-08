import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'limo_capstone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'config', 'ablation'), glob('config/ablation/*.yaml')),
        (os.path.join('share', package_name, 'maps'), [f for f in glob('maps/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'worlds'), [f for f in glob('worlds/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'urdf'), [f for f in glob('urdf/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'models', 'limo_eval'), glob('models/limo_eval/*')),
        (os.path.join('share', package_name, 'models', 'glass_partition'), glob('models/glass_partition/*')),
        (os.path.join('share', package_name, 'models', 'lounge_chair_sled'), glob('models/lounge_chair_sled/*')),
        (os.path.join('share', package_name, 'models', 'lounge_table'), glob('models/lounge_table/*')),
        (os.path.join('share', package_name, 'scripts'), [f for f in glob('scripts/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pcu',
    maintainer_email='user@todo.todo',
    description='LIMO Capstone Project Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_scan_matcher_odom_node = limo_capstone.lidar_scan_matcher_odom_node:main',
            'odom_tf_relay_node = limo_capstone.odom_tf_relay_node:main',
            'stall_detector_node = limo_capstone.stall_detector_node:main',
            'depth_to_pointcloud_fixed_node = limo_capstone.depth_to_pointcloud_fixed_node:main',
            'ground_truth_publisher_node = limo_capstone.ground_truth_publisher_node:main',
            'sensor_normalizer_node = limo_capstone.sensor_normalizer_node:main',
            'nav2_action_monitor_node = limo_capstone.nav2_action_monitor_node:main',
            'integrated_recorder_node = limo_capstone.integrated_recorder_node:main',
        ],
    },
)
