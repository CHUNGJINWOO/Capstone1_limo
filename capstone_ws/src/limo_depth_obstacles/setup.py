from setuptools import find_packages, setup

package_name = 'limo_depth_obstacles'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LIMO Development Team',
    maintainer_email='developer@todo.todo',
    description='Orbbec DaBai 3D Depth Obstacle PointCloud Filtering and Ground Segmentation for LIMO Pro',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_obstacle_filter_node = limo_depth_obstacles.depth_obstacle_filter_node:main',
        ],
    },
)
