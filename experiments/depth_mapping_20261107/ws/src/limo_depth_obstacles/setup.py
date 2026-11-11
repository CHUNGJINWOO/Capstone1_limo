from glob import glob
from setuptools import setup

setup(name='limo_depth_obstacles', version='0.1.0', packages=['limo_depth_obstacles'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/limo_depth_obstacles']),
                  ('share/limo_depth_obstacles', ['package.xml']),
                  ('share/limo_depth_obstacles/config', glob('config/*.yaml')),
                  ('share/limo_depth_obstacles/launch', glob('launch/*.py'))],
      install_requires=['setuptools'], license='Apache-2.0',
      entry_points={'console_scripts': ['depth_obstacle_filter = limo_depth_obstacles.node:main']})
