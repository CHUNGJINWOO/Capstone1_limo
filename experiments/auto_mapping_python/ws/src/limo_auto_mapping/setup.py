from glob import glob
from setuptools import setup
setup(name='limo_auto_mapping', version='0.1.0', packages=['limo_auto_mapping'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/limo_auto_mapping']),
                  ('share/limo_auto_mapping', ['package.xml', 'UPSTREAM_LICENSE']),
                  ('share/limo_auto_mapping/config', glob('config/*.yaml')),
                  ('share/limo_auto_mapping/launch', glob('launch/*.py'))],
      install_requires=['setuptools'], license='Apache-2.0',
      entry_points={'console_scripts': ['mapping_server = limo_auto_mapping.server:main',
                                          'frontier_explorer = limo_auto_mapping.explorer:main']})
