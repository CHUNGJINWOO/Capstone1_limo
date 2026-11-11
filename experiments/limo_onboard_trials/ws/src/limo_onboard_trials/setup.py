from glob import glob
from setuptools import setup

setup(name='limo_onboard_trials', version='0.1.0', packages=['limo_onboard_trials'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/limo_onboard_trials']),
                  ('share/limo_onboard_trials', ['package.xml']),
                  ('share/limo_onboard_trials/config', glob('config/*')),
                  ('share/limo_onboard_trials/launch', glob('launch/*.py'))],
      install_requires=['setuptools'], license='Apache-2.0')
