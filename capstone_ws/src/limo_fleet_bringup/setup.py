from glob import glob
from setuptools import setup

name = 'limo_fleet_bringup'
setup(name=name, version='0.1.0', packages=[name], data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + name]),
    ('share/' + name, ['package.xml']),
    ('share/' + name + '/launch', glob('launch/*.launch.py')),
], install_requires=['setuptools'], maintainer='capstone',
    maintainer_email='noreply@example.com', license='Apache-2.0')
