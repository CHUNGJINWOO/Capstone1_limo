from setuptools import setup
package_name = 'limo_robot_agent'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True, maintainer='capstone', maintainer_email='noreply@example.com',
    description='Capstone1 fleet robot agent', license='TODO: License declaration',
    entry_points={'console_scripts': ['robot_agent = limo_robot_agent.agent_node:main', 'mock_robot = limo_robot_agent.mock_robot_node:main']},
)
