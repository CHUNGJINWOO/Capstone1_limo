from launch_ros.actions import Node


def agent(parameters):
    return Node(package='limo_robot_agent', executable='robot_agent',
                parameters=[{'mode': 'mock', 'simulated': True,
                             'allow_sim_safety_gate': False}, parameters], output='screen')
