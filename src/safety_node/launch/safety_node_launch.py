from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    safety_node = Node(
            package='safety_node',
            executable='safety_node.py',
            name='safety_node',
            parameters=[{
                'safety_distance': 1.4
                }]
            )

    ld.add_action(safety_node)

    return ld
