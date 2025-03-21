from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
            name='static_transform_publisher'
        ),
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node.py',
            name='pure_pursuit_node',
            output='screen'
        ),
        # Node(
        #     package='pure_pursuit',
        #     executable='pure_pursuit_node',
        #     name='pure_pursuit_node',
        #     output='screen'
        # ),
    ])