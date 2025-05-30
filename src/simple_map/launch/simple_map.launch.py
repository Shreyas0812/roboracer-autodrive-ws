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
            package='simple_map',
            executable='simple_map_node.py',
            name='simple_map_node',
            output='screen'
        ),
        Node(
            package='simple_map',
            executable='simple_map_node',
            name='simple_map_node',
            output='screen'
        ),
    ])