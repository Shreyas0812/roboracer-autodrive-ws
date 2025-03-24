
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='waypoint_logger_node.py',
            name='waypoint_logger_node',
            output='screen'
        ),
    ])