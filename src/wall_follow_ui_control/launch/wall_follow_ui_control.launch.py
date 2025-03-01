from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="wall_follow_ui_control",
            executable="wall_follow_ui_control_node.py",
            name="wall_follow_ui_control_node",
            output="screen",

        ),
        Node(
            package="wall_follow_ui_control",
            executable="wall_follow_ui_control",
            name="wall_follow_ui_control_node",
            output="screen",
        )
    ])