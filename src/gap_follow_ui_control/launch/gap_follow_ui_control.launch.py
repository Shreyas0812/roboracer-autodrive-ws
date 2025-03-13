from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="gap_follow_ui_control",
            executable="gap_follow_ui_control_node.py",
            name="gap_follow_ui_control_node",
            output="screen"
        ),
        Node(
            package="gap_follow_ui_control",
            executable="gap_follow_get_params.py",
            name="gap_follow_get_params",
            output="screen"
        )
    ])