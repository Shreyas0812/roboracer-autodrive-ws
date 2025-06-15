from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    # Static transform publisher to publish a static transform from 'map' to 'world'
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
        name='static_transform_publisher'
    )
    ld.add_action(static_transform_publisher)

    visualise_node = Node(
       package='simple_map',
       executable='visualise_node.py',
       name='visualise_node',
       output='screen'
    )
    ld.add_action(visualise_node)
    
    return ld