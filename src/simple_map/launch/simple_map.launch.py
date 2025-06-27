from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Declare a boolean launch argument
    launch_simple_map_arg = DeclareLaunchArgument(
        'launch_simple_map',
        default_value='true',
        description='Whether to launch the simple map node'
    )

    launch_waypoint_logger_arg = DeclareLaunchArgument(
        'launch_waypoint_logger',
        default_value='true',
        description='Whether to launch the waypoint logger node'
    )

    ld = LaunchDescription()
    ld.add_action(launch_simple_map_arg)
    ld.add_action(launch_waypoint_logger_arg)


    # Static transform publisher to publish a static transform from 'map' to 'world'
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
        name='static_transform_publisher'
    )
    ld.add_action(static_transform_publisher)

    # Launch the simple map node conditionally
    simple_map_node = Node(
        package='simple_map',
        executable='simple_map_node.py',
        name='simple_map_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_simple_map'))
    )
    ld.add_action(simple_map_node)

    # Launch the waypoint logger node conditionally
    waypoint_logger_node = Node(
        package='simple_map',
        executable='waypoint_logger_node.py',
        name='waypoint_logger_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_waypoint_logger'))
    )
    ld.add_action(waypoint_logger_node)

    # Launch the waypoint saver service node conditionally
    waypoint_saver_service = Node(
        package='simple_map',
        executable='waypoint_saver_service.py',
        name='waypoint_saver_service',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_waypoint_logger'))
    )
    ld.add_action(waypoint_saver_service)

    return ld

    # return LaunchDescription([
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
    #         name='static_transform_publisher'
    #     ),
    #     Node(
    #         package='simple_map',
    #         executable='simple_map_node.py',
    #         name='simple_map_node',
    #         output='screen'
    #     ),
    #     # Node(
    #     #     package='simple_map',
    #     #     executable='simple_map_node',
    #     #     name='simple_map_node',
    #     #     output='screen'
    #     # ),
    # ])