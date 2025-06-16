#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Pose, Quaternion, Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA

from ament_index_python.packages import get_package_share_directory

from simple_map.utils import load_waypoints

class VisualiseNode(Node):
    def __init__(self):
        super().__init__("visualise_node")
        self.get_logger().info("Visualise Node has been started.")

        self.declare_parameter('waypoints_filepath', 'waypoints.csv')
        self.declare_parameter('centerline_filepath', 'centerline_waypoints.csv')
        self.declare_parameter('visualize_loop', True)

        waypoints_filepath = self.get_parameter('waypoints_filepath').get_parameter_value().string_value
        centerline_filepath = self.get_parameter('centerline_filepath').get_parameter_value().string_value
        visualize_loop = self.get_parameter('visualize_loop').get_parameter_value().bool_value

        package_share_directory = get_package_share_directory('simple_map')
        waypoints_filepath = f"{package_share_directory}/config/{waypoints_filepath}"
        centerline_filepath = f"{package_share_directory}/config/{centerline_filepath}"

        # Load waypoints
        self.waypoints = load_waypoints(waypoints_filepath)
        self.centerline_waypoints = load_waypoints(centerline_filepath)

        # QoS profile for publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers for visualization
        self.waypoint_marker_publisher = self.create_publisher(
            MarkerArray, '/visualization/waypoints', qos_profile)
        self.centerline_marker_publisher = self.create_publisher(
            MarkerArray, '/visualization/centerline_waypoints', qos_profile)

        if visualize_loop:
            # Create a timer that calls visualization function every 5 seconds
            self.timer = self.create_timer(5.0, self.timer_callback)
        else:
            # Visualize once
            self.visualize_waypoints()
            self.visualize_centerline_waypoints()

    def timer_callback(self):
        """Timer callback to continuously visualize waypoints"""
        self.visualize_waypoints()
        self.visualize_centerline_waypoints()

    def visualize_waypoints(self):
        """Visualize regular waypoints as yellow spheres"""
        if len(self.waypoints) == 0:
            self.get_logger().warn("No waypoints to visualize", throttle_duration_sec=2.0)
            return

        marker_array = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            x, y = wp

            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose = Pose()
            marker.pose.position = Point(x=float(x), y=float(y), z=0.0)
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            marker.scale = Vector3(x=0.15, y=0.15, z=0.15)
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)  # Yellow

            marker.lifetime.sec = 0  # Persistent markers

            marker_array.markers.append(marker)

        self.waypoint_marker_publisher.publish(marker_array)
        self.get_logger().info(f"Visualized {len(self.waypoints)} waypoints", throttle_duration_sec=2.0)

    def visualize_centerline_waypoints(self):
        """Visualize centerline waypoints as green spheres"""
        if len(self.centerline_waypoints) == 0:
            self.get_logger().warn("No centerline waypoints to visualize", throttle_duration_sec=2.0)
            return

        marker_array = MarkerArray()
        for i, wp in enumerate(self.centerline_waypoints):
            x, y = wp

            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i + 10000  # Offset ID to avoid conflicts with regular waypoints
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose = Pose()
            marker.pose.position = Point(x=float(x), y=float(y), z=0.1)  # Slightly elevated
            marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)  # Green

            marker.lifetime.sec = 0  # Persistent markers

            marker_array.markers.append(marker)

        self.centerline_marker_publisher.publish(marker_array)
        self.get_logger().info(f"Visualized {len(self.centerline_waypoints)} centerline waypoints", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = VisualiseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()