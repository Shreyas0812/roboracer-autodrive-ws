#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan

class WaypointLoggerNode(Node):
    def __init__(self):
        super().__init__("waypoint_logger_node")
        self.get_logger().info("Waypoint Logger Node has been started.")

        self.declare_parameter('ips_topic', '/autodrive/f1tenth_1/ips')
        self.declare_parameter('scan_topic', '/autodrive/f1tenth_1/lidar')

        ips_topic = self.get_parameter('ips_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        # TODO: Change this later for dynamic file path
        wp_filepath = '/home/shreyas/Documents/Roboraacer/roboracer-ws/src/simple_map/config/waypoints.csv'    
        self.wp_file = open(wp_filepath, 'w')
        self.get_logger().info(f"Waypoints will be saved to: {wp_filepath}")

        centerline_filepath = '/home/shreyas/Documents/Roboraacer/roboracer-ws/src/simple_map/config/centerline_waypoints.csv'
        self.centerline_file = open(centerline_filepath, 'w')
        self.get_logger().info(f"Centerline waypoints will be saved to: {centerline_filepath}")

        # QoS profile for subscribers
        qos_profile_sub = QoSProfile(depth=10)

        # Subscribers
        self.ips_subscriber = self.create_subscription(Point, ips_topic, self.ips_callback, qos_profile_sub)
        self.scan_subscriber = self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sub)

    def ips_callback(self, msg):
        # self.get_logger().info(f"Received IPS data: {msg}")
        self.wp_file.write(f"{msg.x}, {msg.y}\n")

        self.cur_pos = (msg.x, msg.y)

    def scan_callback(self, msg):
        self.get_logger().info(f"Received LaserScan data: {len(msg.ranges)} ranges")

        centerline_point = self.calculate_centerline_point(msg)

        self.get_logger().info(f"Calculated centerline point: {centerline_point}")

        if centerline_point is not None:
            self.centerline_file.write(f"{centerline_point[0]}, {centerline_point[1]}\n")
            self.get_logger().info(f"Centerline point saved: {centerline_point}")

    def calculate_centerline_point(self, scan_msg, window_size=5):

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        valid_indices = np.where((ranges > scan_msg.range_min) & 
                                 (ranges < scan_msg.range_max) &
                                 (~np.isnan(ranges)) &
                                 (~np.isinf(ranges)))[0]
        
        if len(valid_indices) < window_size * 2:
            self.get_logger().warn("Not enough valid ranges to calculate centerline point.")
            return None
        
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        left_indices = valid_angles > 0
        right_indices = valid_angles < 0

        if np.sum(left_indices) >= window_size and np.sum(right_indices) >= window_size:
            left_ranges = valid_ranges[left_indices]
            right_ranges = valid_ranges[right_indices]
            left_angles = valid_angles[left_indices]
            right_angles = valid_angles[right_indices]

            left_min_index = np.argmin(left_ranges)
            right_min_index = np.argmin(right_ranges)

            left_window_start = max(0, left_min_index - window_size // 2)
            left_window_end = min(len(left_ranges), left_min_index + window_size // 2 + 1)

            right_window_start = max(0, right_min_index - window_size // 2)
            right_window_end = min(len(right_ranges), right_min_index + window_size // 2 + 1)


            # Average the ranges
            left_avg_range = np.mean(left_ranges[left_window_start:left_window_end])
            right_avg_range = np.mean(right_ranges[right_window_start:right_window_end])

            # Average the angles
            left_avg_angle = np.mean(left_angles[left_window_start:left_window_end])
            right_avg_angle = np.mean(right_angles[right_window_start:right_window_end])

            # Convert to Cartesian coordinates
            left_wall_x = left_avg_range * np.cos(left_avg_angle)
            left_wall_y = left_avg_range * np.sin(left_avg_angle)

            right_wall_x = right_avg_range * np.cos(right_avg_angle)
            right_wall_y = right_avg_range * np.sin(right_avg_angle)

            centerline_x = (left_wall_x + right_wall_x) / 2.0
            centerline_y = (left_wall_y + right_wall_y) / 2.0

            return (centerline_x, centerline_y)

        return None

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()