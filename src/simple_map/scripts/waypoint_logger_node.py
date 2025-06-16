#!/usr/bin/env python3

import numpy as np
from transforms3d.euler import quat2euler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, LaserScan

from ament_index_python.packages import get_package_share_directory

class WaypointLoggerNode(Node):
    def __init__(self):
        super().__init__("waypoint_logger_node")
        self.get_logger().info("Waypoint Logger Node has been started.")

        self.declare_parameter('ips_topic', '/autodrive/f1tenth_1/ips')
        self.declare_parameter('imu_topic', '/autodrive/f1tenth_1/imu')
        self.declare_parameter('scan_topic', '/autodrive/f1tenth_1/lidar')
        self.declare_parameter('lap_count_topic', '/autodrive/f1tenth_1/lap_count')
        self.declare_parameter('window_size', 5)

        ips_topic = self.get_parameter('ips_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        lap_count_topic = self.get_parameter('lap_count_topic').get_parameter_value().string_value
        
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value

        package_share_directory = get_package_share_directory('simple_map')

        # TODO: Change this later for dynamic file path
        wp_filepath = f'{package_share_directory}/config/waypoints.csv'
        self.wp_file = open(wp_filepath, 'w')
        self.get_logger().info(f"Waypoints will be saved to: {wp_filepath}")

        centerline_filepath = f'{package_share_directory}/config/centerline_waypoints.csv'
        self.centerline_file = open(centerline_filepath, 'w')
        self.get_logger().info(f"Centerline waypoints will be saved to: {centerline_filepath}")

        # QoS profile for subscribers
        qos_profile_sub = QoSProfile(depth=10)

        # Subscribers
        self.ips_subscriber = self.create_subscription(Point, ips_topic, self.ips_callback, qos_profile_sub)
        self.imu_subscriber = self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile_sub)
        self.scan_subscriber = self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sub)
        # self.lap_count_subscriber = self.create_subscription(Int32, lap_count_topic, self.lap_count_callback, qos_profile_sub)

    def ips_callback(self, msg):
        # self.get_logger().info(f"Received IPS data: {msg}")
        self.wp_file.write(f"{msg.x}, {msg.y}\n")

        self.cur_pos = (msg.x, msg.y)

    def imu_callback(self, msg):
        # self.get_logger().info(f"Received IMU data: {msg}", throttle_duration_sec=1.0)

        self.cur_yaw = quat2euler([msg.orientation.w,
                                    msg.orientation.x,
                                    msg.orientation.y,
                                    msg.orientation.z,
                                    ])[2]
        
    def scan_callback(self, msg):
        # self.get_logger().info(f"Received LaserScan data: {len(msg.ranges)} ranges")

        centerline_point = self.calculate_centerline_point(msg)

        # self.get_logger().info(f"Calculated centerline point: {centerline_point}")

        if centerline_point is not None:
            self.centerline_file.write(f"{centerline_point[0]}, {centerline_point[1]}\n")
            # self.get_logger().info(f"Centerline point saved: {centerline_point}")

    def calculate_centerline_point(self, scan_msg):

        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        # Index when angle is -pi/2
        neg_90_index = self.angle_to_index(-np.pi/2, scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment, len(ranges))
        left_avg_range = self.average_around_index(ranges, neg_90_index, self.window_size)

        # self.get_logger().info(f"Left avg range: {left_avg_range}")

        # Index when angle is +pi/2
        pos_90_index = self.angle_to_index(np.pi/2, scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment, len(ranges))
        right_avg_range = self.average_around_index(ranges, pos_90_index, self.window_size)

        # self.get_logger().info(f"Right avg range: {right_avg_range}")

        if not hasattr(self, 'cur_pos') or not hasattr(self, 'cur_yaw'):
            self.get_logger().warn("Current position or yaw not set yet. Waiting for pose data...", throttle_duration_sec=2.0)
            return None
        
        left_point = self.get_point_at_angle_distance(self.cur_pos, self.cur_yaw, -np.pi/2, left_avg_range)
        right_point = self.get_point_at_angle_distance(self.cur_pos, self.cur_yaw, np.pi/2, right_avg_range)

        # Calculate the midpoint between left_point and right_point
        center_x = (left_point[0] + right_point[0]) / 2
        center_y = (left_point[1] + right_point[1]) / 2
        return (center_x, center_y)

    def angle_to_index(self, angle, angle_min, angle_max, angle_increment, num_ranges):
        angle = np.clip(angle, angle_min, angle_max)
        index = int(round((angle - angle_min) / angle_increment))
        index = np.clip(index, 0, num_ranges - 1)
        return index

    def average_around_index(self, range_data, index, window_size=5):
        start_idx = max(index - window_size, 0)
        end_idx = min(index + window_size + 1, len(range_data))
        return np.mean(range_data[start_idx:end_idx])

    def get_point_at_angle_distance(self, position, yaw, angle, distance):
        x, y = position
        global_angle = yaw + angle
        new_x = x + distance * np.cos(global_angle)
        new_y = y + distance * np.sin(global_angle)
        return (new_x, new_y)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()