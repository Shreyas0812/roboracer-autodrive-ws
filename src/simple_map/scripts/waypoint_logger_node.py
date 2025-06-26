#!/usr/bin/env python3

import numpy as np
from transforms3d.euler import quat2euler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int32

from std_srvs.srv import Empty

from ament_index_python.packages import get_package_share_directory

class WaypointLoggerNode(Node):
    def __init__(self):
        super().__init__("waypoint_logger_node")
        self.get_logger().info("Waypoint Logger Node has been started.")

        self.declare_parameter('ips_topic', '/autodrive/f1tenth_1/ips')
        self.declare_parameter('imu_topic', '/autodrive/f1tenth_1/imu')
        self.declare_parameter('scan_topic', '/autodrive/f1tenth_1/lidar')
        self.declare_parameter('lap_count_topic', '/autodrive/f1tenth_1/lap_count')
        self.declare_parameter('window_size', 20)

        self.declare_parameter('reset_wp_logging_service', 'reset_waypoint_logging')

        ips_topic = self.get_parameter('ips_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        lap_count_topic = self.get_parameter('lap_count_topic').get_parameter_value().string_value
        
        self.window_size = self.get_parameter('window_size').get_parameter_value().integer_value

        reset_wp_logging_service = self.get_parameter('reset_wp_logging_service').get_parameter_value().string_value

        package_share_directory = get_package_share_directory('simple_map')

        self.wp_filepath = f'{package_share_directory}/config/waypoints.csv'
        self.centerline_filepath = f'{package_share_directory}/config/centerline_waypoints.csv'

        # file handling
        self.wp_file = None
        self.centerline_file = None

        # lap state
        self.previous_lap = None
        self.is_saving_wp = False
        self.lap_change_count = 0

        # QoS profile for subscribers
        qos_profile_sub = QoSProfile(depth=10)

        # Subscribers
        self.ips_subscriber = self.create_subscription(Point, ips_topic, self.ips_callback, qos_profile_sub)
        self.imu_subscriber = self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile_sub)
        self.scan_subscriber = self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sub)
        self.lap_count_subscriber = self.create_subscription(Int32, lap_count_topic, self.lap_count_callback, qos_profile_sub)

        # reset service to start/stop saving waypoints
        self.reset_service = self.create_service(Empty, reset_wp_logging_service, self.reset_lap_count_callback)

    def ips_callback(self, msg):
        # self.get_logger().info(f"Received IPS data: {msg}")

        if self.is_saving_wp and self.wp_file:
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
        if self.is_saving_wp:
            centerline_point = self.calculate_centerline_point(msg)
            # self.get_logger().info(f"Calculated centerline point: {centerline_point}")

            if centerline_point and self.centerline_file:
                self.centerline_file.write(f"{centerline_point[0]}, {centerline_point[1]}\n")

    def lap_count_callback(self, msg):
        # self.get_logger().info(f"Received lap count: {msg.data}")
        cur_lap = msg.data

        if self.previous_lap is not None and cur_lap != self.previous_lap:
            self.lap_change_count += 1
            self.get_logger().info(f"Lap changed from {self.previous_lap} to {cur_lap}. Lap change count: {self.lap_change_count}")

            if self.lap_change_count == 1:
                self.start_saving_waypoints()
            elif self.lap_change_count == 2:
                self.stop_saving_waypoints()
                # self.lap_change_count = 0

        self.previous_lap = cur_lap    

    # ros2 service call /reset_waypoint_logging std_srvs/srv/Empty
    def reset_lap_count_callback(self, request, response):
        """Reset lap count and waypoint saving state."""
        self.get_logger().info("Resetting lap count and waypoint saving state.")
        self.stop_saving_waypoints()
        self.previous_lap = None
        self.lap_change_count = 0
        self.is_saving_wp = False
        return response

    ######################################################################################################################################################

    def start_saving_waypoints(self):
        """Start saving waypoints to file."""
        if not self.is_saving_wp:
            self.wp_file = open(self.wp_filepath, 'w')
            self.centerline_file = open(self.centerline_filepath, 'w')
            self.is_saving_wp = True
            self.get_logger().info(f"Started saving waypoints to {self.wp_filepath} and \ncenterline waypoints to {self.centerline_filepath}.")

    def stop_saving_waypoints(self):
        """Stop saving waypoints to file."""
        if self.is_saving_wp:
            if self.wp_file:
                self.wp_file.close()
                self.wp_file = None
            if self.centerline_file:
                self.centerline_file.close()
                self.centerline_file = None
            self.is_saving_wp = False
            self.get_logger().info("Stopped saving waypoints.")

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