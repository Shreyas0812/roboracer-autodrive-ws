#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Float32

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        self.get_logger().info("Safety Node")
        self.declare_parameter('safety_distance', 1.0)

        self.transform_base_link_to_laser = TransformStamped()
        self.transform_base_link_to_laser.header.frame_id = 'ego_racecar/base_link'
        self.transform_base_link_to_laser.child_frame_id = 'ego_racecar/laser_model'  
        self.transform_base_link_to_laser.transform.translation.x = 0.0
        self.transform_base_link_to_laser.transform.translation.y = 0.0
        self.transform_base_link_to_laser.transform.translation.z = 0.0
        self.transform_base_link_to_laser.transform.rotation.x = 0.0
        self.transform_base_link_to_laser.transform.rotation.y = 0.0
        self.transform_base_link_to_laser.transform.rotation.z = 0.0
        self.transform_base_link_to_laser.transform.rotation.w = 1.0

        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """

        self.qos_profile = QoSProfile(
            depth=10,
        )

        # Topics 
        self.throttle_pub_topic = "/autodrive/f1tenth_1/throttle_command" # range: -1 to 1 mapped to -100% to 100% (but can go more)

        # Topics 
        self.throttle_sub_topic = "/autodrive/f1tenth_1/throttle" # range: -1 to 1
        self.lidar_sub_topic = "/autodrive/f1tenth_1/lidar" # range: 0 to 10m

        self.speed = 0.0

        # TODO: create ROS subscribers and publishers.
        self.throttle_publisher = self.create_publisher(
            Float32, self.throttle_pub_topic, self.qos_profile
        )

        self.throttle_subscriber = self.create_subscription(
            Float32, self.throttle_sub_topic, self.subscribe_throttle, self.qos_profile
        )

        self.lidar_subscriber = self.create_subscription(
            LaserScan, self.lidar_sub_topic, self.subscribe_lidar, 10
        )

        qos_profile_tf_static = QoSProfile(
            # durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.tf2_static_subsciber = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf2_static_callback,
            qos_profile_tf_static
        )

    def subscribe_throttle(self, throttle_msg):
        # TODO: update current speed
        self.speed = throttle_msg.data

    def subscribe_lidar(self, scan_msg):
        # TODO: calculate TTC
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        valid_ranges = ranges[np.isfinite(ranges)]
        valid_angles = angles[np.isfinite(ranges)]


        x_laser = valid_ranges * np.cos(valid_angles)
        y_laser = valid_ranges * np.sin(valid_angles)

        x_base_link = x_laser + self.transform_base_link_to_laser.transform.translation.x
        y_base_link = y_laser + self.transform_base_link_to_laser.transform.translation.y
        z_base_link = self.transform_base_link_to_laser.transform.translation.z

        ranges_base_link = np.sqrt(x_base_link**2 + y_base_link**2 + z_base_link**2)


        range_rates = self.speed * np.cos(valid_angles)
        range_rates_mask = range_rates > 0


        TTC = np.zeros_like(ranges_base_link)
        TTC[range_rates_mask] = ranges_base_link[range_rates_mask] / range_rates[range_rates_mask]
        TTC[~range_rates_mask] = np.inf


        self.get_logger().info(f"Min TTC: {np.min(TTC)}")

        # TODO: publish command to brake
        if np.any(TTC[np.isfinite(TTC)] < self.get_parameter('safety_distance').value):

            self.get_logger().info(f"Emergency Brake! TTC: {np.min(TTC)}")

            brake_msg = Float32()
            brake_msg.data = 0.0
            self.throttle_publisher.publish(brake_msg)

    #CHECK: tf2_static_callback
    def tf2_static_callback(self, tf_msg):
        self.get_logger().info("TF Static Callback")
        for transform in tf_msg.transforms:
            if transform.header.frame_id == 'f1tenth_1' and transform.child_frame_id == 'lidar':
                self.get_logger().info(f"base_link: {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}")
                self.transform_base_link_to_laser = transform

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()