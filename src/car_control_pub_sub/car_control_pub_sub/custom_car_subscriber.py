#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

import numpy as np


class CustomCarSubscriber(Node):
    def __init__(self):
        super().__init__("custom_car_subscriber")

        self.qos_profile = QoSProfile(depth=10)

        # Topics 
        self.throttle_sub_topic = "/autodrive/f1tenth_1/throttle" # range: -1 to 1
        self.steering_sub_topic = "/autodrive/f1tenth_1/steering" # range: -0.52rad to 0.52rad
        self.lidar_sub_topic = "/autodrive/f1tenth_1/lidar" # range: 0 to 10m

        # Creating subscribers

        self.throttle_subscriber = self.create_subscription(
            Float32, self.throttle_sub_topic, self.subscribe_throttle, self.qos_profile
        )

        self.steering_subscriber = self.create_subscription(
            Float32, self.steering_sub_topic, self.subscribe_steering, self.qos_profile
        )

        self.lidar_subscriber = self.create_subscription(
            LaserScan, self.lidar_sub_topic, self.subscribe_lidar, self.qos_profile
        )

    def subscribe_throttle(self, msg):
        self.get_logger().info(f"Throttle123: {msg.data}")
    
    def subscribe_steering(self, msg):
        self.get_logger().info(f"Steering: {msg.data}")
    
    def subscribe_lidar(self, msg):
        self.get_logger().info(f"Lidar: {msg}")
        

def main():
    rclpy.init()
    custom_car_subscriber = CustomCarSubscriber()
    rclpy.spin(custom_car_subscriber)
    custom_car_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()