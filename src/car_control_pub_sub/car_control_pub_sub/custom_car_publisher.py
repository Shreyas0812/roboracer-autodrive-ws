#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

from std_msgs.msg import Float32

import numpy as np


class CustomCarPublisher(Node):
    def __init__(self):
        super().__init__("custom_car_publisher")

        self.qos_profile = QoSProfile(depth=10)

        # Topics 
        self.throttle_pub_topic = "/autodrive/f1tenth_1/throttle_command" # range: -1 to 1
        self.steering_pub_topic = "/autodrive/f1tenth_1/steering_command" # range: -0.52rad to 0.52rad

        # Values
        self.throttle_command_value = 0.20
        self.steering_command_value = 0.12 

        # Creating publishers

        self.throttle_publisher = self.create_publisher(
            Float32, self.throttle_pub_topic, self.qos_profile
        )
        self.steering_publisher = self.create_publisher(
            Float32, self.steering_pub_topic, self.qos_profile
        )

        self.timer = self.create_timer(0.1, self.publish_commands)

    def publish_commands(self):
        throttle_msg = Float32() # { "data": 0.0 }
        steering_msg = Float32() # { "data": 0.0 }

        throttle_msg.data = float(np.round(self.throttle_command_value, 3))
        steering_msg.data = float(np.round(self.steering_command_value, 3))

        self.throttle_publisher.publish(throttle_msg)
        self.steering_publisher.publish(steering_msg)

def main():
    rclpy.init()
    custom_car_publisher = CustomCarPublisher()
    rclpy.spin(custom_car_publisher)
    custom_car_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()