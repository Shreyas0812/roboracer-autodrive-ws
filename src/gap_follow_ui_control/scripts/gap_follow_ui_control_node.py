#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from roboracer_interfaces.msg import CarControlGapFollow
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

import numpy as np

class GapFollowUIControlNode(Node):
    def __init__(self):
        super().__init__("gap_follow_ui_control_node")
        self.get_logger().info("Gap Follow UI Control Node Started")

        self.throttle = 0.2
        self.windowHalf = 5
        self.disparityExtender = 5
        self.maxActionableDist = 2.0


        qos_profile = QoSProfile(depth=10)

        # Subscriber Topics
        gap_follow_params_sub_topic = "gap_follow_params"
        lidar_sub_topic = '/autodrive/f1tenth_1/lidar'

        self.gap_follow_params_sub_ = self.create_subscription(CarControlGapFollow, gap_follow_params_sub_topic, self.gap_follow_params_callback, qos_profile)
        self.lidar_sub_ = self.create_subscription(LaserScan, lidar_sub_topic, self.lidar_callback, qos_profile)

        # Publisher topics
        steering_pub_topic = '/autodrive/f1tenth_1/steering_command'
        throttle_pub_topic = '/autodrive/f1tenth_1/throttle_command'

        self.throttle_pub = self.create_publisher(Float32, throttle_pub_topic, qos_profile)
        self.steering_pub = self.create_publisher(Float32, steering_pub_topic, qos_profile)

    def gap_follow_params_callback(self, msg):
        if self.throttle != msg.throttle:
            self.throttle = msg.throttle 

        if self.windowHalf != msg.window_half_size:
            self.windowHalf = msg.window_half_size

        if self.disparityExtender != msg.disparity_extender:
            self.disparityExtender = msg.disparity_extender

        if self.maxActionableDist != msg.max_actionable_dist:
            self.maxActionableDist = msg.max_actionable_dist


    def publish_to_car(self, steering_angle, throttle):
        """
        Publish the steering angle and throttle to the car.

        Args:
            steering_angle: Steering angle in radians
            throttle: Throttle value
        Returns:
            None
        """

        self.get_logger().info(f"Steering angle: {steering_angle}, Throttle: {throttle}", throttle_duration_sec=1.0)

        steering_angle = np.clip(steering_angle, -1, 1) # Limit steering angle to [-30, 30] degrees

        steering_msg = Float32()
        steering_msg.data = steering_angle 

        throttle_msg = Float32()
        throttle_msg.data = throttle 

        self.steering_pub.publish(steering_msg)
        self.throttle_pub.publish(throttle_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = GapFollowUIControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()