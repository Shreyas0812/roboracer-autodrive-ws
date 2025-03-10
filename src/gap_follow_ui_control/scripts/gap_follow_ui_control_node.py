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
        self.window_half_size = 5
        self.disparity_extender = 5
        self.max_actionable_dist = 2.0

        gap_follow_params_sub_topic = "gap_follow_params"

        qos_profile = QoSProfile(depth=10)

        # Subscriber Topics
        self.gap_follow_params_sub_ = self.create_subscription(CarControlGapFollow, gap_follow_params_sub_topic, self.gap_follow_params_callback, qos_profile)

    def gap_follow_params_callback(self, msg):
        if self.throttle != msg.throttle:
            self.throttle = msg.throttle 

        if self.window_half_size != msg.window_half_size:
            self.window_half_size = msg.window_half_size

        if self.disparity_extender != msg.disparity_extender:
            self.disparity_extender = msg.disparity_extender

        if self.max_actionable_dist != msg.max_actionable_dist:
            self.max_actionable_dist = msg.max_actionable_dist


def main(args=None):
    rclpy.init(args=args)
    node = GapFollowUIControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()