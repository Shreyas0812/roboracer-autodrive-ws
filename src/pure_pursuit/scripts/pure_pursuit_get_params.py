#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import json

from roboracer_interfaces.msg import CarControlPurePursuit

from ament_index_python.packages import get_package_share_directory

class PurePursuitGetParamsNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_get_params_node")
        self.get_logger().info("Pure Pursuit Get Params Node has been started.")

        qos_profile = QoSProfile(depth=10)

        # Topics
        pure_pursuit_params_topic = "/pure_pursuit_params"

        self.pure_pursuit_params_publisher = self.create_publisher(
            CarControlPurePursuit,
            pure_pursuit_params_topic,
            qos_profile
        )

        self.pure_pursuit_params_publisher_timer = self.create_timer(10.0, self.publish_pure_pursuit_params)

    def publish_pure_pursuit_params(self):
        pure_pursuit_params_msg = CarControlPurePursuit()
        # pure_pursuit_params_msg.lookahead_distance = 1.2
        # pure_pursuit_params_msg.kp = 2.0
        # pure_pursuit_params_msg.throttle = 0.15

        package_share_dir = get_package_share_directory("pure_pursuit")
        absolute_file_path = package_share_dir + "/config/pure_pursuit_params.json"

        with open(absolute_file_path, 'r') as file:
            config_data = json.load(file)

            pure_pursuit_params_msg.lookahead_distance = float(config_data["lookahead_distance"])
            pure_pursuit_params_msg.kp = float(config_data["kp"])
            pure_pursuit_params_msg.throttle = float(config_data["throttle"])

            self.pure_pursuit_params_publisher.publish(pure_pursuit_params_msg)

def main(args=None):
    rclpy.init(args=args)

    pure_pursuit_get_params_node = PurePursuitGetParamsNode()

    rclpy.spin(pure_pursuit_get_params_node)

    pure_pursuit_get_params_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()