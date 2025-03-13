#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import json

from roboracer_interfaces.msg import CarControlGapFollow

from ament_index_python.packages import get_package_share_directory

class GapFollowGetParamsNode(Node):
    def __init__(self):
        super().__init__("gap_follow_get_params")
        self.get_logger().info("Gap Follow Get Params Node Started")

        qos_profile = QoSProfile(depth=10)

        self.gap_follow_ui_control_publisher_ = self.create_publisher(CarControlGapFollow, "gap_follow_params", qos_profile)
        
        self.gap_follow_ui_control_timer_ = self.create_timer(10.0, self.publish_gap_follow_ui_control)
    
    def publish_gap_follow_ui_control(self):
        self.get_logger().info("Publishing Gap Follow UI Control Parameters")

        gap_follow_ui_control_msg = CarControlGapFollow()
        # gap_follow_ui_control_msg.throttle = 0.2
        # gap_follow_ui_control_msg.window_half_size = 5
        # gap_follow_ui_control_msg.disparity_extender = 5
        # gap_follow_ui_control_msg.max_actionable_dist = 2.0

        package_share_dir = get_package_share_directory("gap_follow_ui_control")
        absolute_file_path = package_share_dir + "/config/gap_follow_params.json"

        with open(absolute_file_path, 'r') as file:
            config_data = json.load(file)

            if config_data["throttle"] is None:
                config_data["throttle"] = 404
            
            gap_follow_ui_control_msg.throttle = float(config_data["throttle"])
            gap_follow_ui_control_msg.window_half_size = int(config_data["window_half_size"])
            gap_follow_ui_control_msg.disparity_extender = int(config_data["disparity_extender"])
            gap_follow_ui_control_msg.max_actionable_dist = float(config_data["max_actionable_dist"])

            #Logging:
            self.get_logger().info(f'\nthrottle: {config_data["throttle"]}')
            self.get_logger().info(f'window_half_size: {config_data["window_half_size"]}')
            self.get_logger().info(f'disparity_extender: {config_data["disparity_extender"]}')
            self.get_logger().info(f'max_actionable_dist: {config_data["max_actionable_dist"]}\n')

            self.gap_follow_ui_control_publisher_.publish(gap_follow_ui_control_msg)

            # {'throttle': '0.1', 'window_half_size': '5', 'disparity_extender': '5', 'max_actionable_dist': '2.0'}

def main(args=None):
    rclpy.init(args=args)
    node = GapFollowGetParamsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()