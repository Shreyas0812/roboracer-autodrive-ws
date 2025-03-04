#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import json

from roboracer_interfaces.msg import CarControlWallFollow

from ament_index_python.packages import get_package_share_directory

class WallFollowGetParams(Node):
    def __init__(self):
        super().__init__("wall_follow_ui_control_node")
        self.get_logger().info("wall follow get params node")

        qos_profile = QoSProfile(depth=10)

        self.wall_follow_ui_control_publisher_ = self.create_publisher(CarControlWallFollow, "wall_follow_params", qos_profile)

        self.wall_follow_ui_control_timer_ = self.create_timer(10.0, self.publish_wall_follow_ui_control)
    
    def publish_wall_follow_ui_control(self):
        wall_follow_ui_control_msg = CarControlWallFollow()
        # wall_follow_ui_control_msg.kp = 2.4
        # wall_follow_ui_control_msg.kd = 1.0
        # wall_follow_ui_control_msg.ki = 0.0

        package_share_dir = get_package_share_directory("wall_follow_ui_control")
        absolute_file_path = package_share_dir + "/config/wall_follow_params.json"

        with open(absolute_file_path, 'r') as file:
            config_data = json.load(file)


            wall_follow_ui_control_msg.kp = float(config_data["kp"])
            wall_follow_ui_control_msg.kd = float(config_data["kd"])
            wall_follow_ui_control_msg.ki = float(config_data["ki"])

            self.get_logger().info(f'kp: {config_data["kp"]}')
            self.get_logger().info(f'kd: {config_data["kd"]}')
            self.get_logger().info(f'ki: {config_data["ki"]}')

            self.wall_follow_ui_control_publisher_.publish(wall_follow_ui_control_msg)

            # {'throttle': '0.1', 'lookahead_dist': '0.8', 'kp': '0.8', 'kd': '4.2', 'ki': '1.248', 'flag_reason': 'Latest', 'flag_msg': '213123'}

def main(args=None):
    rclpy.init(args=args)
    wall_follow_ui_control_node =  WallFollowGetParams()
    rclpy.spin(wall_follow_ui_control_node)
    wall_follow_ui_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()