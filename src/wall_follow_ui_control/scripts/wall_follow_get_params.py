#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class WallFollowGetParams(Node):
    def __init__(self):
        super().__init__("wall_follow_ui_control_node")
        self.get_logger().info("Hello Python wall node get params")

def main(args=None):
    rclpy.init(args=args)
    wall_follow_ui_control_node =  WallFollowGetParams()
    rclpy.spin(wall_follow_ui_control_node)
    wall_follow_ui_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()