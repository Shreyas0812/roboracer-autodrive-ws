#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class GapFollowUIControlNode(Node):
    def __init__(self):
        super().__init__("gap_follow_ui_control_node")
        self.get_logger().info("Gap Follow UI Control Node Started")

def main(args=None):
    rclpy.init(args=args)
    node = GapFollowUIControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()