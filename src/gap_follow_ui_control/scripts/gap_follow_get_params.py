#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class GapFollowGetParamsNode(Node):
    def __init__(self):
        super().__init__("gap_follow_get_params")
        self.get_logger().info("Gap Follow Get Params Node Started")

def main(args=None):
    rclpy.init(args=args)
    node = GapFollowGetParamsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()