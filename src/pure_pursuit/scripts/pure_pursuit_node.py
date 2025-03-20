#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")
        self.get_logger().info("Pure Pursuit Node has been started.")

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()