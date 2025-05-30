#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class SimpleMapNode(Node):
    def __init__(self):
        super().__init__("simple_map_node")
        self.get_logger().info("Simple Map Node from Python has been started")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()