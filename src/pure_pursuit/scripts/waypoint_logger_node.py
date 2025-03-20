#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class WaypointLoggerNode(Node):
    def __init__(self):
        super().__init__("waypoint_logger_node")
        self.get_logger().info("Waypoint Logger Node has been started.")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()