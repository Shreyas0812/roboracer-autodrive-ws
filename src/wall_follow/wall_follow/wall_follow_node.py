#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        self.get_logger().info('Wall Follow Node Initialized.')

def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollowNode()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

