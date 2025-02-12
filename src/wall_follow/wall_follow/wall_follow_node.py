#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        self.get_logger().info('Wall Follow Node')

        self.declare_parameter('desired_dist_from_wall', 1.0)

        # Note: 0 is directly in front of the car, positive is clockwise
        self.declare_parameter('degreeTheta1', np.deg2rad(40)) # Start angle of the LiDAR to be considered

        self.declare_parameter('degreeTheta2', np.deg2rad(90)) # End angle of the LiDAR to be considered

        self.desired_dist_from_wall = self.get_parameter('desired_dist_from_wall').value
        self.theta1 = self.get_parameter('degreeTheta1').value
        self.theta2 = self.get_parameter('degreeTheta2').value

def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollowNode()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

