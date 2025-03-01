#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from roboracer_interfaces.msg import CarControlWallFollow

class WallFollowGetParams(Node):
    def __init__(self):
        super().__init__("wall_follow_ui_control_node")
        self.get_logger().info("wall follow get params node")

        qos_profile = QoSProfile(depth=10)

        self.wall_follow_ui_control_publisher_ = self.create_publisher(CarControlWallFollow, "wall_follow_params", qos_profile)

        self.wall_follow_ui_control_timer_ = self.create_timer(1.0, self.publish_wall_follow_ui_control)
    
    def publish_wall_follow_ui_control(self):
        wall_follow_ui_control_msg = CarControlWallFollow()
        wall_follow_ui_control_msg.kp = 2.4
        wall_follow_ui_control_msg.kd = 0.8
        wall_follow_ui_control_msg.ki = 0.0

        self.wall_follow_ui_control_publisher_.publish(wall_follow_ui_control_msg)

def main(args=None):
    rclpy.init(args=args)
    wall_follow_ui_control_node =  WallFollowGetParams()
    rclpy.spin(wall_follow_ui_control_node)
    wall_follow_ui_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()