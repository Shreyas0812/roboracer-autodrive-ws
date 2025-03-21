#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu

from ament_index_python.packages import get_package_share_directory

class WaypointLoggerNode(Node):
    def __init__(self):
        super().__init__("waypoint_logger_node")
        self.get_logger().info("Waypoint Logger Node has been started.")


        waypoint_file_path = "/home/shreyas/Documents/Roboraacer/roboracer-ws/src/pure_pursuit/config/waypoints.csv"
        orientation_file_path = "/home/shreyas/Documents/Roboraacer/roboracer-ws/src/pure_pursuit/config/orientations.csv"
        
        self.waypoint_file = open(waypoint_file_path, "w")
        self.orientation_file = open(orientation_file_path, "w")

        ips_topic = "/autodrive/f1tenth_1/ips" # position
        imu_topic = "/autodrive/f1tenth_1/imu" # orientation

        self.ips_subscriber = self.create_subscription(
            Point,
            ips_topic,
            self.ips_callback,
            QoSProfile(depth=10)
        )

        self.imu_subscriber = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            QoSProfile(depth=10)
        )

    def ips_callback(self, msg):
        # self.get_logger().info(f"IPS Callback: {msg.x}, {msg.y}, {msg.z}")
        self.waypoint_file.write(f"{msg.x}, {msg.y}, {msg.z}\n")

    def imu_callback(self, msg):
        # self.get_logger().info(f"IMU Callback: {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w}")
        self.orientation_file.write(f"{msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w}\n")

    def close_files(self):
        self.waypoint_file.close()
        self.orientation_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointLoggerNode()

    try:
        rclpy.spin(node)
    except:
        pass
    finally:
        node.close_files()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()