#!/usr/bin/env python3

import os
import shutil
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from roboracer_interfaces.srv import SaveWaypoints  

class WaypointSaverService(Node):
    def __init__(self):
        super().__init__('waypoint_saver_service')
        self.srv = self.create_service(SaveWaypoints, 'save_waypoints', self.save_waypoints_callback)

        self.get_logger().info("Waypoint Saver Service has been started.")

    def save_waypoints_callback(self, request, response):
        try:
            # Get the full path to the source file in the package
            package_share_directory = get_package_share_directory('simple_map')
            source_path = os.path.join(package_share_directory, 'config', request.source_filename)

            # Check if the source file exists
            if not os.path.isfile(source_path):
                response.success = False
                response.message = f"Source file does not exist: {source_path}"
                self.get_logger().error(response.message)
                return response

            # Ensure the destination directory exists
            os.makedirs(os.path.dirname(request.destination_filepath), exist_ok=True)

            # Copy the file to the destination
            shutil.copyfile(source_path, request.destination_filepath)

            response.success = True
            response.message = f"Waypoints saved successfully to {request.destination_filepath}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to save waypoints: {str(e)}"
            self.get_logger().error(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSaverService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
