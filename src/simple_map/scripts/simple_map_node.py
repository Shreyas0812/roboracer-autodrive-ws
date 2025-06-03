#!/usr/bin/env python3

import numpy as np
from transforms3d.euler import quat2euler
import yaml
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import OccupancyGrid

from std_srvs.srv import Empty

from simple_map.utils import get_grid_coordinates, get_world_coordinates, update_grid_with_ray

class SimpleMapNode(Node):
    def __init__(self):
        super().__init__("simple_map_node")
        self.get_logger().info("Simple Map Node from Python has been started")

        self.declare_parameter('ips_topic', '/autodrive/f1tenth_1/ips')
        self.declare_parameter('imu_topic', '/autodrive/f1tenth_1/imu')
        self.declare_parameter('scan_topic', '/autodrive/f1tenth_1/lidar')

        self.declare_parameter("map_topic", "/map")

        self.declare_parameter('map_height', 2000)
        self.declare_parameter('map_width', 2000)
        self.declare_parameter('map_resolution', 0.01)
        self.declare_parameter('map_origin', (-10.0, -10.0))

        self.declare_parameter('expand_occ_size', 3) # Minimum size of the area to expand occupancy around the endpoint

        self.declare_parameter('map_name', 'simple_map')
        self.declare_parameter('map_folder', 'maps')

        # Subscribe to topics
        ips_topic = self.get_parameter('ips_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        
        # Publish map topic
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

        # Map parameters
        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_origin = self.get_parameter('map_origin').get_parameter_value().double_array_value

        self.expand_occ_size = self.get_parameter('expand_occ_size').get_parameter_value().integer_value

        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.map_folder = self.get_parameter('map_folder').get_parameter_value().string_value

        # Default Occupancy Grid
        self.occupancy_grid = np.full((self.map_height, self.map_width), -1, dtype=np.int8)

        # QoS profile for subscribers
        qos_profile_sub = QoSProfile(depth=10)

        # Subscribers
        self.ips_subscriber = self.create_subscription(Point, ips_topic, self.ips_callback, qos_profile_sub)
        self.imu_subscriber = self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile_sub)
        self.scan_subscriber = self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sub)

        # QoS profile for publishers
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Publisher for map
        self.map_publisher = self.create_publisher(OccupancyGrid, map_topic, qos_profile_pub)

        # Service to save the map
        # Save the map by calling the service: ros2 service call /save_map std_srvs/srv/Empty
        self.save_map_service = self.create_service(Empty, '/save_map', self.save_map_callback)

    def save_map_callback(self, request, response):
        # Save the occupancy grid to a file
        self.get_logger().info("Saving map...")
        map_path = f"{self.map_folder}/{self.map_name}"
        self.save_map_to_file(map_path)
        return response

    def save_map_to_file(self, filename_prefix):
        # Convert OccupancyGrid to a PGM format

        pgm_grid = np.zeros_like(self.occupancy_grid, dtype=np.uint8)
        pgm_grid[self.occupancy_grid == -1] = 205  # Unknown cells
        pgm_grid[self.occupancy_grid == 0] = 255   # Free cells
        pgm_grid[self.occupancy_grid == 100] = 0    # Occupied cells

        # Flip the grid vertically to match the PGM format
        pgm_grid = np.flipud(pgm_grid)

        # Save PGM image
        img = Image.fromarray(pgm_grid, mode='L')
        img.save(f"{filename_prefix}.pgm")

        # Save YAML file
        metadata = {
            'image': f"{filename_prefix}.pgm",
            'resolution': float(self.map_resolution),
            'origin': [float(self.map_origin[0]), float(self.map_origin[1]), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }

        with open(f"{filename_prefix}.yaml", 'w') as yaml_file:
            yaml.dump(metadata, yaml_file, default_flow_style=False)

        
    def ips_callback(self, msg):
        # self.get_logger().info(f"Received IPS data: {msg.x}, {msg.y}, {msg.z}", throttle_duration_sec=1.0)
        self.cur_pos = np.array([msg.x, msg.y, msg.z])

    def imu_callback(self, msg):
        # self.get_logger().info(f"Received IMU data: {msg}", throttle_duration_sec=1.0)

        self.cur_yaw = quat2euler([msg.orientation.w,
                                    msg.orientation.x,
                                    msg.orientation.y,
                                    msg.orientation.z,
                                    ])[2]

    def scan_callback(self, scan_msg):
        # self.get_logger().info(f"Received LaserScan data: {msg.ranges}", throttle_duration_sec=1.0)
        
        if not hasattr(self, 'cur_pos') or not hasattr(self, 'cur_yaw'):
            self.get_logger().warn("Current position or yaw not set yet. Waiting for pose data...")
            return
        
        scan_data = np.array(scan_msg.ranges)

        cur_grid_x, cur_grid_y = get_grid_coordinates(self.cur_pos[0], self.cur_pos[1], self.map_origin, self.map_resolution)

        cur_grid_x = np.clip(cur_grid_x, 0, self.map_width - 1)
        cur_grid_y = np.clip(cur_grid_y, 0, self.map_height - 1)

        updated_occupancy_grid = np.copy(self.occupancy_grid)

        for i, distance in enumerate(scan_data):
            if distance == float('inf'):
                distance = scan_msg.range_max

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            
            angle_world = angle + self.cur_yaw

            updated_occupancy_grid = update_grid_with_ray(
                updated_occupancy_grid,
                cur_grid_x,
                cur_grid_y,
                angle_world,
                distance,
                self.map_resolution,
                self.map_width,
                self.map_height,
                area_size=self.expand_occ_size
            )
        
        self.occupancy_grid = updated_occupancy_grid

        # Publish the occupancy grid
        self.publish_occupancy_grid()

    def publish_occupancy_grid(self):
        # Create and publish the updated occupancy grid
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = "map"

        occupancy_grid_msg.info.height = self.map_height
        occupancy_grid_msg.info.width = self.map_width
        occupancy_grid_msg.info.resolution = self.map_resolution
        occupancy_grid_msg.info.origin.position.x = self.map_origin[0]
        occupancy_grid_msg.info.origin.position.y = self.map_origin[1]

        occupancy_grid_msg.data = self.occupancy_grid.flatten().astype(np.int8).tolist()

        # Publish the updated occupancy grid
        self.map_publisher.publish(occupancy_grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()