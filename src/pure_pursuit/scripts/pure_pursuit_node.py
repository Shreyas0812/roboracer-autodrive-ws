#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ament_index_python.packages import get_package_share_directory

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA, Float32
from geometry_msgs.msg import Pose, Quaternion, Vector3, Point
from sensor_msgs.msg import Imu

import csv
import numpy as np
from scipy.interpolate import CubicSpline
from transforms3d.euler import quat2euler

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")
        self.get_logger().info("Pure Pursuit Node has been started.")

        # Tunable Parameters
        self.lookahead_distance = 1.2
        self.kp = 2.0
        self.wheelbase = 0.36  # Wheelbase of the vehicle in meters
        self.target_throttle = 0.15  # Target velocity in m/s
        self.waypoint_divider = 12 # Number of waypoints to divide the path into
        
        # Fixed Parameters
        self.waypoint_file_name = "/config/waypoints.csv"
        self.orientation_file_name = "/config/orientations.csv"

        package_share_dir = get_package_share_directory("pure_pursuit")
        absolute_file_path = package_share_dir + self.waypoint_file_name

        # Loading waypoints
        self.waypoints = []
        try:
            with open(absolute_file_path, 'r') as file:
                
                for row in csv.reader(file):
                    # self.get_logger().info(f"Row: {row}")
                    # data.pose.pose.position.x,
                    # data.pose.pose.position.y,
                    # euler[2], # Yaw (rotation about z-axis)
                    # speed
                    self.waypoints.append([float(row[0]), float(row[1])])

            self.waypoints = np.array(self.waypoints)

            # # Process waypoints
            self.original_waypoints = self.waypoints.copy()
            self.reduced_waypoints = self.reduce_waypoints(self.waypoints)

            self.spline_waypoints = self.spline_interpolate(self.reduced_waypoints)

            self.waypoints = self.spline_waypoints

        except FileNotFoundError:
            self.get_logger().error(f"File not found: {absolute_file_path}")

        qos_profile = QoSProfile(depth=10)

        # Topics
        ips_topic = "/autodrive/f1tenth_1/ips"
        imu_topic = "/autodrive/f1tenth_1/imu"

        scan_topic = "/autodrive/f1tenth_1/lidar"

        visualization_topic = "/visualization/waypoints"


        steering_pub_topic = '/autodrive/f1tenth_1/steering_command'
        throttle_pub_topic = '/autodrive/f1tenth_1/throttle_command'

        self.throttle_pub = self.create_publisher(Float32, throttle_pub_topic, qos_profile)
        self.steering_pub = self.create_publisher(Float32, steering_pub_topic, qos_profile)

        # Visualization
        self.waypoint_marker_publisher = self.create_publisher(MarkerArray, visualization_topic, qos_profile)

        # Publish the waypoints as markers
        self.visualization_timer = self.create_timer(5.0, self.publish_waypoint_markers)

        # Subscribers
        self.ips_subscriber = self.create_subscription(Point, ips_topic, self.ips_callback, qos_profile)
        self.imu_subscriber = self.create_subscription(Imu, imu_topic, self.imu_callback, qos_profile)

    def reduce_waypoints(self, waypoints):

        if len(waypoints) < 2:
            return waypoints
        
        reduced_waypoints = [waypoints[0]]

        for i in range(1, len(waypoints)):
            distance = np.linalg.norm(waypoints[i] - waypoints[-1])
            if distance > self.lookahead_distance:
                reduced_waypoints.append(waypoints[i])
        return np.array(waypoints)
    
    def spline_interpolate(self, waypoints):
        """
        Interpolate the waypoints using cubic splines
        :param waypoints: numpy array of waypoints
        :return: numpy array of interpolated waypoints
        """

        x = waypoints[:, 0]
        y = waypoints[:, 1]

        t = np.cumsum(np.sqrt(np.sum(np.diff(waypoints, axis=0)**2, axis=1)))
        t = np.insert(t, 0, 0)  

        epsilon = 1e-6
        for i in range(1, len(t)):
            if t[i] <= t[i-1]:
                t[i] = t[i-1] + epsilon

        # Cubic spline interpolation
        cs_x = CubicSpline(t, x)
        cs_y = CubicSpline(t, y)

        # Generate new waypoints
        t_new = np.linspace(0, t[-1], num=len(waypoints)//self.waypoint_divider)  # 100 points for smoothness
        x_new = cs_x(t_new)
        y_new = cs_y(t_new)
        waypoints = np.column_stack((x_new, y_new))

        return waypoints

    def publish_waypoint_markers(self):
        marker_array = MarkerArray()

        if self.spline_waypoints is not None:
            for waypoint_index, waypoint in enumerate(self.waypoints):
                marker = Marker()
                marker.header = Header()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = waypoint_index
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose = Pose()
                marker.pose.position.x = waypoint[0]
                marker.pose.position.y = waypoint[1]
                marker.pose.position.z = 0.0
                marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

                marker.scale = Vector3(x=0.1, y=0.1, z=0.1)

                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

                marker.lifetime.sec = 0

                marker_array.markers.append(marker)
            
            self.waypoint_marker_publisher.publish(marker_array)


    def ips_callback(self, data):
        self.pos_x = data.x
        self.pos_y = data.y

    def imu_callback(self, data):

        qx = data.orientation.x
        qy = data.orientation.y
        qz = data.orientation.z
        qw = data.orientation.w

        euler = quat2euler([qw, qx, qy, qz])
        self.yaw = euler[2]

        target_waypoint_ego = self.find_lookahead_waypoint(self.pos_x, self.pos_y, self.yaw)

        x_ego = target_waypoint_ego[0]
        y_ego = target_waypoint_ego[1]

        curvature = 2 * y_ego * self.wheelbase / (x_ego**2 + y_ego**2)
        
        steering_angle = self.kp * curvature

        self.publish_drive_msg(steering_angle)
    
    def find_lookahead_waypoint(self, x, y, yaw):

        waypoints_ego = []
        waypoints_global = []
        distances = []

        for i, waypoint in enumerate(self.waypoints):        

            dx = waypoint[0] - x
            dy = waypoint[1] - y
            distance = np.sqrt(dx**2 + dy**2)

            x_ego = dx * np.cos(-yaw) - dy * np.sin(-yaw)
            y_ego = dx * np.sin(-yaw) + dy * np.cos(-yaw)

            # in front of the ego car
            if x_ego > 0:
                waypoints_ego.append([x_ego, y_ego])
                waypoints_global.append(waypoint)
            
                distances.append(distance)      
            else:
                waypoints_ego.append([np.inf, np.inf])
                waypoints_global.append([np.inf, np.inf])
                distances.append(np.inf)

        if len(waypoints_ego) == 0:
            self.get_logger().warn("No forward waypoints found!", throttle_duration_sec=1.0)
            # Fallback: use the closest waypoint

            waypoints_ego = np.array([[x, y]])
            idx = 0

            return waypoints_ego[idx]
        
        waypoints_ego = np.array(waypoints_ego)
        waypoints_global = np.array(waypoints_global)
        distances = np.array(distances)

        # Find the waypoint closest to the lookahead distance

        distances[distances < self.lookahead_distance] = np.inf
        
        idx = np.argmin(np.abs(distances))

        if idx is None or distances[idx] == np.inf:
            # Fallback: use the closest waypoint -- CAN BE CHANGES TO GAP FOLLOW AND REACTIVE NODE TOO
            self.get_logger().info(f"No suitable waypoint found, defaulting to closest waypoint : {idx}", throttle_duration_sec=1.0)

            waypoints_ego = np.array([[x, y]])
            idx = 0

        return waypoints_ego[idx]

    def publish_drive_msg(self, steering_angle):
        # self.get_logger().info(f"Steering Angle: {steering_angle}")

        steering_angle = np.clip(steering_angle, -1, 1)

        steering_msg = Float32()
        steering_msg.data = steering_angle


        throttle_multiplier = 1.0 - 0.5 * np.abs(steering_angle)
        throttle = self.target_throttle * throttle_multiplier

        throttle_msg = Float32()
        throttle_msg.data = throttle


        self.steering_pub.publish(steering_msg)
        self.throttle_pub.publish(throttle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()