#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from roboracer_interfaces.msg import CarControlGapFollow
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

import numpy as np

class GapFollowUIControlNode(Node):
    def __init__(self):
        super().__init__("gap_follow_ui_control_node")
        self.get_logger().info("Gap Follow UI Control Node Started")

        self.throttle = 0.2
        self.windowHalf = 5
        self.disparityExtender = 5
        self.maxActionableDist = 2.0


        qos_profile = QoSProfile(depth=10)

        # Subscriber Topics
        gap_follow_params_sub_topic = "gap_follow_params"
        lidar_sub_topic = '/autodrive/f1tenth_1/lidar'

        self.gap_follow_params_sub_ = self.create_subscription(CarControlGapFollow, gap_follow_params_sub_topic, self.gap_follow_params_callback, qos_profile)
        self.lidar_sub_ = self.create_subscription(LaserScan, lidar_sub_topic, self.lidar_callback, qos_profile)

        # Publisher topics
        steering_pub_topic = '/autodrive/f1tenth_1/steering_command'
        throttle_pub_topic = '/autodrive/f1tenth_1/throttle_command'

        self.throttle_pub = self.create_publisher(Float32, throttle_pub_topic, qos_profile)
        self.steering_pub = self.create_publisher(Float32, steering_pub_topic, qos_profile)

    def gap_follow_params_callback(self, msg):
        if self.throttle != msg.throttle:
            self.throttle = msg.throttle 

        if self.windowHalf != msg.window_half_size:
            self.windowHalf = msg.window_half_size

        if self.disparityExtender != msg.disparity_extender:
            self.disparityExtender = msg.disparity_extender

        if self.maxActionableDist != msg.max_actionable_dist:
            self.maxActionableDist = msg.max_actionable_dist

    def mutate_ranges(self, ranges, center_index, value):
        """ Mutate ranges to be a certain value
        """
        ranges[center_index-self.windowHalf:center_index+self.windowHalf] = value

        return ranges
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.array(ranges)

        gaps = []
        disparities = []

        prev_range = proc_ranges[0]
        for i in range(self.windowHalf, len(proc_ranges)-self.windowHalf):            

            cur_mean = np.mean(proc_ranges[i-self.windowHalf:i+self.windowHalf+1])

            if cur_mean > self.maxActionableDist:
                gaps.append(i)

            cur_range = proc_ranges[i]
            if np.abs(cur_range - prev_range) > 2.0:
                disparities.append(i)
            else:
                prev_range = cur_range

        return proc_ranges, gaps, disparities
    
    def find_max_gap(self, gaps):
        """ Return the start index & end index of the max gap in free_space_ranges
        """

        if len(gaps) == 0:
            return (0, 1080)
        range_gap = []
        start = gaps[0]
        prev = gaps[0]

        for curr in gaps[1:]:
            if curr != prev + 1:
                range_gap.append((start, prev))
                start = curr
            prev = curr

        # Add the last range
        range_gap.append((start, prev))

        largest_range_gap = max(range_gap, key=lambda x: x[1] - x[0])

        return largest_range_gap
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        # max_gap_ranges = ranges[start_i:end_i]

        # furtherest_point_idx = np.argmax(max_gap_ranges) + start_i

        # self.get_logger().info(f'furtherest_point_idx: {furtherest_point_idx}', throttle_duration_sec=1.0)


        # Going to the center of the gap for now
        furtherest_point_idx = int((start_i + end_i) / 2)

        return furtherest_point_idx
    
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges

        proc_ranges, gaps, disparities = self.preprocess_lidar(ranges)

        for index in gaps:
            proc_ranges[index-self.windowHalf:index+self.windowHalf] = self.maxActionableDist

        for index in disparities:
            proc_ranges[index-self.disparityExtender:index+self.disparityExtender] = 0.0

        self.get_logger().info(f'gaps: {len(gaps)}', throttle_duration_sec=1.0)
        self.get_logger().info(f'disp: {len(disparities)}', throttle_duration_sec=1.0)

        gaps = list(set(gaps) - set(disparities))

        self.get_logger().info(f'gaps: {len(gaps)}', throttle_duration_sec=1.0)

        #Find max length gap 
        start_idx, end_idx = self.find_max_gap(gaps)

        ########################################################### THIS CAN BE IMPROVED ###########################################################
        #Find the best point in the gap
        best_point = self.find_best_point(start_idx, end_idx, proc_ranges)

        self.get_logger().info(f'best_point: {best_point}', throttle_duration_sec=1.0)

        ################################################################################################## 100 % sure this is working till this point ###################################################################################################

        angle = data.angle_min + best_point * data.angle_increment
        
        self.get_logger().info(f'angle: {angle}', throttle_duration_sec=1.0)

        #Publish Drive message
        self.publish_to_car(angle, self.throttle)

    def publish_to_car(self, steering_angle, throttle):
        """
        Publish the steering angle and throttle to the car.

        Args:
            steering_angle: Steering angle in radians
            throttle: Throttle value
        Returns:
            None
        """

        self.get_logger().info(f"Steering angle: {steering_angle}, Throttle: {throttle}", throttle_duration_sec=1.0)

        steering_angle = np.clip(steering_angle, -1, 1) # Limit steering angle to [-30, 30] degrees

        steering_msg = Float32()
        steering_msg.data = steering_angle 

        throttle_msg = Float32()
        throttle_msg.data = throttle 

        self.steering_pub.publish(steering_msg)
        self.throttle_pub.publish(throttle_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = GapFollowUIControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()