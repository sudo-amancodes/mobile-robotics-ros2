#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import array
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class ReactiveFollowGap(Node):
    """
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """

    def __init__(self):
        super().__init__('reactive_node')
    
        laser_scan_topic_name = '/scan'
        drive_command_topic_name = '/drive'


        self.lidar_sub = self.create_subscription(
            LaserScan,
            laser_scan_topic_name,
            self.lidar_callback,
            10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_command_topic_name,
            10
        )
    
        self.previous_lidar_ranges = np.zeros((1080,), dtype=np.float32)

    def preprocess_lidar(self, raw_lidar_ranges: array) -> array:
        """ Preprocess the LiDAR scan array. Steps might include:
            1. Setting each value to the mean over some window
            2. Rejecting/Clipping high values (e.g. > 3m)
        """
        maximum_accepted_value = 1.4
        current_processed_ranges = np.array(raw_lidar_ranges, dtype=np.float32)


        averaged_ranges = (current_processed_ranges + self.previous_lidar_ranges) / 2
        self.previous_lidar_ranges = current_processed_ranges


        np.clip(averaged_ranges, 0, maximum_accepted_value, out=averaged_ranges)
        return averaged_ranges
    
    def find_closest_point(self, current_ranges: array) -> int:
        """ Return the index of the closest point """
        closest_point_index = min(range(len(current_ranges)), key=lambda x: current_ranges[x])
        return closest_point_index
    
    def process_bubble(self, filtered_ranges: array, closest_point_index: int) -> array:
        """ Set all the points around the closest point index to zero (bubble elimination) """
        bubble_radius = 100
        bubble_start_index = max(0, closest_point_index - bubble_radius)
        bubble_end_index = min(len(filtered_ranges) - 1, closest_point_index + bubble_radius - 1)
        filtered_ranges[bubble_start_index:bubble_end_index + 1] = [0] * (bubble_end_index - bubble_start_index + 1)
        return filtered_ranges

    def find_max_gap(self, bubble_eliminated_ranges: array) -> (int, int, array):
        """ Return the start index, end index, and the array segment of the largest gap """
        zero_indices = np.where(bubble_eliminated_ranges == 0.0)[0]
        split_ranges = np.split(bubble_eliminated_ranges, zero_indices)
        segment_lengths = np.array([len(segment) for segment in split_ranges])
        longest_segment_index = np.argmax(segment_lengths)

        if longest_segment_index == 0:
            max_gap_start_index = 0
            max_gap_end_index = segment_lengths[0] - 1
        else:
            max_gap_start_index = np.sum(segment_lengths[:longest_segment_index])
            max_gap_end_index = max_gap_start_index + segment_lengths[longest_segment_index] - 1

        max_length_segment = split_ranges[longest_segment_index]
        return max_gap_start_index, max_gap_end_index, max_length_segment
    
    def find_best_point(self, max_gap_start_index: int, max_gap_end_index: int, max_gap_segment: array) -> int:
        """
        Given the largest gap segment and its start/end indices, return the index of the best point.
        Naive approach: choose the furthest point within the max gap segment.
        """
        best_index_candidates = np.where(max_gap_segment == np.max(max_gap_segment))[0]
        mid_index_candidate = round(len(best_index_candidates)/2)
        best_point_index = max_gap_start_index + best_index_candidates[mid_index_candidate]
        return best_point_index
    
    def get_speed(self, steering_angle: float) -> float:
        """ Estimate speed based on steering angle """
        steering_angle_in_degrees = np.degrees(abs(steering_angle))
    
    
        if steering_angle_in_degrees >= 0 and steering_angle_in_degrees < 10:
            return 3.0
        elif steering_angle_in_degrees >= 10 and steering_angle_in_degrees < 20:
            return 1.0
        else:
            return 0.5

    def lidar_callback(self, data: LaserScan):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message """
    
        raw_lidar_ranges = data.ranges
        filtered_lidar_ranges = self.preprocess_lidar(raw_lidar_ranges)
    
        closest_point_index = self.find_closest_point(filtered_lidar_ranges)

        bubble_eliminated_ranges = self.process_bubble(filtered_lidar_ranges, closest_point_index)

        max_gap_start_index, max_gap_end_index, max_gap_segment = self.find_max_gap(bubble_eliminated_ranges)

        best_point_index = self.find_best_point(max_gap_start_index, max_gap_end_index, max_gap_segment)

        target_steering_angle = (best_point_index * data.angle_increment) + data.angle_min

        drive_message = AckermannDriveStamped()
        drive_message.drive.steering_angle = target_steering_angle
        drive_message.drive.speed = self.get_speed(target_steering_angle)
        self.drive_pub.publish(drive_message)


def main(args=None):
	rclpy.init(args=args)
	print("WallFollow Initialized")
	reactive_node = ReactiveFollowGap()
	rclpy.spin(reactive_node)

	reactive_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()