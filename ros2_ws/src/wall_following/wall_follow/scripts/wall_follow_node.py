#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidar_scan_topic_name = '/scan'
        drive_topic_name = '/drive'
        odometry_topic_name = '/ego_racecar/odom'

        self.proportional_gain = 0.01
        self.derivative_gain = 0.002
        self.integral_gain = 0.0

        self.integral_error = 0.0
        self.previous_error = 0.0
        self.current_error = 0.0

        self.scan_angle = np.pi / 3.6
        self.lookahead_distance = 2.0
        self.desired_wall_distance = 1.3

        self.start_time_in_seconds = -1.0
        self.current_time_in_seconds = 0.0
        self.previous_time_in_seconds = 0.0

        self.odometry_subscriber = self.create_subscription(
            Odometry, 
            odometry_topic_name, 
            self.odometry_callback, 
            100
        )

        self.lidar_scan_subscriber = self.create_subscription(
            LaserScan, 
            lidar_scan_topic_name, 
            self.lidar_callback, 
            100
        )

        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, 
            drive_topic_name, 
            10
        )

    def get_range(self, lidar_message, requested_angle):
        """
        Returns the LiDAR range measurement at a specific angle. Handles NaNs and infs if present.
        """

        angle_difference = abs(lidar_message.angle_min - requested_angle)
        angle_step_degrees = angle_difference / lidar_message.angle_increment
        lidar_index = len(lidar_message.ranges) - int(angle_step_degrees)

        measured_range = lidar_message.ranges[lidar_index]

        return measured_range

    def get_error(self, lidar_message, desired_distance_to_wall):
        """
        Calculates the error to the wall based on LiDAR scan.
        """

        range_b = self.get_range(lidar_message, -np.pi / 2)
        range_a = self.get_range(lidar_message, -np.pi / 2 + self.scan_angle)

        alpha_angle = np.arctan(
            (range_a * np.cos(self.scan_angle) - range_b) / 
            (range_a * np.sin(self.scan_angle))
        )

        distance_at_current_time = max(range_b * np.cos(alpha_angle), 0)
        distance_at_future_time = distance_at_current_time + self.lookahead_distance * np.sin(alpha_angle)
        error_value = desired_distance_to_wall - distance_at_future_time

        print("{:.2f} {:.2f} {:.2f}".format(distance_at_current_time, distance_at_future_time, error_value))

        self.previous_error = self.current_error
        self.current_error = error_value
        self.integral_error += self.current_error

        self.previous_time_in_seconds = self.current_time_in_seconds
        time_from_header = lidar_message.header.stamp.sec + (lidar_message.header.stamp.nanosec * 1e-9)
        self.current_time_in_seconds = time_from_header

        if self.start_time_in_seconds < 0.0:
            self.start_time_in_seconds = self.current_time_in_seconds

        return error_value

    def to_radians(self, degrees_value):
        """
        Convert degrees to radians.
        """
        return np.pi * degrees_value / 180.0

    def pid_control(self, error):
        """
        Based on the calculated error, publishes vehicle control commands.
        """

        if self.previous_time_in_seconds == 0.0:
            return

        time_delta = self.current_time_in_seconds - self.previous_time_in_seconds
        total_time_elapsed = self.current_time_in_seconds - self.start_time_in_seconds

        angle = (self.proportional_gain * error +
                 self.integral_gain * self.integral_error * total_time_elapsed +
                 self.derivative_gain * (error - self.previous_error) / time_delta)

        print("Angle: {:.2f}".format(angle))
        print("Error: {:.2f}".format(error))

        drive_message = AckermannDriveStamped()
        drive_message.drive.steering_angle = angle

        angle_abs = abs(angle)
        if angle_abs < self.to_radians(5):
            drive_message.drive.speed = 2.5
        elif angle_abs < self.to_radians(10):
            drive_message.drive.speed = 1.5
        elif angle_abs < self.to_radians(20):
            drive_message.drive.speed = 1.0
        else:
            drive_message.drive.speed = 0.5

        self.drive_publisher.publish(drive_message)

    def lidar_callback(self, lidar_message):
        """
        Callback function for LaserScan messages. Calculates the error and invokes PID control.
        """
        current_error = self.get_error(lidar_message, self.desired_wall_distance)
        self.pid_control(-current_error)

    def odometry_callback(self, odometry_message):
        """
        Callback function for Odometry messages. Updates the vehicle speed.
        """
        self.speed = odometry_message.twist.twist.linear.x

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



