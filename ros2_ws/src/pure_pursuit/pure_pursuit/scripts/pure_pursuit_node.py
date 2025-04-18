#!/usr/bin/env python3

import numpy as np
from scipy.spatial import distance, transform
import os

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
import tf2_ros


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')


        self.is_route_ascending = True  
        self.map_filename = 'Austin_centerline'
        self.centerline_filename = 'Austin_centerline'

        drive_topic_name = '/drive'
    
        odometry_topic_name = '/ego_racecar/odom'
    
        visualization_topic_name = '/visualization_marker_array'

        subscribed_message_type = Odometry
    
        self.pose_subscription = self.create_subscription(subscribed_message_type,
                                                        odometry_topic_name,
                                                        self.pose_callback,
                                                        1)

        self.ackermann_drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic_name, 1)
        self.visualization_publisher = self.create_publisher(MarkerArray, visualization_topic_name, 1)

        self.ackermann_drive_message = AckermannDriveStamped()
        self.marker_array_message = MarkerArray()

        map_absolute_path = os.path.abspath('/home/vboxuser/sim_ws/src/f1tenth_gym_ros/maps/f1tenth_racetracks/Austin')
        map_data = np.loadtxt(map_absolute_path + '/' + self.map_filename + '.csv', delimiter=',', skiprows=0)  
        self.racing_waypoints = map_data[:, 0:2]  
    
        centerline_data = np.loadtxt(map_absolute_path + '/' + self.centerline_filename + '.csv', delimiter=',', skiprows=0)  
        self.centerline_points = centerline_data[:, 0:2]
    
        self.total_waypoints = self.racing_waypoints.shape[0]
    
        self.reference_speed = map_data[:, -1] * 2.5

        self.initialize_visualization_markers()

        self.vehicle_lookahead_distance = 2.5
        self.steering_gain_coefficient = 0.3

        self.current_position_x = None
        self.current_position_y = None
        self.current_position_array = None
        self.vehicle_rotation_matrix = None
        self.all_waypoint_distances = None
        self.closest_waypoint_index = None
        self.closest_waypoint_coordinates = None


    def pose_callback(self, pose_message):
        self.current_position_x = pose_message.pose.pose.position.x
        self.current_position_y = pose_message.pose.pose.position.y
        self.current_position_array = np.array([self.current_position_x, self.current_position_y]).reshape((1, 2))

        orientation_quaternion = pose_message.pose.pose.orientation
        quaternion_array = [orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w]
        rotation_object = transform.Rotation.from_quat(quaternion_array)
        self.vehicle_rotation_matrix = rotation_object.as_matrix()

        self.all_waypoint_distances = distance.cdist(self.current_position_array, self.racing_waypoints, 'euclidean').reshape((self.total_waypoints))
        self.closest_waypoint_index = np.argmin(self.all_waypoint_distances)
        self.closest_waypoint_coordinates = self.racing_waypoints[self.closest_waypoint_index]

        target_waypoint_coordinates = self.get_closest_point_beyond_lookahead_dist(self.vehicle_lookahead_distance)
        translated_waypoint = self.translate_target_point_to_vehicle_frame(target_waypoint_coordinates)
    
        lateral_offset_y = translated_waypoint[1]
        steering_angle_command = self.steering_gain_coefficient * (2 * lateral_offset_y / self.vehicle_lookahead_distance**2)

        steering_angle_command = np.clip(steering_angle_command, -0.35, 0.35)
        self.ackermann_drive_message.drive.steering_angle = steering_angle_command

        speed_multiplier = 1.0
        self.ackermann_drive_message.drive.speed = speed_multiplier * self.reference_speed[self.closest_waypoint_index]

        self.ackermann_drive_publisher.publish(self.ackermann_drive_message)
        print("steering = {}, speed = {}".format(round(self.ackermann_drive_message.drive.steering_angle, 2),
                                                round(self.ackermann_drive_message.drive.speed, 2)))

        self.target_marker.points = [Point(x=target_waypoint_coordinates[0], y=target_waypoint_coordinates[1], z=0.0)]
        self.closest_marker.points = [Point(x=self.closest_waypoint_coordinates[0], y=self.closest_waypoint_coordinates[1], z=0.0)]

        self.marker_array_message.markers = [self.waypoints_marker, self.target_marker, self.closest_marker]
        self.visualization_publisher.publish(self.marker_array_message)


    def get_closest_point_beyond_lookahead_dist(self, lookahead_distance_threshold):
        waypoint_index_candidate = self.closest_waypoint_index
        current_distance = self.all_waypoint_distances[waypoint_index_candidate]

        while current_distance < lookahead_distance_threshold:
            if self.is_route_ascending:
                waypoint_index_candidate += 1
                if waypoint_index_candidate >= len(self.racing_waypoints):
                    waypoint_index_candidate = 0
                current_distance = self.all_waypoint_distances[waypoint_index_candidate]
            else:
                waypoint_index_candidate -= 1
                if waypoint_index_candidate < 0:
                    waypoint_index_candidate = len(self.racing_waypoints) - 1
                current_distance = self.all_waypoint_distances[waypoint_index_candidate]

        chosen_waypoint = self.racing_waypoints[waypoint_index_candidate]
        return chosen_waypoint


    def translate_target_point_to_vehicle_frame(self, target_point_coordinates):
        homogeneous_transform = np.zeros((4, 4))
        homogeneous_transform[0:3, 0:3] = np.linalg.inv(self.vehicle_rotation_matrix)
        homogeneous_transform[0, 3] = self.current_position_x
        homogeneous_transform[1, 3] = self.current_position_y
        homogeneous_transform[3, 3] = 1.0

        position_vector = target_point_coordinates - self.current_position_array
        converted_target_point = (homogeneous_transform @ np.array((position_vector[0, 0], position_vector[0, 1], 0, 0))).reshape((4))
    
        return converted_target_point
    

    def initialize_visualization_markers(self):
        self.waypoints_marker = Marker()
        self.waypoints_marker.header.frame_id = 'map'
        self.waypoints_marker.type = Marker.POINTS
        self.waypoints_marker.color.g = 0.75
        self.waypoints_marker.color.a = 1.0
        self.waypoints_marker.scale.x = 0.05
        self.waypoints_marker.scale.y = 0.05
        self.waypoints_marker.id = 0
        self.waypoints_marker.points = [Point(x=wpt[0], y=wpt[1], z=0.0) for wpt in self.racing_waypoints]

        self.target_marker = Marker()
        self.target_marker.header.frame_id = 'map'
        self.target_marker.type = Marker.POINTS
        self.target_marker.color.r = 0.75
        self.target_marker.color.a = 1.0
        self.target_marker.scale.x = 0.2
        self.target_marker.scale.y = 0.2
        self.target_marker.id = 1

        self.closest_marker = Marker()
        self.closest_marker.header.frame_id = 'map'
        self.closest_marker.type = Marker.POINTS
        self.closest_marker.color.b = 0.75
        self.closest_marker.color.a = 1.0
        self.closest_marker.scale.x = 0.2
        self.closest_marker.scale.y = 0.2
        self.closest_marker.id = 2


def main(args=None):
	rclpy.init(args=args)
	print("PurePursuit Initialized")
	pure_pursuit_node = PurePursuit()
	rclpy.spin(pure_pursuit_node)

	pure_pursuit_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()