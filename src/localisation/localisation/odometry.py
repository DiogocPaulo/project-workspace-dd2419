#!/usr/bin/env python

import numpy as np
from math import cos, sin, pi

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry as OdometryType
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

class Odometry(Node):

    def __init__(self):
        super().__init__("odometry")

        self.create_timer(0.1, self.update_odometry)

        # Parameters
        self.ticks_per_revolution = 48 * 64
        self.wheel_radius = 0.04921
        self.base_width = 0.31

        # Internal variables
        self.then_time = self.get_clock().now()
        self.accumulated_ticks_left = 0
        self.accumulated_ticks_right = 0
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.create_subscription(Encoders, "/motor/encoders", self.encoder_callback, 10)
        self.odom_publisher = self.create_publisher(OdometryType, "odom", 10)
        self.path_publisher = self.create_publisher(Path, "odom_path", 10)
        self.odom_path = Path()
        self.odom_broadcaster = TransformBroadcaster(self)

    def update_odometry(self):
        now_time = self.get_clock().now()
        elapsed_time = now_time - self.then_time
        self.then_time = now_time
        elapsed_time = elapsed_time.nanoseconds / 1e9

        # Consume and then reset the accumulated ticks
        ticks_left = self.accumulated_ticks_left
        ticks_right = self.accumulated_ticks_right
        self.accumulated_ticks_left = 0
        self.accumulated_ticks_right = 0

        distance_left = (ticks_left / self.ticks_per_revolution) * (2 * pi * self.wheel_radius)
        distance_right = (ticks_right / self.ticks_per_revolution) * (2 * pi * self.wheel_radius)
        
        # Calculate odometry
        delta_distance = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / self.base_width

        if delta_distance != 0:
            delta_x = delta_distance * cos(self.theta + (delta_theta / 2))
            delta_y = delta_distance * sin(self.theta + (delta_theta / 2))
            self.x += delta_x
            self.y += delta_y
        if delta_theta != 0:
            self.theta += delta_theta

        # Calculate velocities
        self.linear_velocity = delta_distance / elapsed_time
        self.angular_velocity = delta_theta / elapsed_time

        # Publish odometry transform
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.theta / 2)
        quaternion.w = cos(self.theta / 2)

        transform_msg = TransformStamped()
        transform_msg.header.stamp = now_time.to_msg()
        transform_msg.header.frame_id = "odom"
        transform_msg.child_frame_id = "base_link"

        transform_msg.transform.translation.x = self.x
        transform_msg.transform.translation.y = self.y
        transform_msg.transform.translation.z = 0.0

        transform_msg.transform.rotation.x = quaternion.x
        transform_msg.transform.rotation.y = quaternion.y
        transform_msg.transform.rotation.z = quaternion.z
        transform_msg.transform.rotation.w = quaternion.w

        self.odom_broadcaster.sendTransform(transform_msg)

        # Publish odometry
        odometry_msg = OdometryType()
        odometry_msg.header.stamp = now_time.to_msg()
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base_link"

        odometry_msg.pose.pose.position.x = self.x
        odometry_msg.pose.pose.position.y = self.y
        odometry_msg.pose.pose.position.z = 0.0
        odometry_msg.pose.pose.orientation = quaternion
        
        odometry_msg.twist.twist.linear.x = self.linear_velocity
        odometry_msg.twist.twist.linear.y = 0.0
        odometry_msg.twist.twist.angular.z = self.angular_velocity

        self.odom_publisher.publish(odometry_msg)

        # Publish odometry path
        self.odom_path.header.stamp = now_time.to_msg()
        self.odom_path.header.frame_id = "odom"
        pose = PoseStamped()
        pose.header = self.odom_path.header

        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.01
        pose.pose.orientation = quaternion

        self.odom_path.poses.append(pose)
        self.path_publisher.publish(self.odom_path)

    def encoder_callback(self, msg):
        # Accumulate ticks based on delta of measured ticks
        delta_left = msg.encoder_left - self.last_encoder_left
        delta_right = msg.encoder_right - self.last_encoder_right
        self.accumulated_ticks_left += delta_left
        self.accumulated_ticks_right += delta_right
        self.last_encoder_left = msg.encoder_left
        self.last_encoder_right = msg.encoder_right

def main():
    rclpy.init()
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
