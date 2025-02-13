#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Odometry(Node):
    
    def __init__(self):
        super().__init__("odometry")

        self.transform_broadcaster = TransformBroadcaster(self)

        self.path_publisher = self.create_publisher(
                Path,
                "odom_path",
                10)
        self.robot_path = Path()

        self.create_subscription(
                Encoders,
                '/motor/encoders',
                self.encoder_callback,
                10)

        # 2D pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def encoder_callback(self, msg):

        dt = 50 / 1000                      # Update interval rate (seconds)
        ticks_per_rev = 48 * 64             # Ticks per revolution
        wheel_radius = 0.04921              # Wheel radius (meters)
        base = 0.3                          # Distance between wheels (meters)

        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right
        self.get_logger().info(f"Time [{msg.header.stamp.sec}] \n\tLeft ({msg.delta_encoder_left}) \n\tRight ({msg.delta_encoder_right})")

        delta_left_wheel_angle = (delta_ticks_left/ticks_per_rev) * 2 * (math.pi) * wheel_radius
        delta_right_wheel_angle = (delta_ticks_right/ticks_per_rev) * 2 * (math.pi) * wheel_radius
        translation_speed = (delta_right_wheel_angle + delta_left_wheel_angle) / (2 * dt)
        rotation_speed = (delta_right_wheel_angle - delta_left_wheel_angle) / (base * dt)

        # Odometry
        delta_x = translation_speed * math.cos(self.yaw) * dt
        delta_y = translation_speed * math.sin(self.yaw) * dt
        delta_yaw = rotation_speed * dt
        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw
        
        stamp = msg.header.stamp

        self.broadcast_transform(stamp, self.x, self.y, self.yaw)
        self.publish_path(stamp, self.x, self.y, self.yaw)

    def broadcast_transform(self, stamp, x, y, yaw):
        """Takes a 2D pose and broadcasts it as a ROS transform.
        Broadcasts a 3D transform with z, roll, and pitch all zero. 
        The transform is stamped with the current time and is between the frames 'odom' -> 'base_link'.
        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        # The robot only exists in 2D, thus we set x and y translation
        # coordinates and set the z coordinate to 0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # For the same reason, the robot can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.transform_broadcaster.sendTransform(t)

    def publish_path(self, stamp, x, y, yaw):
        """Takes a 2D pose appends it to the path and publishes the whole path.
        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        self.robot_path.header.stamp = stamp
        self.robot_path.header.frame_id = "map"

        pose = PoseStamped()
        pose.header = self.robot_path.header

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.robot_path.poses.append(pose)
        self.path_publisher.publish(self.robot_path)

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
