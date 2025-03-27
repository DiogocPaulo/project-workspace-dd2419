#!/usr/bin/env python

import numpy as np
from math import cos, sin, pi

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
from sensor_msgs.msg import Imu

class DeadReckoning(Node):

    def __init__(self):
        super().__init__("dead_reckoning")

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Constants
        self.ticks_per_revolution = 48 * 64
        self.wheel_radius = 0.04921
        self.base_width = 0.31
        self.yaw_weight = 0.98
        self.angular_weight = 0.95

        # Internal variables
        self.then_time = self.get_clock().now()
        self.accumulated_ticks_left = 0
        self.accumulated_ticks_right = 0
        self.last_encoder_left = None
        self.last_encoder_right = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.imu_yaw = None
        self.imu_angular_velocity = None

        self.create_subscription(Encoders, "/motor/encoders", self.encoder_callback, qos_profile)
        self.create_subscription(Imu, "/imu/data_raw", self.imu_callback, qos_profile)
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.path_publisher = self.create_publisher(Path, "/odom_path", 10)
        self.odom_path = Path()
        self.odom_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_timer(0.1, self.update_odometry)

    def encoder_callback(self, msg):
        # Init step
        if self.last_encoder_left is None or self.last_encoder_right is None:
            self.last_encoder_left = msg.encoder_left
            self.last_encoder_right = msg.encoder_right
            return
        # Accumulate ticks based on delta of measured ticks
        delta_left = msg.encoder_left - self.last_encoder_left
        delta_right = msg.encoder_right - self.last_encoder_right
        self.accumulated_ticks_left += delta_left
        self.accumulated_ticks_right += delta_right
        self.last_encoder_left = msg.encoder_left
        self.last_encoder_right = msg.encoder_right

    def imu_callback(self, msg):
        try:
            imu_pose = PoseStamped()
            imu_pose.header.stamp = msg.header.stamp
            imu_pose.header.frame_id = msg.header.frame_id
            imu_pose.pose.position.x = 0
            imu_pose.pose.position.y = 0
            imu_pose.pose.position.z = 0
            imu_pose.pose.orientation.x = msg.orientation.x
            imu_pose.pose.orientation.y = msg.orientation.y
            imu_pose.pose.orientation.z = msg.orientation.z

            imu_to_base = self.tf_buffer.lookup_transform(
                "base_link",
                msg.header.frame_id,
                msg.header.stamp,
                rclpy.duration.Duration(seconds=1.0),
            )
            imu_pose_base = tf2_geometry_msgs.do_transform_pose_stamped(imu_pose, imu_to_base)

            q = imu_pose_base.pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            imu_yaw = np.arctan2(siny_cosp, cosy_cosp)
            imu_angular_velocity = msg.angular_velocity.z
            imu_linear_velocity = msg.linear_acceleration.x
            self.get_logger().info(f"Odometry vs IMU - Linear: {self.linear_velocity} vs {imu_linear_velocity}")
            self.get_logger().info(f"Odometry vs IMU - Angular: {self.angular_velocity} vs {imu_angular_velocity}")
            self.get_logger().info(f"Odometry vs IMU - Yaw: {self.theta} vs {imu_yaw}")
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform IMU reading to base_link: {ex}")
            return
        # try:
        #     imu_pose = PoseStamped()
        #     imu_pose.header.stamp = msg.header.stamp
        #     imu_pose.header.frame_id = msg.header.frame_id
        #     imu_pose.pose.position.x = 0
        #     imu_pose.pose.position.y = 0
        #     imu_pose.pose.position.z = 0
        #     imu_pose.orientation.x = msg.orientation.x
        #     imu_pose.orientation.y = msg.orientation.y
        #     imu_pose.orientation.z = msg.orientation.z
        #
        #     imu_to_base = self.tf_buffer.lookup_transform(
        #         "base_link",
        #         msg.header.frame_id,
        #         msg.header.stamp,
        #         rclpy.duration.Duration(seconds=1.0),
        #     )
        #     imu_pose_base = tf2_geometry_msgs.do_transform_pose_stamped(imu_pose, imu_to_base)
        #
        #     q = imu_pose_base.orientation
        #     siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        #     cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        #     self.imu_yaw = np.arctan2(siny_cosp, cosy_cosp)
        #     self.imu_angular_velocity = msg.angular_velocity.z
        # except TransformException as ex:
        #     self.get_logger().warn(f"Could not transform IMU reading to base_link: {ex}")
        #     return

    def update_odometry(self):
        # Wait for encoder init
        if self.last_encoder_left is None or self.last_encoder_right is None:
            return

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

        # Calculate theta, fusing IMU yaw into theta if avaliable
        if self.imu_yaw is not None:
            self.theta = (self.yaw_weight * self.imu_yaw) + ((1 - self.yaw_weight) * (self.theta + delta_theta))
        elif delta_theta != 0:
            self.theta += delta_theta

        # Calculate velocities, fusing IMU angular velocity into angular velocity if avalialbe
        if self.imu_angular_velocity is not None and elapsed_time > 0:
            self.angular_velocity = (self.angular_weight * self.imu_angular_velocity) + ((1 - self.angular_weight) * (delta_theta / elapsed_time))
            self.angular_velocity = delta_theta / elapsed_time
        elif elapsed_time > 0:
            self.linear_velocity = delta_distance / elapsed_time
            self.angular_velocity = delta_theta / elapsed_time
        else:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        # Broadcast odometry transform
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
        odometry_msg = Odometry()
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

def main():
    rclpy.init()
    node = DeadReckoning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
