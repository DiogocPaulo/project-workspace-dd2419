#!/usr/bin/env python3

# ROS 2 core library for node creation and spinning
import rclpy
# Base class for creating ROS 2 nodes
from rclpy.node import Node
# Utility for handling ROS 2 time and timestamps
from rclpy.time import Time

# ROS 2 message types for odometry data
from nav_msgs.msg import Odometry
# ROS 2 message types for laser scans and point clouds
from sensor_msgs.msg import LaserScan, PointCloud2
# Helper module for creating and manipulating PointCloud2 messages
import sensor_msgs_py.point_cloud2 as pc2

# TF2 library for handling transforms between coordinate frames
import tf2_ros
# Utility for transforming geometry messages (e.g., points) using TF2
from tf2_geometry_msgs import do_transform_point
# Functions for converting between quaternions and Euler angles
from tf_transformations import euler_from_quaternion

# NumPy for efficient numerical computations and vectorized operations
import numpy as np
# Deque for a bounded, efficient buffer to store aggregated points
from collections import deque
# Standard message type for headers with stamp and frame_id
from std_msgs.msg import Header

class LidarAggregator(Node):
    def __init__(self):
        super().__init__('lidar_aggregator')
        self.get_logger().info('LidarAggregator node initialized')

        # Parameters
        self.declare_parameter('aggregation_rate', 5.0)  # Hz
        self.declare_parameter('max_points', 10000)     # Limit aggregated points
        self.aggregation_rate = self.get_parameter('aggregation_rate').value
        self.max_points = self.get_parameter('max_points').value

        # State
        self.aggregated_points = deque(maxlen=self.max_points)  # Bounded buffer
        self.current_pose = np.array([0.0, 0.0, 0.0])          # [x, y, yaw]
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.last_scan_header = None                            # Store last LaserScan header

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.cloud_pub = self.create_publisher(PointCloud2, '/ag_scan', 10)

        # Timer for aggregation
        self.timer = self.create_timer(1.0 / self.aggregation_rate, self.publish_aggregated_cloud)

    def odom_callback(self, msg):
        """Update robot pose and velocities from odometry."""
        self.current_pose = self._pose_to_xy_yaw(msg.pose.pose)
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

    def _pose_to_xy_yaw(self, pose):
        """Convert Pose to [x, y, yaw] numpy array."""
        x = pose.position.x
        y = pose.position.y
        _, _, yaw = euler_from_quaternion([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ])
        return np.array([x, y, yaw])

    def scan_callback(self, msg):
        """Process incoming laser scans and transform points."""
        points = self._laser_scan_to_points(msg)
        transformed_points = self._transform_points(points, 'map', msg.header.frame_id, msg.header.stamp)
        if transformed_points is not None:
            self.aggregated_points.extend(transformed_points)
        self.last_scan_header = msg.header  # Store the latest header

    def _laser_scan_to_points(self, msg):
        """Convert LaserScan to list of [x, y, z] points."""
        ranges = np.array(msg.ranges)
        num_points = len(ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_points)  # Match length of ranges
        valid = (msg.range_min < ranges) & (ranges < msg.range_max)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        z = np.zeros_like(x)
        return np.stack((x, y, z), axis=-1).tolist()

    def _transform_points(self, points, target_frame, source_frame, timestamp):
        """Transform points to target frame using TF2."""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, timestamp, timeout=rclpy.duration.Duration(seconds=0.1))
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Transform failed: {ex}")
            return None

        # Vectorized transformation
        points_np = np.array(points)
        if points_np.size == 0:
            return []

        # Apply transform (simplified for 2D planar case)
        trans = transform.transform.translation
        rot = transform.transform.rotation
        yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)

        x = points_np[:, 0] * cos_yaw - points_np[:, 1] * sin_yaw + trans.x
        y = points_np[:, 0] * sin_yaw + points_np[:, 1] * cos_yaw + trans.y
        z = points_np[:, 2] + trans.z
        return np.stack((x, y, z), axis=-1).tolist()

    def publish_aggregated_cloud(self):
        """Publish the aggregated point cloud with the last scan's timestamp."""
        if not self.aggregated_points or self.last_scan_header is None:
            return

        # Use the header from the last laser scan, but update frame_id to 'map'
        header = Header()
        header.stamp = self.last_scan_header.stamp  # Reuse timestamp from last scan
        header.frame_id = 'map'                     # Set frame_id to 'map'
        cloud_msg = pc2.create_cloud_xyz32(header, list(self.aggregated_points))
        self.cloud_pub.publish(cloud_msg)
        self.get_logger().info(f"Published aggregated cloud with {len(self.aggregated_points)} points")

def main(args=None):
    rclpy.init(args=args)
    node = LidarAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()