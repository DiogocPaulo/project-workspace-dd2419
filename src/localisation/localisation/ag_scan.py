#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from std_msgs.msg import Header
from localisation.icp import icp

class LidarAggregator(Node):
    def __init__(self):
        super().__init__('lidar_aggregator')
        self.get_logger().info('LidarAggregator node initialized')

        # Parameters
        self.declare_parameter('max_scans', 5)           # Number of scans to store
        self.declare_parameter('max_points', 10000)     # Limit aggregated points
        self.max_scans = self.get_parameter('max_scans').value
        self.max_points = self.get_parameter('max_points').value

        # State
        self.scan_buffer = np.zeros((self.max_scans, self.max_points, 3))  # Store last 5 scans with x, y, z
        self.current_scan_index = 0  # Index to track where the next scan will be inserted
        self.current_pose = np.array([0.0, 0.0, 0.0])  # [x, y, yaw]
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.last_scan_header = None  # Store last LaserScan header

        # Transform variables
        self.transform_x = 0.0
        self.transform_y = 0.0
        self.transform_theta = 0.0

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.cloud_pub = self.create_publisher(PointCloud2, '/ag_scan', 10)
        self.create_timer(0.1, self.broadcast_transform)  # Repeat every 0.1s

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
            localised_points = self._localise_points(transformed_points)
            if localised_points is not None:
                self._update_scan_buffer(transformed_points)
                self.publish_aggregated_cloud()  # Publish aggregated cloud after each scan update
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

    def _update_scan_buffer(self, points):
        """Store the incoming points in the circular buffer."""
        # Update the buffer by storing the new scan at the current index
        self.scan_buffer[self.current_scan_index] = points
        self.current_scan_index = (self.current_scan_index + 1) % self.max_scans  # Increment index with wrapping

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
    
    def _localise_points(self, new_points):
        """Localise the transformed points using ICP."""
        # Compute the localised points by applying ICP to the transformed points and using the aggregated cloud as reference
        # Begin by ensuring that the current scan is usable for ICP
        if abs(self.angular_vel) > 0.1: # Ignore scans when turning
            return

        aggregated_points = np.vstack(self.scan_buffer)
        try:
            rotation_matrix, translation_vector, localised_points = icp(aggregated_points, new_points)
        except Exception as e:
            self.get_logger().error(f"ICP failed: {str(e)}")
            return
        
        # Update map to odom transform
        """
        rotation = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        self.transform_x = translation_vector[0]
        self.transform_y = translation_vector[1]
        self.transform_theta = rotation
        self.get_logger().info(f"Updated map → odom (x={self.transform_x}, y={self.transform_y}, theta={self.transform_theta})")
        """
        
        return localised_points

    def publish_aggregated_cloud(self):
        """Publish the aggregated point cloud with the last scan's timestamp."""
        # Aggregate all points from the circular buffer
        aggregated_points = np.vstack(self.scan_buffer)

        if aggregated_points.size == 0 or self.last_scan_header is None:
            return

        # Use the header from the last laser scan, but update frame_id to 'map'
        header = Header()
        header.stamp = self.last_scan_header.stamp  # Reuse timestamp from last scan
        header.frame_id = 'map'                     # Set frame_id to 'map'
        cloud_msg = pc2.create_cloud_xyz32(header, aggregated_points.tolist())
        self.cloud_pub.publish(cloud_msg)
        self.get_logger().info(f"Published aggregated cloud with {len(aggregated_points)} points")

    def broadcast_transform(self):
        """Broadcast the map to odom transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = self.transform_x
        t.transform.translation.y = self.transform_y
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.transform_theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.map_odom_broadcaster.sendTransform(t)

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
