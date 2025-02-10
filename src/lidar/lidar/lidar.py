#!/usr/bin/env python3
<<<<<<< HEAD

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarRepublisher(Node):
    def __init__(self):
        super().__init__('lidar')
        
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            LaserScan,
            '/ag_scan',
            10
        )

    def scan_callback(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
=======
>>>>>>> 90022044b423cea3560868e28cf4fb0190b427ed

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import PointStamped

class LidarRepublisher(Node):
    def __init__(self):
        super().__init__('lidar')
        
        self.get_logger().info('LidarRepublisher node initialized.')

        # Initialize empty list to hold aggregated point cloud
        self.aggregated_points = []

        # Frequency variables
        self.frequency_of_adding = 5
        self.scan_counter = 0
        
        # Create a subscriber to /scan
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Create a publisher for the aggregated point cloud
        self.publisher = self.create_publisher(
            PointCloud2,
            '/ag_scan',
            10
        )
        
        # TF listener to transform coordinates
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def scan_callback(self, msg):
        self.scan_counter += 1

        # Perform callback only every Nth call
        if self.scan_counter % self.frequency_of_adding != 0:
            return

        # Convert LaserScan to list of points
        points = self.laser_scan_to_points(msg)

        # Transform each point to the map frame
        transformed_points = self.transform_points(points, "map", msg.header.frame_id, msg.header.stamp)

        if not transformed_points:
            return  # Skip if no transformed points

        # Add the transformed points to the aggregated point cloud
        self.aggregated_points += transformed_points

        # Create and publish the aggregated PointCloud2 message
        aggregated_pc_msg = pc2.create_cloud_xyz32(msg.header, self.aggregated_points)
        self.get_logger().info('Publishing aggrigated cloud')
        self.publisher.publish(aggregated_pc_msg)

    def laser_scan_to_points(self, msg):
        """Convert LaserScan message to a list of 3D points (x, y, z) in the lidar frame."""
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append((x, y, 0.0))
            angle += msg.angle_increment
        return points

    def transform_points(self, points, target_frame, source_frame, timestamp):
        """Transform each point to the target frame and return a new list of transformed points."""
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, timestamp)

            transformed_points = []
            for x, y, z in points:
                point_msg = PointStamped()
                point_msg.header.frame_id = source_frame
                point_msg.header.stamp = timestamp
                point_msg.point.x, point_msg.point.y, point_msg.point.z = x, y, z

                transformed_point = tf2_ros.Buffer().transform(point_msg, target_frame)
                transformed_points.append((transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))

            return transformed_points

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Transform error: {e}")
            return []

def main(args=None):
    rclpy.init(args=args)
    node = LidarRepublisher()
    
    node.get_logger().info('Starting to spin the node.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.get_logger().info('Shutting down the node.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
