#!/usr/bin/env python3

# ROS Node
import rclpy
from rclpy.node import Node

# ROS Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# TF2
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# Math
import numpy as np

class LidarRepublisher(Node):
    def __init__(self):
        super().__init__('lidar')
        
        self.get_logger().info('LidarRepublisher node initialized.')

        # Initialize empty list to hold aggregated point cloud
        self.aggregated_points = []

        # Intialize pose and velocities (Odometry)
        self.pose = Pose()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Frequency variables
        self.frequency_of_adding = 5
        self.scan_counter = 0
        
        # Subscriptions
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publishers
        self.publisher = self.create_publisher(PointCloud2, '/ag_scan', 10)
        
        # TF listener to transform coordinates
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg):
        """Callback for odometry messages. Store the pose and velocities."""
        self.pose = msg.pose.pose
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

    def get_current_pose(self):
        """Converts the PoseStamped to (x, y, yaw)."""
        x = self.pose.position.x
        y = self.pose.position.y
        _, _, yaw = euler_from_quaternion(
            [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        )
        return np.array([x, y, yaw])

    def scan_callback(self, msg):
        """Callback for laser scan messages. Aggregates the transformed points."""
        self.scan_counter += 1

        # Perform callback only every Nth call
        if self.scan_counter % self.frequency_of_adding != 0:
            return

        # Convert LaserScan to list of points
        points = self.laser_scan_to_points(msg)

        # Transform each point to the map frame
        transformed_points = self.transform_points(points, 'odom', msg.header.frame_id, msg.header.stamp)

        if not transformed_points:
            return  # Skip if no transformed points

        # Add the transformed points to the aggregated point cloud
        self.aggregated_points += transformed_points

        # Create and publish the aggregated PointCloud2 message
        aggregated_pc_msg = pc2.create_cloud_xyz32(msg.header, self.aggregated_points)
        aggregated_pc_msg.header.frame_id = 'odom'
        self.get_logger().info('Publishing aggregated cloud')
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
        tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame = target_frame,
            source_frame = source_frame,
            time = timestamp
        )

        rclpy.spin_until_future_complete(self,tf_future, timeout_sec=1)

        try:
            t = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                timestamp)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {source_frame} to {target_frame}: {ex}'
            )
        

        transformed_points = []
        for x, y, z in points:
            point_msg = PointStamped()
            point_msg.header.frame_id = source_frame
            point_msg.header.stamp = timestamp
            point_msg.point.x, point_msg.point.y, point_msg.point.z = x, y, z

            transformed_point = do_transform_point(point_msg,t)
            transformed_points.append((transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))

        return transformed_points

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
