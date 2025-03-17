#!/usr/bin/env python

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import numpy as np

# Import custom classes and functions
from localisation.laser_scan_storage import LaserScanStorage, LaserScanData
from localisation.icp import icp

class Localisation(Node):
    """ROS 2 node for robot localization using laser scans and odometry."""

    def __init__(self):
        """Initialize the Localisation node."""
        super().__init__("localisation")
        
        # Initialize class variables
        self.pose = Pose()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.distance_threshold = 0.1
        self.angular_threshold = 0.1  # radians
        self.laser_scans = LaserScanStorage()
        self.distance_threshold = 0.3
        
        # TF2 broadcaster for map to odom transform
        self.map_odom_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.transform_x = 0.0
        self.transform_y = 0.0
        self.transform_theta = 0.0
        self.create_timer(0.1, self.broadcast_transform)  # Repeat every 0.1s
        
        # Subscriptions
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

    def odom_callback(self, msg):
        """Callback for odometry messages."""
        self.pose = msg.pose.pose
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z
        """self.get_logger().info(
            f"Odometry Pose: x={self.pose.position.x:.3f}, "
            f"y={self.pose.position.y:.3f}, z={self.pose.position.z:.3f}"
        )"""

    def scan_callback(self, msg):
        """Callback for laser scan messages."""
        new_scan = LaserScanData()
        current_pose = self.get_current_pose()
        
        closest_scan = self.laser_scans.get_closest_scan(
            current_pose, max_distance=self.distance_threshold
        )
        
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
        added = new_scan.store_scan(ranges=msg.ranges, angles=angles, pose=current_pose, timestamp=self.get_clock().now().to_msg())
        if not added:
            self.get_logger().error("Failed to store LaserScan data")
            return


        if closest_scan is None or self.use_scan(closest_scan.pose, current_pose):
            self.laser_scans.add_scan(new_scan)
            self.get_logger().info(f"New Laser Scan added with {len(msg.ranges)} points")
            if closest_scan is not None:
                self.update_map_odom_transform(closest_scan, new_scan)

    def update_map_odom_transform(self, scan1, scan2):
        """Update and broadcast the map to odom transform using ICP."""
        try:
            rotation, translation, _ = icp(scan1.points, scan2.points)
        except Exception as e:
            self.get_logger().error(f"ICP failed: {str(e)}")
            return
        
        x = scan2.pose[0] + translation[0]
        y = scan2.pose[1] + translation[1]
        current_yaw = scan2.pose[2]
        theta = math.atan2(math.sin(current_yaw + rotation), math.cos(current_yaw + rotation))
        
        self.transform_x = x - scan2[0]
        self.transform_y = y - scan2[1]
        self.transform_theta = theta - current_yaw
        self.get_logger().info(f"Updated map → odom (x={self.transform_x}, y={self.transform_y}, theta={self.transform_theta})")

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

    def get_current_pose(self):
        """Return the current pose of the robot."""
        x = self.pose.position.x
        y = self.pose.position.y
        _, _, yaw = euler_from_quaternion(
            [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        )
        return np.array([x, y, yaw])

    @staticmethod
    def quaternion_to_yaw(orientation):
        """Convert quaternion to yaw angle."""
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(q)
        return yaw

    @staticmethod
    def compute_pose_difference(pose1, pose2):
        """Compute linear and angular difference between two poses."""
        dx = pose2[0]- pose1[0]
        dy = pose2[1] - pose1 [1]
        dtheta = pose2[2] - pose1[2]
        linear_distance = math.sqrt(dx ** 2 + dy ** 2)
        angular_diff = math.atan2(math.sin(dtheta), math.cos(dtheta))
        
        return linear_distance, angular_diff

    def use_scan(self, pose1, pose2):
        """Check if the robot has moved beyond specified thresholds."""
        linear_dist, angular_diff = self.compute_pose_difference(pose1, pose2)
        #self.get_logger().info(f"Movement: distance={linear_dist:.3f}m, angle={angular_diff:.3f}rad")
        return self.angular_velocity < self.angular_threshold and linear_dist < self.distance_threshold

def main():
    """Main function to run the Localisation node."""
    rclpy.init()
    node = Localisation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()