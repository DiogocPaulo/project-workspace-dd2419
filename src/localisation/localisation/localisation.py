#!/usr/bin/env python

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, TransformStamped
from tf_transformations import quaternion_from_euler  # Correct import
import tf2_ros

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
        self.laser_scans = LaserScanStorage()
        self.distance_threshold = 0.3
        
        # TF2 broadcaster for map to odom transform
        self.map_odom_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscriptions
        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )
        
        self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

    def odom_callback(self, msg):
        """Callback for odometry messages."""
        self.pose = msg.pose.pose  # Direct assignment

        # Log pose for debugging
        self.get_logger().info(
            f"Odometry Pose: x={self.pose.position.x:.3f}, "
            f"y={self.pose.position.y:.3f}, z={self.pose.position.z:.3f}"
        )


    def scan_callback(self, msg):
        """Callback for laser scan messages."""
        # Create new LaserScan object
        new_scan = LaserScanData()
        current_pose = self.get_current_pose()
        
        # Check distance from last scan
        closest_scan = self.laser_scans.get_closest_scan(
            current_pose, 
            threshold=self.distance_threshold
        )
        if closest_scan is None:
            self.laser_scans.add_scan(new_scan)
            return
        
        # Calculate angles for range measurements
        angles = [
            msg.angle_min + i * msg.angle_increment 
            for i in range(len(msg.ranges))
        ]
        
        # Store scan data
        new_scan.store_scan(
            ranges=msg.ranges,
            angles=angles,
            pose=current_pose,
            timestamp=self.get_clock().now().to_msg()
        )
        
        # Add to storage and log
        self.laser_scans.add_scan(new_scan)
        self.get_logger().info(f"New Laser Scan added with {len(msg.ranges)} points")
        
        # Update transform
        self.broadcast_transform(0, 0, 0)
        self.get_logger().info(f"Updated map to odom transform to x=0, y=0, theta=0")
        #self.update_map_odom_transform(closest_scan, new_scan)

    def update_map_odom_transform(self, scan1, scan2):
        """Update and broadcast the map to odom transform using ICP."""
        try:
            rotation, translation, _ = icp(scan1.points, scan2.points)
        except Exception as e:
            self.get_logger().error(f"ICP failed: {str(e)}")
            return
        
        # Compute new pose estimate
        x = scan2.pose.position.x + translation[0]
        y = scan2.pose.position.y + translation[1]
        current_yaw = self.quaternion_to_yaw(scan2.pose.orientation)
        theta = math.atan2(
            math.sin(current_yaw + rotation),
            math.cos(current_yaw + rotation)
        )
        
        # Calculate pose difference
        dx = x - scan2.pose.position.x
        dy = y - scan2.pose.position.y
        dtheta = theta - current_yaw
        
        # Broadcast transform
        self.broadcast_transform(dx, dy, dtheta)

    def broadcast_transform(self, x: float, y: float, theta: float):
        """Broadcast the map to odom transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        
        # Set translation
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        # Set rotation using Euler angles
        q = quaternion_from_euler(0, 0, theta)  # Corrected function call
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Broadcast the transform
        self.map_odom_broadcaster.sendTransform([t])  # Send as a list


    def get_current_pose(self):
        """Return the current pose of the robot."""
        return self.pose

    @staticmethod
    def compute_pose_difference(pose1, pose2):
        """Compute linear and angular difference between two poses."""
        # Linear distance
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        linear_distance = math.sqrt(dx ** 2 + dy ** 2)
        
        # Angular difference
        yaw1 = Localisation.quaternion_to_yaw(pose1.orientation)
        yaw2 = Localisation.quaternion_to_yaw(pose2.orientation)
        angular_diff = math.atan2(
            math.sin(yaw2 - yaw1),
            math.cos(yaw2 - yaw1)
        )
        
        return linear_distance, abs(angular_diff)

    @staticmethod
    def quaternion_to_yaw(orientation):
        """Convert quaternion orientation to yaw angle."""
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        return tf2_ros.transformations.euler_from_quaternion(q)[2]

    def has_moved_enough(self, pose1, pose2):
        """Check if the robot has moved beyond specified thresholds."""
        linear_threshold = 0.1  # meters
        angular_threshold = 0.1  # radians
        
        linear_dist, angular_diff = self.compute_pose_difference(pose1, pose2)
        self.get_logger().info(
            f"Movement: distance={linear_dist:.3f}m, angle={angular_diff:.3f}rad"
        )
        return linear_dist > linear_threshold or angular_diff > angular_threshold


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