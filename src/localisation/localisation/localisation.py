#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
from tf2_ros import TransformBroadcaster
import numpy as np

class MapOdomPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_map_odom_publisher')
        self.map_odom_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transform)  # 10 Hz

        # Transform variables
        self.time_stamp = None
        self.translation = np.array([0.0, 0.0, 0.0])  # Current transform translation
        self.rotation = np.array(quaternion_from_euler(0, 0, 0))  # Current transform rotation [x, y, z, w]

        # Drift parameters (applied directly each step)
        self.drift_translation = np.array([0.005, 0.005, 0.0])  # Drift: 0.5 cm x, 0.5 cm y per step (i.e 5 cm/s)
        self.drift_yaw = 0.0  # Drift: 0 degrees per step (in radians)
        self.drift_quat = np.array(quaternion_from_euler(0, 0, self.drift_yaw))

        # Subscribe to odometry and ICP transform topics
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(TransformStamped, '/icp_transform', self.icp_callback, 10)

        self.get_logger().info("Dynamic map to odom publisher has started.")

    def odom_callback(self, msg):
        self.time_stamp = msg.header.stamp

    def icp_callback(self, msg):
        # Extract ICP transform (already in map frame)
        icp_translation = np.array([
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z
        ])
        icp_rotation = np.array([
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w
        ])

        # Directly update translation (since it's in the map frame)
        self.translation += icp_translation

        # Correct order for updating rotation: new = correction * current
        self.rotation = quaternion_multiply(icp_rotation, self.rotation)

        """self.get_logger().info(
            f"ICP transform combined: translation={self.translation.tolist()}, "
            f"rotation={euler_from_quaternion(self.rotation)[2]:.3f} rad (yaw)"
        )"""


    def publish_transform(self):
        if self.time_stamp is None:
            return

        # Apply drift directly to the current transform
        #self.translation += self.drift_translation
        #self.rotation = quaternion_multiply(self.drift_quat, self.rotation)

        # Create and publish the transform
        t = TransformStamped()
        t.header.stamp = self.time_stamp
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = self.translation[0]
        t.transform.translation.y = self.translation[1]
        t.transform.translation.z = self.translation[2]
        t.transform.rotation.x = self.rotation[0]
        t.transform.rotation.y = self.rotation[1]
        t.transform.rotation.z = self.rotation[2]
        t.transform.rotation.w = self.rotation[3]
        self.map_odom_broadcaster.sendTransform(t)

        # Log the current transform with drift
        yaw = euler_from_quaternion(self.rotation)[2]
        """self.get_logger().info(
            f"Published transform with drift: "
            f"translation={self.translation.tolist()}, yaw={yaw:.3f} rad"
        )"""

def main(args=None):
    rclpy.init()
    node = MapOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()