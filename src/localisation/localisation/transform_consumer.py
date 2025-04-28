#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
import geometry_msgs.msg

class TransformConsumer(Node):
    def __init__(self):
        super().__init__('transform_consumer')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg): # Add msg argument
        self.lookup_transform(msg.header.stamp) # Pass stamp

    def lookup_transform(self, timestamp): # Add timestamp argument
        try:
            # Lookup the transform from 'odom' to 'map' at the timestamp of the scan
            transform = self.tf_buffer.lookup_transform('map', 'odom', timestamp)

            # Access the transform data
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            self.get_logger().info(f"Transform: Translation={translation}, Rotation={rotation}")

            # You can now use the 'transform' data for your calculations
            # ...

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not transform: {ex}")

def main(args=None):
    rclpy.init(args=args) # Add args argument
    node = TransformConsumer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node() # Add destroy_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()