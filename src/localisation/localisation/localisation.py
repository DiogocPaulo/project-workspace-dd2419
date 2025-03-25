#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class MapOdomPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_map_odom_publisher')
        self.map_odom_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transform)

        # Initially, the transform is static (zero)
        self.translation = [0.0, 0.0, 0.0]
        self.rotation = quaternion_from_euler(0, 0, 0)

    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
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

    # Function to update the transform (for future use)
    def update_transform(self, translation, rotation):
        self.translation = translation
        self.rotation = rotation

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