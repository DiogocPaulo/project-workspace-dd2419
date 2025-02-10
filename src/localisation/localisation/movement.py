#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

class Movement(Node):

    def __init__(self):
        super().__init__("movement")

        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(self._tf_buffer, self, spin_thread=True)



    def get_current_pose():
        try:
            current_pose = PoseStamped()
            base_transform = TransformStamped()
            base_transform = self.transform_buffer.lookup_transform(
                            "map",
                            "base_link",
                            rclpy.time.Time(),
                            rclpy.duration.Duration(seconds=1.0))

            current_pose.header.stamp = base_transform.header.stamp
            current_pose.header.frame_id = "map"

            current_pose.pose.position.x = base_transform.transform.translation.x
            current_pose.pose.position.y = base_transform.transform.translation.y
            current_pose.pose.position.z = base_transform.transform.translation.z
            
            current_pose.pose.orientation.x = trans.transform.rotation.x
            current_pose.pose.orientation.y = trans.transform.rotation.y
            current_pose.pose.orientation.z = trans.transform.rotation.z
            current_pose.pose.orientation.w = trans.transform.rotation.w
            return current_pose

        except TransformException as ex:
            self.get_logger().info(f"Could not find transform base_link to map: {ex}")
            return

def main():
    rclpy.init()
    node = Movement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
