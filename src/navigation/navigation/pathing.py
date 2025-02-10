#/usr/bin/env python

# Creates a path from starting point to end point on map frame

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

DEBUG = True

class Pathing(Node):
    
    def __init__(self):
        super().__init__("pathing")

        self.path_publisher = self.create_publisher(
                Path,
                "custom_path",
                10)
        self.timer = self.create_timer(1.0, self.publish_custom_path)

    def publish_custom_path(self):
        custom_path = Path()
        now = self.get_clock().now().to_msg()
        custom_path.header.stamp = now
        custom_path.header.frame_id = "map"

        # Intermidiate points
        path_resolution = 10
        start_x = 0
        start_y = 0
        end_x = 5
        end_y = 2

        for i in range(path_resolution):
            pose = PoseStamped()
            pose.header = custom_path.header
            t = i/(path_resolution - 1)
            pose.pose.position.x = (1 - t) * start_x + t * end_x
            pose.pose.position.y = (1 - t) * start_y + t * end_y
            pose.pose.position.z = 0.0
            # For simplicity, use a neutral orientation (no rotation)
            pose.pose.orientation.w = 1.0
            custom_path.poses.append(pose)

        self.path_publisher.publish(custom_path)
        if DEBUG: self.get_logger().info("Published custom path")

def main():
    rclpy.init()
    node = Pathing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
