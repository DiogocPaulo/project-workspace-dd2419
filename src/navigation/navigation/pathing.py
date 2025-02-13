#/usr/bin/env python

# Creates a path from starting point to end point on map frame
import random

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from example_interfaces.srv import Trigger

DEBUG = True

class Pathing(Node):
    
    def __init__(self):
        super().__init__("pathing")

        self.start_x = 0
        self.start_y = 0
        self.end_x = 1
        self.end_y = 0
        self.path_publisher = self.create_publisher(
                Path,
                "custom_path",
                10)
        #self.new_path_service = self.create_service(Trigger, "new_path", self.new_path_callback)
        self.timer = self.create_timer(2.0, self.publish_custom_path)

    # def new_path_callback(self, request, response):
    #     self.start_x = self.end_x
    #     self.start_y = self.end_y
    #     self.end_x = random.uniform(-2, 2)
    #     self.end_y = random.uniform(-1, 1)
    #     if DEBUG: self.get_logger().info("Message - updated custom path")
    #     response.success = True
    #     response.message = "Updated custom path!"
    #     return response

    def publish_custom_path(self):
        custom_path = Path()
        now = self.get_clock().now().to_msg()
        custom_path.header.stamp = now
        custom_path.header.frame_id = "map"

        # Intermidiate points
        path_resolution = 20
        if self.end_x == 0 and self.end_y == 0:
            self.get_logger().info(f"No end point selected")
            return

        for i in range(path_resolution):
            pose = PoseStamped()
            pose.header = custom_path.header
            t = i/(path_resolution - 1)
            pose.pose.position.x = (1 - t) * self.start_x + t * self.end_x
            pose.pose.position.y = (1 - t) * self.start_y + t * self.end_y
            pose.pose.position.z = 0.0
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
