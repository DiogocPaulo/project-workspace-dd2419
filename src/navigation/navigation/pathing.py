#/usr/bin/env python

# Creates a path from starting point to end point on map frame
<<<<<<< HEAD
import random
=======
>>>>>>> 90022044b423cea3560868e28cf4fb0190b427ed

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

<<<<<<< HEAD
from example_interfaces.srv import Trigger

=======
>>>>>>> 90022044b423cea3560868e28cf4fb0190b427ed
DEBUG = True

class Pathing(Node):
    
    def __init__(self):
        super().__init__("pathing")

<<<<<<< HEAD
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

=======
>>>>>>> 90022044b423cea3560868e28cf4fb0190b427ed
        self.path_publisher = self.create_publisher(
                Path,
                "custom_path",
                10)
<<<<<<< HEAD
        self.new_path_service = self.create_service(Trigger, "new_path", self.new_path_callback)
        self.timer = self.create_timer(2.0, self.publish_custom_path)

    def new_path_callback(self, request, response):
        self.previous_end_x = self.end_x
        self.previous_end_y = self.end_y
        self.end_x = random.uniform(2.2, -2.2)
        self.end_y = random.uniform(1.3, -1.3)
        self.start_x = self.previous_end_x
        self.start_y = self.previous_end_y
        if DEBUG: self.get_logger().info("Message - updated custom path")
        response.success = True
        response.message = "Updated custom path!"
        return response
=======
        self.timer = self.create_timer(1.0, self.publish_custom_path)
>>>>>>> 90022044b423cea3560868e28cf4fb0190b427ed

    def publish_custom_path(self):
        custom_path = Path()
        now = self.get_clock().now().to_msg()
        custom_path.header.stamp = now
        custom_path.header.frame_id = "map"

        # Intermidiate points
<<<<<<< HEAD
        path_resolution = 20
        if self.end_x == 0 and self.end_y == 0:
            self.get_logger().info(f"No end point selected")
            return
=======
        path_resolution = 10
        start_x = 0
        start_y = 0
        end_x = 3
        end_y = 0
>>>>>>> 90022044b423cea3560868e28cf4fb0190b427ed

        for i in range(path_resolution):
            pose = PoseStamped()
            pose.header = custom_path.header
            t = i/(path_resolution - 1)
<<<<<<< HEAD
            pose.pose.position.x = (1 - t) * self.start_x + t * self.end_x
            pose.pose.position.y = (1 - t) * self.start_y + t * self.end_y
            pose.pose.position.z = 0.0
=======
            pose.pose.position.x = (1 - t) * start_x + t * end_x
            pose.pose.position.y = (1 - t) * start_y + t * end_y
            pose.pose.position.z = 0.0
            # For simplicity, use a neutral orientation (no rotation)
>>>>>>> 90022044b423cea3560868e28cf4fb0190b427ed
            pose.pose.orientation.w = 1.0
            custom_path.poses.append(pose)

        self.path_publisher.publish(custom_path)
        if DEBUG: self.get_logger().info("Published custom path")

def main():
    rclpy.init()
    node = Pathing()
<<<<<<< HEAD
    #node.update_end_point()
=======
>>>>>>> 90022044b423cea3560868e28cf4fb0190b427ed
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
