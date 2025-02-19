#!/usr/bin/env python

# Creates a path from starting point to end point on map frame
import random
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from example_interfaces.srv import Trigger

class Pathing(Node):
    
    def __init__(self):
        super().__init__("pathing")


        self.start_x = 0
        self.start_y = 0
        self.end_x = 2
        self.end_y = -0.5

        self.amplitude = 1.0
        self.cycles = 1.0

        array = [[-2.0, 0.0], [2.0, 0.0], [2.0, -1.0]]

        self.path_publisher = self.create_publisher(
                Path,
                "custom_path",
                10)
        #self.new_path_service = self.create_service(Trigger, "new_path", self.new_path_callback)

        self.timer = self.create_timer(2.0, self.publish_straight_path)
        # self.timer = self.create_timer(2.0, self.publish_curved_path)

    # def new_path_callback(self, request, response):
    #     self.start_x = self.end_x
    #     self.start_y = self.end_y
    #     self.end_x = random.uniform(-2, 2)
    #     self.end_y = random.uniform(-1, 1)
    #     if DEBUG: self.get_logger().info("Message - updated custom path")
    #     response.success = True
    #     response.message = "Updated custom path!"
    #     return response

    def publish_curved_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        dx = self.end_x - self.start_x
        dy = self.end_y - self.start_y
        length = np.sqrt(dx**2 + dy**2)
        
        ux = dx / length
        uy = dy / length
        
        nx = -uy
        ny = ux
        
        path_resolution = 20

        for i in range(path_resolution + 1):
            t = i / path_resolution  # Parameter t goes from 0 to 1
            bx = self.start_x + t * dx
            by = self.start_y + t * dy
            
            offset = self.amplitude * np.sin(2 * np.pi * self.cycles * t)
            x = bx + offset * nx
            y = by + offset * ny
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published curved custom path")
        

    def publish_straight_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        # Intermidiate points
        path_resolution = 20
        if self.end_x == 0 and self.end_y == 0:
            self.get_logger().info(f"No end point selected")
            return

        for i in range(path_resolution):
            pose = PoseStamped()
            pose.header = path_msg.header
            t = i/(path_resolution - 1)
            pose.pose.position.x = (1 - t) * self.start_x + t * self.end_x
            pose.pose.position.y = (1 - t) * self.start_y + t * self.end_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        if DEBUG: self.get_logger().info("Published straight custom path")

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
