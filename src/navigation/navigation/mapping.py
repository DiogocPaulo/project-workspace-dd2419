#!/usr/bin/env python

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from nav_msgs.msg import OccupancyGrid

class Mapping(Node):

    def __init__(self):
        super().__init__("mapping")

        self.map_publisher = self.create_publisher(OccupancyGrid, "map", 10)

        self.create_timer(0.5, self.update_map)

        # Parameters
        self.resolution = 0.05  # 5 cm per cell
        self.width = 200        # 200 cells in width
        self.height = 200       # 200 cells in height
        self.origin_x = -5.0     # Map origin x
        self.origin_y = -5.0     # Map origin y

        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.define_workspace()

    def world_to_grid(self, x, y):
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def define_workspace(self):
        x_min, y_min = self.world_to_grid(-2.2, -1.3)
        x_max, y_max = self.world_to_grid(2.2, 1.3)

        for x in range(x_min, x_max + 1):
            self.grid[x, y_min] = 100
            self.grid[x, y_max] = 100

        for y in range(y_min, y_max + 1):
            self.grid[x_min, y] = 100
            self.grid[x_max, y] = 100

    def update_map(self):

        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"

        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        map_msg.info.origin.position.x = self.origin_x
        map_msg.info.origin.position.y = self.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatting grid into a row-major list
        map_msg.data = self.grid.flatten().tolist()

        self.map_publisher.publish(map_msg)
        self.get_logger().info("Published occupancy map grid")

def main():
    rclpy.init()
    node = Mapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
