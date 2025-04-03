#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid

from project_interfaces.msg import Vertex, WorkspaceVertices

class MapWorkspace(Node):
    def __init__(self):
        super().__init__("map_workspace")

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(WorkspaceVertices, "/workspace", self.workspace_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, "/workspace_map", 10)

        # Constants
        self.resolution = 0.05

        # Variables
        self.workspace_vertices = []
        self.map = None

        self.create_timer(2, self.publish_map)

    def workspace_callback(self, msg: WorkspaceVertices):
        if self.workspace_vertices:
            return
        for vertex_msg in msg.vertices:
            x = vertex_msg.x
            y = vertex_msg.y
            self.workspace_vertices.append((x, y))
        # Initialise map based on workspace perimeter
        self.map = Map(self.resolution)
        self.map.initialise_grid(self.workspace_vertices)

    def publish_map(self):
        if self.map is None:
            return
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "odom"

        map_msg.info.resolution = self.map.resolution
        map_msg.info.width = self.map.grid_width
        map_msg.info.height = self.map.grid_height
        map_msg.info.origin.position.x = self.map.origin_x
        map_msg.info.origin.position.y = self.map.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatting grid into a row-major list
        map_msg.data = self.map.grid.flatten().tolist()

        self.map_publisher.publish(map_msg)
        self.get_logger().info("Published workspace occupancy map", once=True)

def main():
    rclpy.init()
    node = MapWorkspace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
