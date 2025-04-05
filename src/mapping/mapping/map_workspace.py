#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid

from project_interfaces.msg import Vertex, WorkspaceVertices

from mapping.map import Map

class MapWorkspace(Node):
    def __init__(self):
        super().__init__("map_workspace")

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(WorkspaceVertices, "/workspace", self.workspace_callback, 10)
        self.workspace_map_publisher = self.create_publisher(OccupancyGrid, "/workspace_map", 10)

        # Variables
        self.workspace_vertices = []
        self.workspace_map = None

        self.create_timer(0.5, self.publish_workspace_map)

    def workspace_callback(self, msg: WorkspaceVertices):
        if self.workspace_vertices:
            return
        for vertex_msg in msg.vertices:
            x = vertex_msg.x
            y = vertex_msg.y
            self.workspace_vertices.append((x, y))
        # Initialise map based on workspace perimeter
        self.workspace_map = Map(msg.grid_resolution)
        self.workspace_map.initialise_grid(self.workspace_vertices)

    def publish_workspace_map(self):
        if self.workspace_map is None:
            return
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "odom"

        map_msg.info.resolution = self.workspace_map.resolution
        map_msg.info.width = self.workspace_map.grid_width
        map_msg.info.height = self.workspace_map.grid_height
        map_msg.info.origin.position.x = self.workspace_map.origin_x
        map_msg.info.origin.position.y = self.workspace_map.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatting grid into a row-major list
        map_msg.data = self.workspace_map.grid.flatten().tolist()

        self.workspace_map_publisher.publish(map_msg)
        self.get_logger().info("Published workspace occupancy map", once=True)

def main():
    rclpy.init()
    node = MapWorkspace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
