#!/usr/bin/env python

import numpy as np
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from tf2_ros import TransformBroadcaster
from nav_msgs.msg import OccupancyGrid
from project_interfaces.msg import Object, ObjectList
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from project_interfaces.msg import Point, Workspace

from navigation.map import Map

class Mapping(Node):

    def __init__(self):
        super().__init__("mapping")

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(Workspace, "/workspace", self.workspace_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, "/map", 10)

        # Parameters
        self.resolution = 0.05  # 5 cm per cell
        self.map = Map(self.resolution)
        self.workspace_vertices = []

        self.create_timer(0.5, self.update_map)


    def workspace_callback(self, msg):
        if self.map.grid is not None:
            return
        for point_msg in msg.points:
            x = point_msg.x
            y = point_msg.y
            self.workspace_vertices.append((x, y))
        # Initalise map based on workspace perimeter
        self.map.initalise_grid_with_workspace(self.workspace_vertices)

    def update_map(self):
        if not self.workspace_vertices:
            return
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"

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
        self.get_logger().info("Published occupancy map", once=True)

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
