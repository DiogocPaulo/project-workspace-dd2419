#!/usr/bin/env python

import numpy as np
import sys

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3

class Mapping(Node):

    def __init__(self):
        super().__init__("mapping")

        self.map_publisher = self.create_publisher(OccupancyGrid, "map", 10)
        self.marker_publisher = self.create_publisher(Marker, 'workspace_perimeter', 10)

        self.create_timer(0.5, self.update_map)
        self.create_timer(0.5, self.update_marker)

        # Parameters
        self.resolution = 0.05  # 5 cm per cell
        self.width = 400        # 200 cells in width
        self.height = 400       # 200 cells in height
        self.origin_x = -10.0     # Map origin x
        self.origin_y = -10.0     # Map origin y

        self.workspace_vertices = [
            (-2.20, -1.30),
            (2.20, -1.30),
            (4.50, 0.66),
            (7.00, 0.66),
            (7.00, 2.84),
            (5.46, 2.84),
            (5.46, 1.30),
            (-2.20, 1.30)
        ]

        # self.workspace_vertices = [
        #     (-2.20, -1.30),
        #     (2.20, -1.30),
        #     (2.20, 1.30),
        #     (-2.20, 1.30),
        # ]

        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.define_workspace(self.width, self.height, self.workspace_vertices)

    def update_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Ensure this frame exists in your TF tree
        marker.header.stamp = self.get_clock().now().to_msg()  # Ensure current timestamp
        marker.ns = "workspace"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set the scale of the lines (thickness)
        marker.scale.x = 0.05  # Increased line thickness for better visibility

        # Set the color of the lines (e.g., green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Add the vertices of the workspace polygon
        for x, y in self.workspace_vertices:
            point = Point()
            point.x = x  # Already in meters
            point.y = y  # Already in meters
            point.z = 0.0  # Workspace is on the ground (z = 0)
            marker.points.append(point)

        # Close the polygon by adding the first vertex again
        first_point = Point()
        first_point.x = self.workspace_vertices[0][0]
        first_point.y = self.workspace_vertices[0][1]
        first_point.z = 0.0
        marker.points.append(first_point)

        # Publish the marker
        self.marker_publisher.publish(marker)

    def world_to_grid(self, x, y):
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def define_workspace(self, width, height, vertices):
        for i in range(width):
            for j in range(height):
                x = self.origin_x + j * self.resolution
                y = self.origin_y + i * self.resolution

                if self.is_workspace_point(x, y, self.workspace_vertices):
                    self.grid[i, j] = 0
                else:
                    self.grid[i, j] = 100

    def is_workspace_point(self, x, y, vertices):
        inside = False
        j = len(vertices) - 1
        for i in range(len(vertices)):
            ax, ay = vertices[i]
            bx, by = vertices[j]
            if ay > by:
                ax, bx = bx, ax
                ay, by = by, ay

            # Make sure point is not at same height as vertex
            if y == ay or y == by:
                y += 0.00001

            if (y > by or y < ay or x > max(ax, bx)):
                # The horizontal ray does not intersect with the edge
                j = i
                continue

            if x < min(ax, bx): # The ray intersects with the edge
                inside = not inside
                j = i
                continue

            try:
                m_edge = (by - ay) / (bx - ax)
            except ZeroDivisionError:
                m_edge = sys.float_info.max

            try:
                m_point = (y - ay) / (x - ax)
            except ZeroDivisionError:
                m_point = sys.float_info.max

            if m_point >= m_edge:
                # The ray intersects with the edge
                inside = not inside
                j = i
                continue

            j = i
        return inside
        

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
