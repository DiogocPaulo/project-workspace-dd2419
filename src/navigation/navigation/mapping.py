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

from navigation.map import Map

class Mapping(Node):

    def __init__(self):
        super().__init__("mapping")

        self.map_publisher = self.create_publisher(OccupancyGrid, "/map", 10)
        self.marker_publisher = self.create_publisher(Marker, "/workspace", 10)

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(
            ObjectList,
            "/detected_objects",
            self.objects_callback,
            qos_profile
        )

        self.create_timer(0.5, self.update_map)
        # self.create_timer(0.5, self.update_marker)

        # Parameters
        self.resolution = 0.05  # 5 cm per cell
        self.objects = None
        self.map = Map(self.resolution)

        # Exploration workspace perimeter
        # self.workspace_vertices = [
        #     (-2.20, -1.30),
        #     (2.20, -1.30),
        #     (4.50, 0.66),
        #     (7.00, 0.66),
        #     (7.00, 2.84),
        #     (5.46, 2.84),
        #     (5.46, 1.30),
        #     (-2.20, 1.30)
        # ]

        # Collection workspace perimeter
        self.workspace_vertices = [
            (-2.20, -1.30),
            (2.20, -1.30),
            (2.20, 1.30),
            (-2.20, 1.30),
        ]

        # Initalise map based on workspace perimeter
        self.map.initalise_grid_with_workspace(self.workspace_vertices)

    def objects_callback(self, msg):
        for object_msg in msg.objects:
            x = object_msg.x
            y = object_msg.y
            angle = object_msg.angle
            object_type = object_msg.object_type
            self.map.add_object(x, y, angle, object_type)

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

    def update_map(self):
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
        self.get_logger().debug("Published occupancy map", once=True)

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
