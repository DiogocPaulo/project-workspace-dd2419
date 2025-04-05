#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid

from project_interfaces.msg import Vertex, WorkspaceVertices
from project_interfaces.msg import Object, ObjectList

from mapping.map import Map

class MapObjects(Node):
    def __init__(self):
        super().__init__("map_objects")

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(WorkspaceVertices, "/workspace", self.workspace_callback, 10)
        self.create_subscription(ObjectList, "/detected_objects", self.objects_callback, qos_profile)
        self.objects_map_publisher = self.create_publisher(OccupancyGrid, "/objects_map", 10)

        # Variables
        self.workspace_vertices = []
        self.objects_map = None
        self.object_list_type = [("object_type", "<U10"), ("x", float), ("y", float), ("angle", float)]
        self.previous_object_list = np.array([], dtype=self.object_list_type)

    def workspace_callback(self, msg: WorkspaceVertices):
        if self.workspace_vertices:
            return
        for vertex_msg in msg.vertices:
            x = vertex_msg.x
            y = vertex_msg.y
            self.workspace_vertices.append((x, y))
        # Initialise map based on workspace perimeter
        self.objects_map = Map(msg.grid_resolution)
        self.objects_map.initialise_grid(self.workspace_vertices, empty=True)

    def objects_callback(self, msg):
        if self.objects_map is None:
            return
        current_object_list = np.array([
            (object_msg.object_type, object_msg.x, object_msg.y, object_msg.angle)
            for object_msg in msg.objects
        ], dtype=self.object_list_type)

        if not np.array_equal(current_object_list, self.previous_object_list):
            self.objects_map.empty_grid()
            for object_type, x, y, angle in current_object_list:
                self.get_logger().info(f"Object Type {str(object_type)}")
                self.objects_map.add_object(str(object_type), x, y, angle)
            self.publish_objects_map()

        self.previous_object_list = msg.objects

    def publish_objects_map(self):
        if self.objects_map is None:
            return
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "odom"

        map_msg.info.resolution = self.objects_map.resolution
        map_msg.info.width = self.objects_map.grid_width
        map_msg.info.height = self.objects_map.grid_height
        map_msg.info.origin.position.x = self.objects_map.origin_x
        map_msg.info.origin.position.y = self.objects_map.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatting grid into a row-major list
        map_msg.data = self.objects_map.grid.flatten().tolist()

        self.objects_map_publisher.publish(map_msg)
        self.get_logger().info("Published objects occupancy map", once=True)

def main():
    rclpy.init()
    node = MapObjects()
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
