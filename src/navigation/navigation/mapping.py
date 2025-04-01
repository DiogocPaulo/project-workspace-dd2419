#!/usr/bin/env python

import math
import numpy as np
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, Pose, Quaternion, Vector3
from sensor_msgs.msg import LaserScan
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
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, "/lidar_map", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Parameters
        self.resolution = 0.05  # 5 cm per cell
        self.map = Map(self.resolution)
        self.workspace_vertices = []
        self.lidar_origin_x = None
        self.lidar_origin_y = None

    def workspace_callback(self, msg: Workspace):
        if self.workspace_vertices:
            return
        for point_msg in msg.points:
            x = point_msg.x
            y = point_msg.y
            self.workspace_vertices.append((x, y))
        # Initalise map based on workspace perimeter
        self.map.initialise_grid(self.workspace_vertices)
        self.update_map()

    def scan_callback(self, msg: LaserScan):
        if self.map.grid is None:
            return

        try:
            if not self.tf_buffer.can_transform(
                "map", msg.header.frame_id, rclpy.time.Time(seconds=0), rclpy.duration.Duration(seconds=1.0)
            ):
                self.get_logger().warn(f"No transform from lidar_link to map found")
                return
            lidar_transform = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, rclpy.time.Time(seconds=0)
            )

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Transform exception for lidar: {ex}")

        if (self.lidar_origin_x, self.lidar_origin_y) == (None, None):
            self.lidar_origin_x, self.lidar_origin_y = self.transform_point(lidar_transform, 0.0, 0.0)

        angle = msg.angle_min

        # Process laser scan readings
        for reading in msg.ranges:
            valid = not (math.isinf(reading) or math.isnan(reading))
            if not valid:
                reading = msg.range_max
                

            if not (angle > -(math.pi * 0.25) and angle < (math.pi * 0.75)):
                angle += msg.angle_increment
                continue

            point_x, point_y = self.transform_point(lidar_transform, reading * math.cos(angle), reading * math.sin(angle))

            self.map.update_obstacles(valid, self.lidar_origin_x, self.lidar_origin_y, point_x, point_y)
            angle += msg.angle_increment

        self.update_map()

    def transform_point(self, transform, x, y):
        point_msg = PointStamped()
        point_msg.header.frame_id = "lidar_link"
        point_msg.header.stamp = rclpy.time.Time(seconds=0)
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0

        transformed_point_msg = tf2_geometry_msgs.do_transform_point(
            point_msg, transform
        )

        transformed_x = transformed_point_msg.point.x
        transformed_y = transformed_point_msg.point.y
        return transformed_x, transformed_y

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
