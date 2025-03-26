#!/usr/bin/env python

import math
import numpy as np
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from tf2_ros import TransformException
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
        self.map_publisher = self.create_publisher(OccupancyGrid, "/map", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Parameters
        self.resolution = 0.05  # 5 cm per cell
        self.map = Map(self.resolution)
        self.workspace_vertices = []

        self.create_timer(0.5, self.update_map)


    def workspace_callback(self, msg: Workspace):
        if self.workspace_vertices:
            return
        for point_msg in msg.points:
            x = point_msg.x
            y = point_msg.y
            self.workspace_vertices.append((x, y))
        # Initalise map based on workspace perimeter
        self.map.initalise_grid_with_workspace(self.workspace_vertices)

    def scan_callback(self, msg: LaserScan):
        if self.map.grid is None:
            return

        # Lidar origin
        lidar_origin = PointStamped()
        lidar_origin.header.stamp = msg.header.stamp
        lidar_origin.header.frame_id = msg.header.frame_id
        lidar_origin.point.x = 0.0
        lidar_origin.point.y = 0.0
        lidar_origin.point.z = 0.0

        try:
            origin_transform = self.tf_buffer.lookup_transform(
                "odom",
                msg.header.frame_id,
                rclpy.time.Time(seconds=0),
                rclpy.duration.Duration(seconds=1.0)
            )
            transformed_origin = tf2_geometry_msgs.do_transform_point(lidar_origin, origin_transform)
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform lidar origin reading ({lidar_origin.point.x}, {lidar_origin.point.y}): {ex}")
            return

        angle = msg.angle_min

        # Process laser scan readings
        for reading in msg.ranges:
            valid = not (math.isinf(reading) or math.isnan(reading))
            if not valid:
                reading = msg.range_max
                

            if not (angle > 0 and angle < (math.pi / 2)):
                angle += msg.angle_increment
                continue

            # Lidar point
            lidar_point = PointStamped()
            lidar_point.header.stamp = msg.header.stamp
            lidar_point.header.frame_id = msg.header.frame_id
            lidar_point.point.x = reading * math.cos(angle)
            lidar_point.point.y = reading * math.sin(angle)
            lidar_point.point.z = 0.0

            try:
                point_transform = self.tf_buffer.lookup_transform(
                    "odom",
                    msg.header.frame_id,
                    rclpy.time.Time(seconds=0),
                    rclpy.duration.Duration(seconds=1.0)
                )
                transformed_point = tf2_geometry_msgs.do_transform_point(lidar_point, point_transform)

            except TransformException as ex:
                self.get_logger().warn(f"Could not transform lidar point reading ({lidar_point.point.x}, {lidar_point.point.y}): {ex}")
                angle += msg.angle_increment
                continue

            self.map.update_obstacles_in_line(transformed_origin.point.x, transformed_origin.point.y, transformed_point.point.x, transformed_point.point.y, valid)
            angle += msg.angle_increment

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
