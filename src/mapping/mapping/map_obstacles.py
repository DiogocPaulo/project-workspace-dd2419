#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, TransformStamped, Pose, Quaternion, Vector3
from sensor_msgs.msg import LaserScan
from project_interfaces.msg import Vertex, WorkspaceVertices

from mapping.map import Map

class MapObstacles(Node):
    
    def __init__(self):
        super().__init__("map_obstacles")

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(WorkspaceVertices, "/workspace", self.workspace_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile)
        self.obstacles_map_publisher = self.create_publisher(OccupancyGrid, "/obstacles_map", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Constants
        self.ignore_distance = 0.25

        # Variables
        self.workspace_vertices = []
        self.obstacles_map = None

    def workspace_callback(self, msg: WorkspaceVertices):
        if self.workspace_vertices:
            return
        for vertex_msg in msg.vertices:
            x = vertex_msg.x
            y = vertex_msg.y
            self.workspace_vertices.append((x, y))
        # Initialise empty map based on workspace perimeter
        self.obstacles_map = Map(msg.grid_resolution)
        self.obstacles_map.initialise_grid(self.workspace_vertices, empty=True)

    def scan_callback(self, msg: LaserScan):
        if self.obstacles_map is None:
            return

        try:
            lidar_transform = self.tf_buffer.lookup_transform(
                "odom",
                msg.header.frame_id,
                rclpy.time.Time(seconds=0),
                rclpy.duration.Duration(seconds=1.0)
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not find transform between lidar to odom frames: {ex}")
            return

        lidar_origin_x, lidar_origin_y = self.transform_point(0.0, 0.0, lidar_transform)

        angle = msg.angle_min

        # Process laser scan readings
        for reading in msg.ranges:
            valid = not (math.isinf(reading) or math.isnan(reading))
            if not valid:
                reading = msg.range_max

            point_x = reading * math.cos(angle)
            point_y = reading * math.sin(angle)
            distance = np.hypot(point_x, point_y)
            if distance > self.ignore_distance:
                lidar_point_x, lidar_point_y = self.transform_point(point_x, point_y, lidar_transform)
                self.obstacles_map.update_obstacles(valid, lidar_origin_x, lidar_origin_y, lidar_point_x, lidar_point_y)

            angle += msg.angle_increment

        self.publish_obstacles_map()

    def transform_point(self, x, y, transform_msg: TransformStamped):
        point_msg = PointStamped()
        point_msg.header.frame_id = transform_msg.header.frame_id
        point_msg.header.stamp = transform_msg.header.stamp
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0

        transformed_point_msg = tf2_geometry_msgs.do_transform_point(
            point_msg, transform_msg
        )

        transformed_x = transformed_point_msg.point.x
        transformed_y = transformed_point_msg.point.y
        return transformed_x, transformed_y

    def publish_obstacles_map(self):
        if self.obstacles_map is None:
            return
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "odom"

        map_msg.info.resolution = self.obstacles_map.resolution
        map_msg.info.width = self.obstacles_map.grid_width
        map_msg.info.height = self.obstacles_map.grid_height
        map_msg.info.origin.position.x = self.obstacles_map.origin_x
        map_msg.info.origin.position.y = self.obstacles_map.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatting grid into a row-major list
        map_msg.data = self.obstacles_map.grid.flatten().tolist()

        self.obstacles_map_publisher.publish(map_msg)
        self.get_logger().info("Published workspace occupancy map", once=True)

def main():
    rclpy.init()
    node = MapObstacles()
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
