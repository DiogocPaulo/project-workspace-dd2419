#!/usr/bin/env python

# Creates a path from starting point to end point on map frame
import random
import numpy as np
import heapq

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from project_interfaces.msg import Object, ObjectList
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped

from project_interfaces.srv import GoToPoint, Trigger

from navigation.map import Map
from navigation.adaptive_a_star import AdaptiveAStar

def grid_intersection(grid1, grid2):
    if grid1 is None or grid2 is None:
        return True
    height1, width1 = grid1.shape
    height2, width2 = grid2.shape

    if height1 != height2 or width1 != width2:
        return True

    return np.any((grid1 > 0) & (grid2 > 0))


class Pathing(Node):
    
    def __init__(self):
        super().__init__("pathing")

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(ObjectList, "/detected_objects", self.objects_callback, qos_profile)
        self.path_publisher = self.create_publisher(Path, "/custom_path", 10)
        self.path_map_publisher = self.create_publisher(OccupancyGrid, "/path_map", 10)
        self.inflated_map_publisher = self.create_publisher(OccupancyGrid, "/inflated_map", 10)
        self.end_point_service = self.create_service(GoToPoint, "/pathing_end_point", self.receive_end_point)

        # Parameters
        self.start_point = (0.0, 0.0)
        self.end_point = (None, None)
        self.amplitude = 0.5
        self.cycles = 1.0
        self.adaptive_h = {}
        self.base = 0.35
        self.region_radius = 1.0
        self.map = None
        self.inflated_map = None
        self.path_grid = None
        self.object_list = []
        self.pathing_failed = False
        
        self.create_timer(0.05, self.publish_astar_path)
        self.create_timer(0.5, self.update_inflated_map)
        self.create_timer(0.5, self.update_path_map)

    def odom_callback(self, msg: Odometry):
        self.start_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Create map or update map grid
        if self.map is None:
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            self.map = Map(resolution, origin_x, origin_y, width, height, grid)
        else:
            self.map.update_grid(grid)

    def objects_callback(self, msg):
        self.object_list = msg.objects

    def receive_end_point(self, request, response):
        if self.end_point != (request.x, request.y):
            self.end_point = (request.x, request.y)
            self.pathing_failed = False

        if not self.pathing_failed:
            response.success = True
            response.message = f"Pathing end point set: ({self.end_point[0]:.2f}, {self.end_point[1]:.2f})"
        else:
            if self.inflated_map.is_free(self.start_point[0], self.start_point[1], 50):
                response.success = False
                response.message = f"Failed to find path to end point: ({self.end_point[0]:.2f}, {self.end_point[1]:.2f})"
            else:
                response.success = True
                response.message = f"Currently within inflation radius waiting on navigation: ({self.start_point[0]:.2f}, {self.start_point[1]:.2f}) = {self.inflated_map.get_occupancy(self.start_point[0], self.start_point[1])}"

        return response

    def update_inflated_map(self):
        if self.inflated_map is None:
            return

        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"

        map_msg.info.resolution = self.inflated_map.resolution
        map_msg.info.width = self.inflated_map.grid_width
        map_msg.info.height = self.inflated_map.grid_height
        map_msg.info.origin.position.x = self.inflated_map.origin_x
        map_msg.info.origin.position.y = self.inflated_map.origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.x = 0.0
        map_msg.info.origin.orientation.y = 0.0
        map_msg.info.origin.orientation.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatting grid into a row-major list
        map_msg.data = self.inflated_map.grid.flatten().tolist()

        self.inflated_map_publisher.publish(map_msg)
        self.get_logger().info("Published inflated occupancy map", once=True)

    def update_path_map(self):
        if self.map is None or self.path_grid is None:
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
        map_msg.data = self.path_grid.flatten().tolist()

        self.path_map_publisher.publish(map_msg)
        self.get_logger().info("Published custom path as occupancy map", once=True)

    def publish_astar_path(self):
        if self.start_point == (None, None):
            self.get_logger().warn("No start point received")
            return
        if self.end_point == (None, None):
            self.get_logger().warn("No end point received")
            return
        if self.map is None:
            self.get_logger().warn("Occupancy map not received")
            return

        start_x, start_y = self.map.world_to_grid(self.start_point[0], self.start_point[1])
        end_x, end_y = self.map.world_to_grid(self.end_point[0], self.end_point[1])

        inflation_radius = self.base
        path = None
        while path is None and not self.pathing_failed:
            self.inflated_map = Map(self.map.resolution, self.map.origin_x, self.map.origin_y, self.map.grid_width, self.map.grid_height, self.map.grid)
            for object_msg in self.object_list:
                x = object_msg.x
                y = object_msg.y
                angle = object_msg.angle
                object_type = object_msg.object_type
                self.inflated_map.add_object(x, y, angle, object_type)
            self.inflated_map.inflate_grid_by_half(inflation_radius)
            if not grid_intersection(self.inflated_map.grid, self.path_grid):
                self.get_logger().info("Current path is still valid")
                return
            path_planner = AdaptiveAStar(self.inflated_map.grid, self.adaptive_h)
            path, self.path_grid = path_planner.plan_path((start_y, start_x), (end_y, end_x))

            if path is not None:
                break
            # if inflation_radius > 0.35:
            #     inflation_radius -= self.map.resolution
            #     self.get_logger().info("No path found, trying smaller inflation radius")
            #     continue
            else:
                # If the path is none
                self.get_logger().warn("No path found")
                self.pathing_failed = True

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        if path is not None:
            for point in path:
                x, y = self.map.grid_to_world(point[1], point[0])
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                path_msg.poses.append(pose)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = self.end_point[0]
            pose.pose.position.y = self.end_point[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        else:
            path_msg.poses = []
            self.get_logger().warn("Publishing empty path")

        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published A star custom path")

def main():
    rclpy.init()
    node = Pathing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
