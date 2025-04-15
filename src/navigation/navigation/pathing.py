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
from geometry_msgs.msg import PoseStamped, Point

from project_interfaces.srv import GoToPoint, Trigger
from project_interfaces.msg import NavPoint, NavPath

from mapping.map import Map
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
        self.create_subscription(OccupancyGrid, "/workspace_map", self.workspace_map_callback, qos_profile)
        self.create_subscription(OccupancyGrid, "/objects_map", self.objects_map_callback, qos_profile)
        self.create_subscription(OccupancyGrid, "/obstacles_map", self.obstacles_map_callback, qos_profile)
        self.path_publisher = self.create_publisher(NavPath, "/custom_path", 10)
        self.temp_path_publisher = self.create_publisher(Path, "/temp_path", 10)
        self.path_map_publisher = self.create_publisher(OccupancyGrid, "/path_map", 10)
        self.inflated_map_publisher = self.create_publisher(OccupancyGrid, "/inflated_map", 10)
        self.end_point_service = self.create_service(GoToPoint, "/pathing_end_point", self.receive_end_point)

        # Constants
        self.adaptive_h = {}
        self.base = 0.35
        self.region_radius = 1.0
        self.workspace_inflation_radius = 0.30
        self.objects_inflation_radius = 0.40
        self.obstacles_inflation_radius = 0.35

        # Variables
        self.start_point = (0.0, 0.0)
        self.end_point = (None, None)
        self.safe_point = (0.0, 0.0)
        self.target_yaw = 0.0
        self.target_velocity = 0.0
        self.workspace_map = None
        self.objects_map = None
        self.obstacles_map = None
        self.inflated_map = None
        self.path_grid = None
        self.pathing_failed = False
        self.backing = False
        self.reversing = False
        self.slow_approach = False

    def odom_callback(self, msg: Odometry):
        self.start_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.inflated_map is None:
            return
        if self.backing and self.inflated_map.are_adjacent_free(self.start_point[0], self.start_point[1], 1, 75):
            self.get_logger().info("Exiting reverse travel mode")
            self.backing = False
            self.calculate_astar_path()

    def workspace_map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        if self.workspace_map is None:
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            self.workspace_map = Map(resolution, origin_x, origin_y, width, height, grid)
            if self.inflated_map is None:
                self.inflated_map = Map(resolution, origin_x, origin_y, width, height)
        else:
            self.workspace_map.update_grid(grid)
        self.workspace_map.inflate_grid(self.workspace_inflation_radius)
        self.update_inflated_map()

    def objects_map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        if self.objects_map is None:
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            self.objects_map = Map(resolution, origin_x, origin_y, width, height, grid)
        else:
            self.objects_map.update_grid(grid)
        self.objects_map.inflate_grid(self.objects_inflation_radius)
        self.update_inflated_map()

    def obstacles_map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        if self.obstacles_map is None:
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            self.obstacles_map = Map(resolution, origin_x, origin_y, width, height, grid)
        else:
            self.obstacles_map.update_grid(grid)
        self.obstacles_map.inflate_grid(self.obstacles_inflation_radius)
        self.update_inflated_map()

    def update_inflated_map(self):
        if self.inflated_map is None:
            return
        empty_grid = np.full((self.inflated_map.grid_height, self.inflated_map.grid_width), -1, dtype=np.int8)
        workspace_grid = empty_grid if self.workspace_map is None else self.workspace_map.grid
        objects_grid = empty_grid if self.objects_map is None else self.objects_map.grid
        obstacles_grid = empty_grid if self.obstacles_map is None else self.obstacles_map.grid

        stacked_grids = np.stack([workspace_grid, objects_grid, obstacles_grid])
        inflated_grid = np.max(stacked_grids, axis=0)
        self.inflated_map.update_grid(inflated_grid)

        if grid_intersection(self.path_grid, self.inflated_map.grid):
            self.calculate_astar_path()
        else:
            self.get_logger().info("Current path is still valid")
        self.publish_inflated_map()

    def receive_end_point(self, request, response):
        if self.end_point != (request.x, request.y):
            self.end_point = (request.x, request.y)
            self.target_yaw = request.yaw
            self.target_velocity = request.velocity
            self.reversing = request.reverse
            self.slow_approach = request.slow_approach
            self.path_grid = None
            self.pathing_failed = False
            self.calculate_astar_path()

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

    def publish_inflated_map(self):
        if self.inflated_map is None:
            return

        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "odom"

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
        self.get_logger().info("Published inflated occupancy map")

    def publish_path_map(self):
        if self.inflated_map is None or self.path_grid is None:
            return

        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "odom"

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
        map_msg.data = self.path_grid.flatten().tolist()

        self.path_map_publisher.publish(map_msg)
        self.get_logger().info("Published custom path as occupancy map")

    def publish_temp_path(self, path):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        if path is not None:
            for point in path:
                x, y = self.inflated_map.grid_to_world(point[1], point[0])
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                path_msg.poses.append(pose)
        else:
            path_msg.poses = []

        self.temp_path_publisher.publish(path_msg)

    def publish_path(self, path):
        path_msg = NavPath()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "odom"

        if path is not None:
            for point in path:
                x, y = self.inflated_map.grid_to_world(point[1], point[0])
                point_msg = NavPoint()
                point_msg.x = x
                point_msg.y = y
                path_msg.path.append(point_msg)
            point_msg = NavPoint()
            point_msg.x = self.end_point[0]
            point_msg.y = self.end_point[1]
            path_msg.path.append(point_msg)
        else:
            path_msg.path = []
            self.get_logger().warn("Publishing empty custom path")

        path_msg.yaw = self.target_yaw
        path_msg.velocity = self.target_velocity
        path_msg.reverse = self.backing or self.reversing
        path_msg.slow_approach = self.slow_approach

        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published custom path")
    
    def calculate_astar_path(self):
        if self.start_point == (None, None):
            self.get_logger().warn("No start point received")
            return
        if self.end_point == (None, None):
            self.get_logger().warn("No end point received")
            return
        if self.inflated_map is None:
            self.get_logger().warn("Inflated occupancy grid not created")
            return

        if not self.backing and not self.inflated_map.is_free(self.start_point[0], self.start_point[1], 75):
            self.get_logger().info("Entering reverse travel mode")
            self.backing = True
            self.safe_point = self.inflated_map.get_safe_point(self.start_point[0], self.start_point[1], 2, 75)
            if self.safe_point is None:
                self.safe_point = (0.0, 0.0)

        path_planner = AdaptiveAStar(self.inflated_map.grid)
        start_x, start_y = self.inflated_map.world_to_grid(self.start_point[0], self.start_point[1])
        if not self.backing:
            end_x, end_y = self.inflated_map.world_to_grid(self.end_point[0], self.end_point[1])
            path = path_planner.plan_path((start_y, start_x), (end_y, end_x), 75)
        else:
            self.get_logger().warn("Creating path from inside inflation radius")
            end_x, end_y = self.inflated_map.world_to_grid(self.safe_point[0], self.safe_point[0])
            path = path_planner.plan_path((start_y, start_x), (end_y, end_x), 100)

        if path is None:
            self.pathing_failed = True
            self.path_grid = None
        self.publish_path(path)
        self.publish_temp_path(path)
        if path is not None:
            self.path_grid = np.full(self.inflated_map.grid.shape, -1, dtype=np.int8)
            for x, y in path:
                self.path_grid[x, y] = 100
        self.publish_path_map()

def main():
    rclpy.init()
    node = Pathing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
