#!/usr/bin/env python

# Creates a path from starting point to end point on map frame
import random
import numpy as np
import heapq

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

from example_interfaces.srv import Trigger

from navigation.mapping import Map

class AdaptiveAStar:
    def __init__(self, grid, adaptive_h=None):
        self.grid = grid
        self.rows, self.columns = grid.shape
        self.adaptive_h = adaptive_h if adaptive_h is not None else {}

    def heuristic(self, node, end_node):
        if node in self.adaptive_h:
            return self.adaptive_h[node]
        # If no adaptive heuristic defaults to manhattan distance
        return abs(node[0] - end_node[0]) + abs(node[1] - end_node[1])

    def get_neighbours(self, node):
        (x, y) = node
        neighbours = []
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            next_x, next_y = x + dx, y + dy
            if 0 <= next_x < self.rows and 0 <= next_y < self.columns:
                if self.grid[next_x, next_y] == 0:  # Check if free cell
                    neighbours.append((next_x, next_y))
        return neighbours

    def plan_path(self, start_node, end_node):
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start_node, end_node), 0, start_node))
        came_from = {}
        g_score = {start_node: 0}
        closed_set = set()

        while open_set:
            f, current_g, current = heapq.heappop(open_set)
            if current == end_node:
                # Reconstruct the path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_node)
                path.reverse()

                # Update adaptive heuristic for all expanded nodes
                for node in closed_set:
                    if node in g_score:
                        self.adaptive_h[node] = g_score[end_node] - g_score[node]
                return path

            closed_set.add(current)
            for neighbour in self.get_neighbours(current):
                tentative_g = g_score[current] + 1  # Uniform cost
                if neighbour in g_score and tentative_g >= g_score[neighbour]:
                    continue  # Not a better path
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g
                f_score = tentative_g + self.heuristic(neighbour, end_node)
                heapq.heappush(open_set, (f_score, tentative_g, neighbour))
        return None  # No path found

class Pathing(Node):
    
    def __init__(self):
        super().__init__("pathing")

        self.create_subscription(OccupancyGrid, "/inflated_map", self.map_callback, 10)
        self.path_publisher = self.create_publisher(Path, "/custom_path", 10)

        # Parameters
        self.start_point = (0, 0)
        self.end_point = (6, 2)
        self.amplitude = 0.5
        self.cycles = 1.0
        self.map_grid = None
        self.adaptive_h = {}
        self.map = None

        self.timer = self.create_timer(2.0, self.publish_astar_path)
        # self.timer = self.create_timer(2.0, self.publish_straight_path)
        # self.timer = self.create_timer(2.0, self.publish_curved_path)


    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        self.map = Map(resolution, origin_x, origin_y)
        self.map_grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

    def publish_astar_path(self):
        if self.map_grid is None or self.map is None:
            self.get_logger().info("Occupancy map grid not received")
            return

        start_x, start_y = self.map.world_to_grid(self.start_point[0], self.start_point[1])
        end_x, end_y = self.map.world_to_grid(self.end_point[0], self.end_point[1])

        path_planner = AdaptiveAStar(self.map_grid, self.adaptive_h)
        path = path_planner.plan_path((start_y, start_x), (end_y, end_x))

        if path is None:
            self.get_logger().info("No path found")
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for point in path:
            x, y = self.map.grid_to_world(point[1], point[0])
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published A star custom path")


    def publish_curved_path(self):
        start_x, start_y = self.start_point
        end_x, end_y = self.end_point

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        dx = end_x - start_x
        dy = end_y - start_y
        length = np.sqrt(dx**2 + dy**2)
        
        ux = dx / length
        uy = dy / length
        
        nx = -uy
        ny = ux
        
        path_resolution = 20

        for i in range(path_resolution + 1):
            t = i / path_resolution  # Parameter t goes from 0 to 1
            bx = start_x + t * dx
            by = start_y + t * dy
            
            offset = self.amplitude * np.sin(2 * np.pi * self.cycles * t)
            x = bx + offset * nx
            y = by + offset * ny
            
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published curved custom path")
        

    def publish_straight_path(self):
        start_x, start_y = self.start_point
        end_x, end_y = self.end_point

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        # Intermidiate points
        path_resolution = 20
        if end_x == 0 and end_y == 0:
            self.get_logger().info("No end point selected")
            return

        for i in range(path_resolution):
            pose = PoseStamped()
            pose.header = path_msg.header
            t = i/(path_resolution - 1)
            pose.pose.position.x = (1 - t) * start_x + t * end_x
            pose.pose.position.y = (1 - t) * start_y + t * end_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published straight custom path")

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
