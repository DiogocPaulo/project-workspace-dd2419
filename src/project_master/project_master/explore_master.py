#!/usr/bin/env python

import math
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import transform

import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from project_interfaces.srv import GoToPoint, Trigger
from nav_msgs.msg import Path, Odometry
from project_interfaces.msg import Vertex, WorkspaceVertices
from mapping.map import Map

from project_master import behaviours

def offset_outer_vertices(workspace_vertices, offset):
    vertices = [np.array(vertex) for vertex in workspace_vertices]
    num_vertices = len(vertices)

    offset_vertices = []

    def compute_offset_normal(p, q):
        edge_vector = q - p
        norm_edge = np.linalg.norm(edge_vector)
        if norm_edge == 0:
            return np.array([0, 0])
        perp_vector = np.array([-edge_vector[1], edge_vector[0]])
        normal_vector = perp_vector / np.linalg.norm(perp_vector)
        cross_product = np.cross(edge_vector, perp_vector)
        if cross_product < 0:
            normal_vector = -normal_vector
        return normal_vector

    for i in range(num_vertices + 1):
        prev = vertices[(i - 1) % num_vertices]
        current = vertices[(i) % num_vertices]
        nxt = vertices[(i + 1) % num_vertices]

        normal1 = compute_offset_normal(prev, current)
        normal2 = compute_offset_normal(current, nxt)

        p1 = current + normal1 * offset
        d1 = current - prev  # direction of the incoming edge
        p2 = current + normal2 * offset
        d2 = nxt - current   # direction of the outgoing edge

        denom = np.cross(d1, d2)
        if np.abs(denom) < 1e-6:
            offset_vertex = current + normal1 * offset
        else:
            t = np.cross((p2 - p1), d2) / denom
            offset_vertex = p1 + t * d1

        offset_vertices.append((offset_vertex[0], offset_vertex[1]))

    return offset_vertices

def offset_inner_vertices(vertices, offset):
    offset_vertices = []
    polygon = Polygon(vertices)
    offset_polygon = polygon.buffer(-offset)
    if offset_polygon.geom_type == 'Polygon':
        offset_vertices = list(offset_polygon.exterior.coords)
    if offset_polygon.geom_type == 'MultiPolygon':
        largest_polygon = max(offset_polygon.geoms, key=lambda p: p.area)
        offset_vertices = list(largest_polygon.exterior.coords)
    return offset_vertices[:-1]

def shortest_edges_midpoints(vertices):
    if len(vertices) < 4:
        return []

    edges = []
    for i in range(len(vertices)):
        current_x, current_y = vertices[i]
        next_x, next_y = vertices[(i + 1) % len(vertices)]
        distance = np.hypot(next_x - current_x, next_y - current_y)
        edges.append({
            "p1": (current_x, current_y),
            "p2": (next_x, next_y),
            "distance": distance
        })
    sorted_edges = sorted(edges, key=lambda edge: edge["distance"])
    shortest_edges = sorted_edges[:2]

    midpoints = []
    headings = []
    for edge in shortest_edges:
        p1 = edge["p1"]
        p2 = edge["p2"]
        midpoint_x = (p1[0] + p2[0]) / 2
        midpoint_y = (p1[1] + p2[1]) / 2
        midpoints.append((midpoint_x, midpoint_y))
        normal_vector_x = -(p2[1] - p1[1])
        normal_vector_y = (p2[0] - p1[0])
        heading = np.arctan2(normal_vector_y, normal_vector_x)
        headings.append(heading)

    return midpoints, headings[-1]

def generate_waypoints(map: Map, workspace_vertices, outer_offset, inner_offset):
    outer_vertices = offset_outer_vertices(workspace_vertices, outer_offset)
    # outer_vertices.reverse()
    inner_vertices = offset_inner_vertices(workspace_vertices, inner_offset)
    inner_midpoints, final_heading = shortest_edges_midpoints(inner_vertices)
    offset_vertices = outer_vertices[:-1] + inner_midpoints
    # offset_vertices = outer_vertices + inner_midpoints
    waypoints = []
    num_vertices = len(offset_vertices)
    for i in range(num_vertices - 1):
        current_x, current_y = offset_vertices[i]
        next_x, next_y = offset_vertices[(i + 1) % num_vertices]
        distance = np.hypot(next_x - current_x, next_y - current_y)
        resolution = max(1, math.ceil(distance * 1.2))
        if (i > len(outer_vertices) - 2):
            resolution = max(1, math.ceil(distance * 0.6))
        for j in range(resolution):
            x = current_x + (next_x - current_x) * (j + 1) / resolution
            y = current_y + (next_y - current_y) * (j + 1) / resolution
            if map.is_free(x, y, 1):
                waypoints.append((x, y, 0.0))

    num_waypoints = len(waypoints) 
    for i in range(num_waypoints):
        current_x, current_y, _ = waypoints[i]
        next_x, next_y, _ = waypoints[(i + 1) % num_waypoints]
        heading = np.arctan2(next_y - current_y, next_x - current_x)
        rounded_x, rounded_y = map.round_world(current_x, current_y)
        if i == (num_waypoints - 1):
            waypoints[i] = (rounded_x, rounded_y, final_heading)
        else:
            waypoints[i] = (rounded_x, rounded_y, heading)

    return waypoints

class ExploreMaster(Node):
    def __init__(self):
        super().__init__("explore_master")

        workspace_file = "workspaces/angled_workspace.tsv"
        self.workspace_vertices = self.read_workspace(workspace_file, skip_header=True)
        self.workspace_publisher = self.create_publisher(WorkspaceVertices, "/workspace", 10)
        self.waypoints_path_publisher = self.create_publisher(Path, "/waypoints_path", 10)
        self.end_points_broadcaster = TransformBroadcaster(self)

        #Constants
        self.target_velocity = 0.16
        self.resolution = 0.05
        self.distance_threshold = 0.1
        self.yaw_threshold = math.radians(7)

        #Variables

        self.map = Map(self.resolution)
        self.map.initialise_grid(self.workspace_vertices)
        self.map.inflate_grid(0.30)
        self.show_waypoints = True
        self.explore_complete = False
        self.end_points = generate_waypoints(self.map, self.workspace_vertices, 0.35, 0.50)

        root = self.create_exploration_tree()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root)
        tree_string = py_trees.display.ascii_tree(root)
        self.get_logger().info(f"Behavior Tree Structure:\n{tree_string}")
        self.tree.setup(timeout=15, node=self)

        self.create_timer(0.1, self.tick_tree) # Tick tree every 100 ms
        self.create_timer(2, self.publish_workspace)
        if self.show_waypoints:
            self.create_timer(2, self.publish_waypoints_path)
            self.create_timer(2, self.broadcast_waypoints)

    def publish_workspace(self):
        workspace_msg = WorkspaceVertices()
        workspace_msg.header.stamp = self.get_clock().now().to_msg()
        workspace_msg.header.frame_id = "map"

        for vertex in self.workspace_vertices:
            vertex_msg = Vertex()
            vertex_msg.x = vertex[0]
            vertex_msg.y = vertex[1]
            workspace_msg.vertices.append(vertex_msg)

        workspace_msg.grid_resolution = self.resolution

        self.workspace_publisher.publish(workspace_msg)
        self.get_logger().info("Published workspace vertices", once=True)

    def publish_waypoints_path(self):
        if not self.end_points:
            return
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"

        for i in range(len(self.end_points)):
            pose_msg = PoseStamped()
            pose_msg.header = path_msg.header
            pose_msg.pose.position.x = self.end_points[i][0]
            pose_msg.pose.position.y = self.end_points[i][1]
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            path_msg.poses.append(pose_msg)

        self.waypoints_path_publisher.publish(path_msg)
        self.get_logger().info("Published waypoints path", once=True)


    def broadcast_waypoints(self):
        for i, point in enumerate(self.end_points):
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = np.sin(point[2] * 0.5)
            quaternion.w = np.cos(point[2] * 0.5)

            transform = TransformStamped()
            transform.header.frame_id = "map"  # Change to your desired parent frame
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.child_frame_id = f"end_point_{i}"
            
            # Set translation from end_point coordinates
            transform.transform.translation.x = point[0]
            transform.transform.translation.y = point[1]
            transform.transform.translation.z = 0.0
            
            transform.transform.rotation.x = quaternion.x
            transform.transform.rotation.y = quaternion.y
            transform.transform.rotation.z = quaternion.z
            transform.transform.rotation.w = quaternion.w
            
            self.end_points_broadcaster.sendTransform(transform)

    def create_exploration_tree(self):
        root = py_trees.composites.Sequence("ExplorationRoot", memory=True)
        exploration_sequence = py_trees.composites.Sequence("ExplorationSequence", memory=True)

        for i, (x, y, yaw) in enumerate(self.end_points):
            point_selector = py_trees.composites.Selector(f"EndPoint_{i}", memory=True)

            end_point_client = behaviours.ServiceClient(
                name=f"GoToPoint_{i}",
                service_type=GoToPoint,
                service_name="/pathing_end_point",
                x=x,
                y=y,
                yaw=yaw,
                velocity=self.target_velocity,
                reverse=False,
                slow_approach=False,
                approaching_object=False,
            )

            reached_end_point = behaviours.ReachedEndPoint(
                name=f"ReachedEndPoint_{i}",
                x=x,
                y=y,
                yaw=yaw,
                distance_threshold=self.distance_threshold,
                yaw_threshold=self.yaw_threshold,
            )

            end_point_sequence = py_trees.composites.Sequence(f"EndPointSequence_{i}", memory=False)
            end_point_sequence.add_children([
                end_point_client,
                reached_end_point,
            ])

            end_point_check = py_trees.composites.Sequence(f"EndPointCheck_{i}", memory=True)
            end_point_check.add_children([
                end_point_sequence,
            ])

            fallback = py_trees.behaviours.Success(name=f"SkipToNext_{i}")

            point_selector.add_children([
                end_point_check,
                fallback,
            ])
            exploration_sequence.add_child(point_selector)

        exploration_complete = py_trees.behaviours.Success(name="ExplorationComplete")
        root.add_children([
            exploration_sequence,
            exploration_complete,
        ])

        return root

    def tick_tree(self):
        try:
            if self.explore_complete:
                return
            self.tree.tick()
            if self.tree.root.status != py_trees.common.Status.RUNNING:
                self.get_logger().info(f"Exploration phase complete (state: {self.tree.root.status})")
                self.explore_complete = True
        except Exception as e:
            self.get_logger().error(f"Exception during tree tick: {e}")

    def read_workspace(self, filename, skip_header=False):
        workspace_vertices = []
        try:
            with open(filename, "r") as file:
                if skip_header:
                    next(file)
                for line in file:
                    line = line.strip()
                    if line:
                        parts = line.split("\t")
                        if len(parts) == 2:
                            x, y = parts
                            workspace_vertices.append((float(x)/100, float(y)/100))
                            self.get_logger().info(f"Added vertex: ({float(x)/100}, {float(y)/100})")
                        else:
                            self.get_logger().warn("Skipping invalid line: {line}")
            return workspace_vertices
        except FileNotFoundError:
            self.get_logger().warn(f"File {filename} not found")
            return []

def main():
    rclpy.init()
    node = ExploreMaster()
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
