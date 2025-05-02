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
from project_interfaces.srv import GoToPoint, Trigger, PickObject, GetDetectedList, JointMove
from nav_msgs.msg import Path, Odometry
from project_interfaces.msg import Vertex, WorkspaceVertices, Object, ObjectList
from mapping.map import Map

from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
from std_msgs.msg import Header
from geometry_msgs.msg import Point as GeometryPoint

from project_master import behaviours

class CollectMaster(Node):
    def __init__(self):
        super().__init__("explore_master")

        self.workspace_publisher = self.create_publisher(WorkspaceVertices, "/workspace", 10)
        self.object_list_publisher = self.create_publisher(ObjectList, "/detected_objects", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Constants
        self.resolution = 0.05

        # Variables
        map_file = "maps/map.csv"
        self.object_list = self.read_map_file(map_file)
        workspace_file = "workspaces/large_workspace.tsv"
        self.workspace_vertices = self.read_workspace(workspace_file, skip_header=True)

        root = self.create_collection_tree()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root)
        tree_string = py_trees.display.ascii_tree(root)
        self.get_logger().info(f"Behavior Tree Structure:\n{tree_string}")
        self.tree.setup(timeout=15, node=self)

        self.update_object_list()
        self.create_timer(0.1, self.tick_tree) # Tick tree every 100 ms
        self.create_timer(2, self.publish_workspace)
        self.create_timer(5, self.update_object_list)

    def update_object_list(self):
        self.publish_object_list()
        self.broadcast_object_list()

    def publish_workspace(self):
        workspace_msg = WorkspaceVertices()
        workspace_msg.header.stamp = self.get_clock().now().to_msg()
        workspace_msg.header.frame_id = "odom"

        for vertex in self.workspace_vertices:
            vertex_msg = Vertex()
            vertex_msg.x = vertex[0]
            vertex_msg.y = vertex[1]
            workspace_msg.vertices.append(vertex_msg)

        workspace_msg.grid_resolution = self.resolution

        self.workspace_publisher.publish(workspace_msg)
        self.get_logger().info("Published workspace vertices", once=True)

    def publish_object_list(self):
        object_list_msg = ObjectList()
        object_list_msg.header.frame_id = "odom"
        object_list_msg.header.stamp = self.get_clock().now().to_msg()
        object_list_msg.length = len(self.object_list)
        object_list_msg.objects = self.object_list
        self.object_list_publisher.publish(object_list_msg)

    def broadcast_object_list(self):
        for i, object_msg in enumerate(self.object_list):
            # Publish object as transform
            angle = object_msg.angle * float(math.pi / 180)
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = np.sin(angle * 0.5)
            quaternion.w = np.cos(angle * 0.5)

            transform_msg = TransformStamped()
            transform_msg.header.stamp = self.get_clock().now().to_msg()
            transform_msg.header.frame_id = "odom"
            transform_msg.child_frame_id = f"{object_msg.object_type}_{i}"
            transform_msg
            transform_msg.transform.translation.x = object_msg.x
            transform_msg.transform.translation.y = object_msg.y
            transform_msg.transform.translation.z = 0.0

            transform_msg.transform.rotation.x = quaternion.x
            transform_msg.transform.rotation.y = quaternion.y
            transform_msg.transform.rotation.z = quaternion.z
            transform_msg.transform.rotation.w = quaternion.w

            self.tf_broadcaster.sendTransform(transform_msg)

    def create_collection_tree(self):
        root = py_trees.composites.Selector("CollectionRoot", memory=True)
        collection_sequence = py_trees.composites.Sequence("Collection", memory=True)

        find_closest_object = behaviours.FindClosestObject(
            name="FindClosestObject",
            object_list=self.object_list.copy(),
            find_box=False,
            output_key="closest_object",
        )

        # Object Safe Point
        object_safe_point_client = behaviours.GoToSafePointClient(
            name="SafePointClient_Object",
            service_name="/pathing_end_point",
            input_key="closest_object",
            output_key="object_safe_waypoint",
        )
        object_reached_safe_point = behaviours.ReachedWaypoint(
            name="ReachedSafePoint_Object",
            input_key="object_safe_waypoint",
        )
        object_safe_point_sequence = py_trees.composites.Sequence(
            name="SafePointSequence_Object",
            memory=False,
        )
        object_safe_point_sequence.add_children([
            object_safe_point_client,
            object_reached_safe_point,
        ])

        pickup_sequence = py_trees.composites.Sequence("PickupSequence", memory=True)
        pickup_sequence.add_children([
            find_closest_object,
            object_safe_point_sequence
        ])

        collection_sequence.add_children([
            pickup_sequence,
        ])

        root.add_child(collection_sequence)
        return root

    def tick_tree(self):
        try:
            self.tree.tick()
        except Exception as e:
            self.get_logger().error(f"Exception during tree tick: {e}")

    def get_closest_object(self, x, y):
        closest_object = None
        min_distance = float("inf")
        for object_msg in self.object_list:
            if obj.object_type != Object.BOX:
                distance = np.hypot(object_msg.x - x, object_msg.y - y)
                if distance < min_distance:
                    min_distance = distance
                    closest_object = object_msg
        return closest_object

    def get_closest_box(self, x, y):
        closest_box = None
        min_distance = float("inf")
        for object_msg in self.object_list:
            if obj.object_type == Object.BOX:
                distance = np.hypot(object_msg.x - x, object_msg.y - y)
                if distance < min_distance:
                    min_distance = distance
                    closest_object = object_msg
        return closest_box

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

    def read_map_file(self, filename):
        try:
            objects = []
            with open(filename, 'r') as file:
                for line in file:
                    parts = line.strip().split(',')
                    if len(parts) < 4:
                        continue
                    
                    object_msg = Object()
                    
                    # Parse object type
                    if parts[0] == "1":
                        object_msg.object_type = Object.CUBE
                    elif parts[0] == "2":
                        object_msg.object_type = Object.SPHERE
                    elif parts[0] == "3":
                        object_msg.object_type = Object.PLUSHIE
                    elif parts[0] == "B":
                        object_msg.object_type = Object.BOX
                    else:
                        continue  # Skip unknown types
                    
                    # Parse coordinates (convert from cm back to meters)
                    object_msg.x = float(parts[1])/100
                    object_msg.y = float(parts[2])/100
                    object_msg.angle = float(parts[3])
                    
                    objects.append(object_msg)
            
            self.get_logger().info(f"Loaded {len(objects)} objects from {filename}")
            return objects
        except FileNotFoundError:
            self.get_logger().warn(f"File {filename} not found")
            return []

def main():
    rclpy.init()
    node = CollectMaster()
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

