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

        # Constants
        self.distance_threshold = 0.1
        self.yaw_threshold = math.radians(8)
        self.resolution = 0.05
        workspace_file = "workspaces/large_workspace.tsv"

        # Variables
        self.workspace_vertices = self.read_workspace(workspace_file, skip_header=True)

        root = self.create_collection_tree()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root)
        tree_string = py_trees.display.ascii_tree(root)
        self.get_logger().info(f"Behavior Tree Structure:\n{tree_string}")
        self.tree.setup(timeout=15, node=self)

        self.create_timer(0.1, self.tick_tree) # Tick tree every 100 ms
        self.create_timer(2, self.publish_workspace)

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

    def create_collection_tree(self):
        root = py_trees.composites.Selector("CollectionRoot", memory=True)
        collection_sequence = py_trees.composites.Sequence("Collection", memory=True)

        find_closest_object = behaviours.FindClosestObject(
            name="FindClosest_Object",
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
            distance_threshold=self.distance_threshold,
            yaw_threshold=self.yaw_threshold,
        )
        object_safe_point_sequence = py_trees.composites.Sequence(
            name="SafePointSequence_Object",
            memory=False,
        )
        object_safe_point_sequence.add_children([
            object_safe_point_client,
            object_reached_safe_point,
        ])

        # Object Approach Point
        object_approach_point_client = behaviours.GoToApproachPointClient(
            name="ApproachPointClient_Object",
            service_name="/pathing_end_point",
            approach_offset=0.20,
            input_key="closest_object",
            output_key="object_approach_waypoint",
        )
        object_reached_approach_point = behaviours.ReachedWaypoint(
            name="ReacheApproachPoint_Object",
            input_key="object_approach_waypoint",
            distance_threshold=self.distance_threshold,
            yaw_threshold=self.yaw_threshold,
        )
        object_approach_point_sequence = py_trees.composites.Sequence(
            name="ApproachPointSequence_Object",
            memory=False,
        )
        object_approach_point_sequence.add_children([
            object_approach_point_client,
            object_reached_approach_point,
        ])

        # Pickup Routine
        object_look = behaviours.Look(
            name=f"Look_Object",
            x=0.2,
            y=0.0,
            t = "objects"
        )
        object_sweep = behaviours.Sweep(
            name=f"Sweep_Object",
            t = "objects"
        )
        object_adjust = behaviours.Adjust(
            name = f"Adjust_Object",
            t = "objects"
        )
        pickup = behaviours.Pick(
            name = f"Pickup",
            t = "objects",
            task = "PICKUP"
        )
        object_check = behaviours.Check(
            name = f"Check_Object",
            t = "objects"
        )
        object_return = behaviours.Return(
            name = f"Return_Object",
            t = "objects",
            task = "RETURN"
        )

        object_look_fallback = py_trees.composites.Selector(f"LookFallback_Object", memory=True)
        object_look_fallback.add_children([
            object_look,
            object_sweep,
        ])

        pickup_routine = py_trees.composites.Sequence(f"PickupRoutine", memory=True)
        pickup_routine.add_children([
            object_look_fallback,
            object_adjust,
            pickup,
            object_check,
        ])

        retry_pickup = py_trees.decorators.Retry(name="RetryPickup", child=pickup_routine, num_failures=1)

        return_after_failed_pickup_fallback = py_trees.composites.Selector("ReturnAfterFailedPickupFallback", memory=True)
        return_after_failed_pickup_fallback.add_children([
            retry_pickup,
        ])

        # Object Reposition Point
        object_reposition_point_client = behaviours.GoToRepositionPointClient(
            name="RepositionPointClient_Object",
            service_name="/pathing_end_point",
            reposition_offset=0.20,
            input_key="object_position",
            output_key="object_reposition_waypoint",
        )

        object_reached_reposition_point = behaviours.ReachedWaypoint(
            name="ReachedRepositionPoint_Object",
            input_key="object_reposition_waypoint",
            distance_threshold=self.distance_threshold,
            yaw_threshold=self.yaw_threshold,
        )

        object_reposition_point_sequence = py_trees.composites.Sequence(
            name="RepositionPointSequence_Object",
            memory=False,
        )

        object_reposition_point_sequence.add_children([
            object_reposition_point_client,
            object_reached_reposition_point,
        ])

        # Reposition Pickup Routine
        reposition_object_look = behaviours.Look(
            name=f"Look_Object",
            x=0.2,
            y=0.0,
            t = "objects"
        )
        reposition_object_sweep = behaviours.Sweep(
            name=f"Sweep_Object",
            t = "objects"
        )
        reposition_object_adjust = behaviours.Adjust(
            name = f"Adjust_Object",
            t = "objects"
        )
        reposition_pickup = behaviours.Pick(
            name = f"Pickup",
            t = "objects",
            task = "PICKUP"
        )
        reposition_object_check = behaviours.Check(
            name = f"Check_Object",
            t = "objects"
        )

        reposition_object_look_fallback = py_trees.composites.Selector(f"LookFallback_Object", memory=True)
        reposition_object_look_fallback.add_children([
            reposition_object_look,
            reposition_object_sweep,
        ])
        reposition_pickup_routine = py_trees.composites.Sequence(f" ", memory=True)
        reposition_pickup_routine.add_children([
            reposition_object_look_fallback,
            reposition_object_adjust,
            reposition_pickup,
            reposition_object_check,
        ])
        reposition_retry_pickup = py_trees.decorators.Retry(name="RetryPickup", child=pickup_routine, num_failures=1)

        reposition_return_after_failed_pickup_fallback = py_trees.composites.Selector("ReturnAfterFailedPickupFallback", memory=True)
        reposition_return_after_failed_pickup_fallback.add_children([
            reposition_retry_pickup,
        ])
    
        reposition_and_pickup = py_trees.composites.Sequence("RepositionAndPickup", memory=True)
        reposition_and_pickup.add_children([
            object_reposition_point_sequence,
            reposition_return_after_failed_pickup_fallback,
        ])

        retry_reposition_and_pickup = py_trees.decorators.Retry("RetryRepositionAndPickup", child=reposition_and_pickup, num_failures=1)

        pickup_attempt = py_trees.composites.Selector("PickupAttempt", memory=True)
        pickup_attempt.add_children([
            return_after_failed_pickup_fallback,
            retry_reposition_and_pickup,
        ])

        pickup_sequence = py_trees.composites.Sequence("PickupSequence", memory=True)
        pickup_sequence.add_children([
            object_return,
            find_closest_object,
            object_safe_point_sequence,
            object_approach_point_sequence,
            pickup_attempt,
        ])

        find_closest_box = behaviours.FindClosestObject(
            name="FindClosest_Box",
            find_box=True,
            output_key="closest_box",
        )

        # Box Safe Point
        box_safe_point_client = behaviours.GoToSafePointClient(
            name="SafePointClient_Box",
            service_name="/pathing_end_point",
            input_key="closest_box",
            output_key="box_safe_waypoint",
        )
        box_reached_safe_point = behaviours.ReachedWaypoint(
            name="ReachedSafePoint_Box",
            input_key="box_safe_waypoint",
            distance_threshold=self.distance_threshold,
            yaw_threshold=self.yaw_threshold,
        )
        box_safe_point_sequence = py_trees.composites.Sequence(
            name="SafePointSequence_Box",
            memory=False,
        )
        box_safe_point_sequence.add_children([
            box_safe_point_client,
            box_reached_safe_point,
        ])

        # Box Approach Point
        box_approach_point_client = behaviours.GoToApproachPointClient(
            name="ApproachPointClient_Box",
            service_name="/pathing_end_point",
            approach_offset=0.25,
            input_key="closest_box",
            output_key="box_approach_waypoint",
        )
        box_reached_approach_point = behaviours.ReachedWaypoint(
            name="ReacheApproachPoint_Box",
            input_key="box_approach_waypoint",
            distance_threshold=self.distance_threshold,
            yaw_threshold=self.yaw_threshold,
        )
        box_approach_point_sequence = py_trees.composites.Sequence(
            name="ApproachPointSequence_Box",
            memory=False,
        )
        box_approach_point_sequence.add_children([
            box_approach_point_client,
            box_reached_approach_point,
        ])

        # Drop Routine
        box_look = behaviours.Look(
            name=f"Look_Box",
            x=0.2,
            y=0.0,
            t = "boxes"
        )
        box_sweep = behaviours.Sweep(
            name=f"Sweep_Box",
            t = "boxes"
        )
        box_adjust = behaviours.Adjust(
            name = f"Adjust_Box",
            t = "boxes"
        )
        drop = behaviours.Drop(
            name = f"Drop",
            t = "boxes",
            task = "DROPOFF"
        )

        box_look_fallback = py_trees.composites.Selector(f"LookFallback_Box", memory=True)
        box_look_fallback.add_children([
            box_look,
            box_sweep,
        ])
        drop_routine = py_trees.composites.Sequence(f"DropRoutine", memory=True)
        drop_routine.add_children([
            box_look_fallback,
            box_adjust,
            drop
        ])
        retry_drop = py_trees.decorators.Retry(name="RetryDrop", child=drop_routine, num_failures=1)

        # Object Reposition Point
        box_reposition_point_client = behaviours.GoToRepositionPointClient(
            name="RepositionPointClient_Box",
            service_name="/pathing_end_point",
            reposition_offset=0.20,
            input_key="object_position",
            output_key="box_reposition_waypoint",
        )

        box_reached_reposition_point = behaviours.ReachedWaypoint(
            name="ReachedRepositionPoint_Box",
            input_key="box_reposition_waypoint",
            distance_threshold=self.distance_threshold,
            yaw_threshold=self.yaw_threshold,
        )

        box_reposition_point_sequence = py_trees.composites.Sequence(
            name="RepositionPointSequence_Box",
            memory=False,
        )

        box_reposition_point_sequence.add_children([
            box_reposition_point_client,
            box_reached_reposition_point,
        ])

        # Reposition Pickup Routine
        reposition_box_look = behaviours.Look(
            name=f"Look_Box",
            x=0.2,
            y=0.0,
            t = "boxes"
        )
        reposition_box_sweep = behaviours.Sweep(
            name=f"Sweep_Box",
            t = "boxes"
        )
        reposition_box_adjust = behaviours.Adjust(
            name = f"Adjust_Box",
            t = "boxes"
        )
        reposition_drop = behaviours.Drop(
            name = f"Drop",
            t = "boxes",
            task = "DROPOFF"
        )

        reposition_box_look_fallback = py_trees.composites.Selector(f"LookFallback_Box", memory=True)
        reposition_box_look_fallback.add_children([
            reposition_box_look,
            reposition_box_sweep,
        ])
        reposition_drop_routine = py_trees.composites.Sequence(f"RepositionDropRoutine", memory=True)
        reposition_drop_routine.add_children([
            reposition_box_look_fallback,
            reposition_box_adjust,
            reposition_drop,
        ])
        reposition_retry_drop = py_trees.decorators.Retry(name="RetryDrop", child=pickup_routine, num_failures=1)

        reposition_return_after_failed_drop_fallback = py_trees.composites.Selector("ReturnAfterFailedDropFallback", memory=True)
        reposition_return_after_failed_drop_fallback.add_children([
            reposition_retry_drop,
        ])
    
        reposition_and_drop = py_trees.composites.Sequence("RepositionAndDrop", memory=True)
        reposition_and_drop.add_children([
            box_reposition_point_sequence,
            reposition_return_after_failed_drop_fallback,
        ])

        retry_reposition_and_drop = py_trees.decorators.Retry("RetryRepositionAndDrop", child=reposition_and_drop, num_failures=1)

        drop_attempt = py_trees.composites.Selector("DropAttempt", memory=True)
        drop_attempt.add_children([
            retry_drop,
            retry_reposition_and_drop,
        ])


        drop_sequence = py_trees.composites.Sequence("DropSequence", memory=True)
        drop_sequence.add_children([
            find_closest_box,
            box_safe_point_sequence,
            box_approach_point_sequence,
            drop_attempt,
        ])

        collection_sequence.add_children([
            pickup_sequence,
            drop_sequence,
        ])

        root.add_child(collection_sequence)
        return root

    def tick_tree(self):
        try:
            self.tree.tick()
        except Exception as e:
            self.get_logger().error(f"Exception during tree tick: {e}")

    def read_workspace(self, filename, skip_header=False):
        try:
            workspace_vertices = []
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
                            self.get_logger().warn(f"Skipping invalid line: {line}")
            return workspace_vertices
        except FileNotFoundError:
            self.get_logger().error(f"Workspace file ({filename}) not found!")
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

