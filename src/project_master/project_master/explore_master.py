#!/usr/bin/env python

import numpy as np
import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from project_interfaces.srv import GoToPoint, Trigger
from nav_msgs.msg import Odometry

from project_master import behaviours

class ServiceClient(py_trees.behaviour.Behaviour):
    def __init__(self, name, service_type, service_name, **kwargs):
        super().__init__(name)
        self.service_type = service_type
        self.service_name = service_name
        self.request_args = kwargs
        self.client = None
        self.future = None
        self.sent_request = False

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False

        self.client = self.node.create_client(self.service_type, self.service_name)
        return True

    def initialise(self):
        self.sent_request = False
        self.future = None

    def update(self):
        if not self.client.service_is_ready():
            self.node.get_logger().info(f"{self.name} - Waiting for service {self.service_name} ...")
            return py_trees.common.Status.RUNNING
            
        if not self.sent_request:
            try:
                request = self.service_type.Request()

                for key, value in self.request_args.items():
                    setattr(request, key, value)

                self.future = self.client.call_async(request)
                self.sent_request = True
                self.node.get_logger().info(f"{self.name} - Sent request to {self.service_name}")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE

        if self.future.done():
            try:
                response = self.future.result()
                self.node.get_logger().info(f"{self.name} - Service call response: {response.success}, {response.message}")
                return py_trees.common.Status.SUCCESS
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Service call failed with exception: {e}")
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

class ReachedEndPoint(py_trees.behaviour.Behaviour):
    """
    A behaviour that checks is an end point has been reached
    """
    def __init__(self, name, x, y, tolerance=0.2):
        super().__init__(name)
        self.current_point = (None, None)
        self.end_point = (x, y)
        self.tolerance = tolerance

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.node.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        return True

    def odom_callback(self, msg: Odometry):
        self.current_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def update(self):
        if self.current_point == (None, None):
            self.node.get_logger().info(f"{self.name}: Waiting for current point ...")
            return py_trees.common.Status.RUNNING
        
        distance = np.hypot(
            self.current_point[0] - self.end_point[0],
            self.current_point[1] - self.end_point[1]
        )

        if distance <= self.tolerance:
            self.node.get_logger().info(f"{self.name}: Reached end point of ({self.end_point[0], self.end_point[1]})")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f"{self.name}: Distance to end point is {distance:.2f}")
            return py_trees.common.Status.RUNNING

def create_offset_end_points(workspace_vertices, offset_distance=0.5):

    end_points = []
    vertices = [np.array(vertex) for vertex in workspace_vertices]

    for i in range(len(vertices)):
        current = vertices[i]
        next = vertices[(i+1) % len(vertices)]

        edge_vector = next - current
        midpoint = (current + next) / 2

        perp_vector = np.array([-edge_vector[1], edge_vector[0]])
        normal_vector = perp_vector / np.linalg.norm(perp_vector)
        
        cross_product = np.cross(edge_vector, perp_vector)
        if cross_product < 0:
            normal_vector = -normal_vector
        
        offset_point = midpoint + (normal_vector * offset_distance)
        
        # Add to waypoints as a tuple
        end_points.append((offset_point[0], offset_point[1], 0.0))

    return end_points

class ExploreMaster(Node):

    def __init__(self):
        super().__init__("explore_master")

        self.workspace_vertices = [
            (-2.20, -1.30),
            (2.20, -1.30),
            (2.20, 1.30),
            (-2.20, 1.30),
        ]
        
        self.end_points = create_offset_end_points(self.workspace_vertices, 0.5)
        root = self.create_exploration_tree()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root)
        tree_string = py_trees.display.ascii_tree(root)
        self.get_logger().info(f"Behavior Tree Structure:\n{tree_string}")
        self.tree.setup(timeout=15, node=self)

        self.create_timer(0.1, self.tick_tree) # Tick tree every 100 ms

    def create_exploration_tree(self):
        root = py_trees.composites.Selector("ExplorationRoot", memory=True)
        exploration_sequence = py_trees.composites.Sequence("Exploration", memory=True)

        for i, (x, y, yaw) in enumerate(self.end_points):
            point_sequence = py_trees.composites.Sequence(f"EndPoint{i}", memory=True)

            pathing_service = ServiceClient(
                name=f"GoToPoint{i}",
                service_type=GoToPoint,
                service_name="/pathing_end_point",
                x=x,
                y=y,
                yaw=yaw
            )
            
            end_point_check = ReachedEndPoint(
                name=f"ReachedEndPoint{i}",
                x=x,
                y=y
            )
            
            point_sequence.add_children([pathing_service, end_point_check])
            exploration_sequence.add_child(point_sequence)

        repeater = py_trees.decorators.FailureIsRunning(
            name="RepeatExploration",
            child=py_trees.decorators.Repeat(
                name="RepeatForever", 
                child=exploration_sequence,
                num_success=-1  # -1 means infinite repetition
            )
        )
        
        root.add_child(repeater)
        return root

    def tick_tree(self):
        try:
            self.tree.tick()
        except Exception as e:
            self.get_logger().error(f"Exception during tree tick: {e}")


def main():
    rclpy.init()
    node = ExploreMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
