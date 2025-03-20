#!/usr/bin/env python

import numpy as np
import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from project_interfaces.srv import GoToPoint, Trigger
from nav_msgs.msg import Odometry
from project_interfaces.msg import Point, Workspace

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
                if response.success:
                    self.node.get_logger().info(f"{self.name} - Service call response: {response.success}, {response.message}")
                    return py_trees.common.Status.SUCCESS
                else:
                    self.node.get_logger().info(f"{self.name} - Service call response: {response.success}, {response.message}")
                    return py_trees.common.Status.FAILURE
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

        workspace_file = "workspaces/large_workspace.tsv"
        self.workspace_vertices = self.read_workspace(workspace_file, skip_header=True)
        self.workspace_publisher = self.create_publisher(Workspace, "/workspace", 10)
        self.publish_workspace()

        self.end_points = [
            (-1.5, 0.8, 0.0),
            (-1.5, -0.8, 0.0),
            (1.5, -0.8, 0.0),
            (1.5, 0.8, 0.0),
        ]
        
        # self.end_points = create_offset_end_points(self.workspace_vertices, 0.5)
        root = self.create_exploration_tree()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root)
        tree_string = py_trees.display.ascii_tree(root)
        self.get_logger().info(f"Behavior Tree Structure:\n{tree_string}")
        self.tree.setup(timeout=15, node=self)

        self.create_timer(0.1, self.tick_tree) # Tick tree every 100 ms

    def publish_workspace(self):
        workspace_msg = Workspace()
        workspace_msg.header.stamp = self.get_clock().now().to_msg()
        workspace_msg.header.frame_id = "map"

        workspace_msg.points = self.workspace_vertices
        self.get_logger().info("Published workspace vertices", once=True)


    def create_exploration_tree(self):
        root = py_trees.composites.Selector("ExplorationRoot", memory=True)
        exploration_sequence = py_trees.composites.Sequence("Exploration", memory=True)

        for i, (x, y, yaw) in enumerate(self.end_points):
            point_selector = py_trees.composites.Selector(f"EndPoint{i}", memory=True)

            service_check_sequence = py_trees.composites.Sequence(f"ServiceCheck{i}", memory=True)

            pathing_service = ServiceClient(
                name=f"GoToPoint{i}",
                service_type=GoToPoint,
                service_name="/pathing_end_point",
                x=x,
                y=y,
                yaw=yaw
            )

            retry_on_endpoint_failure = py_trees.composites.Sequence(f"RetryOnEndpointFailure{i}", memory=False)
            
            end_point_check = ReachedEndPoint(
                name=f"ReachedEndPoint{i}",
                x=x,
                y=y
            )

            retry_endpoint = py_trees.decorators.FailureIsRunning(
                name=f"RetryEndpoint{i}",
                child=end_point_check
            )

            retry_on_endpoint_failure.add_children([pathing_service, retry_endpoint])
            service_check_sequence.add_child(retry_on_endpoint_failure)

            fallback = py_trees.behaviours.Success(name=f"SkipToNext{i}")

            point_selector.add_children([service_check_sequence, fallback])
            exploration_sequence.add_child(point_selector)

        repeater = py_trees.decorators.Repeat(
            name="RepeatExploration", 
            child=exploration_sequence,
            num_success=2
        )
        
        root.add_child(repeater)
        return root

    def tick_tree(self):
        try:
            self.tree.tick()
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

    rclpy.shutdown()

if __name__ == "__main__":
    main()
