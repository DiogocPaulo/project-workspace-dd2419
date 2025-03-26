#!/usr/bin/env python

import numpy as np
import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from project_interfaces.srv import GoToPoint, Trigger
from nav_msgs.msg import Odometry
from project_interfaces.msg import Point, Workspace
from navigation.map import WorkspaceArea

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

def offset_workspace_vertices(workspace_vertices, offset_distance):
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

    # Compute and add offset midpoints for each edge.
    for i in range(num_vertices):
        prev = vertices[i - 1]
        current = vertices[i]
        nxt = vertices[(i + 1) % num_vertices]

        normal1 = compute_offset_normal(prev, current)
        normal2 = compute_offset_normal(current, nxt)

        p1 = current + normal1 * offset_distance
        d1 = current - prev  # direction of the incoming edge
        p2 = current + normal2 * offset_distance
        d2 = nxt - current   # direction of the outgoing edge

        denom = np.cross(d1, d2)
        if np.abs(denom) < 1e-6:
            offset_vertex = current + normal1 * offset_distance
        else:
            t = np.cross((p2 - p1), d2) / denom
            offset_vertex = p1 + t * d1

        offset_vertices.append((offset_vertex[0], offset_vertex[1], 0.0))

    return offset_vertices

def generate_waypoints(workspace_vertices, resolution):

    workspace = WorkspaceArea(workspace_vertices)

    x_min = min(vertex[0] for vertex in workspace_vertices)
    x_max = max(vertex[0] for vertex in workspace_vertices)
    y_min = min(vertex[1] for vertex in workspace_vertices)
    y_max = max(vertex[1] for vertex in workspace_vertices)

    waypoints = []

    x = x_min + (resolution / 2)
    while x < x_max:
        y = y_min + (resolution / 2)
        while y < y_max:
            if workspace.is_within_workspace(x, y):
                waypoints.append((x, y, 0.0))
            y += resolution
        x += resolution
    
    return waypoints

class ExploreMaster(Node):

    def __init__(self):
        super().__init__("explore_master")

        workspace_file = "workspaces/small_workspace.tsv"
        self.workspace_vertices = self.read_workspace(workspace_file, skip_header=True)
        self.workspace_publisher = self.create_publisher(Workspace, "/workspace", 10)
        self.end_points_broadcaster = TransformBroadcaster(self)

        self.to_broadcast_waypoints = False
        # self.end_points = generate_waypoints(offset_workspace_vertices(self.workspace_vertices, 0.4), 0.4)
        self.end_points = offset_workspace_vertices(self.workspace_vertices, 0.6)

        root = self.create_exploration_tree()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root)
        tree_string = py_trees.display.ascii_tree(root)
        self.get_logger().info(f"Behavior Tree Structure:\n{tree_string}")
        self.tree.setup(timeout=15, node=self)

        self.create_timer(0.1, self.tick_tree) # Tick tree every 100 ms
        self.create_timer(2, self.publish_workspace)
        if self.to_broadcast_waypoints:
            self.create_timer(2, self.broadcast_waypoints)

    def publish_workspace(self):
        workspace_msg = Workspace()
        workspace_msg.header.stamp = self.get_clock().now().to_msg()
        workspace_msg.header.frame_id = "map"

        for vertex in self.workspace_vertices:
            point_msg = Point()
            point_msg.x = vertex[0]
            point_msg.y = vertex[1]
            workspace_msg.points.append(point_msg)

        self.workspace_publisher.publish(workspace_msg)
        self.get_logger().info("Published workspace vertices", once=True)

    def broadcast_waypoints(self):
        for i, point in enumerate(self.end_points):
            transform = TransformStamped()
            transform.header.frame_id = 'map'  # Change to your desired parent frame
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.child_frame_id = f'EndPoint{i}'
            
            # Set translation from end_point coordinates
            transform.transform.translation.x = point[0]
            transform.transform.translation.y = point[1]
            transform.transform.translation.z = 0.0
            
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            
            self.end_points_broadcaster.sendTransform(transform)

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
