#!/usr/bin/env python

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
    def __init__(self, name, x, y, yaw, distance_threshold=0.2, yaw_threshold=0.1):
        super().__init__(name)
        self.current_point = (None, None)
        self.current_yaw = 0.0
        self.end_point = (x, y)
        self.target_yaw = yaw
        self.distance_threshold = distance_threshold
        self.yaw_threshold = yaw_threshold

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

        self.node.waypoint_broadcaster = TransformBroadcaster(self.node)
        self.node.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        return True

    def odom_callback(self, msg: Odometry):
        self.current_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = odom_msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def broadcast_waypoint(self):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(self.current_yaw * 0.5)
        quaternion.w = np.cos(self.current_yaw * 0.5)

        transform_msg = TransformStamped()
        transform_msg.header.frame_id = "odom"
        transform_msg.header.stamp = self.node.get_clock().now().to_msg()
        transform_msg.child_frame_id = "waypoint"

        transform_msg.transform.translation.x = self.end_point[0]
        transform_msg.transform.translation.y = self.end_point[1]
        transform_msg.transform.translation.z = 0.0

        transform_msg.transform.rotation.x = quaternion.x
        transform_msg.transform.rotation.y = quaternion.y
        transform_msg.transform.rotation.z = quaternion.z
        transform_msg.transform.rotation.w = quaternion.w

        self.node.waypoint_broadcaster.sendTransform(transform_msg)

    def update(self):
        if self.current_point == (None, None):
            self.node.get_logger().info(f"{self.name}: Waiting for current point ...")
            return py_trees.common.Status.RUNNING
        
        distance_error = np.hypot(
            self.current_point[0] - self.end_point[0],
            self.current_point[1] - self.end_point[1]
        )
        yaw_error = self.target_yaw - self.current_yaw

        if distance_error > self.distance_threshold:
            self.node.get_logger().info(f"{self.name}: Distance to waypoint is {distance_error:.2f}")
            self.broadcast_waypoint()
            return py_trees.common.Status.RUNNING
        elif yaw_error > self.yaw_threshold:
            self.node.get_logger().info(f"{self.name}: Correcting yaw by {yaw_error:.2f}")
            self.broadcast_waypoint()
            return py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().info(f"{self.name}: Reached waypoint of ({self.end_point[0], self.end_point[1]}) at {self.current_yaw}")
            return py_trees.common.Status.SUCCESS
    
def generate_waypoints_with_map(map: Map, x_resolution, y_resolution):

    x_min = 0.0
    x_max = map.cells_to_distance(map.grid_width)
    y_min = 0.0
    y_max = map.cells_to_distance(map.grid_height)-0.35

    waypoints = []
    reverse = False

    x = x_min
    while x < x_max:
        y = y_min if not reverse else y_max
        first = None
        last = None
        while (not reverse and y < y_max) or (reverse and y > y_min):
            if map.is_free(x, y, 1):
                if first is None:
                    first = (x, y, 0.0)
                last = (x, y, 0.0)
            y += y_resolution if not reverse else -y_resolution
        if first is not None:
            waypoints.append(first)
        if last is not None and first != last:
            waypoints.append(last)
        x += x_resolution
        reverse = not reverse
    
    return waypoints

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
    return offset_vertices



def generate_waypoints(map: Map, workspace_vertices, outer_offset, inner_offset, waypoint_resolution):
    outer_vertices = offset_outer_vertices(workspace_vertices, outer_offset)
    inner_vertices = offset_inner_vertices(workspace_vertices, inner_offset)
    offset_vertices = outer_vertices + inner_vertices[::-1]
    waypoints = []
    for i in range(len(offset_vertices) - 1):
        current_x, current_y = offset_vertices[i]
        next_x, nexy_y = offset_vertices[i + 1]
        distance = np.hypot(next_x - current_x, nexy_y - current_y)
        yaw = np.arctan2(next_y - current_y, next_x - current_x)
        for j in range(waypoint_resolution):
            x = current_x + (next_x - current_x) * (j + 1) / waypoint_resolution
            y = current_y + (nexy_y - current_y) * (j + 1) / waypoint_resolution
            if map.is_free(x, y, 1):
                waypoints.append((x, y, yaw))
    return waypoints

class ExploreMaster(Node):

    def __init__(self):
        super().__init__("explore_master")

        workspace_file = "workspaces/small_workspace.tsv"
        self.workspace_vertices = self.read_workspace(workspace_file, skip_header=True)
        self.workspace_publisher = self.create_publisher(WorkspaceVertices, "/workspace", 10)
        self.waypoints_path_publisher = self.create_publisher(Path, "/waypoints_path", 10)
        self.end_points_broadcaster = TransformBroadcaster(self)

        self.resolution = 0.05
        self.map = Map(self.resolution)
        self.map.initialise_grid(self.workspace_vertices)
        self.map.inflate_grid(0.30)
        self.show_waypoints = False
        # self.end_points = generate_waypoints(self.map, self.workspace_vertices, 0.35, 1.0, 3)
        self.end_points = [
            (2.0, 0.0, math.pi),
            (0.0, 0.0, -math.pi),
        ]

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
        workspace_msg.header.frame_id = "odom"

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
        path_msg.header.frame_id = "odom"

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
            transform = TransformStamped()
            transform.header.frame_id = "odom"  # Change to your desired parent frame
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
                yaw=yaw,
                reverse=False,
            )

            retry_on_endpoint_failure = py_trees.composites.Sequence(f"RetryOnEndpointFailure{i}", memory=False)
            
            end_point_check = ReachedEndPoint(
                name=f"ReachedEndPoint{i}",
                x=x,
                y=y,
                yaw=yaw,
            )

            retry_endpoint = py_trees.decorators.FailureIsRunning(
                name=f"RetryEndpoint{i}",
                child=end_point_check,
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
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
