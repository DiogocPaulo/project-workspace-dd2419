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


from project_interfaces.srv import GetDetectedList, JointMove
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time
from std_msgs.msg import Header
from geometry_msgs.msg import Point as GeometryPoint
from project_interfaces.srv import PickObject

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
    def __init__(self, name, x, y, yaw, distance_threshold=0.1, yaw_threshold=0.1):
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
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def broadcast_waypoint(self):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = np.sin(self.target_yaw * 0.5)
        quaternion.w = np.cos(self.target_yaw * 0.5)

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
        yaw_error = math.atan2(math.sin(self.target_yaw - self.current_yaw), math.cos(self.target_yaw - self.current_yaw))

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



def generate_waypoints(map: Map, workspace_vertices, outer_offset, inner_offset):
    outer_vertices = offset_outer_vertices(workspace_vertices, outer_offset)
    inner_vertices = offset_inner_vertices(workspace_vertices, inner_offset)
    offset_vertices = outer_vertices + inner_vertices[::-1]
    waypoints = []
    for i in range(len(offset_vertices) - 1):
        current_x, current_y = offset_vertices[i]
        next_x, next_y = offset_vertices[i + 1]
        distance = np.hypot(next_x - current_x, next_y - current_y)
        resolution = max(1, math.ceil(distance * 1.2))
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
        waypoints[i] = (rounded_x, rounded_y, heading)

    return waypoints

class ExploreMaster(Node):

    def __init__(self):
        super().__init__("explore_master")

        workspace_file = "workspaces/large_workspace.tsv"
        self.workspace_vertices = self.read_workspace(workspace_file, skip_header=True)
        self.workspace_publisher = self.create_publisher(WorkspaceVertices, "/workspace", 10)
        self.waypoints_path_publisher = self.create_publisher(Path, "/waypoints_path", 10)
        self.end_points_broadcaster = TransformBroadcaster(self)

        self.target_velocity = 0.16
        self.resolution = 0.05
        self.map = Map(self.resolution)
        self.map.initialise_grid(self.workspace_vertices)
        self.map.inflate_grid(0.35)
        self.show_waypoints = False
        self.end_points = generate_waypoints(self.map, self.workspace_vertices, 0.35, 1.05)

        self.objects, self.boxes = self.process_map_file("maps/Map_test.txt")

        root = self.create_exploration_tree()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root)
        tree_string = py_trees.display.ascii_tree(root)
        self.get_logger().info(f"Behavior Tree Structure:\n{tree_string}")
        self.tree.setup(timeout=15, node=self)

        self.create_timer(0.1, self.tick_tree) # Tick tree every 100 ms
        self.create_timer(2, self.publish_workspace)
        self.create_timer(1, self.publish_objects)
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
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = np.sin(point[2] * 0.5)
            quaternion.w = np.cos(point[2] * 0.5)

            transform = TransformStamped()
            transform.header.frame_id = "odom"  # Change to your desired parent frame
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.child_frame_id = f'EndPoint{i}'
            
            # Set translation from end_point coordinates
            transform.transform.translation.x = point[0]
            transform.transform.translation.y = point[1]
            transform.transform.translation.z = 0.0
            
            transform.transform.rotation.x = quaternion.x
            transform.transform.rotation.y = quaternion.y
            transform.transform.rotation.z = quaternion.z
            transform.transform.rotation.w = quaternion.w
            
            self.end_points_broadcaster.sendTransform(transform)




    class Object:
        def __init__(self,type,x,y):
            self.x = x
            self.y = y
            self.type = type

    def process_map_file(self, file_path):
        objects = []
        boxes = []
        with open(file_path, "r") as file:
            for line in file:
                parts = line.strip().split(" ")
                O = self.Object(parts[0],float(parts[1])/100,float(parts[2])/100)
                if O.type == "B" or O.type == "b":
                    boxes.append(O)
                else:
                    objects.append(O)

        return objects,boxes

    def publish_objects(self):
        object_number = 0
        box_number = 0
        for object in self.objects:
            self.publish_transform("Object-"+str(object_number),object.x,object.y,0)
            object_number+=1
        for box in self.boxes:
            self.publish_transform("Box-"+str(box_number),box.x,box.y,0)
            box_number+=1


    def publish_transform(self, name, x, y, theta):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = name

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # Convert yaw to quaternion
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.end_points_broadcaster.sendTransform(t)
        # self.get_logger().info(f"Published transform for {name} at ({x}, {y})")

    def create_rob_coordinates(self, point1, point2, offset):
        direction = np.array(point2) - np.array(point1)
        
        unit_direction = direction / np.linalg.norm(direction)
        
        new_endpoint = np.array(point1) + unit_direction * offset

        rob_x = point2[0] - new_endpoint[0]
        rob_y = point2[1] - new_endpoint[1]
        
        return rob_x, rob_y

    def create_exploration_tree(self):
        root = py_trees.composites.Selector("ExplorationRoot", memory=True)
        exploration_sequence = py_trees.composites.Sequence("Exploration", memory=True)

        prev_rob_x = 0
        prev_rob_y = 0

        objects_copy = self.objects.copy()

        i = 0

        while len(objects_copy)>0:
            ################# Pick Up Phase ##################

            # Picks the closest object
            distance = 100000
            closest = None
            O_i = 0
            for iter, O in enumerate(objects_copy):
                distance_O = np.linalg.norm(np.array([O.x,O.y]) - np.array([prev_rob_x,prev_rob_y]))
                if distance_O < distance:
                    closest = O
                    distance = distance_O
                    O_i = iter

            rob_x,rob_y = self.create_rob_coordinates((prev_rob_x,prev_rob_y),(closest.x,closest.y),0.2)
            x = closest.x
            y = closest.y
            yaw = 0.0

            point_selector = py_trees.composites.Selector(f"EndPoint{i}", memory=True)

            service_check_sequence = py_trees.composites.Sequence(f"ServiceCheck{i}", memory=True)

            pathing_service = ServiceClient(
                name=f"GoToPoint{i}",
                service_type=GoToPoint,
                service_name="/pathing_end_point",
                x=rob_x,
                y=rob_y,
                yaw=yaw,
                velocity=self.target_velocity,
                reverse=False,
                slow_approach=False,
                approaching_object=False,
            )

            retry_on_endpoint_failure = py_trees.composites.Sequence(f"RetryOnEndpointFailure{i}", memory=False)
            
            end_point_check = ReachedEndPoint(
                name=f"ReachedEndPoint{i}",
                x=rob_x,
                y=rob_y,
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
            # Collection_sequence.add_child(look_service)
            # Collection_sequence.add_child(adjust_service)
            # Collection_sequence.add_child(pick_service)

            objects_copy.pop(O_i)

            prev_rob_x = rob_x
            prev_rob_y = rob_y

            i = i + 1

            # ################# Drop Off Phase ##################

            # Picks the closest box
            distance = 100000
            closest = None
            O_i = 0
            for iter, O in enumerate(self.boxes):
                distance_O = np.linalg.norm(np.array([O.x,O.y]) - np.array([prev_rob_x,prev_rob_y]))
                if distance_O < distance:
                    closest = O
                    distance = distance_O
                    O_i = iter


            rob_x,rob_y = self.create_rob_coordinates((prev_rob_x,prev_rob_y),(closest.x,closest.y),0.2)
            x = closest.x
            y = closest.y
            yaw = 0.0

            point_selector = py_trees.composites.Selector(f"EndPoint{i}", memory=True)

            service_check_sequence = py_trees.composites.Sequence(f"ServiceCheck{i}", memory=True)

            pathing_service = ServiceClient(
                name=f"GoToPoint{i}",
                service_type=GoToPoint,
                service_name="/pathing_end_point",
                x=rob_x,
                y=rob_y,
                yaw=yaw,
                velocity=self.target_velocity,
                reverse=False,
                slow_approach=False,
                approaching_object=False,
            )

            retry_on_endpoint_failure = py_trees.composites.Sequence(f"RetryOnEndpointFailure{i}", memory=False)
            
            end_point_check = ReachedEndPoint(
                name=f"ReachedEndPoint{i}",
                x=rob_x,
                y=rob_y,
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

            prev_rob_x = rob_x
            prev_rob_y = rob_y

            # Collection_sequence.add_child(look_service)
            # Collection_sequence.add_child(adjust_service)
            # Collection_sequence.add_child(drop_service)

            i = i + 1 


        # for i, (x, y, yaw) in enumerate(self.end_points):
        #     point_selector = py_trees.composites.Selector(f"EndPoint{i}", memory=True)

        #     service_check_sequence = py_trees.composites.Sequence(f"ServiceCheck{i}", memory=True)

        #     pathing_service = ServiceClient(
        #         name=f"GoToPoint{i}",
        #         service_type=GoToPoint,
        #         service_name="/pathing_end_point",
        #         x=x,
        #         y=y,
        #         yaw=yaw,
        #         velocity=self.target_velocity,
        #         reverse=False,
        #         slow_approach=False,
        #         approaching_object=False,
        #     )

        #     retry_on_endpoint_failure = py_trees.composites.Sequence(f"RetryOnEndpointFailure{i}", memory=False)
            
        #     end_point_check = ReachedEndPoint(
        #         name=f"ReachedEndPoint{i}",
        #         x=x,
        #         y=y,
        #         yaw=yaw,
        #     )

        #     retry_endpoint = py_trees.decorators.FailureIsRunning(
        #         name=f"RetryEndpoint{i}",
        #         child=end_point_check,
        #     )

            # retry_on_endpoint_failure.add_children([pathing_service, retry_endpoint])
            # service_check_sequence.add_child(retry_on_endpoint_failure)

            # fallback = py_trees.behaviours.Success(name=f"SkipToNext{i}")

            # point_selector.add_children([service_check_sequence, fallback])
            # exploration_sequence.add_child(point_selector)

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
        





class Look(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, x, y, t, **kwargs):
        super().__init__(name)
        self.x = x
        self.y = y
        self.type = t
        self.node = None
        self.pickup_client = None
        self.camera_client = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False


        self.pickup_client = self.node.create_client(PickObject, 'PickObject')
        while not self.pickup_client.wait_for_service(timeout_sec=1.0):
            self.node.logger.info("Arm Request service not yet avaliable, waiting ...")

        self.camera_client = self.node.create_client(GetDetectedList, 'get_detected_list')
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

        self.clock = self.node.get_clock()

        self.future = None


        return True

    def initialise(self):
        self.stage = 0
        self.future = None

    def update(self):
        if self.stage == 0:
            try:
                request = PickObject.Request()

                request.header = Header()
                request.header.stamp = self.node.get_clock().now().to_msg()
                request.header.frame_id = "arm_base"
                request.point = GeometryPoint()
                request.point.x = self.x
                request.point.y = self.y
                request.point.z = 0.0
                request.description = "LOOK"

                self.future = self.pickup_client.call_async(request)
                self.stage = 1
                self.node.get_logger().info(f"{self.name} - Sent request to Look")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING

        elif self.stage == 1:
            if self.future.done():
                response = self.future.result()
                if response.result == 0:
                    self.stage = 2

            return py_trees.common.Status.RUNNING
                
        elif self.stage == 2:
            request = GetDetectedList.Request()

            self.future = self.camera_client.call_async(request)
            self.stage = 3

            return py_trees.common.Status.RUNNING

        elif self.stage == 3:
            if self.future.done():
                response = self.future.result()
                
                if self.type == "objects":
                    if len(response.objects) > 0:
                        return py_trees.common.Status.SUCCESS
                    else:
                        return py_trees.common.Status.FAILURE
                elif self.type == "boxes":
                    if len(response.boxes) > 0:
                        return py_trees.common.Status.SUCCESS
                    else:
                        return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING
        

class Adjust(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, t, **kwargs):
        super().__init__(name)
        self.type = t
        self.node = None
        self.camera_client = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0
        self.future = None

        self.base = 12000
        self.v1 = 12000
        self.v2 = 12000
        self.v3 = 12000
        self.off_base = 0.14

        self.eps = 5
        self.step_size = 200

        if self.type == "boxes":
            self.eps = 15

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False


        self.camera_client = self.node.create_client(GetDetectedList, 'get_detected_list')
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

        self.pos_subscriber = self.node.create_subscription(
            JointState, '/servo_pos_publisher', self.pos_callback, 10)

        self.joint_publisher = self.node.create_publisher(Int16MultiArray, "/multi_servo_cmd_sub", 10)

        self.clock = self.node.get_clock()


        return True

    def initialise(self):
        self.stage = 0
        self.future = None

    def pos_callback(self,msg):
        self.base = msg.position[5]
        self.v1 = msg.position[4]
        self.v2 = msg.position[3]
        self.v3 = msg.position[2]

    def update(self):
        if self.stage == 0:
            # self.node.get_logger().info(f"Stage: 0")
            request = GetDetectedList.Request()

            self.future = self.camera_client.call_async(request)
            self.stage = 1

            return py_trees.common.Status.RUNNING

        elif self.stage == 1:
            # self.node.get_logger().info(f"Stage: 1")
            if self.future.done():
                response = self.future.result()
                
                objects = response.objects
                boxes = response.boxes

                closest_obj = None
                if self.type == 'objects':
                    closest_obj = min(objects, key=lambda DetectedData: DetectedData.distance)
                elif self.type == 'boxes':
                    closest_obj = min(boxes, key=lambda DetectedData: DetectedData.distance)



                #Check if within threshhold:
                if closest_obj.distance <= self.eps:
                    return py_trees.common.Status.SUCCESS

                base = self.base
                v3 = self.v3

                if closest_obj.distance < 10:
                    self.step_size = 100
                #difference in x-axis
                if(closest_obj.diff_x > 0):
                    base += self.step_size
                elif(closest_obj.diff_x < 0):
                    base -= self.step_size

                #difference in y-axis
                if(closest_obj.diff_y > 0):
                    v3 += self.step_size
                elif(closest_obj.diff_y < 0):
                    v3 -= self.step_size

                if not (base < 23900 and base > 100):
                    base = self.base

                if  not (v3 < 20900 and v3 > 3100):
                    v3 = self.v3

                move_command = JointMove.Request()
                move_command.base = int(base)
                move_command.v1 = int(self.v1)
                move_command.v2 = int(self.v2)
                move_command.v3 = int(v3)
                
                msg = Int16MultiArray()
                msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
                move_time = 100
                pose = [11000,12000,int(v3),int(self.v2),int(self.v1),int(base),move_time,move_time,move_time,move_time,move_time,move_time]
                msg.data = pose
                self.start_time = time.time()
                self.joint_publisher.publish(msg)

                self.stage = 2

            return py_trees.common.Status.RUNNING

        elif self.stage == 2:
            # self.node.get_logger().info(f"Stage: 2")
            if time.time() - self.start_time >= 0.12:
                self.stage = 0

            return py_trees.common.Status.RUNNING

class Sweep(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, t, **kwargs):
        super().__init__(name)
        self.type = t
        self.node = None
        self.camera_client = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0
        self.future = None
        self.wait = 2000

        self.counter = 0


        self.positions = [
            (3000, 5500, 1500, 2.0),  # base, neck, move_time, wait
            (3000, 3000, 1000, 1.5),
            (12000, 5500, 1500, 2.0),
            (12000, 3000, 1000, 1.5),
            (21000, 5500, 1500, 2.0),
            (21000, 3000, 1000, 1.5),
        ]

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False


        self.camera_client = self.node.create_client(GetDetectedList, 'get_detected_list')
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

        self.joint_publisher = self.node.create_publisher(Int16MultiArray, "/multi_servo_cmd_sub", 10)

        self.clock = self.node.get_clock()


        return True

    def initialise(self):
        self.stage = 0
        self.future = None

    def update(self):
        if self.stage == 0:
            # self.node.get_logger().info(f"Stage: 0")

            if self.counter < len(self.positions):
                base, neck, move_time, self.wait = self.positions[self.counter]
                pose = [11000,12000,neck,21000,12000,base,move_time,move_time,move_time,move_time,move_time,move_time]
                msg.data = pose
                self.arm_pub.publish(msg)
            
            msg = Int16MultiArray()
            msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
            pose = [11000,12000,neck,21000,12000,base,move_time,move_time,move_time,move_time,move_time,move_time]
            msg.data = pose


            self.stage = 1

            self.start_time = time.time()

            return py_trees.common.Status.RUNNING

        elif self.stage == 1:
            # self.node.get_logger().info(f"Stage: 2")
            if time.time() - self.start_time >= self.wait:
                self.stage = 2

            return py_trees.common.Status.RUNNING

        elif self.stage == 2:
            # self.node.get_logger().info(f"Stage: 0")
            request = GetDetectedList.Request()

            self.future = self.camera_client.call_async(request)

            self.stage = 3

            return py_trees.common.Status.RUNNING

        elif self.stage == 3:
            # self.node.get_logger().info(f"Stage: 1")
            if self.future.done():
                response = self.future.result()
                
                objects = response.objects
                boxes = response.boxes

                if len(objects) > 0:
                    return py_trees.common.Status.SUCCESS

                if self.counter >= len(self.positions):
                    return py_trees.common.Status.FAILURE

                self.counter += 1

                self.stage = 0

            return py_trees.common.Status.RUNNING


class Check(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, t, **kwargs):
        super().__init__(name)
        self.type = t
        self.node = None
        self.camera_client = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0
        self.future = None
        self.counter = 0
        self.counter_max = 10


    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.node.logger.error(f"{self.name} - Setup failed: {e}")
            return False


        self.camera_client = self.node.create_client(GetDetectedList, 'get_detected_list')
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')


        return True

    def initialise(self):
        self.stage = 0
        self.future = None

    def update(self):
        if self.stage == 0:
            # self.node.get_logger().info(f"Stage: 0")
            request = GetDetectedList.Request()

            self.future = self.camera_client.call_async(request)
            self.stage = 1

            return py_trees.common.Status.RUNNING

        elif self.stage == 1:
            # self.node.get_logger().info(f"Stage: 1")
            if self.future.done():
                response = self.future.result()
                
                objects = response.objects
                boxes = response.boxes

                if len(objects)>0:
                    return py_trees.common.Status.SUCCESS

                self.counter += 1

                if self.counter > self.counter_max:
                    self.counter = 0
                    return py_trees.common.Status.FAILURE
                else:
                    self.stage = 0

            return py_trees.common.Status.RUNNING
        

class Pick(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, t, task, **kwargs):
        super().__init__(name)
        self.type = t
        self.node = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0
        self.future = None
        self.x = None
        self.y = None
        self.task = task

        self.base = 12000
        self.v1 = 12000
        self.v2 = 12000
        self.v3 = 12000
        self.off_base = 0.14


    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.node.get_logger.error(f"{self.name} - Setup failed: {e}")
            return False

        self.clock = self.node.get_clock()

        self.pickup_client = self.node.create_client(PickObject, 'PickObject')
        while not self.pickup_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

        self.pos_subscriber = self.node.create_subscription(
            JointState, '/servo_pos_publisher', self.pos_callback, 10)


        return True

    def initialise(self):
        self.stage = 0
        self.future = None

    def pos_callback(self,msg):
        self.base = msg.position[5]
        self.v1 = msg.position[4]
        self.v2 = msg.position[3]
        self.v3 = msg.position[2]

    def update(self):

        # First two stages is to pass some time to allow the correct joint readings to be read
        if self.stage == 0:
            self.node.get_logger().info(f"Stage: 0")
            self.start_time = time.time()

            self.stage = 1
            return py_trees.common.Status.RUNNING


        elif self.stage == 1:
            self.node.get_logger().info(f"Stage: 1")
            if time.time() - self.start_time >= 0.5:
                self.stage = 2

            return py_trees.common.Status.RUNNING

        # Stage 2 estimates the position of the target given where the arm is pointing
        elif self.stage == 2:
            self.node.get_logger().info(f"Stage: 2")
            l1 = 0.101
            l2 = 0.095
            base = math.radians((12000 - self.base) / 100)
            alpha = math.radians((12000 - self.v1) / 100)
            beta = math.radians((12000 + self.v2) / 100)
            charlie = math.radians((12000 - (self.v3 - 50)) / 100)

            distance_y = l1 + self.off_base #math.sin(alpha)*l1 + math.sin(beta)*l2

            distance = l2 + math.tan((math.pi/2)-charlie)*distance_y + 0.085  #math.cos(alpha)*l1 + math.cos(beta)*l2

            if distance < 0.25: distance -= 0.01

            distance_z = 0 - self.off_base

            distance_y = math.sin(-base)*distance
            distance_x = math.cos(base)*distance

            self.x = distance_x
            self.y = distance_y

            self.stage = 3

            return py_trees.common.Status.RUNNING

        # Pick upp target
        elif self.stage == 3:
            self.node.get_logger().info(f"Stage: 3")
            try:
                request = PickObject.Request()

                request.header = Header()
                request.header.stamp = self.node.get_clock().now().to_msg()
                request.header.frame_id = "arm_base"
                request.point = GeometryPoint()
                request.point.x = self.x
                request.point.y = self.y
                request.point.z = -0.15
                request.description = self.task

                self.future = self.pickup_client.call_async(request)
                self.stage = 4
                self.node.get_logger().info(f"{self.name} - Sent request to Pickup")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE


        elif self.stage == 4:
            self.node.get_logger().info(f"Stage: 4")
            if self.future.done():
                response = self.future.result()
                if response.result == 0:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING
        
class Drop(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, t, task, **kwargs):
        super().__init__(name)
        self.type = t
        self.node = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0
        self.future = None
        self.x = None
        self.y = None
        self.task = task

        self.base = 12000
        self.v1 = 12000
        self.v2 = 12000
        self.v3 = 12000
        self.off_base = 0.14


    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.node.get_logger.error(f"{self.name} - Setup failed: {e}")
            return False

        self.clock = self.node.get_clock()

        self.pickup_client = self.node.create_client(PickObject, 'PickObject')
        while not self.pickup_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

        self.pos_subscriber = self.node.create_subscription(
            JointState, '/servo_pos_publisher', self.pos_callback, 10)


        return True

    def initialise(self):
        self.stage = 0
        self.future = None

    def pos_callback(self,msg):
        self.base = msg.position[5]
        self.v1 = msg.position[4]
        self.v2 = msg.position[3]
        self.v3 = msg.position[2]

    def update(self):

        # First two stages is to pass some time to allow the correct joint readings to be read
        if self.stage == 0:
            self.node.get_logger().info(f"Stage: 0")
            self.start_time = time.time()

            self.stage = 1
            return py_trees.common.Status.RUNNING


        elif self.stage == 1:
            self.node.get_logger().info(f"Stage: 1")
            if time.time() - self.start_time >= 0.5:
                self.stage = 2

            return py_trees.common.Status.RUNNING

        # Stage 2 estimates the position of the target given where the arm is pointing
        elif self.stage == 2:
            self.node.get_logger().info(f"Stage: 2")
            l1 = 0.101
            l2 = 0.095
            base = math.radians((12000 - self.base) / 100)
            alpha = math.radians((12000 - self.v1) / 100)
            beta = math.radians((12000 + self.v2) / 100)
            charlie = math.radians((12000 - (self.v3 - 50)) / 100)

            distance_y = l1 + self.off_base #math.sin(alpha)*l1 + math.sin(beta)*l2

            distance = l2 + math.tan((math.pi/2)-charlie)*distance_y + 0.085  #math.cos(alpha)*l1 + math.cos(beta)*l2

            if distance < 0.25: distance -= 0.03

            distance_z = 0 - self.off_base

            distance_y = math.sin(-base)*distance
            distance_x = math.cos(base)*distance

            self.x = distance_x
            self.y = distance_y

            self.stage = 3

            return py_trees.common.Status.RUNNING

        # Pick upp target
        elif self.stage == 3:
            self.node.get_logger().info(f"Stage: 3")
            try:
                request = PickObject.Request()

                request.header = Header()
                request.header.stamp = self.node.get_clock().now().to_msg()
                request.header.frame_id = "arm_base"
                request.point = GeometryPoint()
                request.point.x = self.x
                request.point.y = self.y
                request.point.z = 0.0
                request.description = self.task

                self.future = self.pickup_client.call_async(request)
                self.stage = 4
                self.node.get_logger().info(f"{self.name} - Sent request to Pickup")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE


        elif self.stage == 4:
            self.node.get_logger().info(f"Stage: 4")
            if self.future.done():
                response = self.future.result()
                if response.result == 0:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING

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
