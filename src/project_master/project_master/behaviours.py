#!/usr/bin/env python

import time
import math
import numpy as np

import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from project_interfaces.srv import GoToPoint, Trigger, PickObject, GetDetectedList, JointMove
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from project_interfaces.msg import Vertex, WorkspaceVertices, Object, ObjectList
from mapping.map import Map

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header
from geometry_msgs.msg import Point as GeometryPoint
from geometry_msgs.msg import PointStamped, TransformStamped, Pose, Quaternion, Vector3

from project_master import behaviours

# a node has executed completely after returning a SUCCESS or FAILURE

# setup - one time constructor
# initialized - run when node was first ticked or execution completed
# update - called every time the node is ticked
class FindClosestObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, find_box, output_key):
        super().__init__(name)
        self.find_box = find_box
        self.object_point_key = output_key
        self.current_point = (0.0, 0.0)
        self.object_list = []
        self.received_objects = False
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=self.object_point_key,
            access=py_trees.common.Access.WRITE
        )

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
        self.node.create_subscription(ObjectList, "/detected_objects", self.objects_callback, qos_profile)
        return True

    def odom_callback(self, msg: Odometry):
        self.current_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def objects_callback(self, msg: ObjectList):
        if self.received_objects:
            return
        self.received_objects = True
        self.object_list = msg.objects

    def update(self):
        if self.current_point == (None, None):
            self.node.get_logger().info(f"{self.name} - Waiting for current point ...")
            return py_trees.common.Status.RUNNING
        if not self.received_objects:
            self.node.get_logger().info(f"{self.name} - Waiting for object list ...")
            return py_trees.common.Status.RUNNING
        if not self.object_list:
            self.node.get_logger().info(f"{self.name} - Object list is empty. No objects to find!")
            return py_trees.common.Status.FAILURE

        closest_object_msg = None
        min_distance = float("inf")
        for object_msg in self.object_list:
            object_type_check = (object_msg.object_type == Object.BOX) if self.find_box else (object_msg.object_type != Object.BOX)
            if object_type_check:
                distance = np.hypot(object_msg.x - self.current_point[0], object_msg.y - self.current_point[1])
                if distance < min_distance:
                    min_distance = distance
                    closest_object_msg = object_msg

        if closest_object_msg is None and self.find_box:
            self.node.get_logger().warning(f"{self.name} - No boxes found in object list")
            return py_trees.common.Status.FAILURE
        if closest_object_msg is None:
            self.node.get_logger().warning(f"{self.name} - No objects found in object list")
            return py_trees.common.Status.FAILURE
        if self.find_box:
            self.node.get_logger().info(f"{self.name} - Closest box found at ({closest_object_msg.x:.2f}, {closest_object_msg.y:.2f}) with (angle: {closest_object_msg.angle:.2f})")
        else:
            self.node.get_logger().info(f"{self.name} - Closest object found (type: {closest_object_msg.object_type}) at ({closest_object_msg.x:.2f}, {closest_object_msg.y:.2f})")
            self.object_list.remove(closest_object_msg)
        object_point = (closest_object_msg.x, closest_object_msg.y)
        self.blackboard.set(self.object_point_key, object_point)
        return py_trees.common.Status.SUCCESS

class ReachedWaypoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, input_key, distance_threshold, yaw_threshold):
        super().__init__(name)
        self.waypoint_key = input_key
        self.distance_threshold = distance_threshold
        self.yaw_threshold = yaw_threshold
        self.current_point = (0.0, 0.0)
        self.current_yaw = 0.0
        self.end_point = None
        self.end_yaw = 0.0
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=self.waypoint_key,
            access=py_trees.common.Access.READ
        )

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

    def initialise(self):
        self.end_point = None
        self.end_yaw = 0.0

    def odom_callback(self, msg: Odometry):
        self.current_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def update(self):
        if self.current_point is None:
            self.node.get_logger().info(f"{self.name} - Waiting for current point ...")
            return py_trees.common.Status.RUNNING
        try:
            waypoint = self.blackboard.get(self.waypoint_key)
        except Exception as e:
            self.logger.error(f"{self.name} - Error reading blackboard: {e}")
            return py_trees.common.Status.FAILURE

        self.end_point = (waypoint[0], waypoint[1])
        self.end_yaw = waypoint[2]

        distance_error = np.hypot(
            self.current_point[0] - self.end_point[0],
            self.current_point[1] - self.end_point[1]
        )
        yaw_error = math.atan2(
            math.sin(self.end_yaw - self.current_yaw),
            math.cos(self.end_yaw - self.current_yaw)
        )

        if distance_error > self.distance_threshold:
            self.node.get_logger().info(f"{self.name} - Distance to waypoint is {distance_error:.2f}")
            return py_trees.common.Status.RUNNING
        elif abs(yaw_error) > self.yaw_threshold:
            self.node.get_logger().info(f"{self.name} - Yaw needs correction by {yaw_error:.2f}")
            return py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().info(f"{self.name} - Reached waypoint of ({self.end_point[0]:.2f}, {self.end_point[1]:.2f}) at {self.current_yaw}")
            return py_trees.common.Status.SUCCESS

class GoToSafePointClient(py_trees.behaviour.Behaviour):
    def __init__(self, name, service_name, input_key, output_key):
        super().__init__(name)
        self.service_name = service_name
        self.object_point_key = input_key
        self.waypoint_key = output_key
        self.current_point = (0.0, 0.0)
        self.current_yaw = 0.0
        self.safe_point = None
        self.safe_yaw = 0.0
        self.previous_object_point = (0.0, 0.0)
        self.inflated_map = None
        self.objects_map = None
        self.objects_inflation_radius = 0.35
        self.client = None
        self.future = None
        self.sent_request = False
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=self.object_point_key,
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=self.waypoint_key,
            access=py_trees.common.Access.WRITE
        )

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

        self.client = self.node.create_client(GoToPoint, self.service_name)
        self.node.create_subscription(OccupancyGrid, "/inflated_map", self.inflated_map_callback, qos_profile)
        self.node.create_subscription(OccupancyGrid, "/objects_map", self.objects_map_callback, qos_profile)
        self.node.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        return True

    def initialise(self):
        self.sent_request = False
        self.future = None

    def odom_callback(self, msg: Odometry):
        self.current_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def inflated_map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        if self.inflated_map is None:
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            self.inflated_map = Map(resolution, origin_x, origin_y, width, height, grid)
        else:
            self.inflated_map.update_grid(grid)

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

    def calculate_safe_point(self, start_point, object_point):
        start_x, start_y = start_point
        object_x, object_y = object_point
        self.safe_point = self.inflated_map.get_best_safe_point(start_x, start_y, object_x, object_y, 3, 75)
        if self.safe_point is None:
            self.safe_point = (0.0, 0.0)
        self.safe_yaw = np.arctan2(object_y - self.safe_point[1], object_x - self.safe_point[0])
        waypoint = (self.safe_point[0], self.safe_point[1], self.safe_yaw)
        self.blackboard.set(self.waypoint_key, waypoint, overwrite=True)

    def update(self):
        if not self.client.service_is_ready():
            self.node.get_logger().info(f"{self.name} - Waiting for service {self.service_name} ...")
            return py_trees.common.Status.RUNNING
        if self.inflated_map is None:
            self.node.get_logger().info(f"{self.name} - Waiting for inflated map ...")
            return py_trees.common.Status.RUNNING
        if self.objects_map is None:
            self.node.get_logger().info(f"{self.name} - Waiting for objects map ...")
            return py_trees.common.Status.RUNNING
        try:
            current_object_point = self.blackboard.get(self.object_point_key)
        except Exception as e:
            self.logger.error(f"{self.name} - Error reading blackboard: {e}")
            return py_trees.common.Status.INVALID

        if self.previous_object_point != current_object_point or self.safe_point is None:
            self.calculate_safe_point(self.current_point, current_object_point)
            self.previous_object_point = current_object_point
        if not self.inflated_map.is_free(self.safe_point[0], self.safe_point[1], 75):
            self.calculate_safe_point(self.current_point, current_object_point)
            self.previous_object_point = current_object_point

        if not self.sent_request:
            try:
                request = GoToPoint.Request()
                request.x = self.safe_point[0]
                request.y = self.safe_point[1]
                request.yaw = self.safe_yaw
                request.velocity = 0.16
                request.reverse = False
                request.slow_approach = False
                request.approaching_object = False

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
                    if not self.objects_map.is_free(self.safe_point[0], self.safe_point[1], 75):
                        self.node.get_logger().info(f"{self.name} - Safe point within objects inflation radius, recalculating")
                        self.sent_request = False
                        self.future = None
                        return py_trees.common.Status.RUNNING
                    return py_trees.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Service call failed with exception: {e}")
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

class GoToRepositionPointClient(py_trees.behaviour.Behaviour):
    def __init__(self, name, service_name, reposition_offset, input_key, output_key):
        super().__init__(name)
        self.service_name = service_name
        self.reposition_offset = reposition_offset
        self.object_point_key = input_key
        self.waypoint_key = output_key
        self.current_point = (0.0, 0.0)
        self.current_yaw = 0.0
        self.reposition_point = None
        self.reposition_yaw = 0.0
        self.previous_object_point = (0.0, 0.0)
        self.client = None
        self.future = None
        self.sent_request = False
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=self.object_point_key,
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=self.waypoint_key,
            access=py_trees.common.Access.WRITE
        )

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

        self.client = self.node.create_client(GoToPoint, self.service_name)
        self.node.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=True)
        return True

    def initialise(self):
        self.sent_request = False
        self.future = None

    def odom_callback(self, msg: Odometry):
        self.current_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def calculate_reposition_point(self, start_point, end_point, offset):
        start_x, start_y = start_point
        end_x, end_y = end_point

        line_length = np.hypot(start_x - end_x, start_y - end_y)
        if offset > line_length:
            self.logger.error(f"{self.name} - Offset distance greater than line length to object")
            return py_trees.common.Status.FAILURE
        if line_length == 0:
            self.logger.error(f"{self.name} - Distance to object is zero")
            return py_trees.common.Status.FAILURE

        norm_x = (start_x - end_x) / line_length
        norm_y = (start_y - end_y) / line_length
        self.reposition_point = (end_x + norm_x * offset, end_y + norm_y * offset)
        self.reposition_yaw = np.arctan2(
            end_y - self.reposition_point[1],
            end_x - self.reposition_point[0]
        )
        waypoint = (self.reposition_point[0], self.reposition_point[1], self.reposition_yaw)
        self.blackboard.set(self.waypoint_key, waypoint, overwrite=True)

    def transform_point(self, x, y, transform_msg: TransformStamped):
        point_msg = PointStamped()
        point_msg.header.frame_id = transform_msg.header.frame_id
        point_msg.header.stamp = transform_msg.header.stamp
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = 0.0

        transformed_point_msg = tf2_geometry_msgs.do_transform_point(
            point_msg, transform_msg
        )

        transformed_x = transformed_point_msg.point.x
        transformed_y = transformed_point_msg.point.y
        return transformed_x, transformed_y

    def update(self):
        if not self.client.service_is_ready():
            self.node.get_logger().info(f"{self.name} - Waiting for service {self.service_name} ...")
            return py_trees.common.Status.RUNNING
        try:
            current_object_point = self.blackboard.get(self.object_point_key)
        except Exception as e:
            self.node.get_logger.error(f"{self.name} - Error reading blackboard: {e}")
            return py_trees.common.Status.INVALID

        if self.previous_object_point != current_object_point or self.reposition_point is None:
            try:
                arm_transform = self.tf_buffer.lookup_transform(
                    "map",
                    "arm_base",
                    rclpy.time.Time(seconds=0),
                    rclpy.duration.Duration(seconds=1.0)
                )
                new_object_x, new_object_y = self.transform_point(current_object_point[0], current_object_point[1], arm_transform)
                new_object_point = (new_object_x, new_object_y)
                self.calculate_reposition_point(self.current_point, new_object_point, self.reposition_offset)
                self.previous_object_point = current_object_point
                self.node.get_logger().info(f"{self.name} - Arm detected object at ({new_object_point[0]:.2f}, {new_object_point[1]:.2f}) repositioning to ({self.reposition_point[0]:.2f}, {self.reposition_point[1]:.2f}, yaw: {self.reposition_yaw:.2f})")
            except tf2_ros.TransformException as ex:
                self.node.get_logger().warn(f"Could not find transform between arm base to map frames: {ex}")
                return

        if not self.sent_request:
            try:
                request = GoToPoint.Request()
                request.x = self.reposition_point[0]
                request.y = self.reposition_point[1]
                request.yaw = self.reposition_yaw
                request.velocity = 0.12
                request.reverse = False
                request.slow_approach = True
                request.approaching_object = True

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

class GoToApproachPointClient(py_trees.behaviour.Behaviour):
    def __init__(self, name, service_name, approach_offset, input_key, output_key):
        super().__init__(name)
        self.service_name = service_name
        self.approach_offset = approach_offset
        self.object_point_key = input_key
        self.waypoint_key = output_key
        self.current_point = (0.0, 0.0)
        self.current_yaw = 0.0
        self.approach_point = None
        self.approach_yaw = 0.0
        self.previous_object_point = (0.0, 0.0)
        self.client = None
        self.future = None
        self.sent_request = False
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=self.object_point_key,
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=self.waypoint_key,
            access=py_trees.common.Access.WRITE
        )

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

        self.client = self.node.create_client(GoToPoint, self.service_name)
        self.node.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        return True

    def initialise(self):
        self.sent_request = False
        self.future = None

    def odom_callback(self, msg: Odometry):
        self.current_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def calculate_approach_point(self, start_point, end_point, offset):
        start_x, start_y = start_point
        end_x, end_y = end_point

        line_length = np.hypot(start_x - end_x, start_y - end_y)
        if offset > line_length:
            self.logger.error(f"{self.name} - Offset distance greater than line length to object")
            return py_trees.common.Status.FAILURE
        if line_length == 0:
            self.logger.error(f"{self.name} - Distance to object is zero")
            return py_trees.common.Status.FAILURE

        norm_x = (start_x - end_x) / line_length
        norm_y = (start_y - end_y) / line_length
        self.approach_point = (end_x + norm_x * offset, end_y + norm_y * offset)
        self.approach_yaw = np.arctan2(
            end_y - self.approach_point[1],
            end_x - self.approach_point[0]
        )
        waypoint = (self.approach_point[0], self.approach_point[1], self.approach_yaw)
        self.blackboard.set(self.waypoint_key, waypoint, overwrite=True)

    def update(self):
        if not self.client.service_is_ready():
            self.node.get_logger().info(f"{self.name} - Waiting for service {self.service_name} ...")
            return py_trees.common.Status.RUNNING
        try:
            current_object_point = self.blackboard.get(self.object_point_key)
        except Exception as e:
            self.logger.error(f"{self.name} - Error reading blackboard: {e}")
            return py_trees.common.Status.INVALID

        if self.previous_object_point != current_object_point or self.approach_point is None:
            self.calculate_approach_point(self.current_point, current_object_point, self.approach_offset)
            self.previous_object_point = current_object_point

        if not self.sent_request:
            try:
                request = GoToPoint.Request()
                request.x = self.approach_point[0]
                request.y = self.approach_point[1]
                request.yaw = self.approach_yaw
                request.velocity = 0.12
                request.reverse = False
                request.slow_approach = True
                request.approaching_object = True

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

class GoToWaypointClient(py_trees.behaviour.Behaviour):
    def __init__(self, name, service_name, velocity, reverse, slow_approach, approaching_object, input_key):
        super().__init__(name)
        self.service_name = service_name
        self.velocity = velocity
        self.reverse = reverse
        self.slow_approach = slow_approach
        self.approaching_object = approaching_object
        self.waypoint_key = input_key
        self.end_point = None
        self.end_yaw = 0.0
        self.client = None
        self.future = None
        self.sent_request = False
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=self.waypoint_key,
            access=py_trees.common.Access.READ
        )

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

        self.client = self.node.create_client(GoToPoint, self.service_name)
        return True

    def initialise(self):
        self.sent_request = False
        self.future = None

    def update(self):
        if not self.client.service_is_ready():
            self.node.get_logger().info(f"{self.name} - Waiting for service {self.service_name} ...")
            return py_trees.common.Status.RUNNING
        try:
            waypoint = self.blackboard.get(self.waypoint_key)
        except Exception as e:
            self.logger.error(f"{self.name} - Error reading blackboard: {e}")
            return py_trees.common.Status.INVALID

        self.end_point = (waypoint[0], waypoint[1])
        self.end_yaw = waypoint[2]

        if not self.sent_request:
            try:
                request = GoToPoint.Request()
                request.x = self.end_point[0]
                request.y = self.end_point[1]
                request.yaw = self.end_yaw
                request.velocity = self.velocity
                request.reverse = self.reverse
                request.slow_approach = self.slow_approach
                request.approaching_object = self.approaching_object

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

        self.see_counter_max = 1
        self.see_counter = 0

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False


        self.pickup_client = self.node.create_client(PickObject, 'PickObject')
        while not self.pickup_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Arm Request service not yet avaliable, waiting ...")

        self.camera_client = self.node.create_client(GetDetectedList, 'get_detected_list')
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

        self.clock = self.node.get_clock()

        self.future = None


        return True

    def initialise(self):
        self.stage = 0
        self.future = None
        self.see_counter = 0

    def print_seen(self,objects):
        for o in objects:
            self.node.get_logger().info(f"Seeing {o.label} at ({o.center_x},{o.center_y}) with confidence: {o.confidence}")

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
                        self.print_seen(response.objects)
                        self.see_counter+=1
                        if self.see_counter>=self.see_counter_max:
                            self.node.get_logger().info("Look for objects succeeded")
                            return py_trees.common.Status.SUCCESS
                        else:
                            self.stage = 2
                    else:
                        self.node.get_logger().info("Look for objects failed")
                        return py_trees.common.Status.FAILURE
                elif self.type == "boxes":
                    if len(response.boxes) > 0:
                        self.print_seen(response.boxes)
                        self.see_counter+=1
                        if self.see_counter>=self.see_counter_max:
                            self.node.get_logger().info("Look for boxes succeeded")
                            return py_trees.common.Status.SUCCESS
                        else:
                            self.stage = 2
                    else:
                        self.node.get_logger().info("Look for boxes failed")
                        return py_trees.common.Status.FAILURE
                    

            return py_trees.common.Status.RUNNING
        

class Adjust(py_trees.behaviour.Behaviour):
    def __init__(self, name, t, **kwargs):
        super().__init__(name)
        self.type = t
        self.node = None
        self.camera_client = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0
        self.future = None
        self.joint_future = None
        self.joint_client = None

        self.see_counter_max = 3
        self.see_counter = 0

        self.correct_counter_max = 1
        self.correct_counter = 0


    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False        


        self.camera_client = self.node.create_client(GetDetectedList, 'get_detected_list')
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')


        self.joint_client = self.node.create_client(JointMove, 'MoveArm')
        while not self.joint_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Joint move service not available, waiting...')


        return True

    def initialise(self):
        self.stage = 0
        self.future = None
        self.joint_future = None
        self.see_counter = 0
        self.correct_counter = 0

    def print_seen(self,objects):
        for o in objects:
            self.node.get_logger().info(f"Seeing {o.label} at ({o.center_x},{o.center_y}) with confidence: {o.confidence}")


    def update(self):
        if self.stage == 0:
            request = GetDetectedList.Request()

            self.future = self.camera_client.call_async(request)
            self.stage = 1

            return py_trees.common.Status.RUNNING

        elif self.stage == 1:
            if self.future.done():
                response = self.future.result()

                objects = response.objects
                boxes = response.boxes

                if self.type == "objects" and len(objects) == 0:
                    self.see_counter += 1
                    if self.see_counter >= self.see_counter_max:
                        self.node.get_logger().info(f"Adjust failed")
                        return py_trees.common.Status.FAILURE
                    else:
                        self.stage = 0
                else:
                    self.print_seen(objects)
                    self.see_counter = 0

                if self.type == "boxes" and len(boxes) == 0:
                    self.see_counter += 1
                    if self.see_counter >= self.see_counter_max:
                        self.node.get_logger().info(f"Adjust failed")
                        return py_trees.common.Status.FAILURE
                    else:
                        self.stage = 0
                else:
                    self.print_seen(boxes)
                    self.see_counter = 0

                

                move_command = JointMove.Request()
                move_command.objects = objects
                move_command.boxes = boxes
                move_command.target = self.type

                self.joint_future = self.joint_client.call_async(move_command)

                self.stage = 2

            return py_trees.common.Status.RUNNING
        
        
        elif self.stage == 2:
            if self.joint_future.done():
                response = self.joint_future.result()
                if response.result == 0:
                    self.correct_counter += 1
                    if self.correct_counter >= self.correct_counter_max:
                        self.node.get_logger().info(f"Adjust succeeded")
                        return py_trees.common.Status.SUCCESS
                    else:
                        self.stage=0
                elif response.result == 1:
                    self.correct_counter = 0
                    self.stage = 0
                    return py_trees.common.Status.RUNNING
                else:
                    self.see_counter += 1
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
        self.see_counter = 0
        self.see_counter_max = 5


        self.positions = [
            (3000, 5500, 1500, 2.0),  # base, neck, move_time, wait
            #(3000, 3000, 1000, 1.5),
            (6000, 5500, 1500, 2.0),
            (12000, 5500, 1500, 2.0),
            #(12000, 3000, 1000, 1.5),
            (18000, 5500, 1500, 2.0),
            #(21000, 3000, 1000, 1.5),
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
        self.counter = 0
        self.see_counter = 0
        self.future = None

    def update(self):
        if self.stage == 0:

            msg = Int16MultiArray()
            msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
            

            if self.counter < len(self.positions):
                base, neck, move_time, self.wait = self.positions[self.counter]
                pose = [11000,12000,neck,21000,12000,base,move_time,move_time,move_time,move_time,move_time,move_time]
                msg.data = pose
                self.joint_publisher.publish(msg)
            


            self.stage = 1

            self.start_time = time.time()

            return py_trees.common.Status.RUNNING

        elif self.stage == 1:
            if time.time() - self.start_time >= self.wait:
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
                
                objects = response.objects
                boxes = response.boxes

                if self.type == "objects" and len(objects) > 0:
                    self.see_counter += 1
                    if self.see_counter >= self.see_counter_max:
                        self.node.get_logger().info(f"Sweep Success")
                        return py_trees.common.Status.SUCCESS
                    else:
                        self.stage = 2
                else:
                    self.see_counter = 0
                    self.stage = 4

                if self.type == "boxes" and len(boxes) > 0:
                    self.see_counter += 1
                    if self.see_counter >= self.see_counter_max:
                        self.node.get_logger().info(f"Sweep Success")
                        return py_trees.common.Status.SUCCESS
                    else:
                        self.stage = 2
                else:
                    self.see_counter = 0
                    self.stage = 4

            return py_trees.common.Status.RUNNING
        
        elif self.stage == 4:
            if self.counter >= len(self.positions):
                self.node.get_logger().info(f"Sweep failed")
                return py_trees.common.Status.FAILURE

            self.counter += 1

            self.stage = 0


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
        self.counter_max = 5


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

        self.object_position_key = "object_position"


        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=self.object_position_key,
            access=py_trees.common.Access.WRITE
        )


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

        # self.camera_client = self.node.create_client(GetDetectedList, 'get_detected_list')
        # while not self.camera_client.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('Service not available, waiting...')

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
            if time.time() - self.start_time >= 0.5:
                self.stage = 2

            return py_trees.common.Status.RUNNING

        # Stage 2 estimates the position of the target given where the arm is pointing
        elif self.stage == 2:
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
            try:
                request = PickObject.Request()

                request.header = Header()
                request.header.stamp = self.node.get_clock().now().to_msg()
                request.header.frame_id = "arm_base"
                request.point = GeometryPoint()
                request.point.x = self.x
                request.point.y = self.y
                request.point.z = -0.14
                request.description = self.task

                self.future = self.pickup_client.call_async(request)
                self.stage = 4
                self.node.get_logger().info(f"{self.name} - Sent request to Pickup")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE


        elif self.stage == 4:
            if self.future.done():
                response = self.future.result()
                if response.result == 0:
                    return py_trees.common.Status.SUCCESS
                else:
                    object_position = (self.x,self.y)
                    self.blackboard.set(self.object_position_key,object_position)
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

        self.emergancy_distance = 0.35  
        self.emergancy_flag = False

        self.object_position_key = "object_position"


        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=self.object_position_key,
            access=py_trees.common.Access.WRITE
        )


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
        self.emergancy_flag = False

    def pos_callback(self,msg):
        self.base = msg.position[5]
        self.v1 = msg.position[4]
        self.v2 = msg.position[3]
        self.v3 = msg.position[2]

    def update(self):

        # First two stages is to pass some time to allow the correct joint readings to be read
        if self.stage == 0:
            self.start_time = time.time()

            self.stage = 1
            return py_trees.common.Status.RUNNING


        elif self.stage == 1:
            if time.time() - self.start_time >= 0.5:
                self.stage = 2

            return py_trees.common.Status.RUNNING

        # Stage 2 estimates the position of the target given where the arm is pointing
        elif self.stage == 2:
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
            if self.future.done():
                response = self.future.result()
                if response.result == 0:
                    return py_trees.common.Status.SUCCESS
                else:
                    object_position = (self.x,self.y)
                    self.blackboard.set(self.object_position_key,object_position)
                    return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING

class Return(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, t, task, **kwargs):
        super().__init__(name)
        self.type = t
        self.node = None
        self.request_args = kwargs
        self.stage = 0
        self.future = None
        self.task = task


    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.node.get_logger.error(f"{self.name} - Setup failed: {e}")
            return False

        self.pickup_client = self.node.create_client(PickObject, 'PickObject')
        while not self.pickup_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')


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
                request.point.x = 0.0
                request.point.y = 0.0
                request.point.z = 0.0
                request.description = self.task

                self.future = self.pickup_client.call_async(request)
                self.stage = 1
                self.node.get_logger().info(f"{self.name} - Sent request to Return")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE


        elif self.stage == 1:
            if self.future.done():
                response = self.future.result()
                if response.result == 0:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING
