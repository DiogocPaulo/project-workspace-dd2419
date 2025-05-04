#!/usr/bin/env python

import math
import numpy as np
import time

import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from project_interfaces.srv import GoToPoint, Trigger
from project_interfaces.msg import Vertex, WorkspaceVertices
from mapping.map import Map

# a node has executed completely after returning a SUCCESS or FAILURE
# setup - one time constructor
# initialized - run when node was first ticked or execution completed
# update - called every time the node is ticked

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
    def __init__(self, name, x, y, yaw, distance_threshold=0.08, yaw_threshold=0.1):
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

        self.node.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        return True

    def odom_callback(self, msg: Odometry):
        self.current_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def update(self):
        if self.current_point == (None, None):
            self.node.get_logger().info(f"{self.name}: Waiting for current point ...")
            return py_trees.common.Status.RUNNING
        
        distance_error = np.hypot(
            self.current_point[0] - self.end_point[0],
            self.current_point[1] - self.end_point[1]
        )
        yaw_error = math.atan2(
            math.sin(self.target_yaw - self.current_yaw),
            math.cos(self.target_yaw - self.current_yaw)
        )

        if distance_error > self.distance_threshold:
            self.node.get_logger().info(f"{self.name}: Distance to waypoint is {distance_error:.2f}")
            return py_trees.common.Status.RUNNING
        elif yaw_error > self.yaw_threshold:
            self.node.get_logger().info(f"{self.name}: Correcting yaw by {yaw_error:.2f}")
            return py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().info(f"{self.name}: Reached waypoint ({self.end_point[0]:.2f}, {self.end_point[1]:.2f}) with yaw {self.current_yaw:.2f}")
            return py_trees.common.Status.SUCCESS

class WaitBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name, duration=0.5):
        super().__init__(name)
        self.duration = duration
        self.start_time = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False
        return True

    def initialise(self):
        self.start_time = time.time()
        self.node.get_logger().info(f"{self.name}: Starting {self.duration}-second wait")

    def update(self):
        if self.start_time is None:
            self.node.get_logger().error(f"{self.name}: Timer not initialized")
            return py_trees.common.Status.FAILURE

        elapsed = time.time() - self.start_time
        if elapsed < self.duration:
            return py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().info(f"{self.name}: Wait complete")
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status=None):
        self.start_time = None

