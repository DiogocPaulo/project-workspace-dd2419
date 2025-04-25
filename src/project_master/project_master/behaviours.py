#!/usr/bin/env python

import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from project_interfaces.srv import GoToPoint, Trigger
from project_interfaces.srv import PickObject

from nav_msgs.msg import Odometry


import rclpy
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
from project_interfaces.msg import DetectedData, DetectedDataArray
from project_interfaces.srv import GetDetectedList, JointMove
from sensor_msgs.msg import JointState
import math
import numpy as np
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from builtin_interfaces.msg import Time
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, TransformStamped


#New imports
from builtin_interfaces.msg import Duration
import time

# a node has executed completely after returning a SUCCESS or FAILURE

# setup - one time constructor
# initialized - run when node was first ticked or execution completed
# update - called every time the node is ticked

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
            self.logger.info("Arm Request service not yet avaliable, waiting ...")

        self.camera_client = self.node.create_client(GetDetectedList, 'get_detected_list')
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

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
                request.header.frame_id = "map"
                request.point = Point()
                request.point.x = self.x
                request.point.y = self.y
                request.point.z = 0.0
                request.description = "LOOK"

                self.future = self.pickup_client.call_async(request)
                self.stage = 1
                self.node.get_logger().info(f"{self.name} - Sent request to Look")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.get_logger().error(f"{self.name} - Failed to send request: {e}")
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
        self.step_size = 50

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False


        self.camera_client = self.node.create_client(GetDetectedList, 'get_detected_list')
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

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
            request = GetDetectedList.Request()

            self.future = self.camera_client.call_async(request)
            self.stage = 1

            return py_trees.common.Status.RUNNING

        elif self.stage == 1:
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
                if closest_obj.distance <= eps:
                    return py_trees.common.Status.SUCCESS

                base = self.base
                v3 = self.v3

                if closest_obj.distance < 30:
                    self.step_size = 10
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
            if time.time() - self.start_time >= 0.1:
                self.stage = 0

            return py_trees.common.Status.RUNNING


class Pick(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, t, **kwargs):
        super().__init__(name)
        self.type = t
        self.node = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0
        self.future = None
        self.x = None
        self.y = None

        self.base = 12000
        self.v1 = 12000
        self.v2 = 12000
        self.v3 = 12000
        self.off_base = 0.14


    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False

        self.clock = self.node.get_clock()

        self.pickup_client = self.node.create_client(PickObject, 'PickObject')
        while not self.pickup_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

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
                request.point = Point()
                request.point.x = self.x
                request.point.y = self.y
                request.point.z = -0.15
                request.description = "PICKUP"

                self.future = self.pickup_client.call_async(request)
                self.stage = 4
                self.node.get_logger().info(f"{self.name} - Sent request to Pickup")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE


        elif self.stage == 4:
            if self.future.done():
                response = self.future.result()
                if response.result == 0:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING
        





