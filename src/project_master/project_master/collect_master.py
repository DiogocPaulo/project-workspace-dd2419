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

class GoToObjectSafePoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, object_point_key, service_name, **kwargs)
        super().__init__(name)
        self.object_point_key = object_point_key
        self.service_name = service_name
        self.current_point = (None, None)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.safe_point = None
        self.client = None
        self.future = None
        self.sent_request = False
        self.inflated_map = None

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
        if not self.blackboard.exists(self.object_point_key):
            self.node.get_logger().warn(f"{self.name} - Blackboard key {self.object_point_key} does not exist")
        return True

    def initialise(self):
        self.sent_request = False
        self.future = None

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

    def update(self):
        if not self.client.service_is_ready():
            self.node.get_logger().info(f"{self.name} - Waiting for service {self.service_name} ...")
            return py_trees.common.Status.RUNNING
        if self.inflated_map is None:
            self.node.get_logger().info(f"{self.name} - Waiting for inflated map ...")
            return py_trees.common.Status.RUNNING

        try:
            object_point = self.blackboard.get(self.object_point_key)
        except Exception as e:
            self.logger.error(f"{self.name} - Error reading blackboard: {e}"):
            return py_trees.common.Status.FAILURE

        self.safe_point = self.inflated_map.get_safe_point(object_point[0], object_point[1], 3, 75)
        if self.safe_point is None:
            self.safe_point = (0.0, 0.0)

        if not self.sent_request:

        

class FindClosestObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, object_list, find_box, output_key, **kwargs):
        super().__init__(name)
        self.object_list = object_list
        self.find_box = find_box
        self.output_key = output_key
        self.current_point = (None, None)
        self.blackboard = py_trees.blackboard.Blackboard()

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
            self.node.get_logger().info(f"{self.name} - Waiting for current point ...")
            return py_trees.common.Status.RUNNING
        if not self.object_list:
            self.node.get_logger().info(f"{self.name} - Object list is empty. No objects to find.")
            return py_trees.common.Status.FAILURE

        closest_object = None
        min_distance = float("inf")
        for object_msg in self.object_list:
            object_type_check = (obj.object_type == Object.BOX) if self.find_box else (obj.object_type != Object.BOX)
            if object_type_check:
                distance = np.hypot(object_msg.x - self.current_point[0], object_msg.y - self.current_point[1])
                if distance < min_distance:
                    min_distance = distance
                    closest_object = object_msg

        if closest_object is not None:
            object_point = (closest_object.x, closest_object.y)
            self.blackboard.set(self.output_key, object_point)
            self.object_list.remove(closest_object_msg)
            self.node.get_logger().info(f"{self.name} - Closest object found (type: {closest_object_msg.object_type}) at ({object_point[0]:.2f}, {object_point[1]:.2f})")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warning(f"{self.name} - No objects (box type: {self.find_box}) found in the list.")
            return py_trees.common.Status.FAILURE

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
            self.node.get_logger().info(f"{self.name} - Waiting for current point ...")
            return py_trees.common.Status.RUNNING
        
        distance_error = np.hypot(
            self.current_point[0] - self.end_point[0],
            self.current_point[1] - self.end_point[1]
        )
        yaw_error = math.atan2(math.sin(self.target_yaw - self.current_yaw), math.cos(self.target_yaw - self.current_yaw))

        if distance_error > self.distance_threshold:
            self.node.get_logger().info(f"{self.name} - Distance to waypoint is {distance_error:.2f}")
            self.broadcast_waypoint()
            return py_trees.common.Status.RUNNING
        elif yaw_error > self.yaw_threshold:
            self.node.get_logger().info(f"{self.name} - Correcting yaw by {yaw_error:.2f}")
            self.broadcast_waypoint()
            return py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().info(f"{self.name} - Reached waypoint of ({self.end_point[0], self.end_point[1]}) at {self.current_yaw}")
            return py_trees.common.Status.SUCCESS

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


class CollectMaster(Node):
    def __init__(self):
        super().__init__("explore_master")

        self.workspace_publisher = self.create_publisher(WorkspaceVertices, "/workspace", 10)
        self.object_list_publisher = self.create_publisher(ObjectList, "/detected_objects", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Constants
        self.target_velocity = 0.16
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

        pickup_sequence = py_trees.composites.Sequence("PickupSequence", memory=True)
        pickup_sequence.add_children([
           find_object,
           goto_object_safe_point,
           goto_object_approach_point,
           pickup_object_routine,
           goto_safe_point,
        ])

        drop_sequence = py_trees.composites.Sequence("DropSequence", memory=True)
        drop_sequence.add_children([
            find_box,
            goto_box_safe_point,
            goto_box_approach_point,
            drop_object_routine,
            goto_safe_point,
        ])

        collection_sequence.add_children([
            pickup_sequence,
            drop_sequence,
        ])

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
                    
                    objects.append(obj)
            
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

