#!/usr/bin/env python

import numpy as np
import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from project_interfaces.srv import GoToPoint, Trigger
from project_interfaces.srv import PickObject

from nav_msgs.msg import Odometry

# a node has executed completely after returning a SUCCESS or FAILURE

# setup - one time constructor
# initialized - run when node was first ticked or execution completed
# update - called every time the node is ticked

class EndPointSelector(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, end_points):
        super().__init__(name)
        self.end_points = end_points
        self.index = 0
        self.navigate_behaviour = None

    def setup(self):
        x, y, _ = self.end_points[self.index]
        self.navigate_behaviour = Navigate(f"Navigate_to_{self.index}", x, y)
        self.navigate_behaviour.setup()
        return True

    def update(self):
        status = self.navigate_behaviour.update()

        if status == py_trees.common.Status.SUCCESS:
            self.navigate_behaviour.terminate()
            self.index = (self.index + 1) % len(self.end_points)

            x, y, _ = self.end_points[self.index]
            self.navigate_behaviour = Navigate(f"Navigate_to_{self.index}", x, y)
            self.navigate_behaviour.setup()

            return py_trees.common.Status.RUNNING
        
        return status

    def terminate(self):
        if self.navigate_behaviour:
            self.navigate_behaviour.terminate()


class Navigate(py_trees.behaviour.Behaviour):
    """
    A behaviour that sends an end point to the autonomous pather.
    """
    def __init__(self, name, x, y, tolerance=0.2):
        super().__init__(name)
        self.current_point = (None, None)
        self.end_point = (x, y)
        self.tolerance = tolerance
        self.node = None  # ROS 2 node will be initialized later

    def setup(self):
        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.node = rclpy.create_node("navigate_behaviour")
        self.node.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        self.end_point_client = self.node.create_client(GoToPoint, "/pathing_end_point")
        while not self.end_point_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("GoToPoint service not yet avaliable, waiting ...")
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def odom_callback(self, msg: Odometry):
        self.current_point[0] = msg.pose.pose.position.x
        self.current_point[1] = msg.pose.pose.position.y

    def update(self):
        if self.current_point == (None, None):
            self.logger.info(f"{self.name}: Waiting for current point ...")
            return py_trees.common.Status.RUNNING
        
        distance = np.hypot(
            self.current_point[0] - self.end_point[0],
            self.current_point[1] - self.end_point[1]
        )

        if distance <= self.tolerance:
            self.logger.info(f"{self.name}: Reached end point of ({self.end_point[0], self.end_point[1]})")
            return py_trees.common.Status.SUCCESS
        else:
            self.send_end_point(self.end_point[0], self.end_point[1])
            self.logger.info(f"{self.name}: Distance to end point is {distance:.2f}")
            return py_trees.common.Status.RUNNING

    def send_end_point(self, x, y):
        # Create new GoToPoint service for pathing node
        request = GoToPoint.Request()
        request.x = x
        request.y = y
        request.yaw = 0.0

        # Handle request and response to pathing node async
        future = self.end_point_client.call_async(request)
        future.add_done_callback(self.pathing_response_callback)

    def pathing_response_callback(self, future):
        try:
            response = future.result()
            self.logger.info(f"Pathing response: {response.success}, {response.message}")
        except Exception as e:
            self.logger.warn(f"Service call to pathing node failed: {e}")

    def terminate(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        self.logger.info(f"{self.name}: Terminated")


class PickUp(py_trees.behaviour.Behaviour):
    """
    A behaviour that sends a position for the arm to pick up at.
    """
    def __init__(self, name, x, y):
        super().__init__(name)
        self.object_position = (x,y)
        self.node = None  # ROS 2 node will be initialized later

    def setup(self):
        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.node = rclpy.create_node("pickup_behaviour")
        self.arm_client = self.create_client(PickObject, 'PickObject')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Arm Request service not yet avaliable, waiting ...")

        # self.node.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)

        # self.end_point_client = self.node.create_client(GoToPoint, "/pathing_end_point")
        # while not self.end_point_client.wait_for_service(timeout_sec=1.0):
        #     self.logger.info("GoToPoint service not yet avaliable, waiting ...")
        self.logger.info(f"{self.name}: Setup complete")
        return True

    def send_arm_request(self, x, y, z, task):
        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        self.get_logger().info("ARMTASK!")
        arm_msg = PickObject.Request()
        arm_msg.header = Header()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        arm_msg.header.frame_id = "base_link"
        arm_msg.point = Point()
        arm_msg.point.x = x
        arm_msg.point.y = y
        arm_msg.point.z = z
        arm_msg.description = task

        future = self.arm_client.call_async(arm_msg)
        rclpy.spin_until_future_complete(self, future)

        response = future.result() 
        if response is not None:
            if response.result == 0:
                self.get_logger().info(f"REQUEST SUCCESFUL!")
            elif response.result == 1:
                self.get_logger().info(f"REQUEST FAILED!: COULD NOT FIND KINEMATIC SOLUTION")
            else:
                self.get_logger().info(f"REQUEST FAILED!: PICKUP DID NOT PICK UP OBJECT")
        else:
            self.get_logger().error("NO RESPONSE RECIEVED!")


        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))


    

    def terminate(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        self.logger.info(f"{self.name}: Terminated")


