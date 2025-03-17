#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from robp_interfaces.msg import DutyCycles
from project_interfaces.srv import GoToPoint, Trigger

from navigation.robot_state import RobotState
from navigation.target_path import TargetPath

# Robot parameters
base = 0.3                  # Wheelbase of the vehicle
lookahead_gain = 0.1        # Look-ahead distance gain
lookahead_min = 0.3         # Minimum look-ahead distance
distance_threshold = 0.2    # Stop distance threshold
yaw_threshold = 0.2         # Stop yaw threshold
target_velocity = 0.22      # Robot's target velocity

def pure_pursuit_control(state, target_path):
    index, lookahead = target_path.search_target_index(state)
    
    if index is None:
        return 0.0, 0
    
    if index < len(target_path.x_points):
        target_x = target_path.x_points[index]
        target_y = target_path.y_points[index]
    else:
        target_x = target_path.x_points[-1]
        target_y = target_path.y_points[-1]
        index = len(target_path.x_points) - 1

    alpha = math.atan2(target_y - state.y, target_x - state.x) - state.yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    kappa = 2.0 * math.sin(alpha) / lookahead

    omega = state.velocity * kappa

    return omega, alpha, index

def calculate_angular_velocity(state, state_yaw, target_yaw):
    state_yaw = (state_yaw + 180) % 360 - 180
    target_yaw = (target_yaw + 180) % 360 - 180

    error = target_yaw - state_yaw
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    alpha = math.atan2(math.sin(error), math.cos(error))
    kappa = 2.0 * math.sin(alpha)
    omega = state.velocity * kappa
    return omega, alpha

class Navigation(Node):

    def __init__(self):
        super().__init__("navigation")

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        self.create_subscription(Path, "/custom_path", self.path_callback, qos_profile)
        self.motor_publisher = self.create_publisher(DutyCycles, "/motor/duty_cycles", 10)
        self.end_point_service = self.create_service(GoToPoint, "/navigation_point", self.set_end_point)
        self.end_point_client = self.create_client(GoToPoint, "/pathing_point")
        while not self.end_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug("GoToPoint service not yet avaliable, waiting ...")
        self.reached_destination_client = self.create_client(Trigger, "/reached_destination")
        while not self.reached_destination_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug("Trigger service not yet avaliable, waiting ...")
        self.clear_pathing_client = self.create_client(Trigger, "/clear_pathing")
        while not self.clear_pathing_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug("Trigger service not yet avaliable, waiting ...")

        # Navigation parameters
        self.state = RobotState()
        self.target_path = TargetPath()
        self.previous_index = 0
        self.end_point = (None, None)
        self.target_yaw = None
        self.waiting_for_path = True
        self.reached_destination = False

        self.create_timer(0.05, self.control_loop)
        self.send_reached_destination()

    def odom_callback(self, msg: Odometry):
        self.state.update_state(msg)

    def path_callback(self, msg: Path):
        if len(msg.poses) > 0:
            self.waiting_for_path = False
            self.target_path.update_path(msg)
            self.get_logger().info(f"Recived new path with end point: ({msg.poses[-1].pose.position.x:.2f}, {msg.poses[-1].pose.position.y:.2f})")
        else:
            self.waiting_for_path = True
            self.get_logger().warn("Recived empty path")

    def set_end_point(self, request, response):
        # New destination received
        self.end_point = (request.x, request.y)
        self.target_yaw = request.yaw
        self.reached_destination = False

        self.send_end_point(self.end_point[0], self.end_point[1], self.target_yaw)

        self.get_logger().info(f"New end point set: ({self.end_point[0]}, {self.end_point[1]}) at {self.target_yaw} radians")
        response.success = True
        response.message = f"Navigation end point set: ({self.end_point[0]}, {self.end_point[1]}) at {self.target_yaw} radians"
        return response

    def send_end_point(self, x, y, yaw):
        self.waiting_for_path = True
        # Create new GoToPoint service for pathing node
        pathing_request = GoToPoint.Request()
        pathing_request.x = x
        pathing_request.y = y
        pathing_request.yaw = yaw

        # Handle request and response to pathing node async
        future = self.end_point_client.call_async(pathing_request)
        future.add_done_callback(self.pathing_response_callback)

    def clear_pathing(self):
        self.waiting_for_path = True
        # Create new Trigger service for pathing node
        pathing_request = Trigger.Request()
        # Handle request and response to pathing node async
        future = self.clear_pathing_client.call_async(pathing_request)
        future.add_done_callback(self.pathing_response_callback)

    def pathing_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Pathing response: {response.success}, {response.message}")
        except Exception as e:
            self.get_logger().warn(f"Service call to pathing node failed: {e}")

    def send_reached_destination(self):
        master_request = Trigger.Request()
        future = self.reached_destination_client.call_async(master_request)
        future.add_done_callback(self.master_response_callback)

    def master_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Master response: {response.success}, {response.message}")
        except Exception as e:
            self.get_logger().warn(f"Service call to master node failed: {e}")

    def publish_duty_cycles(self, left_wheel, right_wheel):
        # Ensure left and right duty cycles are between -1 to 1
        max_value = max(abs(left_wheel), abs(right_wheel))
        if max_value > 1:
            left_wheel = left_wheel / max_value
            right_wheel = right_wheel / max_value

        # Publish duty cycles for motors
        duty_msg = DutyCycles()
        duty_msg.header.frame_id = "base_link"
        duty_msg.header.stamp = self.get_clock().now().to_msg()
        duty_msg.duty_cycle_left = left_wheel
        duty_msg.duty_cycle_right = right_wheel
        self.motor_publisher.publish(duty_msg)

    def control_loop(self):
        if self.reached_destination:
            self.get_logger().info("Waiting for new destination")
            self.publish_duty_cycles(0.0, 0.0)
            return
        if not self.target_path.x_points or self.waiting_for_path:
            self.get_logger().info("Waiting for path")
            self.publish_duty_cycles(0.0, 0.0)
            return

        omega, alpha, self.previous_index = pure_pursuit_control(self.state, self.target_path)

        if self.previous_index >= (len(self.target_path.x_points) - 1):
            distance = self.state.distance_to_state(self, self.end_point[0], self.end_point[1])
            if distance <= distance_threshold and not self.waiting_for_path:
                self.get_logger().info(f"Reached destination")
                self.waiting_for_path = True
                self.reached_destination = True

                # Clear existing target path
                self.target_path.x_points = []
                self.target_path.y_points = []

                self.send_reached_destination()
                self.clear_pathing()
                return

        if abs(alpha) > (math.pi / 2):
            angular_velocity = 0.15
            left_wheel = -angular_velocity
            right_wheel = angular_velocity
            self.get_logger().info(f"Velocity: {angular_velocity:.3f}, Left: {left_wheel:.3f}, Right: {right_wheel:.3f}")
        else:
            # angular_scale = 2 * (np.abs(alpha) / np.pi)
            command_velocity = self.state.velocity * np.exp(-2 * np.abs(alpha))
            left_wheel = command_velocity - (base/2) * omega
            right_wheel = command_velocity + (base/2) * omega
            self.get_logger().info(f"Velocity: {command_velocity:.3f}, Left: {left_wheel:.3f}, Right: {right_wheel:.3f}")

        self.publish_duty_cycles(left_wheel, right_wheel)

def main():
    rclpy.init()
    node = Navigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
