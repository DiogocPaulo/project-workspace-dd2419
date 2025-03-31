#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from robp_interfaces.msg import DutyCycles
from project_interfaces.srv import GoToPoint, Trigger

from navigation.map import Map
from navigation.robot_state import RobotState
from navigation.target_path import TargetPath

# Robot parameters
base = 0.3                  # Wheelbase of the vehicle
lookahead_gain = 0.1        # Look-ahead distance gain
lookahead_min = 0.3         # Minimum look-ahead distance
distance_threshold = 0.15   # Stop distance threshold
yaw_threshold = 0.2         # Stop yaw threshold
target_velocity = 0.15      # Robot's target velocity
backing_velocity = -0.15

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

    if state.target_velocity < 0:
        effective_yaw = state.yaw + math.pi
    else:
        effective_yaw = state.yaw

    alpha = math.atan2(target_y - state.y, target_x - state.x) - effective_yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    kappa = 2.0 * math.sin(alpha) / lookahead

    omega = state.target_velocity * kappa

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
    omega = state.target_velocity * kappa
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
        self.create_subscription(OccupancyGrid, "/inflated_map", self.inflated_map_callback, qos_profile)
        self.motor_publisher = self.create_publisher(DutyCycles, "/motor/duty_cycles", 10)

        # Navigation parameters
        self.state = RobotState(target_velocity)
        self.target_path = TargetPath(lookahead_gain, lookahead_min)
        self.previous_index = 0
        self.waiting_for_path = True
        self.backing_up = False
        self.inflated_map = None

        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.state.update_state(msg)

    def path_callback(self, msg: Path):
        if self.backing_up:
            return
        if len(msg.poses) > 0:
            self.waiting_for_path = False
            self.target_path.update_path(msg)
            self.get_logger().info(f"Recived new path with end point: ({msg.poses[-1].pose.position.x:.2f}, {msg.poses[-1].pose.position.y:.2f})")
        else:
            self.waiting_for_path = True
            self.get_logger().warn("Recived empty path")

    def inflated_map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Create map or update map grid
        if self.inflated_map is None:
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            self.inflated_map = Map(resolution, origin_x, origin_y, width, height, grid)
        else:
            self.inflated_map.update_grid(grid)
        
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
        in_inflated_region = self.inflated_map is not None and not self.inflated_map.is_free(self.state.x, self.state.y, 50)
        if in_inflated_region and not self.backing_up:
            self.get_logger().info("Entering backing up process")
            self.state.target_velocity = backing_velocity
            self.target_path.reverse_path()
            self.backing_up = True
            return
        elif not in_inflated_region and self.backing_up:
            self.get_logger().info("Exiting backing up process")
            self.state.target_velocity = target_velocity
            self.target_path.reverse_path()
            self.backing_up = False
            return

        if not self.target_path.x_points or (self.waiting_for_path and not self.backing_up):
            self.get_logger().info("Waiting for path")
            self.publish_duty_cycles(0.0, 0.0)
            return

        omega, alpha, self.previous_index = pure_pursuit_control(self.state, self.target_path)

        if self.previous_index >= (len(self.target_path.x_points) - 1):
            distance = self.state.distance_to_state(self.target_path.x_points[-1], self.target_path.y_points[-1])
            if distance <= distance_threshold and not self.waiting_for_path:
                self.get_logger().info(f"Reached end of target path")
                self.waiting_for_path = True

                # Clear existing target path
                self.target_path.x_points = []
                self.target_path.y_points = []
                return

        if self.backing_up:
            left_wheel = self.state.target_velocity - (base/2) * omega
            right_wheel = self.state.target_velocity + (base/2) * omega
            self.get_logger().info(f"Velocity: {self.state.velocity:.3f}, Left: {left_wheel:.3f}, Right: {right_wheel:.3f}")
        elif (abs(alpha) > (math.pi / 2)):
            angular_velocity = 0.10
            left_wheel = -angular_velocity
            right_wheel = angular_velocity
            self.get_logger().info(f"Velocity: {angular_velocity:.3f}, Left: {left_wheel:.3f}, Right: {right_wheel:.3f}")
        else:
            # angular_scale = 2 * (np.abs(alpha) / np.pi)
            command_velocity = self.state.target_velocity * np.exp(-2 * np.abs(alpha))
            left_wheel = command_velocity - (base/2) * omega
            right_wheel = command_velocity + (base/2) * omega
            self.get_logger().info(f"Velocity: {self.state.velocity:.3f}, Left: {left_wheel:.3f}, Right: {right_wheel:.3f}")

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
