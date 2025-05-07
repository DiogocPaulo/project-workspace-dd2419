#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

from robp_interfaces.msg import Encoders
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from robp_interfaces.msg import DutyCycles
from project_interfaces.srv import GoToPoint, Trigger
from project_interfaces.msg import NavPoint, NavPath

from mapping.map import Map
from navigation.robot_state import RobotState
from navigation.target_path import TargetPath

# Robot parameters
base = 0.3                      # Wheelbase of the vehicle
lookahead_gain = 0.1            # Look-ahead distance gain
lookahead_min = 0.3             # Minimum look-ahead distance
distance_threshold = 0.05       # Stop distance threshold
yaw_threshold = math.radians(5) # Stop yaw threshold
min_velocity = 0.10             # Minimum velocity
wheel_duty_min = 0.09           # Minimum wheel duty cycles
wait_distance = 5.0             # Distance to travel before waiting
wait_time = rclpy.duration.Duration(seconds=0.5) # Time to wait for localisation

def pure_pursuit_control(state, target_path, velocity, reverse=False, slow_approach=False):
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

    if slow_approach:
        distance_error = state.distance_to_state(target_path.x_points[-1], target_path.y_points[-1])
        if distance_error <= 1.0:
            velocity = max(min_velocity, velocity * (distance_error/1.0))

    if reverse:
        heading_error = math.atan2(target_y - state.y, target_x - state.x) - (state.yaw + math.pi)
        velocity = -abs(velocity)
    else:
        heading_error = math.atan2(target_y - state.y, target_x - state.x) - state.yaw

    alpha = math.atan2(math.sin(heading_error), math.cos(heading_error))

    speed_factor = np.exp(-2 * np.power(alpha, 2))
    linear_velocity = velocity * speed_factor

    kappa = 3.0 * np.arctan(alpha) / lookahead
    angular_velocity = (abs(velocity) * 0.5) * kappa

    return linear_velocity, angular_velocity

def angular_control(state, target_yaw, velocity):
    heading_error = target_yaw - state.yaw
    alpha = math.atan2(math.sin(heading_error), math.cos(heading_error))

    kappa = 3.0 * np.arctan(alpha)
    angular_velocity = (abs(velocity) * 0.5) * kappa

    return angular_velocity

class Navigation(Node):

    def __init__(self):
        super().__init__("navigation")

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        self.create_subscription(NavPath, "/custom_path", self.path_callback, qos_profile)
        self.create_subscription(OccupancyGrid, "/inflated_map", self.inflated_map_callback, qos_profile)
        self.motor_publisher = self.create_publisher(DutyCycles, "/motor/duty_cycles", 10)

        # Navigation parameters
        self.state = RobotState()
        self.target_path = TargetPath(lookahead_gain, lookahead_min)
        self.target_yaw = 0.0
        self.target_velocity = 0.0
        self.slow_approach = False
        self.previous_index = 0
        self.waiting_for_path = True
        self.reverse_travel = False
        self.inflated_map = None
        self.distance_traveled_since_wait = 0.0
        self.previous_x = 0.0
        self.previous_y = 0.0
        self.waiting = False
        self.wait_start_time = None


        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.state.update_state(msg)

        if not self.waiting and not self.waiting_for_path:
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y
            distance_traveled = np.hypot(current_x - self.previous_x, current_y - self.previous_y)
            self.distance_traveled_since_wait += distance_traveled
            self.previous_x = current_x
            self.previous_y = current_y

    def path_callback(self, msg: NavPath):
        if len(msg.path) > 0:
            self.waiting_for_path = False
            self.target_yaw = msg.yaw
            self.target_velocity = msg.velocity
            self.reverse_travel = msg.reverse
            self.slow_approach = msg.slow_approach
            self.target_path.update_path(msg)
            self.get_logger().info(f"Recived new path with end point: ({msg.path[-1].x:.2f}, {msg.path[-1].y:.2f})")
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
        if not self.target_path.x_points or self.waiting_for_path:
            self.get_logger().info("Waiting for path")
            self.publish_duty_cycles(0.0, 0.0)
            return

        if not self.waiting and self.distance_traveled_since_wait >= wait_distance:
            self.get_logger().info(f"Traveled {self.distance_traveled_since_wait:.2f} meters, stopping for {wait_time} seconds.")
            self.publish_duty_cycles(0.0, 0.0)
            self.waiting = True
            self.wait_start_time = self.get_clock().now()
            self.distance_traveled_since_wait = 0.0
            return

        if self.waiting:
            elapsed_time = self.get_clock().now() - self.wait_start_time
            if elapsed_time >= wait_time:
                self.waiting = False
                self.wait_start_time = None
                self.previous_x = self.state.x
                self.previous_y = self.state.y
            else:
                self.publish_duty_cycles(0.0, 0.0)
                return

        distance_error = self.state.distance_to_state(
            self.target_path.x_points[-1],
            self.target_path.y_points[-1],
        )
        yaw_error = math.atan2(
            math.sin(self.target_yaw - self.state.yaw),
            math.cos(self.target_yaw - self.state.yaw),
        )

        if distance_error > distance_threshold:
            linear_velocity, angular_velocity = pure_pursuit_control(
                self.state,
                self.target_path,
                self.target_velocity,
                reverse=self.reverse_travel,
                slow_approach=self.slow_approach,
            )

            left_wheel = linear_velocity - (base/2) * angular_velocity
            right_wheel = linear_velocity + (base/2) * angular_velocity
        elif abs(yaw_error) > yaw_threshold:
            angular_velocity = angular_control(
                self.state,
                self.target_yaw,
                self.target_velocity,
            )

            left_wheel = 0.0 - (base/2) * angular_velocity
            right_wheel = 0.0 + (base/2) * angular_velocity
        else:
            self.get_logger().info(f"Reached end of target path")
            self.waiting_for_path = True

            # Clear existing target path
            self.target_path.x_points = []
            self.target_path.y_points = []
            return

        # Clamp left and right wheel duty cycles
        left_wheel = np.copysign(np.maximum(np.abs(left_wheel), wheel_duty_min), left_wheel)
        right_wheel = np.copysign(np.maximum(np.abs(right_wheel), wheel_duty_min), right_wheel)
        self.get_logger().info(f"Velocity: {self.state.velocity:.3f}, Left: {left_wheel:.3f}, Right: {right_wheel:.3f}")

        self.publish_duty_cycles(left_wheel, right_wheel)

def main():
    rclpy.init()
    node = Navigation()
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
