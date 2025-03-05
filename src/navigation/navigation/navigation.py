#!/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from robp_interfaces.msg import DutyCycles
from project_interfaces.srv import Trigger

# Robot parameters
base = 0.3                  # Wheelbase of the vehicle
lookahead_gain = 0.1        # Look-ahead distance gain
lookahead_min = 0.3         # Minimum look-ahead distance
distance_threshold = 0.2    # Stop distance threshold
yaw_threshold = 0.2         # Stop yaw threshold
target_velocity = 0.18      # Robot's target velocity

class RobotState:
    """Using odometry message to update the current state of the robot"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.velocity = 0.0

    def update_state(self, odom_msg):
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y

        q = odom_msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = np.arctan2(siny_cosp, cosy_cosp)

        odom_velocity = odom_msg.twist.twist.linear.x
        self.velocity = odom_velocity + (target_velocity - odom_velocity)

    def distance_to_state(self, x, y):
        return np.hypot(self.x - x, self.y - y)

class TargetPath:
    """Determines the target route and searching of current point to navigate towards"""
    def __init__(self):
        self.x_points = []
        self.y_points = []
        self.target_yaw = 0.0
        self.old_nearest_point_index = None

    def compare_to_target_yaw(self, yaw, threshold):
        yaw1 = (self.target_yaw + 180) % 360 - 180
        yaw2 = (yaw + 180) % 360 - 180

        error = abs(yaw1 - yaw2)

        if error > 180:
            error -= 360

        return error <= threshold

    def update_path(self, path_msg):
        self.x_points = [pose.pose.position.x for pose in path_msg.poses]
        self.y_points = [pose.pose.position.y for pose in path_msg.poses]

        self.target_yaw = path_msg.poses[-1].pose.orientation.w
        
        self.old_nearest_point_index = None

    def search_target_index(self, state):
        if not self.x_points or not self.y_points:
            # Checks if there exits a path
            return None, None

        if self.old_nearest_point_index is None:
            # Search for nearest point on path to robot state
            dx = [state.x - i for i in self.x_points]
            dy = [state.y - i for i in self.y_points]
            distances = np.hypot(dx, dy)
            index = np.argmin(distances)
        else:
            index = self.old_nearest_point_index
            distance_to_index = state.distance_to_state(self.x_points[index], self.y_points[index])
            while True:
                if (index + 1) >= len(self.x_points):
                    break
                distance_to_next_index = state.distance_to_state(self.x_points[index + 1], self.y_points[index + 1])
                if distance_to_index < distance_to_next_index:
                    break
                index += 1
                distance_to_index = distance_to_next_index
            self.old_nearest_point_index = index

        # Compute the lookahead distance
        lookahead = lookahead_gain * state.velocity + lookahead_min

        # Find index of target point within lookahead distance
        while lookahead > state.distance_to_state(self.x_points[index], self.y_points[index]):
            if (index + 1) >= len(self.x_points):
                break
            index += 1

        return index, lookahead

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

        self.state = RobotState()
        self.target_path = TargetPath()
        self.previous_index = 0

        self.create_subscription(
                Odometry,
                "/odom",
                self.odom_callback,
                10)
        self.create_subscription(
                Path,
                "/custom_path",
                self.path_callback,
                10)
        self.motor_publisher = self.create_publisher(DutyCycles, "/motor/duty_cycles", 10)
        self.reached_destination_client = self.create_client(Trigger, "/reached_destination")
        while not self.reached_destination_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Reached destination service not yet avaliable, waiting ...")

        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.state.update_state(msg)

    def path_callback(self, msg: Path):
        self.target_path.update_path(msg)

    def send_reached_destination(self):
        request = Trigger.Request()
        future = self.reached_destination_client.call_async(request)
        future.add_done_callback(self.reached_destination_response_callback)

    def reached_destination_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Trigger response: {response.success}, {response.message}")
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {e}")

    def control_loop(self):
        if not self.target_path.x_points:
            self.get_logger().info("No target path")
            return

        omega, alpha, self.previous_index = pure_pursuit_control(self.state, self.target_path)

        if self.previous_index >= len(self.target_path.x_points) - 1:
            distance = np.hypot(self.state.x - self.target_path.x_points[-1], self.state.y - self.target_path.y_points[-1])
            if distance <= distance_threshold:
                self.get_logger().info(f"Reached destination")
                self.send_reached_destination()
                return
                # if self.target_path.compare_to_target_yaw(self.state.yaw, yaw_threshold):
                #     self.get_logger().info(f"Reached destination")
                #     self.send_reached_destination()
                #     return
                # else:
                #     omega, alpha = calculate_angular_velocity(self.state, self.state.yaw, self.target_path.target_yaw)

        command_velocity = self.state.velocity * np.exp(-2 * np.abs(alpha))
        left_wheel = command_velocity - (base/2) * omega
        right_wheel = command_velocity + (base/2) * omega
        self.get_logger().info(f"Velocity: {command_velocity}, Left: {left_wheel:.3f}, Right: {right_wheel:.3f}")

        max_value = max(abs(left_wheel), abs(right_wheel))

        if max_value > 1:
            left_wheel = left_wheel / max_value
            right_wheel = right_wheel / max_value

        min_value = 0.08

        # if left_wheel > 0:
        #     left_wheel = max(left_wheel, min_value)
        # else:
        #     left_wheel = min(left_wheel, -min_value)
        # if right_wheel > 0:
        #     right_wheel = max(right_wheel, min_value)
        # else:
        #     right_wheel = min(right_wheel, -min_value)

        dutyCycles = DutyCycles()
        dutyCycles.header.frame_id = "base_link"
        dutyCycles.header.stamp = self.get_clock().now().to_msg()
        dutyCycles.duty_cycle_left = left_wheel
        dutyCycles.duty_cycle_right = right_wheel
        self.motor_publisher.publish(dutyCycles)

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
