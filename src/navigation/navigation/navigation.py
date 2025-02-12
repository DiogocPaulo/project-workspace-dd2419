#/usr/bin/env python

import math
import numpy as np

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from robp_interfaces.msg import DutyCycles

from example_interfaces.srv import Trigger

DEBUG = True

# Robot parameters
base = 0.3  # Wheelbase of the vehicle
lookahead_gain = 0.1   # Look-ahead distance gain
lookahead_min = 1  # Minimum look-ahead distance
distance_threshold = 0.2
alpha_threshold = 0.5

class RobotState:
    """Using odometry message to update the current state of the robot"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.velocity = 0.0

    def update_state(self, odom_msg):
        self.x = odom_msg.poses[-1].pose.position.x
        self.y = odom_msg.poses[-1].pose.position.y

        q = odom_msg.poses[-1].pose.orientation
        t0 = +2.0 * (q.w * q.z + q.x * q.y)
        t1 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(t0, t1)

        self.velocity = 0.3 # Temp fixed velocity

    def distance_to_state(self, x, y):
        return np.hypot(self.x - x, self.y - y)

class TargetPath:
    """Determines the target route and searching of current point to navigate towards"""
    def __init__(self):
        self.x_points = []
        self.y_points = []
        self.old_nearest_point_index = None

    def update_path(self, path_msg):
        self.x_points = [pose.pose.position.x for pose in path_msg.poses]
        self.y_points = [pose.pose.position.y for pose in path_msg.poses]
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

class Navigation(Node):

    def __init__(self):
        super().__init__("navigation")

        self.state = RobotState()
        self.target_path = TargetPath()
        self.previous_index = 0
        self.stamp = None

        self.create_subscription(
                Path,
                "odom_path",
                self.odom_callback,
                10)
        self.create_subscription(
                Path,
                "custom_path",
                self.path_callback,
                10)
        self.motor_publisher = self.create_publisher(DutyCycles, "/motor/duty_cycles", 10)
        self.new_path_client = self.create_client(Trigger, "new_path")
        while not self.new_path_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for service...')
        self.new_path_request()

        self.create_timer(0.05, self.control_loop)


    def odom_callback(self, msg: Path):
        self.state.update_state(msg)
        self.stamp = msg.poses[-1].header.stamp

    def path_callback(self, msg: Path):
        self.target_path.update_path(msg)

    def new_path_request(self):
        request = Trigger.Request()
        future = self.new_path_client.call_async(request)
        future.add_done_callback(self.new_path_callback)

    def new_path_callback(self, future):
        response = future.result()
        if response:
            self.get_logger().info(f"Response: {response.message}")
        else:
            self.get_logger().error("Error - Failed to receive response.")

    def control_loop(self):
        if not self.target_path.x_points:
            self.get_logger().info(f"Error - No target path")
            return

        omega, alpha, self.previous_index = pure_pursuit_control(self.state, self.target_path)
        if DEBUG: self.get_logger().info(f"Velocity: {self.state.velocity}, Steering Angle: {omega:.2f}, Target Index: {self.previous_index}")

        if self.previous_index >= len(self.target_path.x_points) - 1:
            distance = np.hypot(self.state.x - self.target_path.x_points[-1], self.state.y - self.target_path.y_points[-1])
            if distance <= distance_threshold:
                self.get_logger().info(f"Message - Reached destination")
                self.new_path_request()
                return

        velocity = self.state.velocity

        if (alpha > alpha_threshold or alpha < -alpha_threshold):
            velocity = 0

        left_wheel = velocity - (base/2) * omega
        right_wheel = velocity + (base/2) * omega

        if DEBUG: self.get_logger().info(f"Alpha: {alpha:.2f}, Omega: {omega:.2f}, Target Index: {self.previous_index}")

        max_value = max(abs(left_wheel), abs(right_wheel))

        if max_value > 1:
            left_wheel = left_wheel / max_value
            right_wheel = right_wheel / max_value

        max_speed = 0.5
        if left_wheel > max_speed: left_wheel = max_speed
        if left_wheel < -max_speed: left_wheel = -max_speed
        if right_wheel > max_speed: right_wheel = max_speed
        if right_wheel < -max_speed: right_wheel = -max_speed

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
