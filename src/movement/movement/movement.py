#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header

from robp_interfaces.msg import DutyCycles
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped

class Movement(Node):

    def __init__(self):
        super().__init__("movement")

        self.motor_publisher = self.create_publisher(DutyCycles, "/motor/duty_cycles", 10)

        self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.joystick_callback,
            10)


    def joystick_callback(self, msg):
        dutyCycles = DutyCycles()
        dutyCycles.header = msg.header
        left_wheel = msg.twist.linear.x - msg.twist.angular.z
        right_wheel = msg.twist.linear.x + msg.twist.angular.z
        max_speed = 0.3
        if left_wheel > max_speed: left_wheel = max_sped
        if left_wheel < -max_speed: left_wheel = -max_speed
        if right_wheel > max_speed: right_wheel = max_speed
        if right_wheel < -max_speed: right_wheel = -max_speed
        dutyCycles.duty_cycle_left = left_wheel
        dutyCycles.duty_cycle_right = right_wheel
        self.motor_publisher.publish(dutyCycles)
        self.get_logger().info(f"Time [{msg.header.stamp.sec}] \n\tLinear ({msg.twist.linear.x}, {msg.twist.linear.y}, {msg.twist.linear.z}) \n\tAngular ({msg.twist.angular.x}, {msg.twist.angular.y}, {msg.twist.angular.z})")

def main():
    rclpy.init()
    node = Movement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
