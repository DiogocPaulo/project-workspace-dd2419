#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=["serial", "--dev", "/dev/ttyUSB1", "-v6"] # Changed to 1
        ),
        Node(
            package='pick_up',
            executable='ArmCameraNode',
            name='pick_up_camera',
            output='screen',
        ),
    ])

