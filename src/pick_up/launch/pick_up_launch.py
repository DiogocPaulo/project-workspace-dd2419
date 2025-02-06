#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='pick_up',
            executable='talker',
            name='move_servos_publisher',
            # output='screen',
        ),
        Node(
            package='pick_up',
            executable='listener',
            name='servo_pos_subscriber',
            # output='screen',
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=["serial", "--dev", "/dev/ttyUSB1", "-v6"] # Changed to 1
        ),
    ])

"""Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=["serial", "--dev", "/dev/ttyUSB1", "-v6"] # Changed to 1
        ),
        
        
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=["serial", "--dev", "/dev/ttyUSB1", "-v6"] # Changed to 1
        ),"""