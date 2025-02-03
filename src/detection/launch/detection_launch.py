#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([   
        Node(
            package='detection',
            executable='detection',
            name='detection',
            # output='screen',
        ),
    ])