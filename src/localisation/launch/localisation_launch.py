# Launches odometry node (requires setup_launch to be run first)

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="localisation",
            executable="odometry",
            name="robot_odometry",
            output="screen",
        ),
    ])
