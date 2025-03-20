# Launches pathing and navigation

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="project_master",
            executable="explore_master",
            name="robot_master",
            emulate_tty=True
            ros_arguments=["--log-level", "info"]
        ),
        Node(
            package="navigation",
            executable="navigation",
            name="robot_navigation",
            emulate_tty=True
            ros_arguments=["--log-level", "warn"]
        ),
        Node(
            package="navigation",
            executable="mapping",
            name="robot_mapping",
            emulate_tty=True
            ros_arguments=["--log-level", "warn"]
        ),
        Node(
            package="navigation",
            executable="pathing",
            name="robot_pathing",
            emulate_tty=True
            ros_arguments=["--log-level", "info"]
        ),
        Node(
            package="detection",
            executable="detection",
            name="robot_detection",
            emulate_tty=True
            ros_arguments=["--log-level", "warn"]
        ),
    ])
