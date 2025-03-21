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
            ros_arguments=["--log-level", "error"]
        ),
        Node(
            package="navigation",
            executable="navigation",
            name="robot_navigation",
            ros_arguments=["--log-level", "error"]
        ),
        Node(
            package="navigation",
            executable="mapping",
            name="robot_mapping",
            ros_arguments=["--log-level", "error"]
        ),
        Node(
            package="navigation",
            executable="pathing",
            name="robot_pathing",
            ros_arguments=["--log-level", "error"]
        ),
        Node(
            package="detection",
            executable="detection",
            name="robot_detection",
            ros_arguments=["--log-level", "info"]
        ),
    ])
