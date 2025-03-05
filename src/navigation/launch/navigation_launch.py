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
            executable="project_master",
            name="robot_master",
        ),
        Node(
            package="navigation",
            executable="navigation",
            name="robot_navigation",
        ),
        Node(
            package="navigation",
            executable="mapping",
            name="robot_mapping",
        ),
        Node(
            package="navigation",
            executable="pathing",
            name="robot_pathing",
        ),
        Node(
            package='detection',
            executable='detection',
            name='robot_detection',
        ),
    ])
