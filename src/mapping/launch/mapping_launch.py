# Launch nodes relating to mapping

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
            package="mapping",
            executable="map_workspace",
            name="robot_map_workspace",
            ros_arguments=["--log-level", "info"]
        ),
        Node(
            package="mapping",
            executable="map_objects",
            name="robot_map_objects",
            ros_arguments=["--log-level", "info"]
        ),
        Node(
            package="mapping",
            executable="map_obstacles",
            name="robot_map_obstacles",
            ros_arguments=["--log-level", "info"]
        ),
    ])
