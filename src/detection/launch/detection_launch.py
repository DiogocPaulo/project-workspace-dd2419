#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node

def generate_launch_description():
    frames_launch_file = os.path.join(
        get_package_share_directory("robp_launch"), "launch", "frames_launch.xml"
    )
    camera_launch_file = os.path.join(
        get_package_share_directory("robp_launch"), "launch", "rs_d435i_launch.py"
    )
    return LaunchDescription([   
        IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(frames_launch_file),
        ),
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(camera_launch_file),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "1", "map", "base_link"],
            output="screen",
        ),
        Node(
            package='detection',
            executable='detection',
            name='robot_detection',
        ),
    ])
