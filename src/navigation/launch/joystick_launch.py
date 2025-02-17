# Launches joystick based movement

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node

def generate_launch_description():
    joystick_launch_file = os.path.join(
        get_package_share_directory("teleop_twist_joy"), "launch", "teleop-launch.py"
    ) 
    joystick_config_filepath = os.path.join(
        get_package_share_directory("navigation"), "config", "gamepad.yaml"
    ) 
    return LaunchDescription([
        Node(
            package="navigation",
            executable="joystick",
            name="robot_joystick",
        ),
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(joystick_launch_file),
            launch_arguments={
                "config_filepath": joystick_config_filepath,
                "publish_stamped_twist": "true",
            }.items(),
        ),
    ])
