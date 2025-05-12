# Launches phidgets and sets default frames

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node

def generate_launch_description():
    phidgets_launch_file = os.path.join(
        get_package_share_directory("robp_launch"), "launch", "phidgets_launch.py"
    ) 
    camera_launch_file = os.path.join(
        get_package_share_directory("robp_launch"), "launch", "rs_d435i_launch.py"
    )
    lidar_launch_file = os.path.join(
        get_package_share_directory("robp_launch"), "launch", "lidar_launch.yaml"
    ) 
    arm_camera_launch_file = os.path.join(
        get_package_share_directory("robp_launch"), "launch", "arm_camera_launch.yaml"
    ) 
    return LaunchDescription([
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(phidgets_launch_file),
        ),
        IncludeLaunchDescription(
           launch_description_source=PythonLaunchDescriptionSource(camera_launch_file),
        ),
        IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(lidar_launch_file),
        ),
        IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(arm_camera_launch_file),
        ),
    ])
