from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node

<<<<<<< HEAD
def generate_launch_description():
=======
def generate_launch_description(): 
>>>>>>> 90022044b423cea3560868e28cf4fb0190b427ed
    frames_launch_file = os.path.join(
        get_package_share_directory("robp_launch"), "launch", "frames_launch.xml"
    ) 
    lidar_launch_file = os.path.join(
        get_package_share_directory("robp_launch"), "launch", "lidar_launch.yaml"
    ) 
    return LaunchDescription([
        IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(frames_launch_file),
        ),
        IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(lidar_launch_file),
        ),
<<<<<<< HEAD
        Node(
            package="lidar",
            executable="lidar",
            name="lidar",
        ),
=======
>>>>>>> 90022044b423cea3560868e28cf4fb0190b427ed
    ])

