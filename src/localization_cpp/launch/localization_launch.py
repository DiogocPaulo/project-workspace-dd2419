# Launches odometry node (requires setup_launch to be run first)

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    frames_launch_file = os.path.join(
        get_package_share_directory("robp_launch"), "launch", "frames_launch.xml"
    ) 
    return LaunchDescription([
        IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(frames_launch_file),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--qx", "0",
                "--qy", "0",
                "--qz", "0",
                "--qw", "1",
                "--frame-id", "map",
                "--child-frame-id", "odom"
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "0",
                "--y", "0.085",
                "--z", "0.155",
                "--qx", "0",
                "--qy", "0",
                "--qz", "0",
                "--qw", "1",
                "--frame-id", "base_link",
                "--child-frame-id", "lidar_link"
            ],
            output="screen",
        ),
        Node(
            package='localization_cpp',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),
        # Node(
        #     package='localization_cpp',
        #     executable='localization_node',
        #     name='localization_node',
        #     output='screen',
        # ),
    ])
