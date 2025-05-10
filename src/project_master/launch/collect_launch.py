# Launches nodes for collection phase

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node

def generate_launch_description():
    mapping_launch_file = os.path.join(
        get_package_share_directory("mapping"), "launch", "mapping_launch.py"
    ) 
    return LaunchDescription([
        Node(
            package="project_master",
            executable="collect_master",
            name="robot_collect",
            emulate_tty=True,
            ros_arguments=["--log-level", "info"]
        ),
        IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(mapping_launch_file),
        ),
        Node(
            package="navigation", #aka folder
            executable="pathing", #aka py file
            name="robot_pathing",
            emulate_tty=True,
            ros_arguments=["--log-level", "info"]
        ),
        Node(
            package="navigation",
            executable="navigation",
            name="robot_navigation",
            emulate_tty=True,
            ros_arguments=["--log-level", "warn"]
        ),
        Node(
            package="detection",
            executable="object_filter_node",
            name="robot_detection_filter",
            emulate_tty=True,
            ros_arguments=["--log-level", "info"]
        ),
        Node(
            package='pick_up',
            executable='arm_camera',
            name='Arm_Camera',
            output='screen',
            ros_arguments=["--log-level", "warn"]
        ),
        Node(
            package='pick_up',
            executable='talker',
            name='move_servos_publisher',
            # output='screen',
        ),
        Node(
            package='pick_up',
            executable='listener',
            name='servo_pos_subscriber',
            # output='screen',
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            # output='screen',
            arguments=["serial", "--dev", "/dev/ttyUSB1", "-v6"] # Changed to 1
        ),
    ])
