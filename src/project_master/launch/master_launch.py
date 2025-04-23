import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Log to check if launch file is being executed
        # LogInfo(condition=LaunchConfiguration(msg="Master Launch is starting!")),
        Node(
            package='project_master',
            executable='project_master',
            name='project_master',
            # output='screen',
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
        # Node(
        #     package='pick_up',
        #     executable='ArmCameraNode',
        #     name='ArmNode',
        #     output='screen',
        # ),
        # Node(
        #     package='micro_ros_agent',
        #     executable='micro_ros_agent',
        #     name='micro_ros_agent',
        #     # output='screen',
        #     arguments=["serial", "--dev", "/dev/ttyUSB1", "-v6"] # Changed to 1
        # ),
        # Node(
        #     package="navigation",
        #     executable="navigation",
        #     name="robot_navigation",
        # ),
        # Node(
        #     package="navigation",
        #     executable="mapping",
        #     name="robot_mapping",
        # ),
        # Node(
        #     package="navigation",
        #     executable="pathing",
        #     name="robot_pathing",
        # ),
    ])
