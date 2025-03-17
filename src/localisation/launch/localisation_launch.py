# Launches odometry node (requires setup_launch to be run first)

from launch import LaunchDescription
from launch_ros.actions import Node

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
            arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
            output="screen",
        ),
        Node(
            package="localisation",
            executable="odometry",
            name="robot_odometry",
            output="screen",
        ),
    ])
