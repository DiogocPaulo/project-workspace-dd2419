import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to URDF file
    urdf_file = os.path.join(get_package_share_directory('pick_up'), 'urdf', 'Arm.urdf')

    with open(urdf_file, 'r') as file:
        urdf_content = file.read()

    return LaunchDescription([
        # Publish the robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
        ),

        # GUI for moving joints (if applicable)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Launch RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('pick_up'), 'rviz', 'my_robot.rviz')],
            output='screen'
        )
    ])
