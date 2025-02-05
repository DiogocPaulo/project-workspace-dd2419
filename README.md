# Robotics Project Group 4
ROS2 workspace for robotics project (DD2419). Group 4 with robot **Sneezy**!

# Arm controll
ros2 launch pick_up pick_up_launch.py

# Arm Camera
ros2 launch robp_launch arm_camera_launch.yaml

Rviz instructions:
- Add
- By topic
- Arm_camera/image_raw -> image

# Detection


Either (with odom):
ros2 launch robp_launch frames_launch.xml
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id odom
or (no odom)
ros2 launch robp_launch frames_launch.xml
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link

ros2 launch robp_launch rs_d435i_launch.py (launch the camera)
    ros2 launch detection detection_launch.py

Rviz instructions:
- Add
- By topic
- Arm_camera/image_raw -> image