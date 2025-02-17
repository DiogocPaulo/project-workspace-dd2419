# Robotics Project Group 4
ROS2 workspace for robotics project (DD2419). Group 4 with robot **Sneezy**!

# Localisation and Navigation Steps
Have to launch phidgets before Localisation and navigation. Thus for movement do the following:
1. Launch phidgets and frames `make run-setup`
2. Launch localisation includes odometry `make run-localisation`
3. Launch navigation method `make run-navigation` for pure pursuit or `make run-joystick`

# Arm control
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

or (no odom):
ros2 launch robp_launch frames_launch.xml
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link

ros2 launch robp_launch rs_d435i_launch.py (launch the camera)
ros2 launch detection detection_launch.py

Rviz instructions:
- Add
- By topic
- camera_depth/color/points_transformed -> PointCloud2

# Lidar (probably since we havent been able to test it yet)

ros2 launch robp_launch lidar_launch.yaml

Rviz instructions:
- Add
- By topic
- laser_scan

(make sure fixed frame is lidar_link)
