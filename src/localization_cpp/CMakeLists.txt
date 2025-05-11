cmake_minimum_required(VERSION 3.8)
project(localization_cpp)

# Set C++ standard to 17 if not already set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Policies for CMake compatibility
cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0144 NEW)

# Compile options: C++17 with optimizations
add_compile_options(-std=c++17 -O3 -march=native)

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robp_interfaces REQUIRED)  # For OdometryNode's Encoders msg

# Find Open3D (required for ICP in localization_node)
find_package(Open3D REQUIRED)

# Optional: PCL for point cloud handling (comment out if not needed)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include directories with subfolders per node
include_directories(
  include  # For shared headers
  include/localization_node
  include/odometry_node
  ${PCL_INCLUDE_DIRS}  # For PCL, if used
)

# Add executable for localization_node
add_executable(localization_node src/localization_node.cpp)
ament_target_dependencies(localization_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_eigen
  Open3D  # For ICP
  PCL  # For PointCloud handling (optional)
  pcl_conversions
)
target_link_libraries(localization_node Open3D::Open3D ${PCL_LIBRARIES})

# Add executable for odometry_node
add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  robp_interfaces  # For Encoders msg
)

# Install executables
install(TARGETS
  localization_node
  odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and include directories with subfolders
install(DIRECTORY
  launch
  include/  # Installs all subfolders recursively
  DESTINATION share/${PROJECT_NAME}/
)

# Generate package configuration
ament_package()