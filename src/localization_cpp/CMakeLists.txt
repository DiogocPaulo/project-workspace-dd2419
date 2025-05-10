cmake_minimum_required(VERSION 3.8)
project(localization_cpp)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0144 NEW)

add_compile_options(-std=c++17 -O3 -march=native)

# ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robp_interfaces REQUIRED)

# Optional packages
find_package(Open3D REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# G2O - manual search
find_path(G2O_INCLUDE_DIR g2o/core/sparse_optimizer.h
  PATHS /usr/include /usr/local/include /usr/include/eigen3)

find_library(G2O_CORE_LIBRARY g2o_core
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)

find_library(G2O_STUFF_LIBRARY g2o_stuff
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)

find_library(G2O_SOLVER_CHOLMOD_LIBRARY g2o_solver_cholmod
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)

find_library(G2O_TYPES_SLAM3D_LIBRARY g2o_types_slam3d
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)

if(NOT G2O_INCLUDE_DIR OR NOT G2O_CORE_LIBRARY OR NOT G2O_STUFF_LIBRARY OR NOT G2O_SOLVER_CHOLMOD_LIBRARY OR NOT G2O_TYPES_SLAM3D_LIBRARY)
  message(FATAL_ERROR "Could not find all required G2O components")
endif()

include_directories(
  include
  include/localization_node
  include/odometry_node
  ${PCL_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
)

# localization_node
add_executable(localization_node src/localization_node.cpp)
ament_target_dependencies(localization_node
  rclcpp tf2 tf2_ros tf2_geometry_msgs
  nav_msgs sensor_msgs geometry_msgs tf2_eigen
  Open3D PCL pcl_conversions
)

target_link_libraries(localization_node
  Open3D::Open3D
  ${PCL_LIBRARIES}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_CHOLMOD_LIBRARY}
  ${G2O_TYPES_SLAM3D_LIBRARY} 
)

# odometry_node
add_executable(odometry_node src/odometry_node.cpp)
ament_target_dependencies(odometry_node
  rclcpp tf2 tf2_ros tf2_geometry_msgs
  nav_msgs sensor_msgs geometry_msgs robp_interfaces
)

# Install targets
install(TARGETS
  localization_node
  odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  include/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
