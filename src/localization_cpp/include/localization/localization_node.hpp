#pragma once

// ROS and Node-related includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>  // PointCloud2 message
#include <geometry_msgs/msg/transform_stamped.hpp>

// ROS Transform-related includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/msg/point_cloud2.hpp>  // PointCloud2 message
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Math and Linear Algebra includes#include <sensor_msgs/msg/point_cloud2.hpp>  // PointCloud2 message
#include <Eigen/Dense>

// C++ Standard and Memory management includes
#include <array>
#include <memory>

// Custom includes
#include "localization/scan_processing.hpp"
#include "localization/lidar_scan_storage.hpp"
#include "localization/icp.hpp"

// PCL (Point Cloud Library) includes for point cloud processing
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace Localization {

class Node : public rclcpp::Node {
public:
    Node();

private:
    // Callbacks for sensor data
    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

    // Timed publishing of transforms
    void publishTransform();
    void publishPointCloud(const std::vector<Eigen::Vector2d>& points);

    // ROS Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Timer for periodic updates
    rclcpp::TimerBase::SharedPtr timer_;

    // Transform broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Buffer for transform lookup
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Transform state
    std::array<double, 3> translation_;
    tf2::Quaternion rotation_;
    double transform_z_;

    // Timestamping
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time time_stamp_;

    // Localization components
    LidarScanStorage scan_storage_;   // Store LiDAR scans in a grid map
    ICP icp_;
    Pose2D current_pose_;
    double linear_velocity_;
    double angular_velocity_;
    
    // Publisher for the point cloud
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

} // namespace Localization