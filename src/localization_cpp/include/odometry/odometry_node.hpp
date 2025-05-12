#pragma once

// ROS and Node-related includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <robp_interfaces/msg/encoders.hpp>
#include <sensor_msgs/msg/imu.hpp>

// ROS Transform-related includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Math includes
#include <cmath>

// Custom includes
#include "localization/ekf.hpp"

// Function timing
#include <iostream>
#include <chrono>

namespace Localization {

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode();

private:
    void encoderCallback(const robp_interfaces::msg::Encoders::ConstSharedPtr& msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void getEncoderState(double& delta_distance, double& delta_theta, double& linear_velocity, double& angular_velocity, double elapsed_time);
    void updateOdometry();
    void publishOdometry(const rclcpp::Time& current_time);

    // ROS Subscriber
    rclcpp::Subscription<robp_interfaces::msg::Encoders>::SharedPtr encoder_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // ROS Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Transform broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Buffer for transform lookup
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    double ticks_per_revolution_ = 48 * 64;
    double wheel_radius_ = 0.04921;
    double base_width_ = 0.31;

    // Encoder data storage
    rclcpp::Time then_time_;
    int64_t accumulated_ticks_left_ = 0;
    int64_t accumulated_ticks_right_ = 0;
    int64_t last_encoder_left_ = 0;
    int64_t last_encoder_right_ = 0;

    // IMU data storage
    geometry_msgs::msg::Vector3 angular_velocity_imu_;
    geometry_msgs::msg::Vector3 linear_acceleration_imu_;
    geometry_msgs::msg::Quaternion orientation_imu_;
    rclcpp::Time last_imu_time_;
    bool imu_data_received_ = false;

    // EKF and state variables
    EKF ekf_;

    // Path storage
    nav_msgs::msg::Path odom_path_;

    // Store latest odom to map transform
    geometry_msgs::msg::TransformStamped odom_to_map_;
};

} // namespace Localization