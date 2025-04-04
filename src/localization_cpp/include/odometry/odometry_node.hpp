#pragma once

// ROS and Node-related includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <robp_interfaces/msg/encoders.hpp>

// ROS Transform-related includes
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Math includes
#include <cmath>

namespace Localization {

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode();

private:
    // Timer callback for odometry update
    void updateOdometry();

    // Encoder callback
    void encoderCallback(const robp_interfaces::msg::Encoders::ConstSharedPtr& msg);

    // ROS Subscriber
    rclcpp::Subscription<robp_interfaces::msg::Encoders>::SharedPtr encoder_sub_;

    // ROS Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Transform broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Parameters
    double ticks_per_revolution_ = 48 * 64;
    double wheel_radius_ = 0.04921;
    double base_width_ = 0.31;

    // Internal variables
    rclcpp::Time then_time_;
    int64_t accumulated_ticks_left_ = 0;
    int64_t accumulated_ticks_right_ = 0;
    int64_t last_encoder_left_ = 0;
    int64_t last_encoder_right_ = 0;
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    // Path storage
    nav_msgs::msg::Path odom_path_;
};

} // namespace Localization