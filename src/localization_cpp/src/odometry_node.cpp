#include "odometry/odometry_node.hpp"

namespace Localization {

OdometryNode::OdometryNode()
    : Node("odometry"),
      then_time_(this->get_clock()->now()) {
    // Initialize subscriber
    encoder_sub_ = this->create_subscription<robp_interfaces::msg::Encoders>(
        "/motor/encoders", 10,
        std::bind(&OdometryNode::encoderCallback, this, std::placeholders::_1));

    // Initialize publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/odom_path", 10);

    // Initialize timer (0.1s period = 10 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&OdometryNode::updateOdometry, this));

    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize path header
    odom_path_.header.frame_id = "odom";

    RCLCPP_INFO(this->get_logger(), "OdometryNode initialized.");
}

void OdometryNode::updateOdometry() {
    rclcpp::Time now_time = this->get_clock()->now();
    double elapsed_time = (now_time - then_time_).nanoseconds() / 1e9;
    then_time_ = now_time;

    // Consume and reset accumulated ticks
    int64_t ticks_left = accumulated_ticks_left_;
    int64_t ticks_right = accumulated_ticks_right_;
    accumulated_ticks_left_ = 0;
    accumulated_ticks_right_ = 0;

    double distance_left = (ticks_left / ticks_per_revolution_) * (2 * M_PI * wheel_radius_);
    double distance_right = (ticks_right / ticks_per_revolution_) * (2 * M_PI * wheel_radius_);

    // Calculate odometry
    double delta_distance = (distance_left + distance_right) / 2;
    double delta_theta = (distance_right - distance_left) / base_width_;

    if (delta_distance != 0) {
        double delta_x = delta_distance * std::cos(theta_ + (delta_theta / 2));
        double delta_y = delta_distance * std::sin(theta_ + (delta_theta / 2));
        x_ += delta_x;
        y_ += delta_y;
    }
    if (delta_theta != 0) {
        theta_ += delta_theta;
    }

    // Calculate velocities
    linear_velocity_ = (elapsed_time > 0) ? delta_distance / elapsed_time : 0.0;
    angular_velocity_ = (elapsed_time > 0) ? delta_theta / elapsed_time : 0.0;

    // Publish odometry transform
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, theta_);

    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = now_time;
    transform_msg.header.frame_id = "odom";
    transform_msg.child_frame_id = "base_link";
    transform_msg.transform.translation.x = x_;
    transform_msg.transform.translation.y = y_;
    transform_msg.transform.translation.z = 0.0;
    transform_msg.transform.rotation = tf2::toMsg(quaternion);

    tf_broadcaster_->sendTransform(transform_msg);

    // Publish odometry
    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.header.stamp = now_time;
    odometry_msg.header.frame_id = "odom";
    odometry_msg.child_frame_id = "base_link";
    odometry_msg.pose.pose.position.x = x_;
    odometry_msg.pose.pose.position.y = y_;
    odometry_msg.pose.pose.position.z = 0.0;
    odometry_msg.pose.pose.orientation = tf2::toMsg(quaternion);
    odometry_msg.twist.twist.linear.x = linear_velocity_;
    odometry_msg.twist.twist.linear.y = 0.0;
    odometry_msg.twist.twist.angular.z = angular_velocity_;

    odom_pub_->publish(odometry_msg);

    // Publish odometry path
    odom_path_.header.stamp = now_time;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom_path_.header;
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.position.z = 0.01; // Slightly above ground for visualization
    pose.pose.orientation = tf2::toMsg(quaternion);

    odom_path_.poses.push_back(pose);
    path_pub_->publish(odom_path_);
}

void OdometryNode::encoderCallback(const robp_interfaces::msg::Encoders::ConstSharedPtr& msg) {
    // Accumulate ticks based on delta of measured ticks
    int64_t delta_left = msg->encoder_left - last_encoder_left_;
    int64_t delta_right = msg->encoder_right - last_encoder_right_;
    accumulated_ticks_left_ += delta_left;
    accumulated_ticks_right_ += delta_right;
    last_encoder_left_ = msg->encoder_left;
    last_encoder_right_ = msg->encoder_right;
}

} // namespace Localization

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Localization::OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}