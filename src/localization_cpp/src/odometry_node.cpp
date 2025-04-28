#include "odometry/odometry_node.hpp"

namespace Localization {

OdometryNode::OdometryNode() : Node("odometry"), then_time_(this->get_clock()->now()) {
    // Initialize subscribers
    encoder_sub_ = this->create_subscription<robp_interfaces::msg::Encoders>(
        "/motor/encoders", 10,
        std::bind(&OdometryNode::encoderCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_raw", 10,
        std::bind(&OdometryNode::imuCallback, this, std::placeholders::_1));

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

    // Initialize EKF instance
    ekf_ = EKF();

    RCLCPP_INFO(this->get_logger(), "OdometryNode initialized.");
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

void OdometryNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    // Store IMU angular velocity
    angular_velocity_imu_ = msg->angular_velocity;
    
    // Store IMU linear acceleration
    linear_acceleration_imu_ = msg->linear_acceleration;
    
    // Store IMU orientation
    orientation_imu_ = msg->orientation;
    
    // Update timestamp
    last_imu_time_ = msg->header.stamp;
    
    // Mark that we've received IMU data
    imu_data_received_ = true;
}

void OdometryNode::getEncoderState(double& delta_distance, double& delta_theta, double& linear_velocity, double& angular_velocity, double elapsed_time) {
    // Consume and reset accumulated ticks
    int64_t ticks_left = accumulated_ticks_left_;
    int64_t ticks_right = accumulated_ticks_right_;
    accumulated_ticks_left_ = 0;
    accumulated_ticks_right_ = 0;

    // Calculate distance moved by each wheel
    double distance_left = (ticks_left / ticks_per_revolution_) * (2 * M_PI * wheel_radius_);
    double distance_right = (ticks_right / ticks_per_revolution_) * (2 * M_PI * wheel_radius_);

    // Calculate delta distance and delta angle
    delta_distance = (distance_left + distance_right) / 2.0;
    delta_theta = (distance_right - distance_left) / base_width_;

    // Calculate velocities
    linear_velocity = delta_distance / elapsed_time;
    angular_velocity = delta_theta / elapsed_time;
}

void OdometryNode::updateOdometry() {
    rclcpp::Time now_time = this->get_clock()->now();
    double elapsed_time = (now_time - then_time_).nanoseconds() / 1e9;
    then_time_ = now_time;

    // Get encoder state
    double delta_distance, delta_theta, linear_velocity, angular_velocity;
    getEncoderState(delta_distance, delta_theta, linear_velocity, angular_velocity, elapsed_time);
    
    // Predict the state based on encoder data
    ekf_.predict(delta_distance, 0.0, delta_theta, linear_velocity, angular_velocity, elapsed_time);

    // For debugging purposes print the state
    Eigen::VectorXd state = ekf_.getState();
    RCLCPP_INFO(this->get_logger(), "State: x=%.2f, y=%.2f, theta=%.2f, v=%.2f, w=%.2f", state(0), state(1), state(2), state(3), state(4));

    // Correct the state using IMU data (skipping correction == only using encoder data, i.e previous odometry node)
    if (imu_data_received_) { // (now_time - last_imu_time_).nanoseconds() / 1e9 < 0.1 <-- Synchronize with IMU data
        //RCLCPP_INFO(this->get_logger(), "Encoder vs IMU angular velocity: %.2f vs %.2f", angular_velocity, -angular_velocity_imu_.z);

        // Correct the state based on IMU data
        //ekf_.correct(-angular_velocity_imu_.z, elapsed_time); // <-- negative to match encoder data

        // Reset IMU data received flag
        imu_data_received_ = false;

        // For debugging purposes print the corrected state
        state = ekf_.getState();
        RCLCPP_INFO(this->get_logger(), "Corrected State: x=%.2f, y=%.2f, theta=%.2f, v=%.2f, w=%.2f", state(0), state(1), state(2), state(3), state(4));
    }

    // Publish the updated odometry based on the current state
    publishOdometry(now_time);
}


void OdometryNode::publishOdometry(const rclcpp::Time& current_time) {
    // Get the current state from the EKF (x, y, theta, v, w)
    Eigen::VectorXd state = ekf_.getState();
    double x = state(0);
    double y = state(1);
    double theta = state(2);
    double v = state(3);  // Linear velocity
    double w = state(4);  // Angular velocity

    // Create quaternion from orientation
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, theta);

    // Publish odometry transform
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = current_time;
    transform_msg.header.frame_id = "odom";
    transform_msg.child_frame_id = "base_link";
    transform_msg.transform.translation.x = x;
    transform_msg.transform.translation.y = y;
    transform_msg.transform.translation.z = 0.0;
    transform_msg.transform.rotation = tf2::toMsg(quaternion);
    tf_broadcaster_->sendTransform(transform_msg);

    // Publish odometry message
    nav_msgs::msg::Odometry odometry_msg;
    odometry_msg.header.stamp = current_time;
    odometry_msg.header.frame_id = "odom";
    odometry_msg.child_frame_id = "base_link";
    odometry_msg.pose.pose.position.x = x;
    odometry_msg.pose.pose.position.y = y;
    odometry_msg.pose.pose.position.z = 0.0;
    odometry_msg.pose.pose.orientation = tf2::toMsg(quaternion);
    odometry_msg.twist.twist.linear.x = v;
    odometry_msg.twist.twist.linear.y = 0.0;
    odometry_msg.twist.twist.angular.z = w;
    odom_pub_->publish(odometry_msg);

    // Publish odometry path
    odom_path_.header.stamp = current_time;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom_path_.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.01; // Slightly above ground for visualization
    pose.pose.orientation = tf2::toMsg(quaternion);
    odom_path_.poses.push_back(pose);
    path_pub_->publish(odom_path_);
}

} // namespace Localization

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Localization::OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}