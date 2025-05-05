#include "localization/localization_node.hpp"

namespace Localization {

Node::Node() : rclcpp::Node("localization_node") {
    // Initialize subscribers
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS().keep_last(1), std::bind(&Node::scanCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS().keep_last(1), std::bind(&Node::odomCallback, this, std::placeholders::_1));

    // Initialize the point cloud publisher
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("reference_scan_cloud", 10);
    all_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("all_reference_scan_cloud", 10);

    // Initialize timer
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&Node::publishTransform, this));

    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize transform buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize transform state
    translation_ = {0.0, 0.0, 0.0};
    rotation_.setRPY(0.0, 0.0, 0.0);
    transform_z_ = 0.0;

    // Initialize LidarScanStorage and ICP
    scan_storage_ = LidarScanStorage(0.8); // grid size
    icp_ = ICP(0.3, 100); // 10 cm threshold, 50 iterations

    RCLCPP_INFO(this->get_logger(), "Localization node initialized.");
}

void Node::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
    if (std::abs(angular_velocity_) > 0.1) {
        RCLCPP_DEBUG(this->get_logger(), "Robot is rotating too fast, skipping scan processing.");
        return; // Skip processing if robot is not moving
    }

    // Ensure TF buffer has data
    if (!tf_buffer_->canTransform("odom", msg->header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(500))
        || !tf_buffer_->canTransform("map", "odom", tf2::TimePointZero, std::chrono::milliseconds(500))
        || !tf_buffer_->canTransform("odom", "map", tf2::TimePointZero, std::chrono::milliseconds(500))) {
        RCLCPP_WARN(this->get_logger(), "Transform from %s to map not available yet!", msg->header.frame_id.c_str());
        return;
    }
    try {
        // Get the transform from Lidar frame to Odom frame
        RCLCPP_DEBUG(this->get_logger(), "Transform exists from %s to map.", msg->header.frame_id.c_str());
        geometry_msgs::msg::TransformStamped lidar_to_odom = tf_buffer_->lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
        transform_z_ = lidar_to_odom.transform.translation.z;

        // Get the transform from Map frame to Odom frame
        geometry_msgs::msg::TransformStamped map_to_odom = tf_buffer_->lookupTransform("odom", "map", tf2::TimePointZero);

        // Get the transform from Map frame to Odom frame
        geometry_msgs::msg::TransformStamped odom_to_map = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);

        // Convert LaserScan to 2D points in Map frame
        std::vector<Eigen::Vector2d> points = laserScanToPoints(msg, time_stamp_, linear_velocity_, angular_velocity_);
        transformPoints(points, lidar_to_odom);

        // Get stored scan data from LidarScanStorage
        auto stored_scan = scan_storage_.getClosestScan(current_pose_);

        if (stored_scan) {
            // Transform stored points to the Odom frame
            std::vector<Eigen::Vector2d> stored_points = stored_scan->points;
            transformPoints(stored_points, map_to_odom);

            // Align Principal Components before ICP
            Eigen::Matrix2d pre_rotation = computeRotationAlignment(points, stored_points);
            std::vector<Eigen::Vector2d> pre_aligned_points(points.size());
            for (size_t i = 0; i < points.size(); ++i) {
                pre_aligned_points[i] = pre_rotation * points[i];
            }

            // Perform ICP to find transformation and get aligned points
            std::vector<Eigen::Vector2d> aligned_points;
            Eigen::Matrix3d icp_transform;
            icp_.setTarget(stored_points);

            auto start_time = std::chrono::high_resolution_clock::now();
            icp_.computeICP(pre_aligned_points, icp_transform, aligned_points);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            RCLCPP_INFO(this->get_logger(), "ICP computation took %ld ms", duration);

            // Extract translation and rotation from ICP transform
            double icp_translation_x = icp_transform(0, 2);
            double icp_translation_y = icp_transform(1, 2);
            double icp_rotation_theta = std::atan2(icp_transform(1, 0), icp_transform(0, 0));

            // Skip invalid icp
            if (std::abs(icp_translation_x) > 0.4 || std::abs(icp_translation_y) > 0.4 || std::abs(icp_rotation_theta) > M_PI / 4) {
                return;
            }

            // Convert pre_rotation to theta (angle) for quaternion
            double pre_rotation_theta = std::atan2(pre_rotation(1, 0), pre_rotation(0, 0));

            // Convert rotations to quaternions
            tf2::Quaternion pre_rotation_quat, icp_rotation_quat;
            pre_rotation_quat.setRPY(0.0, 0.0, pre_rotation_theta);
            icp_rotation_quat.setRPY(0.0, 0.0, icp_rotation_theta);

            // Update the current transform
            // Combine translations
            translation_[0] += icp_translation_x;
            translation_[1] += icp_translation_y;

            // Combine rotations: current_rotation = icp_rotation * pre_rotation * current_rotation
            //rotation_ = icp_rotation_quat * rotation_;
            rotation_ = icp_rotation_quat * pre_rotation_quat * rotation_;

            // Store the current scan in the LidarScanStorage
            if (std::abs(angular_velocity_) < 0.1 && std::abs(linear_velocity_) < 0.01) { // (current_pose_.position - stored_scan->pose.position).norm() > 0.1 || 
                //stored_scan->agePoints(10); // Age points in the storage
                transformPoints(aligned_points, odom_to_map); // Transform back to map frame
                scan_storage_.addScan(aligned_points, current_pose_);
            }

            // Publish the reference point cloud
            publishPointCloud(stored_points);
            publishAllScans(map_to_odom);

        } else if (std::abs(linear_velocity_) < 0.01) {
            RCLCPP_WARN(this->get_logger(), "No stored scan found for ICP.");
            // Store the current scan in the LidarScanStorage
            scan_storage_.addScan(points, current_pose_);
        }

    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to map: %s", msg->header.frame_id.c_str(), ex.what());
        return; // Skip processing if transform is unavailable
    }
}

void Node::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
    // Store the timestamp
    time_stamp_ = msg->header.stamp;

    // Extract position from the odometry message
    double odom_x = msg->pose.pose.position.x;
    double odom_y = msg->pose.pose.position.y;
    double odom_z = msg->pose.pose.position.z; // Assuming you're handling 3D positions

    // Convert the odometry's quaternion orientation to tf2::Quaternion
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);

    // Apply the transform to convert the pose from the "odom" frame to the "map" frame
    tf2::Vector3 odom_position(odom_x, odom_y, odom_z);

    // Convert translation_ (std::array<double, 3>) into tf2::Vector3
    tf2::Vector3 translation_vector(translation_[0], translation_[1], translation_[2]);

    // Rotate the position using the quaternion rotation_
    tf2::Vector3 transformed_position = tf2::quatRotate(rotation_, odom_position);

    // Now add the translation vector
    transformed_position += translation_vector;  // Adds translation after rotation

    // Update current_pose_ with the transformed position
    current_pose_.position.x() = transformed_position.x();
    current_pose_.position.y() = transformed_position.y();
    
    // Apply rotation (rotation_ is the map-to-odom transform)
    tf2::Quaternion transformed_rotation = rotation_ * quat;
    
    // Store the theta (orientation angle) if needed (from the rotated quaternion)
    current_pose_.theta = transformed_rotation.getAngle();

    // Store the linear and angular velocities
    linear_velocity_ = msg->twist.twist.linear.x;
    angular_velocity_ = msg->twist.twist.angular.z;
}


void Node::publishTransform() {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = time_stamp_;
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "odom";
    tf_msg.transform.translation.x = translation_[0];
    tf_msg.transform.translation.y = translation_[1];
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation = tf2::toMsg(rotation_);

    tf_broadcaster_->sendTransform(tf_msg);
}

void Node::publishPointCloud(const std::vector<Eigen::Vector2d>& points) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    for (const auto& point : points) {
        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x();
        colored_point.y = point.y();
        colored_point.z = transform_z_;
        colored_point.r = 255; // Red
        colored_point.g = 165; // Orange
        colored_point.b = 0;   // Blue
        cloud.points.push_back(colored_point);
    }
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1; // Unorganized
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "odom";
    cloud_msg.header.stamp = time_stamp_;
    cloud_pub_->publish(cloud_msg);
}

void Node::publishAllScans(const geometry_msgs::msg::TransformStamped& map_to_odom) {
    std::vector<Scan> all_scans = scan_storage_.getAllScans();

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    for (const auto& scan : all_scans) {
        // Transform points to the odom frame
        std::vector<Eigen::Vector2d> points = scan.points;
        transformPoints(points, map_to_odom);
        for (const auto& point : points) {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x();
            colored_point.y = point.y();
            colored_point.z = transform_z_;
            colored_point.r = 0; // Red
            colored_point.g = 0; // Orange
            colored_point.b = 255;   // Blue
            cloud.points.push_back(colored_point);
        }
    }

    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1; // Unorganized
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "odom";
    cloud_msg.header.stamp = time_stamp_;
    all_cloud_pub_->publish(cloud_msg);
}

} // namespace Localization

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Localization::Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
