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
    scan_storage_ = LidarScanStorage(2, 0.3); // Range for surrounding scans
    icp_ = ICP(0.2, 100); // 10 cm threshold, 50 iterations

    RCLCPP_INFO(this->get_logger(), "Localization node initialized.");
}

void Node::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
    if (std::abs(angular_velocity_) > 0.2) {
        RCLCPP_DEBUG(this->get_logger(), "Robot is rotating too fast, skipping scan processing.");
        return; // Skip processing if robot is not moving
    }

    // Ensure TF buffer has data
    if (!tf_buffer_->canTransform("odom", msg->header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(500))) {
        RCLCPP_WARN(this->get_logger(), "Transform from %s to map not available yet!", msg->header.frame_id.c_str());
        return;
    }
    try {
        // Get the transform from Lidar frame to Odom frame
        RCLCPP_DEBUG(this->get_logger(), "Transform exists from %s to map.", msg->header.frame_id.c_str());
        geometry_msgs::msg::TransformStamped lidar_to_map = tf_buffer_->lookupTransform("map", msg->header.frame_id, tf2::TimePointZero);
        transform_z_ = lidar_to_map.transform.translation.z;

        // Convert LaserScan to 2D points in Map frame
        std::vector<Eigen::Vector2d> points = laserScanToPoints(msg, time_stamp_, linear_velocity_, angular_velocity_);
        transformPoints(points, lidar_to_map);

        // Get surrounding scans
        std::vector<Eigen::Vector2d> stored_points;

        // Get surrounding scans sorted by ID (oldest first)
        std::vector<Scan> surrounding_scans = scan_storage_.getSurroundingScans(current_pose_);
        std::sort(surrounding_scans.begin(), surrounding_scans.end(), 
            [](const Scan& a, const Scan& b) { return a.id < b.id; });

        // Perform chain alignment if we have enough scans
        if (surrounding_scans.size() > 1) {
            // Find newest and oldest scans (assuming scans are sorted by ID)
            Scan& newest_scan = surrounding_scans.back();
            Scan& oldest_scan = surrounding_scans.front();

            // Only attempt alignment if there's significant drift potential
            if ((newest_scan.id - oldest_scan.id) > 10) {
                std::vector<Scan> all_scans = scan_storage_.getAllScans();
                std::sort(all_scans.begin(), all_scans.end(), 
                    [](const Scan& a, const Scan& b) { return a.id < b.id; });
                performChainAlignment(all_scans);
            }
        }

        /*
        
        */
        if (!surrounding_scans.empty()) {
            for (const Scan& scan : surrounding_scans) {
                stored_points.insert(stored_points.end(), scan.points.begin(), scan.points.end());
            }
        } else {
            // Fall back to the closest scan
            auto stored_scan = scan_storage_.getClosestScan(current_pose_);
            if (stored_scan.has_value()) {
                stored_points = stored_scan->points; // Or insert, depending on intent
            }
        }

        /*
        auto stored_scan = scan_storage_.getClosestScan(current_pose_);
        if (stored_scan.has_value()) {
            stored_points = stored_scan->points; // Or insert, depending on intent
        }
        */
        

        if (!stored_points.empty()) {
            /*
            // Align Principal Components before ICP
            Eigen::Matrix2d pre_rotation = computeRotationAlignment(points, stored_points);
            std::vector<Eigen::Vector2d> pre_aligned_points(points.size());
            for (size_t i = 0; i < points.size(); ++i) {
                pre_aligned_points[i] = pre_rotation * points[i];
            }
            */

            // Perform ICP to find transformation and get aligned points
            std::vector<Eigen::Vector2d> aligned_points;
            Eigen::Matrix3d icp_transform;
            icp_.setTarget(stored_points);

            double fitness;
            double inlier_rmse;

            auto start_time = std::chrono::high_resolution_clock::now();
            //icp_.computeICP2(points, icp_transform, aligned_points);
            icp_.computeICP(points, icp_transform, aligned_points, fitness, inlier_rmse);
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            RCLCPP_INFO(this->get_logger(), "ICP computation took %ld ms", duration);
            RCLCPP_INFO(this->get_logger(), "ICP results -> Fitness: %f and Inliner RMSE: %f", fitness, inlier_rmse);

            // Extract translation and rotation from ICP transform
            double icp_translation_x = icp_transform(0, 2);
            double icp_translation_y = icp_transform(1, 2);
            double icp_rotation_theta = std::atan2(icp_transform(1, 0), icp_transform(0, 0));

            // Skip invalid icp
            if (std::abs(icp_translation_x) > 0.22 || std::abs(icp_translation_y) > 0.22 || std::abs(icp_rotation_theta) > (M_PI / 2)) {
                return;
            }

            // Use ICP if fitness and RMSE is good enough
            if (fitness > 0.5 && inlier_rmse < 0.2) {
                // Convert pre_rotation to theta (angle) for quaternion
                //double pre_rotation_theta = std::atan2(pre_rotation(1, 0), pre_rotation(0, 0));

                // Convert rotations to quaternions
                tf2::Quaternion pre_rotation_quat, icp_rotation_quat;
                //pre_rotation_quat.setRPY(0.0, 0.0, pre_rotation_theta);
                icp_rotation_quat.setRPY(0.0, 0.0, icp_rotation_theta);

                // Update the current transform
                // Combine translations
                translation_[0] += icp_translation_x;
                translation_[1] += icp_translation_y;

                // Combine rotations: current_rotation = icp_rotation * pre_rotation * current_rotation
                rotation_ = icp_rotation_quat * rotation_;
                //rotation_ = icp_rotation_quat * pre_rotation_quat * rotation_;
            }

            // Store the current scan in the LidarScanStorage
            if (std::abs(angular_velocity_) < 0.1 && std::abs(linear_velocity_) < 0.01) { // (current_pose_.position - stored_scan->pose.position).norm() > 0.1 || 
                //stored_scan->agePoints(10); // Age points in the storage
                std::vector<Eigen::Vector2d> range_limited_alinged_points = limitPointsByRangeFromPose(aligned_points, current_pose_.position, 4);
                scan_storage_.addScan(range_limited_alinged_points, current_pose_);
                RCLCPP_INFO(this->get_logger(), "Stored new scan!");
            }

            // Publish the reference point cloud
            publishPointCloud(stored_points);
            publishAllScans();

        } else if (std::abs(linear_velocity_) < 0.01) {
            RCLCPP_WARN(this->get_logger(), "No stored scan found for ICP.");
            // Store the current scan in the LidarScanStorage
            std::vector<Eigen::Vector2d> range_limited_points = limitPointsByRangeFromPose(points, current_pose_.position, 4);
            scan_storage_.addScan(range_limited_points, current_pose_);
        }

    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to map: %s", msg->header.frame_id.c_str(), ex.what());
        return; // Skip processing if transform is unavailable
    }
}

// New function to perform chain-based alignment
void Node::performChainAlignment(const std::vector<Scan>& scans) {
    if (scans.size() < 2) return;

    // Find newest and oldest scans (assuming scans are sorted by ID)
    const Scan& newest_scan = scans.back();
    const Scan& oldest_scan = scans.front();

    // Skip if newest scan was already corrected
    if (corrected_scan_ids_.count(newest_scan.id)) {
        RCLCPP_DEBUG(get_logger(), "Scan %d already corrected - skipping chain alignment", newest_scan.id);
        return;
    }

    // Only attempt alignment if there's significant drift potential
    if ((newest_scan.id - oldest_scan.id) < 10) return;

    RCLCPP_WARN(this->get_logger(), 
        "Performing chain correction (ID %d → %d) to reduce accumulated drift", 
        newest_scan.id, oldest_scan.id);

    /* ====== STEP 1: Align newest to oldest ====== */
    Eigen::Matrix3d base_transform;
    std::vector<Eigen::Vector2d> aligned_points;
    double fitness, inlier_rmse;
    
    icp_.setTarget(oldest_scan.points);
    icp_.computeICP(newest_scan.points, base_transform, aligned_points, fitness, inlier_rmse);

    if (inlier_rmse >= 0.1) {
        RCLCPP_WARN(get_logger(), "Initial alignment failed (RMSE: %.3f)", inlier_rmse);
        return;
    }

    /* ====== STEP 2: Propagate correction backward ====== */
    Eigen::Matrix3d cumulative_transform = base_transform;
    std::vector<Eigen::Matrix3d> transforms;
    std::vector<int> scans_to_correct;

    for (auto it = scans.rbegin() + 1; it != scans.rend() - 1; ++it) {
        // Skip already corrected scans in the chain
        if (corrected_scan_ids_.count(it->id)) {
            RCLCPP_DEBUG(get_logger(), "Skipping already corrected scan ID %d", it->id);
            continue;
        }

        // Apply transform to current scan
        std::vector<Eigen::Vector2d> transformed_points;
        for (const auto& pt : it->points) {
            Eigen::Vector3d homog_pt(pt.x(), pt.y(), 1.0);
            homog_pt = cumulative_transform * homog_pt;
            transformed_points.emplace_back(homog_pt.x(), homog_pt.y());
        }

        // Align to next older scan
        Eigen::Matrix3d incremental_transform;
        icp_.setTarget((it + 1)->points);
        icp_.computeICP(transformed_points, incremental_transform, 
                       transformed_points, fitness, inlier_rmse);

        if (inlier_rmse >= 0.1) {
            RCLCPP_WARN(get_logger(), "Chain alignment failed at ID %d (RMSE: %.3f)", it->id, inlier_rmse);
            break;
        }

        cumulative_transform = incremental_transform * cumulative_transform;
        transforms.push_back(cumulative_transform);
        scans_to_correct.push_back(it->id);
    }

    /* ====== STEP 3: Apply corrections ====== */
    if (!transforms.empty()) {
        size_t transform_idx = 0;
        
        // Apply transforms forward through the chain
        for (auto it = scans.begin() + 1; it != scans.end() - 1; ++it) {
            // Only correct scans that were processed in step 2
            if (std::find(scans_to_correct.begin(), scans_to_correct.end(), it->id) == scans_to_correct.end()) {
                continue;
            }

            if (transform_idx >= transforms.size()) break;
            
            const auto& transform = transforms[transforms.size() - 1 - transform_idx++];
            
            // Transform points
            std::vector<Eigen::Vector2d> corrected_points;
            for (const auto& pt : it->points) {
                Eigen::Vector3d homog_pt(pt.x(), pt.y(), 1.0);
                homog_pt = transform * homog_pt;
                corrected_points.emplace_back(homog_pt.x(), homog_pt.y());
            }
            
            // Update pose
            Pose2D corrected_pose = it->pose;
            Eigen::Vector3d homog_pos(it->pose.position.x(), it->pose.position.y(), 1.0);
            homog_pos = transform * homog_pos;
            corrected_pose.position.x() = homog_pos.x();
            corrected_pose.position.y() = homog_pos.y();
            corrected_pose.theta += std::atan2(transform(1,0), transform(0,0));
            
            // Update storage and mark as corrected
            scan_storage_.updateScan(it->id, corrected_points, corrected_pose);
            corrected_scan_ids_.insert(it->id);
        }

        RCLCPP_INFO(get_logger(), "Chain corrected %zu scans. Final RMSE: %.3f", 
                   scans_to_correct.size(), inlier_rmse);
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
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = time_stamp_;
    cloud_pub_->publish(cloud_msg);
}

void Node::publishAllScans() {
    std::vector<Scan> all_scans = scan_storage_.getAllScans();

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    for (const auto& scan : all_scans) {
        // Transform points to the odom frame
        std::vector<Eigen::Vector2d> points = scan.points;
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
    cloud_msg.header.frame_id = "map";
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
