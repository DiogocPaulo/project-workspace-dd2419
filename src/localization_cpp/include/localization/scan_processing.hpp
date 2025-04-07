#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h> // For tf2::Quaternion
#include <tf2/LinearMath/Matrix3x3.h>  // For tf2::Matrix3x3
#include <rclcpp/time.hpp>

namespace Localization {

// Default distance threshold for segment splitting (in meters)
constexpr double DEFAULT_SEGMENT_THRESHOLD = 0.1;
constexpr double MIN_RANGE = 0.4;
constexpr double MAX_RANGE = 10.0;

// Correct the coordinate of a point based on velocity and time
Eigen::Vector2d correctCoordinate(const Eigen::Vector2d& point,
                                  double delta_time,
                                  double linear_velocity,
                                  double angular_velocity) {
    // Change in robot motion
    double delta_x = linear_velocity * delta_time;
    double delta_theta = angular_velocity * delta_time;

    // Create inverse rotation matrix (negative angle)
    Eigen::Matrix2d inverse_rotation;
    inverse_rotation << std::cos(-delta_theta), -std::sin(-delta_theta),
                        std::sin(-delta_theta),  std::cos(-delta_theta);

    // Translation in robot frame (negative to compensate)
    Eigen::Vector2d translation(-delta_x, 0.0);

    // Apply inverse motion to point
    Eigen::Vector2d corrected_point = inverse_rotation * (point + translation);

    return corrected_point;
}

// Convert a LaserScan message to a vector of 2D points in the sensor frame
std::vector<Eigen::Vector2d> laserScanToPoints(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan, 
                                                const rclcpp::Time& scan_start_time, 
                                                double linear_velocity,
                                                double angular_velocity) {
    std::vector<Eigen::Vector2d> points;
    if (!scan || scan->ranges.empty()) {
        return points; // Return empty vector if scan is invalid
    }

    points.reserve(scan->ranges.size());

    float angle = scan->angle_min;
    const float angle_increment = scan->angle_increment;
    float prev_range = scan->ranges[0]; // Previous range for comparison

    for (size_t i = 0; i < scan->ranges.size(); ++i, angle += angle_increment) {
        float range = scan->ranges[i];
        // Skip invalid ranges
        if (!std::isfinite(range) || range < MIN_RANGE || range > MAX_RANGE) {
            continue;
        }

        // Check against previous range to avoid noise
        if (std::abs(range - prev_range) > DEFAULT_SEGMENT_THRESHOLD) {
            if (i < scan->ranges.size() - 1 && std::abs(scan->ranges[i + 1] - range) > DEFAULT_SEGMENT_THRESHOLD) {
                continue; // Skip this point if too far from the points on either side
            }
        }

        // Convert polar coordinates to Cartesian
        double x = static_cast<double>(range * std::cos(angle));
        double y = static_cast<double>(range * std::sin(angle));
        // Correct the coordinate based on velocity and time
        //rclcpp::Time point_time = scan_start_time + rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(i * scan->time_increment * 1e9));
        //double delta_time = (point_time - scan_start_time).seconds();
        //Eigen::Vector2d corrected_point = correctCoordinate(Eigen::Vector2d(x, y), delta_time, linear_velocity, angular_velocity);
        points.emplace_back(x, y);
        prev_range = range; // Update previous range
    }

    return points;
}


// Transform a set of 2D points using a given transform
void transformPoints(std::vector<Eigen::Vector2d>& points, 
                     const geometry_msgs::msg::TransformStamped& transform) {
    double tx = transform.transform.translation.x;
    double ty = transform.transform.translation.y;

    // Convert geometry_msgs quaternion to tf2::Quaternion
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    );

    // Convert quaternion to rotation matrix and extract yaw
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // Extract roll, pitch, yaw; we only need yaw for 2D

    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    for (auto& point : points) {
        double x = point.x();
        double y = point.y();
        point.x() = x * cos_yaw - y * sin_yaw + tx;
        point.y() = x * sin_yaw + y * cos_yaw + ty;
    }
}

// Transform a single 2D point using precomputed trig values
inline Eigen::Vector2d transformPoint(const Eigen::Vector2d& point, 
                                      double cos_yaw, double sin_yaw, 
                                      double tx, double ty) {
    double x = point.x();
    double y = point.y();
    return Eigen::Vector2d(x * cos_yaw - y * sin_yaw + tx, 
                           x * sin_yaw + y * cos_yaw + ty);
}

} // namespace Localization