#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h> // For tf2::Quaternion
#include <tf2/LinearMath/Matrix3x3.h>  // For tf2::Matrix3x3
#include <rclcpp/time.hpp>

#include <random>

namespace Localization {

// Default distance threshold for segment splitting (in meters)
constexpr double DEFAULT_SEGMENT_THRESHOLD = 0.1;
constexpr double MIN_RANGE = 0.4;
constexpr double MAX_RANGE = 3.0;

// Correct the coordinate of a point based on velocity and time
Eigen::Vector2d correctCoordinate(const Eigen::Vector2d& point,
                                  double delta_time,
                                  double linear_velocity,
                                  double angular_velocity) {
    // Change in robot motion
    double delta_x = linear_velocity * delta_time;
    double delta_theta = angular_velocity * delta_time;
    
    // Create rotation matrix for the motion compensation (opposite of robot's motion)
    Eigen::Matrix2d rotation;
    rotation << std::cos(-delta_theta), -std::sin(-delta_theta),
                std::sin(-delta_theta), std::cos(-delta_theta);
    
    // Translation in robot frame (opposite of robot's movement)
    Eigen::Vector2d translation(-delta_x, 0.0);
    
    // Apply inverse motion to point:
    // 1. Rotate the point by the negative angle (inverse rotation)
    // 2. Subtract the linear displacement (compensate for linear motion)
    Eigen::Vector2d corrected_point = rotation * point;  // Rotate first
    
    // Then compensate for the linear displacement
    corrected_point += translation;

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
    
    // Calculate total scan duration
    double scan_duration = scan->time_increment * (scan->ranges.size() - 1);
    
    points.reserve(scan->ranges.size());
    float angle = scan->angle_min;
    const float angle_increment = scan->angle_increment;
    float prev_range = scan->ranges[0]; // Previous range for comparison
    
    for (size_t i = 0; i < scan->ranges.size(); ++i, angle += angle_increment) {
        float range = scan->ranges[i];
        
        // Skip invalid ranges and out-of-bounds values
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
        Eigen::Vector2d point(x, y);
        
        // Calculate time offset for this point from the start of the scan
        double delta_time = i * scan->time_increment;
        
        // Apply motion compensation
        Eigen::Vector2d corrected_point = correctCoordinate(point, delta_time, linear_velocity, angular_velocity);
        
        points.emplace_back(corrected_point); // Store the corrected point
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

// Transform a set of 2D points using a given transform
void transformPoint(std::vector<Eigen::Vector2d>& points, 
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

Eigen::Matrix2d computeRotationAlignment(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<Eigen::Vector2d>& stored_points) {
    // Parameters
    const size_t min_points = 2;
    const size_t max_iterations = 100;
    const double inlier_threshold = 0.1; // Distance threshold for inliers (in meters, adjust based on scan noise)
    const size_t min_inliers = 10;      // Minimum number of inliers to accept a rotation
    const double min_inlier_ratio = 0.3; // Minimum fraction of points that must be inliers

    // Return identity if insufficient points
    if (points.size() < min_points || stored_points.size() < min_points) {
        return Eigen::Matrix2d::Identity();
    }

    // Random number generator for sampling
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dist_points(0, points.size() - 1);
    std::uniform_int_distribution<size_t> dist_stored(0, stored_points.size() - 1);

    Eigen::Matrix2d best_rotation = Eigen::Matrix2d::Identity();
    size_t best_inlier_count = 0;

    // RANSAC loop
    for (size_t iter = 0; iter < max_iterations; ++iter) {
        // Sample two points from each set
        size_t idx1 = dist_points(gen);
        size_t idx2 = dist_points(gen);
        while (idx2 == idx1) idx2 = dist_points(gen); // Ensure different points
        size_t idx1_stored = dist_stored(gen);
        size_t idx2_stored = dist_stored(gen);
        while (idx2_stored == idx1_stored) idx2_stored = dist_stored(gen);

        // Compute vectors between points
        Eigen::Vector2d vec_points = points[idx2] - points[idx1];
        Eigen::Vector2d vec_stored = stored_points[idx2_stored] - stored_points[idx1_stored];

        // Check for near-zero vectors (degenerate case)
        if (vec_points.norm() < 1e-6 || vec_stored.norm() < 1e-6) {
            continue;
        }

        // Compute rotation angle
        double cos_theta = vec_points.normalized().dot(vec_stored.normalized());
        double sin_theta = vec_points(0) * vec_stored(1) - vec_points(1) * vec_stored(0);
        sin_theta /= (vec_points.norm() * vec_stored.norm());
        double theta = std::atan2(sin_theta, cos_theta);

        // Skip invalid angles
        if (std::isnan(theta) || std::isinf(theta)) {
            continue;
        }

        // Construct rotation matrix
        Eigen::Matrix2d rotation;
        rotation << std::cos(theta), -std::sin(theta),
                    std::sin(theta),  std::cos(theta);

        // Evaluate rotation: count inliers
        size_t inlier_count = 0;
        std::vector<Eigen::Vector2d> inliers_points, inliers_stored;
        for (size_t i = 0; i < points.size(); ++i) {
            // Transform point
            Eigen::Vector2d transformed = rotation * points[i];

            // Find nearest point in stored_points
            double min_dist = std::numeric_limits<double>::max();
            for (const auto& sp : stored_points) {
                double dist = (transformed - sp).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }

            // Check if inlier
            if (min_dist < inlier_threshold) {
                inlier_count++;
                inliers_points.push_back(points[i]);
                inliers_stored.push_back(stored_points[i]); // Store corresponding point (approximate)
            }
        }

        // Update best rotation if more inliers
        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_rotation = rotation;
        }
    }

    // Check if a valid rotation was found
    if (best_inlier_count < min_inliers || 
        best_inlier_count < min_inlier_ratio * std::min(points.size(), stored_points.size())) {
        return Eigen::Matrix2d::Identity();
    }

    // Optional: Refine rotation using all inliers (using PCA on inliers)
    if (best_inlier_count > min_points) {
        // Compute centroids of inlier points
        Eigen::Vector2d centroid_points(0.0, 0.0);
        Eigen::Vector2d centroid_stored(0.0, 0.0);
        for (size_t i = 0; i < points.size(); ++i) {
            Eigen::Vector2d transformed = best_rotation * points[i];
            double min_dist = std::numeric_limits<double>::max();
            size_t min_idx = 0;
            for (size_t j = 0; j < stored_points.size(); ++j) {
                double dist = (transformed - stored_points[j]).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    min_idx = j;
                }
            }
            if (min_dist < inlier_threshold) {
                centroid_points += points[i];
                centroid_stored += stored_points[min_idx];
            }
        }
        centroid_points /= best_inlier_count;
        centroid_stored /= best_inlier_count;

        // Compute covariance matrix
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < points.size(); ++i) {
            Eigen::Vector2d transformed = best_rotation * points[i];
            double min_dist = std::numeric_limits<double>::max();
            size_t min_idx = 0;
            for (size_t j = 0; j < stored_points.size(); ++j) {
                double dist = (transformed - stored_points[j]).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    min_idx = j;
                }
            }
            if (min_dist < inlier_threshold) {
                Eigen::Vector2d p = points[i] - centroid_points;
                Eigen::Vector2d q = stored_points[min_idx] - centroid_stored;
                cov += q * p.transpose();
            }
        }
        cov /= best_inlier_count;

        // SVD to compute refined rotation
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d R = svd.matrixU() * svd.matrixV().transpose();
        // Ensure proper rotation (determinant = 1)
        if (R.determinant() < 0) {
            R.col(1) *= -1;
        }
        if (!std::isnan(R(0, 0)) && !std::isinf(R(0, 0))) {
            best_rotation = R;
        }
    }

    return best_rotation;
}

} // namespace Localization