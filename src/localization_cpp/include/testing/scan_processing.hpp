#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "line_segment.hpp" // Include LineSegment definition
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

    // Calculate the change in position and orientation based on velocity and time
    double delta_x = linear_velocity * delta_time;
    double delta_theta = angular_velocity * delta_time;

    // Create the rotation matrix
    Eigen::Matrix2d rotation;
    rotation << std::cos(delta_theta), -std::sin(delta_theta),
                std::sin(delta_theta), std::cos(delta_theta);

    // Create the translation vector
    Eigen::Vector2d translation(delta_x, 0);

    // Apply the transformation
    Eigen::Vector2d corrected_point = rotation * point + translation;

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
        rclcpp::Time point_time = scan_start_time + rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(i * scan->time_increment * 1e9));
        double delta_time = (point_time - scan_start_time).seconds();
        Eigen::Vector2d corrected_point = correctCoordinate(Eigen::Vector2d(x, y), delta_time, linear_velocity, angular_velocity);
        points.emplace_back(corrected_point);
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

// Merge segments based on proximity of their endpoints
std::vector<LineSegment> mergeLineSegments(
    std::vector<LineSegment>& segments,
    double merge_threshold = DEFAULT_SEGMENT_THRESHOLD) {

    std::vector<LineSegment> merged_segments;
    std::vector<bool> merged(segments.size(), false); // Track which segments have been merged

    for (size_t i = 0; i < segments.size(); ++i) {
        if (merged[i]) {
            continue; // Skip already merged segments
        }

        LineSegment current = segments[i];
        const auto& seg_i_points = current.points();
        Eigen::Vector2d i_start = seg_i_points.front();
        Eigen::Vector2d i_end = seg_i_points.back();
        merged[i] = true;

        // Check all segments except itself and already merged ones
        for (size_t j = i + 1; j < segments.size(); ++j) {
            if (merged[j]) {
                continue; // Skip already merged segments
            }

            const auto& seg_j_points = segments[j].points();
            Eigen::Vector2d j_start = seg_j_points.front();
            Eigen::Vector2d j_end = seg_j_points.back();

            // Check distances and directions
            bool should_merge = false;

            // Distance check: Are any endpoints close enough?
            double dist_ss = (i_start - j_start).norm();
            double dist_se = (i_start - j_end).norm();
            double dist_es = (i_end - j_start).norm();
            double dist_ee = (i_end - j_end).norm();
            if (dist_ss < merge_threshold || dist_se < merge_threshold ||
                dist_es < merge_threshold || dist_ee < merge_threshold) {
                should_merge = true;
            }

            // Directional check: Is an endpoint "between" the other segment?
            if (!should_merge) {
                // Define segment vectors
                Eigen::Vector2d vec_i = i_end - i_start;
                Eigen::Vector2d vec_j = j_end - j_start;
                double len_i = vec_i.norm();
                double len_j = vec_j.norm();

                if (len_i > 0 && len_j > 0) { // Avoid division by zero
                    vec_i /= len_i; // Normalize direction vectors
                    vec_j /= len_j;

                    // Project j_start and j_end onto segment i
                    auto project_onto_i = [&](const Eigen::Vector2d& point) -> double {
                        Eigen::Vector2d vec_p = point - i_start;
                        return vec_p.dot(vec_i); // Projection length along segment i
                    };
                    double proj_j_start_i = project_onto_i(j_start);
                    double proj_j_end_i = project_onto_i(j_end);

                    // Project i_start and i_end onto segment j
                    auto project_onto_j = [&](const Eigen::Vector2d& point) -> double {
                        Eigen::Vector2d vec_p = point - j_start;
                        return vec_p.dot(vec_j); // Projection length along segment j
                    };
                    double proj_i_start_j = project_onto_j(i_start);
                    double proj_i_end_j = project_onto_j(i_end);

                    // Check if projections fall within segment bounds
                    bool j_start_between_i = (proj_j_start_i >= 0 && proj_j_start_i <= len_i);
                    bool j_end_between_i = (proj_j_end_i >= 0 && proj_j_end_i <= len_i);
                    bool i_start_between_j = (proj_i_start_j >= 0 && proj_i_start_j <= len_j);
                    bool i_end_between_j = (proj_i_end_j >= 0 && proj_i_end_j <= len_j);

                    // Distance from endpoint to projection point for containment check
                    if (j_start_between_i && (j_start - (i_start + proj_j_start_i * vec_i)).norm() < merge_threshold) {
                        should_merge = true;
                    } else if (j_end_between_i && (j_end - (i_start + proj_j_end_i * vec_i)).norm() < merge_threshold) {
                        should_merge = true;
                    } else if (i_start_between_j && (i_start - (j_start + proj_i_start_j * vec_j)).norm() < merge_threshold) {
                        should_merge = true;
                    } else if (i_end_between_j && (i_end - (j_start + proj_i_end_j * vec_j)).norm() < merge_threshold) {
                        should_merge = true;
                    }
                }
            }

            if (should_merge) {
                try {
                    current.merge(segments[j]); // Merge j into current
                    merged[j] = true; // Mark j as merged
                } catch (const std::runtime_error& e) {
                    // Log error if merge fails (optional)
                }
            }
        }

        merged_segments.push_back(current); // Add the (possibly merged) segment
    }

    return merged_segments;
}

// Convert a LaserScan to a vector of LineSegments in a single pass
std::vector<LineSegment> laserScanToLineSegments(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan,
    const geometry_msgs::msg::TransformStamped& transform,
    const Eigen::Vector2d& reference_point,
    double base_threshold = DEFAULT_SEGMENT_THRESHOLD) {

    std::vector<LineSegment> segments;
    if (!scan || scan->ranges.empty()) {
        return segments; // Return empty if scan is invalid
    }

    // Precompute transform parameters (applied after segmentation)
    double tx = transform.transform.translation.x;
    double ty = transform.transform.translation.y;
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    std::vector<Eigen::Vector2d> current_segment_points;
    current_segment_points.reserve(scan->ranges.size());

    float angle = scan->angle_min;
    const float angle_increment = scan->angle_increment;
    float prev_range = 0.0; // Previous range for comparison
    bool first_valid_point = true;

    for (size_t i = 0; i < scan->ranges.size(); ++i, angle += angle_increment) {
        float range = scan->ranges[i];

        // Skip invalid ranges without splitting the segment
        if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
            continue;
        }

        // Skip sensor noise (too close or in arm angles)
        if (range < 0.1) { // || angle < -1.0 || angle > 1.0) {
            continue;
        }

        if (!first_valid_point) {
            // Adaptive threshold based on the average range of the current and previous point
            double avg_range = (range + prev_range) / 2.0;
            double adaptive_threshold = base_threshold + (0.1 * avg_range); // Increase by 0.1 for every meter

            // Compare consecutive ranges directly
            double range_diff = std::abs(range - prev_range);
            if (range_diff > adaptive_threshold) {
                if (current_segment_points.size() >= 5) {
                    try {
                        segments.emplace_back(current_segment_points, reference_point);
                    } catch (const std::runtime_error& e) {
                        // Log error if needed
                    }
                }
                current_segment_points.clear();
                current_segment_points.reserve(scan->ranges.size() - i);
            }
        } else {
            first_valid_point = false;
        }

        // Convert to Cartesian and transform only after segmentation decision
        Eigen::Vector2d point(range * std::cos(angle), range * std::sin(angle));
        point = transformPoint(point, cos_yaw, sin_yaw, tx, ty);
        current_segment_points.push_back(point);
        prev_range = range; // Update previous range
    }

    // Handle the last segment
    if (current_segment_points.size() >= 5) {
        try {
            segments.emplace_back(current_segment_points, reference_point);
        } catch (const std::runtime_error& e) {
            // Log error if needed
        }
    }

    // Merge segments based on endpoint proximity
    if (!segments.empty()) {
        segments = mergeLineSegments(segments);
    }

    return segments;
}

} // namespace Localization