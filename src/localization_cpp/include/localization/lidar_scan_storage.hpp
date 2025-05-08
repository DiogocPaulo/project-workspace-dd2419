#pragma once

#include <vector>
#include <Eigen/Dense>
#include <optional>
#include <cmath>

namespace Localization {

// Pose structure: 2D position (x, y) and orientation (theta)
struct Pose2D {
    Eigen::Vector2d position;
    double theta; // Orientation in radians

    Pose2D(double x = 0.0, double y = 0.0, double theta_rad = 0.0)
        : position(x, y), theta(theta_rad) {}
};

// Stored scan: points in a vector with associated pose
struct Scan {
    std::vector<Eigen::Vector2d> points; // All points stored here
    Pose2D pose;
    int id; // Unique identifier for the scan

    Scan(const std::vector<Eigen::Vector2d>& pts, const Pose2D& p, int scan_id)
        : pose(p), id(scan_id) {
        points.reserve(500); // Preallocate for efficiency
        addPoints(pts);
    }

    // Add new points
    void addPoints(const std::vector<Eigen::Vector2d>& new_points) {
        // Check available slots (max 500 points)
        size_t current_count = points.size();
        size_t max_points = 500;
        size_t available_slots = current_count < max_points ? max_points - current_count : 0;

        // Early exit if no slots available
        if (available_slots == 0) {
            return;
        }

        // Add up to available_slots points
        size_t num_to_add = std::min(new_points.size(), available_slots);
        for (size_t i = 0; i < num_to_add; ++i) {
            points.push_back(new_points[i]);
        }
    }

    // Age points (remove all points after a certain number of updates)
    void agePoints(int decrement = 1) {
        // Simple aging: clear points after a fixed number of updates
        static int age_counter = 100; // Retain points for 100 updates
        age_counter = std::max(0, age_counter - decrement);
        if (age_counter == 0) {
            points.clear();
            age_counter = 100; // Reset counter
        }
    }

    // Add this inside the Scan struct
    void applyPoseUpdate(const Pose2D& new_pose) {
        // Compute relative transformation
        double delta_theta = new_pose.theta - pose.theta;
        Eigen::Rotation2Dd rotation(delta_theta);
        Eigen::Vector2d translation = new_pose.position - rotation * pose.position;

        // Apply to all points
        for (auto& point : points) {
            point = rotation * point + translation;
        }

        // Update stored pose
        pose = new_pose;
    }
};

class LidarScanStorage {
public:
    // Constructor with configurable range and minimum scan separation
    explicit LidarScanStorage(double surrounding_range = 1.0, double min_scan_separation = 0.5)
        : surrounding_range_(surrounding_range), 
            min_scan_separation_(min_scan_separation),
            next_id_(0) {  // Initialize ID counter to 0
        scans_.reserve(20); // Preallocate for typical map size
    }

    // Add a scan with its associated pose if it's sufficiently far from existing scans
    void addScan(const std::vector<Eigen::Vector2d>& points, const Pose2D& pose) {
        // Check distance to all existing scans
        for (const auto& scan : scans_) {
            double dist_sq = (scan.pose.position - pose.position).squaredNorm();
            if (dist_sq < min_scan_separation_ * min_scan_separation_) {
                return; // Scan is too close to an existing scan
            }
        }

        // Add the scan with the next available ID, then increment the counter
        scans_.emplace_back(points, pose, next_id_++);
    }

    // Age all scans to forget old points
    void ageScans(int decrement = 1) {
        for (auto& scan : scans_) {
            scan.agePoints(decrement);
        }
        // Remove empty scans
        scans_.erase(
            std::remove_if(scans_.begin(), scans_.end(),
                [](const Scan& scan) { return scan.points.empty(); }),
            scans_.end()
        );
    }

    // Update an existing scan
    void updateScan(int id, const std::vector<Eigen::Vector2d>& points, const Pose2D& pose) {
        auto it = std::find_if(scans_.begin(), scans_.end(), 
            [id](const Scan& s) { return s.id == id; });
        if (it != scans_.end()) {
            it->points = points;
            it->pose = pose;
        }
    }

    // Get scan by ID (helper function)
    std::optional<Scan> getScan(int id) const {
        auto it = std::find_if(scans_.begin(), scans_.end(), 
            [id](const Scan& s) { return s.id == id; });
        if (it != scans_.end()) {
            return *it;
        }
        return std::nullopt;
    }

    // Retrieve the scan closest to the given pose
    std::optional<Scan> getClosestScan(const Pose2D& current_pose) const {
        if (scans_.empty()) {
            return std::nullopt;
        }

        double min_dist_sq = std::numeric_limits<double>::max();
        std::optional<Scan> closest_scan;

        // Search all scans for the closest pose
        for (const auto& scan : scans_) {
            double dist_sq = (scan.pose.position - current_pose.position).squaredNorm();
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_scan = scan;
            }
        }

        return closest_scan;
    }

    // Retrieve all scans within a specified range of the given pose
    std::vector<Scan> getSurroundingScans(const Pose2D& current_pose) const {
        std::vector<Scan> surrounding_scans;
        surrounding_scans.reserve(scans_.size());

        // Include scans whose pose is within surrounding_range_
        for (const auto& scan : scans_) {
            double dist_sq = (scan.pose.position - current_pose.position).squaredNorm();
            if (dist_sq <= surrounding_range_ * surrounding_range_) {
                surrounding_scans.push_back(scan);
            }
        }

        return surrounding_scans;
    }

    // Get all stored scans
    std::vector<Scan> getAllScans() const {
        return scans_;
    }

    // Clear all stored scans and reset ID counter
    void clear() {
        scans_.clear();
        next_id_ = 0;  // Reset ID counter when clearing all scans
    }

    // Get number of stored scans
    size_t size() const {
        return scans_.size();
    }

private:
    double surrounding_range_; // Range for surrounding scans (meters)
    double min_scan_separation_; // Minimum distance between scans (meters)
    std::vector<Scan> scans_; // List of scans with poses
    int next_id_; // Counter for assigning unique IDs to scans
};

} // namespace Localization