#pragma once

#include <unordered_map>
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

// Grid cell key (integer coordinates)
struct GridCell {
    int x, y;

    GridCell(int x_ = 0, int y_ = 0) : x(x_), y(y_) {}

    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};

// Hash function for GridCell to use in unordered_map
struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const {
        return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
    }
};

// Stored scan: points in a vector with an occupancy grid for aging
struct Scan {
    std::vector<Eigen::Vector2d> points; // All points stored here
    std::unordered_map<GridCell, uint8_t, GridCellHash> occupancy_grid; // 0-100 counter per sub-cell
    Pose2D pose;
    double sub_cell_size;

    Scan(const std::vector<Eigen::Vector2d>& pts, const Pose2D& p, double sub_size)
        : pose(p), sub_cell_size(sub_size) {
        points.reserve(1000); // Preallocate for efficiency
        addPoints(pts);
    }

    // Add new points, updating occupancy counters
    void addPoints(const std::vector<Eigen::Vector2d>& new_points) {
        // Check available slots (max 1000 points)
        size_t current_count = points.size();
        size_t max_points = 1000;
        size_t available_slots = current_count < max_points ? max_points - current_count : 0;

        // Early exit if no slots available
        if (available_slots == 0) {
            return;
        }

        // Add up to available_slots points
        size_t num_to_add = std::min(new_points.size(), available_slots);
        for (size_t i = 0; i < num_to_add; ++i) {
            const auto& point = new_points[i];
            points.push_back(point); // Add unique point
            GridCell cell = pointToGridCell(point);
            occupancy_grid[cell] = 100; // Increment occupancy count
        }
    }

    // Age points and remove those with counter reaching 0
    void agePoints(int decrement = 1) {
        std::vector<Eigen::Vector2d> updated_points;
        updated_points.reserve(points.size());

        for (auto it = occupancy_grid.begin(); it != occupancy_grid.end();) {
            it->second = std::max(0, static_cast<int>(it->second) - decrement);
            if (it->second == 0) {
                // Remove point from vector by not copying it
                it = occupancy_grid.erase(it);
            } else {
                // Keep point associated with this cell
                Eigen::Vector2d point(it->first.x * sub_cell_size + sub_cell_size / 2,
                                      it->first.y * sub_cell_size + sub_cell_size / 2);
                updated_points.push_back(point);
                ++it;
            }
        }

        points = std::move(updated_points);
    }

private:
    GridCell pointToGridCell(const Eigen::Vector2d& point) const {
        int x = static_cast<int>(std::floor(point.x() / sub_cell_size));
        int y = static_cast<int>(std::floor(point.y() / sub_cell_size));
        return GridCell(x, y);
    }
};

class LidarScanStorage {
public:
    // Constructor with configurable cell size and sub-cell size (meters)
    explicit LidarScanStorage(double cell_size = 1.0, double sub_cell_size = 0.01)
        : cell_size_(cell_size), sub_cell_size_(sub_cell_size) {
        scans_.reserve(20); // Preallocate for typical map size
    }

    // Add a scan with its associated pose
    void addScan(const std::vector<Eigen::Vector2d>& points, const Pose2D& pose) {
        GridCell cell = poseToGridCell(pose);

        auto it = scans_.find(cell);
        if (it == scans_.end()) {
            // New cell: store the scan
            scans_.emplace(cell, Scan(points, pose, sub_cell_size_));
        } else {
            // Existing cell: add points and update occupancy
            it->second.addPoints(points);
        }
    }

    // Age all scans to forget old points
    void ageScans(int decrement = 1) {
        for (auto& [cell, scan] : scans_) {
            scan.agePoints(decrement);
        }
        // Optionally remove empty scans
        for (auto it = scans_.begin(); it != scans_.end();) {
            if (it->second.points.empty()) {
                it = scans_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // Retrieve the scan from the closest grid cell based on pose
    std::optional<Scan> getClosestScan(const Pose2D& current_pose) const {
        GridCell target_cell = poseToGridCell(current_pose);
        double min_dist_sq = std::numeric_limits<double>::max();
        std::optional<Scan> closest_scan;
    
        // Search all scans for the closest grid cell
        for (const auto& [cell, scan] : scans_) {
            double dist_sq = std::pow(cell.x - target_cell.x, 2) + std::pow(cell.y - target_cell.y, 2);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_scan = scan;
            }
        }
    
        return closest_scan;
    }

    std::optional<Scan> getClosestScan2(const Pose2D& current_pose) const {
        GridCell cell = poseToGridCell(current_pose);
        auto it = scans_.find(cell);

        if (it != scans_.end()) {
            // Found exact cell match
            return it->second;
        }

        // Search neighboring cells for closest pose
        double min_dist_sq = std::numeric_limits<double>::max();
        std::optional<Scan> closest_scan;

        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                GridCell neighbor(cell.x + dx, cell.y + dy);
                auto neighbor_it = scans_.find(neighbor);
                if (neighbor_it != scans_.end()) {
                    double dist_sq = (neighbor_it->second.pose.position - current_pose.position).squaredNorm();
                    if (dist_sq < min_dist_sq) {
                        min_dist_sq = dist_sq;
                        closest_scan = neighbor_it->second;
                    }
                }
            }
        }

        return closest_scan;
    }

    std::vector<Scan> getAllScans() const {
        std::vector<Scan> all_scans;
        for (const auto& pair : scans_) {
            all_scans.push_back(pair.second);
        }
        return all_scans;
    }    

    // Clear all stored scans
    void clear() {
        scans_.clear();
    }

    // Get number of stored scans
    size_t size() const {
        return scans_.size();
    }

    // Get grid cell from pose
    GridCell getGridCell(const Pose2D& pose) const {
        return poseToGridCell(pose);
    }

private:
    // Convert pose to grid cell coordinates
    GridCell poseToGridCell(const Pose2D& pose) const {
        int x = static_cast<int>(std::floor(pose.position.x() / cell_size_));
        int y = static_cast<int>(std::floor(pose.position.y() / cell_size_));
        return GridCell(x, y);
    }

    double cell_size_; // Grid cell size for scans (meters)
    double sub_cell_size_; // Sub-grid cell size for points (meters)
    std::unordered_map<GridCell, Scan, GridCellHash> scans_; // Grid map of scans
};

} // namespace Localization