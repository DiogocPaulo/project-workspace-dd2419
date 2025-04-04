#ifndef LIDAR_SCAN_STORAGE_HPP
#define LIDAR_SCAN_STORAGE_HPP

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

// Stored scan: points and associated pose
struct Scan {
    std::vector<Eigen::Vector2d> points;
    Pose2D pose;

    Scan(const std::vector<Eigen::Vector2d>& pts, const Pose2D& p)
        : points(pts.begin(), pts.begin() + std::min<size_t>(pts.size(), 1000)), pose(p) {}
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
        // Combine x and y using a simple hash (good enough for spatial grid)
        return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
    }
};

class LidarScanStorage {
public:
    // Constructor with configurable cell size (meters)
    explicit LidarScanStorage(double cell_size = 1.0) : cell_size_(cell_size) {
        scans_.reserve(20); // Preallocate for typical map size
    }

    // Add a scan with its associated pose
    void addScan(const std::vector<Eigen::Vector2d>& points, const Pose2D& pose) {
        GridCell cell = poseToGridCell(pose);

        auto it = scans_.find(cell);
        if (it == scans_.end()) {
            // New cell: store the scan
            scans_.emplace(cell, Scan(points, pose));
        } else {
            // Existing cell: append points to previous scan
            if (it->second.points.size() < 1000) {
                size_t remaining_space = 1000 - it->second.points.size();
                size_t points_to_add = std::min(remaining_space, points.size());
                it->second.points.insert(it->second.points.end(), points.begin(), points.begin() + points_to_add);
            }
        }
    }

    // Retrieve the scan from the closest grid cell based on pose
    std::optional<Scan> getClosestScan(const Pose2D& current_pose) const {
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

    double cell_size_; // Grid cell size in meters
    std::unordered_map<GridCell, Scan, GridCellHash> scans_; // Grid map of scans
};

} // namespace Localization

#endif // LIDAR_SCAN_STORAGE_HPP