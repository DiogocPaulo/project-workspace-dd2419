#include "lidar_scan_storage.hpp"
#include <iostream>

int main() {
    // Initialize storage with 1.0m cell size
    Localisation::LidarScanStorage storage(1.0);

    // Test data: three scans with poses
    std::vector<Eigen::Vector2d> scan1 = {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}};
    Localisation::Pose2D pose1(0.5, 0.5, 0.0); // Cell (0, 0)

    std::vector<Eigen::Vector2d> scan2 = {{1.5, 1.5}, {2.5, 1.5}};
    Localisation::Pose2D pose2(1.8, 1.8, 0.1); // Cell (1, 1)

    std::vector<Eigen::Vector2d> scan3 = {{0.2, 0.3}, {1.2, 0.3}};
    Localisation::Pose2D pose3(0.7, 0.7, 0.0); // Cell (0, 0), should append to scan1

    // Add scans
    std::cout << "Adding scan1 at pose (" << pose1.position.x() << ", " << pose1.position.y() << ")\n";
    storage.addScan(scan1, pose1);
    std::cout << "Adding scan2 at pose (" << pose2.position.x() << ", " << pose2.position.y() << ")\n";
    storage.addScan(scan2, pose2);
    std::cout << "Adding scan3 at pose (" << pose3.position.x() << ", " << pose3.position.y() << ")\n";
    storage.addScan(scan3, pose3);

    // Check total number of stored scans
    std::cout << "Total stored scans: " << storage.size() << " (expected 2)\n";

    // Query closest scan
    Localisation::Pose2D query_pose(0.8, 0.8, 0.0); // Near cell (0, 0)
    std::cout << "\nQuerying closest scan to pose (" << query_pose.position.x() << ", "
              << query_pose.position.y() << ")\n";
    auto closest = storage.getClosestScan(query_pose);

    if (closest) {
        std::cout << "Found closest scan at pose (" << closest->pose.position.x() << ", "
                  << closest->pose.position.y() << ", " << closest->pose.theta << ")\n";
        std::cout << "Number of points: " << closest->points.size() << " (expected 5)\n";
        std::cout << "Points:\n";
        for (const auto& p : closest->points) {
            std::cout << "  " << p.transpose() << "\n";
        }
    } else {
        std::cout << "No scan found!\n";
    }

    // Test an unexplored area
    Localisation::Pose2D far_pose(5.0, 5.0, 0.0); // Cell (5, 5), no scans
    std::cout << "\nQuerying closest scan to pose (" << far_pose.position.x() << ", "
              << far_pose.position.y() << ")\n";
    auto far_result = storage.getClosestScan(far_pose);
    if (far_result) {
        std::cout << "Found unexpected scan at pose (" << far_result->pose.position.x() << ", "
                  << far_result->pose.position.y() << ")\n";
    } else {
        std::cout << "No scan found (as expected).\n";
    }

    return 0;
}