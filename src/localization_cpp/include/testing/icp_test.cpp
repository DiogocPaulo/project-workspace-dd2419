#include "icp.hpp"
#include <iostream>

int main() {
    // Sample 2D point clouds
    std::vector<Eigen::Vector2d> source = {{0, 0}, {1, 0}, {0, 1}};
    std::vector<Eigen::Vector2d> target = {{0.1, 0.1}, {0.1, 1.1}, {-0.9, 0.1}};

    Localisation::ICP icp(0.2, 50);
    icp.setTarget(target);

    // Option 1: Get transform only
    auto start = std::chrono::high_resolution_clock::now();
    Eigen::Matrix3d transform = icp.computeTransform(source);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "Time taken to compute transform: " << elapsed.count() << " ms\n";
    std::cout << "Transform:\n" << transform << "\n";

    // Option 2: Get aligned points only
    auto start_aligned = std::chrono::high_resolution_clock::now();
    auto aligned = icp.getAlignedPoints(source);
    auto end_aligned = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_aligned = end_aligned - start_aligned;
    std::cout << "Time taken to get aligned points: " << elapsed_aligned.count() << " ms\n";
    std::cout << "Aligned Points:\n";
    for (const auto& p : aligned) std::cout << p.transpose() << "\n";

    // Option 3: Get both
    Eigen::Matrix3d transform2;
    std::vector<Eigen::Vector2d> aligned2;
    auto start_combined = std::chrono::high_resolution_clock::now();
    icp.computeICP(source, transform2, aligned2);
    auto end_combined = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_combined = end_combined - start_combined;
    std::cout << "Time taken to compute ICP (combined): " << elapsed_combined.count() << " ms\n";
    std::cout << "Transform (combined):\n" << transform2 << "\n";
    std::cout << "Aligned Points (combined):\n";
    for (const auto& p : aligned2) std::cout << p.transpose() << "\n";

    return 0;
}