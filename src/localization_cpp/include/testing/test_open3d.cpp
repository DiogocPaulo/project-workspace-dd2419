#include <open3d/Open3D.h>
#include <chrono>
#include <iostream>
#include <random>

int main() {
    // Generate a 1000-point L-shaped corner
    std::vector<Eigen::Vector3d> source_vec;
    source_vec.reserve(1000);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 10.0); // Corner size: 10 units

    // Vertical leg (500 points along y-axis)
    for (int i = 0; i < 500; ++i) {
        double y = dis(gen);
        source_vec.emplace_back(0.0, y, 0.0);
    }
    // Horizontal leg (500 points along x-axis)
    for (int i = 0; i < 500; ++i) {
        double x = dis(gen);
        source_vec.emplace_back(x, 0.0, 0.0);
    }

    // Known transformation: 5° rotation + (0.1, 0.1) translation
    double angle = 5.0 * M_PI / 180.0; // 5° in radians
    Eigen::Matrix4d known_transform = Eigen::Matrix4d::Identity();
    known_transform(0, 0) = cos(angle);
    known_transform(0, 1) = -sin(angle);
    known_transform(1, 0) = sin(angle);
    known_transform(1, 1) = cos(angle);
    known_transform(0, 3) = 0.1; // Small shift relative to size 10
    known_transform(1, 3) = 0.1;

    // Apply known transform to create target
    auto source = std::make_shared<open3d::geometry::PointCloud>(source_vec);
    auto target = std::make_shared<open3d::geometry::PointCloud>(*source);
    target->Transform(known_transform);

    // Run ICP to recover the transformation
    double threshold = 0.1; // Larger threshold for faster convergence with small shift
    Eigen::Matrix4d init_transform = Eigen::Matrix4d::Identity();
    auto start = std::chrono::high_resolution_clock::now();
    auto result = open3d::pipelines::registration::RegistrationICP(
        *source, *target, threshold, init_transform,
        open3d::pipelines::registration::TransformationEstimationPointToPoint(),
        open3d::pipelines::registration::ICPConvergenceCriteria(100, 1e-6));
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    double milliseconds = duration.count() / 1000.0;

    // Output results
    std::cout << "Open3D ICP Computation Time: " << milliseconds << " ms\n";
    std::cout << "Known Transformation:\n" << known_transform << "\n";
    std::cout << "Recovered Transformation:\n" << result.transformation_ << "\n";

    // Check transformation error
    Eigen::Matrix4d error = result.transformation_.inverse() * known_transform;
    std::cout << "Transformation Error (should be near identity):\n" << error << "\n";

    // Apply recovered transform and sample a few points
    source->Transform(result.transformation_);
    std::cout << "Sample: Source | Transformed | Target\n";
    for (size_t i = 0; i < 5; ++i) {
        std::cout << source_vec[i].transpose() << " | "
                  << source->points_[i].transpose() << " | "
                  << target->points_[i].transpose() << "\n";
    }

    return 0;
}