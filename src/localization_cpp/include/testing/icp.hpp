#ifndef LOCALISATION_ICP_HPP
#define LOCALISATION_ICP_HPP

#include <vector>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

namespace Localization {

class ICP {
public:
    // Constructor with configurable parameters
    ICP(double threshold = 0.2, int max_iterations = 50)
        : threshold_(threshold), criteria_(max_iterations) {
        // Preallocate target cloud to avoid repeated allocations
        target_cloud_ = std::make_shared<open3d::geometry::PointCloud>();
    }

    // Set target points once (for static targets)
    void setTarget(const std::vector<Eigen::Vector2d>& target_pts) {
        target_pts_ = target_pts; // Store 2D points for reuse
        target_cloud_->points_.clear();
        target_cloud_->points_.reserve(target_pts_.size());
        for (const auto& p : target_pts_) {
            target_cloud_->points_.emplace_back(p.x(), p.y(), 0.0);
        }
        // Precompute KD-tree for target (optional, if Open3D supports lazy building)
        target_cloud_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(threshold_, 30));
    }

    // Perform ICP and return transformation
    Eigen::Matrix3d computeTransform(const std::vector<Eigen::Vector2d>& source_pts) {
        if (target_pts_.empty()) {
            std::cerr << "Target not set for ICP!" << std::endl;
            return Eigen::Matrix3d::Identity();
        }

        // Convert source to Open3D PointCloud (reuse preallocated memory if possible)
        auto source_cloud = std::make_shared<open3d::geometry::PointCloud>();
        source_cloud->points_.reserve(source_pts.size());
        for (const auto& p : source_pts) {
            source_cloud->points_.emplace_back(p.x(), p.y(), 0.0);
        }

        // Run ICP
        auto result = open3d::pipelines::registration::RegistrationICP(
            *source_cloud, *target_cloud_, threshold_, Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationPointToPoint(),
            criteria_
        );

        // Extract 2D transform efficiently
        Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
        transform.block<2, 2>(0, 0) = result.transformation_.block<2, 2>(0, 0);
        transform.block<2, 1>(0, 2) = result.transformation_.block<2, 1>(0, 3);

        return transform;
    }

    // Get aligned points (optional, avoids redundant transform computation)
    std::vector<Eigen::Vector2d> getAlignedPoints(const std::vector<Eigen::Vector2d>& source_pts) {
        if (target_pts_.empty()) {
            std::cerr << "Target not set for ICP!" << std::endl;
            return source_pts;
        }

        // Reuse computeTransform’s logic but apply transform to points
        auto source_cloud = std::make_shared<open3d::geometry::PointCloud>();
        source_cloud->points_.reserve(source_pts.size());
        for (const auto& p : source_pts) {
            source_cloud->points_.emplace_back(p.x(), p.y(), 0.0);
        }

        auto result = open3d::pipelines::registration::RegistrationICP(
            *source_cloud, *target_cloud_, threshold_, Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationPointToPoint(),
            criteria_
        );

        source_cloud->Transform(result.transformation_);
        std::vector<Eigen::Vector2d> aligned_pts;
        aligned_pts.reserve(source_cloud->points_.size());
        for (const auto& p : source_cloud->points_) {
            aligned_pts.emplace_back(p.x(), p.y());
        }

        return aligned_pts;
    }

    // Combined method: Get transform and aligned points in one call
    void computeICP(const std::vector<Eigen::Vector2d>& source_pts,
                    Eigen::Matrix3d& transform,
                    std::vector<Eigen::Vector2d>& aligned_pts) {
        if (target_pts_.empty()) {
            std::cerr << "Target not set for ICP!" << std::endl;
            transform = Eigen::Matrix3d::Identity();
            aligned_pts = source_pts;
            return;
        }

        auto source_cloud = std::make_shared<open3d::geometry::PointCloud>();
        source_cloud->points_.reserve(source_pts.size());
        for (const auto& p : source_pts) {
            source_cloud->points_.emplace_back(p.x(), p.y(), 0.0);
        }

        auto result = open3d::pipelines::registration::RegistrationICP(
            *source_cloud, *target_cloud_, threshold_, Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationPointToPoint(),
            criteria_
        );

        // Extract transform
        transform.block<2, 2>(0, 0) = result.transformation_.block<2, 2>(0, 0);
        transform.block<2, 1>(0, 2) = result.transformation_.block<2, 1>(0, 3);

        // Apply transform and extract aligned points
        source_cloud->Transform(result.transformation_);
        aligned_pts.clear();
        aligned_pts.reserve(source_cloud->points_.size());
        for (const auto& p : source_cloud->points_) {
            aligned_pts.emplace_back(p.x(), p.y());
        }
    }

private:
    double threshold_;
    open3d::pipelines::registration::ICPConvergenceCriteria criteria_;
    std::vector<Eigen::Vector2d> target_pts_; // Cache 2D points
    std::shared_ptr<open3d::geometry::PointCloud> target_cloud_; // Preallocated target
};

} // namespace Localization

#endif // LOCALISATION_ICP_HPP