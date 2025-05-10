#pragma once

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <Eigen/Core>
#include <unordered_map>
#include <memory>
#include <string>

namespace Localization {

class PoseGraphOptimizer {
public:
    PoseGraphOptimizer() {
        // Initialize solver and optimizer
        auto linearSolver = std::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            std::make_unique<g2o::BlockSolverX>(std::move(linearSolver)));
        optimizer_.setAlgorithm(solver);
        optimizer_.setVerbose(false); // Disable verbose output by default
    }

    ~PoseGraphOptimizer() {
        optimizer_.clear();
    }

    int addVertex(const Eigen::Isometry3d& pose, bool is_fixed = false) {
        g2o::VertexSE3* vertex = new g2o::VertexSE3();
        vertex->setId(static_cast<int>(vertices_.size()));
        vertex->setEstimate(pose);
        vertex->setFixed(is_fixed);
        optimizer_.addVertex(vertex);
        vertices_[vertex->id()] = vertex;
        return vertex->id();
    }

    void addEdge(int from_id, int to_id, 
                const Eigen::Isometry3d& relative_pose,
                const Eigen::Matrix<double, 6, 6>& information_matrix,
                bool use_robust_kernel = false,
                double kernel_width = 1.0) {
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        edge->setVertex(0, optimizer_.vertex(from_id));
        edge->setVertex(1, optimizer_.vertex(to_id));
        edge->setMeasurement(relative_pose);
        edge->setInformation(information_matrix);

        if (use_robust_kernel) {
            auto rk = g2o::RobustKernelFactory::instance()->construct("Huber");
            if (rk) {
                rk->setDelta(kernel_width);
                edge->setRobustKernel(rk);
            }
        }

        optimizer_.addEdge(edge);
    }

    void optimize(int iterations = 20, bool verbose = false) {
        optimizer_.setVerbose(verbose);
        optimizer_.initializeOptimization();
        optimizer_.optimize(iterations);
    }

    Eigen::Isometry3d getPose(int vertex_id) const {
        auto* vertex = dynamic_cast<const g2o::VertexSE3*>(optimizer_.vertex(vertex_id));
        if (!vertex) {
            throw std::runtime_error("Vertex not found or wrong type");
        }
        return vertex->estimate();
    }

    void saveGraph(const std::string& filename) const {
        optimizer_.save(filename.c_str());
    }

private:
    g2o::SparseOptimizer optimizer_;
    std::unordered_map<int, g2o::VertexSE3*> vertices_;
};

} // namespace Localization
