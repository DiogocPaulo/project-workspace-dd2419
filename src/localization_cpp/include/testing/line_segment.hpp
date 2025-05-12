#pragma once
#include <Eigen/Dense>
#include <vector>
#include <list>
#include <algorithm>
#include <numeric>
#include <utility>
#include <cmath>
#include <unordered_map>
#include <iostream>

namespace Localization {

class LineSegment {
public:
    // Constructor that takes points to initialize the segment
    explicit LineSegment(const std::vector<Eigen::Vector2d>& points, 
                         const Eigen::Vector2d& ref_point)
        : reference_point_(ref_point) {
        if (points.size() < 5) {
            throw std::runtime_error("Line segment must have at least 5 points.");
        }
        
        // Initialize the linked list with points
        for (const auto& point : points) {
            points_list_.push_back(point);
        }
        
        // Compute the average point-to-line distance
        merge_threshold_ = computeAveragePointToLineDistance();
        
        // Ensure points are sorted
        sortPointsList();
    }

    // Compute average distance of points to the line formed by adjacent points
    double computeAveragePointToLineDistance() const {
        if (points_list_.size() < 3) return 0.0;
        
        double total_distance = 0.0;
        int distance_count = 0;
        
        auto it = points_list_.begin();
        auto prev_it = it++;
        auto next_it = std::next(it);
        
        while (next_it != points_list_.end()) {
            // Compute distance from point to line formed by adjacent points
            double distance = computePointToLineDistance(*prev_it, *next_it, *it);
            total_distance += distance;
            distance_count++;
            
            prev_it = it;
            it = next_it;
            ++next_it;
        }
        
        // Return average distance
        return distance_count > 0 ? total_distance / distance_count : 0.0;
    }
#include "line_segment.hpp"
    // Compute distance from a point to a line defined by two points
    double computePointToLineDistance(const Eigen::Vector2d& line_start, 
                                     const Eigen::Vector2d& line_end, 
                                     const Eigen::Vector2d& point) const {
        // Compute line vector
        Eigen::Vector2d line_vec = line_end - line_start;
        Eigen::Vector2d point_vec = point - line_start;
        
        // Compute cross product magnitude (area of parallelogram)
        double cross_product_magnitude = std::abs(line_vec.x() * point_vec.y() - 
                                                 line_vec.y() * point_vec.x());
        
        // Compute line length
        double line_length = line_vec.norm();
        
        // Distance is cross product magnitude divided by line length
        return line_length > 0 ? cross_product_magnitude / line_length : 0.0;
    }

    // Find closest line segment for a point and calculate distance
    std::pair<std::list<Eigen::Vector2d>::const_iterator, double> 
    findClosestSegment(const Eigen::Vector2d& point) const {
        if (points_list_.size() < 2) {
            return {points_list_.begin(), std::numeric_limits<double>::max()};
        }
        
        auto min_it = points_list_.begin();
        double min_distance = std::numeric_limits<double>::max();
        
        auto it = points_list_.begin();
        auto next_it = std::next(it);
        
        while (next_it != points_list_.end()) {
            double distance = computePointToLineDistance(*it, *next_it, point);
            
            if (distance < min_distance) {
                min_distance = distance;
                min_it = it;
            }
            
            it = next_it;
            ++next_it;
        }
        
        return {min_it, min_distance};
    }

    // Find closest point in the segment
    std::list<Eigen::Vector2d>::const_iterator 
    findClosestPoint(const Eigen::Vector2d& point) const {
        if (points_list_.empty()) {
            return points_list_.end();
        }
        
        auto min_it = points_list_.begin();
        double min_distance = (point - *min_it).norm();
        
        for (auto it = std::next(min_it); it != points_list_.end(); ++it) {
            double distance = (point - *it).norm();
            if (distance < min_distance) {
                min_distance = distance;
                min_it = it;
            }
        }
        
        return min_it;
    }

    // Merge with another segment with optimized approach
    void merge(const LineSegment& other) {
        std::cout << "Merging segments..." << std::endl;
        
        if (points_list_.empty() || other.points_list_.empty()) {
            return;
        }
        
        // Find closest points for the endpoints of each segment
        auto this_front_to_other_it = other.findClosestPoint(points_list_.front());
        auto this_back_to_other_it = other.findClosestPoint(points_list_.back());
        auto other_front_to_this_it = findClosestPoint(other.points_list_.front());
        auto other_back_to_this_it = findClosestPoint(other.points_list_.back());
        
        // Calculate distances for endpoint relationships
        double this_front_to_other_dist = (points_list_.front() - *this_front_to_other_it).norm();
        double this_back_to_other_dist = (points_list_.back() - *this_back_to_other_it).norm();
        double other_front_to_this_dist = (other.points_list_.front() - *other_front_to_this_it).norm();
        double other_back_to_this_dist = (other.points_list_.back() - *other_back_to_this_it).norm();
        
        // Points to add before, after, and between segments
        std::list<Eigen::Vector2d> points_to_add;
        
        // Determine which endpoints extend beyond current segment
        bool extends_before_start = false;
        bool extends_after_end = false;
        
        // Check if other segment extends before our start
        if (std::distance(other.points_list_.begin(), this_front_to_other_it) > 0) {
            extends_before_start = true;
        }
        
        // Check if other segment extends after our end
        if (std::distance(this_back_to_other_it, other.points_list_.end()) > 1) {
            extends_after_end = true;
        }
        
        // Add points that extend before our segment
        if (extends_before_start) {
            auto it = other.points_list_.begin();
            while (it != this_front_to_other_it) {
                points_to_add.push_back(*it);
                std::cout << "Adding point before segment start" << std::endl;
                ++it;
            }
        }
        
        // Process interior points from other segment
        auto start_it = extends_before_start ? this_front_to_other_it : other.points_list_.begin();
        auto end_it = extends_after_end ? this_back_to_other_it : other.points_list_.end();
        
        // Create map to store insertion points for interior points
        std::map<std::list<Eigen::Vector2d>::const_iterator, std::vector<Eigen::Vector2d>, ListConstIteratorComparator> insertion_map;
        
        for (auto it = start_it; it != end_it; ++it) {
            // Skip if this point is too close to an endpoint
            if (it == this_front_to_other_it || it == this_back_to_other_it) {
                continue;
            }
            
            // Find closest segment and check distance
            auto [segment_it, distance] = findClosestSegment(*it);
            
            if (distance <= merge_threshold_) {
                // This point is close enough to our segment - insert after the segment start
                insertion_map[segment_it].push_back(*it);
                std::cout << "Adding interior point" << std::endl;
            }
        }
        
        // Add points that extend after our segment
        std::list<Eigen::Vector2d> after_points;
        if (extends_after_end) {
            auto it = std::next(this_back_to_other_it);
            while (it != other.points_list_.end()) {
                after_points.push_back(*it);
                std::cout << "Adding point after segment end" << std::endl;
                ++it;
            }
        }
        
        // Apply all changes to our point list
        // 1. Add points that extend before our segment
        points_list_.splice(points_list_.begin(), points_to_add);
        
        // 2. Insert interior points at appropriate positions
        for (auto& [pos_it, points_vec] : insertion_map) {
            auto insert_pos = std::next(pos_it);
            for (const auto& point : points_vec) {
                points_list_.insert(insert_pos, point);
            }
        }
        
        // 3. Add points that extend after our segment
        points_list_.splice(points_list_.end(), after_points);
        
        if (points_to_add.empty() && insertion_map.empty() && after_points.empty()) {
            std::cout << "No points added." << std::endl;
        }
        
        // Recompute merge threshold after adding points
        merge_threshold_ = computeAveragePointToLineDistance();
        
        // Make sure points are in proper order
        sortPointsList();
    }

    // Check if a point is close to the segment
    bool isPointCloseToSegment(const Eigen::Vector2d& point) const {
        auto [segment_it, distance] = findClosestSegment(point);
        return distance <= merge_threshold_;
    }

    // Sort points list based on angle from reference point
    void sortPointsList() {
        // Convert list to vector for sorting
        std::vector<std::pair<double, Eigen::Vector2d>> angle_point_pairs;
        
        for (const auto& point : points_list_) {
            Eigen::Vector2d diff = point - reference_point_;
            double angle = std::atan2(diff.y(), diff.x());
            angle_point_pairs.emplace_back(angle, point);
        }
        
        // Sort by angle
        std::sort(angle_point_pairs.begin(), angle_point_pairs.end(),
                 [](const auto& a, const auto& b) { return a.first < b.first; });
        
        // Clear list and repopulate in sorted order
        points_list_.clear();
        for (const auto& [angle, point] : angle_point_pairs) {
            points_list_.push_back(point);
        }
    }

    double match_score(const LineSegment& other) const {
        // Compute the number of points in common
        size_t common_points = 0;
        
        for (const auto& point : points_list_) {
            if (other.isPointCloseToSegment(point)) {
                common_points++;
            }
        }
        
        // Compute the match score as the ratio of common points to total points
        return static_cast<double>(common_points) / points_list_.size();
    }

    // Accessors
    std::vector<Eigen::Vector2d> points() const { 
        return std::vector<Eigen::Vector2d>(points_list_.begin(), points_list_.end()); 
    }
    
    const Eigen::Vector2d& referencePoint() const { return reference_point_; }
    double mergeThreshold() const { return merge_threshold_; }
    
    // Debug print
    void printPoints() const {
        std::cout << "Line Segment Points:" << std::endl;
        for (const auto& point : points_list_) {
            std::cout << "(" << point.x() << ", " << point.y() << ")" << std::endl;
        }
    }

private:
    std::list<Eigen::Vector2d> points_list_; // Ordered sequence of points as linked list
    Eigen::Vector2d reference_point_; // Reference point for angle-based sorting
    double merge_threshold_ = 0.0; // Dynamically computed merge threshold

    // Define a custom comparator for std::_List_const_iterator<Eigen::Vector2d>
    struct ListConstIteratorComparator {
        template <typename T>
        bool operator()(const std::_List_const_iterator<T>& lhs, const std::_List_const_iterator<T>& rhs) const {
            return &(*lhs) < &(*rhs); // Compare based on the memory address of the pointed-to object
        }
    };
};

} // namespace Localization