#!/usr/bin/env python

import numpy as np
import math

def point_based_matching(point_pairs):
    """Compute rotation and translation between two sets of matched points."""
    if len(point_pairs) == 0:
        return None, None, None

    point_pairs = np.array(point_pairs, dtype=float)
    source_points = point_pairs[:, :2]
    target_points = point_pairs[:, 2:]

    source_mean = np.mean(source_points, axis=0)
    target_mean = np.mean(target_points, axis=0)

    source_centered = source_points - source_mean
    target_centered = target_points - target_mean

    H = source_centered.T @ target_centered
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    rot_angle = np.arctan2(R[1, 0], R[0, 0])
    translation = target_mean - R @ source_mean

    return rot_angle, translation[0], translation[1]

def generate_test_data(num_points=10, rotation_deg=30, translation_x=1.0, translation_y=0.5):
    """Generate artificial source and target points with a known transformation."""
    np.random.seed(12345)

    # Generate source points
    source_points = np.random.uniform(-1, 1, (num_points, 2))

    # Define true transformation
    theta = math.radians(rotation_deg)
    R_true = np.array([[math.cos(theta), -math.sin(theta)],
                      [math.sin(theta), math.cos(theta)]])
    T_true = np.array([translation_x, translation_y])

    # Generate target points by applying transformation
    target_points = (source_points @ R_true.T) + T_true

    # Combine into point pairs [x_s, y_s, x_t, y_t]
    point_pairs = np.hstack((source_points, target_points))

    return point_pairs, R_true, T_true

def test_point_based_matching():
    """Test the point_based_matching function with artificial data."""
    # Test case 1: Standard transformation (30 degrees, [1.0, 0.5])
    point_pairs, R_true, T_true = generate_test_data(num_points=10, rotation_deg=30, translation_x=1.0, translation_y=0.5)
    
    # Run the function
    rot_angle_est, trans_x_est, trans_y_est = point_based_matching(point_pairs)

    # Expected rotation angle
    theta_true = math.radians(30)
    
    # Print results
    print("Test Case 1: Standard Transformation")
    print(f"True Rotation Matrix:\n{R_true}")
    print(f"True Translation: {T_true}")
    print(f"True Rotation Angle: {theta_true:.6f} radians")
    print(f"Estimated Rotation Angle: {rot_angle_est:.6f} radians")
    print(f"Estimated Translation: [{trans_x_est:.6f}, {trans_y_est:.6f}]")
    print(f"Rotation Error: {abs(rot_angle_est - theta_true):.6f} radians")
    print(f"Translation Error: {np.linalg.norm(np.array([trans_x_est, trans_y_est]) - T_true):.6f}\n")

    # Test case 2: No rotation, only translation
    point_pairs_no_rot, R_true_no_rot, T_true_no_rot = generate_test_data(num_points=10, rotation_deg=0, translation_x=2.0, translation_y=-1.0)
    rot_angle_est_no_rot, trans_x_est_no_rot, trans_y_est_no_rot = point_based_matching(point_pairs_no_rot)

    print("Test Case 2: No Rotation, Only Translation")
    print(f"True Rotation Matrix:\n{R_true_no_rot}")
    print(f"True Translation: {T_true_no_rot}")
    print(f"True Rotation Angle: 0.0 radians")
    print(f"Estimated Rotation Angle: {rot_angle_est_no_rot:.6f} radians")
    print(f"Estimated Translation: [{trans_x_est_no_rot:.6f}, {trans_y_est_no_rot:.6f}]")
    print(f"Rotation Error: {abs(rot_angle_est_no_rot):.6f} radians")
    print(f"Translation Error: {np.linalg.norm(np.array([trans_x_est_no_rot, trans_y_est_no_rot]) - T_true_no_rot):.6f}\n")

    # Test case 3: Only rotation, no translation
    point_pairs_only_rot, R_true_only_rot, T_true_only_rot = generate_test_data(num_points=10, rotation_deg=45, translation_x=0.0, translation_y=0.0)
    rot_angle_est_only_rot, trans_x_est_only_rot, trans_y_est_only_rot = point_based_matching(point_pairs_only_rot)

    theta_true_only_rot = math.radians(45)
    print("Test Case 3: Only Rotation, No Translation")
    print(f"True Rotation Matrix:\n{R_true_only_rot}")
    print(f"True Translation: {T_true_only_rot}")
    print(f"True Rotation Angle: {theta_true_only_rot:.6f} radians")
    print(f"Estimated Rotation Angle: {rot_angle_est_only_rot:.6f} radians")
    print(f"Estimated Translation: [{trans_x_est_only_rot:.6f}, {trans_y_est_only_rot:.6f}]")
    print(f"Rotation Error: {abs(rot_angle_est_only_rot - theta_true_only_rot):.6f} radians")
    print(f"Translation Error: {np.linalg.norm(np.array([trans_x_est_only_rot, trans_y_est_only_rot]) - T_true_only_rot):.6f}")

if __name__ == "__main__":
    test_point_based_matching()