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

def combine_transformations(rot_angles, translations):
    """Combine multiple rotation angles and translations into a total transformation."""
    total_rotation_matrix = np.eye(2)
    total_translation = np.zeros(2)

    for rot_angle, translation in zip(rot_angles, translations):
        # Compute rotation matrix for current step
        c, s = math.cos(rot_angle), math.sin(rot_angle)
        step_rotation_matrix = np.array([[c, -s], [s, c]])

        # Update total rotation (compose rotations)
        total_rotation_matrix = step_rotation_matrix @ total_rotation_matrix

        # Update total translation (apply current rotation to previous translation, then add new translation)
        total_translation = total_translation @ step_rotation_matrix.T + translation

    return total_rotation_matrix, total_translation

def test_cumulative_transformations():
    """Test summing multiple transformations and verify correctness."""
    np.random.seed(12345)

    # Generate initial set of points
    initial_points = np.random.uniform(-1, 1, (10, 2))

    # Define multiple transformations
    transformations = [
        (math.radians(15), np.array([0.5, 0.2])),  # 15° rotation, [0.5, 0.2] translation
        (math.radians(10), np.array([0.3, -0.1])),  # 10° rotation, [0.3, -0.1] translation
        (math.radians(5), np.array([-0.2, 0.4]))   # 5° rotation, [-0.2, 0.4] translation
    ]

    # Apply transformations step-by-step to generate point sets
    point_sets = [initial_points]
    current_points = initial_points.copy()

    for rot_angle, translation in transformations:
        c, s = math.cos(rot_angle), math.sin(rot_angle)
        R = np.array([[c, -s], [s, c]])
        current_points = (current_points @ R.T) + translation
        point_sets.append(current_points.copy())

    # Compute true total transformation by applying all steps directly
    true_total_rotation_matrix = np.eye(2)
    true_total_translation = np.zeros(2)
    for rot_angle, translation in transformations:
        c, s = math.cos(rot_angle), math.sin(rot_angle)
        R = np.array([[c, -s], [s, c]])
        true_total_rotation_matrix = R @ true_total_rotation_matrix
        true_total_translation = true_total_translation @ R.T + translation

    true_total_angle = math.atan2(true_total_rotation_matrix[1, 0], true_total_rotation_matrix[0, 0])

    # Estimate transformations step-by-step using point_based_matching
    estimated_rot_angles = []
    estimated_translations = []

    for i in range(len(point_sets) - 1):
        point_pairs = np.hstack((point_sets[i], point_sets[i + 1]))
        rot_angle_est, trans_x_est, trans_y_est = point_based_matching(point_pairs)
        estimated_rot_angles.append(rot_angle_est)
        estimated_translations.append(np.array([trans_x_est, trans_y_est]))

    # Combine estimated transformations
    est_total_rotation_matrix, est_total_translation = combine_transformations(estimated_rot_angles, estimated_translations)
    est_total_angle = math.atan2(est_total_rotation_matrix[1, 0], est_total_rotation_matrix[0, 0])

    # Print results
    print("True Step Transformations:")
    for i, (rot, trans) in enumerate(transformations):
        print(f"Step {i + 1}: Rotation = {math.degrees(rot):.1f}°, Translation = {trans}")

    print("\nEstimated Step Transformations:")
    for i, (rot, trans) in enumerate(zip(estimated_rot_angles, estimated_translations)):
        print(f"Step {i + 1}: Rotation = {math.degrees(rot):.6f}°, Translation = {trans}")

    print("\nTrue Total Transformation:")
    print(f"Rotation Matrix:\n{true_total_rotation_matrix}")
    print(f"Total Angle: {math.degrees(true_total_angle):.6f}°")
    print(f"Translation: {true_total_translation}")

    print("\nEstimated Total Transformation:")
    print(f"Rotation Matrix:\n{est_total_rotation_matrix}")
    print(f"Total Angle: {math.degrees(est_total_angle):.6f}°")
    print(f"Translation: {est_total_translation}")

    # Compute errors
    angle_error = abs(est_total_angle - true_total_angle)
    trans_error = np.linalg.norm(est_total_translation - true_total_translation)
    print(f"\nErrors:")
    print(f"Rotation Error: {math.degrees(angle_error):.6f}°")
    print(f"Translation Error: {trans_error:.6f}")

if __name__ == "__main__":
    test_cumulative_transformations()