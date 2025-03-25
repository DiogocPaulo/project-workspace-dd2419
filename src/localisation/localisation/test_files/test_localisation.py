#!/usr/bin/env python

import math
import numpy as np
import matplotlib.pyplot as plt

from laser_scan_storage import LaserScanStorage, LaserScanData
from icp import icp

def generate_laser_scan(pose: np.ndarray, num_points: int = 50) -> np.ndarray:
    """Generate a sample laser scan from a given pose.

    Args:
        pose: Robot pose as [x, y, theta] in world frame
        num_points: Number of points to generate

    Returns:
        Array of points in world coordinates (shape: [num_points, 2])
    """
    np.random.seed(12345)  # For reproducibility
    
    ranges = np.random.uniform(0.5, 2.0, num_points)
    angles = np.linspace(0, 2 * math.pi, num_points)
    
    local_x = ranges * np.cos(angles)
    local_y = ranges * np.sin(angles)
    local_points = np.vstack((local_x, local_y)).T
    
    theta = pose[2]
    rot = np.array([[math.cos(theta), -math.sin(theta)],
                   [math.sin(theta), math.cos(theta)]])
    world_points = np.dot(local_points, rot) + pose[:2]
    
    return world_points

def create_test_data_with_poses():
    """Create test data with poses and points simulating robot movement and error."""
    pose1 = np.array([0.0, 0.0, 0.0])
    points1 = generate_laser_scan(pose1)
    
    delta_x = 0.5 * math.cos(pose1[2])
    delta_y = 0.5 * math.sin(pose1[2])
    delta_theta = math.radians(15)
    pose2 = np.array([pose1[0] + delta_x, pose1[1] + delta_y, pose1[2] + delta_theta])
    points2 = generate_laser_scan(pose2)
    
    error_x = np.random.uniform(-0.1, 0.1)
    error_y = np.random.uniform(-0.1, 0.1)
    error_theta = np.random.uniform(-math.radians(5), math.radians(5))
    pose3_err = pose2 + np.array([error_x, error_y, error_theta])
    
    return pose1, points1, pose2, points2, pose3_err

def correct_pose_with_icp(reference_points: np.ndarray, points_to_align: np.ndarray,
                         initial_pose: np.ndarray) -> tuple[float, np.ndarray]:
    """Correct pose using ICP, mimicking map-to-odom transform.

    Args:
        reference_points: Reference scan points
        points_to_align: Points to align to reference
        initial_pose: Initial pose estimate [x, y, theta]

    Returns:
        Tuple of (corrected_theta, corrected_translation)
    """
    rotation, translation, aligned_points = icp(reference_points, points_to_align)
    
    # Handle different possible rotation outputs from icp()
    if isinstance(rotation, np.ndarray):
        if rotation.size == 1:  # Scalar wrapped in array, e.g., [0.123]
            rotation_scalar = float(rotation)
        elif rotation.shape == (2, 2):  # 2x2 rotation matrix
            rotation_scalar = np.arctan2(rotation[1, 0], rotation[0, 0])
        else:
            raise ValueError(f"Unexpected rotation format from icp: {rotation}")
    else:
        rotation_scalar = float(rotation)  # Assume it's already a scalar
    
    # Compute corrected pose adjustment
    current_yaw = initial_pose[2]
    corrected_theta = math.atan2(
        math.sin(current_yaw + rotation_scalar),
        math.cos(current_yaw + rotation_scalar)
    )
    corrected_translation = translation.flatten() if translation.ndim > 1 else translation
    
    return corrected_theta, corrected_translation, aligned_points

def transform_pose(initial_pose, translation, rotation_matrix):
    """Compute a new pose given an initial pose, translation vector, and rotation matrix."""
    # Ensure inputs are in the correct format
    initial_pose = np.asarray(initial_pose, dtype=float)
    translation = np.asarray(translation, dtype=float).flatten()
    rotation_matrix = np.asarray(rotation_matrix, dtype=float)

    if initial_pose.shape != (3,):
        raise ValueError(f"Initial pose must be a 3-element array, got {initial_pose.shape}")
    if translation.shape != (2,):
        raise ValueError(f"Translation must be a 2-element vector, got {translation.shape}")
    if rotation_matrix.shape != (2, 2):
        raise ValueError(f"Rotation matrix must be 2x2, got {rotation_matrix.shape}")

    # Extract initial position and orientation
    x, y, theta = initial_pose
    
    # Compute new position: apply translation in world frame
    x_new = x + translation[0]
    y_new = y + translation[1]
    
    # Compute new orientation: combine initial theta with rotation from matrix
    # Extract rotation angle from the 2x2 matrix
    delta_theta = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    theta_new = theta + delta_theta
    # Normalize theta to [-pi, pi]
    theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))
    
    return np.array([x_new, y_new, theta_new])

def plot_results(pose1, points1, pose2, points2, pose3_err, corrected_pose, aligned_points):
    """Plot the points and poses."""
    plt.figure(figsize=(10, 8))
    
    plt.scatter(points1[:, 0], points1[:, 1], c='r', marker='x', label='Reference Points (Pose 1)')
    plt.scatter(points2[:, 0], points2[:, 1], c='b', marker='o', label='True Points (Pose 2)')
    plt.scatter(aligned_points[:, 0], aligned_points[:, 1], c='g', marker='+', label='Corrected Points')
    
    arrow_len = 0.3
    plt.quiver(pose1[0], pose1[1], arrow_len * math.cos(pose1[2]),
               arrow_len * math.sin(pose1[2]), color='r', label='Pose 1')
    plt.quiver(pose2[0], pose2[1], arrow_len * math.cos(pose2[2]),
               arrow_len * math.sin(pose2[2]), color='b', label='Pose 2')
    plt.quiver(pose3_err[0], pose3_err[1], arrow_len * math.cos(pose3_err[2]),
               arrow_len * math.sin(pose3_err[2]), color='orange', label='Pose 3 (Err)')
    plt.quiver(corrected_pose[0], corrected_pose[1], arrow_len * math.cos(corrected_pose[2]),
               arrow_len * math.sin(corrected_pose[2]), color='g', label='Corrected Pose')
    
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.title("Laser Scan Points and Poses with ICP Correction")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.show()

if __name__ == "__main__":
    pose1, points1, pose2, points2, pose3_err = create_test_data_with_poses()
    
    storage = LaserScanStorage()
    scan1 = LaserScanData()
    scan2 = LaserScanData()
    
    scan1.store_points(points1, pose1, 0)
    scan2.store_points(points2, pose2, 0)
    
    storage.add_scan(scan1)
    storage.add_scan(scan2)

    rotation, translation, aligned_points = icp(points1, points2)

    print(f"Pose 1: {pose1}")

    print(f"Rotation: {rotation}")
    print(f"Translation: {translation}")

    # Correct the pose using roation and translation
    corrected_pose = transform_pose(pose3_err, translation, rotation)
    
    plot_results(pose1, points1, pose2, points2, pose3_err, corrected_pose, aligned_points)