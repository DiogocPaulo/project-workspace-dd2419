import numpy as np
from sklearn.neighbors import NearestNeighbors

"""
from numba import jit

@jit(nopython=True)
"""

def point_based_matching(point_pairs):
    if len(point_pairs) == 0:
        return None, None, None

    # Convert to NumPy arrays for better slicing
    point_pairs = np.array(point_pairs)  
    source_points = point_pairs[:, :2]  # Extract (x, y) from the first scan
    target_points = point_pairs[:, 2:]  # Extract (x', y') from the second scan

    # Compute centroids
    source_mean = np.mean(source_points, axis=0)
    target_mean = np.mean(target_points, axis=0)

    # Center the points
    source_centered = source_points - source_mean
    target_centered = target_points - target_mean

    # Compute H (covariance) matrix
    H = source_centered.T @ target_centered  # (2xN) @ (Nx2) = (2x2)

    # Compute rotation angle using arctan2
    U, _, Vt = np.linalg.svd(H)  # Singular Value Decomposition
    R = Vt.T @ U.T  # Compute rotation matrix

    if np.linalg.det(R) < 0:  # Ensure proper rotation
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    rot_angle = np.arctan2(R[1, 0], R[0, 0])  # Extract rotation from matrix
    translation = target_mean - R @ source_mean  # Compute translation

    return rot_angle, translation[0], translation[1]

def icp(reference_points, points, max_iterations=100,
        convergence_translation_threshold=1e-3, convergence_rotation_threshold=1e-4, 
        point_pairs_threshold=10):

    # Initialize nearest neighbor search
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(reference_points)

    # Cumulative transformation
    total_rotation = np.eye(2)  # Identity matrix (2x2)
    total_translation = np.zeros(2)  # (x, y) translation

    for i in range(max_iterations):

        # Find the closest points
        distances, indices = nbrs.kneighbors(points)
        median_distance = np.median(distances)
        mask = distances.flatten() < median_distance  # Filter valid point pairs
        if np.sum(mask) < point_pairs_threshold:
            break

        # Extract valid matched pairs
        closest_point_pairs = np.hstack((points[mask], reference_points[indices[mask].flatten()]))
        #valid_indices = indices[mask.flatten(), 0]  # Convert to 1D array of integer indices
        #closest_point_pairs = np.hstack((points[mask], reference_points[valid_indices]))


        # Compute transformation using point-based matching
        closest_rot_angle, closest_translation_x, closest_translation_y = point_based_matching(closest_point_pairs)

        # Compute rotation matrix
        c, s = np.cos(closest_rot_angle), np.sin(closest_rot_angle)
        rotation_matrix = np.array([[c, -s], [s, c]])

        # Apply transformation
        points = (points @ rotation_matrix.T) + np.array([closest_translation_x, closest_translation_y])

        # Update cumulative transformation
        total_rotation = rotation_matrix @ total_rotation  # Rotate previous rotation
        total_translation = rotation_matrix @ total_translation + np.array([closest_translation_x, closest_translation_y])

        # Check for convergence
        if (abs(closest_rot_angle) < convergence_rotation_threshold and
                abs(closest_translation_x) < convergence_translation_threshold and
                abs(closest_translation_y) < convergence_translation_threshold):
            break

    return total_rotation, total_translation, points