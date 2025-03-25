import numpy as np

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

def accumulate_transformations(point_pairs_list):
    total_rot_angle = 0.0
    total_translation = np.array([0.0, 0.0])

    for point_pairs in point_pairs_list:
        rot_angle, trans_x, trans_y = point_based_matching(point_pairs)

        if rot_angle is not None:
            # Accumulate rotation
            total_rot_angle += rot_angle

            # Accumulate translation
            rotation_matrix = np.array([
                [np.cos(total_rot_angle), -np.sin(total_rot_angle)],
                [np.sin(total_rot_angle), np.cos(total_rot_angle)]
            ])
            translated_vector = np.dot(rotation_matrix, np.array([trans_x, trans_y], dtype=float))
            total_translation += translated_vector

    return total_rot_angle, total_translation

# Example usage
point_pairs_list = [
    [[1, 2, 3, 4], [2, 3, 4, 5]],  # Example point pairs
    [[4, 5, 6, 7], [5, 6, 7, 8]]
]

total_rot_angle, total_translation = accumulate_transformations(point_pairs_list)
print("Total Rotation Angle:", total_rot_angle)
print("Total Translation:", total_translation)

