import numpy as np
from sklearn.neighbors import NearestNeighbors

def estimate_normals(points, k=5):
    """Estimate normals for points using k-nearest neighbors."""
    n_points = len(points)
    if n_points < 2:
        return np.zeros((n_points, 2))  # No meaningful normal for 0 or 1 point
    
    # Adjust k to be at most n_points - 1 (excluding the point itself)
    k = min(k, n_points - 1)
    nbrs = NearestNeighbors(n_neighbors=k + 1, algorithm='kd_tree').fit(points)
    distances, indices = nbrs.kneighbors(points)
    
    normals = np.zeros((n_points, 2))
    for i in range(n_points):
        neighbors = points[indices[i, 1:]]  # Exclude the point itself
        if len(neighbors) < 2:  # Need at least 2 points for covariance
            normals[i] = np.array([0, 1])  # Default normal if too few neighbors
            continue
        cov = np.cov(neighbors.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        normal = eigenvectors[:, np.argmin(eigenvalues)]  # Smallest eigenvalue direction
        normals[i] = normal / np.linalg.norm(normal)  # Normalize
    
    return normals

def point_to_plane_matching(source_points, target_points, target_normals):
    """Compute transformation by minimizing point-to-plane distances."""
    if len(source_points) == 0 or len(target_points) == 0 or len(target_normals) != len(target_points):
        return None, None, None

    source_points = np.array(source_points)
    target_points = np.array(target_points)
    target_normals = np.array(target_normals)

    A = np.zeros((len(source_points), 3))  # tx, ty, theta
    b = np.zeros(len(source_points))

    for i in range(len(source_points)):
        p = source_points[i]
        q = target_points[i]
        n = target_normals[i]
        
        A[i, 0] = n[0]  # tx
        A[i, 1] = n[1]  # ty
        A[i, 2] = -n[0] * p[1] + n[1] * p[0]  # theta
        b[i] = n @ (q - p)  # Point-to-plane distance

    try:
        x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        tx, ty, theta = x
    except np.linalg.LinAlgError:
        return None, None, None

    rot_angle = theta
    translation = np.array([tx, ty])
    return rot_angle, translation[0], translation[1]

def point_to_plane_icp(reference_points, points, max_iterations=100,
        convergence_translation_threshold=1e-3, convergence_rotation_threshold=1e-4,
        point_pairs_threshold=10):
    """Point-to-plane ICP implementation."""
    reference_points = np.array(reference_points)
    points = np.array(points)
    
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(reference_points)
    reference_normals = estimate_normals(reference_points)

    total_rotation = np.eye(2)
    total_translation = np.zeros(2)

    for i in range(max_iterations):
        distances, indices = nbrs.kneighbors(points)
        median_distance = np.median(distances)
        mask = distances.flatten() < median_distance
        if np.sum(mask) < point_pairs_threshold:
            break

        source_subset = points[mask]
        target_indices = indices[mask].flatten()
        target_subset = reference_points[target_indices]
        target_normals_subset = reference_normals[target_indices]

        rot_angle, tx, ty = point_to_plane_matching(source_subset, target_subset, target_normals_subset)
        if rot_angle is None:
            break

        c, s = np.cos(rot_angle), np.sin(rot_angle)
        rotation_matrix = np.array([[c, -s], [s, c]])
        points = (points @ rotation_matrix.T) + np.array([tx, ty])

        total_rotation = rotation_matrix @ total_rotation
        total_translation = rotation_matrix @ total_translation + np.array([tx, ty])

        if (abs(rot_angle) < convergence_rotation_threshold and
                abs(tx) < convergence_translation_threshold and
                abs(ty) < convergence_translation_threshold):
            break

    return total_rotation, total_translation, points

# Example usage
if __name__ == "__main__":
    # Test with more points to avoid the neighbor issue
    ref_points = np.array([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0], [5, 0], [6, 0]])
    src_points = np.array([[0.1, 0.2], [1.1, 0.3], [2.1, 0.1], [3.0, 0.2], [4.1, 0.3], [5.1, 0.2], [6.1, 0.3]])

    rotation, translation, aligned_points = point_to_plane_icp(ref_points, src_points)
    print("Rotation matrix:\n", rotation)
    print("Translation:", translation)
    print("Aligned points:\n", aligned_points)