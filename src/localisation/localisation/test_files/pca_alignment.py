import numpy as np
import matplotlib.pyplot as plt

def pca_alignment(boundary1, boundary2):
    """Aligns boundary2 to boundary1 using PCA after centering both sets."""
    def compute_pca(points):
        mean = np.mean(points, axis=0)
        centered = points - mean
        U, S, Vt = np.linalg.svd(centered, full_matrices=False)
        return Vt, mean  # Principal components + centroid
    
    # Center both point sets by subtracting their centroids
    mean1 = np.mean(boundary1, axis=0)
    mean2 = np.mean(boundary2, axis=0)
    
    boundary1_centered = boundary1 - mean1
    boundary2_centered = boundary2 - mean2
    
    # Compute PCA for both centered point sets
    V1, _ = compute_pca(boundary1_centered)
    V2, _ = compute_pca(boundary2_centered)
    
    # Compute initial rotation (align principal axes)
    R_init = V1.T @ V2  # Align the principal components
    
    # Apply the rotation to boundary2
    boundary2_aligned = (boundary2_centered @ R_init) + mean1

    return boundary2_aligned, R_init

# Example usage:
boundary1 = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])  # Square
boundary2 = np.array([[0, 0], [0, 1], [-1, 1], [-1, 0]])  # Rotated square

aligned_boundary, _ = pca_alignment(boundary1, boundary2)

# Plotting
plt.figure(figsize=(6,6))
plt.plot(boundary1[:, 0], boundary1[:, 1], label='Boundary 1', color='blue')
plt.plot(boundary2[:, 0], boundary2[:, 1], label='Boundary 2', color='red')
plt.plot(aligned_boundary[:, 0], aligned_boundary[:, 1], label='Aligned Boundary 2', color='green')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
