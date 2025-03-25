import numpy as np
import matplotlib.pyplot as plt

def align_boundaries_by_centering(source, target):
    """Aligns boundary2 to boundary1 by centering both sets and then translating boundary2 to boundary1's centroid."""
    # Step 1: Compute the centroids of both boundaries
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    
    # Step 2: Move source to the position of targets's centroid
    source_aligned = source + (target_centroid - source_centroid)

    return source_aligned

# Example usage:
source = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])  # Square
target = np.array([[0, 0], [0, 1], [-1, 1], [-1, 0]])  # Rotated square

source_aligned = align_boundaries_by_centering(source, target)

# Plotting
plt.figure(figsize=(6,6))
plt.plot(source[:, 0], source[:, 1], label='Source', color='blue')
plt.plot(target[:, 0], target[:, 1], label='Target', color='red')
plt.plot(source_aligned[:, 0], source_aligned[:, 1], label='Aligned Source', color='green')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
