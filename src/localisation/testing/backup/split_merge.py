import numpy as np
import matplotlib.pyplot as plt
import cv2

from scan_from_verticies import generate_room_points, visualize_room

def fit_line_to_points(points):
    """Fits a line to a set of points using OpenCV."""
    points = np.array(points, dtype=np.float32)
    [vx, vy, x0, y0] = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
    
    # Compute line endpoints for plotting
    x1, y1 = x0 - vx * 100, y0 - vy * 100
    x2, y2 = x0 + vx * 100, y0 + vy * 100
    
    return (x1, y1, x2, y2)

# Define room boundary
room_boundary = [(1, 0), (6, 2), (3, 4), (0, 4)]  # Example room vertices

# Generate points with noise and outliers
noisy_points = generate_room_points(room_boundary, point_spacing=0.2, noise_std=0.05, outlier_percentage=0.05)

# Fit line
x1, y1, x2, y2 = fit_line_to_points(noisy_points)

# Visualize results
plt.scatter(*zip(*noisy_points), label="Laser Points", color='blue', s=10)
plt.plot([x1, x2], [y1, y2], 'r-', label="Fitted Line")
plt.legend()
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Line Fitting with Noisy Room Data")
plt.grid()
plt.show()
