import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor

from scan_from_verticies import generate_room_points, visualize_room

def ransac_line_fitting(points):
    """Fits a line to noisy points using RANSAC."""
    points = np.array(points)
    X = points[:, 0].reshape(-1, 1)  # x-values
    y = points[:, 1]  # y-values

    ransac = RANSACRegressor()
    ransac.fit(X, y)
    
    slope = ransac.estimator_.coef_[0]
    intercept = ransac.estimator_.intercept_

    return slope, intercept

# Define room boundary (for a rectangular room)
room_boundary = [(1, 0), (6, 2), (3, 4), (0, 4)]  # Example room vertices

# Generate points with noise and outliers
points = generate_room_points(room_boundary, point_spacing=0.2, noise_std=0.05, outlier_percentage=0.05)

# Ensure points are converted to a NumPy array
points = np.array(points)

# Fit line using RANSAC
m, b = ransac_line_fitting(points)

# Generate fitted line values over the x-range of the data
x_vals = np.linspace(np.min(points[:, 0]), np.max(points[:, 0]), 100)
y_vals = m * x_vals + b


# Plot the results
plt.scatter(points[:, 0], points[:, 1], label="Laser Points", color='blue', alpha=0.5)
plt.plot(x_vals, y_vals, 'r-', label=f"RANSAC Line: y={m:.2f}x + {b:.2f}")
plt.legend()
plt.xlabel("X")
plt.ylabel("Y")
plt.title("RANSAC Line Fitting")
plt.grid(True)
plt.show()
