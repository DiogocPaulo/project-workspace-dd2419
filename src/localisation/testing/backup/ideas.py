import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline

import time

class LidarScan:
    def __init__(self, points, timestamp=None):
        """Initialize LidarScan with points and optional timestamp."""
        self.timestamp = timestamp
        self.points = points
        self.line_segments = []

    def get_points(self):
        """Return the Cartesian points."""
        return self.points

    def extract_line_segments(self, distance_threshold=0.2, min_points=5):
        """Extract a single line segment using RANSAC.

        Args:
            distance_threshold: Max distance from line for points to be included.
            min_points: Minimum number of points to form a line segment.
        """
        points = self.points.copy()
        segment_found = True
        iteration = 0

        while segment_found:
            iteration += 1
            print(f"Iteration {iteration}")
            # RANSAC for line fitting
            model_ransac = make_pipeline(PolynomialFeatures(1), RANSACRegressor(min_samples=2, max_trials=100, residual_threshold=distance_threshold))
            model_ransac.fit(points[:, 0].reshape(-1, 1), points[:, 1])

            # Get inliers
            inlier_mask = model_ransac.named_steps['ransacregressor'].inlier_mask_
            inliers = points[inlier_mask]

            # Check if enough inliers are found
            if len(inliers) >= min_points:
                # Fit line to inliers
                line_model = np.polyfit(inliers[:, 0], inliers[:, 1], 1)
                line_x = np.array([min(inliers[:, 0]), max(inliers[:, 0])])
                line_y = np.polyval(line_model, line_x)

                # Store the line segment
                line_segment = {
                    'start_point': (line_x[0], line_y[0]),
                    'end_point': (line_x[1], line_y[1]),
                    'points': inliers
                }
                self.line_segments.append(line_segment)
                
                # Remove inliers from points
                points = points[~inlier_mask]
            else:
                segment_found = False

    def get_line_segments(self):
        """Return the extracted line segment."""
        return self.line_segments

    def visualize_scan(self, title="Lidar Scan with Line Segment"):
        """Visualizes the scan points and the extracted line segment."""
        plt.figure(figsize=(8, 8))
        plt.scatter(self.points[:, 0], self.points[:, 1], s=10, label="Scan Points")

        # Plot the line segments
        for segment in self.line_segments:
            plt.plot([segment['start_point'][0], segment['end_point'][0]],
                     [segment['start_point'][1], segment['end_point'][1]],
                     color='red', linewidth=2, label="Line Segment")

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title(title)
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

# Example usage
if __name__ == "__main__":
    # Define a more complicated room layout
    room_boundary = [(1, 0), (6, 2), (5, 5), (3, 7), (1, 6), (0, 4)]

    def generate_room_points(room_boundary, point_spacing=0.2, noise_std=0.05, outlier_percentage=0.05):
        """Generate points along the boundary of the room with added noise and outliers."""
        points = []
        for i in range(len(room_boundary)):
            start = np.array(room_boundary[i])
            end = np.array(room_boundary[(i + 1) % len(room_boundary)])
            distance = np.linalg.norm(end - start)
            num_points = max(int(distance / point_spacing), 1)
            for j in range(num_points):
                t = j / num_points
                point = start * (1 - t) + end * t
                points.append(point)

        points = np.array(points)
        noise = np.random.normal(0, noise_std, points.shape)
        points += noise

        num_outliers = int(len(points) * outlier_percentage)
        outliers = np.random.uniform(0, 1, (num_outliers, 2)) * np.array([max(points[:, 0]), max(points[:, 1])])
        points = np.vstack((points, outliers))

        return points

    points = generate_room_points(room_boundary, point_spacing=0.2, noise_std=0.05, outlier_percentage=0.05)
    scan = LidarScan(points)

    # Extract line segments
    start_time = time.time()
    scan.extract_line_segments(distance_threshold=0.1, min_points=10)
    end_time = time.time()
    duration = end_time - start_time
    print(f"Line segment extraction took {duration:.4f} seconds.")
    if scan.get_line_segments():
        print(f"Extracted {len(scan.get_line_segments())} line segments:")
    else:
        print("No line segment found.")

    scan.visualize_scan(title="Complicated Room Layout with Extracted Line Segment")
