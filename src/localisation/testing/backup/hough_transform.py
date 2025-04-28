from scan_from_verticies import generate_room_points, visualize_room
from lidar_scan import LidarScan

import numpy as np
import time

# Generate points for a room with noise and outliers
room_boundary = [(1, 0), (6, 2), (5, 5), (3, 7), (1, 6), (0, 4)]
points = generate_room_points(room_boundary, point_spacing=0.2, noise_std=0.05, outlier_percentage=0.05)
points = np.array(points)
visualize_room(room_boundary, points)

# Genereate ojects in the room
object1_boundary = [(2, 2), (2.2, 2.5)]
object1_points = generate_room_points(object1_boundary, point_spacing=0.2, noise_std=0.05, outlier_percentage=0.05)
object1_points = np.array(object1_points)
visualize_room(object1_boundary, object1_points)

# Genereate ojects in the room
object2_boundary = [(4.8, 3), (5.0, 2.5)]
object2_points = generate_room_points(object2_boundary, point_spacing=0.2, noise_std=0.05, outlier_percentage=0.05)
object2_points = np.array(object2_points)
visualize_room(object2_boundary, object2_points)

# Add object points to the room points
points = np.concatenate([points, object1_points])
points = np.concatenate([points, object2_points])
visualize_room(room_boundary, points)

scan = LidarScan(points)
start_time = time.time()
scan.extract_line_segments(distance_threshold=0.5, min_points=4, max_point_gap=1)
end_time = time.time()
duration = end_time - start_time
print(f"Line segment extraction took {duration:.4f} seconds.")
print(f"Extracted {len(scan.get_line_segments())} line segments:")

scan.visualize_scan(title="Lidar Scan with Extracted Line Segments")