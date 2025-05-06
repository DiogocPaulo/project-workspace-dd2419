
import numpy as np
import matplotlib.pyplot as plt
from lidar_scan import LidarScan

def visualize_scan(scan, title="Lidar Scan and Features"):
    """
    Visualizes the laser scan points and any extracted features.
    """
    points = scan.get_points_xy()
    corners = scan.get_corners()
    line_segments = scan.get_line_segments()

    plt.figure(figsize=(8, 8))
    plt.scatter(points[:, 0], points[:, 1], s=10, label="Scan Points")

    # Plot corners
    corner_points = [corner['position'] for corner in corners]
    if corner_points:
        corner_points = np.array(corner_points)
        plt.scatter(corner_points[:, 0], corner_points[:, 1], color='red', marker='x', s=50, label="Corners")

    # Plot line segments
    for segment in line_segments:
        start = segment['start_point']
        end = segment['end_point']
        plt.plot([start[0], end[0]], [start[1], end[1]], color='green', linewidth=2, label="Line Segments" if segment == line_segments[0] else "")

    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    # Example of a scan resembling a corner (part of a room)
    corner_room_ranges = np.array([1.0, 1.1, 1.2, 1.3, 1.4,
                                   2.5, 2.6, 2.7, 2.8, 2.9])
    corner_room_angles = np.array([0.1, 0.2, 0.3, 0.4, 0.5,
                                   1.5, 1.6, 1.7, 1.8, 1.9])
    corner_room_scan = LidarScan(corner_room_ranges, corner_room_angles)
    corner_room_scan.extract_line_segments(distance_threshold=1, max_point_gap=0.5)
    corner_room_scan.extract_corners(angle_threshold_rad=np.pi/6)
    visualize_scan(corner_room_scan, title="Lidar Scan Resembling a Corner")

    # Example of a scan resembling a straight wall
    wall_ranges = np.linspace(2.0, 2.5, 30)
    wall_angles = np.linspace(np.pi/4 - 0.02, np.pi/4 + 0.02, 30)
    wall_scan = LidarScan(wall_ranges, wall_angles)
    wall_scan.extract_line_segments(distance_threshold=0.1)
    wall_scan.extract_corners(angle_threshold_rad=np.pi/6)
    visualize_scan(wall_scan, title="Lidar Scan Resembling a Straight Wall")

    # Example of a scan with two perpendicular walls (a more defined corner)
    two_walls_ranges = np.concatenate([np.linspace(1.0, 2.0, 20), np.linspace(1.0, 1.5, 15)])
    two_walls_angles = np.concatenate([np.linspace(0.1, 0.5, 20), np.linspace(1.5, 1.9, 15)])
    two_walls_scan = LidarScan(two_walls_ranges, two_walls_angles)
    two_walls_scan.extract_line_segments(distance_threshold=0.1)
    two_walls_scan.extract_corners(angle_threshold_rad=np.pi/6)
    visualize_scan(two_walls_scan, title="Lidar Scan with Two Perpendicular Walls")

    # The previous curved scan example (for comparison)
    curved_ranges = np.linspace(1, 2, 50) + 0.2 * np.sin(np.linspace(0, 4 * np.pi, 50))
    curved_angles = np.linspace(0, np.pi, 50)
    curved_scan = LidarScan(curved_ranges, curved_angles)
    # Assuming you have implemented these methods with reasonable thresholds
    curved_scan.extract_line_segments(distance_threshold=0.1)
    curved_scan.extract_corners(angle_threshold_rad=np.pi/4, distance_threshold=0.5)
    visualize_scan(curved_scan, title="Curved Lidar Scan - Expecting Few/No Features (If Algorithms are Correct)")

