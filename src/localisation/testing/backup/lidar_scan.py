import numpy as np
import matplotlib.pyplot as plt

def convert_to_xy(ranges, angles):
        """Convert polar coordinates to Cartesian (x, y) points, filtering invalid ranges."""
        mask = np.isfinite(ranges) & (ranges >= 0)
        cos_angles = np.cos(angles[mask])
        sin_angles = np.sin(angles[mask])
        return np.column_stack((ranges[mask] * cos_angles, ranges[mask] * sin_angles))

class LidarScan:
    def __init__(self, points, timestamp=None):
        """Initialize LidarScan with ranges, angles, and optional timestamp."""
        self.timestamp = timestamp
        self.points = points
        self.line_segments = []
        self.corners = []

    def get_points(self):
        """Return the Cartesian points."""
        return self.points

    def extract_line_segments(self, distance_threshold=0.1, min_points=5, max_point_gap=0.2):
        """Extract line segments using Split-and-Merge algorithm.

        Args:
            distance_threshold: Max distance from line for points to be included (curvature).
            min_points: Minimum number of points to form a line segment.
            max_point_gap: Max Euclidean distance between consecutive points.
        """
        if len(self.points) < min_points:
            self.line_segments = []
            return

        points = self.points
        self.line_segments = []

        def split_segment(start_idx, end_idx):
            """Recursively split segment based on distance from line."""
            if end_idx - start_idx + 1 < min_points:
                return

            p1 = points[start_idx]
            p2 = points[end_idx]
            segment_vec = p2 - p1
            segment_len_sq = np.sum(segment_vec**2)
            if segment_len_sq < 1e-6:
                return

            # Compute distances from line for all points in range
            segment_points = points[start_idx:end_idx + 1]
            distances = np.abs(np.cross(segment_vec, segment_points - p1)) / np.sqrt(segment_len_sq)
            max_dist_idx = np.argmax(distances) + start_idx
            max_dist = distances[max_dist_idx - start_idx]

            # Check consecutive point gaps
            gaps = np.linalg.norm(np.diff(segment_points, axis=0), axis=1)
            if np.any(gaps > max_point_gap):
                split_idx = np.argmax(gaps) + start_idx + 1
                split_segment(start_idx, split_idx - 1)
                split_segment(split_idx, end_idx)
                return

            if max_dist > distance_threshold:
                split_segment(start_idx, max_dist_idx)
                split_segment(max_dist_idx, end_idx)
            else:
                self.line_segments.append({
                    'start_index': start_idx,
                    'end_index': end_idx,
                    'start_point': p1,
                    'end_point': p2,
                    'points': segment_points
                })

        # Start with the full point set
        split_segment(0, len(points) - 1)

        # Filter segments by min_points (in case splits leave small fragments)
        self.line_segments = [seg for seg in self.line_segments if (seg['end_index'] - seg['start_index'] + 1) >= min_points]

    def get_line_segments(self):
        """Return the extracted line segments."""
        return self.line_segments

    def _line_intersection(self, p1, v1, p2, v2):
        """Compute intersection point of two lines, return None if parallel."""
        cross_vv = np.cross(v1, v2)
        if np.abs(cross_vv) < 1e-9:
            return None
        t = np.cross(p2 - p1, v2) / cross_vv
        return p1 + t * v1

    def extract_corners(self, distance_threshold=0.2, angle_threshold_rad=np.pi/6, max_segment_gap=0.5):
        """Extract corners from consecutive line segments efficiently.

        Args:
            distance_threshold: Max distance from segment endpoints to intersection.
            angle_threshold_rad: Min angle between segments for a corner.
            max_segment_gap: Max distance between consecutive segment endpoints.
        """
        self.corners = []
        segments = self.line_segments
        if len(segments) < 2:
            return

        for i in range(len(segments) - 1):
            seg1 = segments[i]
            seg2 = segments[i + 1]

            # Check if segments are consecutive and close enough
            dist_end_to_start = np.linalg.norm(seg1['end_point'] - seg2['start_point'])
            if dist_end_to_start > max_segment_gap:
                continue

            p1 = seg1['start_point']
            v1 = seg1['end_point'] - p1
            p2 = seg2['start_point']
            v2 = seg2['end_point'] - p2

            intersection = self._line_intersection(p1, v1, p2, v2)
            if intersection is None:
                continue

            # Check proximity to endpoints
            dist1_end = np.linalg.norm(intersection - seg1['end_point'])
            dist2_start = np.linalg.norm(intersection - seg2['start_point'])
            if dist1_end > distance_threshold or dist2_start > distance_threshold:
                continue

            # Compute angle between segments
            angle = np.arctan2(np.cross(v1, v2), np.dot(v1, v2))
            abs_angle = np.abs(angle)
            if abs_angle > angle_threshold_rad and abs_angle < (np.pi - angle_threshold_rad):
                distances = np.linalg.norm(self.points - intersection, axis=1)
                closest_index = np.argmin(distances)
                self.corners.append({
                    'index': closest_index,
                    'position': intersection,
                    'segment_indices': (i, i + 1)
                })

    def add_corner(self, index, position):
        """Manually add a corner."""
        self.corners.append({'index': index, 'position': np.array(position)})

    def get_corners(self):
        """Return the extracted corners."""
        return self.corners

    def __len__(self):
        """Return number of points."""
        return len(self.points)

    def __str__(self):
        """String representation."""
        return f"LidarScan(n_points={len(self)}, timestamp={self.timestamp}, n_corners={len(self.corners)}, n_lines={len(self.line_segments)})"

    def visualize_scan(self, title="Lidar Scan with Features"):
        """Visualizes the scan points, line segments, and corners."""
        plt.figure(figsize=(8, 8))
        plt.scatter(self.points[:, 0], self.points[:, 1], s=10, label="Scan Points")

        # Plot line segments
        for segment in self.line_segments:
            plt.plot([segment['start_point'][0], segment['end_point'][0]],
                     [segment['start_point'][1], segment['end_point'][1]],
                     color='red', linewidth=2, label="Line Segments" if segment == self.line_segments[0] else "")

        # Plot corners
        corner_points = np.array([corner['position'] for corner in self.corners])
        if corner_points.size > 0:
            plt.scatter(corner_points[:, 0], corner_points[:, 1], color='green', marker='x', s=50, label="Corners")

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title(title)
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.show()

# Test with an L-shape scan
if __name__ == "__main__":
    angles = np.linspace(0, np.pi/2, 100)
    ranges = np.concatenate([np.ones(50) * 2.0, np.linspace(2.0, 0.0, 50)])
    points = convert_to_xy(ranges, angles)
    scan = LidarScan(points)

    scan.extract_line_segments(distance_threshold=0.05, min_points=10, max_point_gap=0.2)
    print(f"Extracted {len(scan.get_line_segments())} line segments:")
    for seg in scan.get_line_segments():
        print(f"  {seg['start_index']} to {seg['end_index']}: {seg['start_point']} -> {seg['end_point']}")

    scan.extract_corners(distance_threshold=0.1, angle_threshold_rad=np.pi/6, max_segment_gap=0.3)
    print(f"Extracted {len(scan.get_corners())} corners:")
    for corner in scan.get_corners():
        print(f"  Index {corner['index']}: {corner['position']}")

    scan.visualize_scan(title="L-Shape Lidar Scan with Extracted Features")

    # Test with a rectangular scan
    angles_rect = np.linspace(0, 2 * np.pi, 400, endpoint=False)
    ranges_rect = np.concatenate([
        np.ones(100) * 2,
        np.linspace(2, 3, 100),
        np.ones(100) * 3,
        np.linspace(3, 2, 100)
    ])
    points_rect = convert_to_xy(ranges_rect, angles_rect)
    scan_rect = LidarScan(points_rect)
    scan_rect.extract_line_segments(distance_threshold=0.05, min_points=20, max_point_gap=0.1)
    scan_rect.extract_corners(distance_threshold=0.1, angle_threshold_rad=np.pi/3, max_segment_gap=0.2)
    scan_rect.visualize_scan(title="Rectangular Lidar Scan with Extracted Features")

    # Test with a more complex shape
    angles_complex = np.linspace(0, 2 * np.pi, 600, endpoint=False)
    ranges_complex = 3 + 0.5 * np.sin(5 * angles_complex) + 0.2 * np.cos(12 * angles_complex)
    points_complex = convert_to_xy(ranges_complex, angles_complex)
    scan_complex = LidarScan(points_complex)
    scan_complex.extract_line_segments(distance_threshold=0.03, min_points=15, max_point_gap=0.15)
    scan_complex.extract_corners(distance_threshold=0.08, angle_threshold_rad=np.pi/4, max_segment_gap=0.25)
    scan_complex.visualize_scan(title="Complex Shape Lidar Scan with Extracted Features")