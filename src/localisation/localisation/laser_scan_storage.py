import numpy as np
from scipy.spatial import KDTree

class LaserScanData:
    def __init__(self):
        """Initialize with empty points, pose, and timestamp."""
        self.points = np.array([])  # Use NumPy array for consistency
        self.pose = None
        self.timestamp = None

    def store_scan(self, ranges, angles, pose, timestamp=None, max_range=10.0):
        """Converts polar coordinates to Cartesian, transforms to odom frame, and saves valid points, pose, and timestamp."""
        # Convert to NumPy arrays if not already
        ranges = np.array(ranges)
        angles = np.array(angles)

        # Filter out invalid ranges (inf, nan, or beyond max_range)
        valid_mask = np.isfinite(ranges) & (ranges >= 0) & (ranges <= max_range)
        if not np.any(valid_mask):
            self.points = np.array([])  # Empty array if no valid points
            self.pose = pose
            self.timestamp = timestamp
            return False  # Indicate that no points were added

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # Convert to Cartesian coordinates in the laser frame
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points_laser = np.column_stack((x, y))

        # Transform points to the odom frame using the pose
        theta = pose[2]  # Orientation (yaw) in radians
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                         [np.sin(theta), np.cos(theta)]])
        translation = pose[:2]  # Translation (x, y)
        points_odom = points_laser @ rotation_matrix.T + translation

        self.points = points_odom
        self.pose = pose
        self.timestamp = timestamp
        return True  # Indicate that points were successfully added

    def store_points(self, points, pose, timestamp=None):
        """Stores precomputed points, pose, and timestamp after validation."""
        # Validate points input
        points = np.array(points, dtype=np.float64)  # Ensure NumPy array and float64
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError(f"Points must be a 2D array with shape (N, 2), got {points.shape}")
        
        # Filter out inf/nan
        valid_mask = np.isfinite(points).all(axis=1)
        if not np.any(valid_mask):
            self.points = np.array([])  # Empty if all invalid
            self.pose = pose
            self.timestamp = timestamp
            return False  # Indicate no points were added
        else:
            self.points = points[valid_mask]
        
        self.pose = pose
        self.timestamp = timestamp
        return True  # Indicate points were successfully added

    def get_points(self):
        """Returns the stored points, guaranteed to be finite."""
        # Double-check for safety (though store methods should ensure this)
        if len(self.points) > 0:
            return self.points[np.isfinite(self.points).all(axis=1)]
        return self.points  # Empty array if no valid points

    def get_pose(self):
        """Returns the stored pose."""
        return self.pose

    def get_timestamp(self):
        """Returns the stored timestamp."""
        return self.timestamp
    
class LaserScanStorage:
    def __init__(self):
        self.scans = []
        self.kd_tree = None
        self.poses = []

    def add_scan(self, scan):
        """Adds a LaserScan object to the storage and updates the KDTree."""
        self.scans.append(scan)
        self.poses.append(scan.get_pose())
        self.kd_tree = KDTree(self.poses)

    def get_closest_scan(self, pose, max_distance=np.inf):
        """Finds the closest LaserScan object to the given pose within the max_distance."""
        if not self.kd_tree:
            return None

        distance, index = self.kd_tree.query(pose)
        if distance <= max_distance:
            return self.scans[index]
        return None

    def get_closest_distance(self, pose):
        """Returns the distance to the closest scan."""
        if not self.kd_tree:
            return None

        distance, _ = self.kd_tree.query(pose)
        return distance
    
if __name__ == "__main__":
    scan1 = LaserScanData()
    scan1.store_scan(np.array([1, 2, 3]), np.array([0, np.pi/2, np.pi]), np.array([0, 0, 0]))
    scan2 = LaserScanData()
    scan2.store_scan(np.array([1, 2, 3]), np.array([0, np.pi/2, np.pi]), np.array([1, 1, 0]))
    
    storage = LaserScanStorage()
    storage.add_scan(scan1)
    storage.add_scan(scan2)
    
    closest_scan = storage.get_closest_scan(np.array([0, 0, 0]))
    print(closest_scan.get_pose())  # Expected output: [0, 0, 0]
    
    closest_scan = storage.get_closest_scan(np.array([2, 2, 0]))
    print(closest_scan.get_pose())  # Expected output: [1, 1, 0]
    
    closest_scan = storage.get_closest_scan(np.array([3, 2, 0]), max_distance=2)
    print(closest_scan.get_pose() if closest_scan else None)  # Expected output: None