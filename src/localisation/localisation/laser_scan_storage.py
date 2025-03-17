import numpy as np
from scipy.spatial import KDTree

class LaserScanData:
    def __init__(self):
        self.points = []
        self.pose = None
        self.timestamp = None

    def store_scan(self, ranges, angles, pose, timestamp=None):
        """Converts polar coordinates to Cartesian, saves the points, pose, and timestamp."""
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        self.points = np.column_stack((x, y))
        self.pose = pose
        self.timestamp = timestamp

    def store_points(self, points, pose, timestamp=None):
        """Stores the points, pose, and timestamp."""
        self.points = points
        self.pose = pose
        self.timestamp = timestamp

    def get_points(self):
        """Returns the stored points."""
        return self.points

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

    def find_closest_scan(self, pose, max_distance=np.inf):
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
    
    closest_scan = storage.find_closest_scan(np.array([0, 0, 0]))
    print(closest_scan.get_pose())  # Expected output: [0, 0, 0]
    
    closest_scan = storage.find_closest_scan(np.array([2, 2, 0]))
    print(closest_scan.get_pose())  # Expected output: [1, 1, 0]
    
    closest_scan = storage.find_closest_scan(np.array([2, 2, 0]), max_distance=1)
    print(closest_scan)  # Expected output: None