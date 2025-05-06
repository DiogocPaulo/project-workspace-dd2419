import math
import matplotlib.pyplot as plt

from laser_scan_utils import polar_to_cartesian

class LaserScan:
    def __init__(self, ranges, angles=None, timestamp=None):
        self.ranges = ranges
        if angles is None:
            num_readings = len(ranges)
            if num_readings > 0:
                self.angles = [2 * math.pi * i / num_readings for i in range(num_readings)]
            else:
                self.angles = []
        else:
            if len(ranges) != len(angles):
                raise ValueError("Number of ranges and angles must be the same.")
            self.angles = angles
        self.timestamp = timestamp

    def get_scan_in_cartesian(self):
        cartesian_points = []
        for i, r in enumerate(self.ranges):
            if 0 < r < float('inf'):
                x, y = polar_to_cartesian(r, self.angles[i])
                cartesian_points.append((x, y))
        return cartesian_points

class Keypoint:
    def __init__(self, index, position):
        self.index = index
        self.position = position

class Descriptor:
    def __init__(self, data, keypoint_index):
        self.data = data
        self.keypoint_index = keypoint_index

def visualize_scan(laser_scan: LaserScan, title="Laser Scan"):
    cartesian_points = laser_scan.get_scan_in_cartesian()
    if not cartesian_points:
        print("No valid points to visualize in the scan.")
        return

    x = [p[0] for p in cartesian_points]
    y = [p[1] for p in cartesian_points]

    plt.figure()
    plt.scatter(x, y, s=5)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title(title)
    plt.grid(True)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

if __name__ == '__main__':
    example_ranges = [1.0, 1.1, 1.2, 1.0, 0.9, 0.8, 1.5, 1.6, 2.0, 2.1, 2.2, 2.0, 1.9, 1.8]
    example_angles = [0.0 + 0.1 * i for i in range(len(example_ranges))]
    scan1 = LaserScan(example_ranges, example_angles, timestamp=1678886400.0)
    visualize_scan(scan1, title="Example Laser Scan 1")

    example_ranges_uniform = [2.0] * 360
    scan2 = LaserScan(example_ranges_uniform)
    visualize_scan(scan2, title="Example Laser Scan 2 (Uniform Angles)")

    example_ranges_invalid = [1.0, float('inf'), 1.2, 0.0, 0.9]
    example_angles_invalid = [0.0, 0.5, 1.0, 1.5, 2.0]
    scan3 = LaserScan(example_ranges_invalid, example_angles_invalid)
    visualize_scan(scan3, title="Example Laser Scan 3 (With Invalid Readings)")