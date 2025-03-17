import math
import numpy as np
import matplotlib.pyplot as plt

from laser_scan_storage import LaserScanStorage, LaserScanData
from icp import icp

def create_test_data():
    # set seed for reproducible results
    np.random.seed(12345)

    # create a set of points to be the reference for ICP
    xs = np.random.random_sample((50, 1))
    ys = np.random.random_sample((50, 1))
    reference_points = np.hstack((xs, ys))

    # create a set of points to be aligned to the reference points
    points_to_be_aligned = reference_points[1:47]

    # apply rotation to the new point set
    theta = math.radians(12)
    c, s = math.cos(theta), math.sin(theta)
    rot = np.array([[c, -s],
                    [s, c]])
    points_to_be_aligned = np.dot(points_to_be_aligned, rot)

    # apply translation to the new point set
    points_to_be_aligned += np.array([np.random.random_sample(), np.random.random_sample()])

    return reference_points, points_to_be_aligned

if __name__ == '__main__':

    # Initialize LaserScanStorage
    storage = LaserScanStorage()

    # Create 2 LaserScan objects
    scan1 = LaserScanData()
    scan2 = LaserScanData()

    # Create test data
    reference_points, points_to_be_aligned = create_test_data()

    # Store the points in the LaserScan objects
    scan1.store_points(reference_points, np.array([0, 0, 0]), 0)
    scan2.store_points(points_to_be_aligned, np.array([0, 0, 0]), 0)

    # Add the LaserScan objects to the storage
    storage.add_scan(scan1)
    storage.add_scan(scan2)

    # Run ICP
    rotation, translation, aligned_points = icp(scan1.points, scan2.points)

    # Print the results
    print(f"Rotation: {rotation}")
    print(f"Translation: {translation}")

    # Plot the results
    plt.plot(reference_points[:, 0], reference_points[:, 1], 'rx', label='reference points')
    plt.plot(points_to_be_aligned[:, 0], points_to_be_aligned[:, 1], 'b1', label='points to be aligned')
    plt.plot(aligned_points[:, 0], aligned_points[:, 1], 'g+', label='aligned points')
    plt.legend()
    plt.show()