import numpy as np
import math
import matplotlib.pyplot as plt

def generate_laser_scan(pose: np.ndarray, num_points: int = 50, room_vertices: np.ndarray = None, num_objects: int = 5) -> np.ndarray:
    """Generate a laser scan from a given pose, following room vertices and placing random objects.

    Args:
        pose: Robot pose as [x, y, theta] in world frame
        num_points: Number of points to generate
        room_vertices: Array of room boundary vertices (shape: [n, 2]), default is a 5x5 square
        num_objects: Number of random objects to place in the room

    Returns:
        Array of points in world coordinates (shape: [num_points, 2])
    """
    np.random.seed(12345)  # For reproducibility

    # Default room: 5x5 square if no vertices provided
    if room_vertices is None:
        room_vertices = np.array([[0, 0], [5, 0], [5, 5], [0, 5], [0, 0]])

    # Generate angles for the laser scan (full 360-degree sweep)
    angles = np.linspace(0, 2 * math.pi, num_points)

    # Initialize output points
    world_points = np.zeros((num_points, 2))
    x, y, theta = pose

    # Rotation matrix for robot orientation
    rot = np.array([[math.cos(theta), -math.sin(theta)],
                    [math.sin(theta), math.cos(theta)]])

    # Generate random objects (position and radius) inside the room
    obj_centers = np.random.uniform(1, 4, (num_objects, 2))  # Keep within 5x5 room, avoiding edges
    obj_radii = np.random.uniform(0.2, 0.5, num_objects)

    for i, angle in enumerate(angles):
        # Direction vector in local frame
        dir_vec = np.array([math.cos(angle), math.sin(angle)])

        # Default max range if no intersection
        max_range = 10.0
        closest_dist = max_range

        # Check intersection with room vertices (walls)
        for j in range(len(room_vertices) - 1):
            v1, v2 = room_vertices[j], room_vertices[j + 1]
            wall_vec = v2 - v1
            ray_start = pose[:2]
            ray_vec = dir_vec

            # Line intersection test (parametric form)
            denom = np.cross(ray_vec, wall_vec)
            if abs(denom) < 1e-6:  # Parallel lines
                continue

            t = np.cross(v1 - ray_start, wall_vec) / denom
            u = np.cross(v1 - ray_start, ray_vec) / denom

            if 0 <= t <= max_range and 0 <= u <= 1:
                dist = t
                if dist < closest_dist:
                    closest_dist = dist

        # Check intersection with objects (circles)
        for center, radius in zip(obj_centers, obj_radii):
            oc = center - ray_start
            proj = np.dot(oc, ray_vec)
            if proj < 0:  # Object behind robot
                continue

            closest_point_dist = np.linalg.norm(oc - proj * ray_vec)
            if closest_point_dist <= radius:  # Ray hits the object
                dist_to_center = np.linalg.norm(oc)
                dist_to_edge = dist_to_center - radius
                if dist_to_edge < closest_dist:
                    closest_dist = dist_to_edge

        # Convert closest intersection to a point
        local_point = dir_vec * closest_dist
        world_points[i] = np.dot(local_point, rot) + pose[:2]

    return world_points, room_vertices, obj_centers, obj_radii  # Return extra data for plotting

def plot_laser_scan(pose: np.ndarray, scan_data: tuple):
    """Plot the laser scan, room, objects, and robot pose.

    Args:
        pose: Robot pose as [x, y, theta]
        scan_data: Tuple of (world_points, room_vertices, obj_centers, obj_radii) from generate_laser_scan
    """
    world_points, room_vertices, obj_centers, obj_radii = scan_data

    plt.figure(figsize=(8, 8))

    # Plot room boundaries
    plt.plot(room_vertices[:, 0], room_vertices[:, 1], 'b-', label='Room Walls')

    # Plot random objects (as circles)
    for center, radius in zip(obj_centers, obj_radii):
        circle = plt.Circle(center, radius, color='g', alpha=0.5, label='Object' if center is obj_centers[0] else "")
        plt.gca().add_patch(circle)

    # Plot laser scan points
    plt.scatter(world_points[:, 0], world_points[:, 1], c='r', s=10, label='Laser Scan Points')

    # Plot robot pose (position and orientation)
    plt.plot(pose[0], pose[1], 'ko', label='Robot Position')
    arrow_length = 0.5
    dx = arrow_length * math.cos(pose[2])
    dy = arrow_length * math.sin(pose[2])
    plt.arrow(pose[0], pose[1], dx, dy, head_width=0.1, head_length=0.2, fc='k', ec='k', label='Orientation')

    # Set plot properties
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.xlabel('X (world coordinates)')
    plt.ylabel('Y (world coordinates)')
    plt.title('Laser Scan Simulation')
    plt.show()

# Example usage
if __name__ == "__main__":
    pose = np.array([2, 2, math.pi / 4])  # Robot at center, 45-degree angle
    scan_data = generate_laser_scan(pose, num_points=50, num_objects=3)
    plot_laser_scan(pose, scan_data)