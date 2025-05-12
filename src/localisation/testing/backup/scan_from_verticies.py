import numpy as np
import matplotlib.pyplot as plt

def generate_room_points(boundary_vertices, point_spacing=0.1, noise_std=0.0, outlier_percentage=0.0):
    """
    Generate points along the line segments of a room's boundary with optional noise and outliers.
    
    Args:
        boundary_vertices (list of tuples): List of (x, y) coordinates defining the room's boundary.
        point_spacing (float): Distance between generated points.
        noise_std (float): Standard deviation of Gaussian noise to add to points.
        outlier_percentage (float): Percentage of points to be replaced with outliers.
        
    Returns:
        list of tuples: Generated points along the boundary lines with noise and outliers.
    """
    points = []
    num_vertices = len(boundary_vertices)
    
    for i in range(num_vertices):
        x1, y1 = boundary_vertices[i]
        x2, y2 = boundary_vertices[(i + 1) % num_vertices]  # Wrap around to the first vertex
        
        # Compute segment length and number of points
        segment_length = np.hypot(x2 - x1, y2 - y1)
        num_points = max(int(segment_length / point_spacing), 1)
        
        # Generate points along the segment
        for j in range(num_points + 1):
            t = j / num_points
            x = (1 - t) * x1 + t * x2
            y = (1 - t) * y1 + t * y2
            
            # Add Gaussian noise
            x += np.random.normal(0, noise_std)
            y += np.random.normal(0, noise_std)
            
            points.append((x, y))
    
    # Introduce outliers
    num_outliers = int(len(points) * outlier_percentage)
    for _ in range(num_outliers):
        x_outlier = np.random.uniform(min(x for x, _ in boundary_vertices), max(x for x, _ in boundary_vertices))
        y_outlier = np.random.uniform(min(y for _, y in boundary_vertices), max(y for _, y in boundary_vertices))
        points.append((x_outlier, y_outlier))
    
    return points

def visualize_room(boundary_vertices, points):
    """Visualize the room boundary and generated points."""
    boundary_x, boundary_y = zip(*boundary_vertices + [boundary_vertices[0]])  # Close the loop
    points_x, points_y = zip(*points)
    
    plt.figure(figsize=(8, 8))
    plt.plot(boundary_x, boundary_y, 'k-', label='Room Boundary')
    plt.scatter(points_x, points_y, color='red', s=10, label='Generated Points')
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Room Boundary and Generated Points")
    plt.axis("equal")
    plt.show()

# Example usage
if __name__ == "__main__":
    room_boundary = [(1, 0), (6, 2), (3, 4), (0, 4)]  # Example rectangular room
    generated_points = generate_room_points(room_boundary, point_spacing=0.2, noise_std=0.05, outlier_percentage=0.1)
    visualize_room(room_boundary, generated_points)
