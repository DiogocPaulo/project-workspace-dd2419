import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import maximum_filter, rotate

class Map:
    """
    Class provides by default the utility functions such as conversions, but also store the map.
    """
    def __init__(self, resolution, origin_x, origin_y, height=None, width=None):
        # Initialize map properties
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.height = height
        self.width = width
        self.grid = None
        self.inflated_grid = None
        self.workspace_vertices = None

    def initialize_map(self):
        # Compute grid dimensions and create grid maps
        if self.width != None and self.height != None:
            self.grid_height = int(self.height / self.resolution)
            self.grid_width = int(self.width / self.resolution)
            self.grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
            self.inflated_grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
        else:
            raise ValueError("Map was not intialized width height and width.")

    def set_grid_value(self, x, y, value):
        # Set a grid cell to a value (0 to 100)
        if self.grid is None:
            raise ValueError("Grid is not initialized. Call initialize_map() first.")
        if not (0 <= value <= 100):
            raise ValueError("Value must be between 0 and 100.")
        if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
            self.grid[y, x] = value  # Grid indices are (row, col)
        else:
            raise IndexError("Grid coordinates out of bounds.")

    def add_object(self, x, y, width, height, angle=None):
        if self.grid is None:
            self.get_logger().debug("Grid not defined")
            return

        grid_x, grid_y = self.world_to_grid(x, y);
        grid_half_width = self.distance_to_units(width) / 2;
        grid_half_height = self.distance_to_units(height) / 2;

        if (self.grid[grid_y, grid_x] == 100):
            self.get_logger().info("Object alreay in map")
            return

        object_vertices = np.array([
            [grid_half_width, grid_half_height],
            [grid_half_width, -grid_half_height],
            [-grid_half_width, -grid_half_height],
            [-grid_half_width, grid_half_height]
        ]);

        if angle is not None:
            angle_cos = np.cos(angle)
            angle_sin = np.sin(angle)

            rotation_matrix = np.array([[cos_angle, -sin_angle],
                                        [sin_angle,  cos_angle]])
            object_vertices = (rotation_matrix @ object_vertices.T).T

        object_vertices += np.array([grid_x, grid_y])

        min_x = np.min(vertices[:, 0])
        max_x = np.max(vertices[:, 0])
        min_y = np.min(vertices[:, 1])
        max_y = np.max(vertices[:, 1])

        for j in range(min_y, max_y):
            for i in range(min_x, max_x):
                if self.winding_number(i, j, object_vertices):
                    self.grid[j, i] = 100;


    def set_workspace_vertices(self, vertices):
        """Sets the workspace boundary as a list of (x, y) vertices and marks grid units outside the workspace."""
        self.workspace_vertices = vertices

        # Mark grid units outside the workspace
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                world_x, world_y = self.grid_to_world(x, y)
                if not self.is_within_workspace(world_x, world_y):
                    self.grid[y, x] = 100

    def is_on_line(self, x, y, x1, y1, x2, y2):
        """Checks if (x, y) lies exactly on the line segment (x1, y1) -> (x2, y2)."""
        if min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2):
            cross_product = (y - y1) * (x2 - x1) - (x - x1) * (y2 - y1)
            if abs(cross_product) < 1e-6:  # Close to zero → on the line
                return True
        return False

    def winding_number(self, x, y, vertices):
        """Calculate the winding number for a point (x, y) to determine if it is inside the polygon."""
        wn = 0  # Winding number counter
        n = len(vertices)

        for i in range(n):
            x1, y1 = vertices[i]
            x2, y2 = vertices[(i + 1) % n]

            # Check if point is on the boundary (on the line segment)
            if self.is_on_line(x, y, x1, y1, x2, y2):
                return True  # Point is on the boundary, considered inside

            # Check if the point is between the y-bounds of the edge
            if y1 <= y < y2 or y2 <= y < y1:
                # Calculate the x-coordinate of the intersection of the edge with a horizontal line from the point
                x_intersection = x1 + (y - y1) * (x2 - x1) / (y2 - y1)

                if x < x_intersection:
                    wn += 1 if y1 < y2 else -1  # Increment or decrement based on the edge direction

        return wn != 0  # If the winding number is non-zero, the point is inside

    def is_within_workspace(self, x, y):
        """Checks if a point (x, y) is inside the workspace or on its boundary."""
        return self.winding_number(x, y, self.workspace_vertices)

    def is_free(self, x, y, value_threshold):
        # Check if a grid cell is free based on a value threshold using the inflated grid
        if self.inflated_grid is None:
            raise ValueError("Inflated grid is not initialized. Call inflate_map() first.")
        grid_x, grid_y = self.world_to_grid(x, y)
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            return self.inflated_grid[grid_y, grid_x] < value_threshold
        return False

    def world_to_grid(self, x, y):
        # Convert world coordinates to grid indices
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, x, y):
        # Convert grid indices to world coordinates
        world_x = self.origin_x + x * self.resolution
        world_y = self.origin_y + y * self.resolution
        return world_x, world_y

    def distance_to_units(self, distance):
        # Convert world distance to grid units
        return int(distance / self.resolution)

    def units_to_distance(self, units):
        return float(units * self.resolution)

    def inflate_map(self, inflation_radius):
        # Inflate obstacles in the grid
        if self.grid is None:
            raise ValueError("Grid is not initialized. Call initialize_map() first.")

        inflation_cells = int(inflation_radius / self.resolution)  # Convert meters to grid cells
        self.inflated_grid = maximum_filter(self.grid, size=2 * inflation_cells + 1, mode='constant', cval=-1)

    def visualize_grid(self, use_inflated=False, path=None):
        """
        Visualizes the grid or inflated grid with clearly defined squares and optionally overlays a path.

        Args:
            use_inflated (bool): If True, displays the inflated grid; otherwise, displays the original grid.
            path (list of tuples): A list of (x, y) coordinates representing the path.
        """
        if self.grid is None:
            raise ValueError("Grid is not initialized. Call initialize_map() first.")
        
        grid_to_plot = self.inflated_grid if use_inflated else self.grid
        cmap = plt.get_cmap("gray_r")  # Inverted grayscale for better visibility

        plt.figure(figsize=(8, 8))
        plt.pcolor(grid_to_plot, cmap=cmap, edgecolors='k', linewidths=0.5)  # [::-1] flips the y-axis (following numpy indexing)   
        plt.colorbar(label="Grid Value")

        title = "Inflated Grid" if use_inflated else "Original Grid"
        plt.title(title)
        plt.xlabel("Grid X")
        plt.ylabel("Grid Y")
        plt.xticks(np.arange(grid_to_plot.shape[1] + 1) - 0.5)
        plt.yticks(np.arange(grid_to_plot.shape[0] + 1) - 0.5)
        plt.grid(True, which="both", color="black", linestyle="-", linewidth=0.5)

        # Overlay the path if provided
        if path:
            path_grid = [self.grid_to_world(x, y) for x, y in path]  # Convert path to grid coordinates
            path_x, path_y = zip(*path_grid)  # Extract x and y coordinates
            print(f"Grid to world (0, 7): {self.world_to_grid(0, 7)}")
            plt.plot(path_x, path_y, marker='o', color='red', linestyle='-', linewidth=2, markersize=5, label="Path")

        plt.legend()
        plt.show()


# Example usage
if __name__ == "__main__":
    resolution = 1
    origin_x, origin_y = 0, 0

    grid = Map(resolution, origin_x, origin_y, 10, 10)
    grid.initialize_map()
    grid.set_workspace_vertices([
        (0, 0),
        (7, 0),
        (7, 7),
        (0, 7)
    ])

    test_point = (5.001, 1)  
    print(f"Point {test_point} inside workspace? {grid.is_within_workspace(*test_point)}")

    # Visualize map
    grid.set_grid_value(2, 2, 100)
    grid.set_grid_value(1, 1, 100)
    grid.inflate_map(1)

    print(f"Grid Map:\n{grid.grid}")
    print(f"Inflated Map:\n{grid.inflated_grid}")
    grid.visualize_grid(False)
    grid.visualize_grid(True)
    
