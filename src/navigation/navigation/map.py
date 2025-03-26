import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.ndimage import maximum_filter, rotate

from project_interfaces.msg import Object, ObjectList

class WorkspaceArea:
    def __init__(self, workspace_vertices):
        self.workspace_vertices = workspace_vertices
    def is_within_workspace(self, x, y):
        return self.winding_number(x, y)
    def winding_number(self, x, y):
        counter = 0
        for i in range(len(self.workspace_vertices)):
            x_current, y_current = self.workspace_vertices[i]
            x_next, y_next = self.workspace_vertices[(i + 1) % len(self.workspace_vertices)]

            if y_current <= y:
                if y_next > y:
                    if self.is_left(x, y, x_current, y_current, x_next, y_next) > 0: 
                        counter += 1
            else:
                if y_next <= y:
                    if self.is_left(x, y, x_current, y_current, x_next, y_next) < 0:
                        counter -= 1
        return counter != 0
    def is_left(self, x, y, x_current, y_current, x_next, y_next):
        return ((x_next - x_current) * (y - y_current) - (y_next - y_current) * (x - x_current))

class Map:
    """
    Class provides by default the utility functions such as conversions, but also store the map.
    """
    def __init__(self, resolution, origin_x=None, origin_y=None, grid_width=None, grid_height=None, grid=None):
        # Map properties
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.grid = grid
        self.workspace_vertices = None
        self.occupancy_increase = 25
        self.occupancy_decrease = 5

    def initialise_grid(self):
        # Initialise grid based map properties
        if self.grid_width is None or self.grid_height is None:
            raise ValueError("Grid width and height not initialised")
        if self.origin_x is None or self.origin_y is None:
            raise ValueError("Grid origin not initialised")

        self.grid = np.full((self.grid_height, self.grid_width), 0, dtype=np.int8)

    def initalise_grid_with_workspace(self, workspace_vertices):
        # Initialise grid based on a workspace perimeter
        x_list = [vertex[0] for vertex in workspace_vertices]
        y_list = [vertex[1] for vertex in workspace_vertices]

        x_min = math.floor(min(x_list) / self.resolution) - 1
        x_max = math.ceil(max(x_list) / self.resolution) + 1
        y_min = math.floor(min(y_list) / self.resolution) - 1
        y_max = math.ceil(max(y_list) / self.resolution) + 1

        self.grid_width = x_max - x_min + 1
        self.grid_height = y_max - y_min
        
        self.origin_x = (x_min) * self.resolution
        self.origin_y = (y_min) * self.resolution

        self.grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
        self.set_workspace_vertices(workspace_vertices)

    def update_grid(self, grid):
        self.grid = grid

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

    def add_object(self, x, y, angle, object_type):
        if self.grid is None:
            # Grid is not yet initalised
            return

        if object_type == Object.CUBE:
            width = 0.05
            height = 0.05
        elif object_type == Object.SPHERE:
            width = 0.05
            height = 0.05
        elif object_type == Object.PLUSHIE:
            width = 0.10
            height = 0.10
        elif object_type == Object.BOX:
            width = 0.20
            height = 0.25

        grid_x, grid_y = self.world_to_grid(x, y)
        grid_half_width = self.distance_to_cells(width) / 2
        grid_half_height = self.distance_to_cells(height) / 2

        if not (0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height):
            # Grid coordinates out of bounds
            return

        if (self.grid[grid_y, grid_x] == 100):
            # Object already in map
            return


        object_vertices = np.array([
            [grid_half_width, grid_half_height],
            [grid_half_width, -grid_half_height],
            [-grid_half_width, -grid_half_height],
            [-grid_half_width, grid_half_height]
        ])

        # if angle is not None:
        #     cos_angle = np.cos(angle)
        #     sin_angle = np.sin(angle)
        #
        #     rotation_matrix = np.array([[cos_angle, -sin_angle],
        #                                 [sin_angle,  cos_angle]])
        #     object_vertices = (rotation_matrix @ object_vertices.T).T

        object_vertices += np.array([grid_x, grid_y])

        min_x = int(np.min(object_vertices[:, 0]))
        max_x = int(np.max(object_vertices[:, 0]))
        min_y = int(np.min(object_vertices[:, 1]))
        max_y = int(np.max(object_vertices[:, 1]))

        self.grid[grid_y, grid_x] = 100

        for j in range(min_y, max_y):
            for i in range(min_x, max_x):
                if self.winding_number(i, j, object_vertices):
                    self.grid[j, i] = 100

    def add_obstacle_point(self, x, y):
        grid_x, grid_y = self.world_to_grid(x, y)
        if not (0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height):
            # Grid coordinates out of bounds
            return
        self.grid[grid_y, grid_x] += self.occupancy_increase

    def point_to_line(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        n = dx + dy
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1
        error = dx - dy
        cells = []
        
        for _ in range(n + 1):
            cells.append((x, y))
            if error > 0:
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
        return cells

    def update_obstacles_in_line(self, start_x, start_y, end_x, end_y, valid):
        start_x, start_y = self.world_to_grid(start_x, start_y)
        end_x, end_y = self.world_to_grid(end_x, end_y)
        cells = self.point_to_line(start_x, start_y, end_x, end_y)
        if valid:
        # For valid readings increase occupancy of the last cell decrease the rest
            for cell in cells:
                x, y = cell
                if not (0 < x < self.grid_width-1 and 0 < y < self.grid_height-1):
                    continue
                if cell == cells[-1]:
                    self.grid[y, x] = min(self.grid[y, x] + self.occupancy_increase, 100)
                else:
                    self.grid[y, x] = max(self.grid[y, x] - self.occupancy_decrease, 0)
        else:
        # For invalid readings decrease occupancy of all cells
            for cell in cells:
                x, y = cell
                if not (0 < x < self.grid_width-1 and 0 < y < self.grid_height-1):
                    continue
                self.grid[y, x] = max(self.grid[y, x] - self.occupancy_decrease, 0)


    def set_workspace_vertices(self, vertices):
        """Sets the workspace boundary as a list of (x, y) vertices and marks grid cells outside the workspace."""
        self.workspace_vertices = vertices

        # Mark grid cells outside the workspace
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
        # Check if a grid cell is free based on a value threshold
        if self.grid is None:
            raise ValueError("Grid is not initalised")
        grid_x, grid_y = self.world_to_grid(x, y)
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
            return self.grid[grid_y, grid_x] < value_threshold
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

    def distance_to_cells(self, distance):
        # Convert world distance to a number of grid cells
        return int(distance / self.resolution)

    def cells_to_distance(self, cells):
        # Converts a number of grid cells to world distance
        return float(cells * self.resolution)

    def inflate_grid(self, inflation_radius):
        # Returns a grid that has inflated occupied cells by an inflation radius
        if self.grid is None:
            raise ValueError("Grid is not initialised")

        inflation_cells = self.distance_to_cells(inflation_radius)

        # Define a circular footprint using inflation radius
        circular_footprint = np.zeros(
            (2 * inflation_cells + 1, 2 * inflation_cells + 1),
            dtype=int,
        )
        ty, tx = np.ogrid[
            -inflation_cells : inflation_cells + 1,
            -inflation_cells : inflation_cells + 1,
        ]
        mask = tx**2 + ty**2 <= inflation_cells**2
        circular_footprint[mask] = 1

        inflated_grid = maximum_filter(self.grid, footprint=circular_footprint, mode="constant", cval=0)
        self.grid = inflated_grid

    def inflate_grid_in_region(self, inflation_radius, region_radius, robot_x, robot_y):
        # Returns a grid that has inflated occupied cells by an inflation radius within a region radius around to robot
        if self.grid is None:
            raise ValueError("Grid is not initialised")


        inflation_cells = self.distance_to_cells(inflation_radius)
        region_cells = self.distance_to_cells(region_radius)
        grid_x, grid_y = self.world_to_grid(robot_x, robot_y)
        if not (0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height):
            # Grid coordinates out of bounds
            return

        # Define region within grid to inflate
        x_min = max(0, grid_x - region_cells)
        x_max = min(self.grid_width, grid_x + region_cells + 1)
        y_min = max(0, grid_y - region_cells)
        y_max = min(self.grid_height, grid_y + region_cells + 1)

        inflated_grid = self.grid.copy()
        region = self.grid[y_min:y_max, x_min:x_max].copy()

        # Define a circular footprint using inflation radius
        circular_footprint = np.zeros(
            (2 * inflation_cells + 1, 2 * inflation_cells + 1),
            dtype=int,
        )
        ty, tx = np.ogrid[
            -inflation_cells : inflation_cells + 1,
            -inflation_cells : inflation_cells + 1,
        ]
        mask = tx**2 + ty**2 <= inflation_cells**2
        circular_footprint[mask] = 1

        inflated_region = maximum_filter(region, footprint=circular_footprint, mode="constant", cval=-1)
        inflated_grid[y_min:y_max, x_min:x_max] = inflated_region
        self.grid = inflated_grid
