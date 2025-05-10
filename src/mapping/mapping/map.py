import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.ndimage import maximum_filter, rotate, convolve, distance_transform_edt
import scipy.ndimage

from project_interfaces.msg import Object, ObjectList

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
        self.occupancy_decrease = 1

    def initialise_grid(self, workspace_vertices, empty=False):
        # Initialise grid based on a workspace perimeter
        x_list = [vertex[0] for vertex in workspace_vertices]
        y_list = [vertex[1] for vertex in workspace_vertices]

        x_min = math.floor(min(x_list) / self.resolution) - 1.5
        x_max = math.ceil(max(x_list) / self.resolution) + 0.5
        y_min = math.floor(min(y_list) / self.resolution) - 1.5
        y_max = math.ceil(max(y_list) / self.resolution) + 0.5

        self.grid_width = math.ceil(x_max - x_min)
        self.grid_height = math.ceil(y_max - y_min) - 1
        
        self.origin_x = (x_min) * self.resolution
        self.origin_y = (y_min) * self.resolution

        self.grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)
        self.workspace_vertices = [self.world_to_grid(x, y) for x, y in workspace_vertices]
        if not empty:
            self.add_workspace_perimeter()

    def update_grid(self, grid):
        self.grid = grid

    def empty_grid(self):
        if self.grid is None or self.grid_height is None or self.grid_width is None:
            return
        self.grid = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)

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

    def add_object(self, object_type, x, y, angle):
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
        grid_half_width = math.ceil(self.distance_to_cells(width) / 2)
        grid_half_height = math.ceil(self.distance_to_cells(height) / 2)

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

        if angle > 0.0:
            angle = angle * float(math.pi / 180)
            cos_angle = np.cos(angle)
            sin_angle = np.sin(angle)

            rotation_matrix = np.array([[cos_angle, -sin_angle],
                                        [sin_angle,  cos_angle]])
            object_vertices = (rotation_matrix @ object_vertices.T).T

        object_vertices += np.array([grid_x, grid_y])

        min_x = int(np.min(object_vertices[:, 0]))
        max_x = int(np.max(object_vertices[:, 0]))
        min_y = int(np.min(object_vertices[:, 1]))
        max_y = int(np.max(object_vertices[:, 1]))

        self.grid[grid_y, grid_x] = 100

        for j in range(min_y, max_y):
            for i in range(min_x, max_x):
                if self.winding_number(i, j, object_vertices):
                    if self.is_within_grid(i, j):
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

    def update_obstacles(self, valid, start_x, start_y, end_x, end_y):
        start_x, start_y = self.world_to_grid(start_x, start_y)
        end_x, end_y = self.world_to_grid(end_x, end_y)
        cells = self.point_to_line(start_x, start_y, end_x, end_y)
        if not cells:
            return
        for cell in cells[:-1]:
            x, y = cell
            if valid and self.is_within_grid(x, y, world=False):
                self.grid[y, x] = max(self.grid[y, x] - self.occupancy_decrease, 0)
        x, y = cells[-1]
        if valid and self.is_within_grid(x, y, world=False):
            self.grid[y, x] = min(self.grid[y, x] + self.occupancy_increase, 100)

    def add_workspace_perimeter(self):
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                if not self.is_within_workspace(x, y, world=False):
                    self.grid[y, x] = 100

    def winding_number(self, x, y, vertices):
        counter = 0
        for i in range(len(vertices)):
            x_current, y_current = vertices[i]
            x_next, y_next = vertices[(i + 1) % len(vertices)]

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

    def is_within_workspace(self, x, y, world=True):
        if world:
            x, y = self.world_to_grid(x, y)
        return self.winding_number(x, y, self.workspace_vertices)

    def is_within_grid(self, x, y, world=False):
        if world:
            x, y = self.world_to_grid(x, y)
        return (0 <= x < self.grid_width and 0 <= y < self.grid_height)

    def is_free(self, x, y, threshold, world=True):
        # Check if a grid cell is free based on a value threshold
        if self.grid is None:
            return False
        if world:
            x, y = self.world_to_grid(x, y)
        if not self.is_within_grid(x, y):
            return False
        return self.grid[y, x] < threshold

    def get_occupancy(self, x, y):
        # Check if a grid cell is free based on a value threshold
        if self.grid is None:
            return 100
        grid_x, grid_y = self.world_to_grid(x, y)
        if not (0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height):
            return 100
        return self.grid[grid_y, grid_x]

    def are_adjacent_free(self, x, y, radius, threshold, world=True):
        if self.grid is None:
            return False
        if world:
            x, y = self.world_to_grid(x, y)
        if not self.is_within_grid(x, y):
            return False
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy
                if not self.is_within_grid(nx, ny):
                    continue
                if not self.grid[ny, nx] < threshold:
                    return False
        return True

    def get_best_safe_point(self, robot_x, robot_y, x, y, radius, threshold, max_search_radius=20, world=True):
        if self.grid is None:
            return None
        if world:
            x, y = self.world_to_grid(x, y)
            robot_x, robot_y = self.world_to_grid(robot_x, robot_y)
        if not self.is_within_grid(x, y) or not self.is_within_grid(robot_x, robot_y):
            return None
        
        potential_safe_points = []
        for search_radius in range(1, max_search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                # Bottom edge
                nx = x - search_radius
                ny = y + dy
                if self.are_adjacent_free(nx, ny, radius, threshold, world=False):
                    potential_safe_points.append((nx, ny))
                # Top edge
                nx = x + search_radius
                ny = y + dy
                if self.are_adjacent_free(nx, ny, radius, threshold, world=False):
                    potential_safe_points.append((nx, ny))
            for dx in range(-search_radius, search_radius + 1):
                # Left edge
                nx = x + dx
                ny = y - search_radius
                if self.are_adjacent_free(nx, ny, radius, threshold, world=False):
                    potential_safe_points.append((nx, ny))
                # Right edge
                nx = x + dx
                ny = y + search_radius
                if self.are_adjacent_free(nx, ny, radius, threshold, world=False):
                    potential_safe_points.append((nx, ny))

        best_safe_point = None
        min_cost = float("inf")
        for safe_point in potential_safe_points:
            centre_distance = np.hypot(safe_point[0] - x, safe_point[1] - y)
            robot_distance = np.hypot(safe_point[0] - robot_x, safe_point[1] - robot_y)
            cost = (1 * robot_distance) + (2 * centre_distance)
            if cost < min_cost:
                min_cost = cost
                best_safe_point = safe_point
        if best_safe_point is not None:
            sx, sy = self.grid_to_world(best_safe_point[0], best_safe_point[1])
            return (sx, sy)

        return None

    def get_closest_safe_point(self, x, y, radius, threshold, max_search_radius=30, world=True):
        if self.grid is None:
            return None
        if world:
            x, y = self.world_to_grid(x, y)
        if not self.is_within_grid(x, y):
            return None

        closest_safe_point = None
        min_distance = float("inf")
        for search_radius in range(1, max_search_radius + 1):
            potential_safe_points = []
            for dy in range(-search_radius, search_radius + 1):
                # Bottom edge
                nx = x - search_radius
                ny = y + dy
                if self.are_adjacent_free(nx, ny, radius, threshold, world=False):
                    potential_safe_points.append((nx, ny))
                # Top edge
                nx = x + search_radius
                ny = y + dy
                if self.are_adjacent_free(nx, ny, radius, threshold, world=False):
                    potential_safe_points.append((nx, ny))
            for dx in range(-search_radius, search_radius + 1):
                # Left edge
                nx = x + dx
                ny = y - search_radius
                if self.are_adjacent_free(nx, ny, radius, threshold, world=False):
                    potential_safe_points.append((nx, ny))
                # Right edge
                nx = x + dx
                ny = y + search_radius
                if self.are_adjacent_free(nx, ny, radius, threshold, world=False):
                    potential_safe_points.append((nx, ny))

            for safe_point in potential_safe_points:
                distance = np.hypot(safe_point[0] - x, safe_point[1] - y)
                if distance < min_distance:
                    min_distance = distance
                    closest_safe_point = safe_point
            if closest_safe_point is not None:
                sx, sy = self.grid_to_world(closest_safe_point[0], closest_safe_point[1])
                return (sx, sy)

        return None

    def world_to_grid(self, x, y):
        # Convert world coordinates to grid indices
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, x, y):
        # Convert grid indices to world coordinates
        world_x = self.origin_x + (x + 0.5) * self.resolution
        world_y = self.origin_y + (y + 0.5) * self.resolution
        return world_x, world_y

    def round_world(self, x, y):
        grid_x, grid_y = self.world_to_grid(x, y)
        world_x, world_y = self.grid_to_world(grid_x, grid_y)
        return world_x, world_y

    def distance_to_cells(self, distance):
        # Convert world distance to a number of grid cells
        return int(distance / self.resolution)

    def cells_to_distance(self, cells):
        # Converts a number of grid cells to world distance
        return float(cells * self.resolution)

    def inflate_grid(self, inflation_radius, percent=75):
        if self.grid is None:
            return

        occupied_mask = self.grid > 75

        distance_map = distance_transform_edt(~occupied_mask, sampling=self.resolution)

        inflated_grid = np.zeros_like(self.grid, dtype=np.int8)
        inflation_mask = (distance_map <= inflation_radius) & (~occupied_mask)
        inflated_grid[inflation_mask] = int(percent)
        inflated_grid[occupied_mask] = np.maximum(inflated_grid[occupied_mask], self.grid[occupied_mask])

        self.grid = inflated_grid.astype(self.grid.dtype)

