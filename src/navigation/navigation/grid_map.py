import numpy as np
import heapq

import matplotlib.pyplot as plt

class GridMap:
    def __init__(self, width, height, resolution=1.0):
        """
        Initializes a grid map.
        :param width: Number of columns (grid width in cells)
        :param height: Number of rows (grid height in cells)
        :param resolution: Size of each grid cell (default: 1.0)
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.original_grid = np.zeros((height, width), dtype=bool)  # Stores original obstacles
        self.inflated_grid = np.zeros((height, width), dtype=bool)  # Stores inflated obstacles

    def set_occupied(self, x, y):
        """Marks a cell as occupied."""
        if 0 <= x < self.width and 0 <= y < self.height:
            self.original_grid[y, x] = True
            self.inflated_grid[y, x] = True

    def set_free(self, x, y):
        """Marks a cell as free."""
        if 0 <= x < self.width and 0 <= y < self.height:
            self.original_grid[y, x] = False
            self.inflated_grid[y, x] = False

    def batch_update(self, points, occupied=True):
        """Updates multiple cells at once.
        :param points: List of (x, y) tuples representing grid coordinates.
        :param occupied: Whether to mark the cells as occupied (default: True)
        """
        for x, y in points:
            if 0 <= x < self.width and 0 <= y < self.height:
                self.original_grid[y, x] = occupied
                self.inflated_grid[y, x] = occupied

    def is_occupied(self, x, y):
        """Checks if a cell is occupied."""
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.inflated_grid[y, x]
        return False  # Out-of-bounds cells are treated as free

    def world_to_grid(self, wx, wy):
        """Converts world coordinates to grid indices."""
        gx = int(wx / self.resolution)
        gy = int(wy / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Converts grid indices to world coordinates."""
        wx = gx * self.resolution
        wy = gy * self.resolution
        return wx, wy

    def inflate_obstacles(self, radius):
        """Inflates occupied cells by a given radius."""
        self.inflated_grid = np.copy(self.original_grid)
        offset = int(radius / self.resolution)
        for y in range(self.height):
            for x in range(self.width):
                if self.original_grid[y, x]:
                    for dy in range(-offset, offset + 1):
                        for dx in range(-offset, offset + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < self.width and 0 <= ny < self.height:
                                self.inflated_grid[ny, nx] = True

    def print_map(self, grid):
        """Prints the given grid map for visualization."""
        for row in grid:
            print("".join(["#" if cell else "." for cell in row]))

    def heuristic(self, a, b):
        """Heuristic function for A* (Manhattan distance)."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star_search(self, start, goal):
        """
        Performs A* search to find a path from start to goal.
        :param start: Tuple (x, y) representing the start coordinates.
        :param goal: Tuple (x, y) representing the goal coordinates.
        :return: List of tuples representing the path from start to goal, or None if no path exists.
        """
        start = self.world_to_grid(*start)
        goal = self.world_to_grid(*goal)

        if self.is_occupied(*start) or self.is_occupied(*goal):
            return None  # Start or goal is occupied

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    if self.is_occupied(*neighbor):
                        continue

                    tentative_g_score = g_score[current] + 1

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found

    def reconstruct_path(self, came_from, current):
        """Reconstructs the path from the came_from dictionary."""
        total_path = [self.grid_to_world(*current)]
        while current in came_from:
            current = came_from[current]
            total_path.append(self.grid_to_world(*current))
        return total_path[::-1]

    def visualize_map_and_path(self, path=None):
        """Visualizes the map and path using Matplotlib."""
        plt.figure(figsize=(self.width, self.height))

        # Plot the grid map
        for y in range(self.height):
            for x in range(self.width):
                if self.inflated_grid[y, x]:
                    plt.plot(x, y, 'ks', markersize=10)  # Black square for obstacles

        # Plot the path if provided
        if path:
            path_grid = [self.world_to_grid(*p) for p in path]
            path_x, path_y = zip(*path_grid)
            plt.plot(path_x, path_y, 'r-', linewidth=2, label='Path')  # Red line for path
            plt.plot(path_x, path_y, 'ro', markersize=5)  # Red circles for path points

        plt.xlim(-1, self.width)
        plt.ylim(-1, self.height)
        plt.gca().invert_yaxis()
        plt.title("Grid Map with Path")
        plt.xlabel("X coordinate")
        plt.ylabel("Y coordinate")
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    # Example usage
    grid_map = GridMap(10, 5)
    grid_map.batch_update([(3, 2), (6, 4)])
    print("Original Map:")
    grid_map.print_map(grid_map.original_grid)

    grid_map.inflate_obstacles(1)
    print("\nInflated Map:")
    grid_map.print_map(grid_map.inflated_grid)

    start = (0, 0)
    goal = (9, 4)
    path = grid_map.a_star_search(start, goal)
    print("\nPath found:", path)

    # Visualize the map and path
    grid_map.visualize_map_and_path(path)
