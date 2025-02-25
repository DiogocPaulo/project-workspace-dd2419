import numpy as np
import heapq

class AdaptiveAStar:
    def __init__(self, grid, adaptive_h=None):
        self.grid = grid
        self.rows, self.columns = grid.shape  # (height, width)
        self.adaptive_h = adaptive_h if adaptive_h is not None else {}

    def heuristic(self, node, end_node):
        if node in self.adaptive_h:
            return self.adaptive_h[node]
        # Default to Manhattan distance
        return abs(node[0] - end_node[0]) + abs(node[1] - end_node[1])

    def get_neighbours(self, node):
        (y, x) = node  # Correct order: (row, column)
        neighbours = []
        for dy, dx in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:  # Include diagonals
            next_y, next_x = y + dy, x + dx
            if 0 <= next_y < self.rows and 0 <= next_x < self.columns:
                if self.grid[next_y, next_x] < 100:  # Ensure traversability
                    # Check for diagonal movement
                    if abs(dy) == 1 and abs(dx) == 1:
                        if self.grid[y + dy, x] < 100 and self.grid[y, x + dx] < 100:  # Ensure both adjacent cells are free
                            neighbours.append((next_y, next_x))
                    else:
                        neighbours.append((next_y, next_x))
        return neighbours

    def plan_path(self, start_node, end_node):
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start_node, end_node), 0, start_node))
        came_from = {}
        g_score = {start_node: 0}
        closed_set = set()

        while open_set:
            f, current_g, current = heapq.heappop(open_set)
            if current == end_node:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_node)
                path.reverse()

                # Update adaptive heuristic if the goal was reached
                if end_node in g_score:
                    for node in closed_set:
                        if node in g_score:
                            self.adaptive_h[node] = g_score[end_node] - g_score[node]
                return path

            closed_set.add(current)
            for neighbour in self.get_neighbours(current):
                tentative_g = g_score[current] + 1  # Uniform cost
                if neighbour in g_score and tentative_g >= g_score[neighbour]:
                    continue  # Not a better path
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g
                f_score = tentative_g + self.heuristic(neighbour, end_node)
                heapq.heappush(open_set, (f_score, tentative_g, neighbour))
        return None  # No path found


if __name__ == "__main__":
    from grid_map2 import Map

    # Initialize the grid map
    grid = Map(1, 0, 0, 15, 15)
    grid.initialize_map()
    grid.set_workspace_vertices([
        (0, 0),
        (10, 0),
        (10, 10),
        (0, 10)
    ])
    grid.set_grid_value(0, 0, 100)  # Mark obstacles
    grid.set_grid_value(3, 3, 100)
    grid.set_grid_value(7, 7, 100)

    grid.inflate_map(1)  # Inflate obstacles

    path_planner = AdaptiveAStar(grid.inflated_grid, {})

    # Convert world coordinates to grid coordinates
    grid_x, grid_y = map(int, grid.world_to_grid(7, 0.5))
    grid_x_end, grid_y_end = map(int, grid.world_to_grid(2, 8))

    # Fix indexing when accessing the grid
    print(f"Value at start: {grid.inflated_grid[grid_y, grid_x]}")
    print(f"Value at end: {grid.inflated_grid[grid_y_end, grid_x_end]}")

    path = path_planner.plan_path((grid_y, grid_x), (grid_y_end, grid_x_end))
    print("Path:", path)

    # Visualize the grid
    grid.visualize_grid(False, path)
    grid.visualize_grid(True, path)
