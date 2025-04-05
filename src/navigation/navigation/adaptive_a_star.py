import numpy as np
import heapq
import random

class AdaptiveAStar:
    def __init__(self, grid, adaptive_h=None):
        self.grid = grid
        self.rows, self.columns = grid.shape  # (height, width)
        self.adaptive_h = {} if adaptive_h is None else adaptive_h

    def update_grid(self, grid):
        self.grid = grid

    def heuristic(self, node, end_node):
        if node in self.adaptive_h:
            return self.adaptive_h[node]
        # Default to Manhattan distance
        return abs(node[0] - end_node[0]) + abs(node[1] - end_node[1])

    def get_neighbours(self, node, occupancy):
        (y, x) = node  # Correct order: (row, column)
        neighbours = []
        costs = []
        directions = [(0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1),     # Right, Left, Down, Up
                      (1, 1, 2), (1, -1, 2), (-1, 1, 2), (-1, -1, 2)]   # Diagonals

        for dy, dx, cost in directions:
            next_y, next_x = y + dy, x + dx
            if 0 <= next_y < self.rows and 0 <= next_x < self.columns:
                if self.grid[next_y, next_x] < occupancy:
                    # For diagonal movements, checks that both adjacent cells are free
                    if abs(dy) == 1 and abs(dx) == 1:
                        if self.grid[y + dy, x] < occupancy and self.grid[y, x + dx] < occupancy:
                            neighbours.append((next_y, next_x))
                            costs.append(cost)
                    else:
                        neighbours.append((next_y, next_x))
                        costs.append(cost)
        return neighbours, costs

    def plan_path(self, start_node, end_node, occupancy):
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start_node, end_node), 0, start_node))
        came_from = {}
        g_score = {start_node: 0}
        closed_set = set()

        while open_set:
            f, current_g, current = heapq.heappop(open_set)
            if current == end_node:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_node)
                path.reverse()

                if end_node in g_score:
                    for node in closed_set:
                        if node in g_score:
                            self.adaptive_h[node] = g_score[end_node] - g_score[node]
                return path

            closed_set.add(current)
            neighbours, costs = self.get_neighbours(current, occupancy)
            for neighbour, cost in zip(neighbours, costs):
                tentative_g = g_score[current] + cost
                if neighbour in g_score and tentative_g >= g_score[neighbour]:
                    continue
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g
                f_score = tentative_g + self.heuristic(neighbour, end_node)
                heapq.heappush(open_set, (f_score, tentative_g, neighbour))
        return None
