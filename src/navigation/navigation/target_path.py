import numpy as np
from navigation.robot_state import RobotState
from project_interfaces.msg import NavPath

class TargetPath:
    def __init__(self, lookahead_gain, lookahead_min):
        self.x_points = []
        self.y_points = []
        self.old_nearest_point_index = None
        self.lookahead_gain = lookahead_gain
        self.lookahead_min = lookahead_min

    def update_path(self, path_msg: NavPath):
        self.x_points = [point.x for point in path_msg.path]
        self.y_points = [point.y for point in path_msg.path]
        self.old_nearest_point_index = None  # Reset when updating path

    def search_target_index(self, state: RobotState, reverse=False):
        if not self.x_points or not self.y_points:
            return None, None

        if self.old_nearest_point_index is None:
            dx = [state.x - i for i in self.x_points]
            dy = [state.y - i for i in self.y_points]
            distances = np.hypot(dx, dy)
            index = np.argmin(distances)
        else:
            index = self.old_nearest_point_index
            distance_to_index = state.distance_to_state(
                self.x_points[index], self.y_points[index]
            )

            # Adjust index based on direction
            if reverse:
                while True:
                    if (index - 1) < 0:  # Prevent going below 0
                        break
                    distance_to_next_index = state.distance_to_state(
                        self.x_points[index - 1], self.y_points[index - 1]
                    )
                    if distance_to_index < distance_to_next_index:
                        break
                    index -= 1
                    distance_to_index = distance_to_next_index
            else:  # Forward motion
                while True:
                    if (index + 1) >= len(self.x_points):
                        break
                    distance_to_next_index = state.distance_to_state(
                        self.x_points[index + 1], self.y_points[index + 1]
                    )
                    if distance_to_index < distance_to_next_index:
                        break
                    index += 1
                    distance_to_index = distance_to_next_index

        self.old_nearest_point_index = index

        lookahead = self.lookahead_gain * abs(state.velocity) + self.lookahead_min

        # Find target point within lookahead distance
        while lookahead > state.distance_to_state(
            self.x_points[index], self.y_points[index]
        ):
            if reverse:
                if (index - 1) < 0:  # Prevent going below 0
                    break
                index -= 1
            else:
                if (index + 1) >= len(self.x_points):
                    break
                index += 1

        return index, lookahead
