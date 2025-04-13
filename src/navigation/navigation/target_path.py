import numpy as np
from navigation.robot_state import RobotState
from project_interfaces.msg import NavPath

class TargetPath:
    """
    A class used to define a target route for navigation.
    Uses a path message to update the target route.
    """
    def __init__(self, lookahead_gain, lookahead_min):
        self.x_points = []
        self.y_points = []
        self.old_nearest_point_index = None
        self.lookahead_gain = lookahead_gain
        self.lookahead_min = lookahead_min

    def update_path(self, path_msg: NavPath):
        self.x_points = [point.x for point in path_msg.path]
        self.y_points = [point.y for point in path_msg.path]
        
        self.old_nearest_point_index = None

    def reverse_path(self):
        self.x_points.reverse()
        self.y_points.reverse()

        self.old_nearest_point_index = None

    def search_target_index(self,  state: RobotState, reverse=False):
        if not self.x_points or not self.y_points:
            # Checks if there exits a path
            return None, None

        if reverse:
            # Disables optimisation in reverse travel
            self.old_nearest_point_index = None
        if self.old_nearest_point_index is None:
            # Search for nearest point on path to robot state
            dx = [state.x - i for i in self.x_points]
            dy = [state.y - i for i in self.y_points]
            distances = np.hypot(dx, dy)
            index = np.argmin(distances)
        else:
            index = self.old_nearest_point_index
            distance_to_index = state.distance_to_state(self.x_points[index], self.y_points[index])
            while True:
                if (index + 1) >= len(self.x_points):
                    break
                distance_to_next_index = state.distance_to_state(self.x_points[index + 1], self.y_points[index + 1])
                if distance_to_index < distance_to_next_index:
                    break
                index += 1
                distance_to_index = distance_to_next_index
            self.old_nearest_point_index = index

        # Compute the lookahead distance
        lookahead = self.lookahead_gain * abs(state.velocity) + self.lookahead_min

        if reverse:
            # Find target behind the robot's current state
            while lookahead > state.distance_to_state(self.x_points[index], self.y_points[index]):
                if (index - 1) < 0:
                    break
                index -= 1
        else:
            # Find index of target point within lookahead distance
            while lookahead > state.distance_to_state(self.x_points[index], self.y_points[index]):
                if (index + 1) >= len(self.x_points):
                    break
                index += 1

        return index, lookahead
