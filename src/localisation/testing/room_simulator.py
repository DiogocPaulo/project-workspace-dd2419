import math
import random
import numpy as np
import matplotlib.pyplot as plt

from laser_scan_structs import LaserScan

class RoomGenerator:
    def __init__(self, width=10.0, height=8.0):
        self.width = width
        self.height = height
        self.objects = []

    def add_object(self, shape, params):
        self.objects.append({'shape': shape, 'params': params})

    def is_inside(self, x, y):
        return 0 <= x <= self.width and 0 <= y <= self.height

    def get_distance(self, x_robot, y_robot, angle_rad):
        max_range = max(self.width, self.height) * 1.5
        end_x = x_robot + max_range * math.cos(angle_rad)
        end_y = y_robot + max_range * math.sin(angle_rad)

        min_dist_sq = float('inf')

        def check_intersection(p1, p2, p3, p4):
            def on_segment(p, q, r):
                return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                        q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

            def orientation(p, q, r):
                val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
                if val == 0: return 0
                return 1 if val > 0 else 2

            o1 = orientation(p1, p2, p3)
            o2 = orientation(p1, p2, p4)
            o3 = orientation(p3, p4, p1)
            o4 = orientation(p3, p4, p2)

            if o1 != o2 and o3 != o4:
                return True

            if o1 == 0 and on_segment(p1, p3, p2): return True
            if o2 == 0 and on_segment(p1, p4, p2): return True
            if o3 == 0 and on_segment(p3, p1, p4): return True
            if o4 == 0 and on_segment(p3, p2, p4): return True

            return False

        robot_pos = (x_robot, y_robot)
        room_boundaries = [
            ((0, 0), (self.width, 0)),
            ((self.width, 0), (self.width, self.height)),
            ((self.width, self.height), (0, self.height)),
            ((0, self.height), (0, 0))
        ]

        for seg in room_boundaries:
            if check_intersection(robot_pos, (end_x, end_y), seg[0], seg[1]):
                intersect_x, intersect_y = None, None
                x1, y1 = robot_pos
                x2, y2 = (end_x, end_y)
                x3, y3 = seg[0]
                x4, y4 = seg[1]

                denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
                if denominator != 0:
                    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator
                    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator
                    if 0 <= t <= 1 and 0 <= u <= 1:
                        intersect_x = x1 + t * (x2 - x1)
                        intersect_y = y1 + t * (y2 - y1)
                        dist_sq = (intersect_x - x_robot)**2 + (intersect_y - y_robot)**2
                        min_dist_sq = min(min_dist_sq, dist_sq)

        for obj in self.objects:
            if obj['shape'] == 'circle':
                center_x, center_y = obj['params']['center']
                radius = obj['params']['radius']

                dx = end_x - x_robot
                dy = end_y - y_robot
                a = dx**2 + dy**2
                b = 2 * (dx * (x_robot - center_x) + dy * (y_robot - center_y))
                c = (x_robot - center_x)**2 + (y_robot - center_y)**2 - radius**2

                discriminant = b**2 - 4 * a * c
                if discriminant >= 0:
                    t1 = (-b + math.sqrt(discriminant)) / (2 * a)
                    t2 = (-b - math.sqrt(discriminant)) / (2 * a)

                    if 0 <= t1 <= 1:
                        intersect_x = x_robot + t1 * dx
                        intersect_y = y_robot + t1 * dy
                        dist_sq = (intersect_x - x_robot)**2 + (intersect_y - y_robot)**2
                        min_dist_sq = min(min_dist_sq, dist_sq)
                    if 0 <= t2 <= 1:
                        intersect_x = x_robot + t2 * dx
                        intersect_y = y_robot + t2 * dy
                        dist_sq = (intersect_x - x_robot)**2 + (intersect_y - y_robot)**2
                        min_dist_sq = min(min_dist_sq, dist_sq)

            elif obj['shape'] == 'rectangle':
                min_x, max_x, min_y, max_y = obj['params']['min_x'], obj['params']['max_x'], obj['params']['min_y'], obj['params']['max_y']
                rect_boundaries = [
                    ((min_x, min_y), (max_x, min_y)),
                    ((max_x, min_y), (max_x, max_y)),
                    ((max_x, max_y), (min_x, max_y)),
                    ((min_x, max_y), (min_x, min_y))
                ]
                for seg in rect_boundaries:
                    if check_intersection(robot_pos, (end_x, end_y), seg[0], seg[1]):
                        x1, y1 = robot_pos
                        x2, y2 = (end_x, end_y)
                        x3, y3 = seg[0]
                        x4, y4 = seg[1]

                        denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
                        if denominator != 0:
                            t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator
                            u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator
                            if 0 <= t <= 1 and 0 <= u <= 1:
                                intersect_x = x1 + t * (x2 - x1)
                                intersect_y = y1 + t * (y2 - y1)
                                dist_sq = (intersect_x - x_robot)**2 + (intersect_y - y_robot)**2
                                min_dist_sq = min(min_dist_sq, dist_sq)

        if min_dist_sq == float('inf'):
            return float('inf')
        else:
            return math.sqrt(min_dist_sq)

    def generate_scan(self, x_robot, y_robot, num_beams=360, angle_min=-math.pi, angle_max=math.pi,
                       noise_std=0.01, outlier_prob=0.02, outlier_range_factor=2.0):
        angles = np.linspace(angle_min, angle_max, num_beams)
        ranges = []
        for angle in angles:
            distance = self.get_distance(x_robot, y_robot, angle)

            if distance != float('inf'):
                distance += random.gauss(0, noise_std)
                distance = max(0, distance)

            if random.random() < outlier_prob:
                if distance != float('inf'):
                    distance *= outlier_range_factor
                else:
                    distance = max(self.width, self.height) * 1.2

            ranges.append(distance)

        return LaserScan(ranges=ranges, angles=list(angles))

def visualize_room_and_scan(room_generator, laser_scan, robot_x, robot_y):
    fig, ax = plt.subplots()
    ax.set_xlim(0, room_generator.width)
    ax.set_ylim(0, room_generator.height)
    ax.set_aspect('equal', adjustable='box')

    rect = plt.Rectangle((0, 0), room_generator.width, room_generator.height, edgecolor='black', facecolor='none')
    ax.add_patch(rect)

    for obj in room_generator.objects:
        if obj['shape'] == 'circle':
            circle = plt.Circle(obj['params']['center'], obj['params']['radius'], edgecolor='blue', facecolor='lightblue')
            ax.add_patch(circle)
        elif obj['shape'] == 'rectangle':
            rect_obj = plt.Rectangle((obj['params']['min_x'], obj['params']['min_y']),
                                     obj['params']['max_x'] - obj['params']['min_x'],
                                     obj['params']['max_y'] - obj['params']['min_y'],
                                     edgecolor='green', facecolor='lightgreen')
            ax.add_patch(rect_obj)

    ax.plot(robot_x, robot_y, 'ro', markersize=8, label='Robot')

    cartesian_points = []
    for i, r in enumerate(laser_scan.ranges):
        if r != float('inf'):
            angle = laser_scan.angles[i]
            x = robot_x + r * math.cos(angle)
            y = robot_y + r * math.sin(angle)
            cartesian_points.append((x, y))

    if cartesian_points:
        x_scan, y_scan = zip(*cartesian_points)
        ax.scatter(x_scan, y_scan, s=5, color='red', label='Laser Scan Points')

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Room and Simulated Laser Scan")
    ax.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    room = RoomGenerator(width=10.0, height=8.0)
    room.add_object('circle', {'center': (3.0, 4.0), 'radius': 1.0})
    room.add_object('rectangle', {'min_x': 6.0, 'max_x': 8.0, 'min_y': 1.0, 'max_y': 3.0})

    robot_x = 1.5
    robot_y = 1.5

    laser_scan = room.generate_scan(robot_x, robot_y, num_beams=180,
                                     noise_std=0.05, outlier_prob=0.05, outlier_range_factor=1.5)

    visualize_room_and_scan(room, laser_scan, robot_x, robot_y)

    laser_scan_2 = room.generate_scan(robot_x, robot_y, num_beams=180,
                                       noise_std=0.05, outlier_prob=0.05, outlier_range_factor=1.5)
    visualize_room_and_scan(room, laser_scan_2, robot_x, robot_y)