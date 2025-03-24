import math

def polar_to_cartesian(range_val, angle):
    x = range_val * math.cos(angle)
    y = range_val * math.sin(angle)
    return x, y

def euclidean_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle