import numpy as np
from scipy.interpolate import CubicSpline

def point_in_polygon(x, y, vertices):
    n = len(vertices)
    inside = False
    p1x, p1y = vertices[0]
    for i in range(n + 1):
        p2x, p2y = vertices[i % n]
        if y > min(p1y, p2y) and y <= max(p1y, p2y) and x <= max(p1x, p2x):
            if p1y != p2y:
                xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
            if p1x == p2x or x <= xinters:
                inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def point_in_circle(x, y, center, radius):
    return (x - center[0])**2 + (y - center[1])**2 <= radius**2

def point_in_rectangle(x, y, center, width, height, orientation=0):
    half_width = width / 2
    half_height = height / 2
    
    # Translate point to origin
    x_translated = x - center[0]
    y_translated = y - center[1]
    
    # Rotate point back
    cos_theta = np.cos(-orientation)
    sin_theta = np.sin(-orientation)
    x_rotated = x_translated * cos_theta - y_translated * sin_theta
    y_rotated = x_translated * sin_theta + y_translated * cos_theta
    
    # Check if point is within the rectangle bounds
    return -half_width <= x_rotated <= half_width and -half_height <= y_rotated <= half_height

def generate_defined_path(vertices, waypoints, num_points=50):
    if not vertices or len(vertices) < 3:
        raise ValueError("At least 3 vertices required")
    if not waypoints or len(waypoints) < 2:
        raise ValueError("At least 2 waypoints required for spline")
    for v in vertices + waypoints:
        if not (isinstance(v, tuple) and len(v) == 2):
            raise ValueError("Points must be (x, y) tuples")
    
    valid_waypoints = [wp for wp in waypoints if point_in_polygon(wp[0], wp[1], vertices)]
    if len(valid_waypoints) < 2:
        raise ValueError("Need at least 2 valid waypoints within workspace")
    
    x, y = zip(*valid_waypoints)
    t = np.arange(len(valid_waypoints))
    cs_x = CubicSpline(t, x)
    cs_y = CubicSpline(t, y)
    
    t_fine = np.linspace(0, len(valid_waypoints) - 1, num_points)
    x_spline = cs_x(t_fine)
    y_spline = cs_y(t_fine)
    
    dx = cs_x(t_fine, 1)
    dy = cs_y(t_fine, 1)
    heading = np.arctan2(dy, dx)
    
    path = []
    for x_val, y_val, theta in zip(x_spline, y_spline, heading):
        if point_in_polygon(x_val, y_val, vertices):
            path.append((x_val, y_val, theta))
    
    if not path:
        raise ValueError("No valid path generated")
    return path