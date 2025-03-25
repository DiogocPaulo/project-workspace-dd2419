import numpy as np
import matplotlib.pyplot as plt
from utils import generate_defined_path, point_in_polygon

class SimulateLocalisation:
    def __init__(self):
        self.workspace = None
        self.current_pose = None
    
    def create_workspace(self, vertices):
        if not vertices or len(vertices) < 3:
            raise ValueError("At least 3 vertices required")
        for vertex in vertices:
            if not (isinstance(vertex, tuple) and len(vertex) == 2):
                raise ValueError("Vertices must be (x, y) tuples")
        self.workspace = {'vertices': vertices, 'num_vertices': len(vertices), 'closed': True}
        return self.workspace
    
    def generate_laser_scan(self, pose, objects=[], deadzones=[], angular_res=1, max_range=5.0, restrict_to_workspace=True):
        """Generates a realistic laser scan simulation for detecting objects of various shapes."""
        if not self.workspace and restrict_to_workspace:
            raise ValueError("No workspace defined when restrict_to_workspace is True")

        x, y, theta = pose
        angles = np.radians(np.arange(-90, 90, angular_res))
        scan_points = []

        for angle in angles:
            if any(lower <= angle <= upper for lower, upper in deadzones):
                continue  # Skip deadzones

            ray_theta = theta + angle
            hit_detected = False

            # Discretize the ray into points up to max_range
            for distance in np.linspace(0, max_range, 100):
                scan_x = x + distance * np.cos(ray_theta)
                scan_y = y + distance * np.sin(ray_theta)

                # Check workspace boundary if restricted
                if restrict_to_workspace:
                    if not point_in_polygon(scan_x, scan_y, self.workspace['vertices']):
                        #scan_points.append((scan_x, scan_y))  # Record boundary hit
                        #hit_detected = True
                        break

                # Check for intersection with any object
                for obj in objects:
                    if point_in_polygon(scan_x, scan_y, obj['vertices']):
                        #scan_points.append((scan_x, scan_y))  # Record object hit
                        #hit_detected = True
                        break

                if hit_detected:
                    break

            # If no hit detected, append the max range point
            if not hit_detected:
                scan_x = x + max_range * np.cos(ray_theta)
                scan_y = y + max_range * np.sin(ray_theta)
                if not restrict_to_workspace or point_in_polygon(scan_x, scan_y, self.workspace['vertices']):
                    scan_points.append((scan_x, scan_y))

        return scan_points
    
    def visualise(self, pose=None, path=None, scan=None):
        if not self.workspace:
            raise ValueError("No workspace defined")
        vertices = self.workspace['vertices']
        x, y = zip(*vertices)
        plt.clf()
        plt.plot(list(x) + [x[0]], list(y) + [y[0]], 'b-')
        plt.fill(x, y, 'b', alpha=0.3)
        
        if path:
            path_x, path_y, theta = zip(*path)
            plt.plot(path_x, path_y, 'r-', linewidth=2, alpha=0.5)
        
        if pose:
            x, y, theta = pose
            plt.plot(x, y, 'ro', markersize=10)
            dx = np.cos(theta) * 0.5
            dy = np.sin(theta) * 0.5
            plt.arrow(x, y, dx, dy, color='r', head_width=0.2)
        
        if scan:
            scan_x, scan_y = zip(*scan)
            plt.scatter(scan_x, scan_y, color='g', s=10, alpha=0.6)
        
        plt.grid(True)
        plt.axis('equal')
        plt.draw()
        plt.pause(1)  # Minimal pause for rendering
    
    def sim_frame(self, pose):
        """Simulates a single frame update with laser scan."""
        self.current_pose = pose
        scan = self.generate_laser_scan(pose)
        self.visualise(pose=self.current_pose, scan=scan)
    
    def run_sim(self, path):
        if not path:
            raise ValueError("Path required for simulation")
        for pose in path:
            self.sim_frame(pose)
        plt.show()

if __name__ == "__main__":
    sim = SimulateLocalisation()
    vertices = [(-2.2, -1.3), (2.2, -1.3), (4.5, 0.66), (7.0, 0.66), (7.0, 2.84), (5.46, 2.84), (5.46, 1.3), (-2.2, 1.3)]
    waypoints = [(-1, 0), (1, 0), (2, 0.9), (5, 0.9), (6.5, 2)]
    objects = [(3, 1, 0.5), (6, 2, 0.3)]  # Example objects with (x, y, radius)
    try:
        sim.create_workspace(vertices)
        defined_path = generate_defined_path(vertices, waypoints, num_points=10)
        sim.sim_frame(defined_path[-1])
        plt.show()
    except ValueError as e:
        print(f"Error: {e}")
