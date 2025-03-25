import math
import numpy as np
from scipy.signal import find_peaks

from laser_scan_structs import LaserScan, Keypoint
from laser_scan_utils import polar_to_cartesian, euclidean_distance

class FALKOKeypointDetector:
    def __init__(self, neighbor_radius_factor=0.5, min_neighbors=2, triangle_height_factor=0.05, polar_sectors=8, nms_radius=0.5):
        self.neighbor_radius_factor = neighbor_radius_factor
        self.min_neighbors = min_neighbors
        self.triangle_height_factor = triangle_height_factor
        self.polar_sectors = polar_sectors
        self.nms_radius = nms_radius

    def detect(self, laser_scan: LaserScan) -> list[Keypoint]:
        cartesian_points = laser_scan.get_scan_in_cartesian()
        if not cartesian_points or len(cartesian_points) < 5:
            return []

        keypoints = []
        scores = {}
        candidate_indices = list(range(len(cartesian_points)))
        original_indices_map = [i for i, r in enumerate(laser_scan.ranges) if 0 < r < float('inf')]

        for i_cartesian in candidate_indices:
            pi = cartesian_points[i_cartesian]
            original_index = original_indices_map[i_cartesian]
            range_pi = laser_scan.ranges[original_index]
            ri = self.neighbor_radius_factor * range_pi

            neighbors_indices_cartesian = [j for j in range(len(cartesian_points)) if j != i_cartesian and euclidean_distance(pi, cartesian_points[j]) < ri]
            neighbors_indices_cartesian.sort()

            cl_indices_cartesian = [idx for idx in neighbors_indices_cartesian if idx < i_cartesian]
            cr_indices_cartesian = [idx for idx in neighbors_indices_cartesian if idx > i_cartesian]

            if len(cl_indices_cartesian) < self.min_neighbors or len(cr_indices_cartesian) < self.min_neighbors:
                continue

            xl_index_cartesian = min(cl_indices_cartesian)
            xr_index_cartesian = max(cr_indices_cartesian)
            xl = cartesian_points[xl_index_cartesian]
            xr = cartesian_points[xr_index_cartesian]

            base_length = euclidean_distance(xl, xr)
            height = point_to_line_distance(pi, xl, xr)

            if base_length < ri / self.triangle_height_factor or height < ri / self.triangle_height_factor:
                continue

            score_l = 0
            if len(cl_indices_cartesian) > 1:
                orientations_l = [self._quantize_orientation(cartesian_points[idx], pi, self.polar_sectors) for idx in cl_indices_cartesian]
                for h_idx in range(len(orientations_l)):
                    for k_idx in range(h_idx):
                        score_l += self._orientation_distance(orientations_l[h_idx], orientations_l[k_idx], self.polar_sectors)

            score_r = 0
            if len(cr_indices_cartesian) > 1:
                orientations_r = [self._quantize_orientation(cartesian_points[idx], pi, self.polar_sectors) for idx in cr_indices_cartesian]
                for h_idx in range(len(orientations_r)):
                    for k_idx in range(h_idx):
                        score_r += self._orientation_distance(orientations_r[h_idx], orientations_r[k_idx], self.polar_sectors)

            scores[i_cartesian] = score_l + score_r

        if not scores:
            return []

        sorted_scores = sorted(scores.items(), key=lambda item: item[1])
        suppressed = [False] * len(cartesian_points)
        final_keypoints = []

        for index_cartesian, score in sorted_scores:
            if not suppressed[index_cartesian]:
                original_index = original_indices_map[index_cartesian]
                final_keypoints.append(Keypoint(index=original_index, position=cartesian_points[index_cartesian]))
                pi = cartesian_points[index_cartesian]
                for other_index_cartesian in range(len(cartesian_points)):
                    if other_index_cartesian != index_cartesian and not suppressed[other_index_cartesian] and euclidean_distance(pi, cartesian_points[other_index_cartesian]) < self.nms_radius:
                        suppressed[other_index_cartesian] = True

        return final_keypoints

    def _quantize_orientation(self, p_neighbor, p_ref, num_sectors):
        angle = math.atan2(p_neighbor[1] - p_ref[1], p_neighbor[0] - p_ref[0])
        return int((angle + math.pi) / (2 * math.pi) * num_sectors) % num_sectors

    def _orientation_distance(self, phi1, phi2, num_sectors):
        diff = (phi1 - phi2 + num_sectors / 2) % num_sectors - num_sectors / 2
        return abs(diff)

class OrthogonalCornerDetector:
    def __init__(self, hough_theta_res=1.0, hough_rho_res=0.05, hough_num_rho=1200, alignment_tolerance=0.04,
                 neighbor_radius_a=0.2, neighbor_radius_b=0.07, nms_radius=0.20):
        self.hough_theta_res = math.radians(hough_theta_res)
        self.hough_rho_res = hough_rho_res
        self.hough_num_theta = int(math.pi / self.hough_theta_res)
        self.hough_num_rho = hough_num_rho
        self.alignment_tolerance = alignment_tolerance
        self.neighbor_radius_a = neighbor_radius_a
        self.neighbor_radius_b = neighbor_radius_b
        self.nms_radius = nms_radius

    def detect(self, laser_scan: LaserScan) -> list[Keypoint]:
        cartesian_points = laser_scan.get_scan_in_cartesian()
        if not cartesian_points or len(cartesian_points) < 5:
            return []

        hough_space = np.zeros((self.hough_num_theta, self.hough_num_rho), dtype=int)
        rho_values = np.linspace(-self.hough_num_rho / 2 * self.hough_rho_res, self.hough_num_rho / 2 * self.hough_rho_res, self.hough_num_rho)
        theta_values = np.linspace(0, math.pi, self.hough_num_theta, endpoint=False)
        original_indices_map = [i for i, r in enumerate(laser_scan.ranges) if 0 < r < float('inf')]

        for x, y in cartesian_points:
            for i, theta in enumerate(theta_values):
                rho = x * math.cos(theta) + y * math.sin(theta)
                rho_index = np.argmin(np.abs(rho_values - rho))
                hough_space[i, rho_index] += 1

        hough_spectrum = np.sum(hough_space**2, axis=1)
        orthogonal_hough_spectrum = hough_spectrum + np.roll(hough_spectrum, self.hough_num_theta // 2)
        dominant_theta_index = np.argmax(orthogonal_hough_spectrum)
        dominant_theta = theta_values[dominant_theta_index]

        rotated_points = []
        for x, y in cartesian_points:
            rotated_x = x * math.cos(-dominant_theta) - y * math.sin(-dominant_theta)
            rotated_y = x * math.sin(-dominant_theta) + y * math.cos(-dominant_theta)
            rotated_points.append((rotated_x, rotated_y))

        scores = {}
        for i_cartesian, pi in enumerate(rotated_points):
            radius_i = self.neighbor_radius_a * math.exp(self.neighbor_radius_b * euclidean_distance((0, 0), pi))
            neighbors = [pj for j, pj in enumerate(rotated_points) if j != i_cartesian and euclidean_distance(pi, pj) < radius_i]

            cx_count = 0
            cy_count = 0
            for pj in neighbors:
                if abs(pj[0] - pi[0]) < self.alignment_tolerance and abs(pj[1] - pi[1]) > self.alignment_tolerance:
                    cx_count += 1
                elif abs(pj[1] - pi[1]) < self.alignment_tolerance and abs(pj[0] - pi[0]) > self.alignment_tolerance:
                    cy_count += 1

            score = cx_count + cy_count - abs(cx_count - cy_count)
            scores[i_cartesian] = score

        if not scores:
            return []

        sorted_scores = sorted(scores.items(), key=lambda item: item[1], reverse=True)
        suppressed = [False] * len(cartesian_points)
        final_keypoints = []

        for index_cartesian, score in sorted_scores:
            if not suppressed[index_cartesian] and score > 0:
                original_index = original_indices_map[index_cartesian]
                final_keypoints.append(Keypoint(index=original_index, position=cartesian_points[index_cartesian]))
                pi = cartesian_points[index_cartesian]
                for other_index_cartesian in range(len(cartesian_points)):
                    if other_index_cartesian != index_cartesian and not suppressed[other_index_cartesian] and euclidean_distance(pi, cartesian_points[other_index_cartesian]) < self.nms_radius:
                        suppressed[other_index_cartesian] = True

        return final_keypoints

def point_to_line_distance(point, line_start, line_end):
    if line_start == line_end:
        return euclidean_distance(point, line_start)
    px, py = point
    x1, y1 = line_start
    x2, y2 = line_end
    segment_length_sq = (x2 - x1)**2 + (y2 - y1)**2
    t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / segment_length_sq
    t = max(0, min(1, t))
    closest_x = x1 + t * (x2 - x1)
    closest_y = y1 + t * (y2 - y1)
    return euclidean_distance(point, (closest_x, closest_y))

if __name__ == '__main__':
    from room_simulator import RoomGenerator
    import matplotlib.pyplot as plt

    room = RoomGenerator(width=10.0, height=8.0)
    room.add_object('circle', {'center': (3.0, 4.0), 'radius': 1.0})
    room.add_object('rectangle', {'min_x': 6.0, 'max_x': 8.0, 'min_y': 1.0, 'max_y': 3.0})
    robot_x = 1.5
    robot_y = 1.5
    laser_scan = room.generate_scan(robot_x, robot_y, num_beams=360, noise_std=0.02, outlier_prob=0.01)
    cartesian_scan = laser_scan.get_scan_in_cartesian()

    falko_detector = FALKOKeypointDetector()
    falko_keypoints = falko_detector.detect(laser_scan)
    print("FALKO Keypoint Indices:", [kp.index for kp in falko_keypoints])

    oc_detector = OrthogonalCornerDetector()
    oc_keypoints = oc_detector.detect(laser_scan)
    print("OC Keypoint Indices:", [kp.index for kp in oc_keypoints])

    def visualize_scan_with_keypoints(laser_scan, keypoints, title="Scan with Keypoints"):
        cartesian_points = laser_scan.get_scan_in_cartesian()
        if not cartesian_points:
            print("No valid points to visualize.")
            return

        x = [p[0] for p in cartesian_points]
        y = [p[1] for p in cartesian_points]
        kp_x = [cartesian_points[kp.index][0] for kp in keypoints if kp.index < len(cartesian_points)]
        kp_y = [cartesian_points[kp.index][1] for kp in keypoints if kp.index < len(cartesian_points)]

        plt.figure()
        plt.scatter(x, y, s=5, label="Laser Scan")
        plt.scatter(kp_x, kp_y, s=20, color='red', label="Keypoints")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title(title)
        plt.grid(True)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.legend()
        plt.show()

    visualize_scan_with_keypoints(laser_scan, falko_keypoints, title="FALKO Keypoints")
    visualize_scan_with_keypoints(laser_scan, oc_keypoints, title="Orthogonal Corner Keypoints")