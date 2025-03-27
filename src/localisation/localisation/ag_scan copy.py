import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
import geometry_msgs.msg
import tf2_ros
from scipy.signal import savgol_filter

class LidarAggregator(Node):
    def __init__(self):
        super().__init__('lidar_aggregator')
        self.get_logger().info('LidarAggregator node initialized')

        # Parameters
        self.declare_parameter('distance_threshold', 0.5)
        self.declare_parameter('match_threshold', 0.4)

        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.match_threshold = self.get_parameter('match_threshold').value
        self.min_point_density = 0.02
        self.max_point_density = 0.1

        # State
        self.last_scan_header = None
        self.last_odom_header = None
        self.transform_z = 0.0
        self.previous_segments = []

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # <-- Nedded for timestamp (Transforms are synced with odometry, but odom not synced with scan)

        # Publisher
        self.marker_pub = self.create_publisher(Marker, '/line_segments_markers', 10)

    def odom_callback(self, msg):
        self.last_odom_header = msg.header

    def scan_callback(self, msg):
        points = self._laser_scan_to_points(msg)
        
        if self.last_odom_header is None:
            return
        
        transformed_points = self._transform_points(points, 'map', msg.header.frame_id, self.last_odom_header.stamp)

        if transformed_points is None:
            self.get_logger().warn("Transformed points are None, skipping line segment extraction")
            return

        if transformed_points.size == 0:
            self.get_logger().warn("Transformed points are empty, skipping line segment extraction")
            return

        current_segments = self._extract_line_segments(transformed_points)
        current_segments = self._smooth_line_segments(current_segments)

        if not self.previous_segments:  # First scan
            self.previous_segments = current_segments
        else:
            self.previous_segments = self._merge_segments(self.previous_segments, current_segments)

        self.publish_line_segments(self.previous_segments, self.last_odom_header)
        self.last_scan_header = msg.header

    def _laser_scan_to_points(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float64)
        num_points = len(ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_points)
        valid = (msg.range_min < ranges) & (ranges < msg.range_max)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        self.get_logger().info(f"Extracted {len(x)} valid points from scan")
        return np.column_stack((x, y))

    def _transform_points(self, points, target_frame, source_frame, timestamp):
        try:
            # Define a time tolerance
            tolerance = rclpy.duration.Duration(seconds=0)

            # Check if the transform is available within the tolerance
            if self.tf_buffer.can_transform(target_frame, source_frame, timestamp, tolerance):
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, timestamp)
                if points.size == 0:
                    return np.empty((0, 2), dtype=np.float64)

                trans = transform.transform.translation
                rot = transform.transform.rotation
                yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]
                cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)

                x = points[:, 0] * cos_yaw - points[:, 1] * sin_yaw + trans.x
                y = points[:, 0] * sin_yaw + points[:, 1] * cos_yaw + trans.y
                self.transform_z = trans.z
                return np.column_stack((x, y))
            else:
                return
                self.get_logger().warn(f"Transform unavailable at {timestamp} with tolerance")

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not transform: {ex}")

    def _extract_line_segments(self, points):
        line_segments = []
        current_segment = []

        for i, point in enumerate(points):
            if not current_segment:
                current_segment.append(point)
            else:
                if np.linalg.norm(point - current_segment[-1]) < self.distance_threshold:
                    current_segment.append(point)
                else:
                    if len(current_segment) > 1:
                        line_segments.append(np.array(current_segment))
                    current_segment = [point]

        if len(current_segment) > 1:
            line_segments.append(np.array(current_segment))

        return line_segments

    def _smooth_line_segments(self, line_segments):
        smoothed_segments = []
        for segment in line_segments:
            if len(segment) >= 5:  # Check if the segment has at least 5 points
                x = savgol_filter(segment[:, 0], 5, 3)
                y = savgol_filter(segment[:, 1], 5, 3)
                smoothed_segments.append(np.column_stack((x, y)))
            else:
                smoothed_segments.append(segment)

        return smoothed_segments

    def _merge_segments(self, previous_segments, current_segments):
        merged_segments = []
        matched_indices = set()
        min_deviation = 0.05  # Minimum deviation threshold to keep a point (in meters)

        for current_segment in current_segments:
            best_match_index = None
            best_match_score = float('inf')

            for prev_index, prev_segment in enumerate(previous_segments):
                if prev_index in matched_indices:
                    continue
                match_score, points_within_range = self._calculate_match_score(current_segment, prev_segment)
                if match_score < best_match_score and match_score < self.match_threshold:
                    best_match_score = match_score
                    best_match_index = prev_index

            if best_match_index is not None and points_within_range.size > 0:
                combined_points = np.vstack((previous_segments[best_match_index], points_within_range))
                ordered_points = self._order_points(combined_points)
                smoothed_segment = self._smooth_segment(ordered_points)
                #simplified_segment = self._resample_segment(smoothed_segment, self.point_density)
                merged_segments.append(smoothed_segment)
                matched_indices.add(best_match_index)
            else:
                smoothed_segment = self._smooth_segment(current_segment)
                #simplified_segment = self._resample_segment(smoothed_segment, self.point_density)
                merged_segments.append(smoothed_segment)

        return merged_segments

    def _resample_segment(self, segment, point_density):
        """Resamples a segment to achieve a desired point density."""
        if len(segment) < 2:
            return segment

        min_distance = 1.0 / point_density  # Minimum distance between points

        new_points = [segment[0]]  # Always keep the first point
        last_kept_index = 0

        for i in range(1, len(segment)):
            distance = np.linalg.norm(segment[i] - new_points[-1])
            if distance >= min_distance:
                new_points.append(segment[i])
                last_kept_index = i

        if last_kept_index != len(segment) - 1 and len(segment) > 1:
            new_points.append(segment[-1])  # Always keep the last point

        return np.array(new_points)

    def _simplify_segment_distance(self, segment, min_deviation):
        if len(segment) < 3:
            return segment

        keep_points = [segment[0]]  # Always keep the first point
        for i in range(1, len(segment) - 1):
            prev_point = keep_points[-1]
            curr_point = segment[i]
            next_point = segment[i + 1]
            
            # Calculate the perpendicular distance from curr_point to the line prev_point -> next_point
            line_vec = next_point - prev_point
            point_vec = curr_point - prev_point
            line_len = np.linalg.norm(line_vec)
            
            if line_len == 0:
                continue
            
            # Distance from point to line
            distance = np.linalg.norm(point_vec - (np.dot(point_vec, line_vec) / (line_len ** 2)) * line_vec)
            
            if distance >= min_deviation:  # Keep point if it deviates enough
                keep_points.append(curr_point)
        
        keep_points.append(segment[-1])  # Always keep the last point
        return np.array(keep_points)

    def _calculate_match_score(self, segment1, segment2):
        """
        Calculates the average distance between points in segment1 and segment2.
        """
        if len(segment1) == 0 or len(segment2) == 0:
            return float('inf')  # Return infinity if either segment is empty

        total_dist = 0.0
        points_within_range = []
        for point1 in segment1:
            min_dist = float('inf')
            for point2 in segment2:
                dist = np.linalg.norm(point1 - point2)
                min_dist = min(min_dist, dist)
            total_dist += min_dist
            # Only include points within max_distance
            if self.min_point_density < min_dist < self.max_point_density:
                points_within_range.append(point1)

        points_within_range = np.array(points_within_range) if points_within_range else np.array([])

        return (total_dist / len(segment1)), points_within_range  # Return the average distance and points_within_range

    def _smooth_segment(self, segment):
        if len(segment) >= 5:
            x = savgol_filter(segment[:, 0], 4, 3)
            y = savgol_filter(segment[:, 1], 4, 3)
            return np.column_stack((x, y))
        else:
            return segment

    def _order_points(self, points):
        """Order points along their principal direction using PCA."""
        if len(points) < 2:
            return points

        # Center the points
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid

        # Compute PCA (principal direction) using SVD
        _, _, vt = np.linalg.svd(centered_points)
        principal_direction = vt[0]  # First principal component

        # Project points onto the principal direction
        projections = centered_points.dot(principal_direction)
        
        # Sort points based on their projection values
        sorted_indices = np.argsort(projections)
        ordered_points = points[sorted_indices]

        return ordered_points

    def publish_line_segments(self, line_segments, header_msg):
        if not line_segments:
            self.get_logger().warn("No line segments, skipping publish")
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = header_msg.stamp
        marker.ns = "line_segments"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for segment in line_segments:
            for i in range(len(segment) - 1):
                p1 = segment[i]
                p2 = segment[i + 1]
                marker.points.append(geometry_msgs.msg.Point(x=p1[0], y=p1[1], z=self.transform_z))
                marker.points.append(geometry_msgs.msg.Point(x=p2[0], y=p2[1], z=self.transform_z))

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published {len(line_segments)} line segments as markers")

def main(args=None):
    rclpy.init()
    node = LidarAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()