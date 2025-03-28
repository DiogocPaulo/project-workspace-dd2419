import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion
import numpy as np
from visualization_msgs.msg import Marker
import geometry_msgs.msg
import tf2_ros
from scipy.signal import savgol_filter

from localisation.point_to_plane_icp import point_to_plane_icp
from localisation.icp import icp

class LidarAggregator(Node):
    def __init__(self):
        super().__init__('lidar_aggregator')
        
        # Parameters
        self.range_min = 0.3
        self.range_max = 10.0
        self.distance_threshold = 0.2
        self.match_threshold = 0.4
        self.max_segments = 100
        self.min_point_density = 0.0
        self.max_point_density = 0.1
        self.point_spacing = 0.1
        
        # State
        self.last_odom_header = None
        self.current_pose = None
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.rotation = np.eye(2)
        self.translation = np.zeros(2)
        self.transform_z = 0.0
        self.previous_segments = []
        
        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscriptions and Publisher
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/line_segments_markers', 10)
        self.transform_pub = self.create_publisher(geometry_msgs.msg.TransformStamped, '/icp_transform', 10)

        #self.transform_pub = TransformBroadcaster(self)
        
        self.get_logger().info('Lidar Aggregator node has started.')

    def odom_callback(self, msg):
        self.last_odom_header = msg.header
        self.current_pose = self._pose_to_xy_yaw(msg.pose.pose)
        self.linear_velocity = msg.twist.twist.linear.x
        self.angular_velocity = msg.twist.twist.angular.z

    def _pose_to_xy_yaw(self, pose):
        """Convert Pose to [x, y, yaw] numpy array."""
        x = pose.position.x
        y = pose.position.y
        _, _, yaw = euler_from_quaternion([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ])
        return np.array([x, y, yaw])

    def scan_callback(self, msg):
        points = self._laser_scan_to_points(msg)
        if self.last_odom_header is None:
            return
        
        transformed_points = self._transform_points(points, 'map', msg.header.frame_id, self.last_odom_header.stamp)
        if transformed_points is None or transformed_points.size == 0:
            return

        current_segments = self._extract_line_segments(transformed_points)
        rotation, translation, current_segments = self._perform_icp(current_segments, self.previous_segments)
        
        if abs(self.angular_velocity) < 0.1:
            self._publish_transform(rotation, translation)
            self.previous_segments = self._merge_segments(self.previous_segments, current_segments) 
        
        
        self.publish_line_segments([seg['points'] for seg in self.previous_segments], self.last_odom_header)

    def _laser_scan_to_points(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float64)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid = (self.range_min < ranges) & (ranges < self.range_max)
        valid_count = np.sum(valid)
        
        points = np.empty((valid_count, 2), dtype=np.float64)
        points[:, 0] = ranges[valid] * np.cos(angles[valid])
        points[:, 1] = ranges[valid] * np.sin(angles[valid])
        return points

    def _transform_points(self, points, target_frame, source_frame, timestamp):
        try:
            if not self.tf_buffer.can_transform(target_frame, source_frame, timestamp, rclpy.duration.Duration(seconds=0)):
                return None
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, timestamp)
            if points.size == 0:
                return np.empty((0, 2), dtype=np.float64)

            trans = transform.transform.translation
            rot = transform.transform.rotation
            yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]
            cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)

            transformed_points = np.empty((points.shape[0], 2), dtype=np.float64)
            transformed_points[:, 0] = points[:, 0] * cos_yaw - points[:, 1] * sin_yaw + trans.x
            transformed_points[:, 1] = points[:, 0] * sin_yaw + points[:, 1] * cos_yaw + trans.y
            self.transform_z = trans.z
            return transformed_points
        except tf2_ros.TransformException:
            return None

    def _extract_line_segments(self, points):
        if points.size == 0:
            return []

        line_segments = []
        current_segment = [points[0]]

        for point in points[1:]:
            if np.linalg.norm(point - current_segment[-1]) < self.distance_threshold:
                current_segment.append(point)
            else:
                if len(current_segment) > 1:
                    segment_array = np.array(current_segment)
                    smoothed = self._smooth_and_resample(segment_array)
                    line_segments.append({'points': smoothed, 'length': self._arc_length(smoothed)})
                current_segment = [point]

        if len(current_segment) > 1:
            segment_array = np.array(current_segment)
            smoothed = self._smooth_and_resample(segment_array)
            line_segments.append({'points': smoothed, 'length': self._arc_length(smoothed)})
        return line_segments

    def _merge_segments(self, previous_segments, current_segments):
        merged_segments = []
        matched_indices = set()

        for current_segment in current_segments:
            curr_points = current_segment['points']
            curr_length = current_segment['length']
            best_match_index = None
            best_match_score = float('inf')

            for i, prev_segment in enumerate(previous_segments):
                if i in matched_indices:
                    continue
                score, points_within_range = self._calculate_match_score(curr_points, prev_segment['points'])
                if score < best_match_score and score < self.match_threshold:
                    best_match_score = score
                    best_match_index = i

            if best_match_index is not None and points_within_range.size > 0:
                prev_points = previous_segments[best_match_index]['points']
                combined = np.vstack((prev_points, points_within_range))
                ordered = self._order_points(combined)
                smoothed = self._smooth_and_resample(ordered, target_points=len(prev_points))
                new_length = self._arc_length(smoothed)
                
                if new_length > previous_segments[best_match_index]['length'] * 1.1:
                    target_points = max(len(prev_points), int(new_length / self.point_spacing) + 1)
                    smoothed = self._smooth_and_resample(ordered, target_points=target_points)
                
                merged_segments.append({'points': smoothed, 'length': new_length})
                matched_indices.add(best_match_index)
            else:
                merged_segments.append(current_segment)

        return merged_segments[-self.max_segments:]

    def _calculate_match_score(self, segment1, segment2):
        if len(segment1) == 0 or len(segment2) == 0:
            return float('inf'), np.array([])

        diffs = segment1[:, np.newaxis, :] - segment2[np.newaxis, :, :]
        distances = np.linalg.norm(diffs, axis=2)
        min_dists = np.min(distances, axis=1)

        mask = (self.min_point_density < min_dists) & (min_dists < self.max_point_density)
        points_within_range = segment1[mask]
        return np.mean(min_dists), points_within_range

    def _smooth_and_resample(self, points, target_points=None):
        """Smooth points with Savitzky-Golay and resample to target number of points."""
        if len(points) < 2:
            return points

        # Skip smoothing if too few points for Savitzky-Golay
        if len(points) < 5:
            smoothed = points
        else:
            # Ensure window_size is odd and <= len(points)
            window_size = min(11, len(points))
            if window_size % 2 == 0:  # Make odd
                window_size -= 1
            if window_size < 5:  # Minimum for polyorder=3
                window_size = 5 if len(points) >= 5 else len(points)
            x_smooth = savgol_filter(points[:, 0], window_size, 3)
            y_smooth = savgol_filter(points[:, 1], window_size, 3)
            smoothed = np.column_stack((x_smooth, y_smooth))

        # Resample based on arc length
        if target_points is None:
            length = self._arc_length(smoothed)
            target_points = max(2, int(length / self.point_spacing) + 1)

        if len(smoothed) <= target_points:
            return smoothed

        diffs = np.diff(smoothed, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        cum_dist = np.concatenate(([0], np.cumsum(distances)))
        total_length = cum_dist[-1]

        new_distances = np.linspace(0, total_length, target_points)
        x_interp = np.interp(new_distances, cum_dist, smoothed[:, 0])
        y_interp = np.interp(new_distances, cum_dist, smoothed[:, 1])
        return np.column_stack((x_interp, y_interp))

    def _arc_length(self, points):
        if len(points) < 2:
            return 0.0
        diffs = np.diff(points, axis=0)
        return np.sum(np.linalg.norm(diffs, axis=1))

    def _order_points(self, points):
        if len(points) < 2:
            return points
        centroid = np.mean(points, axis=0)
        centered = points - centroid
        _, _, vt = np.linalg.svd(centered)
        principal_direction = vt[0]
        projections = centered.dot(principal_direction)
        sorted_indices = np.argsort(projections)
        return points[sorted_indices]
    
    def _perform_icp(self, current_segments, previous_segments):
        if not previous_segments:
            return np.eye(2), np.zeros(2), current_segments
        
        total_rotation = np.eye(2)
        total_translation = np.zeros(2)
        
        ref_points = np.vstack([seg['points'] for seg in previous_segments])
        curr_points = np.vstack([seg['points'] for seg in current_segments])
        
        rotation, translation, aligned_points = icp(ref_points, curr_points, max_iterations=100, point_pairs_threshold=4)
        self.get_logger().info(f'ICP: {rotation}, {translation}')
        
        total_rotation = total_rotation @ rotation
        total_translation += translation
        
        aligned_current_segments = self._extract_line_segments(aligned_points)
        
        return total_rotation, total_translation, aligned_current_segments


    def _publish_transform(self, rotation, translation):
        # Extract angle from 2x2 rotation matrix
        theta = np.arctan2(rotation[1, 0], rotation[0, 0])  # sinθ / cosθ
        
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.last_odom_header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "map"
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = np.sin(theta / 2)
        t.transform.rotation.w = np.cos(theta / 2)
        self.transform_pub.publish(t)
        #self.get_logger().info(f"Transform published with translation: {translation} and rotation: {rotation}")

    def publish_line_segments(self, line_segments, header_msg):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = header_msg.stamp
        marker.ns = "line_segments"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.a = 1.0

        for segment in line_segments:
            for i in range(len(segment) - 1):
                p1 = segment[i]
                p2 = segment[i + 1]
                marker.points.append(geometry_msgs.msg.Point(x=p1[0], y=p1[1], z=self.transform_z))
                marker.points.append(geometry_msgs.msg.Point(x=p2[0], y=p2[1], z=self.transform_z))

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()