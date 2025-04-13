import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from std_msgs.msg import Header
from localisation.icp import icp
from collections import defaultdict

class LidarAggregator(Node):
    def __init__(self):
        super().__init__('lidar_aggregator')
        self.get_logger().info('LidarAggregator node initialized')

        # Parameters
        self.declare_parameter('num_scans', 5)
        self.declare_parameter('num_scan_points', 360)
        self.declare_parameter('grid_size', 0.2)
        self.declare_parameter('max_points_per_cell', 10.0)
        self.num_scans = self.get_parameter('num_scans').value
        self.num_scan_points = self.get_parameter('num_scan_points').value
        self.grid_size = self.get_parameter('grid_size').value
        self.max_points_per_cell = self.get_parameter('max_points_per_cell').value

        # State
        self.scan_buffer = np.empty((0, 2), dtype=np.float64)  # Explicitly specify dtype
        self.grid_map = defaultdict(list)
        self.current_pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.last_scan_header = None

        # Transform variables
        self.map_odom_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.transform_x = 0.0
        self.transform_y = 0.0
        self.transform_z = 0.0
        self.transform_theta = 0.0

        # TF2 Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.cloud_pub = self.create_publisher(PointCloud2, '/ag_scan', 10)
        self.create_timer(0.1, self.broadcast_transform)

    def odom_callback(self, msg):
        self.current_pose = self._pose_to_xy_yaw(msg.pose.pose)
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

    def _pose_to_xy_yaw(self, pose):
        x = pose.position.x
        y = pose.position.y
        _, _, yaw = euler_from_quaternion([
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w
        ])
        return np.array([x, y, yaw], dtype=np.float64)

    def scan_callback(self, msg):
        points = self._laser_scan_to_points(msg)
        self.get_logger().info(f"Received {len(points)} points from scan")
        
        transformed_points = self._transform_points(points, 'map', msg.header.frame_id, msg.header.stamp)
        if transformed_points is None or transformed_points.size == 0:
            self.get_logger().warn("Transformed points are empty or None")
            return
            
        self.get_logger().info(f"Transformed points shape: {transformed_points.shape}")
        localised_points = self._localise_points(transformed_points)
        if localised_points is not None and localised_points.size > 0:
            self.get_logger().info(f"Localised points shape: {localised_points.shape}")
            self._update_scan_buffer(localised_points)
            self.publish_aggregated_cloud()
            self.last_scan_header = msg.header

    def _laser_scan_to_points(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float64)
        num_points = len(ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_points)
        valid = (msg.range_min < ranges) & (ranges < msg.range_max)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        return np.column_stack((x, y))

    def _update_scan_buffer(self, points):
        for point in points:
            grid_cell = self._get_grid_cell(point)
            if self._is_low_density(grid_cell, point):
                self.grid_map[grid_cell].append(point.tolist())  # Convert to list for defaultdict
        
        # Convert grid map to scan buffer
        all_points = []
        for cell_points in self.grid_map.values():
            all_points.extend(cell_points)
        if all_points:
            self.scan_buffer = np.array(all_points, dtype=np.float64)
        else:
            self.scan_buffer = np.empty((0, 2), dtype=np.float64)

    def _get_grid_cell(self, point):
        x, y = point
        grid_x = int(np.floor(x / self.grid_size))
        grid_y = int(np.floor(y / self.grid_size))
        return (grid_x, grid_y)

    def _is_low_density(self, grid_cell, point):
        return len(self.grid_map[grid_cell]) < self.max_points_per_cell

    def _transform_points(self, points, target_frame, source_frame, timestamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, timestamp, timeout=rclpy.duration.Duration(seconds=0.1))
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Transform failed: {ex}")
            return None

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

    def _localise_points(self, new_points):
        if abs(self.angular_vel) > 0.1:
            return None
        
        if True:
            return new_points

        aggregated_points = self.scan_buffer[~np.isnan(self.scan_buffer).any(axis=1)]
        new_points = np.array(new_points, dtype=np.float64)

        self.get_logger().info(f"Aggregated points shape: {aggregated_points.shape}")
        self.get_logger().info(f"New points shape: {new_points.shape}")

        try:
            rotation_matrix, translation_vector, localised_points = icp(aggregated_points, new_points)
            self.get_logger().info(f"ICP: Rotation={rotation_matrix}, Translation={translation_vector}")
            return localised_points
        except Exception as e:
            self.get_logger().error(f"ICP failed: {str(e)}")
            return None

    def publish_aggregated_cloud(self):
        if self.scan_buffer.size == 0 or self.last_scan_header is None:
            self.get_logger().warn("Scan buffer empty or no header, skipping publish")
            return

        aggregated_points = self.scan_buffer[~np.isnan(self.scan_buffer).any(axis=1)]
        if aggregated_points.size == 0:
            return

        header = Header()
        header.stamp = self.last_scan_header.stamp
        header.frame_id = 'map'
        aggregated_points_3d = np.hstack((aggregated_points, 
                                        np.full((aggregated_points.shape[0], 1), 
                                              self.transform_z, dtype=np.float64)))
        cloud_msg = pc2.create_cloud_xyz32(header, aggregated_points_3d)
        self.cloud_pub.publish(cloud_msg)
        self.get_logger().info(f"Published aggregated cloud with {len(aggregated_points)} points")

    def broadcast_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = self.transform_x
        t.transform.translation.y = self.transform_y
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.transform_theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.map_odom_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()