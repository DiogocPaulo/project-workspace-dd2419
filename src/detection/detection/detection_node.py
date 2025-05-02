#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import ctypes
import struct
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Pose, Quaternion, Vector3
from sklearn.decomposition import PCA
from sensor_msgs_py.point_cloud2 import create_cloud
import time
from project_interfaces.msg import Object, ObjectList, Vertex, WorkspaceVertices
from mapping.map import Map

class ObjectDetectorNode(Node):

    def __init__(self):
        super().__init__('object_detector')

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, qos_profile)
        self.create_subscription(WorkspaceVertices, "/workspace",  self.workspace_callback, 10)
        
        # Publisher for raw detected objects
        self.raw_object_publisher = self.create_publisher(ObjectList, "/raw_detected_objects", 10)
        # Publisher for clusters (for visualization)
        self.cluster_publisher = self.create_publisher(PointCloud2, '/clusters', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        # Variables
        self.workspace_vertices = []
        self.workspace_map = None
        self.raw_object_list = []
        self.message_counter = 0

        self.get_logger().info("Object Detector Node initialized")

    def workspace_callback(self, msg: WorkspaceVertices):
        if self.workspace_vertices:
            return
        for vertex_msg in msg.vertices:
            x = vertex_msg.x
            y = vertex_msg.y
            self.workspace_vertices.append((x, y))
        # Initialize map based on workspace perimeter
        self.workspace_map = Map(msg.grid_resolution)
        self.workspace_map.initialise_grid(self.workspace_vertices)

    def cloud_callback(self, msg: PointCloud2):
        # Increment the message counter
        self.message_counter += 1
        # Only process every 2nd message
        if self.message_counter % 2 != 0:
            return
        # Reset the counter to avoid overflow
        self.message_counter = 0

        start_time = time.time()

        # Read point cloud data from the message
        points_data = pc2.read_points_numpy(msg, skip_nans=True)

        # Extract XYZ coordinates from the point cloud
        points = points_data[:, :3]  # Shape (N, 3)

        # Compute Euclidean distance of each point from the origin
        distances = np.linalg.norm(points, axis=1)

        # Create a boolean mask to filter points
        mask = (distances > 0.04) & (distances < 0.9) & (points[:, 1] < 0.080) & (0.01 < points[:, 1])

        # Apply the mask to filter points before processing colors
        points = points[mask]

        # The color is stored as a floating-point number in the 4th column
        color_floats = points_data[mask, 3].view(np.uint32)

        # Extract RGB channels using bitwise operations
        red = (color_floats >> 16) & 255
        green = (color_floats >> 8) & 255
        blue = color_floats & 255
        # Normalize colors to the range [0, 1] for consistency
        colors = np.stack((red, green, blue), axis=1).astype(np.float32) / 255

        # Perform spatial clustering using DBSCAN
        clusters, labels = self.dbscan(points, eps=0.05, min_samples=300)

        # Publish clusters for visualization
        self.publish_clusters(points, labels, msg.header)

        valid_labels = labels[labels != -1]  # Filter out noise points (label = -1)
        unique_labels = np.unique(valid_labels)

        # Define HSV ranges for filtering color ranges
        lower_red, upper_red = np.array([2, 230, 95]), np.array([2, 240, 100])
        lower_green1, upper_green1 = np.array([81, 100, 44]), np.array([84, 255, 105])
        lower_green2, upper_green2 = np.array([73, 210, 90]), np.array([74, 240, 120])
        lower_blue, upper_blue = np.array([99, 254, 75]), np.array([99, 255, 80])

        # Temporary list for objects detected in this frame
        frame_objects = []

        # Iterate over each cluster
        for cluster_label in unique_labels:
            if cluster_label == -1:
                continue  # Skip noise points

            # Extract points and colors for the current cluster
            cluster_mask = (labels == cluster_label)
            cluster_points = points[cluster_mask]
            cluster_colors = colors[cluster_mask]

            # Convert RGB to HSV for the current cluster
            rgb_colors = cluster_colors * 255  # Scale back to 0-255
            hsv_colors = cv2.cvtColor(rgb_colors.reshape(1, -1, 3).astype(np.uint8), 
                          cv2.COLOR_RGB2HSV).reshape(-1, 3)

            # Create masks for the current cluster
            red_mask = ((hsv_colors[:, 0] >= lower_red[0]) & (hsv_colors[:, 0] <= upper_red[0]))
            green_mask = (hsv_colors[:, 0] >= lower_green1[0]) & (hsv_colors[:, 0] <= upper_green1[0]) | \
                         ((hsv_colors[:, 0] >= lower_green2[0]) & (hsv_colors[:, 0] <= upper_green2[0]))
            blue_mask = (hsv_colors[:, 0] >= lower_blue[0]) & (hsv_colors[:, 0] <= upper_blue[0])

            # Calculate the total number of points in the cluster
            total_points = len(cluster_points)

            # Calculate the ratio of red, green, and blue points
            red_ratio = len(cluster_points[red_mask]) / total_points
            green_ratio = len(cluster_points[green_mask]) / total_points
            blue_ratio = len(cluster_points[blue_mask]) / total_points

            pure_red = pure_green = pure_blue = False 

            # Check if the cluster is predominantly red, green, or blue
            if red_ratio > 0.001 and green_ratio == 0.0 and blue_ratio == 0.0:
                pure_red = True
            elif green_ratio > 0.001 and red_ratio == 0.0 and blue_ratio == 0.0:
                pure_green = True
            elif blue_ratio > 0.001 and red_ratio == 0.0 and green_ratio == 0.0:
                pure_blue = True

            x, y, z = np.mean(cluster_points, axis=0)

            if pure_red or pure_green or pure_blue:
                # Classify based on floor contact points for the current cluster
                object_type = self.classify_based_on_floor_contact(cluster_points)
                if object_type == "sphere":
                    obj = self.create_object(x, z + 0.02, 0.0, Object.SPHERE, msg.header.stamp)
                    if obj is not None:
                        frame_objects.append(obj)
                elif object_type == "cube":
                    obj = self.create_object(x, z + 0.02, 0.0, Object.CUBE, msg.header.stamp)
                    if obj is not None:
                        frame_objects.append(obj)
            elif self.is_plushie(cluster_points):
                print(f"red:{red_ratio}, blue:{blue_ratio}, geen:{green_ratio}")
                obj = self.create_object(x + 0.01, z, 0.0, Object.PLUSHIE, msg.header.stamp)
                if obj is not None:
                    frame_objects.append(obj)
            elif self.is_box(cluster_points):
                angle = self.estimate_box_orientation(cluster_points)
                obj = self.create_object(x, z + 0.08, angle, Object.BOX, msg.header.stamp)
                if obj is not None:
                    frame_objects.append(obj)

        # Publish the raw detected objects from this frame
        if frame_objects:
            self.get_logger().info(f"frame:{frame_objects}")
            self.publish_raw_objects(frame_objects, msg.header.stamp)

    def create_object(self, x, y, angle, object_type, stamp):
        # Create a PointStamped message for the input coordinates
        point_in = PointStamped()
        point_in.header.frame_id = 'camera_depth_optical_frame'
        point_in.header.stamp = stamp
        point_in.point.x = x
        point_in.point.y = 0.09
        point_in.point.z = y

        try:
            tf_future = self.tf_buffer.wait_for_transform_async(
            target_frame="odom",
            source_frame=point_in.header.frame_id,
            time=stamp
        )
            
            rclpy.spin_until_future_complete(self, tf_future, timeout_sec=1.0)

            if tf_future.done():
                transform = self.tf_buffer.lookup_transform(
                    "odom",
                    point_in.header.frame_id,
                    point_in.header.stamp,
                    rclpy.duration.Duration(seconds=1.0)
                )
                
                # Transform the point to the map frame
                point_out = do_transform_point(point_in, transform)

                # Extract the transformed coordinates
                x_transformed = point_out.point.x
                y_transformed = point_out.point.y

                # Check if within workspace
                if self.workspace_map is not None:
                    is_in = self.workspace_map.is_free(x_transformed, y_transformed, 75)
                else:
                    is_in = False

                if is_in:
                    object_msg = Object()
                    object_msg.x = x_transformed
                    object_msg.y = y_transformed
                    object_msg.angle = angle
                    object_msg.object_type = object_type
                    return object_msg
            else:
                self.get_logger().error("Transform future not completed in time.")
                return None

        except TransformException as e:
            self.get_logger().error(f"Failed coordinate transform: {e}")
            return None

    def publish_raw_objects(self, objects, stamp):
        object_list_msg = ObjectList()
        object_list_msg.header.frame_id = "odom"
        object_list_msg.header.stamp = stamp
        object_list_msg.length = len(objects)
        object_list_msg.objects = objects
        self.raw_object_publisher.publish(object_list_msg)

    def estimate_box_orientation(self, cluster_points):
        # Find the point with the lowest Z-coordinate
        lowest_z_point = cluster_points[np.argmin(cluster_points[:, 2])]

        # Find the point with the highest Z-coordinate
        highest_z_point = cluster_points[np.argmax(cluster_points[:, 2])]

        # Calculate the vector between the highest and lowest Z-points
        box_vector = highest_z_point - lowest_z_point

        # Normalize the vector
        box_vector_normalized = box_vector / np.linalg.norm(box_vector)

        # Fixed x-axis in the reference frame
        x_axis = np.array([1, 0, 0])

        # Compute the dot product and magnitude to calculate the angle
        dot_product = np.dot(x_axis, box_vector_normalized)
        angle = np.arccos(np.clip(dot_product, -1.0, 1.0))

        # Convert to degrees
        angle_deg = np.degrees(angle)

        # Compute the dimensions of the box
        min_coords = np.min(cluster_points, axis=0)
        max_coords = np.max(cluster_points, axis=0)
        dimensions = max_coords - min_coords
        length, width, height = sorted(dimensions, reverse=True)

        # Check if the box is aligned with the X-axis
        if length < 0.17:
            angle_deg = 90.0
        elif 0.17 < length < 0.25:
            angle_deg = 0.0

        return angle_deg

    def is_box(self, cluster_points):
        EXPECTED_LENGTH = 0.26
        EXPECTED_WIDTH = 0.16
        EXPECTED_HEIGHT = 0.10
        TOLERANCE = 0.035

        z_values = cluster_points[:, 1]
        height = np.max(z_values) - np.min(z_values)
        height_ok = abs(height - EXPECTED_HEIGHT) < TOLERANCE

        xy_points = cluster_points[:, [0, 2]]
        pca = PCA(n_components=2)
        pca.fit(xy_points)

        projected = xy_points @ pca.components_.T
        h_length = np.ptp(projected[:, 0])
        h_width = np.ptp(projected[:, 1])

        dim_match = (
            (abs(h_length - EXPECTED_LENGTH) < TOLERANCE) or
            (abs(h_length - EXPECTED_WIDTH) < TOLERANCE) or
            (abs(h_width - EXPECTED_WIDTH) < TOLERANCE)
        )

        expected_aspect_1 = EXPECTED_LENGTH / EXPECTED_HEIGHT
        expected_aspect_2 = EXPECTED_WIDTH / EXPECTED_HEIGHT
        actual_aspect = h_length / height
        aspect_ok = abs(actual_aspect - expected_aspect_1) < 0.2 or abs(actual_aspect - expected_aspect_2) < 0.2

        return height_ok and (dim_match or aspect_ok)

    def is_plushie(self, cluster_points):
        EXPECTED_LENGTH = 0.09
        EXPECTED_WIDTH = 0.045
        TOLERANCE = 0.04

        xy_points = cluster_points[:, [0, 1]]
        pca = PCA(n_components=2)
        pca.fit(xy_points)

        projected = xy_points @ pca.components_.T
        h_length = np.ptp(projected[:, 0])
        h_width = np.ptp(projected[:, 1])

        dim_match = (
            (abs(h_length - EXPECTED_LENGTH) < TOLERANCE) and
            (abs(h_width - EXPECTED_WIDTH) < TOLERANCE)
        )

        return dim_match

    def classify_based_on_floor_contact(self, cluster_points, middle_layer_range=0.02, top_layer_range=0.005):
        cluster_points = np.array(cluster_points)
        median_height = np.median(cluster_points[:, 1])
        middle_layer_points = cluster_points[
            (cluster_points[:, 1] >= median_height - middle_layer_range) &
            (cluster_points[:, 1] <= median_height + middle_layer_range)
        ]
        num_middle_layer_points = len(middle_layer_points)

        min_height = np.min(cluster_points[:, 1])
        highest_layer_points = cluster_points[
            (cluster_points[:, 1] >= min_height) &
            (cluster_points[:, 1] <= min_height + top_layer_range)
        ]
        num_highest_layer_points = len(highest_layer_points)

        if num_highest_layer_points == 0:
            return "unknown"

        ratio = num_middle_layer_points / num_highest_layer_points

        if 1 < ratio <= 6.5:
            return "cube"
        elif 14 > ratio > 6.5:
            return "sphere"
        else:
            return "unknown"

    def dbscan(self, points, eps=0.05, min_samples=200):
        if len(points) == 0:
            return [], np.array([])

        db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = db.labels_
        clusters = [points[labels == i] for i in np.unique(labels) if i != -1]
        return clusters, labels

    def publish_clusters(self, points, labels, original_header):
        if len(points) == 0:
            return

        valid_mask = labels != -1
        filtered_points = points[valid_mask]
        filtered_labels = labels[valid_mask]

        if len(filtered_points) == 0:
            return

        unique_colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (128, 0, 0), (0, 128, 0), (0, 0, 128)
        ]

        cloud_data = []
        for i, (x, y, z) in enumerate(filtered_points):
            cluster_id = filtered_labels[i]
            color = unique_colors[int(cluster_id) % len(unique_colors)]

            r, g, b = color
            rgb = struct.unpack('f', struct.pack('BBBB', b, g, r, 0))[0]

            cloud_data.append((x, y, z, rgb))

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        cluster_msg = create_cloud(original_header, fields, cloud_data)
        self.cluster_publisher.publish(cluster_msg)

def main():
    rclpy.init()
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
