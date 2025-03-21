#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import ctypes
import struct
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, TransformStamped
import os
from scipy.spatial import cKDTree
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from sklearn.decomposition import PCA
from itertools import permutations
from sensor_msgs_py.point_cloud2 import create_cloud
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from project_interfaces.msg import Object, ObjectList

class ExamineImage(Node):

    def __init__(self):
        super().__init__('examine_image')

        self.mat = None

        self.get_logger().info(f"Init detection")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.object_list_broadcaster = tf2_ros.TransformBroadcaster(self)

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.sub2 = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            qos_profile
        )

        self.pub = self.create_publisher(PointCloud2, '/depth_points_filtered', 100)

        # Create the 'maps' folder if it doesn't exist
        folder_path = os.path.join(os.getcwd(), 'maps')
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)

        # Define the path to the map file
        self.file_path = os.path.join(folder_path, 'Map.txt')

        # Initialize an empty list to store detected objects
        self.object_list = []

        # Publishers for detected objects as a list
        self.object_list_publisher = self.create_publisher(ObjectList, "/detected_objects", 10)

        # Publisher for clusters
        self.cluster_publisher = self.create_publisher(PointCloud2, '/clusters', 10)

        self.broadcaster = self.create_timer(5.0, self.broadcast_object_list)

        # Timer to periodically write to map.txt
        self.write_timer = self.create_timer(5.0, self.write_to_file)  # Write every 5 seconds

        # Add a counter to track the number of messages received
        self.message_counter = 0

        self.read = 0

        self.read_map_file()

    def read_map_file(self):
        if not os.path.exists(self.file_path):
            self.get_logger().warn(f"Map file {self.file_path} does not exist.")
            return

        with open(self.file_path, 'r') as file:
            lines = file.readlines()
            
        for line in lines:
            parts = line.strip().split(',')
            if len(parts) < 4:
                continue

            object_type = parts[0]
            x_cm = float(parts[1])
            y_cm = float(parts[2])
            angle = float(parts[3])

            x = x_cm / 100.0
            y = y_cm / 100.0

            # Create new object message
            object_msg = Object()
            object_msg.x = x
            object_msg.y = y
            object_msg.angle = angle
            object_msg.object_type = object_type

            self.object_list.append(object_msg)

            self.read = 1

            self.get_logger().info(f"Published new object list now includes: {object_type} at ({x:.2f}, {y:.2f})")

            self.get_logger().info(f"Published TF for object: {object_type} at ({x:.2f}, {y:.2f})")

        object_list_msg = ObjectList()
        object_list_msg.header.frame_id = "map"
        object_list_msg.header.stamp = self.get_clock().now().to_msg()
        object_list_msg.length = len(self.object_list)
        object_list_msg.objects = self.object_list
        self.object_list_publisher.publish(object_list_msg)
        

    def cloud_callback(self, msg: PointCloud2):
        # Increment the message counter
        self.message_counter += 1
        # Only process every 5th message
        if self.message_counter % 2 != 0:
            return
        # Reset the counter to avoid overflow
        self.message_counter = 0

        start_time = time.time()  # DEBUGGING EFFICIENCY

        # Read point cloud data from the message
        points_data = pc2.read_points_numpy(msg, skip_nans=True)

        # Extract XYZ coordinates from the point cloud
        points = points_data[:, :3]  # Shape (N, 3)

        # Compute Euclidean distance of each point from the origin
        distances = np.linalg.norm(points, axis=1)

        # Create a boolean mask to filter points:
        # - Points within max_dist from the sensor
        # - Points above the floor (y < 0.09) (y-axis points downwards)
        mask = (distances > 0.04) & (distances < 0.9) & (points[:, 1] < 0.085) & (0.01 < points[:, 1])

        # Apply the mask to filter points before processing colors
        points = points[mask]

        # The color is stored as a floating-point number in the 4th column
        color_floats = points_data[mask, 3].view(np.uint32)  # Convert float to uint32 directly

        # Extract RGB channels using bitwise operations
        red = (color_floats >> 16) & 255
        green = (color_floats >> 8) & 255
        blue = color_floats & 255
        # Normalize colors to the range [0, 1] for consistency
        colors = np.stack((red, green, blue), axis=1).astype(np.float32) / 255

        # Perform spatial clustering using DBSCAN
        clusters, labels = self.dbscan(points, eps=0.05, min_samples=300)  # CHECK

        # Publish clusters using the original header
        self.publish_clusters(points, labels, msg.header)  # CAN BE COMMENTED OUT AFTER TESTING

        valid_labels = labels[labels != -1]  # Filter out noise points (label = -1)
        unique_labels = np.unique(valid_labels)

        # Define HSV [Hue (0-179), Saturation(0-255), Value(0-255)] ranges for filtering color ranges for red, green, and blue
        lower_red, upper_red = np.array([2, 230, 95]), np.array([2, 240, 100])
        lower_green1, upper_green1 = np.array([81, 100, 44]), np.array([84, 255, 105])
        lower_green2, upper_green2 = np.array([73, 210, 90]), np.array([74, 240, 120])
        lower_blue, upper_blue = np.array([99, 254, 75]), np.array([99, 255, 80])
        lower_brown, upper_brown = np.array([15, 68, 137]), np.array([17, 76, 134])

        # Iterate over each cluster
        for cluster_label in unique_labels:
            if cluster_label == -1:
                continue  # Skip noise points (label = -1)

            # Extract points and colors for the current cluster
            cluster_mask = (labels == cluster_label)
            cluster_points = points[cluster_mask]
            cluster_colors = colors[cluster_mask]

            # Convert RGB to HSV for the current cluster
            rgb_colors = cluster_colors * 255  # Scale back to 0-255
            hsv_colors = cv2.cvtColor(rgb_colors.reshape(1, -1, 3).astype(np.uint8), cv2.COLOR_RGB2HSV).reshape(-1, 3)

            # Create masks for the current cluster
            red_mask = ((hsv_colors[:, 0] >= lower_red[0]) & (hsv_colors[:, 0] <= upper_red[0]))
            green_mask = (hsv_colors[:, 0] >= lower_green1[0]) & (hsv_colors[:, 0] <= upper_green1[0]) | \
                         ((hsv_colors[:, 0] >= lower_green2[0]) & (hsv_colors[:, 0] <= upper_green2[0]))
            blue_mask = (hsv_colors[:, 0] >= lower_blue[0]) & (hsv_colors[:, 0] <= upper_blue[0])
            brown_mask = (hsv_colors[:, 0] >= lower_brown[0]) & (hsv_colors[:, 0] <= upper_brown[0])

            # Apply masks for the current cluster
            red_points = cluster_points[red_mask]
            green_points = cluster_points[green_mask]
            blue_points = cluster_points[blue_mask]
            brown_points = cluster_points[brown_mask]

            # Calculate the total number of points in the cluster
            total_points = len(cluster_points)

            # Calculate the ratio of red, green, and blue points
            red_ratio = len(red_points) / total_points
            green_ratio = len(green_points) / total_points
            blue_ratio = len(blue_points) / total_points
            brown_ratio = len(brown_points) / total_points

            pure_red = pure_green = pure_blue = pure_brown = False

            # Check if the cluster is predominantly red, green, or blue
            if red_ratio > 0.001 and green_ratio == 0.0 and blue_ratio == 0.0:
                pure_red = True
            elif green_ratio > 0.001 and red_ratio == 0.0 and blue_ratio == 0.0 and brown_ratio == 0.0:
                pure_green = True
            elif blue_ratio > 0.001 and red_ratio == 0.0 and green_ratio == 0.0 and brown_ratio < 0.01:
                pure_blue = True
            elif brown_ratio > 0.001 and red_ratio == 0.0 and green_ratio == 0.0 and blue_ratio == 0.0:
                pure_brown = True

            # Classify based on floor contact points for the current cluster
            object_type = self.classify_based_on_floor_contact(cluster_points)

            x, y, z = np.mean(cluster_points, axis=0)

            if pure_brown:
                if object_type == "cube":
                    self.get_logger().info(f'🟫 Cluster {cluster_label} is a cube!')
                    self.publish_object(x, z + 0.02, 0.0, Object.CUBE, msg.header.stamp)

            if pure_red or pure_green or pure_blue:
                if object_type == "sphere":
                    if pure_red:
                        emoji = "🔴"  # Red circle emoji
                    elif pure_green:
                        emoji = "🟢"  # Green circle emoji
                    elif pure_blue:
                        emoji = "🔵"  # Blue circle emoji
                    self.get_logger().info(f'{emoji} Cluster {cluster_label} is a sphere!')
                    self.publish_object(x, z + 0.02, 0.0, Object.SPHERE, msg.header.stamp)
                elif object_type == "cube":
                    if pure_red:
                        emoji = "🟥"  # Red square emoji
                    elif pure_green:
                        emoji = "🟩"  # Green square emoji
                    elif pure_blue:
                        emoji = "🟦"  # Blue square emoji
                    self.get_logger().info(f'{emoji} Cluster {cluster_label} is a cube!')
                    self.publish_object(x, z + 0.02, 0.0, Object.CUBE, msg.header.stamp)
                elif object_type == "unknown":
                    self.get_logger().info(f'Object not identified :(!')

            elif self.is_plushie(cluster_points):
                self.get_logger().info(f'🧸 Cluster {cluster_label} is a plushie!')
                self.publish_object(x + 0.01, z, 0.0, Object.PLUSHIE, msg.header.stamp)

            elif self.is_box(cluster_points):  # If detected object is a box
                self.get_logger().info(f'📦 Cluster {cluster_label} is a box!')

                # Compute the orientation angle of the box
                angle = self.estimate_box_orientation(cluster_points)

                # Store box with angle information[detection-5] [INFO] [1742488983.869258672] [robot_detection]:
                if angle == 0.0:
                    self.publish_object(x, z + 0.08, angle, Object.BOX, msg.header.stamp)
                elif angle == 90.0:
                    self.publish_object(x, z + 0.12, angle, Object.BOX, msg.header.stamp)
                else:
                    self.publish_object(x, z + 0.08, angle, Object.BOX, msg.header.stamp)

            else:
                if pure_brown:
                    continue
                else:
                    self.get_logger().info(f'Cluster {cluster_label} is NOT a recognized object.')

            # ------------ TIMER FOR EFFICIENCY CHECK (move where desired) ------------
            end_time = time.time()
            # self.get_logger().info(f"Processing time: {end_time - start_time:.4f} seconds")
            # ------------------------------------------------------------------------

    def write_to_file(self):
        with open(self.file_path, 'w') as file:
            if self.read == 1:
                file.write('\n')
                self.read = 0
            else:
                for object_msg in self.object_list:
                    file.write(f"{object_msg.object_type} {object_msg.x:.2f} {object_msg.y:.2f} {object_msg.angle:.1f}\n")

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
        angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Clip to handle floating-point precision

        # Convert to degrees
        angle_deg = np.degrees(angle)

        # Compute the dimensions of the box
        min_coords = np.min(cluster_points, axis=0)
        max_coords = np.max(cluster_points, axis=0)
        dimensions = max_coords - min_coords
        length, width, height = sorted(dimensions, reverse=True)

        # Check if the box is aligned with the X-axis
        if length < 0.17:  # Tolerance for floating-point comparison
            angle_deg = 90.0  # Rotate by 90 degrees to align with the long edge
        elif 0.17 < length < 0.25:
            angle_deg = 0.0

        self.get_logger().info(f"angle: {angle_deg}")

        return angle_deg

    def is_box(self, cluster_points):
        # Expected box dimensions (meters)
        EXPECTED_LENGTH = 0.26
        EXPECTED_WIDTH = 0.16
        EXPECTED_HEIGHT = 0.10
        TOLERANCE = 0.035  # 3cm tolerance

        # 1. Calculate TRUE VERTICAL HEIGHT (Z-axis)
        z_values = cluster_points[:, 1]
        height = np.max(z_values) - np.min(z_values)
        height_ok = abs(height - EXPECTED_HEIGHT) < TOLERANCE

        # 2. Calculate HORIZONTAL DIMENSIONS (X-Y plane)
        xy_points = cluster_points[:, [0, 2]]
        pca = PCA(n_components=2)
        pca.fit(xy_points)

        # Project points onto horizontal principal axes
        projected = xy_points @ pca.components_.T
        h_length = np.ptp(projected[:, 0])  # Primary horizontal dimension
        h_width = np.ptp(projected[:, 1])  # Secondary horizontal dimension

        # 3. Match dimensions to expected length/width
        dim_match = (
            (abs(h_length - EXPECTED_LENGTH) < TOLERANCE) or
            (abs(h_length - EXPECTED_WIDTH) < TOLERANCE) or
            (abs(h_width - EXPECTED_WIDTH) < TOLERANCE)
        )

        # 4. Aspect ratio validation
        expected_aspect_1 = EXPECTED_LENGTH / EXPECTED_HEIGHT
        expected_aspect_2 = EXPECTED_WIDTH / EXPECTED_HEIGHT
        actual_aspect = h_length / height
        aspect_ok = abs(actual_aspect - expected_aspect_1) < 0.2 or abs(actual_aspect - expected_aspect_2) < 0.2

        return height_ok and (dim_match or aspect_ok)

    def is_plushie(self, cluster_points):
        # Expected box dimensions (meters)
        EXPECTED_LENGTH = 0.09
        EXPECTED_WIDTH = 0.045
        TOLERANCE = 0.04  # 3cm tolerance

        # 2. Calculate HORIZONTAL DIMENSIONS (X-Y plane)
        xy_points = cluster_points[:, [0, 1]]  # This extracts x and y coordinates
        pca = PCA(n_components=2)
        pca.fit(xy_points)

        # Project points onto horizontal principal axes
        projected = xy_points @ pca.components_.T
        h_length = np.ptp(projected[:, 0])  # Primary horizontal dimension
        h_width = np.ptp(projected[:, 1])  # Secondary horizontal dimension

        # 3. Match dimensions to expected length/width
        dim_match = (
            (abs(h_length - EXPECTED_LENGTH) < TOLERANCE) and
            (abs(h_width - EXPECTED_WIDTH) < TOLERANCE)
        )

        return dim_match

    def classify_based_on_floor_contact(self, cluster_points, middle_layer_range=0.02, top_layer_range=0.005):
        cluster_points = np.array(cluster_points)

        # Dynamically calculate the middle layer of the cluster
        # The middle layer is defined as points within a small range around the median height of the cluster
        median_height = np.median(cluster_points[:, 1])  # Median height of the cluster
        middle_layer_points = cluster_points[
            (cluster_points[:, 1] >= median_height - middle_layer_range) &
            (cluster_points[:, 1] <= median_height + middle_layer_range)
        ]
        num_middle_layer_points = len(middle_layer_points)

        # Dynamically calculate the highest layer of the cluster
        min_height = np.min(cluster_points[:, 1])  # Maximum height of the cluster
        highest_layer_points = cluster_points[
            (cluster_points[:, 1] >= min_height) &
            (cluster_points[:, 1] <= min_height + top_layer_range)
        ]
        num_highest_layer_points = len(highest_layer_points)

        # Calculate the ratio of middle layer points to highest layer points
        if num_highest_layer_points == 0:
            return "unknown"  # Avoid division by zero

        ratio = num_middle_layer_points / num_highest_layer_points

        # Classification based on the ratio
        if 1 < ratio <= 6.5:  # Cube: ratio is approximately 1
            return "cube"
        elif 14 > ratio > 6.5:  # Sphere: middle layer has significantly more points
            return "sphere"
        else:
            return "unknown"  # Undefined object

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

        # Filter out noise points (labels == -1)
        valid_mask = labels != -1
        filtered_points = points[valid_mask]
        filtered_labels = labels[valid_mask]

        # Return if no valid points
        if len(filtered_points) == 0:
            return

        # Assign a unique color to each cluster
        unique_colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (128, 0, 0), (0, 128, 0), (0, 0, 128)
        ]

        # Prepare data for PointCloud2
        cloud_data = []
        for i, (x, y, z) in enumerate(filtered_points):
            cluster_id = filtered_labels[i]
            color = unique_colors[int(cluster_id) % len(unique_colors)]

            r, g, b = color
            rgb = struct.unpack('f', struct.pack('BBBB', b, g, r, 0))[0]  # Pack into float

            cloud_data.append((x, y, z, rgb))

        # Define the PointCloud2 message fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # Create a new PointCloud2 message with the filtered points and colors
        cluster_msg = create_cloud(original_header, fields, cloud_data)

        # Publish the clusters
        self.cluster_publisher.publish(cluster_msg)

    def publish_object(self, x, y, angle, object_type, stamp):
        # Create a PointStamped message for the input coordinates
        point_in = PointStamped()
        point_in.header.frame_id = 'camera_depth_optical_frame'  # Input frame
        point_in.header.stamp = stamp  # Timestamp of the original point cloud
        point_in.point = Point(x=x, y=0.09, z=y)  # Set the point coordinates

        try:
            # Lookup the transform from camera_depth_optical_frame to map
            transform = self.tfBuffer.lookup_transform(
                'map',  # Target frame
                point_in.header.frame_id,  # Source frame
                point_in.header.stamp,  # Time of the transform
                rclpy.duration.Duration(seconds=1.0)  # Timeout
            )

            # Transform the point to the map frame
            point_out = do_transform_point(point_in, transform)

            # Extract the transformed coordinates
            x_transformed = point_out.point.x
            y_transformed = point_out.point.y
            z_transformed = point_out.point.z

            # Check if the new object is a duplicate based on proximity
            is_duplicate = False
            for obj in self.object_list:
                distance = np.sqrt((x_transformed - obj.x)**2 + (y_transformed - obj.y)**2)
                if distance < 0.01:  # If the object is within 1 cm of an existing object
                    is_duplicate = True
                    break

            # If not a duplicate, add the new object to the list
            if not is_duplicate:
                # Create new object message
                object_msg = Object()
                object_msg.x = x_transformed
                object_msg.y = y_transformed
                object_msg.angle = angle
                object_msg.object_type = object_type

                self.object_list.append(object_msg)

                object_list_msg = ObjectList()
                object_list_msg.header.frame_id = "map"
                object_list_msg.header.stamp = stamp
                object_list_msg.length = len(self.object_list)
                object_list_msg.objects = self.object_list
                self.object_list_publisher.publish(object_list_msg)

                self.get_logger().info(f"Published new object list now includes: {object_type} at ({x_transformed:.2f}, {y_transformed:.2f})")

        except TransformException as e:
            self.get_logger().error(f"Failed coordinate transform for newly detected object: {e}")


    def broadcast_object_list(self):
        for i, object_msg in enumerate(self.object_list):
            transform = TransformStamped()
            transform.header.frame_id = 'map'  # Change to your desired parent frame
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.child_frame_id = f"{object_msg.object_type}_{i}"
            
            # Set translation from end_point coordinates
            transform.transform.translation.x = object_msg.x
            transform.transform.translation.y = object_msg.y
            transform.transform.translation.z = 0.0
            
            # Set the rotation (quaternion from yaw angle)
            q = Quaternion()
            q.z = np.sin(object_msg.angle / 2.0)  # Convert yaw angle to quaternion
            q.w = np.cos(object_msg.angle / 2.0)
            transform.transform.rotation = q

            
            self.object_list_broadcaster.sendTransform(transform)
        

def main(args=None):
    rclpy.init(args=args)

    examine_image = ExamineImage()

    try:
        rclpy.spin(examine_image)
    except KeyboardInterrupt:
        pass

    examine_image.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
