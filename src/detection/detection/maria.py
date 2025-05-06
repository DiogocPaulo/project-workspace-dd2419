#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
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
from geometry_msgs.msg import PointStamped
from scipy.spatial import cKDTree


class ExamineImage(Node):

    def __init__(self):
        super().__init__('examine_image')

        self.mat = None

        self.get_logger().info(f"Init detection")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer,self)

        self.sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            100)
        
        self.sub2 = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            100)
        
        self.pub = self.create_publisher(PointCloud2,'camera/camera_depth/color/points_transformed',100)

        self.cluster_publisher = self.create_publisher(PointCloud2, 'clusters', 10)

    def image_callback(self, msg):
        sz = (msg.height, msg.width)
        if False:
            print('{encoding} {width} {height} {step} {data_size}'.format(
                encoding=msg.encoding, width=msg.width, height=msg.height,
                step=msg.step, data_size=len(msg.data)))
        if msg.step * msg.height != len(msg.data):
            print('bad step/height/data size')
            return

        if msg.encoding == 'rgb8':
            dirty = (self.mat is None or msg.width != self.mat.shape[1] or
                     msg.height != self.mat.shape[0] or len(self.mat.shape) < 2 or
                     self.mat.shape[2] != 3)
            if dirty:
                self.mat = np.zeros([msg.height, msg.width, 3], dtype=np.uint8)
            self.mat[:, :, 2] = np.array(msg.data[0::3]).reshape(sz)
            self.mat[:, :, 1] = np.array(msg.data[1::3]).reshape(sz)
            self.mat[:, :, 0] = np.array(msg.data[2::3]).reshape(sz)
        elif msg.encoding == 'mono8':
            self.mat = np.array(msg.data).reshape(sz)
        else:
            print('unsupported encoding {}'.format(msg.encoding))
            return
        if self.mat is not None:
            cv2.imshow('image', self.mat)
            cv2.waitKey(5)

    def cloud_callback(self, msg: PointCloud2):
        # Convert ROS -> NumPy

        gen = pc2.read_points_numpy(msg, skip_nans=True)
        points = gen[:, :3]
        colors = np.empty(points.shape, dtype=np.uint32)

        for idx, x in enumerate(gen):
            c = x[3]
            s = struct.pack('>f', c)
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value
            colors[idx, 0] = np.asarray((pack >> 16) & 255, dtype=np.uint8)
            colors[idx, 1] = np.asarray((pack >> 8) & 255, dtype=np.uint8)
            colors[idx, 2] = np.asarray(pack & 255, dtype=np.uint8)

        colors = colors.astype(np.float32) / 255

        max_dist = 0.9
        distance = np.linalg.norm(points,axis=1)
        mask = (distance < max_dist) & (points[:,1] >= 0.05) & (points[:,1] <= 0.15)
        points = points[mask] 
        colors = colors[mask]

        # Convert RGB to HSV
        rgb_colors = colors * 255  # Scale back to 0-255
        hsv_colors = cv2.cvtColor(rgb_colors.reshape(1, -1, 3).astype(np.uint8), cv2.COLOR_RGB2HSV).reshape(-1, 3)

        # Define HSV ranges for filtering
        lower_red, upper_red = np.array([2, 230, 95]), np.array([2, 240, 100])
        lower_green1, upper_green1 = np.array([81,100,44]), np.array([84,255,105])
        lower_green2, upper_green2 = np.array([73,210,90]), np.array([74,240,120])
        lower_blue, upper_blue = np.array([99, 254, 75]), np.array([99, 255, 80])

        # Create masks
        red_mask = ((hsv_colors[:, 0] >= lower_red[0]) & (hsv_colors[:, 0] <= upper_red[0]))
        green_mask = (hsv_colors[:, 0] >= lower_green1[0]) & (hsv_colors[:, 0] <= upper_green1[0])  | \
           ((hsv_colors[:, 0] >= lower_green2[0]) & (hsv_colors[:, 0] <= upper_green2[0]))
        blue_mask = (hsv_colors[:, 0] >= lower_blue[0]) & (hsv_colors[:, 0] <= upper_blue[0])

        # Apply masks
        red_points = points[red_mask]
        green_points = points[green_mask]
        blue_points = points[blue_mask]
  
        """         if len(green_points)>400:
            self.get_logger().info('GREEN')
         if len(blue_points)>400:
            self.get_logger().info('BLUE') 
        if len(red_points)>400:
            self.get_logger().info('RED')

        if len(red_points) > 400 or len(green_points) > 400:
            #self.get_logger().info('Colored points detected. Starting classification...')
            if self.is_cube(points):
                self.get_logger().info('cube detected!')
            elif self.is_sphere(points):
                self.get_logger().info('sphere detected!')
            else:
                self.get_logger().info('Object detected but not classified as a sphere or cube.')
        elif self.is_box(points):
                self.get_logger().info('BOXIEEEEEEE detected!')
        elif self.is_plushie(points):
                self.get_logger().info('ANIMAL PLUSHIE IM HERE!')
        else:
            self.get_logger().info('No significant colored points detected. No object classified.')  """

        
        # Step 1: Detect Clusters using DBSCAN
        clusters, labels = self.dbscan(points, eps=0.05, min_samples=1000)

        self.get_logger().info(f'Found {len(clusters)} clusters')

        self.publish_clusters(points, labels)

        for idx, cluster in enumerate(clusters):
            if len(cluster) < 50:
                continue  # Ignore small clusters

            # Step 2: Compute bounding box
            min_vals = np.min(cluster, axis=0)
            max_vals = np.max(cluster, axis=0)
            bbox = max_vals - min_vals  # Length, Width, Height

            # Step 3: Classify the Cluster
            if self.is_box(cluster):
                self.get_logger().info(f'Cluster {idx}: BOX detected!')
            elif self.is_sphere(cluster):
                self.get_logger().info(f'Cluster {idx}: SPHERE detected!')
            elif self.is_plushie(cluster):
                self.get_logger().info(f'Cluster {idx}: PLUSHIE detected!')
            else:
                self.get_logger().info(f'Cluster {idx}: Unknown object.')

        

        """ #  Transform points to 'map' frame
        frame_id = msg.header.frame_id
        time_stamp = msg.header.stamp

        try:
            t = self.tfBuffer.lookup_transform('map', frame_id, time_stamp)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {frame_id} to map: {ex}')
            return

        cloud_data = []
        for i, p in enumerate(points):
            point_stamped = PointStamped()
            point_stamped.header.frame_id = frame_id
            point_stamped.header.stamp = time_stamp
            point_stamped.point.x, point_stamped.point.y, point_stamped.point.z = p
            transformed_point = do_transform_point(point_stamped, t)

            # Preserve the original color
            r, g, b = (colors[i] * 255).astype(np.uint8)
            color = struct.unpack('I', struct.pack('BBBB', r, g, b, 0))[0]
            cloud_data.append(struct.pack('ffff', transformed_point.point.x, transformed_point.point.y, transformed_point.point.z, color))

        # Create a new PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = time_stamp
        cloud_msg.header.frame_id = 'map'
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * len(points)
        cloud_msg.is_dense = True
        cloud_msg.data = b''.join(cloud_data)

        # Publish the transformed filtered point cloud
        self.pub.publish(cloud_msg) """
  
    def is_sphere(self, points):

            # Compute bounding box dimensions
            min_vals = np.min(points, axis=0)
            max_vals = np.max(points, axis=0)
            dimensions = max_vals - min_vals 
            
            # Compute the bounding box ratio
            max_dim = np.max(dimensions)
            min_dim = np.min(dimensions)
            ratio = max_dim / min_dim   
            
            if ratio < 8:
                #self.get_logger().info(f'Object classified as a sphere with sphericity: {sphericity:.4f}')
                return True

            return False  # Not a sphere


    def is_cube(self, points):
        # Compute bounding box dimensions
        min_vals = np.min(points, axis=0)
        max_vals = np.max(points, axis=0)
        dimensions = max_vals - min_vals

        # Compute the bounding box ratio
        max_dim = np.max(dimensions)
        min_dim = np.min(dimensions)
        ratio = max_dim / min_dim

        # Fixing the bounding box ratio check
        if ratio > 15:  # A cube should have similar width, height, and depth
            return True  # The object is not a cube
        
        return False


    def is_box(self, points_filtered):
        #Detect if the object is a box based on dimensions.
        min_coords = np.min(points_filtered, axis=0)
        max_coords = np.max(points_filtered, axis=0)
        dimensions = max_coords - min_coords
        length, width, height = dimensions

        #self.get_logger().info(f"Box Check - length: {length:.3f}, width: {width:.3f}, height: {height:.3f}")

        # Check box dimensions (with tolerance)
        return (0.5 <= length <= 0.9 and 0.1 <= width <= 0.12 and 0.5 <= height <= 0.8) 
    
    def is_plushie(self, points_filtered):
        #Detect if the object is a box based on dimensions.
        min_coords = np.min(points_filtered, axis=0)
        max_coords = np.max(points_filtered, axis=0)
        dimensions = max_coords - min_coords
        length, width, height = dimensions

        #self.get_logger().info(f"Plushie Check - length: {length:.3f}, width: {width:.3f}, height: {height:.3f}")

        # Check box dimensions (with tolerance)
        return (0.8 <= length <= 0.9 and 0.11 <= width <= 0.15 and 0.56 <= height <= 0.66)
    

    def dbscan(self, points, eps=0.05, min_samples=10):
        """
        Custom DBSCAN implementation for clustering point clouds.
        
        :param points: Nx3 numpy array of 3D points.
        :param eps: Maximum distance for two points to be in the same cluster.
        :param min_samples: Minimum number of points required to form a cluster.
        :return: List of clusters, cluster labels (same length as points array).
        """
        if len(points) == 0:
            return [], np.array([])

        tree = cKDTree(points)
        labels = -np.ones(len(points))  # -1 means unclassified
        cluster_id = 0

        for i in range(len(points)):
            if labels[i] != -1:
                continue  # Already classified

            neighbors = tree.query_ball_point(points[i], eps)

            if len(neighbors) < min_samples:
                continue  # Noise, not a cluster

            labels[i] = cluster_id
            queue = list(neighbors)

            while queue:
                point_idx = queue.pop()

                if labels[point_idx] == -1:  # If noise, mark as part of the cluster
                    labels[point_idx] = cluster_id
                if labels[point_idx] != -1:
                    continue  # Already assigned to a cluster

                labels[point_idx] = cluster_id
                new_neighbors = tree.query_ball_point(points[point_idx], eps)

                if len(new_neighbors) >= min_samples:
                    queue.extend(new_neighbors)  # Expand cluster

            cluster_id += 1

        clusters = []
        for i in range(cluster_id):
            clusters.append(points[labels == i])

        return clusters, labels  # Return both clusters and their labels

    def publish_clusters(self, points, labels):

        if len(points) == 0:
            return

        cluster_msg = PointCloud2()
        cluster_msg.header.stamp = self.get_clock().now().to_msg()
        cluster_msg.header.frame_id = "map"  # Adjust to your robot's frame

        cluster_msg.height = 1
        cluster_msg.width = len(points)
        cluster_msg.is_dense = False
        cluster_msg.is_bigendian = False
        cluster_msg.point_step = 16  # 4 floats (x, y, z, color)
        cluster_msg.row_step = cluster_msg.point_step * len(points)

        # Define PointCloud2 fields (XYZ + RGB)
        cluster_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # Assign a unique color to each cluster
        unique_colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255),
            (0, 255, 255), (128, 0, 0), (0, 128, 0), (0, 0, 128)
        ]

        # Prepare data for PointCloud2
        cloud_data = []
        for i, (x, y, z) in enumerate(points):
            cluster_id = labels[i]
            if cluster_id == -1:  # Noise points in black
                color = (0, 0, 0)
            else:
                color = unique_colors[int(cluster_id) % len(unique_colors)]

            r, g, b = color
            rgb = struct.unpack('f', struct.pack('BBBB', b, g, r, 0))[0]  # Pack into float

            cloud_data.append(struct.pack('ffff', x, y, z, rgb))

        cluster_msg.data = b''.join(cloud_data)
        
        # Publish the clusters
        self.cluster_publisher.publish(cluster_msg)


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