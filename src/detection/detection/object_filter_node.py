#!/usr/bin/env python3

import math
import numpy as np
import os

import rclpy
from rclpy.node import Node

from project_interfaces.msg import Object, ObjectList
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, Quaternion

import tf2_ros

from mapping.map import Map

class ObjectFilterNode(Node):

    def __init__(self):
        super().__init__('object_filter')

        self.create_subscription(ObjectList, "/raw_detected_objects", self.raw_objects_callback, 10)
        self.create_subscription(OccupancyGrid, "/obstacles_map", self.obstacles_map_callback, 10)
        self.object_list_publisher = self.create_publisher(ObjectList, "/detected_objects", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Variables
        self.raw_objects = []
        self.object_list = []
        self.initial_object_list = []  
        self.obstacles_map = None
        
        # Constant
        self.map_file = "maps/map.csv"
        
        self.read_map(self.map_file)
        self.create_timer(2.0, self.update_object_list)

    def obstacles_map_callback(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Create map or update map grid
        if self.obstacles_map is None:
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            self.obstacles_map = Map(resolution, origin_x, origin_y, width, height, grid)
        else:
            self.obstacles_map.update_grid(grid)

    def raw_objects_callback(self, msg: ObjectList):
        for raw_obj in msg.objects:
            self.initial_object_list.append(raw_obj)

            # Check if the new object is a duplicate based on proximity
            is_duplicate = False
            for i, obj in enumerate(self.initial_object_list[:-1]):
                distance = np.sqrt((raw_obj.x - obj.x)**2 + (raw_obj.y - obj.y)**2)
                if raw_obj.object_type == "box" or obj.object_type == "box":
                    if distance < 0.24:    #f the object is within 1 cm of an existing object
                        is_duplicate = True
                        break
                elif distance < 0.06:   # If the object is within 1 cm of an existing object
                    is_duplicate = True
                    break

            if self.obstacles_map is not None:
                free_from_obstacles = self.obstacles_map.are_adjacent_free(raw_obj.x, raw_obj.y, 1, 75)
            else:
                free_from_obstacles = True

            self.get_logger().info(f"initial:{self.initial_object_list} and free:{free_from_obstacles}")

            # If not a duplicate, add the new object to the list
            if not is_duplicate and free_from_obstacles:
                # Create new object message
                object_msg = Object()
                object_msg.x = raw_obj.x
                object_msg.y = raw_obj.y
                object_msg.angle = raw_obj.angle
                object_msg.object_type = raw_obj.object_type

                self.object_list.append(object_msg)

                object_list_msg = ObjectList()
                object_list_msg.header.frame_id = "map"
                object_list_msg.header.stamp = msg.header.stamp
                object_list_msg.length = len(self.object_list)
                object_list_msg.objects = self.object_list
                self.object_list_publisher.publish(object_list_msg)

                self.get_logger().info(f"Published new object list now includes: {raw_obj.object_type} at ({raw_obj.x:.2f}, {raw_obj.y:.2f})")

                # Confidence-based correction
            CONFIDENCE_RADIUS = 0.06 # 5cm
            MIN_CONSISTENT = 2        # Need at least 2 consistent observations
            self.get_logger().info(f"obstacles:{self.object_list}")
            
            # Find all objects in this area
            nearby = []
            for obj in self.initial_object_list:
                dist = np.sqrt((raw_obj.x - obj.x)**2 + (raw_obj.y - obj.y)**2)
                if raw_obj.object_type == "box" or obj.object_type == "box":
                    if dist <= 0.24:
                        nearby.append(obj)
                elif dist <= CONFIDENCE_RADIUS: # If the object is within 1 cm of an existing object
                    nearby.append(obj)
            
            # Count object types in this area
            type_counts = {}
            for obj in nearby:
                type_counts[obj.object_type] = type_counts.get(obj.object_type, 0) + 1
            
            # Find most common type if we have enough consistent observations
            if len(nearby) >= MIN_CONSISTENT:
                most_common, count = max(type_counts.items(), key=lambda x: x[1])
                if count >= MIN_CONSISTENT:
                    for i, obj in enumerate(self.object_list):
                        # Check if this object is in the nearby area
                        for nearby_obj in nearby:
                            dist = np.sqrt((obj.x - nearby_obj.x)**2 + (obj.y - nearby_obj.y)**2)
                            if obj.object_type == "box":
                                if dist <= 0.24:
                                    self.object_list[i].object_type = "box"
                                    break
                            elif dist <= CONFIDENCE_RADIUS:
                                # Update the type in the main object list
                                self.object_list[i].object_type = most_common
                                break

                    # Re-publish the corrected object list
                    object_list_msg = ObjectList()
                    object_list_msg.header.frame_id = "map"
                    object_list_msg.header.stamp = msg.header.stamp
                    object_list_msg.length = len(self.object_list)
                    object_list_msg.objects = self.object_list
                    self.object_list_publisher.publish(object_list_msg)
                    
                    self.get_logger().info(f"final:{self.object_list}")
                    #self.get_logger().info(f"Corrected object at ({raw_obj.x:.2f}, {raw_obj.y:.2f}) to {most_common}")
            
    def check_duplicates(self):
        duplicates_removed = 0
        n = len(self.object_list)
        to_remove = set()  # Stores indices of objects to remove

        for i in range(n):
            if i in to_remove:
                continue  # Skip if already marked for removal
            
            obj1 = self.object_list[i]
            
            for j in range(i + 1, n):
                if j in to_remove:
                    continue  # Skip if already marked
                
                obj2 = self.object_list[j]
                dx = obj1.x - obj2.x
                dy = obj1.y - obj2.y
                dist = np.sqrt(dx**2 + dy**2)

                # Determine threshold based on types
                if obj1.object_type == "box" or obj2.object_type == "box":
                    threshold = 0.18  # Boxes need 18cm
                elif obj1.object_type == obj2.object_type: 
                    threshold = 0.15
                else:
                    threshold = 0.06 # Others need 5cm

                # If too close, mark the second object for removal
                if dist < threshold:
                    to_remove.add(j)
                    duplicates_removed += 1

        # Rebuild the list, excluding duplicates
        self.object_list = [obj for idx, obj in enumerate(self.object_list) if idx not in to_remove]
        return

    def publish_object_list(self):
        object_list_msg = ObjectList()
        object_list_msg.header.frame_id = "map"
        object_list_msg.header.stamp = self.get_clock().now().to_msg()
        object_list_msg.length = len(self.object_list)
        object_list_msg.objects = self.object_list
        self.object_list_publisher.publish(object_list_msg)

    def broadcast_object_list(self):
        for i, object_msg in enumerate(self.object_list):
            # Publish object as transform
            angle = object_msg.angle * float(math.pi / 180)
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = np.sin(angle * 0.5)
            quaternion.w = np.cos(angle * 0.5)

            transform_msg = TransformStamped()
            transform_msg.header.stamp = self.get_clock().now().to_msg()
            transform_msg.header.frame_id = "map"
            transform_msg.child_frame_id = f"{object_msg.object_type}_{i}"
            transform_msg
            transform_msg.transform.translation.x = object_msg.x
            transform_msg.transform.translation.y = object_msg.y
            transform_msg.transform.translation.z = 0.0

            transform_msg.transform.rotation.x = quaternion.x
            transform_msg.transform.rotation.y = quaternion.y
            transform_msg.transform.rotation.z = quaternion.z
            transform_msg.transform.rotation.w = quaternion.w

            self.tf_broadcaster.sendTransform(transform_msg)

    def update_object_list(self):
        self.check_duplicates()
        self.publish_object_list()
        self.broadcast_object_list()

    def read_map(self, filename):
        objects = []
        try:
            with open(filename, "r") as file:
                for line in file:
                    parts = line.strip().split(',')
                    if len(parts) < 4:
                        self.get_logger().warn(f"Skipping invalid line: {line}")
                        continue
                    object_msg = Object()
                    if parts[0] == "1":
                        object_msg.object_type = Object.CUBE
                    elif parts[0] == "2":
                        object_msg.object_type = Object.SPHERE
                    elif parts[0] == "3":
                        object_msg.object_type = Object.PLUSHIE
                    elif parts[0] == "B":
                        object_msg.object_type = Object.BOX
                    else:
                        self.get_logger().warn(f"Skipping line due to unknown object type: {line}")
                        continue
                    object_msg.x = float(parts[1])/100
                    object_msg.y = float(parts[2])/100
                    object_msg.angle = float(parts[3])
                    objects.append(object_msg)
                    self.get_logger().info(f"Added object (type: {object_msg.object_type}) at ({object_msg.x:.2f}, {object_msg.y:.2f})")
            self.get_logger().info(f"Read {len(objects)} objects from {filename}")
            self.object_list.extend(objects)
            self.update_object_list()
        except FileNotFoundError:
            self.get_logger().warn(f"Map file ({filename}) not found!")

    def write_map(self, filename):
        try:
            with open(filename, 'w') as file:
                for object_msg in self.object_list:
                    if object_msg.object_type == Object.CUBE:
                        type_label = "1"
                    elif object_msg.object_type == Object.SPHERE:
                        type_label = "2"
                    elif object_msg.object_type == Object.PLUSHIE:
                        type_label = "3"
                    elif object_msg.object_type == Object.BOX:
                        type_label = "B"
                    else:
                        self.get_logger().warn(f"Skipping writing object due to unknown object type: {object_msg.object_type}")
                        continue
                    file.write(f"{type_label}, {object_msg.x*100:.2f}, {object_msg.y*100:.2f}, {object_msg.angle:.1f}\n")
            self.get_logger().info(f"Wrote {len(self.object_list)} objects to {filename}")
        except Exception as e:
            self.get_logger().error(f"Error writing map file (filename): {str(e)}")

def main():
    rclpy.init()
    node = ObjectFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
        node.write_map(node.map_file)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
