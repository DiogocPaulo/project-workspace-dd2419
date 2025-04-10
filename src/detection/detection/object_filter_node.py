#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from project_interfaces.msg import Object, ObjectList
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import numpy as np
import os
from mapping.map import Map

class ObjectFilterNode(Node):

    def __init__(self):
        super().__init__('object_filter')

        # Subscriber to raw detected objects
        self.create_subscription(ObjectList, "/raw_detected_objects", 
                              self.raw_objects_callback, 10)
        
        # Publisher for filtered objects
        self.filtered_object_publisher = self.create_publisher(ObjectList, "/detected_objects", 10)
        
        # TF broadcaster for visualization
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Timers
        self.create_timer(5.0, self.broadcast_object_list) 

        # Variables
        self.raw_objects = []
        self.filtered_objects = []
        self.initial_object_list = []  
        self.obstacles_map = None
        
        # File I/O setup
        self.maps_dir = os.path.join(os.getcwd(), "maps")
        if not os.path.exists(self.maps_dir):
            os.makedirs(self.maps_dir)
        self.map_file = os.path.join(self.maps_dir, "map.csv")
        
        # Load existing map on startup
        self.load_map()

    def raw_objects_callback(self, msg: ObjectList):
        for raw_obj in msg.objects:
            if self.workspace_map is not None:
                    is_in = self.workspace_map.is_free(raw_obj.x, raw_obj.y, 50)
            else:
                    is_in = False

            if is_in:
                initial_object_msg = Object()
                initial_object_msg.x = raw_obj.x
                initial_object_msg.y = raw_obj.y
                initial_object_msg.angle = raw_obj.angle
                initial_object_msg.object_type = raw_obj.object_type
                self.initial_object_list.append(initial_object_msg)

                initial_list_msg = ObjectList()
                initial_list_msg.header.frame_id = "odom"
                initial_list_msg.header.stamp = msg.stamp
                initial_list_msg.length = len(self.initial_object_list)
                initial_list_msg.objects = self.initial_object_list
                self.object_list_publisher.publish(initial_list_msg)

                # Check if the new object is a duplicate based on proximity
                is_duplicate = False
                for i, obj in enumerate(self.initial_object_list[:-1]):
                    distance = np.sqrt((raw_obj.x - obj.x)**2 + (raw_obj.y - obj.y)**2)
                    if raw_obj.object_type == "box" or obj.object_type == "box":
                        if distance < 0.20:    #f the object is within 1 cm of an existing object
                            is_duplicate = True
                            break
                    elif distance < 0.06:   # If the object is within 1 cm of an existing object
                        is_duplicate = True
                        break

                if self.obstacles_map is not None:
                    free_from_obstacles = self.obstacles_map.are_adjacent_cells_free(raw_obj.x, raw_obj.y, 1, 75)
                else:
                    free_from_obstacles = True

                #self.get_logger().info(f"initial:{self.initial_object_list}")

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
                    object_list_msg.header.frame_id = "odom"
                    object_list_msg.header.stamp = msg.stamp
                    object_list_msg.length = len(self.object_list)
                    object_list_msg.objects = self.object_list
                    self.object_list_publisher.publish(object_list_msg)

                    self.get_logger().info(f"Published new object list now includes: {raw_obj.object_type} at ({x_transformed:.2f}, {y_transformed:.2f})")

                 # Confidence-based correction
                CONFIDENCE_RADIUS = 0.06 # 5cm
                MIN_CONSISTENT = 2        # Need at least 2 consistent observations
                self.get_logger().info(f"obstacles:{self.object_list}")
                
                # Find all objects in this area
                nearby = []
                for obj in self.initial_object_list:
                    dist = np.sqrt((raw_obj.x - obj.x)**2 + (raw_obj.y - obj.y)**2)
                    if raw_obj.object_type == "box" or obj.object_type == "box":
                        if dist <= 0.20:
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
                                    if dist <= 0.20:
                                        self.object_list[i].object_type = "box"
                                        break
                                elif dist <= CONFIDENCE_RADIUS:
                                    # Update the type in the main object list
                                    self.object_list[i].object_type = most_common
                                    break

                        # Re-publish the corrected object list
                        object_list_msg = ObjectList()
                        object_list_msg.header.frame_id = "odom"
                        object_list_msg.header.stamp = msg.stamp
                        object_list_msg.length = len(self.object_list)
                        object_list_msg.objects = self.object_list
                        self.object_list_publisher.publish(object_list_msg)
                        
                        self.get_logger().info(f"final:{self.object_list}")
                        self.get_logger().info(f"Corrected object at ({raw_obj.x:.2f}, {raw_obj.y:.2f}) to {most_common}")
            
    def check_duplicates(self):
        duplicates_removed = 0
        n = len(self.initial_object_list)
        to_remove = set()  # Stores indices of objects to remove

        for i in range(n):
            if i in to_remove:
                continue  # Skip if already marked for removal
            
            obj1 = self.initial_object_list[i]
            
            for j in range(i + 1, n):
                if j in to_remove:
                    continue  # Skip if already marked
                
                obj2 = self.initial_object_list[j]
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
        self.filtered_objects = [obj for idx, obj in enumerate(self.initial_object_list) if idx not in to_remove]
        return

    def publish_object_list(self):
        """Same as your original implementation"""
        object_list_msg = ObjectList()
        object_list_msg.header.frame_id = "odom"
        object_list_msg.header.stamp = self.get_clock().now().to_msg()
        object_list_msg.length = len(self.filtered_objects)
        object_list_msg.objects = self.filtered_objects
        self.filtered_object_publisher.publish(object_list_msg)

    def broadcast_object_list(self):
        """Same as your original implementation"""
        self.check_duplicates()

        for i, object_msg in enumerate(self.filtered_objects):
            transform = TransformStamped()
            transform.header.frame_id = "odom"
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

            self.tf_broadcaster.sendTransform(transform)

    def load_map(self):
        """Load objects from map file on startup"""
        if not os.path.exists(self.map_file):
            self.get_logger().warn(f"Map file not found: {self.map_file}")
            return

        try:
            objects = []
            with open(self.map_file, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) < 4:
                        continue
                    
                    obj = Object()
                    
                    # Parse object type
                    if parts[0] == "1":
                        obj.object_type = Object.CUBE
                    elif parts[0] == "2":
                        obj.object_type = Object.SPHERE
                    elif parts[0] == "3":
                        obj.object_type = Object.PLUSHIE
                    elif parts[0] == "B":
                        obj.object_type = Object.BOX
                    else:
                        continue  # Skip unknown types
                    
                    # Parse coordinates (convert from cm back to meters)
                    obj.x = float(parts[1])/100
                    obj.y = float(parts[2])/100
                    obj.angle = float(parts[3])
                    
                    objects.append(obj)
            
            # Merge with current objects through the standard processing pipeline
            self.initial_object_list.extend(objects)
            self.publish_object_list()
            self.get_logger().info(f"Loaded {len(objects)} objects from {self.map_file}")
            
        except Exception as e:
            self.get_logger().error(f"Error loading map: {str(e)}")

    def save_map(self):
        """Save current objects to map file"""
        try:
            with open(self.map_file, 'w') as f:
                for obj in self.filtered_objects:
                    if obj.object_type == Object.CUBE:
                        type_label = "1"
                    elif obj.object_type == Object.SPHERE:
                        type_label = "2"
                    elif obj.object_type == Object.PLUSHIE:
                        type_label = "3"
                    elif obj.object_type == Object.BOX:
                        type_label = "B"
                    else:
                        continue
                    
                    f.write(f"{type_label},{obj.x*100:.2f},{obj.y*100:.2f},{obj.angle:.1f}\n")
            
            self.get_logger().info(f"Saved {len(self.filtered_objects)} objects to {self.map_file}")

            
        except Exception as e:
            self.get_logger().error(f"Error saving map: {str(e)}")



def main():
    rclpy.init()
    node = ObjectFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
        node.save_map()  # Save on shutdown
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()