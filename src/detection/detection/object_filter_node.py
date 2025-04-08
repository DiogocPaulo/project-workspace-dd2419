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
        
        # Timer for periodic publishing and TF broadcasting
        self.create_timer(2.0, self.publish_filtered_objects)
        self.create_timer(5.0, self.broadcast_object_transforms)
        
        # Variables
        self.raw_objects = []
        self.filtered_objects = []
        self.obstacles_map = None
        
        self.get_logger().info("Object Filter Node initialized")

    def raw_objects_callback(self, msg: ObjectList):
        self.raw_objects = msg.objects
        self.filter_objects()

    def filter_objects(self):
        # Confidence-based filtering
        CONFIDENCE_RADIUS = 0.06
        MIN_CONSISTENT = 2
        
        # First pass - find consistent objects
        consistent_objects = []
        
        for obj in self.raw_objects:
            # Find nearby objects
            nearby = []
            for other in self.raw_objects:
                dist = np.sqrt((obj.x - other.x)**2 + (obj.y - other.y)**2)
                if obj.object_type == "box" or other.object_type == "box":
                    if dist <= 0.20:
                        nearby.append(other)
                elif dist <= CONFIDENCE_RADIUS:
                    nearby.append(other)
            
            # Count object types in this area
            type_counts = {}
            for nearby_obj in nearby:
                type_counts[nearby_obj.object_type] = type_counts.get(nearby_obj.object_type, 0) + 1
            
            # If we have enough consistent observations, keep this object
            if len(nearby) >= MIN_CONSISTENT:
                most_common, count = max(type_counts.items(), key=lambda x: x[1])
                if count >= MIN_CONSISTENT:
                    # Update the object type to the most common one in the area
                    obj.object_type = most_common
                    consistent_objects.append(obj)
        
        # Second pass - remove duplicates
        self.filtered_objects = []
        n = len(consistent_objects)
        to_remove = set()

        for i in range(n):
            if i in to_remove:
                continue
            
            obj1 = consistent_objects[i]
            self.filtered_objects.append(obj1)
            
            for j in range(i + 1, n):
                if j in to_remove:
                    continue
                
                obj2 = consistent_objects[j]
                dx = obj1.x - obj2.x
                dy = obj1.y - obj2.y
                dist = np.sqrt(dx**2 + dy**2)

                # Determine threshold based on types
                if obj1.object_type == "box" or obj2.object_type == "box":
                    threshold = 0.18
                elif obj1.object_type == obj2.object_type: 
                    threshold = 0.15
                else:
                    threshold = 0.06

                # If too close, mark as duplicate
                if dist < threshold:
                    to_remove.add(j)

    def publish_filtered_objects(self):
        object_list_msg = ObjectList()
        object_list_msg.header.frame_id = "odom"
        object_list_msg.header.stamp = self.get_clock().now().to_msg()
        object_list_msg.length = len(self.filtered_objects)
        object_list_msg.objects = self.filtered_objects
        self.filtered_object_publisher.publish(object_list_msg)

    def broadcast_object_transforms(self):
        for i, obj in enumerate(self.filtered_objects):
            transform = TransformStamped()
            transform.header.frame_id = "odom"
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.child_frame_id = f"{obj.object_type}_{i}"
            
            transform.transform.translation.x = obj.x
            transform.transform.translation.y = obj.y
            transform.transform.translation.z = 0.0
            
            q = Quaternion()
            q.z = np.sin(obj.angle / 2.0)
            q.w = np.cos(obj.angle / 2.0)
            transform.transform.rotation = q

            self.tf_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    node = ObjectFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()