import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import os
import onnx
from project_interfaces.msg import DetectedData, DetectedDataArray
from project_interfaces.srv import GetDetectedList
from project_interfaces.srv import PickObject
from std_msgs.msg import Header
import rclpy.time
import math
import numpy as np


class ArmCamera(Node):
    def __init__(self):
        super().__init__("arm_camera_detector")

        self.model = YOLO("runs/detect/train2/weights/best.pt")

        self.srv = self.create_service(GetDetectedList, 'get_detected_list', self.get_detected_callback)

        # self.service = self.create_service(PickObject, 'PickObject_test', self.test_debug_callback)

        

        self.publisher = self.create_publisher(Image, '/yolov8/detections', 10)

        self.handle_camera = False

        self.publisher_detected = self.create_publisher(DetectedDataArray, '/yolov8/detections_data', 10)

        self.bridge = CvBridge()

        self.detected_objects = []
        self.detected_boxes = []
        self.clock = self.get_clock()

        self.subscription = self.create_subscription(
            Image, '/arm_camera/image_raw', self.image_callback, 10)
        
        
        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model(cv_image)

        
        class_names = ['Box','objects']


        found_objects = False
        found_boxes = False

        # Center crosshairs
        height, width, _ = cv_image.shape
        center_x_screen, center_y_screen = width // 2, height // 2

        
        for result in results[0].boxes: 
            x1, y1, x2, y2 = map(int, result.xyxy[0])
            confidence = float(result.conf[0])
            class_idx = int(result.cls[0])

            label = class_names[class_idx] if class_idx < len(class_names) else "Unknown"

            # Draw rectangle around detected box
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw label text
            cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Calculate center of the box
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # Draw green dot at the center of the box
            cv2.circle(cv_image, (center_x, center_y), 5, (0, 255, 0), -1)

            entity = DetectedData()
            entity.label = label
            entity.confidence = confidence
            entity.center_x = center_x
            entity.center_y = center_y
            entity.diff_x = center_x_screen - center_x
            entity.diff_y = center_y_screen - center_y
            entity.distance = int(self.compute_distance(center_x,center_y,center_x_screen,center_y_screen))
            entity.timestamp=msg.header.stamp

            if entity.label == 'objects':
                if not found_objects:
                    self.detected_objects = []
                    found_objects = True
                self.detected_objects.append(entity)
            elif entity.label == 'Box':
                if not found_boxes:
                    self.detected_boxes = []
                    found_objects = True
                self.detected_boxes.append(entity)


        cv2.line(cv_image, (center_x_screen - 20, center_y_screen), (center_x_screen + 20, center_y_screen), (0, 0, 255), 2)
        cv2.line(cv_image, (center_x_screen, center_y_screen - 20), (center_x_screen, center_y_screen + 20), (0, 0, 255), 2)
        cv2.circle(cv_image, (center_x_screen, center_y_screen), 5, (0, 0, 255), -1)

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        self.publisher.publish(ros_image)

    def compute_distance(self,x1,y1,x2,y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # def test_debug_callback(self, request, response):
    #     self.get_logger().info(f"REQUEST RECIEVED")

    #     response.result = 0

    #     self.get_logger().info(f"REQUEST HANDLED")

    #     return response
    
    def get_detected_callback(self, request, response):
        self.get_logger().info(f"REQUEST RECIEVED")

        response.objects = self.detected_objects
        response.boxes = self.detected_boxes

        self.get_logger().info(f"REQUEST HANDLED")

        return response
    
def main():
    rclpy.init()
    node = ArmCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
