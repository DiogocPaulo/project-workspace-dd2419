#!/home/robot/yolo_venv/bin/python3
import rclpy
from rclpy.node import Node
# import sys
# sys.path.insert(0, "home/robot/yolo_venv/lib/python3.12.3/site-packages")
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

class detected_entity:
    def __init__(self,label,x,y,timestamp):
        self.x = x
        self.y = y
        self.label = label
        self.timestamp = timestamp

class ArmCamera(Node):
    def __init__(self):
        super().__init__("Arm_Camera_Detector")

        self.model = YOLO("runs/detect/train2/weights/best.pt")

        self.srv = self.create_service(GetDetectedList, 'get_detected_list', self.get_detected_callback)

        self.service = self.create_service(PickObject, 'PickObject_test', self.test_debug_callback)

        

        self.publisher = self.create_publisher(Image, '/yolov8/detections', 10)

        self.handle_camera = False

        self.publisher_detected = self.create_publisher(DetectedDataArray, '/yolov8/detections_data', 10)

        self.bridge = CvBridge()

        self.detected_objects = []
        self.detected_boxes = []
        self.clock = self.get_clock()

        # self.subscription = self.create_subscription(
        #     Image, '/arm_camera/image_raw', self.image_callback, 10)
        
        
        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # results = self.model(cv_image)

        
        class_names = ['Box','objects']


        found_objects = False
        found_boxes = False


        
        # for result in results[0].boxes: 
        #     x1, y1, x2, y2 = map(int, result.xyxy[0])
        #     confidence = float(result.conf[0])
        #     class_idx = int(result.cls[0])

        #     label = class_names[class_idx] if class_idx < len(class_names) else "Unknown"

        #     # Draw rectangle around detected box
        #     cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        #     # Draw label text
        #     cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        #     # Calculate center of the box
        #     center_x = (x1 + x2) // 2
        #     center_y = (y1 + y2) // 2

        #     # Draw green dot at the center of the box
        #     cv2.circle(cv_image, (center_x, center_y), 5, (0, 255, 0), -1)

        # #     entity = DetectedData()
        # #     entity.confidence = confidence
        # #     entity.xmin=x1
        # #     entity.xmax=x2
        # #     entity.ymin=y1
        # #     entity.ymax=y2
        # #     entity.timestamp=msg.header.stamp

        #     if entity.label == 'objects':
        #         if not found_objects:
        #             self.detected_objects = []
        #             found_objects = True
        #         self.detected_objects.append(entity)
        #     elif entity.label == 'Box':
        #         if not found_boxes:
        #             self.detected_boxes = []
        #             found_objects = True
        #         self.detected_boxes.append(entity)


        # Center crosshairs
        height, width, _ = cv_image.shape
        center_x, center_y = width // 2, height // 2

        cv2.line(cv_image, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)
        cv2.line(cv_image, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)
        cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        self.publisher.publish(ros_image)

    def test_debug_callback(self, request, response):
        self.get_logger().info(f"REQUEST RECIEVED")

        response.result = 0

        self.get_logger().info(f"REQUEST HANDLED")

        return response
    
    def get_detected_callback(self, request, response):
        self.get_logger().info(f"REQUEST RECIEVED")

        objects = []
        entity = DetectedData()
        entity.label = 'objects'
        entity.confidence = 0.5
        entity.xmin=1
        entity.xmax=2
        entity.ymin=1
        entity.ymax=2
        entity.timestamp=self.get_clock().now().to_msg()
        objects = []
        objects.append(entity)

        # boxes = []
        # entity = DetectedData()
        # entity.label = 'box'
        # entity.confidence = 0.5
        # entity.xmin=1
        # entity.xmax=2
        # entity.ymin=1
        # entity.ymax=2
        # entity.timestamp=self.get_clock().now().to_msg()
        # boxes = []
        # boxes.append(entity)


        # response.objects = objects
        # response.boxes = boxes

        response.object = entity

        self.get_logger().info(f"REQUEST HANDLED")

        return response



class FrameExtractor(Node):
    def __init__(self):
        super().__init__("FrameExtractor")

        self.model = YOLO("yolov8n.pt")

        self.subscription = self.create_subscription(
            Image, '/arm_camera/image_raw', self.image_callback, 10)
        
        self.image_count = 0

        self.bridge = CvBridge()

        self.output_dir = "ros_bag_images"
        os.makedirs(self.output_dir, exist_ok=True)

        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_path = os.path.join(self.output_dir, f"frame_{self.image_count}.jpg")
        cv2.imwrite(image_path, cv_image)
        self.get_logger().info(f"Saved {image_path}")
        self.image_count += 1

class YOLOTrainNode(Node):
    def __init__(self):
        super().__init__("Trainer")

        self.model = YOLO("yolov8n.pt")

        self.model.train(data="/home/robot/project-workspace-dd2419/datasets/Image Labeling.v2i.yolov8/data.yaml", epochs=10, imgsz=640)


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

