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
from std_msgs.msg import Header

class ArmCamera(Node):
    def __init__(self):
        super().__init__("multi_servo_publisher")

        self.model = YOLO("runs/detect/train5/weights/best.pt")

        self.subscription = self.create_subscription(
            Image, '/arm_camera/image_raw', self.image_callback, 10)
        

        self.publisher = self.create_publisher(Image, '/yolov8/detections', 10)

        self.publisher_detected = self.create_publisher(DetectedDataArray, '/yolov8/detections_data', 10)

        self.bridge = CvBridge()

        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model(cv_image)

        
        class_names = ['Box','objects']  

        detection_msg = DetectedDataArray()

        
        for result in results[0].boxes: 
            x1, y1, x2, y2 = map(int, result.xyxy[0])
            confidence = float(result.conf[0])
            class_idx = int(result.cls[0])

            label = class_names[class_idx] if class_idx < len(class_names) else "Unknown"

            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            detection = DetectedData()
            detection.label = label
            detection.confidence = confidence
            detection.xmin = x1
            detection.ymin = y1
            detection.xmax = x2
            detection.ymax = y2
            detection.timestamp = msg.header.stamp

            detection_msg.detections.append(detection)


        
        height, width, _ = cv_image.shape
        center_x, center_y = width // 2, height // 2

        cv2.line(cv_image, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)
        cv2.line(cv_image, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)
        cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")


        self.publisher.publish(ros_image)

        self.publisher_detected.publish(detection_msg)


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
    node = FrameExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()

