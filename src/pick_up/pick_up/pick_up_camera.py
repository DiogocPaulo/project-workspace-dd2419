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


class ArmCamera(Node):
    def __init__(self):
        super().__init__("multi_servo_publisher")

        self.model = YOLO("yolov8n.pt")

        self.subscription = self.create_subscription(
            Image, '/arm_camera/image_raw', self.image_callback, 10)
        

        self.publisher = self.create_publisher(Image, '/yolov8/detections', 10)

        self.bridge = CvBridge()

        
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model(cv_image)

        
        class_names = ['cube', 'sphere', 'plushie']  

        
        for result in results[0].boxes: 
            x1, y1, x2, y2 = map(int, result.xyxy[0])
            confidence = result.conf[0]
            class_idx = int(result.cls[0])

            label = class_names[class_idx] if class_idx < len(class_names) else "Unknown"

            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        self.publisher.publish(ros_image)


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

