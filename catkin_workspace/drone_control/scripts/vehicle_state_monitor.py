#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
from ultralytics import YOLO
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D

# Load the YOLOv8 model
try:
    model = YOLO("yolov8l.pt")  # Load YOLOv8 model
    if torch.cuda.is_available():
        model = model.to("cuda")  # Move to GPU if available
except Exception as e:
    rospy.logerr(f"Failed to load the YOLOv8 model: {e}")
    exit(1)

class ObjectDetectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/processed_image", Image, queue_size=10)
        self.classification_pub = rospy.Publisher("object_classification", String, queue_size=10)
        self.detection_pub = rospy.Publisher("/yolo/truck_bbox", Detection2DArray, queue_size=10)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Perform object detection
            results = model.predict(cv_image, imgsz=640, conf=0.5)

            detection_array_msg = Detection2DArray()
            detection_array_msg.header = data.header
            detected_objects = []

            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = model.names[cls]
                detected_objects.append(f"{label} ({conf:.2f})")

                # Create Detection2D msg
                detection = Detection2D()
                detection.bbox.center.x = (x1 + x2) / 2.0
                detection.bbox.center.y = (y1 + y2) / 2.0
                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.id = cls
                hypothesis.score = conf
                detection.results.append(hypothesis)

                # Only publish trucks (you can remove this condition to send all detections)
                if label == "truck":
                    detection_array_msg.detections.append(detection)

                # Draw bounding box and label on the image
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{label} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # Publish processed image
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_pub.publish(processed_image_msg)

            # Publish classification labels
            classification_msg = String()
            classification_msg.data = ", ".join(detected_objects)
            self.classification_pub.publish(classification_msg)

            # Publish truck detections
            self.detection_pub.publish(detection_array_msg)

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error in image processing: {e}")

if __name__ == '__main__':
    rospy.init_node('yolov8_detection_node')
    node = ObjectDetectionNode()
    rospy.spin()
