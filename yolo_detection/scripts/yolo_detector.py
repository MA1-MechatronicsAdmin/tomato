#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo_detection.msg import BoundingBoxArray, BoundingBox
from ultralytics import YOLO
import os

class YOLODetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)
        
        # Initialize YOLO model
        model_path = rospy.get_param('~model_path', 'yolov8n.pt')  # Use YOLOv8n instead
        
        try:
            # This will automatically download YOLOv8n if it doesn't exist
            self.model = YOLO(model_path)
            rospy.loginfo(f"YOLO model loaded: {model_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            return
        
        # Camera setup
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            rospy.logerr(f"Failed to open camera {self.camera_id}")
            return
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # ROS setup
        self.bridge = CvBridge()
        self.bbox_pub = rospy.Publisher('/yolo_detections', BoundingBoxArray, queue_size=10)
        self.image_pub = rospy.Publisher('/yolo_image', Image, queue_size=1)
        
        # Detection parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.3)
        self.publish_image = rospy.get_param('~publish_image', True)
        self.show_camera_window = rospy.get_param('~show_camera_window', True)
        
        # Advanced detection parameters
        self.use_multiple_scales = rospy.get_param('~use_multiple_scales', True)
        self.use_augmentation = rospy.get_param('~use_augmentation', True)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.5)  # Non-max suppression
        self.min_detection_size = rospy.get_param('~min_detection_size', 20)  # Minimum bounding box size
        
        # COCO classes that might work for tomatoes/fruits
        self.fruit_classes = ['apple', 'orange', 'banana', 'carrot', 'broccoli', 'pizza', 'donut', 'cake']
        self.target_classes = rospy.get_param('~target_classes', self.fruit_classes)
        
        # Setup camera display window
        if self.show_camera_window:
            cv2.namedWindow('YOLO Detection', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Processed Image', cv2.WINDOW_AUTOSIZE)
            rospy.loginfo("Camera display windows created - Press 'q' to close")
        
        rospy.loginfo("YOLO Detector initialized successfully")
        rospy.loginfo(f"Available COCO classes: {list(self.model.names.values())}")
        rospy.loginfo(f"Looking for fruit-like classes: {self.fruit_classes}")
        
    def detect_and_publish(self):
        rate = rospy.Rate(30)  # 30 FPS
        
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame")
                continue
            
            # Run YOLO detection with lower confidence for fruits
            results = self.model(frame, conf=max(0.1, self.confidence_threshold * 0.5), verbose=False)
            
            # Create bounding box array message
            bbox_array = BoundingBoxArray()
            bbox_array.header.stamp = rospy.Time.now()
            bbox_array.header.frame_id = "camera_frame"
            
            # Process detections
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])
                        class_name = self.model.names[class_id]
                        
                        # Log all detections for debugging
                        if confidence >= 0.1:  # Very low threshold for debugging
                            rospy.loginfo(f"Detected: {class_name} with confidence {confidence:.3f}")
                        
                        if confidence >= self.confidence_threshold:
                            # Create bounding box message
                            bbox = BoundingBox()
                            bbox.class_id = class_id
                            bbox.class_name = class_name
                            bbox.confidence = confidence
                            
                            # Get coordinates (xyxy format)
                            coords = box.xyxy[0].tolist()
                            bbox.x_min = coords[0]
                            bbox.y_min = coords[1]
                            bbox.x_max = coords[2]
                            bbox.y_max = coords[3]
                            
                            bbox_array.bounding_boxes.append(bbox)
                            
                            # Choose color based on class type
                            color = (0, 255, 0)  # Green for general objects
                            if class_name in self.fruit_classes:
                                color = (0, 0, 255)  # Red for fruits
                            elif class_name in ['apple', 'orange']:
                                color = (255, 0, 0)  # Blue for apples/oranges
                            
                            # Draw bounding box on frame for visualization
                            if self.publish_image:
                                cv2.rectangle(frame, 
                                            (int(coords[0]), int(coords[1])), 
                                            (int(coords[2]), int(coords[3])), 
                                            color, 2)
                                cv2.putText(frame, 
                                          f"{bbox.class_name}: {confidence:.2f}",
                                          (int(coords[0]), int(coords[1]) - 10),
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Publish bounding boxes
            self.bbox_pub.publish(bbox_array)
            
            # Display camera window
            if self.show_camera_window:
                # Add detection count to the frame
                detection_count = len(bbox_array.bounding_boxes)
                cv2.putText(frame, f"Detections: {detection_count}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(frame, "Press 'q' to quit", 
                           (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow('YOLO Detection', frame)
                
                # Check for 'q' key press to quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.loginfo("Quit key pressed - shutting down")
                    rospy.signal_shutdown("User requested shutdown")
                    break
            
            # Publish annotated image
            if self.publish_image:
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    image_msg.header = bbox_array.header
                    self.image_pub.publish(image_msg)
                except Exception as e:
                    rospy.logwarn(f"Failed to publish image: {e}")
            
            rate.sleep()
    
    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'show_camera_window') and self.show_camera_window:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = YOLODetector()
        detector.detect_and_publish()
    except rospy.ROSInterruptException:
        pass