#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo_detection.msg import BoundingBoxArray, BoundingBox
from ultralytics import YOLO
from scipy import ndimage
from sklearn.cluster import DBSCAN
import os

class EnhancedTomatoDetector:
    def __init__(self):
        rospy.init_node('enhanced_tomato_detector', anonymous=True)
        
        # Initialize YOLO model
        model_path = rospy.get_param('~model_path', 'yolov8n.pt')
        
        try:
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
        
        # Set camera properties for better quality
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # Higher resolution
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)
        self.cap.set(cv2.CAP_PROP_CONTRAST, 0.5)
        self.cap.set(cv2.CAP_PROP_SATURATION, 0.6)
        
        # ROS setup
        self.bridge = CvBridge()
        self.bbox_pub = rospy.Publisher('/yolo_detections', BoundingBoxArray, queue_size=10)
        self.image_pub = rospy.Publisher('/yolo_image', Image, queue_size=1)
        self.processed_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        
        # Enhanced detection parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.2)
        self.publish_image = rospy.get_param('~publish_image', True)
        self.show_camera_window = rospy.get_param('~show_camera_window', True)
        
        # Occlusion handling parameters
        self.use_color_enhancement = rospy.get_param('~use_color_enhancement', True)
        self.use_multi_scale = rospy.get_param('~use_multi_scale', True)
        self.use_partial_detection = rospy.get_param('~use_partial_detection', True)
        self.use_edge_enhancement = rospy.get_param('~use_edge_enhancement', True)
        self.use_temporal_smoothing = rospy.get_param('~use_temporal_smoothing', True)
        
        # Tomato-specific color ranges (HSV)
        self.red_lower1 = np.array([0, 50, 50])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 50, 50])
        self.red_upper2 = np.array([180, 255, 255])
        
        # Green tomato detection
        self.green_lower = np.array([40, 40, 40])
        self.green_upper = np.array([80, 255, 255])
        
        # Temporal smoothing for consistent detection
        self.previous_detections = []
        self.detection_history = []
        self.max_history = 5
        
        # Setup display windows
        if self.show_camera_window:
            cv2.namedWindow('Original', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Enhanced', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Color Mask', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Final Detection', cv2.WINDOW_AUTOSIZE)
        
        rospy.loginfo("Enhanced Tomato Detector initialized")
        
    def enhance_image_for_tomatoes(self, frame):
        """Apply multiple enhancement techniques to improve tomato visibility"""
        enhanced = frame.copy()
        
        # 1. Color space enhancement
        if self.use_color_enhancement:
            # Convert to LAB color space for better color separation
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            
            # Enhance the 'a' channel (green-red axis) to make tomatoes pop
            a = cv2.equalizeHist(a)
            enhanced_lab = cv2.merge([l, a, b])
            enhanced = cv2.cvtColor(enhanced_lab, cv2.COLOR_LAB2BGR)
            
            # Increase saturation for better red detection
            hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
            hsv[:, :, 1] = cv2.multiply(hsv[:, :, 1], 1.3)  # Increase saturation
            enhanced = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        
        # 2. Edge enhancement to handle partial occlusions
        if self.use_edge_enhancement:
            # Unsharp masking
            gaussian = cv2.GaussianBlur(enhanced, (5, 5), 2.0)
            enhanced = cv2.addWeighted(enhanced, 1.5, gaussian, -0.5, 0)
        
        return enhanced
    
    def create_tomato_mask(self, frame):
        """Create a mask highlighting tomato-colored regions"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create masks for red tomatoes (two ranges due to HSV wraparound)
        mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # Create mask for green tomatoes
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        
        # Combine masks
        tomato_mask = cv2.bitwise_or(red_mask, green_mask)
        
        # Clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        tomato_mask = cv2.morphologyEx(tomato_mask, cv2.MORPH_CLOSE, kernel)
        tomato_mask = cv2.morphologyEx(tomato_mask, cv2.MORPH_OPEN, kernel)
        
        return tomato_mask
    
    def detect_partial_tomatoes(self, frame, tomato_mask):
        """Detect partially occluded tomatoes using color information"""
        partial_detections = []
        
        # Find contours in the tomato mask
        contours, _ = cv2.findContours(tomato_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area for tomato
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate roundness (circularity)
                perimeter = cv2.arcLength(contour, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    
                    # Tomatoes should be somewhat circular
                    if circularity > 0.3:  # Relaxed threshold for partial occlusion
                        # Calculate aspect ratio
                        aspect_ratio = float(w) / h
                        if 0.5 < aspect_ratio < 2.0:  # Reasonable aspect ratio
                            partial_detections.append({
                                'bbox': [x, y, x + w, y + h],
                                'confidence': min(0.7, circularity * 2),  # Confidence based on circularity
                                'area': area,
                                'circularity': circularity
                            })
        
        return partial_detections
    
    def apply_temporal_smoothing(self, current_detections):
        """Smooth detections over time to reduce flickering"""
        if not self.use_temporal_smoothing:
            return current_detections
        
        # Add current detections to history
        self.detection_history.append(current_detections)
        if len(self.detection_history) > self.max_history:
            self.detection_history.pop(0)
        
        # If we have enough history, apply smoothing
        if len(self.detection_history) >= 3:
            smoothed_detections = []
            
            # Cluster detections across frames
            all_centers = []
            all_detections = []
            
            for frame_detections in self.detection_history:
                for det in frame_detections:
                    bbox = det['bbox']
                    center_x = (bbox[0] + bbox[2]) / 2
                    center_y = (bbox[1] + bbox[3]) / 2
                    all_centers.append([center_x, center_y])
                    all_detections.append(det)
            
            if len(all_centers) > 0:
                # Use DBSCAN to cluster nearby detections
                clustering = DBSCAN(eps=50, min_samples=2).fit(all_centers)
                
                # For each cluster, create a smoothed detection
                unique_labels = set(clustering.labels_)
                for label in unique_labels:
                    if label != -1:  # Ignore noise points
                        cluster_detections = [all_detections[i] for i in range(len(all_detections)) 
                                            if clustering.labels_[i] == label]
                        
                        if len(cluster_detections) >= 2:  # Stable detection
                            # Average the bounding boxes
                            avg_bbox = np.mean([det['bbox'] for det in cluster_detections], axis=0)
                            avg_confidence = np.mean([det['confidence'] for det in cluster_detections])
                            
                            smoothed_detections.append({
                                'bbox': avg_bbox.tolist(),
                                'confidence': min(avg_confidence * 1.2, 1.0),  # Boost confidence for stable detections
                                'area': np.mean([det['area'] for det in cluster_detections]),
                                'circularity': np.mean([det.get('circularity', 0.5) for det in cluster_detections])
                            })
            
            return smoothed_detections
        
        return current_detections
    
    def merge_yolo_and_color_detections(self, yolo_detections, color_detections):
        """Merge YOLO detections with color-based detections"""
        merged_detections = []
        
        # Add YOLO detections (higher priority)
        for det in yolo_detections:
            merged_detections.append({
                'bbox': det['bbox'],
                'confidence': det['confidence'],
                'class_name': det['class_name'],
                'source': 'yolo'
            })
        
        # Add color detections that don't overlap significantly with YOLO detections
        for color_det in color_detections:
            overlap = False
            color_bbox = color_det['bbox']
            
            for yolo_det in yolo_detections:
                yolo_bbox = yolo_det['bbox']
                
                # Calculate IoU (Intersection over Union)
                iou = self.calculate_iou(color_bbox, yolo_bbox)
                if iou > 0.3:  # Significant overlap
                    overlap = True
                    break
            
            if not overlap:
                merged_detections.append({
                    'bbox': color_bbox,
                    'confidence': color_det['confidence'],
                    'class_name': 'tomato',
                    'source': 'color'
                })
        
        return merged_detections
    
    def calculate_iou(self, bbox1, bbox2):
        """Calculate Intersection over Union of two bounding boxes"""
        x1_min, y1_min, x1_max, y1_max = bbox1
        x2_min, y2_min, x2_max, y2_max = bbox2
        
        # Calculate intersection
        inter_x_min = max(x1_min, x2_min)
        inter_y_min = max(y1_min, y2_min)
        inter_x_max = min(x1_max, x2_max)
        inter_y_max = min(y1_max, y2_max)
        
        if inter_x_max <= inter_x_min or inter_y_max <= inter_y_min:
            return 0.0
        
        inter_area = (inter_x_max - inter_x_min) * (inter_y_max - inter_y_min)
        
        # Calculate union
        area1 = (x1_max - x1_min) * (y1_max - y1_min)
        area2 = (x2_max - x2_min) * (y2_max - y2_min)
        union_area = area1 + area2 - inter_area
        
        return inter_area / union_area if union_area > 0 else 0.0
    
    def detect_and_publish(self):
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame")
                continue
            
            original_frame = frame.copy()
            
            # 1. Enhance image for better tomato detection
            enhanced_frame = self.enhance_image_for_tomatoes(frame)
            
            # 2. Create tomato color mask
            tomato_mask = self.create_tomato_mask(enhanced_frame)
            
            # 3. Run YOLO detection on enhanced frame
            yolo_results = self.model(enhanced_frame, conf=self.confidence_threshold, verbose=False)
            
            # Process YOLO detections
            yolo_detections = []
            for result in yolo_results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])
                        class_name = self.model.names[class_id]
                        
                        # Focus on fruit-like objects
                        if class_name in ['apple', 'orange', 'banana'] or confidence > 0.4:
                            coords = box.xyxy[0].tolist()
                            yolo_detections.append({
                                'bbox': coords,
                                'confidence': confidence,
                                'class_name': class_name,
                                'class_id': class_id
                            })
            
            # 4. Detect partial tomatoes using color information
            if self.use_partial_detection:
                color_detections = self.detect_partial_tomatoes(enhanced_frame, tomato_mask)
                color_detections = self.apply_temporal_smoothing(color_detections)
            else:
                color_detections = []
            
            # 5. Merge YOLO and color-based detections
            all_detections = self.merge_yolo_and_color_detections(yolo_detections, color_detections)
            
            # 6. Create ROS message
            bbox_array = BoundingBoxArray()
            bbox_array.header.stamp = rospy.Time.now()
            bbox_array.header.frame_id = "camera_frame"
            
            # Convert to ROS message format
            for det in all_detections:
                bbox = BoundingBox()
                bbox.class_id = det.get('class_id', 999)  # Use 999 for color detections
                bbox.class_name = det['class_name']
                bbox.confidence = det['confidence']
                bbox.x_min = det['bbox'][0]
                bbox.y_min = det['bbox'][1]
                bbox.x_max = det['bbox'][2]
                bbox.y_max = det['bbox'][3]
                bbox_array.bounding_boxes.append(bbox)
            
            # 7. Visualize results
            result_frame = original_frame.copy()
            for det in all_detections:
                bbox = det['bbox']
                confidence = det['confidence']
                class_name = det['class_name']
                source = det.get('source', 'unknown')
                
                # Color based on detection source
                if source == 'yolo':
                    color = (0, 255, 0)  # Green for YOLO
                else:
                    color = (255, 0, 0)  # Blue for color-based
                
                # Draw bounding box
                cv2.rectangle(result_frame, 
                            (int(bbox[0]), int(bbox[1])), 
                            (int(bbox[2]), int(bbox[3])), 
                            color, 2)
                
                # Draw label
                label = f"{class_name}: {confidence:.2f} ({source})"
                cv2.putText(result_frame, label,
                          (int(bbox[0]), int(bbox[1]) - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # 8. Display results
            if self.show_camera_window:
                cv2.imshow('Original', original_frame)
                cv2.imshow('Enhanced', enhanced_frame)
                cv2.imshow('Color Mask', tomato_mask)
                cv2.imshow('Final Detection', result_frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.signal_shutdown("User requested shutdown")
                    break
            
            # 9. Publish results
            self.bbox_pub.publish(bbox_array)
            
            if self.publish_image:
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(result_frame, "bgr8")
                    image_msg.header = bbox_array.header
                    self.image_pub.publish(image_msg)
                    
                    # Publish processed image
                    processed_msg = self.bridge.cv2_to_imgmsg(enhanced_frame, "bgr8")
                    processed_msg.header = bbox_array.header
                    self.processed_pub.publish(processed_msg)
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
        detector = EnhancedTomatoDetector()
        detector.detect_and_publish()
    except rospy.ROSInterruptException:
        pass