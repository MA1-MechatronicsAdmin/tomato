#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo_detection.msg import BoundingBoxArray, BoundingBox
from ultralytics import YOLO
import os
from scipy import ndimage

class EnhancedTomatoDetector:
    def __init__(self):
        rospy.init_node('enhanced_tomato_detector', anonymous=True)
        
        # Enhanced model loading
        model_path = rospy.get_param('~model_path', 'yolov8m.pt')  # Medium model for better accuracy
        custom_model_path = rospy.get_param('~custom_model_path', None)
        
        if custom_model_path and os.path.exists(custom_model_path):
            model_path = custom_model_path
            rospy.loginfo(f"Using custom tomato model: {model_path}")
        
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
            rospy.signal_shutdown("Camera initialization failed")
            return
        
        # Enhanced camera settings for better tomato detection
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)   # Higher resolution
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 15)             # Lower FPS for processing time
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 50)      # Adjust for greenhouse lighting
        self.cap.set(cv2.CAP_PROP_CONTRAST, 50)
        self.cap.set(cv2.CAP_PROP_SATURATION, 60)      # Enhance red detection
        
        # ROS setup
        self.bridge = CvBridge()
        self.bbox_pub = rospy.Publisher('/tomato_detections', BoundingBoxArray, queue_size=10)
        self.image_pub = rospy.Publisher('/tomato_image', Image, queue_size=1)
        self.processed_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        
        # Enhanced detection parameters for occluded tomatoes
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.15)  # Lower for partial objects
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.3)  # Lower to keep overlapping detections
        self.min_detection_size = rospy.get_param('~min_detection_size', 10)
        
        # Tomato-specific parameters
        self.use_color_filtering = rospy.get_param('~use_color_filtering', True)
        self.use_multi_scale = rospy.get_param('~use_multi_scale', True)
        self.use_augmentation = rospy.get_param('~use_augmentation', True)
        self.merge_overlapping = rospy.get_param('~merge_overlapping', True)
        
        # Color ranges for tomato detection (HSV)
        self.tomato_color_lower = np.array([0, 50, 50])    # Red lower bound
        self.tomato_color_upper = np.array([10, 255, 255]) # Red upper bound
        self.tomato_color_lower2 = np.array([170, 50, 50]) # Red wraparound
        self.tomato_color_upper2 = np.array([180, 255, 255])
        
        # Green color for filtering out stems/leaves
        self.green_lower = np.array([40, 40, 40])
        self.green_upper = np.array([80, 255, 255])
        
        # Display settings
        self.show_camera_window = rospy.get_param('~show_camera_window', True)
        if self.show_camera_window:
            cv2.namedWindow('Tomato Detection', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Color Filtered', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow('Processing Steps', cv2.WINDOW_AUTOSIZE)
        
        rospy.loginfo("Enhanced Tomato Detector initialized")
        rospy.loginfo(f"Detection confidence threshold: {self.confidence_threshold}")
        rospy.loginfo(f"Using color filtering: {self.use_color_filtering}")
        
    def preprocess_image(self, image):
        """Enhanced preprocessing for better tomato detection"""
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask for red colors (tomatoes)
        mask1 = cv2.inRange(hsv, self.tomato_color_lower, self.tomato_color_upper)
        mask2 = cv2.inRange(hsv, self.tomato_color_lower2, self.tomato_color_upper2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # Create mask for green colors (stems/leaves) to identify occlusion
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        
        # Enhance red regions
        enhanced_image = image.copy()
        if self.use_color_filtering:
            # Apply morphological operations to clean up the mask
            kernel = np.ones((3,3), np.uint8)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
            
            # Enhance red regions in the original image
            enhanced_image[:,:,2] = np.where(red_mask > 0, 
                                           np.minimum(enhanced_image[:,:,2] * 1.3, 255),
                                           enhanced_image[:,:,2])
        
        return enhanced_image, red_mask, green_mask
    
    def detect_with_multiple_scales(self, image):
        """Run detection at multiple scales to catch partially occluded tomatoes"""
        all_detections = []
        
        # Original scale
        results = self.model(image, conf=self.confidence_threshold, iou=self.iou_threshold, verbose=False)
        all_detections.extend(self.extract_detections(results))
        
        if self.use_multi_scale:
            # Larger scale (zoom in to catch small tomatoes)
            height, width = image.shape[:2]
            
            # Create overlapping crops for multi-scale detection
            crop_size = min(width, height) // 2
            overlap = crop_size // 4
            
            for y in range(0, height - crop_size, crop_size - overlap):
                for x in range(0, width - crop_size, crop_size - overlap):
                    if y + crop_size > height or x + crop_size > width:
                        continue
                        
                    crop = image[y:y+crop_size, x:x+crop_size]
                    crop_results = self.model(crop, conf=self.confidence_threshold * 0.8, 
                                            iou=self.iou_threshold, verbose=False)
                    
                    # Adjust coordinates back to original image
                    crop_detections = self.extract_detections(crop_results)
                    for det in crop_detections:
                        det['x_min'] += x
                        det['y_min'] += y
                        det['x_max'] += x
                        det['y_max'] += y
                    
                    all_detections.extend(crop_detections)
        
        return all_detections
    
    def extract_detections(self, results):
        """Extract detection information from YOLO results"""
        detections = []
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = self.model.names[class_id]
                    
                    # Focus on objects that could be tomatoes
                    tomato_keywords = ['apple', 'orange', 'banana', 'carrot', 'donut', 'sports ball']
                    if class_name in tomato_keywords or 'tomato' in class_name.lower():
                        coords = box.xyxy[0].tolist()
                        
                        detection = {
                            'class_id': class_id,
                            'class_name': class_name,
                            'confidence': confidence,
                            'x_min': coords[0],
                            'y_min': coords[1],
                            'x_max': coords[2],
                            'y_max': coords[3]
                        }
                        detections.append(detection)
        
        return detections
    
    def filter_detections_by_color(self, detections, red_mask, image):
        """Filter detections based on color information"""
        filtered_detections = []
        
        for det in detections:
            x1, y1, x2, y2 = int(det['x_min']), int(det['y_min']), int(det['x_max']), int(det['y_max'])
            
            # Extract the region of interest
            roi_mask = red_mask[y1:y2, x1:x2]
            
            if roi_mask.size == 0:
                continue
                
            # Calculate the percentage of red pixels in the detection
            red_percentage = np.sum(roi_mask > 0) / roi_mask.size
            
            # Keep detections with significant red content
            if red_percentage > 0.1:  # At least 10% red pixels
                det['red_percentage'] = red_percentage
                filtered_detections.append(det)
        
        return filtered_detections
    
    def merge_overlapping_detections(self, detections):
        """Merge overlapping detections that might be the same tomato"""
        if not self.merge_overlapping or len(detections) < 2:
            return detections
        
        merged = []
        used = set()
        
        for i, det1 in enumerate(detections):
            if i in used:
                continue
                
            # Find overlapping detections
            overlapping = [det1]
            for j, det2 in enumerate(detections[i+1:], i+1):
                if j in used:
                    continue
                    
                # Calculate IoU
                iou = self.calculate_iou(det1, det2)
                if iou > 0.3:  # Overlapping threshold
                    overlapping.append(det2)
                    used.add(j)
            
            # Merge overlapping detections
            if len(overlapping) > 1:
                merged_det = self.merge_detections(overlapping)
                merged.append(merged_det)
            else:
                merged.append(det1)
                
            used.add(i)
        
        return merged
    
    def calculate_iou(self, det1, det2):
        """Calculate Intersection over Union"""
        x1 = max(det1['x_min'], det2['x_min'])
        y1 = max(det1['y_min'], det2['y_min'])
        x2 = min(det1['x_max'], det2['x_max'])
        y2 = min(det1['y_max'], det2['y_max'])
        
        if x2 <= x1 or y2 <= y1:
            return 0
            
        intersection = (x2 - x1) * (y2 - y1)
        area1 = (det1['x_max'] - det1['x_min']) * (det1['y_max'] - det1['y_min'])
        area2 = (det2['x_max'] - det2['x_min']) * (det2['y_max'] - det2['y_min'])
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0
    
    def merge_detections(self, detections):
        """Merge multiple detections into one"""
        # Use the detection with highest confidence as base
        best_det = max(detections, key=lambda x: x['confidence'])
        
        # Expand bounding box to encompass all detections
        min_x = min(det['x_min'] for det in detections)
        min_y = min(det['y_min'] for det in detections)
        max_x = max(det['x_max'] for det in detections)
        max_y = max(det['y_max'] for det in detections)
        
        merged = best_det.copy()
        merged['x_min'] = min_x
        merged['y_min'] = min_y
        merged['x_max'] = max_x
        merged['y_max'] = max_y
        merged['confidence'] = max(det['confidence'] for det in detections)
        
        return merged
    
    def detect_and_publish(self):
        rate = rospy.Rate(10)  # Lower rate for processing intensive operations
        
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame")
                continue
            
            # Preprocess image for better tomato detection
            enhanced_frame, red_mask, green_mask = self.preprocess_image(frame)
            
            # Run multi-scale detection
            all_detections = self.detect_with_multiple_scales(enhanced_frame)
            
            # Filter detections by color
            if self.use_color_filtering:
                all_detections = self.filter_detections_by_color(all_detections, red_mask, frame)
            
            # Merge overlapping detections
            final_detections = self.merge_overlapping_detections(all_detections)
            
            # Create ROS message
            bbox_array = BoundingBoxArray()
            bbox_array.header.stamp = rospy.Time.now()
            bbox_array.header.frame_id = "camera_frame"
            
            # Visualize detections
            vis_frame = frame.copy()
            
            for det in final_detections:
                # Create bounding box message
                bbox = BoundingBox()
                bbox.class_id = det['class_id']
                bbox.class_name = f"tomato ({det['class_name']})"  # Rename to tomato
                bbox.confidence = det['confidence']
                bbox.x_min = det['x_min']
                bbox.y_min = det['y_min']
                bbox.x_max = det['x_max']
                bbox.y_max = det['y_max']
                
                bbox_array.bounding_boxes.append(bbox)
                
                # Draw on visualization
                color = (0, 0, 255)  # Red for tomatoes
                cv2.rectangle(vis_frame, 
                            (int(det['x_min']), int(det['y_min'])), 
                            (int(det['x_max']), int(det['y_max'])), 
                            color, 2)
                
                # Add confidence and additional info
                label = f"Tomato: {det['confidence']:.2f}"
                if 'red_percentage' in det:
                    label += f" (R:{det['red_percentage']:.1f})"
                
                cv2.putText(vis_frame, label,
                          (int(det['x_min']), int(det['y_min']) - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Publish results
            self.bbox_pub.publish(bbox_array)
            
            # Display windows
            if self.show_camera_window:
                # Add detection count
                cv2.putText(vis_frame, f"Tomatoes: {len(final_detections)}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(vis_frame, "Press 'q' to quit", 
                           (10, vis_frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow('Tomato Detection', vis_frame)
                cv2.imshow('Color Filtered', cv2.bitwise_and(frame, frame, mask=red_mask))
                
                # Show processing steps
                processing_vis = np.hstack([
                    cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR),
                    cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)
                ])
                cv2.imshow('Processing Steps', processing_vis)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.signal_shutdown("User requested shutdown")
                    break
            
            # Publish processed image
            try:
                image_msg = self.bridge.cv2_to_imgmsg(vis_frame, "bgr8")
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
        detector = EnhancedTomatoDetector()
        detector.detect_and_publish()
    except rospy.ROSInterruptException:
        pass