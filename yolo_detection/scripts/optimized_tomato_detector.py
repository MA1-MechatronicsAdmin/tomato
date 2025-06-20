#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolo_detection.msg import BoundingBoxArray, BoundingBox
from ultralytics import YOLO
import os
import threading
import time
from collections import deque

class OptimizedTomatoDetector:
    def __init__(self):
        rospy.init_node('optimized_tomato_detector', anonymous=True)
        
        # Model loading - use smaller model for speed
        model_path = rospy.get_param('~model_path', 'yolov8n.pt')  # Nano for speed
        custom_model_path = rospy.get_param('~custom_model_path', None)
        
        if custom_model_path and os.path.exists(custom_model_path):
            model_path = custom_model_path
            rospy.loginfo(f"Using custom tomato model: {model_path}")
        
        try:
            self.model = YOLO(model_path)
            # Optimize model for inference speed
            self.model.to('cuda' if self.has_cuda() else 'cpu')
            rospy.loginfo(f"YOLO model loaded: {model_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            return
        
        # Camera setup - optimized settings
        self.camera_id = rospy.get_param('~camera_id', 0)
        self.cap = cv2.VideoCapture(self.camera_id)
        
        if not self.cap.isOpened():
            rospy.logerr(f"Failed to open camera {self.camera_id}")
            rospy.signal_shutdown("Camera initialization failed")
            return
        
        # Optimized camera settings for speed
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)    # Lower resolution for speed
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)       # Reduce buffer for real-time
        
        # ROS setup
        self.bridge = CvBridge()
        self.bbox_pub = rospy.Publisher('/tomato_detections', BoundingBoxArray, queue_size=1)
        self.image_pub = rospy.Publisher('/tomato_image', Image, queue_size=1)
        
        # Optimized detection parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.25)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)
        
        # Performance optimization settings
        self.process_every_nth_frame = rospy.get_param('~process_every_nth_frame', 1)  # Process every frame by default
        self.use_threading = rospy.get_param('~use_threading', True)
        self.max_fps = rospy.get_param('~max_fps', 15)  # Cap FPS for consistent performance
        
        # Simple color filtering (optional, faster than complex preprocessing)
        self.use_simple_color_filter = rospy.get_param('~use_simple_color_filter', False)
        
        # Tomato detection classes (map common objects to tomato-like)
        self.tomato_classes = {
            'apple': 'tomato',
            'orange': 'tomato', 
            'carrot': 'tomato',
            'banana': 'tomato',
            'donut': 'tomato',
            'cake': 'tomato'
        }
        
        # Threading setup
        self.frame_queue = deque(maxlen=2)  # Small queue to prevent lag
        self.detection_queue = deque(maxlen=2)
        self.processing_lock = threading.Lock()
        self.latest_frame = None
        self.latest_detections = []
        
        # Performance monitoring
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.processing_times = deque(maxlen=30)
        
        # Display settings
        self.show_camera_window = rospy.get_param('~show_camera_window', True)
        if self.show_camera_window:
            cv2.namedWindow('Fast Tomato Detection', cv2.WINDOW_AUTOSIZE)
        
        rospy.loginfo("Optimized Tomato Detector initialized")
        rospy.loginfo(f"Processing every {self.process_every_nth_frame} frame(s)")
        rospy.loginfo(f"Target max FPS: {self.max_fps}")
        
    def has_cuda(self):
        """Check if CUDA is available"""
        try:
            import torch
            return torch.cuda.is_available()
        except:
            return False
    
    def simple_color_filter(self, image):
        """Fast color filtering for red objects"""
        if not self.use_simple_color_filter:
            return image
            
        # Simple red enhancement in BGR
        enhanced = image.copy()
        # Boost red channel slightly
        enhanced[:,:,2] = np.clip(enhanced[:,:,2] * 1.1, 0, 255)
        # Reduce green channel slightly to make reds pop more
        enhanced[:,:,1] = np.clip(enhanced[:,:,1] * 0.9, 0, 255)
        
        return enhanced
    
    def fast_detect(self, image):
        """Optimized detection with minimal processing"""
        start_time = time.time()
        
        # Apply simple color filter if enabled
        processed_image = self.simple_color_filter(image)
        
        # Run YOLO detection with optimized settings
        results = self.model(
            processed_image,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            verbose=False,
            device='cuda' if self.has_cuda() else 'cpu',
            half=True if self.has_cuda() else False  # Use half precision on GPU
        )
        
        detections = []
        
        # Fast extraction of relevant detections
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = self.model.names[class_id]
                    
                    # Quick check for tomato-like objects
                    if class_name in self.tomato_classes or confidence > 0.5:
                        coords = box.xyxy[0].tolist()
                        
                        # Basic size filtering (very fast)
                        width = coords[2] - coords[0]
                        height = coords[3] - coords[1]
                        
                        if width > 15 and height > 15:  # Minimum size check
                            detection = {
                                'class_id': class_id,
                                'class_name': self.tomato_classes.get(class_name, class_name),
                                'confidence': confidence,
                                'coords': coords
                            }
                            detections.append(detection)
        
        # Track processing time
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        
        return detections
    
    def detection_thread(self):
        """Separate thread for running detections"""
        while not rospy.is_shutdown():
            if self.frame_queue:
                with self.processing_lock:
                    if self.frame_queue:
                        frame = self.frame_queue.popleft()
                        detections = self.fast_detect(frame)
                        self.latest_detections = detections
            else:
                time.sleep(0.001)  # Small sleep to prevent busy waiting
    
    def create_bbox_message(self, detections, timestamp):
        """Fast creation of bounding box message"""
        bbox_array = BoundingBoxArray()
        bbox_array.header.stamp = timestamp
        bbox_array.header.frame_id = "camera_frame"
        
        for det in detections:
            bbox = BoundingBox()
            bbox.class_id = det['class_id']
            bbox.class_name = det['class_name']
            bbox.confidence = det['confidence']
            bbox.x_min = det['coords'][0]
            bbox.y_min = det['coords'][1] 
            bbox.x_max = det['coords'][2]
            bbox.y_max = det['coords'][3]
            
            bbox_array.bounding_boxes.append(bbox)
        
        return bbox_array
    
    def draw_detections(self, image, detections):
        """Fast visualization of detections"""
        vis_image = image.copy()
        
        for det in detections:
            coords = det['coords']
            
            # Choose color based on confidence
            if det['confidence'] > 0.7:
                color = (0, 255, 0)  # Green for high confidence
            elif det['confidence'] > 0.5:
                color = (0, 255, 255)  # Yellow for medium confidence  
            else:
                color = (0, 0, 255)  # Red for lower confidence
            
            # Draw bounding box
            cv2.rectangle(vis_image,
                         (int(coords[0]), int(coords[1])),
                         (int(coords[2]), int(coords[3])),
                         color, 2)
            
            # Draw label
            label = f"{det['class_name']}: {det['confidence']:.2f}"
            cv2.putText(vis_image, label,
                       (int(coords[0]), int(coords[1]) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return vis_image
    
    def run(self):
        # Start detection thread if threading is enabled
        if self.use_threading:
            detection_thread = threading.Thread(target=self.detection_thread)
            detection_thread.daemon = True
            detection_thread.start()
            rospy.loginfo("Detection thread started")
        
        # Main loop timing
        target_frame_time = 1.0 / self.max_fps
        frame_count = 0
        
        rate = rospy.Rate(self.max_fps)
        
        while not rospy.is_shutdown():
            loop_start = time.time()
            
            # Capture frame
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to capture frame")
                continue
            
            timestamp = rospy.Time.now()
            
            # Process frame (every nth frame or if not using threading)
            if frame_count % self.process_every_nth_frame == 0:
                if self.use_threading:
                    # Add to queue for threaded processing
                    with self.processing_lock:
                        self.frame_queue.append(frame.copy())
                        if len(self.frame_queue) > 1:
                            self.frame_queue.popleft()  # Keep only latest
                else:
                    # Direct processing
                    self.latest_detections = self.fast_detect(frame)
            
            # Create and publish messages
            bbox_msg = self.create_bbox_message(self.latest_detections, timestamp)
            self.bbox_pub.publish(bbox_msg)
            
            # Visualization
            if self.show_camera_window or self.image_pub.get_num_connections() > 0:
                vis_frame = self.draw_detections(frame, self.latest_detections)
                
                # Add performance info
                if self.processing_times:
                    avg_processing_time = np.mean(self.processing_times)
                    fps_display = 1.0 / avg_processing_time if avg_processing_time > 0 else 0
                    
                    cv2.putText(vis_frame, f"FPS: {fps_display:.1f}", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(vis_frame, f"Detections: {len(self.latest_detections)}", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Display window
                if self.show_camera_window:
                    cv2.imshow('Fast Tomato Detection', vis_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        rospy.signal_shutdown("User requested shutdown")
                        break
                
                # Publish image
                if self.image_pub.get_num_connections() > 0:
                    try:
                        image_msg = self.bridge.cv2_to_imgmsg(vis_frame, "bgr8")
                        image_msg.header = bbox_msg.header
                        self.image_pub.publish(image_msg)
                    except Exception as e:
                        rospy.logwarn(f"Failed to publish image: {e}")
            
            frame_count += 1
            
            # FPS monitoring
            self.fps_counter += 1
            if self.fps_counter % 30 == 0:  # Log every 30 frames
                elapsed = time.time() - self.fps_start_time
                actual_fps = 30 / elapsed
                rospy.loginfo(f"Actual FPS: {actual_fps:.1f}")
                self.fps_start_time = time.time()
            
            # Rate limiting
            rate.sleep()
    
    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'show_camera_window') and self.show_camera_window:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = OptimizedTomatoDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass