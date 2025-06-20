#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
from yolo_detection.msg import BoundingBoxArray, BoundingBox
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Header
import math

class RealSenseTomatoDetector:
    def __init__(self):
        rospy.init_node('realsense_tomato_detector', anonymous=True)
        
        # Load YOLOv8 model
        model_path = rospy.get_param('~model_path', 'yolov8n.pt')  # Use your custom tomato model
        self.yolo = YOLO(model_path, verbose=False)  # Suppress YOLO detection prints
        
        # Define classes that should be treated as tomatoes
        # You can modify this list to include/exclude classes as needed
        self.tomato_like_classes = rospy.get_param('~tomato_like_classes', [
            'tomato',
            'apple', 
            'orange',
            'sports ball',
            'banana',
            'tennis ball',
            'baseball',
            'basketball',
            'soccer ball'
        ])
        
        rospy.loginfo(f"Classes treated as tomatoes: {self.tomato_like_classes}")
        
        # RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams (matching your existing resolution preference)
        width = rospy.get_param('~image_width', 640)
        height = rospy.get_param('~image_height', 480)
        fps = rospy.get_param('~fps', 30)
        
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        # Start streaming
        try:
            self.profile = self.pipeline.start(self.config)
            rospy.loginfo("RealSense camera started successfully")
        except Exception as e:
            rospy.logerr(f"Failed to start RealSense camera: {e}")
            return
        
        # Get camera intrinsics
        self.depth_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth))
        self.color_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.color))
        self.depth_intrinsics = self.depth_profile.get_intrinsics()
        self.color_intrinsics = self.color_profile.get_intrinsics()
        
        # Align depth to color
        self.align = rs.align(rs.stream.color)
        
        # Create depth colorizer for visualization
        self.colorizer = rs.colorizer()
        
        # ROS setup
        self.bridge = CvBridge()
        
        # Publishers - matching your existing topic names
        self.bbox_pub = rospy.Publisher('tomato_detections', BoundingBoxArray, queue_size=1)
        self.image_pub = rospy.Publisher('/realsense/annotated_image', Image, queue_size=1)
        self.depth_pub = rospy.Publisher('/realsense/depth_colorized', Image, queue_size=1)
        self.tomato_point_pub = rospy.Publisher('/realsense/tomato_3d_point', PointStamped, queue_size=1)
        
        # Parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        self.tomato_class_name = rospy.get_param('~tomato_class_name', 'tomato')
        
        # Camera transformation parameters (similar to your cam_angle)
        self.camera_tilt_angle = rospy.get_param('~camera_tilt_angle', 0.0)  # degrees
        self.camera_height = rospy.get_param('~camera_height', 0.0)  # meters above arm base
        self.camera_offset_x = rospy.get_param('~camera_offset_x', 0.0)  # meters from arm base
        
        # Convert to radians
        self.cam_angle_rad = math.radians(self.camera_tilt_angle)
        
        # Depth filtering parameters
        self.min_depth = rospy.get_param('~min_depth', 0.1)  # meters
        self.max_depth = rospy.get_param('~max_depth', 3.0)   # meters
        
        rospy.loginfo("RealSense Tomato Detector initialized")
        
    def is_tomato_like(self, class_name):
        """
        Check if the detected class should be treated as a tomato
        """
        return class_name.lower() in [cls.lower() for cls in self.tomato_like_classes]
        
    def get_3d_coordinates(self, center_x, center_y, depth_frame):
        """
        Get 3D coordinates from pixel coordinates and depth
        Returns coordinates in camera frame (meters)
        """
        # Get depth value at center point (in mm)
        depth_value = depth_frame.get_distance(int(center_x), int(center_y))
        
        if depth_value == 0 or depth_value < self.min_depth or depth_value > self.max_depth:
            return None
            
        # Deproject pixel to 3D point in camera frame
        point_3d = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [center_x, center_y], depth_value)
        
        return point_3d  # [x, y, z] in meters in camera frame
    
    def transform_to_arm_coordinates(self, camera_coords):
        """
        Transform from camera coordinates to arm coordinate system
        This replaces your matrix transformation logic
        """
        if camera_coords is None:
            return None
            
        x_cam, y_cam, z_cam = camera_coords
        
        # Convert to centimeters (matching your existing code)
        x_cam_cm = x_cam * 100
        y_cam_cm = y_cam * 100  
        z_cam_cm = z_cam * 100
        
        # Apply camera tilt transformation (similar to your cam_angle logic)
        # Assuming camera is tilted downward by cam_angle_rad
        x_arm = z_cam_cm * math.cos(self.cam_angle_rad) - y_cam_cm * math.sin(self.cam_angle_rad)
        y_arm = y_cam_cm * math.cos(self.cam_angle_rad) + z_cam_cm * math.sin(self.cam_angle_rad)
        z_arm = x_cam_cm  # Left-right remains the same
        
        # Apply camera offset if needed
        x_arm += self.camera_offset_x * 100  # convert to cm
        y_arm += self.camera_height * 100
        
        return [x_arm, y_arm, z_arm]
    
    def detect_tomatoes(self, color_image, depth_frame):
        """
        Detect tomatoes using YOLOv8 and get their 3D coordinates
        """
        # Run YOLO detection with verbose=False to suppress prints
        results = self.yolo(color_image, conf=self.confidence_threshold, verbose=False)
        
        # Create BoundingBoxArray message
        bbox_array = BoundingBoxArray()
        bbox_array.header.stamp = rospy.Time.now()
        bbox_array.header.frame_id = self.camera_frame
        
        annotated_image = color_image.copy()
        best_tomato_3d = None
        min_distance = float('inf')
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Get class name
                    class_id = int(box.cls[0])
                    original_class_name = self.yolo.names[class_id]
                    confidence = float(box.conf[0])
                    
                    # Check if this class should be treated as a tomato
                    if self.is_tomato_like(original_class_name) and confidence >= self.confidence_threshold:
                        # Get bounding box coordinates
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        
                        # Calculate center and size
                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2
                        width = x2 - x1
                        height = y2 - y1
                        
                        # Check aspect ratio (similar to your existing logic)
                        aspect_ratio_diff = abs(width - height) / max(width, height)
                        if aspect_ratio_diff > 0.1:  # Skip non-circular objects
                            continue
                        
                        # Get 3D coordinates
                        camera_coords = self.get_3d_coordinates(center_x, center_y, depth_frame)
                        if camera_coords is None:
                            continue
                            
                        arm_coords = self.transform_to_arm_coordinates(camera_coords)
                        if arm_coords is None:
                            continue
                            
                        # Calculate distance for closest tomato selection
                        distance = math.sqrt(sum(coord**2 for coord in camera_coords))
                        
                        # Create BoundingBox message (always use "tomato" as class name)
                        bbox_msg = BoundingBox()
                        bbox_msg.class_name = "tomato"  # Always report as tomato
                        bbox_msg.confidence = confidence
                        bbox_msg.x_min = int(x1)
                        bbox_msg.y_min = int(y1)
                        bbox_msg.x_max = int(x2)
                        bbox_msg.y_max = int(y2)
                        
                        bbox_array.bounding_boxes.append(bbox_msg)
                        
                        # Track closest tomato
                        if distance < min_distance:
                            min_distance = distance
                            best_tomato_3d = arm_coords
                        
                        # Annotate image with only "tomato" label
                        cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        
                        # Show only "tomato" label
                        label_text = f"tomato {confidence:.2f}"
                        cv2.putText(annotated_image, label_text, (int(x1), int(y1)-40), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        
                        # Add 3D coordinates text
                        coord_text = f"3D: ({arm_coords[0]:.1f}, {arm_coords[1]:.1f}, {arm_coords[2]:.1f})cm"
                        cv2.putText(annotated_image, coord_text, (int(x1), int(y1)-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        
                        # Add distance text
                        dist_text = f"Dist: {distance:.2f}m"
                        cv2.putText(annotated_image, dist_text, (int(x1), int(y1)-25), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        return bbox_array, annotated_image, best_tomato_3d
    
    def publish_3d_point(self, arm_coords):
        """
        Publish the 3D point of the closest tomato
        """
        if arm_coords is None:
            return
            
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = self.camera_frame
        point_msg.point.x = arm_coords[0] / 100.0  # Convert back to meters
        point_msg.point.y = arm_coords[1] / 100.0
        point_msg.point.z = arm_coords[2] / 100.0
        
        self.tomato_point_pub.publish(point_msg)
    
    def run(self):
        """
        Main detection loop
        """
        rate = rospy.Rate(30)  # 30 Hz
        
        try:
            while not rospy.is_shutdown():
                # Get frames
                frames = self.pipeline.wait_for_frames()
                
                # Align depth frame to color frame
                aligned_frames = self.align.process(frames)
                
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    continue
                
                # Convert to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Detect tomatoes
                bbox_array, annotated_image, best_tomato_3d = self.detect_tomatoes(color_image, depth_frame)
                
                # Publish results
                self.bbox_pub.publish(bbox_array)
                
                # Publish 3D point of closest tomato
                self.publish_3d_point(best_tomato_3d)
                
                # Publish annotated image
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
                    image_msg.header.stamp = rospy.Time.now()
                    image_msg.header.frame_id = self.camera_frame
                    self.image_pub.publish(image_msg)
                except Exception as e:
                    rospy.logwarn(f"Failed to publish annotated image: {e}")
                
                # Publish colorized depth image
                try:
                    colorized_depth = np.asanyarray(self.colorizer.colorize(depth_frame).get_data())
                    depth_msg = self.bridge.cv2_to_imgmsg(colorized_depth, "bgr8")
                    depth_msg.header.stamp = rospy.Time.now()
                    depth_msg.header.frame_id = self.camera_frame
                    self.depth_pub.publish(depth_msg)
                except Exception as e:
                    rospy.logwarn(f"Failed to publish depth image: {e}")
                
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down RealSense Tomato Detector")
        finally:
            self.pipeline.stop()

if __name__ == '__main__':
    try:
        detector = RealSenseTomatoDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in RealSense Tomato Detector: {e}")