#include <ros/ros.h>
#include <yolo_detection/BoundingBoxArray.h>
#include <yolo_detection/BoundingBox.h>
#include <iostream>
#include <vector>

class BoundingBoxListener
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber bbox_sub_;
    
public:
    BoundingBoxListener()
    {
        // Subscribe to bounding box topic
        bbox_sub_ = nh_.subscribe("/yolo_detections", 10, 
                                 &BoundingBoxListener::bboxCallback, this);
        
        ROS_INFO("Bounding Box Listener initialized");
        ROS_INFO("Listening for bounding boxes on /yolo_detections");
    }
    
    void bboxCallback(const yolo_detection::BoundingBoxArray::ConstPtr& msg)
    {
        // std::cout << "----------------------------------------" << std::endl;
        // ROS_INFO("Received %lu bounding boxes at time: %f", 
        //          msg->bounding_boxes.size(), msg->header.stamp.toSec());
        
        // Process each bounding box
        for (size_t i = 0; i < msg->bounding_boxes.size(); ++i)
        {
            
            const yolo_detection::BoundingBox& bbox = msg->bounding_boxes[i];
            double distance = 3/((bbox.x_max - bbox.x_min) + (bbox.y_max - bbox.y_min))/2.0 *3111;

            // ROS_INFO("Detection %lu:", i + 1);
            // ROS_INFO("  Class: %s (ID: %d)", bbox.class_name.c_str(), bbox.class_id);
            // ROS_INFO("  Confidence: %.3f", bbox.confidence);
            // ROS_INFO("  Bounding Box: (%.1f, %.1f) to (%.1f, %.1f)", 
            //          bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max);
            // ROS_INFO("  Center: (%.1f, %.1f)", // (center screen 300, 256), (158 - 297 = 4 cm. scale ~= 35)
            //          (bbox.x_min + bbox.x_max) / 2.0, 
            //          (bbox.y_min + bbox.y_max) / 2.0);
            // ROS_INFO("  Size: %.1f x %.1f", 
            //          bbox.x_max - bbox.x_min, 
            //          bbox.y_max - bbox.y_min);
            // ROS_INFO("distance: %.9f +- %.9f",
            //         2.8/((bbox.x_max - bbox.x_min) + (bbox.y_max - bbox.y_min))/2.0 *3111,
            //         0.3/((bbox.x_max - bbox.x_min) + (bbox.y_max - bbox.y_min))/2.0 *3111
            //         );
            // ROS_INFO("pos: %.9f , %.9f",// ratio 15/17
            //         (300 - (bbox.x_min + bbox.x_max) / 2.0)/600 * 15 / 17 * distance , 
            //         ((bbox.y_min + bbox.y_max) / 2.0 - 256)/512 * 15 / 17 * distance
            //         );
            
            // Add your custom processing logic here
            processDetection(bbox);
        }        
        std::cout << "----------------------------------------" << std::endl;
    }
    
    void processDetection(const yolo_detection::BoundingBox& bbox)
    {
        // Custom processing logic for each detection
        // Examples:
        
        // // 1. Filter by class
        // if (bbox.class_name == "person")
        // {
        //     ROS_INFO("Person detected at center: (%.1f, %.1f)", 
        //              (bbox.x_min + bbox.x_max) / 2.0, 
        //              (bbox.y_min + bbox.y_max) / 2.0);
        // }
        
        // // 2. Check if object is in specific region
        // double center_x = (bbox.x_min + bbox.x_max) / 2.0;
        // double center_y = (bbox.y_min + bbox.y_max) / 2.0;
        
        // if (center_x > 200 && center_x < 400 && center_y > 150 && center_y < 350)
        // {
        //     ROS_INFO("Object '%s' is in the center region!", bbox.class_name.c_str());
        // }
        
        // // 3. Size-based filtering
        // double width = bbox.x_max - bbox.x_min;
        // double height = bbox.y_max - bbox.y_min;
        // double area = width * height;
        
        // if (area > 10000)  // Large objects
        // {
        //     ROS_INFO("Large object detected: %s (area: %.0f)", bbox.class_name.c_str(), area);
        // }
        
        // Add your own custom logic here based on your application needs
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bounding_box_listener");
    
    BoundingBoxListener listener;
    
    ROS_INFO("Bounding Box Listener node started");
    
    ros::spin();
    
    return 0;
}