#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

void bboxCallback(const geometry_msgs::Polygon::ConstPtr& msg)
{
    if (msg->points.size() != 4)
    {
        ROS_WARN("Received polygon does not have 4 points!");
        return;
    }

    float x1 = msg->points[0].x;  // top-left
    float y1 = msg->points[0].y;

    float x2 = msg->points[1].x;  // top-right
    float y2 = msg->points[2].y;  // bottom-right y

    // For rectangle bounding box:
    float width = x2 - x1;
    float height = y2 - y1;

    ROS_INFO("Bounding Box: x=%.2f, y=%.2f, w=%.2f, h=%.2f", x1, y1, width, height);

    // Here you can add code to use the bounding box for robot control
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bbox_listener_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/bbox_coords", 10, bboxCallback);

    ros::spin();

    return 0;
}
