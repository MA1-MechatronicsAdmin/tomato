#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Polygon, Point32
import cv2
from ultralytics import YOLO

def bbox_to_points(x1, y1, x2, y2):
    return [
        Point32(x=x1, y=y1, z=0),
        Point32(x=x2, y=y1, z=0),
        Point32(x=x2, y=y2, z=0),
        Point32(x=x1, y=y2, z=0),
    ]

def main():
    rospy.init_node("yolov8_bbox_publisher")
    pub = rospy.Publisher("/bbox_coords", Polygon, queue_size=10)

    model = YOLO("yolov8n.pt")  # Replace with your trained model if needed
    cap = cv2.VideoCapture(0)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to grab frame")
            continue

        results = model(frame, conf=0.1)[0]
        polygon_msg = Polygon()

        for box in results.boxes:
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            label = model.names[cls_id]

            # Filter only tomatoes (or remove this to include all)
            if label != "banana" or label != "apple" or label != "orange" or label != "carrot":
                continue
            
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            polygon_msg.points.extend(bbox_to_points(x1, y1, x2, y2))

            # Optional: Visualize
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if polygon_msg.points:
            pub.publish(polygon_msg)
            rospy.loginfo(f"Published {len(polygon_msg.points)//4} bounding boxes")

        cv2.imshow("YOLOv8 Bounding Boxes", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
