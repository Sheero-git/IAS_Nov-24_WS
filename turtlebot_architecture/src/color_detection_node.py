#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

def detect_colors(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color ranges for red, blue, and white
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 50, 50])
    upper_red2 = np.array([180, 255, 255])

    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([130, 255, 255])

    # Adjust white range
    lower_white = np.array([0, 0, 150])
    upper_white = np.array([180, 80, 255])

    mask_red1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
    mask_red = cv2.add(mask_red1, mask_red2)

    mask_blue = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    mask_white = cv2.inRange(hsv_frame, lower_white, upper_white)

    # Apply Gaussian blur to reduce noise
    mask_red = cv2.GaussianBlur(mask_red, (5, 5), 0)
    mask_blue = cv2.GaussianBlur(mask_blue, (5, 5), 0)
    mask_white = cv2.GaussianBlur(mask_white, (5, 5), 0)

    # Apply morphological operations to reduce noise further
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
    mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)

    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_white, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    min_area = 500
    max_area = 500000

    detected_objects = []

    for contour in contours_red:
        if cv2.contourArea(contour) > min_area and cv2.contourArea(contour) < max_area:
            x, y, w, h = cv2.boundingRect(contour)
            # Check if the y-coordinate is less than or equal to 200
            if y >= 350:
                center_x, center_y = x + w // 2, y + h // 2
                detected_objects.append(f"Red object center: ({center_x}, {center_y})")
                # Draw rectangle and center point
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    for contour in contours_blue:
        if cv2.contourArea(contour) > min_area and cv2.contourArea(contour) < max_area:
            x, y, w, h = cv2.boundingRect(contour)
            # Check if the y-coordinate is less than or equal to 200
            if y >= 350:
                center_x, center_y = x + w // 2, y + h // 2
                detected_objects.append(f"Blue object center: ({center_x}, {center_y})")
                # Draw rectangle and center point
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

    for contour in contours_white:
        if cv2.contourArea(contour) > min_area and cv2.contourArea(contour) < max_area:
            x, y, w, h = cv2.boundingRect(contour)
            # Check if the y-coordinate is less than or equal to 200
            if y  >= 350:
                center_x, center_y = x + w // 2, y + h // 2
                detected_objects.append(f"White object center: ({center_x}, {center_y})")
                # Draw rectangle and center point
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
                cv2.circle(frame, (center_x, center_y), 5, (255, 255, 255), -1)

    return detected_objects

def image_callback(msg):
    global bridge
    global detected_objects_pub
    global processed_image_pub

    try:
        # Convert ROS Image message to OpenCV image
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Resize frame to desired dimensions (500x500)
        frame_resized = cv2.resize(frame, (500, 500), interpolation=cv2.INTER_AREA)

        # Process the frame for color detection
        detected_objects = detect_colors(frame_resized)
        detected_objects_message = "\n".join(detected_objects)
        
        # Display the resized frame with detected objects
        cv2.imshow('Detected Objects', frame_resized)
        cv2.waitKey(1)  # Ensure the frame is updated in the window
        
        # Publish detected objects
        detected_objects_pub.publish(detected_objects_message)

        # Convert frame to ROS Image message and publish
        try:
            ros_image = bridge.cv2_to_imgmsg(frame_resized, encoding="bgr8")
            processed_image_pub.publish(ros_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", str(e))

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: %s", str(e))

def main():
    global bridge
    global detected_objects_pub
    global processed_image_pub

    rospy.init_node('color_detection_node')

    # Initialize CvBridge
    bridge = CvBridge()

    # Publishers for detected objects and processed image
    detected_objects_pub = rospy.Publisher('/color_detection/detected_objects', String, queue_size=10)
    processed_image_pub = rospy.Publisher('/color_detection/processed_image', Image, queue_size=10)

    # Subscriber to the /camera/image topic
    rospy.Subscriber('/camera/image', Image, image_callback)

    rospy.spin()

    # Ensure all OpenCV windows are closed after ROS shutdown
    cv2.destroyAllWindows()

if _name_ == "_main_":
    main()