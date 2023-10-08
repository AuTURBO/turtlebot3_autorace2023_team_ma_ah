#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


pipeline = " ! ".join(["v4l2src device=/dev/video0",
                       "video/x-raw, width=640, height=480, framerate=30/1",
                       "videoconvert",
                       "video/x-raw, format=(string)BGR",
                       "appsink"
                       ])

def camera_publisher():
    # Initialize the node with the name 'camera_publisher'
    rospy.init_node('camera_publisher', anonymous=True)

    # Create a publisher on the 'camera_image/compressed' topic for compressed images
    pub = rospy.Publisher('/camera/image/compressed', CompressedImage, queue_size=2)

    # Set the video source to the default camera.
    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        rospy.logerr("Unable to open camera")
        return

    rate = rospy.Rate(60)  # 10 Hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        #cv2.imshow("frame", frame)
        #cv2.waitKey(1)
        if not ret:
            rospy.logerr("Failed to read from camera")
            break

        # Convert the OpenCV image to a JPEG image
        try:
            jpeg_image = cv2.imencode('.jpg', frame)[1].tostring()
            # Construct a ROS compressed image message
            compressed_image = CompressedImage()
            compressed_image.header.stamp = rospy.Time.now()
            compressed_image.format = "jpeg"
            compressed_image.data = jpeg_image
            pub.publish(compressed_image)
        except Exception as e:
            rospy.logerr(e)

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
