#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def callback(msg):
    global pub, bridge

    # CompressedImage 메시지를 OpenCV 이미지로 변환
    try:
        cv_image = CvBridge().compressed_imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # OpenCV 이미지를 Image 메시지로 변환
    try:
        image_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        pub.publish(image_msg)
    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('compressed_to_image_node', anonymous=True)

    # CvBridge 객체 초기화
    bridge = CvBridge()

    # Subscriber와 Publisher 설정
    rospy.Subscriber('/camera/image/compressed', CompressedImage, callback)
    pub = rospy.Publisher('/camera/image', Image, queue_size=10)

    rospy.spin()
