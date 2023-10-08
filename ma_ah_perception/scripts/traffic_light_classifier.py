#!/usr/bin/env python3

import os
import numpy as np
import sys 
import rospy

import cv2
from cv_bridge import CvBridge

from std_msgs.msg import Int32, Float64, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray

class Traffic_light_classifier:
    def __init__(self, img_topic, yolo_detection_topic, traffic_color_topic):
        self.blue = (255, 0, 0)
        self.green = (0, 255, 0)
        self.red = (0, 0, 255)

        self.roi_x = 0
        self.roi_y = 0
        self.cv_image = None
        self.traffic_color = String()
        self.traffic_color.data = "red"
        self.done = False
        self.detection_msg = None

        self.labels = ("boom_barrier", "intersection", "left", "no_entry", "obstacle", \
                       "parking", "right", "stop", "traffic_light", "tunnel")

        self.img_subscriber = rospy.Subscriber(img_topic, CompressedImage, self.img_cb)
        # self.filtered_detection_subscriber = rospy.Subscriber(filtered_detection_topic, String, self.filtered_detection_cb)
        self.yolo_detection_subscriber =  rospy.Subscriber(yolo_detection_topic, Detection2DArray, self.detection_cb)
        self.traffic_color_publisher = rospy.Publisher(traffic_color_topic, String, queue_size=10)
    
    def img_cb(self, img_msg):
        try:
            # cv_image = CvBridge().imgmsg_to_cv2(img_msg, "bgr8")
            self.cv_image = None
            self.cv_image = CvBridge().compressed_imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            pass
            # cv2.imshow("traffic", self.cv_image)
            # key = cv2.waitKey(1)
            # if key == ord('q'):
            #     sys.exit(0)
            #cv2.imshow('callback', cv_image)	# 프레임 보여주기
            # self.process(cv_image)
    
    # def filtered_detection_cb(self, msg):
    #     try:
    #     except CvBridgeError as e:
    #         print(e)
    #     else:
    #         #cv2.imshow('callback', cv_image)	# 프레임 보여주기
    #         self.process(cv_image)

    def detection_cb(self, detection_msg):
        print("-----------")
        self.detection_msg = detection_msg

    def process(self):
        print(self.detection_msg)
        if  self.detection_msg is not None:
            print(len(self.detection_msg.detections))
            if len(self.detection_msg.detections) > 0:
                for i in range(len(self.detection_msg.detections)):
                    obj_id =  self.detection_msg.detections[i].results[0].id 

                    # If detect traffic_light

                    center_y = None                 
                    center_x = None
                    bbox_size_x = None
                    bbox_size_y = None

                    if  self.cv_image is not None:
                        if self.labels[obj_id] == "traffic_light":

                            center_x = int(self.detection_msg.detections[i].bbox.center.x)
                            center_y = int(self.detection_msg.detections[i].bbox.center.y)
                            bbox_size_x = int(self.detection_msg.detections[i].bbox.size_x)
                            bbox_size_y = int(self.detection_msg.detections[i].bbox.size_y)

                            self.roi_x = int(center_x - (bbox_size_x // 2))
                            self.roi_y = int(center_y - (bbox_size_y // 2))

                            # cv2.imshow("origin", self.cv_image)
                            #print("roi")
                            roi = self.cv_image[self.roi_y:self.roi_y+bbox_size_y, self.roi_x:self.roi_x+bbox_size_x]  
                            #self.process(roi)

                            green_img = self.color_filtering(roi)

                            gray = cv2.cvtColor(green_img, cv2.COLOR_BGR2GRAY)

                            pixel_num = cv2.countNonZero(gray)
                            print(pixel_num)

                            if pixel_num > 700:
                                self.traffic_color.data = "green"
                                self.done = True

                            self.traffic_color_publisher.publish(self.traffic_color)
                            #cv2.imshow("traffic_light_classifier", self.cv_image)
                            #cv2.waitKey(0)

    def color_filtering(self, img):
        hsv_lower_green = (45, 40, 40)
        hsv_upper_green = (75, 255, 255)

        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #cv2.imshow("hsv_img", hsv_img)
        # 색상 범위를 제한하여 mask 생성
        hsv_mask = cv2.inRange(hsv_img, hsv_lower_green, hsv_upper_green)

        # ## Slice the green
        # imask = hsv_mask>0
        # green = np.zeros_like(img, np.uint8)
        # green[imask] = img[imask]
        # # 원본 이미지를 가지고 Object 추출 이미지로 생성
        hsv_result = cv2.bitwise_and(img, img, mask=hsv_mask)
        cv2.imshow("hsv_result", hsv_result)

        return hsv_result


if __name__ == '__main__':
    rospy.init_node("traffic_light_classifier")
    
    image_topic = "/camera/image/compressed"
    #filtered_detection_topic = "/filtered/detection"
    yolo_detection_topic = "/yolov7/detection"
    traffic_color_topic = "/traffic_color"
    traffic_light_classifier = Traffic_light_classifier(image_topic, yolo_detection_topic, traffic_color_topic)

    while True:
        if traffic_light_classifier.done == False:
            traffic_light_classifier.process()
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
    rospy.spin()