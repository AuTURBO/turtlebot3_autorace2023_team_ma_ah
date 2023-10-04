#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from matplotlib import pyplot as plt
from ma_ah_perception.preprocessor import PreProcessor

from std_msgs.msg import Int32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from ma_ah_perception.undistort import undistort_func
import sys 

roi_height = 200
roi_width = 640

pre_module = PreProcessor(roi_height, roi_width)


class Lane_detector:
    def __init__(self, image_topic, cmd_vel_topic):

        self.lane_bin_th = 120  # 145
        self.frameWidth = 0
        self.frameHeight = 0

        self.blue = (255, 0, 0)
        self.green = (0, 255, 0)
        self.red = (0, 0, 255)

        self.img_subscriber = rospy.Subscriber(image_topic, CompressedImage, self.img_cb)
        # self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.cener_line_publisher = rospy.Publisher("/detect/lane", Float64, queue_size=10)
    
    def img_cb(self, img_msg):
        try:
            # cv_image = CvBridge().imgmsg_to_cv2(img_msg, "bgr8")
            cv_image = CvBridge().compressed_imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            self.process(cv_image)

    def map(self, x,input_min,input_max,output_min,output_max):
        return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.

    def simple_controller(self, lx, ly, mx, my, rx, ry):
        target = 320
        side_margin = 180

        if lx != None and rx != None and len(lx) > 5 and len(rx) > 5:
            print("ALL!!!")
            target = (lx[0] + rx[0]) // 2
        elif mx != None and len(mx) > 3:
            # print("Mid!!!")
            target = mx[0]
        elif lx != None and len(lx) > 3:
            print("Right!!!")
            #print(f"val: {lx[0]}")
            target = lx[0] + side_margin
        elif rx != None and len(rx) > 3:
            print("Left!!!")
            target = rx[0] - side_margin

        print(f"target: {target}")
        return int(target)


    def process(self, frame):
        print(frame.shape)

        frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)

        # cv2.imshow("Distort", frame)

        frame = undistort_func(frame)
        cv2.imshow("Undistort", frame)

        gblur_img  = cv2.GaussianBlur(frame, (3, 3), sigmaX = 0, sigmaY = 0)
        #cv2.imshow("gblur_img", gblur_img)

        gray = cv2.cvtColor(gblur_img, cv2.COLOR_BGR2GRAY)
        binary = self.threshold_binary(gray, self.lane_bin_th, "otsu", window_name="otsu", show=True)
        cv2.imshow("otsu", binary)

        warped_img = pre_module.warp_perspect(binary)
        cv2.imshow('warped_img', warped_img)	

        edge = self.canny(warped_img, 70, 210, show=False)

        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
        closing = cv2.morphologyEx(warped_img, cv2.MORPH_CLOSE,kernel_close)
        cv2.imshow('closing', closing)	# 프레임 보여주기

        msk, lx, ly, mx, my, rx, ry = pre_module.sliding_window(closing)

        filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry = pre_module.filtering_lane(msk, lx, ly, mx, my, rx, ry)
        pre_module.drawing_lane(msk, filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry)

        target = self.simple_controller(filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry)

        angle = 320 - target
        angle = self.map(angle, 100, -100, 1.0, -1.0)
        # angle = angle * 0.5
        # print(f"angle: {angle}")

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.1
        cmd_vel_msg.angular.z = angle

        # self.cmd_vel_publisher.publish(cmd_vel_msg)

        center_line_msg = Float64()
        center_line_msg.data = target
        self.cener_line_publisher.publish(center_line_msg)

        cv2.circle(frame, (int(target), int(480 - 135)), 1, (120, 0, 255), 10)

        cv2.imshow("Lane Detection - Sliding Windows", msk)
        cv2.imshow('frame', frame)	# 프레임 보여주기

        key = cv2.waitKey(1)  # frameRate msec동안 한 프레임을 보여준다
        
        # 키 입력을 받으면 키값을 key로 저장
        if key == ord('q'):
            sys.exit(0)

    def threshold_binary(self, img, lane_bin_th, method, thresholding_type=cv2.THRESH_BINARY, window_name="threshold", show=False):
        if method == "adaptive":
            lane = cv2.adaptiveThreshold(img, 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
        cv2.THRESH_BINARY_INV,11,9)
        elif method == "otsu":
            _, lane = cv2.threshold(img, 0, 255, thresholding_type+cv2.THRESH_OTSU)
        elif method == "basic":
            _, lane = cv2.threshold(img, lane_bin_th, 255, thresholding_type)

        if show == True:
            cv2.imshow(window_name, lane)
        return lane


    def canny(self, img, low_threshold, high_threshold, show=False): # Canny 알고리즘
        canny = cv2.Canny(img, low_threshold, high_threshold)

        if show == True:
            cv2.imshow("Canny", canny)
        return canny

    def draw_lines(self,  img, lines, color=[0, 0, 255], thickness=2): # 선 그리기
        for line in lines:
            for x1,y1,x2,y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)

    def hough_lines(self,  img, rho, theta, threshold, min_line_len, max_line_gap, show=False): # 허프 변환
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        self.draw_lines(line_img, lines)

        if show == True:
            cv2.imshow("hough_lines", line_img)
        return line_img

if __name__ == '__main__':
    rospy.init_node("lane_detector")
    
    image_topic = "/camera/image/compressed"
    cmd_vel_topic = "cmd_vel"
    center_line_topic = "/detect/lane"

    # lane_detector = Lane_detector(image_topic, cmd_vel_topic)
    lane_detector = Lane_detector(image_topic, center_line_topic)
    rospy.spin()
