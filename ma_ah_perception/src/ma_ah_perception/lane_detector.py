#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy as np
from matplotlib import pyplot as plt
from ma_ah_perception.preprocessor import PreProcessor

from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from ma_ah_perception.undistort import undistort_func
import sys

roi_height = 200
roi_width = 640

pre_module = PreProcessor(roi_height, roi_width)


class Lane_detector:
    def __init__(self, state_topic, image_topic, cmd_vel_topic, center_lane_topic):

        self.lane_bin_th = 120  # 145
        self.frameWidth = 0
        self.frameHeight = 0

        self.blue = (255, 0, 0)
        self.green = (0, 255, 0)
        self.red = (0, 0, 255)

        self.state_subscriber = rospy.Subscriber(state_topic, String, self.state_cb)
        self.img_subscriber = rospy.Subscriber(image_topic, CompressedImage, self.img_cb)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.center_line_publisher = rospy.Publisher(center_lane_topic, Float64, queue_size=10)

        self.state = None

    def img_cb(self, img_msg):

        if self.state != "avoid":
            try:
                # cv_image = CvBridge().imgmsg_to_cv2(img_msg, "bgr8")
                cv_image = CvBridge().compressed_imgmsg_to_cv2(img_msg, "bgr8")
            except CvBridgeError as e:
                print(e)
            else:
                self.process(cv_image)

    def state_cb(self, msg):
        self.state = msg.data

    def map(self, x,input_min,input_max,output_min,output_max):
        return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.

    def simple_controller(self, lx, ly, rx, ry):
        target = 320
        side_margin = 200

        if lx != None and rx != None and len(lx) > 5 and len(rx) > 5:
            print("ALL!!!")
            print(f"lx :{lx[0]}, rx :{rx[0]}")
            target = lx[0] + ((rx[0] - lx[0]) // 2)
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
        #print(frame.shape)

        frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)

        # cv2.imshow("Distort", frame)

        frame = undistort_func(frame)
        #cv2.imshow("Undistort", frame)

        gblur_img  = cv2.GaussianBlur(frame, (3, 3), sigmaX = 0, sigmaY = 0)
        #cv2.imshow("gblur_img", gblur_img)

        warped_img = pre_module.warp_perspect(gblur_img, "gazebo")
        cv2.imshow('warped_img', warped_img)

        left_lane_img, right_lane_img = self.color_filtering(warped_img)

        lane_pixel_img = cv2.add(left_lane_img, right_lane_img)
        #cv2.imshow("lane_pixel_img", lane_pixel_img)

        gray = cv2.cvtColor(lane_pixel_img, cv2.COLOR_BGR2GRAY)

        # binary = self.threshold_binary(gray, self.lane_bin_th, "otsu", window_name="otsu", show=True)
        # cv2.imshow("otsu", binary)

        # edge = self.canny(binary, 70, 210, show=False)

        # kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
        # closing = cv2.morphologyEx(gray, cv2.MORPH_CLOSE,kernel_close)
        # cv2.imshow('closing', closing)	# 프레임 보여주기

        left_lane_gray = cv2.cvtColor(left_lane_img, cv2.COLOR_BGR2GRAY)
        right_lane_gray = cv2.cvtColor(right_lane_img, cv2.COLOR_BGR2GRAY)

        left_msk, lx, ly = pre_module.sliding_window(left_lane_gray, "left")
        right_msk, rx, ry = pre_module.sliding_window(right_lane_gray, "right")

        cv2.imshow('left_msk', left_msk)	# 프레임 보여주기
        cv2.imshow('right_msk', right_msk)	# 프레임 보여주기

        msk = cv2.add(left_msk, right_msk)
        # filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry = pre_module.filtering_lane(msk, lx, ly, mx, my, rx, ry)
        # pre_module.drawing_lane(msk, filtered_lx, filtered_ly, filtered_rx, filtered_ry)
        pre_module.drawing_lane(msk, lx, ly, rx, ry)

        if self.state == "left":
            rx = None
        elif self.state == "right":
            lx = None

        # target = self.simple_controller(filtered_lx, filtered_ly, filtered_rx, filtered_ry)
        target = self.simple_controller(lx, ly, rx, ry)

        # target = 0
        angle = 320 - target
        angle = self.map(angle, 100, -100, 1.0, -1.0)
        # angle = angle * 0.5
        # print(f"angle: {angle}")

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.1 #0.05
        cmd_vel_msg.angular.z = angle
        #self.cmd_vel_publisher.publish(cmd_vel_msg)

        center_line_msg = Float64()
        center_line_msg.data = target
        self.center_line_publisher.publish(center_line_msg)

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

    def color_filtering(self, img):
        hls_low = 220
        hls_high = 255

        lab_low = 190
        lab_high = 255

        hls_lower_white = (0, 235, 0)
        hls_upper_white = (255, 255, 255)

        hls_img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        # 색상 범위를 제한하여 mask 생성
        hls_mask = cv2.inRange(hls_img, hls_lower_white, hls_upper_white)
        # 원본 이미지를 가지고 Object 추출 이미지로 생성
        hls_result = cv2.bitwise_and(img, img, mask=hls_mask)

        # cv2.imshow('origin', img)
        # cv2.imshow('hls_mask', hls_mask)
        # cv2.imshow('hls_result', hls_result)

        lab_lower_yellow= (100, 0, 150)
        lab_upper_yellow = (255, 255, 255)

        lab_img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

        lab_l, lab_a, lab_b = cv2.split(lab_img)

        lab_mask = cv2.inRange(lab_img, lab_lower_yellow, lab_upper_yellow)
        lab_result = cv2.bitwise_and(img, img, mask=lab_mask)
        # cv2.imshow('lab_img', lab_img)
        # cv2.imshow('lab_mask', lab_mask)
        # cv2.imshow('lab_result', lab_result)

        # cv2.imshow('lab_l', lab_l)
        # cv2.imshow('lab_a', lab_a)
        # cv2.imshow('lab_b', lab_b)

        # cv2.imshow('hsv_h', hsv_h)
        # cv2.imshow('hsv_s', hsv_s)
        # cv2.imshow('hsv_v', hsv_v)
        left_lane_img = lab_result 
        right_lane_img = hls_result
        return left_lane_img, right_lane_img

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

    state_topic = "/avoid/state"
    image_topic = "/camera/image/compressed"
    cmd_vel_topic = "/cmd_vel"
    lane_topic = "/detect/lane"

    # lane_detector = Lane_detector(image_topic, cmd_vel_topic)
    lane_detector = Lane_detector(state_topic, image_topic, cmd_vel_topic, lane_topic)
    rospy.spin()
