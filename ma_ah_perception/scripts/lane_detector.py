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
import sys, time

roi_height = 200
roi_width = 640

pre_module = PreProcessor(roi_height, roi_width)


class Lane_detector:
    def __init__(self, img_topic, cmd_vel_topic, center_lane_topic, left_lane_topic, right_lane_topic, processed_img_topic):
        self.lane_bin_th = 200  # 145
        self.frameWidth = 0
        self.frameHeight = 0
        self.prevTime = 0

        self.blue = (255, 0, 0)
        self.green = (0, 255, 0)
        self.red = (0, 0, 255)

        self.img_subscriber = rospy.Subscriber(img_topic, CompressedImage, self.img_cb)
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.cener_line_publisher = rospy.Publisher(center_lane_topic, Float64, queue_size=10)

        self.processed_img_publisher = rospy.Publisher(processed_img_topic, Image, queue_size=10)

        self.left_lane_publisher = rospy.Publisher(left_lane_topic, Float64, queue_size=10)
        self.right_lane_publisher = rospy.Publisher(right_lane_topic, Float64, queue_size=10)

        self.cv_image = None
    
        cv2.namedWindow("Track_bar")
 
        cv2.createTrackbar("hls_l_low", "Track_bar",0, 255, self.h_change)
        cv2.setTrackbarPos("hls_l_low", "Track_bar", 127)

        # cv2.createTrackbar("lab_l_low", "Track_bar",0, 255, lambda x:x)
        # cv2.setTrackbarPos("lab_l_low", "Track_bar", 127)

        # cv2.createTrackbar("lab_l_high", "Track_bar",0, 255, lambda x:x)
        # cv2.setTrackbarPos("lab_l_high", "Track_bar", 127)



    def img_cb(self, img_msg):
        try:
            # cv_image = CvBridge().imgmsg_to_cv2(img_msg, "bgr8")
            self.cv_image = CvBridge().compressed_imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            pass
            #cv2.imshow('callback', cv_image)	# 프레임 보여주기
            # output_img = self.process(cv_image)

            # curTime = time.time()

            # sec = curTime - self.prevTime
            # self.prevTime = curTime

            # # 1 / time per frame
            # fps = 1/(sec)

            # # print(f"Time {sec} ")
            # # print(f"Estimated fps {fps} ") 

            # str = "FPS : %0.1f" % fps

            
            # cv2.putText(output_img, str, (0, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255))

            # cv2.imshow('lane_detector_frame', output_img)	# 프레임 보여주기


    def map(self, x,input_min,input_max,output_min,output_max):
        return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.

    def simple_controller(self, lx, ly, rx, ry):
        target = 320
        side_margin = 220

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

    def lane_publish(self, lx, ly, rx, ry):
        left_line_msg = Float64()
        right_line_msg = Float64()

        none_value = 1000
    
        if lx != None and rx != None and len(lx) > 5 and len(rx) > 5:
            left_line_msg.data = lx[1]
            right_line_msg.data = rx[1]
        elif lx != None and len(lx) > 3:
            left_line_msg.data = lx[1]
            right_line_msg.data = none_value
        elif rx != None and len(rx) > 3:
            left_line_msg.data = none_value
            right_line_msg.data = rx[1]

        self.left_lane_publisher.publish(left_line_msg)
        self.right_lane_publisher.publish(right_line_msg)

    def h_change(value):
        pass

    def process(self):
        if self.cv_image is not None:

            frame = self.cv_image
            #print(frame.shape)

            frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)

            # cv2.imshow("Distort", frame)

            # frame = undistort_func(frame)
            #cv2.imshow("Undistort", frame)

            gblur_img  = cv2.GaussianBlur(frame, (3, 3), sigmaX = 0, sigmaY = 0)
            #cv2.imshow("gblur_img", gblur_img)

            warped_img = pre_module.warp_perspect(gblur_img, "usb_cam")
            #cv2.imshow('warped_img', warped_img)	

            left_lane_img, right_lane_img = self.color_filtering(warped_img)

            left_lane_gray = cv2.cvtColor(left_lane_img, cv2.COLOR_BGR2GRAY)

            left_lane_gray = self.threshold_binary(left_lane_gray, self.lane_bin_th, "basic", window_name="otsu", show=True)
            
            right_lane_gray = cv2.cvtColor(right_lane_img, cv2.COLOR_BGR2GRAY)

            right_lane_gray = self.threshold_binary(right_lane_gray, self.lane_bin_th, "basic", window_name="otsu", show=True)

            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
            left_lane_gray_close = cv2.morphologyEx(left_lane_gray, cv2.MORPH_CLOSE,kernel_close)
            #cv2.imshow("left_lane_gray_close", left_lane_gray_close)


            lane_pixel_img = cv2.add(left_lane_gray, right_lane_gray)
            #cv2.imshow("lane_pixel_img", lane_pixel_img)

            # gray = cv2.cvtColor(lane_pixel_img, cv2.COLOR_BGR2GRAY)
            
            # binary = self.threshold_binary(gray, self.lane_bin_th, "otsu", window_name="otsu", show=True)
            # cv2.imshow("otsu", binary)

            # edge = self.canny(binary, 70, 210, show=False)

            # left_lane_gray = cv2.cvtColor(left_lane_img, cv2.COLOR_BGR2GRAY)
            # right_lane_gray = cv2.cvtColor(right_lane_img, cv2.COLOR_BGR2GRAY)

            # left_lane_gray = self.threshold_binary(left_lane_gray, self.lane_bin_th, "otsu", window_name="otsu", show=False)

            # kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
            # left_lane_gray_close = cv2.morphologyEx(left_lane_gray, cv2.MORPH_CLOSE,kernel_close)
            # cv2.imshow('left_lane_gray_close', left_lane_gray_close)	# 프레임 보여주기

            #left_lane_gray = self.threshold_binary(left_lane_gray, self.lane_bin_th, "otsu", window_name="otsu", show=False)

            left_msk, lx, ly = pre_module.sliding_window(left_lane_gray, "left")
            right_msk, rx, ry = pre_module.sliding_window(right_lane_gray, "right")

            #cv2.imshow('left_msk', left_msk)	# 프레임 보여주기
            #cv2.imshow('right_msk', right_msk)	# 프레임 보여주기

            msk = cv2.add(left_msk, right_msk)
            # filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry = pre_module.filtering_lane(msk, lx, ly, mx, my, rx, ry)
            # pre_module.drawing_lane(msk, filtered_lx, filtered_ly, filtered_rx, filtered_ry)
            pre_module.drawing_lane(msk, lx, ly, rx, ry)

            # target = self.simple_controller(filtered_lx, filtered_ly, filtered_rx, filtered_ry)
            target = self.simple_controller(lx, ly, rx, ry)

            # target = 0
            angle = 320 - target
            angle = self.map(angle, 100, -100, 2.0, -2.0) # 0.5
            # angle = angle * 0.5
            # print(f"angle: {angle}")

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0 # 0.2 # 0.1
            cmd_vel_msg.angular.z = 0 #angle

            #self.cmd_vel_publisher.publish(cmd_vel_msg)

            self.lane_publish(lx, ly, rx, ry)

            cv2.circle(frame, (int(target), int(480 - 135)), 1, (120, 0, 255), 10)

            cv2.imshow("Lane Detection - Sliding Windows", msk)
            cv2.imshow('lane_detector_frame', frame)	# 프레임 보여주기

            key = cv2.waitKey(1)  # frameRate msec동안 한 프레임을 보여준다
                
            # 키 입력을 받으면 키값을 key로 저장
            if key == ord('q'):
                sys.exit(0)

            #return frame

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
        hls_l_low =  cv2.getTrackbarPos("hls_l_low", "Track_bar")

        # lab_l_low = cv2.getTrackbarPos("lab_l_low", "Track_bar")
        # lab_l_high = cv2.getTrackbarPos("lab_l_high", "Track_bar")
        #hls_l = cv2.getTrackbarPos("h", "Track_bar")

        lab_low = 190
        lab_high = 255
        
        hls_lower_white = (0, 228, 0)
        hls_upper_white = (255, 255, 30)

        hls_img = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        # 색상 범위를 제한하여 mask 생성
        hls_mask = cv2.inRange(hls_img, hls_lower_white, hls_upper_white)
        # 원본 이미지를 가지고 Object 추출 이미지로 생성
        hls_result = cv2.bitwise_and(img, img, mask=hls_mask)

        # cv2.imshow('origin', img)
        # cv2.imshow('hls_mask', hls_mask)
        # cv2.imshow('hls_result', hls_result)

        lab_lower_yellow= (0, 0, 130)
        lab_upper_yellow = (255, 255, 180)

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
    
    image_topic = "/camera/image/compressed"
    cmd_vel_topic = "/cmd_vel"
    center_lane_topic = "/detect/lane"
    processed_img_topic = "/processed/img"

    left_lane_topic = "/detect/left/lane"
    right_lane_topic = "/detect/right/lane"

    # lane_detector = Lane_detector(image_topic, cmd_vel_topic)
    lane_detector = Lane_detector(image_topic, cmd_vel_topic, center_lane_topic, left_lane_topic, right_lane_topic, processed_img_topic)

    while True:
        lane_detector.process() 
    rospy.spin()
