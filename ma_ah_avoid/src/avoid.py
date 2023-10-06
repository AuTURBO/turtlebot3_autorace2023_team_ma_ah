#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math, tf, time, cv2
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from enum import Enum
from ma_ah_perception.preprocessor import PreProcessor
from ma_ah_perception.undistort import undistort_func

class Mode(Enum):
    LANE = 1
    AVOID= 2
    EXIT = 3

class AvoidNode:
    def __init__(self):

        rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size = 1)

        self.steer_angle = Twist()
        self.center_lane = Float64()

        self.parameter_init()

    def parameter_init(self):
        self.roi_height = 200
        self.roi_width = 640
        self.last_current_theta = 0.0
        self.lane_bin_th = 120  # 145
        self.pre_module = PreProcessor(self.roi_height, self.roi_width)
        self.cv_image = None
        self.is_obstacle_detected = False

    def image_callback(self, msg):
        global lane_bin_th
        try:
            # cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            self.cv_image = CvBridge().compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        #else:
        #    self.lane_detector(cv_image)
            #cv2.imshow("Image window", cv_image)
        #    cv2.waitKey(1)

    def odom_callback(self, odom_msg):
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)

        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y

        self.current_pos = math.sqrt(self.current_pos_x**2 + self.current_pos_y**2)

        #print(self.current_pos_x, self.current_pos_y)

    def scan_callback(self, scan):
        self.checkLeftObstacle(scan)

    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta

    def checkLeftObstacle(self, scan):
        scan_start = 175
        scan_end = 185
        threshold_distance = 0.21
        is_obstacle_detected = False

        for i in range(scan_start, scan_end):
            if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                is_obstacle_detected = True
                print(scan.ranges[i])

        self.is_obstacle_detected = is_obstacle_detected
        if (self.is_obstacle_detected):
            print("obstacle!!!!")

    def lane_detector(self, frame):
        prev_target = 320
        frameRate = 11 #33
        frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_AREA)
        # cv2.imshow("Distort", frame)

        frame = undistort_func(frame)

        cv2.imshow("Undistort", frame)

        gblur_img  = cv2.GaussianBlur(frame, (3, 3), sigmaX = 0, sigmaY = 0)
        #cv2.imshow("gblur_img", gblur_img)

        gray = cv2.cvtColor(gblur_img, cv2.COLOR_BGR2GRAY)
        adaptive_binary = self.threshold_binary(gray, self.lane_bin_th, "otsu", window_name="adaptive_binary", show=True)
        cv2.imshow("adaptive_binary", adaptive_binary)

        warped_img = self.pre_module.warp_perspect(adaptive_binary)
        cv2.imshow('warped_img', warped_img)

        edge = self.canny(warped_img, 70, 210, show=False)

        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(13,13))
        kernel_erosion = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

        closing = cv2.morphologyEx(warped_img, cv2.MORPH_CLOSE,kernel_close)
        cv2.imshow('closing', closing)	# 프레임 보여주기

        # edge_to_closing = cv2.morphologyEx(edge, cv2.MORPH_CLOSE,kernel_close)
        # cv2.imshow('edge_to_closing', edge_to_closing)	# 프레임 보여주기
        # edge_to_closing = cv2.medianBlur(edge_to_closing,5)

        msk, lx, ly, mx, my, rx, ry = self.pre_module.sliding_window(closing)

        filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry = self.pre_module.filtering_lane(msk, lx, ly, mx, my, rx, ry)
        self.pre_module.drawing_lane(msk, filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry)

        target = self.simple_controller(filtered_lx, filtered_ly, filtered_mx, filtered_my, filtered_rx, filtered_ry)

        #target = LowPassFilter(0.9, prev_target, target)
        prev_target = target
        #print(f"filtered_target: {target}")

        angle = 320 - target
        angle = self.map(angle, 100, -100, 1.0, -1.0)
        # angle = angle * 0.5
        # print(f"angle: {angle}")

        self.steer_angle.linear.x = 0.1
        self.steer_angle.angular.z = angle

        self.pub_cmd_vel.publish(self.steer_angle)

        # center lane pub
        self.center_lane.data = (target)
        # center_lane_publisher.publish(center_lane)

        cv2.circle(frame, (int(target), int(480 - 135)), 1, (120, 0, 255), 10)

        cv2.imshow("Lane Detection - Sliding Windows", msk)
        # erosion = cv2.erode(closing,kernel_erosion,iterations = 1)
        # opeing = cv2.morphologyEx(closing, cv2.MORPH_OPEN,kernel_erosion)

        cv2.imshow('frame', frame)	# 프레임 보여주기

        key = cv2.waitKey(frameRate)  # frameRate msec동안 한 프레임을 보여준다

        # 키 입력을 받으면 키값을 key로 저장 -> esc == 27(아스키코드)
        if key == 27:
            os._exit(0)

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

    def simple_controller(self, lx, ly, mx, my, rx, ry):
        target = 320
        side_margin = 140

        if lx != None and rx != None and len(lx) > 5 and len(rx) > 5:
            #print("ALL!!!")
            target = (lx[0] + rx[0]) // 2
        elif mx != None and len(mx) > 3:
            # print("Mid!!!")
            target = mx[0]
        elif lx != None and len(lx) > 3:
            #print("Right!!!")
            #print(f"val: {lx[0]}")
            target = lx[0] + side_margin
        elif rx != None and len(rx) > 3:
            #print("Left!!!")
            target = rx[0] - side_margin

        #print(f"target: {target}")
        return int(target)

    def map(self, x,input_min,input_max,output_min,output_max):
        return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.

    def rotate_90_degrees(self, start_angle, theta):
        while not rospy.is_shutdown():
            current_angle = self.current_theta
            print(current_angle, start_angle)
            if abs(current_angle - start_angle) < math.pi/2:
                self.drive(0.0, theta)
            else:
                self.drive(0.0, 0.0)
                break
            time.sleep(0.1)

    def move_forward(self, distance, start_distance):
        while not rospy.is_shutdown():
            current_distance = self.current_pos
            diff = abs(start_distance - current_distance)
            print( diff)
            if diff <= distance:
                self.drive(0.2, 0.0)
            else:
                self.drive(0.0, 0.0)
                break
            time.sleep(0.1)

    def drive(self, linear_x, angular_z):
        self.steer_angle.linear.x = linear_x
        self.steer_angle.angular.z = angular_z

        self.pub_cmd_vel.publish(self.steer_angle)

    def avoid(self):
        print("AVOID !!!!!")

        self.drive(0.0, 0.0)

        self.rotate_90_degrees(self.current_theta, 0.5)
        self.move_forward(0.21, self.current_pos)
        self.rotate_90_degrees(self.current_theta, -0.5)
        self.move_forward(0.19, self.current_pos)
        self.rotate_90_degrees(self.current_theta, -0.5)
        self.move_forward(0.19, self.current_pos)
        self.rotate_90_degrees(self.current_theta, 0.5)


    def main(self):
        rate = rospy.Rate(10)
        rospy.wait_for_message("/camera/image/compressed", CompressedImage, timeout=10)
        self.current_mode = Mode.LANE
        #self.direction = "left"
        while not rospy.is_shutdown():

            if self.current_mode == Mode.LANE:
                if self.cv_image is not None:
                    self.lane_detector(self.cv_image)

                if self.is_obstacle_detected:
                    self.current_mode = Mode.AVOID

            if self.current_mode == Mode.AVOID:
                self.avoid()
                self.current_mode = Mode.LANE

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('avoid_contruction')
    node = AvoidNode()
    node.main()
