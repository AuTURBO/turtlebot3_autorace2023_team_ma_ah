#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math, tf, os
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from enum import Enum
from ma_ah_perception.lane_detector import Lane_detector

class Mode(Enum):
    LANE = 1
    AVOID= 2
    EXIT = 3

class AvoidNode:
    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('/detect/lane', Float64, self.lane_callback, queue_size=1)

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.pub_state = rospy.Publisher('/avoid/state', String, queue_size=1)

        self.steer_angle = Twist()

        self.parameter_init()

    def parameter_init(self):
        self.roi_height = 200
        self.roi_width = 640
        self.last_current_theta = 0.0
        self.lane_bin_th = 120  # 145
        state_topic = "/avoid/state"
        image_topic = "/camera/image/compressed"
        cmd_vel_topic = "/cmd_vel"
        lane_topic = "/detect/lane"
        self.lane_detect_module = Lane_detector(state_topic, image_topic, cmd_vel_topic, lane_topic)
        self.lane_msg = Float64()
        self.state_msg = String()
        self.is_obstacle_detected = False

    def odom_callback(self, odom_msg):
        # 현재 robot의 yaw 값을 current_theta에 저장
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

        # 현재 robot의 위치를 current_pos에 저장

        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y
        self.current_pos = math.sqrt(self.current_pos_x**2 + self.current_pos_y**2)

    def scan_callback(self, scan):
        self.checkObstacle(scan)

    def lane_callback(self, msg):
        self.target_pos = msg.data

    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta

    # 전방 장애물 확인을 위한 함수
    # scan 범위의 데이터가 threshold_distance 보다 작은 값이 나올 때
    # 장애물을 검출로 판단
    def checkObstacle(self, scan):
        scan_start = 175
        scan_end = 185
        threshold_distance = 0.21
        is_obstacle_detected = False

        for i in range(scan_start, scan_end):
            if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                is_obstacle_detected = True
                print(scan.ranges[i])

        self.is_obstacle_detected = is_obstacle_detected
        #if (self.is_obstacle_detected):
        #    print("obstacle!!!!")

    def simple_controller(self, lx, rx):
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

    def map(self, x,input_min,input_max,output_min,output_max):
        return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.

    # odom 정보를 받아서 원하는 각도만큼 회전하는 함수
    # angular_vel는 각속도
    # target_angle만큼 회전하도록함
    # 오차범위가 있을경우 임의로 숫자를 더하거나 빼줌
    def rotate_odom(self, start_ang, target_ang, angular_vel):
        while not rospy.is_shutdown():
            current_angle = self.current_theta
            print(current_angle, start_ang)
            if abs(current_angle - start_ang) < target_ang:
                self.drive(0.0, angular_vel)
            else:
                self.drive(0.0, 0.0)
                break

    # odom 정보를 받아서 원하는 만큼 전진하는 함수
    # 시작 좌표에서 distance 만큼 차이나는곳까지 전진
    def move_forward_odom(self, start_dist, target_dist, linear_vel):
        while not rospy.is_shutdown():
            current_distance = self.current_pos
            diff = abs(start_dist - current_distance)
            #print( diff)
            if diff <= target_dist:
                self.drive(linear_vel, 0.0)
            else:
                self.drive(0.0, 0.0)
                break

    def drive(self, linear_x, angular_z):
        self.steer_angle.linear.x = linear_x
        self.steer_angle.angular.z = angular_z

        self.pub_cmd_vel.publish(self.steer_angle)

    # 장애물 회피 함수
    # 회피할 방향에 따라 odom 정보를 사용하여 이동하는 함수
    def avoid(self, lane_direction):
        print("AVOID !!!!!")

        self.drive(0.0, 0.0)

        if lane_direction == "left":
            self.rotate_odom(self.current_theta, math.radians(80), -0.5)
            self.move_forward_odom(self.current_pos, 0.2, 0.2)
            self.rotate_odom(self.current_theta, math.radians(80), 0.5)
            self.lane_direction = "exit"

        if lane_direction == "right":
            self.rotate_odom(self.current_theta, math.radians(80), 0.5)
            self.move_forward_odom(self.current_pos, 0.2, 0.2)
            self.rotate_odom(self.current_theta, math.radians(80), -0.5)
            self.lane_direction = "left"

    def lane_following(self, target_pos):

        angle = 320 - target_pos
        angle = self.map(angle, 100, -100, 1.0, -1.0)

        self.steer_angle.linear.x = 0.1
        self.steer_angle.angular.z = angle
        self.pub_cmd_vel.publish(self.steer_angle)


    def main(self):
        rate = rospy.Rate(30)
        rospy.wait_for_message("/detect/lane", Float64, timeout=10)
        self.current_mode = Mode.LANE # 시작 모드
        self.lane_direction = "right" # 시작 후 보고 갈 차선의 방향
        while not rospy.is_shutdown():

            if self.current_mode == Mode.LANE:
                self.state_msg.data = self.lane_direction
                self.pub_state.publish(self.lane_direction)

                self.lane_following(self.target_pos)

                if self.is_obstacle_detected:
                    self.current_mode = Mode.AVOID

            if self.current_mode == Mode.AVOID:
                state = "avoid"
                self.pub_state.publish(state)
                self.avoid(self.lane_direction) # direction을 넣어주어 회피방향을 알려준다. ex) 오른 차선 보고 가면 왼쪽으로 회피
                self.is_obstacle_detected = False # 회피가 끝나면 장애물 검출 변수를 False로 원복
                self.current_mode = Mode.LANE

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('avoid_contruction')
    node = AvoidNode()
    node.main()