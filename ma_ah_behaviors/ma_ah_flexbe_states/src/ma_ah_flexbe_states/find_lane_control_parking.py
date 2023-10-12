#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

class ControlLaneFindParkingState(EventState):
    '''
    Example for a state to detect parking spots.
    This state listens to a topic for parking information and reacts accordingly.

    <= parking_proceed     Parking spot available.
    <= no_parking_spot            No parking spot available.

    '''

    def __init__(self):
        # Declare outcomes by calling the super constructor with the corresponding arguments.
        super(ControlLaneFindParkingState, self).__init__(outcomes=['left', 'right', 'proceed'], input_keys=['lane_info'])

        # Initialize class variables or state parameters here if needed.
        self.sub_scan = ProxySubscriberCached({"/scan": LaserScan})

        self.is__left_obstacle_detected = False
        self.is__right_obstacle_detected = False


        
        self.lastError = 0
        self._MAX_VEL = 0.1
        
        self.sub_middle_lane = ProxySubscriberCached({"/detect/middle/lane": Float64})
        self.sub_left_lane = ProxySubscriberCached({"/detect/left/lane": Float64})
        self.sub_right_lane = ProxySubscriberCached({"/detect/right/lane": Float64})
        self.sub_max_vel = ProxySubscriberCached({"/control/max_vel": Float64})

        self.pub_cmd_vel = ProxyPublisher({"/cmd_vel": Twist})
        
        # pure pursuit control
        self.WB = 0.20
        self.Lf = 0.20



    def checkObstacle(self):
        if self.sub_scan.has_msg("/scan"):
            scan = self.sub_scan.get_last_msg("/scan")
            left_scan_start = 85
            left_scan_end = 95
            right_scan_start = 265
            right_scan_end = 275
            threshold_distance = 0.2
            is_left_obstacle_detected = False
            is_right_obstacle_detected = False
            for i in range(left_scan_start, left_scan_end):
                if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                    is_left_obstacle_detected = True

                    # print(scan.ranges[i])
                    break
            
            for i in range(right_scan_start, right_scan_end):
                if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                    is_right_obstacle_detected = True

                    # print(scan.ranges[i])
                    break
            
            self.is__left_obstacle_detected = is_left_obstacle_detected
            self.is__right_obstacle_detected = is_right_obstacle_detected



    # pid control
    def pid_control(self, desired_center):
        center = desired_center
        error = center - 320
        # 0 - target
        # error = map(error, 100, -100, 2.5, -2.5)
        Kp = 0.015
        Ki = 0.000
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error
        
        twist = Twist()
        twist.linear.x =  min(self._MAX_VEL * ((1 - abs(error) / 320) ** 2.2), 0.06)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        
        self.pub_cmd_vel.publish("/cmd_vel", twist)
        Logger.loginfo("Following lane...")

    # pure pursuit control
    def pure_pursuit_control(self, desired_center):
        error = desired_center - 310
        current_angle = error
        # 현재 조향각인 current_angle과 목표값(0으로 가정)을 이용하여 알고리즘을 적용합니다.
        # 다음 줄을 알고리즘에 맞게 수정해주세요.
        target_angle = 0  # -5  # 여기에 Pure Pursuit 알고리즘을 적용하여 목표 조향각을 계산합니다.

        # 현재 조향각과 목표 조향각과의 차이를 계산합니다.
        self.diff_angle = target_angle - current_angle

        # degree to radian
        self.diff_angle = self.diff_angle * math.pi / 180

        angular_z = math.atan2(2.0 * self.WB * math.sin(self.diff_angle), self.Lf)
        print("angular_z: ", angular_z)

        twist = Twist()
        twist.linear.x = min(self._MAX_VEL * ((1 - abs(error) / 320) ** 2.2), 0.05)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish("/cmd_vel", twist)

        # Logger.loginfo("Following lane...")

    def map(self, x,input_min,input_max,output_min,output_max):
        return (x-input_min)*(output_max-output_min)/(input_max-input_min)+output_min #map()함수 정의.

    # Minwoo's controller 
    def simple_controller(self, left_lane_data, right_lane_data, direction_sign_data):
        target = 320
        side_margin = 220
        Logger.loginfo("Simple controller user_info: {}".format(direction_sign_data))
        Logger.loginfo("Left lane:  {}".format(left_lane_data))
        Logger.loginfo("Right lane: {}".format(right_lane_data))
        no_line_margin = 280
        if direction_sign_data == "left": # Set Mode (left lane following)
            Logger.loginfo("See Left lane!!") 
            #target = left_lane_data + side_margin

            if (abs(left_lane_data - 0.0) <= 0.01) or left_lane_data == 1000 : # if exist only left lane
                Logger.loginfo("Error except!!")
                target = no_line_margin
            elif left_lane_data != 1000 : # if No exist only left lane
                target = left_lane_data + side_margin

        elif direction_sign_data == "right": # Set Mode Only (right lane following)
            Logger.loginfo("See Right lane!!")
            Logger.loginfo("Lane Data type: {}".format(type(right_lane_data)))

            Logger.loginfo("ABS Error: {}".format(abs(right_lane_data - 0.0)))    

            if (abs(right_lane_data - 0.0) <= 0.01) or right_lane_data == 1000 : # if No exist only left lane
                Logger.loginfo("Error except!!")
                target = 640 - no_line_margin
            elif right_lane_data != 1000: # if exist only left lane
                target = right_lane_data - side_margin


        # 1000 is represent None value 

        elif direction_sign_data == "middle": # Set Mode only (two lane following)
            Logger.loginfo("See Middle lane!!")
            if left_lane_data != 1000 and right_lane_data != 1000: # if exist two lane
                Logger.loginfo("Autonomous ALL!!!")
                target = left_lane_data + ((right_lane_data - left_lane_data) // 2)
            elif left_lane_data != 1000 and right_lane_data == 1000: # if exist only left lane
                Logger.loginfo("Autonomous See Left Lane!!!")
                target = left_lane_data + side_margin
            elif left_lane_data == 1000 and right_lane_data != 1000: # if exist only right lane
                Logger.loginfo("Autonomous See Right Lane!!!!!")
                target = right_lane_data - side_margin
            elif left_lane_data == 1000 and right_lane_data == 1000: # if don't exist lane
                target = 0

        angle = 320 - target
        Logger.loginfo("target: {}".format(target))
        Logger.loginfo("Angle: {}".format(angle))
        print(f"target: {target}")
        angle = self.map(angle, 100, -100, 1.5, -1.5) # 0.5
        print(f"angle: {angle}")
        return float(angle)



    def on_enter(self, userdata):
        Logger.loginfo("Starting obstacle...")

            
    def execute(self, userdata):
        
        # lane control logic
        self.checkObstacle()

        if self.is__left_obstacle_detected == False and self.is__right_obstacle_detected == False:
            left_lane_data = self.sub_left_lane.get_last_msg("/detect/left/lane").data
            right_lane_data = self.sub_right_lane.get_last_msg("/detect/right/lane").data


            angle = self.simple_controller(left_lane_data, right_lane_data, userdata.lane_info)
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.05 # 0.1
            cmd_vel_msg.angular.z = 0.0 #angle
            self.pub_cmd_vel.publish("/cmd_vel", cmd_vel_msg)
            return 'proceed'


        elif self.is__left_obstacle_detected == True and self.is__right_obstacle_detected == False:
            Logger.loginfo('Left obstacle detected')
            return 'left'
        elif self.is__left_obstacle_detected == False and self.is__right_obstacle_detected == True:
            Logger.loginfo('Right obstacle detected')
            return 'right'
                # self.pid_control(target) 

    def on_exit(self, userdata):

        pass # 이 예시에서는 할 일이 없습니다.


    def on_start(self):

        pass

    def on_stop(self):
        pass # 이 예시에서는 할 일이 없습니다.