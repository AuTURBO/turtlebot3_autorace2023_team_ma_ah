#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from std_msgs.msg import Float64, String

from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher


import math
 
class ControlLaneState(EventState):
    '''
    Implements a FlexBE state that controls the lane of a robot.
    '''

    def __init__(self):
        super(ControlLaneState, self).__init__(outcomes=['lane_control', 'mission_control'], input_keys=['lane_info', 'pid_info', 'vel_info'])
        
        self.lastError = 0
        self._MAX_VEL = 0.1
        
        self.sub_middle_lane = ProxySubscriberCached({"/detect/middle/lane": Float64})
        self.sub_left_lane = ProxySubscriberCached({"/detect/left/lane": Float64})
        self.sub_right_lane = ProxySubscriberCached({"/detect/right/lane": Float64})
        self.sub_max_vel = ProxySubscriberCached({"/control/max_vel": Float64})

        self.sub_direction_sign = ProxySubscriberCached({"/direction_sign": String})
        self._filtered_detection_sub = ProxySubscriberCached({"/filtered/detection": String})
        self._stop_sign_sub = ProxySubscriberCached({"/stop_sign": String})

        self.pub_cmd_vel = ProxyPublisher({"/cmd_vel": Twist})
        # pure pursuit control
        self.WB = 0.20
        self.Lf = 0.20

        self.prev_error = 0
        self.Kp = 0.0 
        self.Ki = 0.0
        self.Kd = 0.0 

        self.linear_vel = 0.0

    # pid control
    def pid_control(self, desired_center):
        center = desired_center
        error = center - 320
        # 0 - target
        # error = map(error, 100, -100, 2.5, -2.5)


        angular_z = self.Kp * error + self.Kd* (error - self.lastError) + self.Ki * (error + self.lastError)
        self.lastError = error
        
        twist = Twist()
        twist.linear.x =  min(self._MAX_VEL * ((1 - abs(error) / 320) ** 2.2), 0.06)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        
        # self
        # self.pub_cmd_vel.publish(twist)
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
        one_lane_mode_margin = 235
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
                target = left_lane_data + one_lane_mode_margin

        elif direction_sign_data == "right": # Set Mode Only (right lane following)
            Logger.loginfo("See Right lane!!")
            Logger.loginfo("Lane Data type: {}".format(type(right_lane_data)))

            Logger.loginfo("ABS Error: {}".format(abs(right_lane_data - 0.0)))    

            if (abs(right_lane_data - 0.0) <= 0.01) or right_lane_data == 1000 : # if No exist only left lane
                Logger.loginfo("Error except!!")
                target = 640 - no_line_margin
            elif right_lane_data != 1000: # if exist only left lane
                target = right_lane_data - one_lane_mode_margin


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

        p_gain = 0.8 # 0.85
        d_gain = 0.1 #0.2

        error = 320 - target
        Logger.loginfo("target: {}".format(target))
        Logger.loginfo("Angle: {}".format(error))
        print(f"target: {target}")
        error = self.map(error, 100, -100, 1.5, -1.5) # 0.5

        diff = self.prev_error - error
        Logger.loginfo("diff: {}".format(diff))

        
        angle = self.Kp * error + self.Kd * diff + self.Ki * (error + self.prev_error)
        self.prev_error = error

        return float(angle)

    def on_enter(self, userdata):
        Logger.loginfo("Starting lane control...")
        self.Kp = userdata.pid_info[0] 
        self.Ki = userdata.pid_info[1] 
        self.Kd = userdata.pid_info[2] 
        self.linear_vel = userdata.vel_info

            
    def execute(self, userdata):
        
        # lane control logic
        if self.sub_left_lane.has_msg("/detect/left/lane") and self.sub_right_lane.has_msg("/detect/right/lane"):
            left_lane_data = self.sub_left_lane.get_last_msg("/detect/left/lane").data
            right_lane_data = self.sub_right_lane.get_last_msg("/detect/right/lane").data
        
            angle = self.simple_controller(left_lane_data, right_lane_data, userdata.lane_info)

            cmd_vel_msg = Twist()
            
            if self._sub.has_msg("/filtered/detection"):
                self._traffic_sign = self._filtered_detection_sub.get_last_msg("/filtered/detection").data
                if self._traffic_sign == "['boom_barrier']" or self._traffic_sign == "['boom_barrier', 'stop']" or self._traffic_sign == "['stop', 'boom_barrier']" :
                    cmd_vel_msg.linear.x = 0.0
                    Logger.loginfo("Stop!!!")
                else:
                    cmd_vel_msg.linear.x = self.linear_vel # 0.15
            # cmd_vel_msg = Twist()
            # cmd_vel_msg.linear.x = self.linear_vel # 0.15
            cmd_vel_msg.angular.z = angle 
            self.pub_cmd_vel.publish("/cmd_vel", cmd_vel_msg)

        # When robot detect any traffic sign
        if self._sub.has_msg("/filtered/detection"):
            self._traffic_sign = self._filtered_detection_sub.get_last_msg("/filtered/detection").data
            Logger.loginfo("Traffic_sign: {}".format(self._traffic_sign))
            Logger.loginfo("-------")
            Logger.loginfo("userdata.lane_info: {}".format(userdata.lane_info))
            

            # recursion lane control
            if self._traffic_sign == "[]" or self._traffic_sign == "['stop']" or self._traffic_sign == "['boom_barrier']" or self._traffic_sign == "['boom_barrier', 'stop']" or self._traffic_sign == "['stop', 'boom_barrier']":
                Logger.loginfo("lane control recursion")
                return 'lane_control'
     
                #self.pid_control(desired_center)
            else: # move mission_control
                Logger.loginfo("start mission control")
                return 'mission_control'
        


    def on_exit(self, userdata):

        pass # 이 예시에서는 할 일이 없습니다.


    def on_start(self):

        pass

    def on_stop(self):
        pass # 이 예시에서는 할 일이 없습니다.