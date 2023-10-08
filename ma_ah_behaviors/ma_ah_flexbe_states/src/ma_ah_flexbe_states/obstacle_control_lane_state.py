#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from std_msgs.msg import Float64, String
from sensor_msgs.msg import LaserScan  


from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher


import math
 
class ObstacleControlLaneState(EventState):
    '''
    Implements a FlexBE state that controls the lane of a robot.
    '''

    def __init__(self):
        super(ObstacleControlLaneState, self).__init__(outcomes=['obstacle_lane_control', 'obstacle_mission_control'], input_keys=['lane_info'])
        
        self.lastError = 0
        self._MAX_VEL = 0.1
        
        self.sub_middle_lane = ProxySubscriberCached({"/detect/lane": Float64})
        self.sub_left_lane = ProxySubscriberCached({"/detect/lane": Float64})
        self.sub_right_lane = ProxySubscriberCached({"/detect/lane": Float64})
        self.sub_max_vel = ProxySubscriberCached({"/control/max_vel": Float64})
        self.pub_cmd_vel = ProxyPublisher({"/cmd_vel": Twist})

        self.sub_lane_detector_cmd_vel = ProxySubscriberCached({"/lane_detector/cmd_vel": Twist})

        self.sub_scan = ProxySubscriberCached({"/scan": LaserScan})

        self.is_obstacle_detected = False

        # pure pursuit control
        self.WB = 0.20
        self.Lf = 0.20

    def checkObstacle(self):
        scan = self.sub_scan.get_last_msg("/scan")
        scan_start = 175
        scan_end = 185
        threshold_distance = 0.23
        is_obstacle_detected = False

        for i in range(scan_start, scan_end):
            if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                is_obstacle_detected = True
                # print(scan.ranges[i])

        self.is_obstacle_detected = is_obstacle_detected



    # pid control
    def pid_control(self, desired_center):
        center = desired_center
        error = center - 310
        # 0 - target
        # error = map(error, 100, -100, 2.5, -2.5)
        Kp = 0.013
        Ki = 0.00
        Kd = 0.00

        angular_z = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error
        
        twist = Twist()
        twist.linear.x =  min(self._MAX_VEL * ((1 - abs(error) / 310) ** 2.2), 0.1)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        
        # self
        # self.pub_cmd_vel.publish(twist)
        self.pub_cmd_vel.publish("/cmd_vel", twist)
        Logger.loginfo("Following lane...")

    # pure pursuit control
    def pure_pursuit_control(self, desired_center):
        error = desired_center - 320
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



    def on_enter(self, userdata):
        Logger.loginfo("Starting lane control...")

            

    def execute(self, userdata):
        if self.is_obstacle_detected == False:
            # Logger.loginfo("Traffic sign size: {}".format(self.sub_traffic_sign_size.get_last_msg("/traffic_sign_size").data))
            if userdata.lane_info == 'left':
                desired_center = self.sub_middle_lane.get_last_msg("/detect/lane").data
            elif userdata.lane_info == 'right':
                desired_center = self.sub_left_lane.get_last_msg("/detect/lane").data
            else:
                desired_center = self.sub_right_lane.get_last_msg("/detect/lane").data
            self.pid_control(desired_center)
            # lane detector calculated cmd_vel (minwoo logic)
            # lane_detector_cmd_vel = self.sub_lane_detector_cmd_vel.get_last_msg("/lane_detector/cmd_vel")
            # self.pub_cmd_vel.publish("/cmd_vel", lane_detector_cmd_vel)
            self.checkObstacle()
            # Logger.loginfo("Following lane...")
            return 'obstacle_lane_control'
        

        else:
            return 'obstacle_mission_control'
        


    def on_exit(self, userdata):
        # 이 메서드는 결과가 반환되고 다른 상태가 활성화될 때 호출됩니다.
        # on_enter에서 시작된 실행 중인 프로세스를 중지하는 데 사용할 수 있습니다.

        pass # 이 예시에서는 할 일이 없습니다.


    def on_start(self):
        # 이 메서드는 행동이 시작될 때 호출됩니다.
        # 가능하면, 일반적으로 사용된 리소스를 생성자에서 초기화하는 것이 더 좋습니다
        # 왜냐하면 무언가 실패하면 행동은 시작조차 되지 않을 것이기 때문입니다.

        # 이 예시에서는 이 이벤트를 사용하여 올바른 시작 시간을 설정합니다.
        pass

    def on_stop(self):
        # 이 메서드는 행동이 실행을 중지할 때마다 호출됩니다, 취소된 경우에도 마찬가지입니다.
        # 이 이벤트를 사용하여 요청된 리소스와 같은 것들을 정리하세요.

        pass # 이 예시에서는 할 일이 없습니다.