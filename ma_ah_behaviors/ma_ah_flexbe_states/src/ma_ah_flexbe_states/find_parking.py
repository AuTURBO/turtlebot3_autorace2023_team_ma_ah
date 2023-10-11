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

class FindParkingState(EventState):
    '''
    Example for a state to detect parking spots.
    This state listens to a topic for parking information and reacts accordingly.

    <= parking_proceed     Parking spot available.
    <= no_parking_spot            No parking spot available.

    '''

    def __init__(self):
        # Declare outcomes by calling the super constructor with the corresponding arguments.
        super(FindParkingState, self).__init__(outcomes=['left', 'right', 'proceed'])

        # Initialize class variables or state parameters here if needed.
        self.sub_scan = ProxySubscriberCached({"/scan": LaserScan})
        self.pub_cmd_vel = ProxyPublisher({"/cmd_vel": Twist})

        self.is__left_obstacle_detected = False
        self.is__right_obstacle_detected = False

    def checkObstacle(self):
        scan = self.sub_scan.get_last_msg("/scan")
        left_scan_start = 85
        left_scan_end = 95
        right_scan_start = 265
        right_scan_end = 275
        
        threshold_distance = 0.5
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



    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Its main purpose is to check the condition of the state and trigger the corresponding outcome.
        # If no outcome is returned, the state will stay active.
        if self.is__left_obstacle_detected == False and self.is__right_obstacle_detected == False:
            Logger.loginfo('No obstacle detected')
            twist = Twist()
            twist.linear.x = 0.2
            self.pub_cmd_vel.publish("/cmd_vel", twist)
            return 'proceed'
        elif self.is__left_obstacle_detected == True and self.is__right_obstacle_detected == False:
            Logger.loginfo('Left obstacle detected')
            return 'left'
        elif self.is__left_obstacle_detected == False and self.is__right_obstacle_detected == True:
            Logger.loginfo('Right obstacle detected')
            return 'right'
        
        
    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e., when transitioning to this state.
        # It is typically used to start actions related to this state.
        Logger.loginfo('Entered state Parking')
        self.checkObstacle()

    # You can define other state lifecycle methods like on_exit, on_start, and on_stop if needed.

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