#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from std_msgs.msg import Float64

from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher

from std_msgs.msg import String

import math
 
class ControlLaneState(EventState):
    '''
    Implements a FlexBE state that controls the lane of a robot.
    '''

    def __init__(self):
        super(ControlLaneState, self).__init__(outcomes=['lane_control', 'mission_control'], input_keys=['lane_info'])
        
        self.lastError = 0
        self._MAX_VEL = 0.5
        
        self.sub_middle_lane = ProxySubscriberCached({"/detect/middle/lane": Float64})
        self.sub_middle_lane = ProxySubscriberCached({"/detect/left/lane": Float64})
        self.sub_middle_lane = ProxySubscriberCached({"/detect/right/lane": Float64})
        self.sub_max_vel = ProxySubscriberCached({"/control/max_vel": Float64})
        self.pub_cmd_vel = ProxyPublisher({"/cmd_vel": Twist})
        self.sub_traffic_sign = ProxySubscriberCached({"/traffic_sign": String})
        self.sub_traffic_sign_size = ProxySubscriberCached({"/traffic_sign_size": Float64})
    
        # pure pursuit control
        self.WB = 0.20
        self.Lf = 0.20


    # pid control
    def pid_control(self, desired_center):
        center = desired_center
        error = center - 320
        # 0 - target
        # error = map(error, 100, -100, 2.5, -2.5)
        Kp = 0.013
        Ki = 0.001
        Kd = 0.006

        angular_z = Kp * error + Kd * (error - self.lastError) + Ki * (error)
        self.lastError = error
        
        twist = Twist()
        twist.linear.x =  min(self._MAX_VEL * ((1 - abs(error) / 320) ** 2.2), 0.06)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        # self
        # self.pub_cmd_vel.publish(twist)
        self.pub_cmd_vel.publish("/cmd_vel", twist)
        Logger.loginfo("Following lane...")

    


    def on_enter(self, userdata):
        Logger.loginfo("Starting lane control...")

            

    def execute(self, userdata):
        if self.sub_traffic_sign_size.has_msg("/traffic_sign_size") < 60:
            Logger.loginfo("Traffic sign size: {}".format(self.sub_traffic_sign_size.get_last_msg("/traffic_sign_size").data))
            if userdata.lane_info == 'left':
                desired_center = self.sub_middle_lane.get_last_msg("/detect/left/lane").data
            elif userdata.lane_info == 'right':
                desired_center = self.sub_middle_lane.get_last_msg("/detect/right/lane").data
            else:
                desired_center = self.sub_middle_lane.get_last_msg("/detect/middle/lane").data
            # pid lane control
            self.pid_control(desired_center)
            return 'lane_control'
        else:
            Logger.loginfo("start mission control")
            return 'mission_control'
        


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