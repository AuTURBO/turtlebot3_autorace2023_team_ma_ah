#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher

from std_msgs.msg import String


class ControlLaneState22(EventState):
    '''
    Implements a FlexBE state that controls the lane of a robot.
    '''

    def __init__(self):
        super(ControlLaneState22, self).__init__(outcomes=['proceed', 'traffic_sign'])
        
        self.lastError = 0
        self._MAX_VEL = 0.5
        
        self._sub = ProxySubscriberCached({"/detect/lane": Float64})
        self.sub_max_vel = ProxySubscriberCached({"/control/max_vel": Float64})
        self.pub_cmd_vel = ProxyPublisher({"/cmd_vel": Twist})
        self.sub_traffic_sign = ProxySubscriberCached({"/traffic_sign": String})

    def on_enter(self, userdata):
        Logger.loginfo("Starting lane control...")
        # if self._sub.has_msg("/traffic_sign"):
            

    def execute(self, userdata):
        if self._sub.has_msg("/detect/lane"):
            desired_center = self._sub.get_last_msg("/detect/lane").data
            # pid lane control
            self.pid_control(desired_center)
            # pure pursuit lane control
            # self.pure_pursuit_control(desired_center)
            return 'proceed'
        

            angular_z = Kp * error + Kd * (error - self.lastError)
            self.lastError = error
            
            twist = Twist()
            twist.linear.x = min(self._MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05)
            twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
            # self
            # self.pub_cmd_vel.publish(twist)
            self.pub_cmd_vel.publish("/cmd_vel", twist)
            Logger.loginfo("Following lane...")
            return 'proceed'
        elif self.sub_traffic_sign.has_msg("/traffic_sign"):
            traffic_sign = self.sub_traffic_sign.get_last_msg("/traffic_sign").data
            Logger.loginfo("Traffic sign: {}".format(traffic_sign))
            return 'traffic_sign'
    

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