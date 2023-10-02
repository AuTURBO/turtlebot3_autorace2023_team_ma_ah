#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import numpy as np
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher
from std_msgs.msg import String
from enum import Enum
from turtlebot3_autorace_msgs.msg import MovingParam



class ParkingState(EventState):
    '''
    Example for a state to detect parking spots.
    This state listens to a topic for parking information and reacts accordingly.

    <= parking_proceed     Parking spot available.
    <= no_parking_spot            No parking spot available.

    '''

    def __init__(self):
        # Declare outcomes by calling the super constructor with the corresponding arguments.
        super(ParkingState, self).__init__(outcomes=['parking_proceed', 'done'])

        # Initialize class variables or state parameters here if needed.
        self._sub = ProxySubscriberCached({"/parking_detection": String})
        self._parking_proceed = False

        # subscribes state 
        self.sub_scan_obstacle = rospy.Subscriber('/detect/scan', LaserScan, self.cbScanObstacle, queue_size=1)
        
        # publishes state
        self.pub_parking_return = rospy.Publisher('/detect/parking_stamped', UInt8, queue_size=1)
        self.pub_moving = rospy.Publisher('/control/moving/state', MovingParam, queue_size= 1)
        self.pub_max_vel = rospy.Publisher('/control/max_vel', Float64, queue_size = 1)

        self.StepOfParking = Enum('StepOfParking', 'parking exit')
        self.start_obstacle_detection = False
        self.is_obstacle_detected_R = False
        self.is_obstacle_detected_L = False


    # def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Its main purpose is to check the condition of the state and trigger the corresponding outcome.
        # If no outcome is returned, the state will stay active.

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Its main purpose is to check the condition of the state and trigger the corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._sub.has_msg("/parking_detection"):
            parking_info = self._sub.get_last_msg("/parking_detection").data
            Logger.loginfo("Parking info: {}".format(parking_info))

            # Assuming 'parking_info' is a string that indicates parking spot availability.
            if parking_info == "parking_proceed":
                self._parking_proceed = True
                Logger.loginfo("go straight")
                msg_moving = MovingParam()
                msg_moving.moving_type= 3
                msg_moving.moving_value_angular=0.5
                msg_moving.moving_value_linear=0.45
                self.pub_moving.publish(msg_moving)
            else:
                Logger.loginfo("Finished Parking Mode.")
                self._parking_proceed = False
                return 'done'

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e., when transitioning to this state.
        # It is typically used to start actions related to this state.

        Logger.loginfo('Entered state Parking')

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