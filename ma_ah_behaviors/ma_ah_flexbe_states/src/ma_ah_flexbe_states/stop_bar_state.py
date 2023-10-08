#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher

from std_msgs.msg import String

class StopBarState(EventState):
    '''
    Example for a state to detect a stop bar and react accordingly.
    This state listens to a topic for stop bar information and stops the robot when the bar is down.

    <= stop             Stop bar is down.
    <= proceed          Stop bar is up.

    '''

    def __init__(self):
        # Declare ouㅌ`ㅌ`tcomes by calling the super constructor with the corresponding arguments.
        super(StopBarState, self).__init__(outcomes=['proceed', 'done'])

        # Initialize class variables or state parameters here if needed.
        self._filtered_detection_sub = ProxySubscriberCached({"/filtered_detection": String})
        self.boom_barrier_detected = False


    #
    def execute(self, userdata):
        if self._sub.has_msg("/filtered_detection"):
            self._sign = self._sub.get_last_msg("/filtered_detection").data

            if self._sign == "['boom_barrier']" and self.boom_barrier_detected == False:
                Logger.loginfo("Boom Barrier Detected")
                self.boom_barrier_detected = True
                
                # 스탑 바 인식 후 정지, cmd_vel이 누적되어 앞으로 나가지는 않을 지?
                return 'done'
            
        else:   #stop 표시판을 인식하고 boom_barrier를 인식하지 못했을 때 거리 이동을 위해
            Logger.loginfo("No Sign, Keep Lane control")
            return 'proceed'


    def on_enter(self, userdata):
        Logger.loginfo('Entered state StopBar')

    def on_exit(self, userdata):

        pass # 이 예시에서는 할 일이 없습니다.


    def on_start(self):
        pass

    def on_stop(self):
        pass # 이 예시에서는 할 일이 없습니다.