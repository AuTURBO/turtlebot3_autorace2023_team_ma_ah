#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher

from std_msgs.msg import String

class LeftSignState(EventState):
    '''
    Example for a state to provide guidance to the robot at a crossroad based on a sign.
    This state listens to a topic for crossroad sign information and guides the robot accordingly.

    <= turn_left        Guide the robot to turn left.
    <= turn_right       Guide the robot to turn right.
    <= no_sign          No guidance sign available.

    '''




    def __init__(self):
        # Declare outcomes by calling the super constructor with the corresponding arguments.
        super(LeftSignState, self).__init__(outcomes=['turn_left', 'proceed', 'stop'])

        # Initialize class variables or state parameters here if needed.
        self._filtered_detection_sub = ProxySubscriberCached({"/filtered/detection": String})
        self._pub = ProxyPublisher({"/cmd_vel": Twist})
        self.Obstacle_sign = False

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Its main purpose is to check the condition of the state and trigger the corresponding outcome.
        # If no outcome is returned, the state will stay active.
        Logger.loginfo("Left Direction Mode execute")

        # When robot detect any traffic sign
        if self._sub.has_msg("/filtered/detection"):
            self._direction = self._filtered_detection_sub.get_last_msg("/filtered/detection").data
            Logger.loginfo("Direction sign: {}".format(self._direction))

            #Yolo 표지판 인식 에러 (ex 왼쪽을 오른쪽으로 인식) 방지
            if self._direction == "['left']" or self._direction == "['right']":
                Logger.loginfo("Direction sign: Left")
                return 'turn_left'  
            
            elif self._direction == "['stop']" or self._direction == "['boom_barrier']": #boom_barrier
                Logger.loginfo("Direction sign: Stop")
                return 'stop'
            # When robot did't detect sign yet , keep lane control            
            else:
                Logger.loginfo("No Left, Right sign, Keep Lane control")
                return 'proceed'


    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e., when transitioning to this state.
        # It is typically used to start actions related to this state.
        pass
    # You can define other state lifecycle methods like on_exit, on_start, and on_stop if needed.

