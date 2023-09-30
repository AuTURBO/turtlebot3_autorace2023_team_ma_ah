#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from flexbe_core.proxy import ProxyPublisher

from std_msgs.msg import String

class IntersectionState(EventState):
    '''
    Example for a state to provide guidance to the robot at a crossroad based on a sign.
    This state listens to a topic for crossroad sign information and guides the robot accordingly.

    <= turn_left        Guide the robot to turn left.
    <= turn_right       Guide the robot to turn right.
    <= no_sign          No guidance sign available.

    '''

    def __init__(self):
        # Declare outcomes by calling the super constructor with the corresponding arguments.
        super(IntersectionState, self).__init__(outcomes=['turn_left', 'turn_right'])

        # Initialize class variables or state parameters here if needed.
        self._sub = ProxySubscriberCached({"/crossroad_sign": String})
        self._crossroad_sign = None

    def execute(self, userdata):
        # This method is called periodically while the state is active.
        # Its main purpose is to check the condition of the state and trigger the corresponding outcome.
        # If no outcome is returned, the state will stay active.

        if self._sub.has_msg("/crossroad_sign"):
            crossroad_info = self._sub.get_last_msg("/crossroad_sign").data
            Logger.loginfo("Crossroad sign: {}".format(crossroad_info))

            # Assuming 'crossroad_info' is a string that indicates the crossroad sign.
            if crossroad_info == "left":
                self._crossroad_sign = "left"
                return 'turn_left'
            elif crossroad_info == "right":
                self._crossroad_sign = "right"
                return 'turn_right'
        # else:
        #     Logger.loginfo("Finished intersection Mode")
        #     self._crossroad_sign = None
        #     return 'done'

    def on_enter(self, userdata):
        # This method is called when the state becomes active, i.e., when transitioning to this state.
        # It is typically used to start actions related to this state.

        Logger.loginfo('Entered state Intersection Mode')

    # You can define other state lifecycle methods like on_exit, on_start, and on_stop if needed.

